function q_ddot = simplifiedModel_v8(q_, q_dot, v__n_c, tau_sail, delta_r, cfg, rudder, keel) %tau_tail, tau_ext)

% simplified model of the saildrone dynamics

% q_ = the system state [q_1=eta, q_2=epsilon, q_3= delta_t, q_4 = delta_r]
% q_dot = system state derivatives [nu, q_2_dot, q_3_dot] delta_r_dot is % not used here

% nu_current is the current velocity of the waterworld/ocean given in NED

% tau_tail_act  = the tail acuator moment at the joint (between sail and tail)
% tau_sail_act  = the sail "actuator moment" at the joint between sail and vessel, 
% there might not be a actuator here, but the input can be used to add friction models 
% tau_tail, tau_sail, tau_keel, tau_rudder      = the vector of aerodynamic/hydrodynamic forces and moment acting in AC of
% each link within the link's frame

% cfg is the sail drone configuration parameters calulated in the init
% script, where cfg is a struct

%% Extraction 
% extract position and attitude
eta = q_(1:6);
p__n_nb = eta(1:3);
THETA_nb = eta(4:6);
phi = eta(4);
theta = eta(5);
psi = eta(6);
epsilon = q_(7); %use xi or epsilon?
% delta_t = q_(8);
% delta_r = q_(9);

% extract velocities
nu = q_dot(1:6);
v__b_nb = nu(1:3);
omega__b_nb = nu(4:6);
p = nu(4);
q = nu(5);
r = nu(6);
% q_2_dot = q_dot(7);
% q_3_dot = q_dot(8);

% expression for relation between omegas and the qdots and K__i_i-1 v_i-1


%% Definitions
I3x3 = eye(3);
O3x3 = zeros(3,3);
g = 9.81;
rho = cfg.rho;
T = cfg.T;

% gravity in NED
g__n = [0; 0; g];


% time constants in (s) for linear damping estimation
T_surge = cfg.T_surge;   
T_sway = cfg.T_sway; % keel seems to be dominating this damping
T_yaw = cfg.T_yaw;

% damping ratios for linear damping estimation
zeta_heave = cfg.zeta_heave;
zeta_roll = cfg.zeta_roll;
zeta_pitch = cfg.zeta_pitch;
   

%% Rotation Transforms

% NED and body
R__n_b = Rzyx(phi, theta, psi);
R__b_n = transpose(R__n_b);

% body and sail
R__b_s = Rzyx(0,0,epsilon);
% R__s_b = transpose(R__b_s);

% sail and tail transforms 
% R__s_t = Rzyx(0,0,delta_t);
% R__t_s = transpose(R__s_t);
% R__t_af % create this transform

% Rudder and body
R__b_r = Rzyx(0,0,delta_r);
% R__r_b = transpose(R__b_r);

% NED and sail
% R__n_s = R__n_b*R__b_s;
% R__n_s = Rzyx(phi, theta, psi+epsilon) % alternative simpler expression, only for small angles! as epsilon is relative to body and not NED 
% R__s_n = transpose(R__n_s);

% NED and tail
% R__n_t = R__n_s*R__s_t;
% R__t_n = transpose(R__n_t);

%% Position vectors
% Vector from vessel {b} to {s} in body
r__b_bs = cfg.r__b_bs;

% % Vector from sail {s} to {t} in sail
% r__s_st = cfg.r__s_st;

% Vector from CO in {b} to rudder AC, assuming AC is at rudder pivot axis
r__b_brAC = cfg.r__b_brAC;

% Vector from CO in {b} to keel AC
r__b_bkAC = cfg.r__b_bkAC; 

% % Vector from sail frame at pivot point to sail AC
% r__s_ssAC = cfg.r__s_sAC;
% 
% % Vector from tail frame at pivot point to tail AC
% r__t_ttAC = cfg.r__t_tAC;

%% Spatial transforms for Motion and force

% Motion transforms K, transform a motion vector from one frame to another 
% K__2_1 = spatialTransformK(R__s_b,r__b_bs,'motion'); % 1 = body
% K__3_2 = spatialTransformK(R__t_s,r__s_st,'motion'); % 2 = saw
% K__3_1 = K__3_2*K__2_1;                              % 3 = tail

% for keel, sail, rudder and tail, how effect relative flow velocity
% effect -> use absolute velocity, relative flow velocity is used only for
% CA DA, when current is assumed constant and irational nu_dot = nu_r_dot

% K__rAC_b = spatialTransformK(R__r_b, r__b_brAC, 'motion'); % transform of motion vector from body to rudder rudder AC in rudder frame 
% K__kAC_b = spatialTransformK(eye(3), r__b_bkAC, 'motion'); % transform of motion vector from body to keel AC 
% 
% K__sAC_s = spatialTransformK(eye(3), r__s_ssAC, 'motion'); % sail pivot point to sail AC
% K__tAC_t = spatialTransformK(eye(3), r__t_ttAC, 'motion'); % tail pivot point to tail AC

% Force transforms Kf, transform a force vector from one frame to another
Kf__b_ss = spatialTransformK(R__b_s, r__b_bs, 'force-r__a_ab');   % assume that the sail induced forces act in the pivot point
Kf__b_r = spatialTransformK(R__b_r, r__b_brAC, 'force-r__a_ab'); % use vector from next frame
Kf__b_k = spatialTransformK(eye(3), r__b_bkAC, 'force-r__a_ab'); % use vector from next frame (frame do not rotate hence they are the same)

% Kf__s_sAC = spatialTransformK(eye(3), r__s_ssAC, 'force-r__a_ab'); % use vector from next frame
% Kf__t_tAC = spatialTransformK(eye(3), r__t_ttAC, 'force-r__a_ab'); % use vector from next frame


%% Transform inputs
v__b_c = R__b_n*v__n_c;
nu__b_c = [v__b_c; 0; 0; 0];

%% Dynamics

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Saildrone  %%%%%%%%%%%%%%%%%%%%%%
% realtive velocity 
nu_r = nu - nu__b_c;
nu_rc = [nu_r(1:2); nu_r(4); nu_r(6)];
u_rc = nu_r(1);
v_rc = nu_r(2);

% Inertia of vessel and keel in there CG 
I__CG_saildrone = cfg.I__CG_saildrone;

% Mass of vessel and keel
m_drone = cfg.m_saildrone;

% Vector from body frame to saildrone CG in body frame
r__b_bg = cfg.r__b_bg;

% System inertia and coriolis matrix in CG
MRB_CG = [m_drone*I3x3,        O3x3;
           O3x3,                I__CG_saildrone];

% CRB_CG = [m_drone*Smtrx(omega__b_nb)       O3x3;
%             O3x3                            -Smtrx(I__CG_saildrone*omega__b_nb)];


% NEED TO ADD ADDED MASS AND CORIOLIS CENTRIPITAL MATRIX TO FULLY EXPRESS THE VESSEL

% System inertia and coriolis matrix in the body frame
H = Hmtrx(r__b_bg);
MRB = transpose(H)*MRB_CG*H;
% CRB = transpose(H)*CRB_CG*H;

% ADDED MASS AND CORIOLIS GOES HERE (do not effect rest of system as vessel is the first link)
% Hydrodynamic added mass (best practise) from FOSSEN MSS toolbox for otter vessel
Xudot = -addedMassSurge(cfg.m_saildrone,cfg.l_hull); % use the parameter for the full drone, default rho = 1025  
Yvdot = -1.5 * cfg.m_saildrone;
Zwdot = -1.0 * cfg.m_saildrone;
Kpdot = -0.2 * cfg.I__CG_saildrone(1,1);
Mqdot = -0.8 * cfg.I__CG_saildrone(2,2);
Nrdot = -1.7 * cfg.I__CG_saildrone(3,3);

MA = -diag([Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot]);
% CA  = m2c(MA, nu_r);

% until nonlinear damping is added: for simplified version the munk moment
% might be removed to create simulations closer to the nonlinear one
% CA(6,1) = 0; % Assume that the Munk moment in yaw can be neglected
% CA(6,2) = 0; % These terms, if nonzero, must be balanced by adding nonlinear damping

% System mass and Coriolis-centripetal matrices here link 1 is the only one
M_SD = MRB + MA;  % M of saildrone
% C_SD = CRB + CA;  % C of saildrone

%%%% DAMPING
%%%%%% w3...5 assume keel is equal to water desity and calulate GMT and GML

% Natural frequencies
G_mtrx_SD = cfg.G_mtrx;
w3 = sqrt( G_mtrx_SD(3,3)/M_SD(3,3)); % 5.26 4.9        
w4 = sqrt( G_mtrx_SD(4,4)/M_SD(4,4)); % 1.76 1.84
w5 = sqrt( G_mtrx_SD(5,5)/M_SD(5,5)); % 5.43 5.2

% Linear damping terms (hydrodynamic derivatives)
Xu = -M_SD(1,1)/T_surge;                 % specified using the time constants specified 
Yv = -M_SD(2,2)/T_sway;                  % specified using the time constants specified 
Zw = -2 * zeta_heave*w3 * M_SD(3,3);     % specified using relative damping factors 
Kp = -2 * zeta_roll *w4 * M_SD(4,4);
Mq = -2 * zeta_pitch *w5 * M_SD(5,5);
Nr = -M_SD(6,6)/T_yaw;                   % specified using the time constant in 

D = -diag([Xu, Yv, Zw, Kp, Mq, Nr]);

% nonlinear surge damping
% nonlinear damping due to the keel is assmued to be captured by the lift/drag model
Lpp = cfg.l_hull;
eps = 0.001;            
CR = 0;                     % residual friction due to hull roughness, pressure resistance ++
k = 0.1;                    % typical value ship in transit
WS = cfg.WS;        % wetted surface of the hull (keel resistance modeled seperate)
kv = 1e-6;                  
Rn = (Lpp / kv) * abs(cfg.u_rc_avarage); % Reynolds number
Cf = (0.075 / ((log10(Rn) - 2)^2 + eps)) + CR; % flate plate frition from ITTC 1957 line
% Xns = -0.5*rho*WS*(1+k)*Cf*abs(nu_r(1))*nu_r(1);
Xns_star = -0.5*rho*WS*(1+k)*Cf*abs(nu_r(1));
% 
% % nonlinear cross-flow drag
% Cd_2d = cfg.Cd_2d;
% dx = Lpp/10;
% Ycf = 0; 
% Ncf = 0;
% index = 1;
% for xL = (dx/2 - Lpp/2):dx:(Lpp/2 - dx/2)
%     Ucf = abs(nu_r(2) + xL * nu_r(6)) * (nu_r(2) + xL * nu_r(6));
%     Ycf = Ycf - 0.5 * rho * T * Cd_2d(index) * Ucf * dx;
%     Ncf = Ncf - 0.5 * rho * T * Cd_2d(index) * xL * Ucf * dx;
%     index = index + 1;
% end
% 
% d_n = -[Xns; Ycf; 0; 0; 0; Ncf];

% Term correspond to N(nu_r)nu_r in Fossen
% N_nu_r = C_SD*nu_r.*1 + D*nu_r.*1 + d_n.*1;
% N_nu_r = C_SD*nu_r.*1 + D*nu_r.*1;

% Restoring forces 
% encounting for all links + neglecting rudder
G_eta = cfg.G_mtrx*eta;

% The current acceleration: M^1*M*nu_current_dot_sys =  nu_current_dot_sys
% derivative of nu_current
u_c = nu__b_c(1);
v_c = nu__b_c(2);
% r = nu(6);
% nu_current_dot = [r*v_c; -r*u_c; 0; 0; 0; 0];
nu_current_dot4 = [r*v_c; -r*u_c; 0; 0];


%% Reduced 4DOF model
% System inertia matrix 4dof 
M_4 = diag([M_SD(1,1), M_SD(2,2), M_SD(4,4), M_SD(6,6)]);

% Damping and C matrix
% change Xu to Xu_star
N_4 = [-Xu - Xns_star,          -m_drone*r,     0,      Yvdot*v_rc;
       m_drone*r,               -Yv,            0,      -Xudot*u_rc;
       0,                       0,             -Kp,     0;
       -Yvdot*v_rc,             Xudot*u_rc,     0,      -Nr];

% Rudder
S_r = rudder.S;
C_Dpr = rudder.C_D_parasitic;
k_CLr = rudder.K_C_L;
r__b_brACx = cfg.r__b_brAC(1);
r__b_brACz = cfg.r__b_brAC(3);
   
N__R = [-rho*C_Dpr*S_r*u_rc/2,          0,          0,                  0;
        0,                          -u_rc*k_CLr,    0,  -u_rc*k_CLr*r__b_brACx;
        0,                r__b_brACz*u_rc*k_CLr,    0,  r__b_brACz*u_rc*k_CLr*r__b_brACx;
        0,               -r__b_brACx*u_rc*k_CLr,    0,  -(r__b_brACx^2)*u_rc*k_CLr];
    
% delta_r = 0

ctau__b_r = [-(u_rc^2)*k_CLr*(delta_r^2);
             (u_rc^2)*k_CLr*delta_r;
             -r__b_brACz*(u_rc^2)*k_CLr*delta_r;
             r__b_brACx*(u_rc^2)*k_CLr*delta_r];
         
% Keel
S_k = keel.S;
C_Dpk = keel.C_D_parasitic;
k_CLk = keel.K_C_L;
r__b_bkACz = cfg.r__b_bkAC(3);
kappa_k = keel.C_L_alpha/(pi*keel.e_OS*keel.AR);

N__K12 = k_CLk*(v_rc-p*r__b_bkACz)*(1-kappa_k);
N__K13 = k_CLk*(-v_rc*r__b_bkACz + p*(r__b_bkACz^2))*(1-kappa_k);

N__K = [-rho*C_Dpk*S_k*u_rc/2,      N__K12,                     N__K13,             0;
        0,                      -u_rc*k_CLk,            u_rc*k_CLk*r__b_bkACz,     0;
        0,                   r__b_bkACz*u_rc*k_CLk,  -(r__b_bkACz^2)*u_rc*k_CLk,    0;
        0,                          0,                          0,                  0];
    
% gravity
G_eta4 = [G_eta(1:2); G_eta(4); G_eta(6)];
  

%% %%%%%%%%%% External forces %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% forces acting on the diretly on the vessel (hull or keel) do not need to
% be transformed as the transform is inly I
% tau_sys = PHI'*L'*tau_gext


%% Sail and tail aerodynamic forces
% % remember tail AC in not in link
% tau_sail_aero = Kf__s_sAC*tau_sail;
% tau_tail_aero = Kf__t_tAC*tau_tail;   %% create this to transfroms!!!
% 
% tau_aero = [zeros(6,1); tau_sail_aero; tau_tail_aero];
% tau_aero_sys = transpose(PHI)*transpose(L)*tau_aero;

% tail froces mainly contribute to rotate the sail system is neglected

% Sail forces in the body frame
tau__b_sail = Kf__b_ss*tau_sail; % tranfrom into body frame and {b}
tau__b_sail = [tau__b_sail(1:2); tau__b_sail(4); tau__b_sail(6)];

nu_dot = nu_current_dot4 + M_4\((-N_4+N__K+N__R)*nu_rc - G_eta4 + ctau__b_r + tau__b_sail);
q_ddot_ = [nu_dot(1:2); 0; nu_dot(3); 0; nu_dot(4)];

%% Rudder hydrodynamic forces (act directly on vessel)
% Assuming small angle we can get the forces in the rudder forces in the
% body fram directly as in fossen p 248
% tau__b_rudder = Kf__b_r*tau_rudder;

%% Keel hydrodynamic forces (act directly on vessel)
% tau__b_keel = Kf__b_k*tau_keel;


%% output
% q_ddot_ = nu_current_dot + M_SD\(-N_nu_r - G_eta + tau__b_rudder + tau__b_keel + tau__b_sail);
% q_ddot_ = M\( tau__b_g + tau__b_buoy + tau__b_rudder + tau__b_keel + tau__b_sail);
q_ddot = cfg.Active_states(1:6,1:6)*q_ddot_;

end