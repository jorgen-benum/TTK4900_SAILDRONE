function [q_ddot, tau__b_rudder] = moduleBasedModel_v7(q_, q_dot, v__b_c, tau_tail_act, tau_sail_act, tau_tail, tau_sail, tau_keel, tau_rudder, cfg) %tau_tail, tau_ext)

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
epsilon = q_(7);
delta_t = q_(8);
delta_r = q_(9);

% extract velocities
nu = q_dot(1:6);
v__b_nb = nu(1:3);
omega__b_nb = nu(4:6);
q_2_dot = q_dot(7);
q_3_dot = q_dot(8);

nu__b_c = [v__b_c; 0; 0; 0];

% expression for relation between omegas and the qdots and K__i_i-1 v_i-1

% derivative of nu_current
u_c = nu__b_c(1);
v_c = nu__b_c(2);
r = nu(6);

nu_current_dot = [r*v_c; -r*u_c; 0; 0; 0; 0];

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

%% Calculate the "S^I" matrix and get the joint configurations 

PHI_1 = cfg.PHI_1; % floating base
PHI_2 = cfg.PHI_2;  % revolute joint
PHI_3 = cfg.PHI_3;  % revolute joint

SS_PHI1qd1 = SmtrxSpatial(PHI_1*nu); 
SS_PHI2qd2 = SmtrxSpatial(PHI_2*q_2_dot); % 
SS_PHI3qd3 = SmtrxSpatial(PHI_3*q_3_dot); % 

SSI = blkdiag(SS_PHI1qd1, SS_PHI2qd2, SS_PHI3qd3);

   

%% Rotation Transforms

% NED and body
R__n_b = Rzyx(phi, theta, psi);
R__b_n = transpose(R__n_b);

% body and sail
R__b_s = Rzyx(0,0,epsilon);
R__s_b = transpose(R__b_s);

% sail and tail transforms 
R__s_t = Rzyx(0,0,delta_t);
R__t_s = transpose(R__s_t);
% R__t_af % create this transform

% Rudder and body
R__b_r = Rzyx(0,0,delta_r);
R__r_b = transpose(R__b_r);

% NED and sail
R__n_s = R__n_b*R__b_s;
% R__n_s = Rzyx(phi, theta, psi+epsilon) % alternative simpler expression, only for small angles! as epsilon is relative to body and not NED 
R__s_n = transpose(R__n_s);

% NED and tail
R__n_t = R__n_s*R__s_t;
R__t_n = transpose(R__n_t);

%% Position vectors
% Vector from vessel {b} to {s} in body
r__b_bs = cfg.r__b_bs;

% Vector from sail {s} to {t} in sail
r__s_st = cfg.r__s_st;

% Vector from CO in {b} to rudder AC, assuming AC is at rudder pivot axis
r__b_brAC = cfg.r__b_brAC;

% Vector from CO in {b} to keel AC
r__b_bkAC = cfg.r__b_bkAC; 

% Vector from sail frame at pivot point to sail AC
r__s_ssAC = cfg.r__s_sAC;

% Vector from tail frame at pivot point to tail AC
r__t_ttAC = cfg.r__t_tAC;

%% Spatial transforms for Motion and force

% Motion transforms K, transform a motion vector from one frame to another 
K__2_1 = spatialTransformK(R__s_b,r__b_bs,'motion'); % 1 = body
K__3_2 = spatialTransformK(R__t_s,r__s_st,'motion'); % 2 = saw
K__3_1 = K__3_2*K__2_1;                              % 3 = tail

% for keel, sail, rudder and tail, how effect relative flow velocity
% effect -> use absolute velocity, relative flow velocity is used only for
% CA DA, when current is assumed constant and irational nu_dot = nu_r_dot

K__rAC_b = spatialTransformK(R__r_b, r__b_brAC, 'motion'); % transform of motion vector from body to rudder rudder AC in rudder frame 
K__kAC_b = spatialTransformK(eye(3), r__b_bkAC, 'motion'); % transform of motion vector from body to keel AC 

K__sAC_s = spatialTransformK(eye(3), r__s_ssAC, 'motion'); % sail pivot point to sail AC
K__tAC_t = spatialTransformK(eye(3), r__t_ttAC, 'motion'); % tail pivot point to tail AC

% Force transforms Kf, transform a force vector from one frame to another
Kf__b_r = spatialTransformK(R__b_r, r__b_brAC, 'force-r__a_ab'); % use vector from next frame
Kf__b_k = spatialTransformK(eye(3), r__b_bkAC, 'force-r__a_ab'); % use vector from next frame (frame do not rotate hence they are the same)

Kf__s_sAC = spatialTransformK(eye(3), r__s_ssAC, 'force-r__a_ab'); % use vector from next frame
Kf__t_tAC = spatialTransformK(eye(3), r__t_ttAC, 'force-r__a_ab'); % use vector from next frame

%% Constants

%% Links
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Vessel link  %%%%%%%%%%%%%%%%%%%%%%

nu_r = nu-nu__b_c; 
omega__b_nb_rel = nu_r(4:6);

% Inertia of vessel and keel in there CG 
I__CG_vessel = cfg.I__CG_vessel;

% Mass of vessel and keel
m_ves = cfg.m_vessel;

% Vector from body frame to vessel CG in body frame
r__b_bvg = cfg.r__b_bvg;

% System inertia and coriolis matrix in CG
MRB1_CG = [m_ves*I3x3, O3x3;
           O3x3,    I__CG_vessel];

CRB1_CG = [m_ves*Smtrx(omega__b_nb)     O3x3;
            O3x3                            -Smtrx(I__CG_vessel*omega__b_nb)];
        
% System inertia and coriolis matrix in the body frame
H1 = Hmtrx(r__b_bvg);
MRB_1 = transpose(H1)*MRB1_CG*H1;
CRB_1 = transpose(H1)*CRB1_CG*H1;

% ADDED MASS AND CORIOLIS GOES HERE (do not effect rest of system as vessel is the first link)
% Hydrodynamic added mass (best practise) from FOSSEN MSS toolbox for otter vessel
Xudot = -addedMassSurge(cfg.m_saildrone,cfg.l_hull); % use the parameter for the full drone, default rho = 1025  
Yvdot = -1.5 * cfg.m_saildrone;
Zwdot = -1.0 * cfg.m_saildrone;
Kpdot = -0.2 * cfg.I__CG_saildrone(1,1);
Mqdot = -0.8 * cfg.I__CG_saildrone(2,2);
Nrdot = -1.7 * cfg.I__CG_saildrone(3,3);

MA = -diag([Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot]);
CA  = m2c(MA, nu_r);

% until nonlinear dmping is added :
% CA(6,1) = 0; % Assume that the Munk moment in yaw can be neglected
% CA(6,2) = 0; % These terms, if nonzero, must be balanced by adding nonlinear damping

% System mass and Coriolis-centripetal matrices
M_1 = MRB_1 + MA;
C_1 = CRB_1 + CA;


%%%% DAMPING
% Use M and G_mtrx matrix of hole system instead to get same as other models
% nonlinear damping is not effected by this

% Natural frequencies using the G_mtrx and M for the entire saildrone (in order to be same for all models)
G_mtrx_SD = cfg.G_mtrx;
M_SD = cfg.MRB_SD + MA; % M for full saildrone
w3 = sqrt( G_mtrx_SD(3,3)/M_SD(3,3)); % 5.26 4.9        
w4 = sqrt( G_mtrx_SD(4,4)/M_SD(4,4)); % 1.76 1.84
w5 = sqrt( G_mtrx_SD(5,5)/M_SD(5,5)); % 5.43 5.2

% Linear damping terms (hydrodynamic derivatives) using the paramters
% from the full saildrone (in order to be same for all models)
Xu = -M_SD(1,1)/T_surge;                 % specified using the time constants specified -58.57 with T = 18 old: -28.98  it T = 35
Yv = -M_SD(2,2)/T_sway;                  % specified using the time constants specified -138.88 with T = 18 old:-114.8 if T = 25 -82 if T = 35
Zw = -2 * zeta_heave*w3 * M_SD(3,3);     % specified using relative damping factors 
Kp = -2 * zeta_roll *w4 * M_SD(4,4);
Mq = -2 * zeta_pitch *w5 * M_SD(5,5);
Nr = -M_SD(6,6)/T_yaw;                   % specified using the time constant in T_yaw -643.5

D = -diag([Xu, Yv, Zw, Kp, Mq, Nr]);

% nonlinear surge damping
% nonlinear damping due to the keel is assmued to be captured by the lift/drag model
Lpp = cfg.l_hull;
eps = 0.001;            
CR = 0;                     % residual friction due to hull roughness, pressure resistance ++
k = 0.1;                    % typical value ship in transit
WS = cfg.WS;        % wetted surface of the hull (keel resistance modeled seperate)
kv = 1e-6;                  
Rn = (Lpp / kv) * abs(nu_r(1)); % Reynolds number
Cf = (0.075 / ((log10(Rn) - 2)^2 + eps)) + CR; % flate plate frition from ITTC 1957 line
Xns = -0.5*rho*WS*(1+k)*Cf*abs(nu_r(1))*nu_r(1);

% nonlinear cross-flow drag
Cd_2d = cfg.Cd_2d;
dx = Lpp/10;
Ycf = 0; 
Ncf = 0;
index = 1;
for xL = (dx/2 - Lpp/2):dx:(Lpp/2 - dx/2)
    Ucf = abs(nu_r(2) + xL * nu_r(6)) * (nu_r(2) + xL * nu_r(6));
    Ycf = Ycf - 0.5 * rho * T * Cd_2d(index) * Ucf * dx;
    Ncf = Ncf - 0.5 * rho * T * Cd_2d(index) * xL * Ucf * dx;
    index = index + 1;
end

d_n = -[Xns; Ycf; 0; 0; 0; Ncf];

%%%%% Forces acting in the vessel link due to coriolis/centripial and damping effects %%%%%
% coriolis/centripital, damping and other effects not included in the
% subsystem CRB_1 matrix. CRB_vessel can be set to 0 if added here
% instead. This term correspond to N(nu_r)nu_r in Fossen
N_nu_r = C_1*nu_r.*1 + D*nu_r.*1 + d_n.*1;

% Restoring forces using the G_mtrx for the vessel (not including sailsystem 
% as these moments are added in other links)
G_eta = cfg.G_mtrx_vessel*eta;

% The current acceleration: M^1*M*nu_current_dot_sys =  nu_current_dot_sys
nu_current_dot_sys = [nu_current_dot; 0; 0]; % added in the end actually!! 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Sail and weight (SAW) link %%%%%%%%%%%%%%%%%%%%%%%%%%
% SAW includes the counter weight

% velocity of sail relative to NED
nu_sail = K__2_1*nu + PHI_2*q_2_dot;
omega_s_ns = nu_sail(4:6); % only angular velocity

% Inertia of SAW in CG
I__CG_saw = cfg.I__CG_saw;

% Mass of sail
m_saw = cfg.m_saw;

% Vector from sail pivot point to the saw CG
r__s_ssawg = cfg.r__s_ssawg;

% System inertia and coriolis matrix in CG
MRB2_CG = [m_saw*I3x3, O3x3;
           O3x3,    I__CG_saw];

CRB2_CG = [m_saw*Smtrx(omega_s_ns)     O3x3;
            O3x3                            -Smtrx(I__CG_saw*omega_s_ns)];

% System inertia and coriolis matrix in the sail frame
H2 = Hmtrx(r__s_ssawg);
MRB_2 = transpose(H2)*MRB2_CG*H2;
CRB_2 = transpose(H2)*CRB2_CG*H2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Tail link  %%%%%%%%%%%%%%%%%%%%%%%%%

% velocity of tail relative to NED
nu_tail = K__3_2*nu_sail + PHI_3*q_3_dot;
omega_t_nt = nu_tail(4:6); % only angular velocity

% Inertia of tail in CG
I__CG_t = cfg.I__CG_tail;

% Mass of tail
m_tail = cfg.m_tail;

% Vector from pivot point to the tail CG
r__t_tg = cfg.r__t_tg;

% System inertia and coriolis matrix in CG
MRB3_CG = [m_tail*I3x3, O3x3;
           O3x3,    I__CG_t];

CRB3_CG = [m_tail*Smtrx(omega_t_nt)     O3x3;
            O3x3                            -Smtrx(I__CG_t*omega_t_nt)];

% System inertia and coriolis matrix in the tail frame
H3 = Hmtrx(r__t_tg);
MRB_3 = transpose(H3)*MRB3_CG*H3;
CRB_3 = transpose(H3)*CRB3_CG*H3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Total system  %%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Module based modeling
% Each link/subsystem is modeled with respect to itself and its interation with 
% the other links and the environment

% Joint configuration matrix
PHI = blkdiag(PHI_1, PHI_2, PHI_3);

% System inertia of link (subsystem) inertia matrices 
% M = blkdiag(MRB_1, MRB_2, MRB_3);
M = blkdiag(M_1, MRB_2, MRB_3); % use mass matrix with added mass for link 1

% Matrix of link (subsystem) coriolis and centrifugal matrices
% C = blkdiag(CRB_1, CRB_2, CRB_3);
C = blkdiag(CRB_1*.0, CRB_2, CRB_3); % remove CRB_1 as it is a part of the vessel N_nu_r instead

% 
W = blkdiag(K__2_1, K__3_2); 
O_zeros = zeros(size(K__2_1,1),size(K__2_1,2));
W_top = [O_zeros, O_zeros];
W_right = [O_zeros; O_zeros; O_zeros];
W = [W_top; W];
W = [W, W_right];

%
I = eye(size(K__2_1,1));
L = blkdiag(I,I,I)+W+[O_zeros, O_zeros, O_zeros;
                      O_zeros, O_zeros, O_zeros;
                      K__3_1,  O_zeros, O_zeros]; % what to do with these?

% M of entire system including all 6dof of each link
M_sys = transpose(PHI)*transpose(L)*M*L*PHI;

% C of entire system including all 6dof of each link
C_sys_part1 = transpose(PHI)*transpose(L)*(-M*L*SSI*W)*L*PHI;
C_sys_part2 = transpose(PHI)*transpose(L)*(C)*L*PHI;
C_sys = C_sys_part1 + C_sys_part2;


%% %%%%%%%%%% Gravity and external forces %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% forces acting on the diretly on the vessel (hull or keel) do not need to
% be transformed as the transform is inly I
% tau_sys = PHI'*L'*tau_gext

%% Forces due to gravity and buoancy

% for a floating vessel the total force due to the mass of the saildrone is
% cancelled by the bouancy force. As well as the moment acting on the
% vessel (rudder assumed be neglectable %%%%to have density = rho and can be neglected).

% Moment due to gravity acting on other links/subsystem can create moment
% which will be handeled by the restoring forces and moments
% Comment, the keel weight is allready accoundet for in the
% vessel CG henc r__b_vg. It is also included in the calculation of the
% buocy point used to calulate the GMT and GML hence these forces are
% included!!!

% Restoring forces take CG and BOU into account hence moments in vessel link can be ingnored.
% Use Gmtrx_vessel for restoring forces as sailsystem weight still create moment! And is not
% encounted for in the vessel CG, which is not the same as the total CG!
% Hence the extra stiffness is given by not including the sailsystem, hence
% these moments need to be included in the sum of generalized forces.

% force due to gravity acting on the vessel (hull + keel) in body frame  
fg1__b_vg = R__b_n*g__n*m_ves;
% tau_1_g = [fg1__b_vg; cross(r__b_bvg,fg1__b_vg)]; % need a bounacy force to compensate this + others
% the forces and moments due to the hull and keel are 
% assumed to be canceled by bouancy using a keel with desity aprrox = rho
% tau_1_g = zeros(6,1); 
% tau_1_g = [fg1__b_vg; cross(r__b_bvg,fg1__b_vg)];
% tau_1_buoy = -tau_1_g;
% moment acting in the first link are included in the restoring forces as
% well as forces due to heave deviation
tau_1_g = [fg1__b_vg; zeros(3,1)];


% force due to gravity acting on the sail and weight combined in sail frame
fg2__s_sawg = R__s_n*g__n*m_saw; 
% tau_2_g = [zeros(3,1); cross(r__s_ssawg,fg2__s_sawg)]; % bouancy cancel forces but not moment
tau_2_g = [fg2__s_sawg; cross(r__s_ssawg,fg2__s_sawg)];
% tau_2_buoy = -[fg2__s_sawg; zeros(3,1)];

% force due to gravity actong on the tail in tail frame
fg3__t_tg = R__t_n*g__n*m_tail; 
% tau_3_g = [zeros(3,1); cross(r__t_tg,fg3__t_tg)]; % bouancy cancel forces but not moment
tau_3_g = [fg3__t_tg; cross(r__t_tg,fg3__t_tg)];
% tau_3_bouy = -[fg3__t_tg; zeros(3,1)];

tau__b_g = [tau_1_g; tau_2_g; tau_3_g];
tau__b_g_sys = transpose(PHI)*transpose(L)*tau__b_g;

% tau_b_buoy = [-R__b_n*g__n*cfg.m_saildrone; -cross(r__b_bvg,fg1__b_vg)];
% only cancel forces, restroing forces are seperate
tau_b_buoy = [-R__b_n*g__n*cfg.m_saildrone; zeros(3,1)];
tau__b_buoy_sys = [tau_b_buoy; 0; 0];


%% Sail and tail aerodynamic forces
% remember tail AC in not in link
tau_sail_aero = Kf__s_sAC*tau_sail;
tau_tail_aero = Kf__t_tAC*tau_tail;   %% create this to transfroms!!!

tau_aero = [zeros(6,1); tau_sail_aero; tau_tail_aero];
tau_aero_sys = transpose(PHI)*transpose(L)*tau_aero;

%% Rudder hydrodynamic forces (act directly on vessel)
% tau_rudder_sys = [Kf__b_r*(tau_rudder); 0; 0]; 
tau__b_rudder = Kf__b_r*(tau_rudder);
tau_rudder_sys = [tau__b_rudder; 0; 0]; % transfered from pervious link

%% Keel hydrodynamic forces (act directly on vessel)
% tau_keel_sys = [Kf__b_k*tau_keel; 0; 0];
tau_keel_sys = [Kf__b_k*(tau_keel); 0; 0];

%% Forces at joints
tau_joints = [0;0;0;0;0;0; tau_sail_act; tau_tail_act];
% tau_joints = [0;0;0;0;0;0; 0; 1] % remember the vessel forces 1-6 are in
% the body frame, while the q_2 and q_3 is for the sailpivot point and tail
% pivot point

% Spring stopper for tail
% if delta_t > cfg.tail_saturation_limit
%     tau_tail_limit = ((180/pi)*(delta_t - cfg.tail_saturation_limit))^4
%     tau_tail_limit_sys = [0;0;0;0;0;0; 0; -tau_tail_limit]
% elseif delta_t < -cfg.tail_saturation_limit
%     tau_tail_limit = ((180/pi)*(delta_t + cfg.tail_saturation_limit))^4
%     tau_tail_limit_sys = [0;0;0;0;0;0; 0; tau_tail_limit]
% else
%     tau_tail_limit_sys = [0;0;0;0;0;0; 0; 0]
% end

%% Force vector of forces acting on the vessel hull

N = [N_nu_r; zeros(6,1); zeros(6,1)];
N_sys = transpose(PHI)*transpose(L)*N;

% restoring forces
G_sys = transpose(PHI)*transpose(L)*[G_eta; zeros(6,1); zeros(6,1)];

%% output
% q_ddot_ = M_sys\(-C_sys*q_dot + tau_joints + tau__b_g_sys + tau__b_buoy_sys + tau_rudder_sys + tau_keel_sys + tau_aero_sys + tau_tail_limit_sys);
q_ddot_ = nu_current_dot_sys + M_sys\(-C_sys*q_dot - N_sys - G_sys + tau_joints + tau__b_g_sys + tau__b_buoy_sys + tau_rudder_sys + tau_keel_sys + tau_aero_sys);
q_ddot = cfg.Active_states*q_ddot_;

% if abs(delta_t) > cfg.tail_saturation_limit
%     tau_stopper = (M_sys*q_ddot + C_sys*q_dot + tau_joints + tau__b_g_sys + tau__b_buoy_sys + tau_rudder_sys + tau_keel_sys + tau_aero_sys)
%     q_ddot_stopper = -M_sys\tau_stopper
%     q_ddot(8) = q_ddot_stopper(8)
% end
% nu_sailAC   = K__sAC_s*nu_sail
% nu_tailAC   = K__tAC_t*nu_tail
% nu_rudderAC = K__rAC_b*nu
% nu_keelAC   = K__kAC_b*nu

end