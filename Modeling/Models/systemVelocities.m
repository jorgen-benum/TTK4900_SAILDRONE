function [nu_keelAC, nu_sailAC, nu_tailAC, nu_rudderAC, v__n_sailAC] = systemVelocities(q_, q_dot, q_4_dot, cfg) %tau_tail, tau_ext)nu_ACVector



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



%% joint configurations 

PHI_1 = cfg.PHI_1; % floating base
PHI_2 = cfg.PHI_2;  % revolute joint
PHI_3 = cfg.PHI_3;  % revolute joint
PHI_4 = cfg.PHI_4;  % revolute joint 

%% Rotation Transforms

% NED and body
R__n_b = Rzyx(phi, theta, psi);
% R__b_n = transpose(R__n_b);

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
% R__s_n = transpose(R__n_s);

% NED and tail
% R__n_t = R__n_s*R__s_t;
% R__t_n = transpose(R__n_t);

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
% K__3_1 = K__3_2*K__2_1;                              % 3 = tail

% for keel, sail, rudder and tail, how effect relative flow velocity
% effect -> use absolute velocity, relative flow velocity is used only for
% CA DA, when current is assumed constant and irational nu_dot = nu_r_dot

K__rAC_b = spatialTransformK(R__r_b, r__b_brAC, 'motion'); % transform of motion vector from body to rudder rudder AC in rudder frame 
K__kAC_b = spatialTransformK(eye(3), r__b_bkAC, 'motion'); % transform of motion vector from body to keel AC 

K__sAC_s = spatialTransformK(eye(3), r__s_ssAC, 'motion'); % sail pivot point to sail AC
K__tAC_t = spatialTransformK(eye(3), r__t_ttAC, 'motion'); % tail pivot point to tail AC

%% Constants


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Vessel link  %%%%%%%%%%%%%%%%%%%%%%
% Velocity of keel relative to NED
nu_keelAC   = K__kAC_b*nu;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Sail and weight (SAW) link %%%%%%%%%%%%%%%%%%%%%%%%%%
% SAW includes the counter weight

% Velocity of sail relative to NED
nu_sail = K__2_1*nu + PHI_2*q_2_dot;
% Velocity of sail AC relative to NED
nu_sailAC   = K__sAC_s*nu_sail;
v__n_sailAC = R__n_s*nu_sailAC(1:3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Tail link  %%%%%%%%%%%%%%%%%%%%%%%%%

% velocity of tail relative to NED
nu_tail = K__3_2*nu_sail + PHI_3*q_3_dot;
% velocity of tail AC relative to NED
nu_tailAC   = K__tAC_t*nu_tail;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Rudder link %%%%%%%%%%%%%
nu_rudderAC = K__rAC_b*nu + PHI_4*q_4_dot; % -> aero is only effected by relative linear velocity

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Total system  %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% nu_ACVector = [nu_keelAC; nu_sailAC; nu_tailAC; nu_rudderAC];




end