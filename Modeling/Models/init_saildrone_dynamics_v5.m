

%% Configuration of the saildrone both for simple and full model
% precalulate vectors and inertias needed in the dynamics


%% %%%%%%%%%%%%% Saildrone configuration 

%%%%%% Setting parameters

% Active states in the simulation model (inactive state remain at 0)
cfg.Active_states = diag([0,0,0,0,0,1,1,1]); % rudder cannot be inactive (state 9)
cfg.Active_states = diag([1,1,0,0,0,1,1,1]);
cfg.Active_states = diag([1,1,1,1,1,1,1,1]);
% cfg.Active_states = diag([1,1,1,0,1,1,1,1]); % no roll

% Constants
g = 9.81;           % [m/s] acceleration due to gravity 
rho = 1025;         % [kg/m^3] denisty of water
rho_air = 1.225     % [kg/m^3] density of water
rad2deg = 180/pi;
deg2rad = pi/180;

% Joints configuration
cfg.PHI_1 = diag([1,1,1,1,1,1]); % floating base           % vessel 
cfg.PHI_2 = [0; 0; 0; 0; 0; 1];  % revolute joint around z % sail
cfg.PHI_3 = [0; 0; 0; 0; 0; 1];  % revolute joint around z % tail
cfg.PHI_4 = [0; 0; 0; 0; 0; 1];  % revolute joint around z % rudder

% Joint limitations
cfg.rudder_saturation_limit = 35* deg2rad;%35* deg2rad;
cfg.rudder_time_constant = 0.5%0.5%2; % 2 0.1
cfg.rudder_gain = 1;

cfg.tail_saturation_limit = 80* deg2rad;
cfg.tail_moment_limit = 50; %[Nm], joint [1] in a UR 10 is static at 125NM at max reach and load
cfg.tail_time_constant = 0.01; 
cfg.tail_gain = 1;

% Paramter to adjust to fit test experiments
% Time constants in (s) for linear damping estimation
cfg.T_surge = 35;   
cfg.T_sway = 18; % keel seems to be dominating this damping
cfg.T_yaw = 10;

% Damping ratios for linear damping estimation
cfg.zeta_heave = 0.3;
cfg.zeta_roll = 0.2;
cfg.zeta_pitch = 0.4;

% Simplified nonlinear surge damping
cfg.u_rc_avarage = 1.4; % [m/s] use nonlinear surge damping at avarage speed this becomes quadratic damping


%% %%%%%%%%%%%% Vessel parameters 

%% Rudder
h_rudder = 0.8;
w_rudder = 0.12;

l_rudder = 0.4; % moving blade length

% % foil lift parameters of symmetric rudder
rudder.L = l_rudder;
rudder.S = h_rudder*l_rudder;
rudder.AR = (h_rudder^2)/rudder.S;
% rudder.C_L_0 = 0
rudder.C_L_alpha = (pi*rudder.AR)/(1 + sqrt(1 + (rudder.AR/2)^2));
rudder.K_C_L = (1/2)*rho*rudder.S*rudder.C_L_alpha;

% foil drag paramters
rudder.e_OS = 0.9; % OSwald efficiency factor typical value is in the range 0.8-1.0, see Introduction to Aircraft Flight Mechanics
rudder.C_D_parasitic = 0.008; %0.01373; % drag at 0 deg angle of attack (C_D(alpha) = C_D_parasitic + 0) of NACA 0015 wing

%% Hull
h_hull = 0.7;         % [m] hull height 
w_hull = 1;       % [m] hull width
l_hull = 7.0;       % [m] hull lenght
m_hull = 650;       % [kg] hull weight 

%% Keel
h_keel = 2;         % [m] keel height 
w_keel = 0.07;      % [m] keel width 
l_keel = 0.6;       % [m] keel lenght 
m_keel = 300;       % [kg] hull weight % check with respect to rho

% relation between weight of keel foil and keel (counter)weight
ratio_keel_weight = 0.20;
m_keel_foil = m_keel*ratio_keel_weight
m_keel_weight = m_keel*(1-ratio_keel_weight);

% Volum and area of keel for elliptic cylinder
V_keel = pi*h_keel*(w_keel/2)*(l_keel/2)
A_keel = pi*(w_keel/2)*(l_keel/2)

% foil lift parameters of symmetric rudder
keel.L = l_keel
keel.S = h_keel*l_keel;
keel.AR = (h_keel^2)/keel.S;
% rudder.C_L_0 = 0
keel.C_L_alpha = (pi*keel.AR)/(1 + sqrt(1 + (keel.AR/2)^2));
keel.K_C_L = (1/2)*rho*keel.S*keel.C_L_alpha;

% foil drag paramters
keel.e_OS = 0.9; % OSwald efficiency factor typical value is in the range 0.8-1.0, see Introduction to Aircraft Flight Mechanics
keel.C_D_parasitic = 0.008; %0.01373; %0.00000001;% 0.01373; % drag at 0 deg angle of attack (C_D(alpha) = C_D_parasitic + 0) of NACA 0015 wing

%% %%%%%%%%%%% Sail system parameters
% Assuming CG of sail system at pivot point

% sail
l_sail = 1.0;    % [m]
w_sail = 0.08%0.15;   % [m]
h_sail = 5;      % [m]
m_sail = 40;    % [kg]

% foil lift parameters of symmetric sail
sail.L = l_sail;
sail.S = h_sail*l_sail;
sail.AR = (h_sail^2)/sail.S;
sail.C_L_alpha = (pi*sail.AR)/(1 + sqrt(1 + (sail.AR/2)^2));
sail.K_C_L = (1/2)*rho_air*sail.S*sail.C_L_alpha;

% foil drag paramters
sail.e_OS = 0.9; % OSwald efficiency factor typical value is in the range 0.8-1.0, see Introduction to Aircraft Flight Mechanics
sail.C_D_parasitic = 0.008; %0.01373; % drag at 0 deg angle of attack (C_D(alpha) = C_D_parasitic + 0) of NACA 0015 wing

% tail
l_tail = 0.44;    % [m]
w_tail = 0.05;   % [m]
h_tail = 2.2;    % [m]
m_tail = 5;     % [kg]

% foil lift parameters of symmetric tail
tail.L = l_tail;
tail.S = h_tail*l_tail;
tail.AR = (h_tail^2)/tail.S;
tail.C_L_alpha = (pi*tail.AR)/(1 + sqrt(1 + (tail.AR/2)^2));
tail.K_C_L = (1/2)*rho_air*tail.S*tail.C_L_alpha;

% foil drag paramters
tail.e_OS = 0.9; % OSwald efficiency factor typical value is in the range 0.8-1.0, see Introduction to Aircraft Flight Mechanics
tail.C_D_parasitic = 0.008; %0.01373; % drag at 0 deg angle of attack (C_D(alpha) = C_D_parasitic + 0) of NACA 0015 wing

% counter weight
m_weight = m_tail;

% sail system mass
m_sailsystem = m_sail + m_tail + m_weight;

% Sail and weight (saw)combined mass
m_saw = m_sail + m_weight;

%% Total mass
m_saildrone = m_sailsystem + m_keel + m_hull;

%% Vessel mass
m_vessel = m_keel + m_hull;



%% %%%%%%%%%%%%%%%%%%%% Position vectors

%% Position vectors in sail system
r__s_stCG = [-2.3; 0; 0];          % vector from sail system pivot to tail CG in sail frame when tail is straight
r__s_stAC = r__s_stCG;           % vector from sail system pivot to tail AC in sail frame when tail is straight
r__s_sCG = [0; 0; 0];            % vector from sail system pivot to sail CG in sail frame
r__s_sAC = [0; 0; 0];            % vector from sail pivot to AC in sail frame

% select weight position such that CG of sail system remains on the pivot point
r__s_swCG = [-(r__s_stCG(1))*m_tail/m_weight; 0; 0]; % vector from sail system pivot to weight CG in sail frame  

% position of sailsystem CG
r__s_sssCG = (m_tail*r__s_stCG + m_weight*r__s_swCG + m_sail*r__s_sCG)/m_sailsystem; % vector from sail pivot frame to sail system CG (should be zero)

if (r__s_sssCG(1) ~= 0 || r__s_sssCG(2) ~= 0)
    fprintf('Warning sail system CG is not i pivot center, simplified model will diviate more')
end

% Position of sail and weight combined CG in sail frame
r__s_ssawCG = (m_sail*r__s_sCG + m_weight*r__s_swCG)/(m_sail+m_weight);

% Position vectors in tail
r__t_tCG = [-l_tail/4; 0; 0];     % CG at quater chord in tail
r__t_tAC = r__t_tCG;              % AC at quater chord in tail
r__s_st = r__s_stCG - r__t_tCG;   % Position of tail frame in sail frame 

if r__s_stCG ~= r__s_stAC
    fprintf('Warning tail AC and CG is not at same position, changes to r__s_st is needed, as it is used for calulation of the relative velocity at tail AC')
end
    
%% CG positions in body frame relative to hull center
r__b_hsCO = [0; 0; -(h_hull/2+h_sail/2)];    % Vector from hull center to sail pivot point and assumed CG for simplified system
r__b_hhCG = [0; 0; 0]                   ;    % Vector from hull center to hull CG
% r__b_hkCG = [0; 0; (h_hull/2+h_keel/2)] ;  % Vector from hull center to keel CG
r__b_hkwCG = [0; 0; (h_hull/2+h_keel)] ;     % Vector from hull center to keel weight CG
r__b_hkAC = [0; 0; (h_hull/2+h_keel/2)] ;    % Vector from hull center to keel AC and keel foil CG
r__b_hrAC = [-0.9*(l_hull/2); 0; (h_hull/2+h_rudder/2)]; % Vector from hull center to rudder AC
r__b_hkCG = (m_keel_foil*r__b_hkAC + r__b_hkwCG*m_keel_weight)/m_keel % Vector from hull center to keel CG


% for motion without roll and pitch 
% r__b_hsCO = [0; 0; 0];    % Vector from hull center to sail pivot point and assumed CG for simplified system
% r__b_hhCG = [0; 0; 0];                      % Vector from hull center to hull CG
% r__b_hkCG = [0; 0; 0];    % Vector from hull center to keel CG
% r__b_hkAC = [0; 0; 0] ;    % Vector from hull center to keel AC
% r__b_hrAC = [-0.9*(l_hull/2); 0; 0]; % Vector from hull center to rudder AC


% Vector from hull center to vessel (hull and keel) CG
r__b_hvCG = (m_hull*r__b_hhCG + m_keel*r__b_hkCG)/(m_hull + m_keel)
% Vector from hull center to saildrone CG
r__b_hCG = (m_hull*r__b_hhCG + m_keel*r__b_hkCG + m_sailsystem*r__b_hsCO)/(m_hull + m_keel + m_sailsystem)

%% Position of CO relative to hull center
r__b_hCO = [0; 0; 0];%[1; 0; 0]; 
% Vector from body origin CO to saildrone CG
r__b_bg = -r__b_hCO + r__b_hCG;
% Vector from body origin CO to vessel CG
r__b_bvg = -r__b_hCO + r__b_hvCG;

% Position relative to CO 
r__b_bs = -r__b_hCO + r__b_hsCO     % CO to sail frame in body
r__b_bhCG = -r__b_hCO + r__b_hhCG   % CO to hull CG
r__b_bkCG = -r__b_hCO + r__b_hkCG   % CO to keel CG
r__b_bkAC = -r__b_hCO + r__b_hkAC   % CO to keel AC
r__b_brAC = -r__b_hCO + r__b_hrAC   % CO to rudder AC

% Position relative to CG for full saildrone
r__b_gkCG = - r__b_hCG + r__b_hkCG  % CG to keel CG   
r__b_gsCG = - r__b_hCG + r__b_hsCO  % CG to sail system CG
r__b_ghCG = - r__b_hCG              % CG to hull CG
r__b_gkAC = - r__b_hCG + r__b_hkAC  % CG to keel AC
r__b_gkwCG = - r__b_hCG + r__b_hkwCG % CG to keel weight CG

% Position relative to CG for keel and hull (vessel)
r__b_vgkCG = - r__b_hvCG + r__b_hkCG
r__b_vghCG = - r__b_hvCG
r__b_vgkAC = - r__b_hvCG + r__b_hkAC
r__b_vgkwCG = - r__b_hvCG + r__b_hkwCG

% Position relative to SAW CG (sail and weight)
r__sawCGsCG = -r__s_ssawCG + r__s_sCG
r__sawCGwCG =  r__s_swCG - r__s_ssawCG


%% %%%%%%%%%%% Transforms

% body and sail
epsilon = 0; % nominal sail angle for simplified model, for calulating inertia of sailsystem 
R__b_s_nom = Rzyx(0,0,epsilon);
R__s_b_nom = transpose(R__b_s_nom);


%% %%%%%%%%%%%% Inertias 

%% Hull inertia
% Model hull as homogen ellipsoid
I__b_xx_hull = m_hull*(1/5)*((h_hull/2)^2 + (w_hull/2)^2) % inertia [kgm^2]
I__b_yy_hull = m_hull*(1/5)*((h_hull/2)^2 + (l_hull/2)^2)
I__b_zz_hull = m_hull*(1/5)*((l_hull/2)^2 + (w_hull/2)^2)


% Model as 1 elliptic cylinder, making it easier to caltulate the draft T
% I__b_xx_hull = m_hull*((1/4)*(w_hull/2)^2 + (1/12)*h_hull^2)
% I__b_yy_hull = m_hull*((1/4)*(l_hull/2)^2 + (1/12)*h_hull^2)
% I__b_zz_hull = m_hull*(1/4)*((w_hull/2)^2 + (l_hull/2)^2)

% Model as 2 elliptic cylinder, to have better control of CG?

% Hull inertia in hull CG
I__b_hull = diag([I__b_xx_hull, I__b_yy_hull, I__b_zz_hull])

% Volum and area of hull for elliptic cylinder
V_hull = pi*h_hull*(w_hull/2)*(l_hull/2)
A_hull = pi*(w_hull/2)*(l_hull/2)


%% Keel inertia
% Model keel as  elliptic cylinder
% ratio_keel_weight = 0.2;
% m_keel_foil = m_keel*ratio_keel_weight
% m_keel_weight = m_keel*(1-ratio_keel_weight);

% I__b_xx_keel = m_keel*((1/4)*(w_keel/2)^2 + (1/12)*h_keel^2);
% I__b_yy_keel = m_keel*((1/4)*(l_keel/2)^2 + (1/12)*h_keel^2);
% I__b_zz_keel = m_keel*(1/4)*((w_keel/2)^2 + (l_keel/2)^2);

I__b_xx_keel = m_keel_foil*((1/4)*(w_keel/2)^2 + (1/12)*h_keel^2);
I__b_yy_keel = m_keel_foil*((1/4)*(l_keel/2)^2 + (1/12)*h_keel^2);
I__b_zz_keel = m_keel_foil*(1/4)*((w_keel/2)^2 + (l_keel/2)^2);

% Keel foil inertia in keel center (AC)
I__b_keel = diag([I__b_xx_keel, I__b_yy_keel, I__b_zz_keel]) 

% Keel weight is modeled as a point mass, see calulation of inertia further down


%% Rudder model
% Assume rudder inertia to be so low that it has neglectble effect
% 

%% Inertia of sail system

% tail inertia in nominal inertia center of tail assumed to be in AC and center of box shape
I_tail_zz = m_tail*(l_tail^2+w_tail^2)/12;
I_tail_yy = m_tail*(l_tail^2+h_tail^2)/12;
I_tail_xx = m_tail*(h_tail^2+w_tail^2)/12;

% inertia of tail in tail CG
I_tail = diag([I_tail_xx, I_tail_yy, I_tail_zz])

cfg.I__t_t_tail = parallelAxisTH(r__t_tCG,m_tail,I_tail) % inertia to use in tail angle controller

% sail inertia in nominal inertia center of sail
I_sail_zz = m_sail*(l_sail^2+w_sail^2)/12;
I_sail_yy = m_sail*(l_sail^2+h_sail^2)/12;
I_sail_xx = m_sail*(h_sail^2+w_sail^2)/12;

% inertia of sail in sail CG
I_sail = diag([I_sail_xx, I_sail_yy, I_sail_zz])

% Inertia at pivot point in sail frame
I__s_s_tail = parallelAxisTH(r__s_stCG,m_tail,I_tail)
I__s_s_sail = parallelAxisTH(r__s_sCG, m_sail,I_sail)

% counter weight inertia in sail frame (point mass) 
I__s_s_weight = parallelAxisTH(r__s_swCG,m_weight,zeros(3,3))

% Inertia of sailsystem in sail frame
I__s_ss = I__s_s_sail + I__s_s_tail + I__s_s_weight % is diagonal
cfg.I__PP_saw = I__s_s_sail + I__s_s_weight % inertia of saw in pivot point

% Inertia in sail direction subsystem (rotation of sail system around pivot point)  
I__s_sailsys_zz = (zeros(3,3)); % assuming low quadratic velocity is dominated by anglar acceleration
I__s_sailsys_zz(3,3) = I__s_ss(3,3)

% Inertia transfered to the vessel
I__s_sailsys = I__s_ss;     
I__s_sailsys(3,3) = 0;                              % remove inertia around pivot point
I__b_sailsys = R__b_s_nom*I__s_sailsys*transpose(R__b_s_nom)  % similarity transform coorect?%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inertia of sail and weight (saw) at there combined CG
I__s_saw_sail = parallelAxisTH(r__sawCGsCG, m_sail,I_sail)        % use vector from combined CG to sail CG
I__s_saw_weight = parallelAxisTH(r__sawCGwCG,m_weight,zeros(3,3)) % use vector from combined CG to weight CG 
I__s_saw = I__s_saw_weight + I__s_saw_sail                        % Inertia of SAW in SAW CG in sail frame

%% Saildrone inertia

% have calulated inertia of sail system in body

I__b_hull_g = parallelAxisTH(r__b_ghCG,m_hull,I__b_hull);
I__b_keel_g = parallelAxisTH(r__b_gkAC,m_keel_foil,I__b_keel) + parallelAxisTH(r__b_gkwCG,m_keel_weight,zeros(3,3));
I__b_sailsys_g = parallelAxisTH(r__b_gsCG,m_sailsystem,I__b_sailsys);

% the inertia of the hull and keel is so high that the I__b_g never becomes
% singular due to epsilon ?
I__b_g = I__b_hull_g + I__b_keel_g + I__b_sailsys_g;

% The inertia of the sail system is dominated by the keel and hull, hence a
% simplification can be to use the inertia of the sail system when the sail
% is at 45 deg, the cross terms at 45 deg are neglectale hence use 0 instead, creating the fastest roll dynamics 
I__CG_saildrone = I__b_g

%% Vessel inertia

I__b_hull_vg = parallelAxisTH(r__b_vghCG,m_hull,I__b_hull);
I__b_keel_vg = parallelAxisTH(r__b_vgkAC,m_keel_foil,I__b_keel) + parallelAxisTH(r__b_vgkwCG,m_keel_weight,zeros(3,3));
I__CG_vessel = I__b_hull_vg + I__b_keel_vg
%% Vessel model
% The draft T seems to only be used for cross flow drag: use only draft of hull 
% as drag due to keel will be separate
% 
% Keel will allways be under water hence on adjust when calulating T

%% Draft, volum, surface and coefficients
% low draft compared to pictures -> volum of ellipsoid is smaller than for elliptic cylinder
Cb_hull = 0.75; %4/6; % the volumetric relation between a ellepsioide and the real hull

nabla = (m_saildrone)/rho               % nabla = displaced water volum
V_displacement_hull = nabla - V_keel    % Water volum displaced by hull 
% estimated real submerged volum of hull is T*A_hull*Cb_hull = V_displacement_hull
% T = V_displacement_hull/(A_hull*Cb_hull)          
T = (3/2)*(V_displacement_hull/(pi*(w_hull/2)*(l_hull/2)))/Cb_hull % Draft of hull


% vector from cg to CB -> g(eta)?
% total CB is a sum of forces hence it can be found similar to cg

% vector from hull center to bouyancy point in body frame
% r__b_hvbuoy = (V_keel*r__b_hkCG + (V_displacement_hull)*r__b_hhCG)/(V_keel + V_displacement_hull);
r__b_hvbuoy = [0; 0; ((-T*(5/8)+(h_hull/2))*V_displacement_hull + r__b_hkAC(3)*V_keel)/nabla]; % Volum center of semi ellepsioide at 3/8 of height or 5/8 from hull bottom
r__b_bbuoy = -r__b_hCO + r__b_hvbuoy;

if T > h_hull
    fprintf('Warning draft is larger than height of hull, is this a USV or AUV?')
elseif T > h_hull*(3/4)
    fprintf('Warning draft is large = 3/4 of hull height, parameter error?')
elseif T < h_hull*(1/4)
    fprintf('Warning draft is low = 1/4 of hull height, parameter error?')
end

% Wetted surface of the vessel, approximated by a semi ellepsiode with the
% height of T. Assuming T will be close to the half of the hull
% The Knud Thomsen formula for the surface area of an semi ellipsoid
k_KTF = 1.6075;
cfg.WS = (1/2)*4*pi*((((l_hull/2)*(w_hull/2))^k_KTF + ((l_hull/2)*(T))^k_KTF + ((w_hull/2)*(T))^k_KTF )/3)^(1/k_KTF);

% Precalculate Cd_2d table
Lpp = l_hull;
Cd_2d = [];
% rgdg = []
dx = l_hull/10;
for xL = (dx/2 - Lpp/2):dx:(Lpp/2 - dx/2)
    B = sqrt(((w_hull/2)^2) * (1 - (xL^2/((l_hull/2)^2))));
    Cd_2d = [Cd_2d Hoerner(B,T)];
%     rgdg = [rgdg B]
end

cfg.Cd_2d = Cd_2d;

%% Meta center height
% keel is accounded for when calulating the vessel and saildrone CG and CB
% second moment of area assuming the draft corespond to half the hull height
I_L = (pi/4)*((l_hull/2)^3)*(w_hull/2);
I_T = (pi/4)*((w_hull/2)^3)*(l_hull/2);

KG = (h_hull/2-r__b_bg(3));
KB = (h_hull/2-r__b_bbuoy(3));
BG = KG - KB;

BM_T = I_T/nabla;
BM_L = I_L/nabla;

cfg.GM_T = BM_T - BG;
cfg.GM_L = BM_L - BG;

% Links 1 / vessel only
KG_vessel = (h_hull/2-r__b_bvg(3));
BG_vessel = KG_vessel - KB;
cfg.GM_T_vessel = BM_T - BG_vessel
cfg.GM_L_vessel = BM_L - BG_vessel

% Assume half of the hull is submerged
LFC = -r__b_hCO(1); % CF at hull center relative to CO

% cfg.G_mtrx = Gmtrx(V_displacement_hull, A_hull, cfg.GM_T, cfg.GM_L, LFC, [0;0;0]);
% cfg.G_mtrx_vessel = Gmtrx(V_displacement_hull, A_hull, cfg.GM_T_vessel, cfg.GM_L_vessel, LFC, [0;0;0]);
cfg.G_mtrx = Gmtrx(nabla, A_hull, cfg.GM_T, cfg.GM_L, LFC, [0;0;0]);
cfg.G_mtrx_vessel = Gmtrx(nabla, A_hull, cfg.GM_T_vessel, cfg.GM_L_vessel, LFC, [0;0;0]);


%% %%%%%%%%%%%%%%%%%%%%%%% Save paramters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cfg.rho = rho;
cfg.rho_air = rho_air;
cfg.T = T;

%% Sail subsystem

% Tail dimensions
cfg.m_tail = m_tail;
cfg.h_tail = h_tail;
cfg.l_tail = l_tail;
cfg.w_tail = w_tail;

% Sail dimensions
cfg.m_sail = m_sail;
cfg.h_sail = h_sail;
cfg.l_sail = l_sail;
cfg.w_sail = w_sail;

cfg.m_weight = m_weight;

% Sailsystem
cfg.m_sailsystem = m_sailsystem;

% Sail and counter weight (saw)
cfg.m_saw = m_saw;

%% Vessel parameters
cfg.m_vessel = m_vessel;
cfg.l_hull = l_hull;

% Keel dimentions
cfg.m_keel = m_keel;
cfg.h_keel = h_keel;        
cfg.w_keel = w_keel;    
cfg.l_keel = l_keel;     

% Rudder dimentions
cfg.h_rudder = h_rudder;
cfg.w_rudder = w_rudder;
cfg.l_rudder = l_rudder;



%% FULL Saildrone parameters
cfg.m_saildrone = m_saildrone;
cfg.T_ss = T;        % steady state draft of saildrone

% Definitions
I3x3 = eye(3);
O3x3 = zeros(3,3);

% System inertia and in CG of entire SailDrone (used for natural frequency calculations)
MRB_CG_SD = [m_saildrone*I3x3,        O3x3;
             O3x3,                I__CG_saildrone];
         
% System inertia in the body frame
H = Hmtrx(r__b_bg);
cfg.MRB_SD = transpose(H)*MRB_CG_SD*H;


% Hydrodynamic added mass (best practise) from FOSSEN MSS toolbox for otter vessel
cfg.Xudot = -addedMassSurge(m_saildrone,l_hull); % use the parameter for the full drone, default rho = 1025  
cfg.Yvdot = -1.5 * m_saildrone;
cfg.Zwdot = -1.0 * m_saildrone;
cfg.Kpdot = -0.2 * I__CG_saildrone(1,1);
cfg.Mqdot = -0.8 * I__CG_saildrone(2,2);
cfg.Nrdot = -1.7 * I__CG_saildrone(3,3);

MA = -diag([cfg.Xudot, cfg.Yvdot, cfg.Zwdot, cfg.Kpdot, cfg.Mqdot, cfg.Nrdot]);

cfg.M_SD = cfg.MRB_SD + MA;

cfg.Xu = -cfg.M_SD(1,1)/cfg.T_surge;
cfg.Yv = -cfg.M_SD(2,2)/cfg.T_sway;
cfg.Nr = -cfg.M_SD(6,6)/cfg.T_yaw; 

%% System inertia and coriols-centripital matrix
% link based model
cfg.I__CG_vessel =  I__CG_vessel;
cfg.I__CG_sail =    I_sail;
cfg.I__CG_saw =     I__s_saw;            % inertia of sail and weight link in there CG
cfg.I__CG_tail =    I_tail;
% decoupled model
cfg.I__CG_saildrone = I__CG_saildrone;        % simplified
cfg.I__PP_sailsystem = I__s_sailsys_zz;      % simplified inertia at sail pivot point

% These under (C) will vary
%         M_vessel    % at {b}
%         C_vessel    % at {b}
%         
% cfg.M_saildrone % at {b} (simplified) will be constant
% cfg.C_saildrone % at {b}
%         
%         MRB_sail    % at {s}
%         CRB_sail
%         
%         MRB_tail    % at {t}
%         CRB_tail    % at {t}

%% Position vectors of CG
% CG positions
cfg.r__b_bg = r__b_bg;           % postion of total CG of saildrone (simplified)
cfg.r__b_bvg = r__b_bvg;         % position of vessel CG in body frame 
cfg.r__s_sg = r__s_sCG;          % position of sail CG in sail (at pivot point) 
cfg.r__t_tg = r__t_tCG;          % position of tail CG in tail
cfg.r__s_sssg = r__s_sssCG;      % position of sailsystem CG (ssCG) in sail frame
cfg.r__s_swg = r__s_swCG;        % position of counter weight in sailframe
cfg.r__s_ssawg = r__s_ssawCG;    % position of combined CG of sail and weight given in sail frame

%         r__b_gssg    % position of sail system CG relative to saildrone CG?
%         r__b_bkg     % position of keel CG in body (included in vessel CG)

% Frame positions
cfg.r__b_bs = r__b_bs;     % position of sail frame in body
cfg.r__s_st = r__s_st;     % position of tail frame in sail frame

% AC positions
cfg.r__b_bkAC = r__b_bkAC;  % position of keel AC in body
cfg.r__b_brAC = r__b_brAC;   % CO to rudder AC
cfg.r__s_sAC  = r__s_sAC;   % position of aerodynamic center in sail frame
cfg.r__t_tAC  = r__t_tAC;   % position of aerodynamic center in tail frame
cfg.r__s_stAC = r__s_stAC;   % position of AC of tail in sail frame (simplified with tail straight)

% Buoyency position
cfg.r__b_bbuoy = r__b_bbuoy;

%% Volum and displacement parameters
cfg.V_hull = V_hull;
cfg.A_hull = A_hull;

cfg.V_keel = V_keel;
cfg.A_keel = A_keel;