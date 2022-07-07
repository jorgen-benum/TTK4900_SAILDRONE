
deg2rad = pi/180;

%% Tail moment controller
% sampling time 
lowel_moment_controller_Hz = 100
lowel_moment_controller_samplingtime = 1/lowel_moment_controller_Hz

% Controller gain

zeta_t = 1 % damping ratio
omega_bw_t = 6*2*pi%5*2*pi%6*2*pi%6*2*pi%1*2*pi%3*2*pi%3*2*pi%5*2*pi % 
omega_n_t = omega_bw_t/(sqrt(1 - 2*zeta_t^2 + sqrt(4*zeta_t^4 - 4*zeta_t^2 + 2))) % natural frequency give omega_bandwith 
 
I_zz_tail_pivot = cfg.I__t_t_tail(3,3); % Inertia of tail around tail pivot point

K_p_t = (omega_n_t^2)*I_zz_tail_pivot
K_d_t = 2*zeta_t*omega_n_t*I_zz_tail_pivot
K_i_t = 0; %(omega_n_t/35)*K_p_t; REDUCED TO PD controller, Remember AW if used

%% Tail reference model

wn_ref_t = omega_n_t*0.4%0.9; % reference model 5% slower
zeta_ref_t = 1;


%% Sail AOA controller
% sampling time 
sail_AOA_controller_Hz = 40 %100 % 40 give margin of 6.7 down to controller bw at 12m/s wind
sail_AOA_controller_samplingtime = 1/sail_AOA_controller_Hz
sail_AOA_saturation = 55*deg2rad;
sail_AOA_linearization_point = pi/4; %linearize system around this angle of attacl

zeta_s = 1

% gain example calulation
omega_bw_s = 0.05*pi*2%0.05*pi*2%0.078*2*pi %0.053*2*pi%0.02*2*pi %U_r_tail_eks*omega_bw_slope%0.1*2*pi%0.4*2*pi%0.2*2*pi%0.3*2*pi%0.04*2*pi%0.5*2*pi%0.08*2*pi                 % 0.3 for high wind!
omega_n_s = omega_bw_s/(sqrt(1 - 2*zeta_s^2 + sqrt(4*zeta_s^4 - 4*zeta_s^2 + 2)))

% Constants in invaraint system
K_a = r__s_stAC(1)^2*((cos(sail_AOA_linearization_point))^2)*tail.K_C_L
K_b = r__s_stAC(1)*((cos(sail_AOA_linearization_point)))*tail.K_C_L

k_p_s = ((omega_n_s^2)*I__s_sailsys_zz(3,3)+K_b)/-K_b
k_d_s = (-2*zeta_s*omega_n_s*I__s_sailsys_zz(3,3) + K_a)/K_b
k_i_s = 1*omega_n_s*k_p_s/10


