

deg2rad = pi/180;



%% Tail moment controller
% sampling time 
lowel_moment_controller_Hz = 100
lowel_moment_controller_samplingtime = 1/lowel_moment_controller_Hz

% Controller gain

zeta_t = 1 % damping ratio
omega_bw_t = 6*2*pi%1*2*pi%3*2*pi%3*2*pi%5*2*pi % bandwidth
omega_n_t = omega_bw_t/(sqrt(1 - 2*zeta_t^2 + sqrt(4*zeta_t^4 - 4*zeta_t^2 + 2))) % natural frequency give omega_bandwith 
 
I_zz_tail_pivot = cfg.I__t_t_tail(3,3); % Inertia of tail around tail pivot point

K_p_t = (omega_n_t^2)*I_zz_tail_pivot
K_d_t = 2*zeta_t*omega_n_t*I_zz_tail_pivot
K_i_t = 0; %(omega_n_t/35)*K_p_t; REDUCED TO PD controller, Remember AW if used

%% Tail reference model

wn_ref_t = omega_n_t*0.9; % reference model 5% slower
zeta_ref_t = 1;




%% Sail AOA controller
% sampling time 
sail_AOA_controller_Hz = 40 %100 % 40 give margin of 6.7 down to controller bw at 12m/s wind
sail_AOA_controller_samplingtime = 1/sail_AOA_controller_Hz
sail_AOA_saturation = 55*deg2rad;
sail_AOA_linearization_point = pi/4; %linearize system around this angle of attacl

% comment 0.8 and slope (0.33/10)*2*pi give good response however 0.5 and is faster (0.5/10)*2*pi 
zeta_s = 0.5%0.8%0.5%1%0.5%0.8%1%0.8%0.5%1%0.5
omega_bw_slope = (0.5/10)*2*pi%(0.45/10)*2*pi%(0.33/10)*2*pi%(0.13/10)*2*pi%(0.5/10)*2*pi 


% gain example calulation
% at tail speed :
U_r_tail_eks = 12;
omega_bw_s = U_r_tail_eks*omega_bw_slope%0.1*2*pi%0.4*2*pi%0.2*2*pi%0.3*2*pi%0.04*2*pi%0.5*2*pi%0.08*2*pi                 % 0.3 for high wind!
omega_n_s = omega_bw_s/(sqrt(1 - 2*zeta_s^2 + sqrt(4*zeta_s^4 - 4*zeta_s^2 + 2)))

%0.5*2*pi good for high speeds
%0.5 good for high speeds
%low speed
%0.2*2*pi% good at 4m/s

K_p_s = (omega_n_s^2)*I__s_sailsys_zz(3,3)
K_d_s = 2*zeta_s*omega_n_s*I__s_sailsys_zz(3,3)
K_i_s = 1*omega_n_s*K_p_s/10 % for 3m/s and up
% eksample gain might look high but remeber that it will be devided by the
% inertia when entering the system 

%% Sail AOA reference model

wn_ref_sAOA = omega_n_s*0.9;
zeta_ref_sAOA = 1;
