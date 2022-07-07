%% Optimal sail AOA lookup table  
% sampling time 
sail_AOA_lookup_Hz = 40 %40 % 10
sail_AOA_lookup_samplingtime = 1/sail_AOA_lookup_Hz


%% Heading controller

% sampling time
heading_controller_Hz = 40;
heading_controller_samplingtime = 1/heading_controller_Hz;

% U_linearization_heading = 2%2; % velocity system is lineaiazied at
% zeta_heading = 1.5%1.2;%1.5
% omega_bw_heading = 0.05*2*pi%0.07*2*pi;
% omega_n_heading = omega_bw_heading/(sqrt(1 - 2*zeta_heading^2 + sqrt(4*zeta_heading^4 - 4*zeta_heading^2 + 2)))

% K_p_heading =(omega_n_heading^2)*cfg.M_SD(6,6) % inertia and added mass inertia
% K_d_heading = 2*zeta_heading*omega_n_heading*cfg.M_SD(6,6) + cfg.Nr - rudder.K_C_L*((r__b_brAC(1))^2)*U_linearization_heading
% K_i_heading = 0*omega_n_heading*K_p_heading/10%0%0.5*omega_n_heading*K_p_heading/10 %0
% 
% Ka_lin = (cfg.r__b_brAC(1)^2)*rudder.K_C_L*U_linearization_heading - cfg.Nr
% Kb_lin = cfg.r__b_brAC(1)*rudder.K_C_L*U_linearization_heading^2
% 
% K_p_head_lin = -omega_n_heading^2*cfg.M_SD(6,6)/Kb_lin
% K_d_head_lin = (Ka_lin - 2*zeta_heading*omega_n_heading*cfg.M_SD(6,6))/Kb_lin
% K_i_head_lin = 1*omega_n_heading*K_p_head_lin/10


zeta_heading2 = 1.5%1.5%1.2%1.2%1.5;
omega_bw_heading2 = 0.04*2*pi%0.04*2*pi %0.03*2*pi%0.05*2*pi%0.02*2*pi;
omega_n_heading2 = omega_bw_heading2/(sqrt(1 - 2*zeta_heading2^2 + sqrt(4*zeta_heading2^4 - 4*zeta_heading2^2 + 2)))


Kah = (cfg.r__b_brAC(1)^2)*rudder.K_C_L;
Kbh = cfg.r__b_brAC(1)*rudder.K_C_L;

K_p_heading2 =(-omega_n_heading2^2)*cfg.M_SD(6,6)/Kbh
K_d_heading2 = (-2*zeta_heading2*omega_n_heading2*cfg.M_SD(6,6) + Kah)/Kbh
K_i_heading2 = 1*omega_n_heading2*K_p_heading2/10

%% Heading reference model

wn_ref_heading = omega_n_heading2*0.5%0.5%0.9;%%0.5;
zeta_ref_heading = 1;
heading_rate_ref = (90/10)*pi/180;

% zet_test = (Ka_lin-K_d_head_lin*Kb_lin)/(2*omega_n_heading*cfg.M_SD(6,6)) % with Ka
% zet_test = (-K_d_head_lin*Kb_lin)/(2*omega_n_heading*cfg.M_SD(6,6)) % witout Ka