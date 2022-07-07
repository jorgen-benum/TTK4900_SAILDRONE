clear all
close all
clc

%% paths
addpath(genpath('C:\Users\jorge\local_repo\TTK4900_SAILDRONE\Modeling'))
addpath(genpath('C:\Users\jorge\local_repo\TTK4900_SAILDRONE\Plotting'))
addpath(genpath('C:\Users\jorge\local_repo\TTK4900_SAILDRONE\Control'))
addpath(genpath('C:\Users\jorge\local_repo\TTK4900_SAILDRONE\Guidance'))
addpath(genpath('C:\Users\jorge\local_repo\TTK4900_SAILDRONE\lookupTables'))
addpath(genpath('C:\Users\jorge\local_repo\TTK4900_SAILDRONE\COLAV'))

%% Load data
sail_AOA_table = load('sail_AOA_table'); % the optimal angle with respect to body x axis
% sail_AOA_table_relative = load('sail_AOA_table_relative');

tack_mi = load('C:\Users\jorge\local_repo\TTK4900_SAILDRONE\lookupTables\VMG\tack_minus.mat')
tacking_minus = tack_mi.tacklist_minus

tack_pu = load('C:\Users\jorge\local_repo\TTK4900_SAILDRONE\lookupTables\VMG\tack_plis.mat')
tacking_plus = tack_pu.tacklist_plus

%% Switches
animation_on = false;                       % anitmate result
plot_simplified_model_response_on = true;   % plot respone of simplified model
display_saildrone = true;                   % plot saildrone pose along the path


%% Init saildorne configuration
init_saildrone_dynamics_v5;     % init sail drone dynamics and its configuration
init_low_level_controllers_2;   % init all low level controllers
init_mid_level_controllers;     % init all mid level controllers
init_guidance;

%% parameters 
theta0 = 90*pi/180;%45*pi/180;      % heading
delta_t0 = 0;                       % tail angle relative to sail
delta_r0 = 0;                       % rudder angle relative to heading
beta_V_w = 180*pi/180               % wind direction
gamma_c = 180*pi/180;               % current direction
U_c = 0.1
U_w = 10;
sail_wind_offset = 0*pi/180         % initial angle between wind and sail 

U_w_step = 0;
timeconstant_w = 50;
start_w_trans = 600;

v_wind_unit = [cos(beta_V_w); sin(beta_V_w); 0];
v_wind = [U_w*cos(beta_V_w); U_w*sin(beta_V_w); 0];
v_current = [U_c*cos(gamma_c); U_c*sin(gamma_c); 0];

% initial conditions calculation
xi0 = ssa(beta_V_w - pi + sail_wind_offset)
epsilon0 = ssa(xi0 - theta0);     % sail angle relative to heading


q0 = [0;0;0; 0;0;theta0; epsilon0; delta_t0; delta_r0]
% % simplified model is not fan of negative velocity
% q_dot0 = [0.1;0;0;0;0;0;0;0;0]
% % aoide start jumps
q_dot0 = [0.0;0;0;0;0;0;0;0;0]


xi0 = ssa(q0(6) + q0(7))

%% Obsticals
ObjectsPosition = [400,   -1000; 
                   30, -1000]

ObjectsPosition = [1000,   -1000; 
                   1000, -1000]
               
% ObjectsPosition = [300,   -1000;  % case 1
%                    75, -1000]
               
% ObjectsPosition = [350,   -1000;  % case 2
%                    75, -1000]
              
% ObjectsPosition = [400,   -1000;  % case 3
%                    75, -1000]
             
obstical_radius = 20; % [m]
d_max = 30;           % largest tacking distance   
d_safe = 20;          % safety margin
d_USV = 7;            % hull is a circle of diameter [m]  



%% Waypoints

 WP = [  0   26.8929  209.2025  234.7627  250.0000, nan;    % x-components
         0   50.0000  -40.0000    7.5221         0, nan];   % y-components

      
GWP = [  0,  200, 250,  -30, 250;
         0,  0,    250, 0, -250]; 

     %%%%%%%%%%%%%%%%%%%%%%%%%%% jib testing %%%%%%%%%%%%%%%%

GWP = [  0, 0, 500, 150,  0, -50,    0;
         0, 50, 50, 150, 0, -280, -1000];
     
GWP = [  0, 0,  200, 250,  500, 450, 350, 250, 250, 0, 0, 0;
         0, 50, 50,   0,    50, 250, 350, 250, 350, 350,0, -1000];
    
%      
%      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%      %%%%%%%%%%%%%%%%%%%%%%%%%%% jib testing 2 %%%%%%%%%%%%%%%%
%      
% GWP = [0, 0, -50, 150, 0, 450, 0;
%        0, 50, 50, 150, 150, 0,   0]
%    
% GWP = [0, 0, -50, 150, 150, 450, 0;
%        0, 50, 50, 150, 0, 0,   0]
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

     

display_nr_GWP = 5;
display_nr_GWP = max(size(GWP,2), display_nr_GWP);

%% Simulation in simulink

simulation_sampling_time = 0.1;
simulation_time = 100; % length of simulation in [s]
simulation = sim('combined_models_v16',simulation_time); % run simulink

%% extract data 
% heading 
theta = simulation.heading_states(:,1);
beta_c = simulation.crab_angle;
chi = theta + beta_c;

delta_r = simulation.heading_states(:,2);

% wind and sail
% beta_V_w = simulation.sail_states(:,1);
epsilon = simulation.sail_states(:,1);

% tail
delta_t = simulation.tail_states(:,1);

% for testing
U = simulation.speed_states(:,1);

% position data in NED
x__n = simulation.position_states(:,1);
y__n = simulation.position_states(:,2);
z__n = zeros(length(x__n),1);
position = [x__n'; y__n'; z__n'];


%% Animaate simulation data
animation_speed_gain = 5;  % reduce pauseing to increase animation speed
animation_resolution = 1; % plot every x point of the data (must be >=1)

if animation_on == true
    g = figure(1);
    steps = floor(length(theta)/animation_resolution);
    for i= 1:steps
        i_scaled = i*animation_resolution;
        plot_saildrone_2D(theta(i_scaled), chi(i_scaled), U(i_scaled), beta_V_w, U_w, gamma_c, U_c, epsilon(i_scaled), delta_t(i_scaled), delta_r(i_scaled), [x__n(i_scaled);y__n(i_scaled);0], true, true);
        pause(simulation_sampling_time/animation_speed_gain)
        if i ~= steps
            clf(g)
        end
    end
end
%% Plot path
plot_resolution = 160;

figure(2)
steps = floor(length(theta)/plot_resolution);
pathSpeedPlotter(position, U); % plot path
hold on
plot(GWP(2,1:display_nr_GWP), GWP(1,1:display_nr_GWP), 'p','color','#0072BD')
viscircles([GWP(2,1:display_nr_GWP)', GWP(1,1:display_nr_GWP)'], 20*ones(1,display_nr_GWP), 'color','#0072BD', 'linewidth', 1, 'linestyle', ':')
plot(GWP(2,1:display_nr_GWP), GWP(1,1:display_nr_GWP), ':','color','#A2142F', 'linewidth', 1)
viscircles([ObjectsPosition(2,:)', ObjectsPosition(1,:)'], (obstical_radius)*ones(1,length(ObjectsPosition)), 'color','k', 'linewidth', 2, 'linestyle', '-')
viscircles([ObjectsPosition(2,:)', ObjectsPosition(1,:)'], (obstical_radius+d_safe+d_USV/2)*ones(1,length(ObjectsPosition)), 'color','#0000FF', 'linewidth', 0.5, 'linestyle', '-.')
viscircles([ObjectsPosition(2,:)', ObjectsPosition(1,:)'], (obstical_radius+d_max+d_safe+d_USV/2)*ones(1,length(ObjectsPosition)), 'color','#0072BD', 'linewidth', 0.5, 'linestyle', '-.')
if display_saildrone == true
    for i= 1:steps
        i_scaled = i*plot_resolution;
        plot_saildrone_2D(theta(i_scaled), chi(i_scaled), U(i_scaled), beta_V_w, U_w, gamma_c, U_c, epsilon(i_scaled), delta_t(i_scaled), delta_r(i_scaled), [x__n(i_scaled);y__n(i_scaled);0], false, false) % plot state along path

    end
end
% plot GWP


% plot(position(2),position(1)) % plot path
% 
% fprintf('simulation end \n')


%% extract data 
% heading 
% theta_si = simulation.heading_states_lin(:,1);
% beta_c_si = simulation.crab_angle_si;
% chi_si = theta_si + beta_c_si;
% 
% delta_r_si = simulation.heading_states_lin(:,2);
% 
% % wind and sail
% % beta_V_w = simulation.sail_states(:,1);
% epsilon_si = simulation.sail_states_lin(:,1);
% 
% % tail
% delta_t_si = simulation.tail_states_lin(:,1);
% 
% % for testing
% U_si = simulation.speed_states_si(:,1);
% 
% % position data in NED
% x__n_si = simulation.position_states_lin(:,1);
% y__n_si = simulation.position_states_lin(:,2);
% z__n_si = zeros(length(x__n_si),1);
% position = [x__n_si'; y__n_si'; z__n_si'];
% % 
% % 
% % %% Animaate simulation data simplified model
% % 
% % animation_speed_gain = 5;  % reduce pauseing to increase animation speed
% % animation_resolution = 1; % plot every x point of the data (must be >=1)
% % 
% % if animation_on == true
% %     g = figure(1);
% %     steps = floor(length(theta_si)/animation_resolution);
% %     for i= 1:steps
% %         i_scaled = i*animation_resolution;
% %         plot_saildrone_2D(theta_si(i_scaled), chi_si(i_scaled), U_si(i_scaled), beta_V_w, U_w, gamma_c, U_c, epsilon_si(i_scaled), delta_t_si(i_scaled), delta_r_si(i_scaled), [x__n_si(i_scaled);y__n_si(i_scaled);0], true, true);
% %         pause(simulation_sampling_time/animation_speed_gain)
% %         if i ~= steps
% %             clf(g)
% %         end
% %     end
% % end
% % %% Plot path
% % plot_resolution = 80;
% % 
% if plot_simplified_model_response_on == true
%     figure(2)
%     steps = floor(length(theta_si)/plot_resolution);
%     pathSpeedPlotter(position, U_si); % plot path
%     hold on
%     for i= 1:steps
%         i_scaled = i*plot_resolution;
%         plot_saildrone_2D(theta_si(i_scaled), chi_si(i_scaled), U_si(i_scaled), beta_V_w, U_w, gamma_c, U_c, epsilon_si(i_scaled), delta_t_si(i_scaled), delta_r_si(i_scaled), [x__n_si(i_scaled);y__n_si(i_scaled);0], false, false) % plot state along path
% 
%     end
%     % plot(position(2),position(1)) % plot path
% end
% 
% % fprintf('simulation end \n')


% 
% 
% 
% % %% extract data 
% % % heading 
% % theta = simulation.heading_states_lin(:,1);
% % chi = theta;
% % delta_r = simulation.heading_states_lin(:,2);
% % 
% % % wind and sail
% % % beta_V_w = simulation.sail_states(:,1);
% % epsilon = simulation.sail_states_lin(:,1);
% % 
% % % tail
% % delta_t = simulation.tail_states_lin(:,1);
% % 
% % % for testing
% % U = simulation.speed_states_lin(:,1);
% % 
% % % position data in NED
% % x__n = simulation.position_states_lin(:,1);
% % y__n = simulation.position_states_lin(:,2);
% % z__n = zeros(length(x__n),1);
% % position = [x__n'; y__n'; z__n'];
% % 
% % 
% % %% Animaate simulation data
% % animation_speed_gain = 5;  % reduce pauseing to increase animation speed
% % animation_resolution = 1; % plot every x point of the data (must be >=1)
% % 
% % if animation_on == true
% %     g = figure(1);
% %     steps = floor(length(theta)/animation_resolution);
% %     for i= 1:steps
% %         i_scaled = i*animation_resolution;
% %         plot_saildrone_2D(theta(i_scaled), chi(i_scaled), U(i_scaled), beta_V_w, U_w, gamma_c, U_c, epsilon(i_scaled), delta_t(i_scaled), delta_r(i_scaled), [x__n(i_scaled);y__n(i_scaled);0], true, true);
% %         pause(simulation_sampling_time/animation_speed_gain)
% %         if i ~= steps
% %             clf(g)
% %         end
% %     end
% % end
