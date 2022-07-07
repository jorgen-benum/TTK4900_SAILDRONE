clear all
close all
clc
%% Switches
animation_on = true;


%% parameters 
theta = pi/8;       % heading
epsilon = pi/6;     % sail angle relative to heading
delta_t = pi/7;     % tail angle relative to sail
delta_r = pi/4;     % rudder angle relative to heading
beta_V_w = pi/4;      % wind direction
chi = pi/8;         % course
gamma_c = 3*pi/4;   % current direction
U = 5;
U_c = 3;
U_w = 7;

% initial conditions
theta0 = pi/8;       % heading
epsilon0 = pi/6;     % sail angle relative to heading


%% Simulation in simulink

simulation_sampling_time = 0.1;
simulation_time = 15; % length of simulation in [s]
simulation = sim('testmodel',simulation_time); % run simulink

%% extract data 
% heading 
theta = simulation.heading_states(:,1);
chi = theta;
delta_r = simulation.heading_states(:,2);

% wind and sail
% beta_V_w = simulation.sail_states(:,1);
epsilon = simulation.sail_states(:,1);

% tail
delta_t = simulation.tail_states(:,1);

% for testing
U = simulation.speed_states(:,1);

% position data in NED
x__n = simulation.position_states(:,1)
y__n = simulation.position_states(:,2)
z__n = zeros(length(x__n),1)
position = [x__n'; y__n'; z__n']
% 


%% Animaate simulation data
animation_speed_gain = 5;  % reduce pauseing to increase animation speed
animation_resolution = 1; % plot every x point of the data (must be >=1)

if animation_on == true
    g = figure(1);
    steps = floor(length(theta)/animation_resolution)
    for i= 1:steps
        i_scaled = i*animation_resolution;
        plot_saildrone_2D(theta(i_scaled), chi(i_scaled), U(i_scaled), beta_V_w, U_w, gamma_c, U_c, epsilon(i_scaled), delta_t(i_scaled), delta_r(i_scaled), [x__n(i_scaled);y__n(i_scaled);0], true, true)
        pause(simulation_sampling_time/animation_speed_gain)
        if i ~= steps
            clf(g)
        end
    end
end
%% Plot path
plot_resolution = 20

figure(2)
steps = floor(length(theta)/plot_resolution)
pathSpeedPlotter(position, U) % plot path
hold on
for i= 1:steps
    i_scaled = i*plot_resolution;
    plot_saildrone_2D(theta(i_scaled), chi(i_scaled), U(i_scaled), beta_V_w, U_w, gamma_c, U_c, epsilon(i_scaled), delta_t(i_scaled), delta_r(i_scaled), [x__n(i_scaled);y__n(i_scaled);0], false, false) % plot state along path

end
% plot(position(2),position(1)) % plot path

fprintf('simulation end \n')
