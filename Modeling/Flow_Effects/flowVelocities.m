function [v__b_w, v__s_w, v__t_w, v__b_c, v__r_c]= flowVelocities(q_, v__n_w, v__n_c)

% take the wind and current velocities in NED {n} and rotate them into the
% respective frames

%% Extraction 
% extract position and attitude
eta = q_(1:6);
% p__n_nb = eta(1:3);
% THETA_nb = eta(4:6);
phi = eta(4);
theta = eta(5);
psi = eta(6);
epsilon = q_(7);
delta_t = q_(8);
delta_r = q_(9);

%% Rotation Transforms

% NED and body
R__n_b = Rzyx(phi, theta, psi);
R__b_n = transpose(R__n_b);

% body and sail
R__b_s = Rzyx(0,0,epsilon);
% R__s_b = transpose(R__b_s);

% sail and tail transforms 
R__s_t = Rzyx(0,0,delta_t);
% R__t_s = transpose(R__s_t);
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

% NED and rudder
R__r_n = R__r_b*R__b_n;

%% Transform the velocities

v__b_w = R__b_n*v__n_w;
v__s_w = R__s_n*v__n_w;
v__t_w = R__t_n*v__n_w;
v__b_c = R__b_n*v__n_c;
v__r_c = R__r_n*v__n_c;