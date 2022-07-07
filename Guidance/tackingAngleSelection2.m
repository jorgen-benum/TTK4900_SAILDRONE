% function y = fcn(target_WP, q, beta_c, U_c, beta_w, U_w)
% 
% x__n = q(1);
% y__n = q(2);
% 
% % direct desired course to target WP
% chi_d = atan2(target_WP(2)-y__n, target_WP(1)-x__n);
% addpath(genpath('C:\Users\jorge\local_repo\TTK4900_2\lookupTables'))
% 
% tack_mi = load('C:\Users\jorge\local_repo\TTK4900_2\lookupTables\VMG\tack_minus.mat')
% tacking_minus = tack_mi.tacklist_minus
% 
% tack_pu = load('C:\Users\jorge\local_repo\TTK4900_2\lookupTables\VMG\tack_plis.mat')
% tacking_plus = tack_pu.tacklist_plus

function [tacking, chi_tack, U_rc_tack, U_rc_along_main_course, U_along_main_course] = tackingAngleSelection(chi_d, beta_c, U_c, beta_w, U_w, tacking_minus, tacking_plus)


% chi_d = 0.2
% beta_c = pi/2
% U_c = 0.40
% beta_w = pi/1
% U_w = 8

% Current vector in NED
% v__n_c = U_c*[cos(beta_c); sin(beta_c)];

% Tacking angles setup
tack_limit_uw = 30; % [deg] up windlimit
tack_limit_dw = 170; % [deg] down windlimit

tack_angle_uw = 44; %44;
tack_angle_dw = 163; %163; % 


%% transfrom vectors into VMG frame
beta_w_bar = ssa(beta_w - pi);
chi_bar = ssa(chi_d - beta_w_bar);
beta_c_bar = ssa(beta_c - beta_w_bar);

v_div_u = tan(chi_bar);


a = 1/v_div_u;

% limit value
if a > 1000
    a = 1000;
elseif a < -1000
    a = -1000;
end

% the point where the U is defined is at the start of the current vector
% relative to the center of the VMG, hence the minus
p = -U_c*[cos(beta_c_bar); sin(beta_c_bar)];

b = p(1)-a*p(2);
% if v_div_u = 0

v_vel = 0;

% See if tacking is needed and calulate tacking angles
if abs(chi_bar) <= pi/2 
    % the course is againts the wind
    
    % Caulate a velocity estimate Umax along VMG north axis
    if U_w <= 4
       weight = U_w/4;
       Umax = 0*(1-weight) + tacking_plus(2,1)*weight;
    elseif U_w <= 8
       weight = U_w/8;
       Umax = tacking_plus(2,1)*(1-weight) + tacking_plus(2,2)*weight; 
    elseif U_w <= 12
       weight = U_w/12;
       Umax = tacking_plus(2,2)*(1-weight) + tacking_plus(2,3)*weight;
    elseif U_w <= 16
       weight = U_w/16;
       Umax = tacking_plus(2,3)*(1-weight) + tacking_plus(2,4)*weight;
    else
        fprintf('no data');
        Umax = 0;
    end
    
    % find the relative course angle
    if U_c == 0
        chi_r_bar_est = chi_bar;
    else
        v_vel = (Umax-b)/a;
        chi_r_bar_est = atan2(v_vel,Umax);
    end
    
    % tacking parameters
    if abs(chi_r_bar_est) <= tack_limit_uw*pi/180;
        tacking = 1;
        tacking_angle1 = tack_angle_uw*pi/180;
        tacking_angle2 = -tack_angle_uw*pi/180;
    else
        tacking = 0;
        tacking_angle1 = 0;
        tacking_angle2 = 0;
    end
    
else
    % the course is with the wind
    
    % Caulate a velocity estimate Umax along VMG north axis
    if U_w <= 4
       weight = U_w/4;
       Umax = 0*(1-weight) + tacking_minus(2,1)*weight;
    elseif U_w <= 8
       weight = U_w/8;
       Umax = tacking_minus(2,1)*(1-weight) + tacking_minus(2,2)*weight; 
    elseif U_w <= 12
       weight = U_w/12;
       Umax = tacking_minus(2,2)*(1-weight) + tacking_minus(2,3)*weight;
    elseif U_w <= 16
       weight = U_w/16;
       Umax = tacking_minus(2,3)*(1-weight) + tacking_minus(2,4)*weight;
    else
        fprintf('no data');
        Umax = 0;
    end
    
    % find the relative course angle
    if U_c == 0
        chi_r_bar_est = chi_bar;
    else
        v_vel = (-Umax-b)/a;
        chi_r_bar_est = atan2(v_vel,-Umax);
    end
    Umax = - Umax; % quick fix of sign
    
    % tacking parameters
    if abs(chi_r_bar_est) >= tack_limit_dw*pi/180
        tacking = 1;
        % changing sign to keep left and right
        tacking_angle1 = -tack_angle_dw*pi/180;%tack_angle_dw*pi/180;
        tacking_angle2 = tack_angle_dw*pi/180;%-tack_angle_dw*pi/180;
    else
        tacking = 0;
        tacking_angle1 = 0;
        tacking_angle2 = 0;
    end
end

% calulate tacking course 
if tacking ==1
    % course change due to current (of resulting course not each course)
    chi_c_bar = chi_bar - chi_r_bar_est;

    U_esti_VMGframe = [Umax; v_vel] - p;
    U_along_main_course = norm(U_esti_VMGframe);
    U_rc_along_main_course = sqrt(Umax^2 + v_vel^2);
    U_rc_tack = [abs(Umax/cos(tacking_angle1));
                abs(Umax/cos(tacking_angle2))];
            
    % tacking course in VMG frame
    V_c_vector = -p;
    vector1 = U_rc_tack(1)*[cos(tacking_angle1); sin(tacking_angle1)] + V_c_vector;
    vector2 = U_rc_tack(2)*[cos(tacking_angle2); sin(tacking_angle2)] + V_c_vector;
    
    chi_tack1_bar = atan2(vector1(2),vector1(1));
    chi_tack2_bar = atan2(vector2(2),vector1(1));
    
    % angle shift only!
%     chi_tack1_bar = tacking_angle1 + chi_c_bar;
%     chi_tack2_bar = tacking_angle2 + chi_c_bar;

    % tacking in NED
    chi_tack = [ssa(chi_tack1_bar + beta_w_bar);
                ssa(chi_tack2_bar + beta_w_bar)];            
         
else
    chi_tack = [0; 0];
    U_rc_tack = [0; 0];
    U_rc_along_main_course = 0;
    U_along_main_course = 0;
end

end
% tacking balance? only use limits?


% when U_c = 0, then chi_r_bar = chi_bar
    
