% function [tacking_points_n, circ_acceptance_R, circ_acceptance_L] = pathCreator(target_WP, prev_WP, q, beta_c, U_c, beta_w, U_w, tacking_minus, tacking_plus, calulate_local_WPS, tacking_points_n, circ_acceptance_R, circ_acceptance_L)
% function [tacking_points_n, circ_acceptance_R, circ_acceptance_L] = pathCreator2(target_WP, prev_WP, q, chi, beta_c, U_c, beta_w, U_w, tacking_minus, tacking_plus, calulate_local_WPS)
function [tacking_points_n, circ_acceptance_R, circ_acceptance_L] = pathCreator2(target_WP, prev_WP, q, chi, beta_c, U_c, beta_w, U_w, tacking_minus, tacking_plus)

    % jib vs tack?
    % use cross track error of main path to select cicle of acceptance 

    % std dL and dR are the distance to the right and left of the path
    % to the target
%% Parameters
    dL = -45;%-30;
    dR = 45; %30;
    storeageWP = 100;


%% testing parameteres
%     prev_WP = [-50; 0];
%     target_WP = [250;0]
% 
%     chi = pi/4;
% 
%     % chi_d = 0.2
%     beta_c = pi/2
%     U_c = 0.8
%     beta_w = 1*pi
%     U_w = 10
% 
%     % Current vector in NED
%     % v__n_c = U_c*[cos(beta_c); sin(beta_c)];
% 
%     x__n = 0;%q(1);
%     y__n = 0;%q(2);

%% Main part

    x__n = q(1);
    y__n = q(2);


    % direct desired course to target WP
    % chi_target_d = atan2(target_WP(2)-y__n, target_WP(1)-x__n);
    chi_target_d = atan2(target_WP(2)-prev_WP(2), target_WP(1)-prev_WP(1));
    R__n_ptrack = [cos(chi_target_d) -sin(chi_target_d);
                   sin(chi_target_d) cos(chi_target_d)];

    error_p = R__n_ptrack'*([x__n; y__n] - prev_WP);
    x__p_e = sqrt((target_WP(2) - prev_WP(2))^2 + (target_WP(1) - prev_WP(1))^2) - (error_p(1));%error_p(1)
    y__p_e = error_p(2);


    % Current vector in NED
    v__n_c = U_c*[cos(beta_c); sin(beta_c)];


    [tacking, chi_tack, U_rc_tack, U_rc_along_main_course, U_along_main_course] = tackingAngleSelection(chi_target_d, beta_c, U_c, beta_w, U_w, tacking_minus, tacking_plus);


    
    tacking_points_n = NaN(2,storeageWP);
    if tacking == 1
        tacking_points_b = pathGeneration(chi_tack, chi_target_d, chi , x__p_e, y__p_e, dL, dR);
        tacking_points_n_ = tacking_points_b + [x__n; y__n]*ones(1,length(tacking_points_b));
%         tacking_points_n = [tacking_points_n_, NaN(2,storeageWP-length(tacking_points_n_))]
        tacking_points_n(:,1:length(tacking_points_n_)) = tacking_points_n_
        circ_acceptance_R = 5;
        circ_acceptance_L = 5;
    else
        tacking_points_n(:,1:2) = [prev_WP, target_WP]
        circ_acceptance_R = 5;
        circ_acceptance_L = 5;
    end
    
%     size(tacking_points_n)
%     tacking_points_n = NaN(2,storeageWP)
    
    % update memory 
%     prev_tacking_points_n = tacking_points_n;
%     prev_circ_acceptance_R = circ_acceptance_R;
%     prev_circ_acceptance_L = circ_acceptance_L;

% transform into along path coordinates
% generate WPS

% transfrom back

% give list to new path manager


% least error between curret course and desired course or other if first is not feasible ? or jib

end

% figure(99)
% plot(tacking_points_n(2,:),tacking_points_n(1,:), '-b')
% % plot([tack_points_rot(2,:)],[tack_points_rot(1,:)], '-b')
% % plot([tack_points_b(2,:)],[tack_points_b(1,:)], '-b')
% hold on
% % plot(y__p_e, (0), '-ob')
% % plot(0, x__p_e, '-or')
% % plot([0,0], [0,x__p_e], '--')
% % xlim([dL-5, dR+5])
% % ylim([-5, x__p_e+5])
% grid on
% hold off


    
