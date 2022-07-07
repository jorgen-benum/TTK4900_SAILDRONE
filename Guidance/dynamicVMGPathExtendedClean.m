function [chi_selected, x__p_e,y__p_e, tacking, typeC, prev_tacking, tack_count, prev_WP_update, prev_WP_original, ssa_on, jib_angle] = dynamicVMGPathExtendedClean(target_WP, prev_WP, q, chi, U_rc, beta_c, U_c, beta_w, U_w, tacking_minus, tacking_plus, delta ,typeC, prev_tacking, tack_count, prev_chi_selected, prev_WP_update, prev_WP_original, ssa_on, jib_angle)

    % jib vs tack?
    % use cross track error of main path to select cicle of acceptance 

    % std dL and dR are the distance to the right and left of the path
    % to the target
%% New parameters
% prev_tacking = 1
% u_rc = 3
% % typeC  The coruse type 1 = right 0 is left
% typeC = 0    
    
%% Parameters
    dL = -30;%-30;
    dR = 30;%30;
    dLj = dL*0.6;
    dRj = dR*0.6;
    LOS_treshold = 5*pi/180; %deg
    u_rc_limit = 1.5; % m/s
    jib_off_angle = 5; % deg
    jib_margin = 2; % m offset from jib line, create the segment of jibturn start when tacking



%% testing parameteres
%     prev_WP = [-50; 0];
%     target_WP = [-250;0]
% 
%     chi = pi/4;
% 
%     % chi_d = 0.2
%     beta_c = pi/2
%     U_c = 0
%     beta_w = pi
%     U_w = 10
% 
%     % Current vector in NED
%     % v__n_c = U_c*[cos(beta_c); sin(beta_c)];
% 
%     x__n = 0;%q(1);
%     y__n = 50;%q(2);

%% Look for updates of WP
if prev_WP_original == prev_WP % Has the original changed?
    % no
    if isnan(prev_WP_update(1)) 
        % prev_WP = prev_WP do nothing
    else                            % do we have a updated value?
        prev_WP = prev_WP_update;
    end
else
    % we have recived a new Global WP 
%     prev_WP = prev_WP; % us it
    prev_WP_original = prev_WP;  % update the new original
    prev_WP_update = NaN(2,1);   % reset any updated value
end
    


%% Main part

    % current position
    x__n = q(1); 
    y__n = q(2);

    % course angle of global path
    chi_target_d = atan2(target_WP(2)-prev_WP(2), target_WP(1)-prev_WP(1));
    R__n_ptrack = [cos(chi_target_d) -sin(chi_target_d);
                   sin(chi_target_d) cos(chi_target_d)];

    % global path error
    error_p = R__n_ptrack'*([x__n; y__n] - prev_WP);
    x__p_e = sqrt((target_WP(2) - prev_WP(2))^2 + (target_WP(1) - prev_WP(1))^2) - (error_p(1));%error_p(1)
    y__p_e = error_p(2);


    % Current vector in NED
    v__n_c = U_c*[cos(beta_c); sin(beta_c)];


    % Direct course to target
    chi_to_target = atan2(target_WP(2)-y__n, target_WP(1)-x__n);

    % This function is oly concerned with the direction of travel not
    % position, hence a exra check is needed when close to the goal for
    % tacking
    
    % State if crossing is needed and the angle of it
    % The angle at which crossing is needed is at a lower angle than when it is actually the best, this to prevent very short legs 
%     [tacking, chi_tack, U_rc_tack, U_rc_along_main_course, U_along_main_course] = tackingAngleSelection(chi_target_d, beta_c, U_c, beta_w, U_w, tacking_minus, tacking_plus);
    %%testing version and transfrom the angles in the optimal way!
    [tacking, chi_tack, U_rc_tack, U_rc_along_main_course, U_along_main_course] = tackingAngleSelection2(chi_target_d, beta_c, U_c, beta_w, U_w, tacking_minus, tacking_plus);
    
%     tacking
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%% course and LOS selcetion %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % the tacking angles
    chi_tack_R = ssa(chi_tack(1));
    chi_tack_L = ssa(chi_tack(2));
    
    % rotate into the global path frame
    chi_p_R = ssa(chi_tack_R - chi_target_d);
    chi_p_L = ssa(chi_tack_L - chi_target_d);
    chi_to_target_p = ssa(chi_to_target - chi_target_d);
    
    % Find closes angle to current course 
    [M, start_course_index] = min([abs(ssa(chi_tack_R-chi)), abs(ssa(chi_tack_L-chi))]);
   
    % indicate if last course is used
    used_last_chi = 0;
 
    %% LOGIC
    
%     % When tacking, when to jib
%     if abs(ssa(chi_target_d - beta_w + pi)) < pi/4 % also check for direction vs wind
%         if (U_rc < u_rc_limit) && (prev_tacking == 1) && (tacking == 1)% to small velocity
%             if (dRj - 2) < y__p_e && y__p_e < dRj % within a region to the right
%                 if typeC == 1                   % going right
%                     ssa_on = 0;
%                     chi_selected = chi_tack_L; % turn around
%                     typeC = 0;
%                 end
%             elseif (dLj + 2) > y__p_e && y__p_e > dLj
%                 if typeC == 0
%                     ssa_on = 0;
%                     chi_selected = chi_tack_R;
%                     typeC = 1;
%                 end
% %             elseif (tacking == 1) 
% %                 if (abs(ssa(chi_target_d - beta_w + pi)) < pi/4)
% %                     ssa_on = 0;
% %                 end
%             end
%         end
%     end

    
    
    % Course direction selection 
    if ((tacking == 1) && (prev_tacking ==1))   
        if ((chi_to_target_p > (chi_p_R + LOS_treshold)) || (chi_to_target_p < (chi_p_L - LOS_treshold)))
        % for the spesial case when we are tacking and closing into the target
%             tacking_current = 0; % overide with LOS as a direct path is possible 
            if (ssa_on == 1) %&& (U_rc > u_rc_limit)
%                 if (U_rc > u_rc_limit-1) && (abs(ssa(chi_target_d - beta_w + pi)) > pi/4)
                    prev_WP_update = [x__n; y__n] % this move the whole problem into a new frame with a new desried direction to target
                    tacking = 0;
%             end
            end
        end
        if (typeC == 1) % going right
            if (y__p_e < dR) % not crossed limit yet
                chi_selected = chi_tack_R; % keep going
            else
                chi_selected = chi_tack_L; % turn around
                typeC = 0;
            end
        else % going left
            if (y__p_e > dL) % not crossed limit yet
                chi_selected = chi_tack_L; % keep going
            else
                chi_selected = chi_tack_R; % turn around
                typeC = 1;
            end
        end
    elseif ((tacking == 1) && (prev_tacking ==0)) % transition and first tack selection
        if abs(y__p_e) <= min(abs(dL),abs(dR))  % inside main track
%             if jib_vel_limit > u_rc
            if start_course_index == 1
                typeC = 1;
            else
                typeC = 0;
            end
        elseif y__p_e > dR % far to the right
            typeC = 0; % go left
        elseif y__p_e < dL % far to the left
            typeC = 1; % go right
        else
            fprintf('no valid state')
        end
        
        chi_selected = ssa(prev_chi_selected);
        used_last_chi = 1;
    else % tacking = 0
        if ssa_on == 1
            pi_p = chi_target_d;%atan2(target_WP(2)-prev_WP(2), target_WP(1)-prev_WP(1));
            % Guidance parameters
            % delta = 40;%50; %look ahead distance
            K_p = 1/delta;
            chi_selected = pi_p - atan(K_p*y__p_e);
        else
            chi_selected = chi_to_target; % course direct to target
        end
    end
    
    
    % When tacking, when to jib
    if abs(ssa(chi_target_d - beta_w + pi)) < pi/2 % also check for direction vs wind
        if (U_rc < u_rc_limit) && (prev_tacking == 1) && (tacking == 1)% to small velocity
            if (dRj - jib_margin) < y__p_e && y__p_e < dRj % within a region to the right
                if typeC == 1                   % going right
                    ssa_on = 0;
                    chi_selected = chi_tack_L; % turn around
                    typeC = 0;
                end
            elseif (dLj + jib_margin) > y__p_e && y__p_e > dLj
                if typeC == 0
                    ssa_on = 0;
                    chi_selected = chi_tack_R;
                    typeC = 1;
                end
%             elseif (tacking == 1) 
%                 if (abs(ssa(chi_target_d - beta_w + pi)) < pi/4)
%                     ssa_on = 0;
%                 end
            end
        end
    end
    
    
    % When not in tacking mode, when to jib?
    % if need to cross the wind jib
    if abs(ssa(chi - beta_w + pi)) < pi/2 % only if current direction is against the wind 
        if (U_rc < u_rc_limit) % if velocity to low
            if (tacking == 0) || (((tacking == 1) && (prev_tacking ==0))) % if not tacking or we are entering a tacking segment
                if (ssa(ssa(chi) - (beta_w + pi)) > 0) && (ssa(chi_selected - (beta_w + pi)) < 0) % cross the wind
                    if ssa(chi_selected-chi) < 0 % is connected with the previous, if going right of the wind only jib if angle to the selected course deviate more than 180 right
                        ssa_on = 0;
                    end
                elseif (ssa(ssa(chi) - (beta_w + pi)) < 0) && (ssa(chi_selected - (beta_w + pi)) > 0)
                    if ssa(chi_selected-chi) > 0
                    
                        ssa_on = 0;
                    end
                end
            end
        end
    end
    
    
    
    if (ssa_on == 0)
        % if not a valid value select one if vaild keep it until error is
        % reduced and turn is finished
        if jib_angle < -0.5  && used_last_chi == 0 
            chi1 = abs(ssa(chi_selected) - ssa(chi));
            chi2 = abs(abs(ssa(chi_selected) - ssa(chi))-2*pi);
            if chi1 <= chi2
                jib_angle = 2*pi;
            else
                jib_angle = 0;
            end
        end
        chi_selected = chi_selected - jib_angle;
    end 
        
       %         fprintf('ttt')
%         jib_angle 
        
        
        %             jib_angle
%             chi1 = abs(ssa(chi_selected - chi));
%             chi2 = abs(ssa(chi_selected - 2*pi -chi));
%     end
      %                 fprintf('_______________________________')  
%             if chi1 < chi2
%                 chi_selected = ssa(chi_selected);
%             end
%           if ssa(chi_selected) - ssa(chi) > pi
%               chi_selected = chi_selected
%           else
%               chi_selected = ssa(chi_selected) - 2*pi;
%           end
          % in 2 first cases we already have the largest angle
%           if (ssa((chi)) > pi/2) && (ssa((chi_selected)) < -pi/2)
%               % edit nothing
%           elseif (ssa((chi)) < -pi/2) && (ssa((chi_selected)) > pi/2)
%               % edit nothing
%           else
%               chi_selected = ssa(chi_selected) - 2*pi;
%           end
            % else keep value 
% %         fff = chi_selected - 2*pi;
% %         chi_selected = chi_selected;% - 2*pi;
%         chi_selected = ;
    
    
    %% jib deactivation when error is reduced
    if abs(ssa(chi_selected - chi)) < jib_off_angle*pi/180
        ssa_on = 1;
        jib_angle = -1;
%         fprintf('_______________________________')
    end
    
    prev_tacking = tacking;
    prev_chi_selected = chi_selected;
%     prev_ssa_on = ssa_on;
    
    

    
end



    
