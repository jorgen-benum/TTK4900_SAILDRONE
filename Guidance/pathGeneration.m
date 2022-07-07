%% inputs
% chi_tack = [0.5; -0.9];     % tacking course
% chi = -0.5;                  % current course
% 
% chi_d_target = 0.3;          % along track course to target
% x__p_e = 250;              % along track lenght from current position
% 
% y__p_e = 10; % cross track error with respect to track to target
% 
% 
% %% Parameters
% dR = 20; % max track width right
% dL = -20;

function tack_points_b = pathGeneration(chi_tack, chi_d_target, chi , x__p_e, y__p_e, dL, dR)

%% Calulations 
chi_tack_R = chi_tack(1);
chi_tack_L = chi_tack(2);

% rotate into the track frame
chi_p_R = chi_tack_R - chi_d_target;
chi_p_L = chi_tack_L - chi_d_target;
chi_p = chi - chi_d_target;

chi_p_R = ssa(chi_tack_R - chi_d_target);
chi_p_L = ssa(chi_tack_L - chi_d_target);
chi_p = ssa(chi - chi_d_target);

% % caluate the progression lengths
% % l_S = dS/(tan(chi_p_S))
% l_R = (dR-dL)/(tan(chi_p_R))
% l_L = (-dR+dL)/(tan(chi_p_L))


% select start course
[M, start_course_index] = min([abs(ssa(chi_p_R-chi_p)), abs(ssa(chi_p_L-chi_p))]);

% tack_points = [];
nr_tack = [0; 0];
if start_course_index == 1
    chi_p_S = chi_p_R;
    dS = dR - y__p_e;
    typeS = 1; % start type == 1 if right
    nr_tack(1) = nr_tack(1) +1;
    tack_points = [0; dR];
else
    chi_p_S = chi_p_L
    dS = dL - y__p_e;
    typeS = 0; % start type == 1 if right
    nr_tack(2) = nr_tack(2) +1;
    tack_points = [0; dL];
end

% caluate the progression lengths
l_S = dS/(tan(chi_p_S));
l_R = (dR-dL)/(tan(chi_p_R));
l_L = (-dR+dL)/(tan(chi_p_L));

% if l_S > x__p_e
%     tack_points = []; 
% end
    


% Calculate main segment (when start tack is over)

rest1 = x__p_e - l_S;

% full_tack_nr = floor(rest1/(l_R+l_L));
% rest2 = rest1 - (l_R+l_S)*full_tack_nr; % rest length after all full tacks

% nr_tack = nr_tack + [full_tack_nr; full_tack_nr]


l_LR = (-dR)/(tan(chi_p_L)); % cross in from right
l_RL = (-dL)/(tan(chi_p_R)); % crossing from left


if typeS == 0
    l_next = l_R;
    l_nexthalf = l_LR; % half length after next tack
else
    l_next = l_L;
    l_nexthalf = l_RL;
end
% 
% if rest2 > l_next
%     rest2 = rest2-l_next
% elseif rest2 > l_next/2
%     %
% end
    

% l_next + l_nexthalf
% rest1
if rest1 < l_next + l_nexthalf
    active = 0;
else
    active = 1;
end

count = 0;
length_sum = 0;
switcher = typeS;
% while length_sum < rest1
while active == 1
    if switcher == 0 % must be oppisite of start
        length_sum = length_sum + l_R;
        nr_tack(1) = nr_tack(1) +1;
        switcher = 1;
        tack_points = [tack_points, [length_sum; dR]];
    else
        length_sum = length_sum + l_L;
        nr_tack(2) = nr_tack(2) +1;
        switcher = 0;
        tack_points = [tack_points, [length_sum; dL]];
    end
    
    if switcher == 0
        if (length_sum + l_R + l_LR) > rest1
            active = 0;
        end
    else
        if (length_sum + l_L + l_RL) > rest1
            active = 0;
        end
    end  
    
    count = count+1;
    if count > 1000 % stop iterating as no solution is found
        active = 0;
    end
%     if length_sum < rest1

%     if (length_sum + l_R + l_L) > rest1
%             active = 0;
%     end
    
end

angleA = pi - abs(chi_p_R) - abs(chi_p_L);

% l_LR = (-dR)/(tan(chi_p_L)) % cross in from right
% l_RL = (-dL)/(tan(chi_p_R)) % crossing from left

if switcher==1 % then the last added WP was to the right R
    if rest1-length_sum > l_LR
        l_E = rest1-length_sum-l_LR; 
        
        c = abs(l_E*sin(chi_p_R)/sin(angleA));

        dist_L = -abs(c*sin(chi_p_L));
        dist_T = sqrt(c^2-dist_L^2);
        tack_points = [tack_points, [rest1-(l_E-dist_T); dist_L]];
    end
else
    if rest1-length_sum > l_RL
        l_E = rest1-length_sum-l_RL;
        c = abs(l_E*sin(chi_p_L)/sin(angleA));
        dist_L = abs(c*sin(chi_p_R));
        dist_T = sqrt(c^2-dist_L^2);
        tack_points = [tack_points, [rest1-(l_E-dist_T); dist_L]];
    end
end

tack_points = [tack_points, [rest1; 0]]; % final WP using LOS

tack_points = tack_points +[l_S; 0]*ones(1, length(tack_points)); % move all to parrallel to current positions

if rest1 < 0
    tack_points = [];% remove the WPs as it is past the target and create a triangle
    
    if typeS == 1
        chi1 = chi_p_R;
        chi2 = chi_p_L;
    else
        chi1 = chi_p_L;
        chi2 = chi_p_R;
    end
    
    if abs(atan2(-y__p_e,x__p_e))> abs(chi1)
        tack_points = [x__p_e; 0];
    else
        Am = [cos(chi1) cos(chi2);
              sin(chi1) sin(chi2)];
        % x0y0 = 0, y__p_e
        % x1y1 = x__p_e, 0
        Bm = [x__p_e; 0-y__p_e];
        X = linsolve(Am,Bm);
        tack_points = [tack_points, X(1)*[cos(chi1);(sin(chi1))] + [0; y__p_e]];
        tack_points = [tack_points, X(2)*[cos(chi2);sin(chi2)]+tack_points(:,1)];
    end    
end

% transfrom and move
% chi_d_target = -0.2
R__n_p = [cos(chi_d_target), -sin(chi_d_target);
          sin(chi_d_target), cos(chi_d_target)];

tack_points_rot = R__n_p*tack_points;

% transfrom shift to NED frame axis
tack_points_shift = R__n_p*([0; -y__p_e]*ones(1,size(tack_points_rot,2)));

% tack_points_b = tack_points_rot + [0; -y__p_e]*ones(1,length(tack_points_rot));
tack_points_b = tack_points_rot + tack_points_shift;
tack_points_b = [[0; 0], tack_points_b]; % add start WP to create line to "first" WP
        
end

% if rest is
% rest1-length_sum;
% 
% figure(99)
% plot([y__p_e, tack_points(2,:)],[0, tack_points(1,:)], '-b')
% plot([tack_points_rot(2,:)],[tack_points_rot(1,:)], '-b')
% plot([tack_points_b(2,:)],[tack_points_b(1,:)], '-b')
% hold on
% plot(y__p_e, (0), '-ob')
% plot(0, x__p_e, '-or')
% plot([0,0], [0,x__p_e], '--')
% % xlim([dL-5, dR+5])
% % ylim([-5, x__p_e+5])
% grid on
% hold off
