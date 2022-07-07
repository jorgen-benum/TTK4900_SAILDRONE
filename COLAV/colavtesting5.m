function [next_GWP, prev_GWP, chi_commanded, prev_path_state, blocked_count] = colavtesting5(x__n, y__n, theta, next_GWP, prev_GWP, map, x__p_e_prev, y__p_e_prev, gird_size, prev_path_state, blocked_count)
% clear all

%% setup paramters
% gird_size = 1400;
d_max = 30;
d_safe = 20;
d_USV = 7/2;
d_expand = d_max + d_safe + d_USV;

nr_blocked = 3; % number of times object need to block path before COLAV is used, remove the unsertanty when object in edge of detection range

nr_obsicals = 4; %%%%%%%%% max number of obsticals
% angle_margin = 10*pi/180;

% sensor setting
detection_range = 250; % [m]
resulution = 2; %6 [deg]

%% map
% map = binaryOccupancyMap(gird_size, gird_size, 1); %size and resolution in cells per meter
% object_locations = [650, 800; 800, 800];
% setOccupancy(map, object_locations, ones(2,1));
% inflate(map, 10);

% object_locations_NED = [300, 50; 
%                         10, 50];
% 
% object_locations = [flip(object_locations_NED(:,1)'); flip(object_locations_NED(:,2)')];
% 
% map = binaryOccupancyMap(gird_size, gird_size, 1); %size and resolution in cells per meter
% object_locations = object_locations + [gird_size/2, gird_size/2; gird_size/2, gird_size/2];
% % object_locations = [10, 200; 50, 50] + [gird_size/2, gird_size/2; gird_size/2, gird_size/2];
% setOccupancy(map, object_locations, ones(2,1));
% inflate(map, 10);

%% testparameters
% x__n = 000;
% y__n = 000;
% theta = -0*pi/4;
% next_GWP = [120; -20];
% prev_GWP = [0; 0];
% y__p_e_prev = -40;
% x__p_e_prev = 100;


%% bsic def
R__n_b = [cos(theta), -sin(theta);
          sin(theta), cos(theta)];

deg2rad = pi/180;

chi_path = atan2(next_GWP(2)-prev_GWP(2), next_GWP(1)-prev_GWP(1));
chi_to_target = atan2(next_GWP(2)-y__n, next_GWP(1)-x__n)

vehiclePose = [y__n+gird_size/2, x__n+gird_size/2, pi/2-(theta)];

% number_of_rays = round(180/resulution) + 1;
% number_of_rays = round(270/resulution) + 1;% increased view
number_of_rays = round(360/resulution) + 1;% increased view
angle_array = zeros(1,number_of_rays);

for i = 1:number_of_rays
%     angle_array(1,i) = -pi/2 + resulution*deg2rad*(i-1);
%     angle_array(1,i) = -pi*3/4 + resulution*deg2rad*(i-1); % increased view
    angle_array(1,i) = -pi + resulution*deg2rad*(i-1); % increased view
end

%% Ray iditification settings

number_of_rays = size(angle_array,2);


%% Get sensor distance data for each ray

% get meansurements [x, y]
intsectionPts = rayIntersection(map,vehiclePose,angle_array(1,1:number_of_rays),detection_range);
% transform to body
body_offset = ([vehiclePose(1).*ones(number_of_rays,1), vehiclePose(2).*ones(number_of_rays,1)]);
intsectionPts_body = intsectionPts - body_offset;

magnitudes_body = sqrt(intsectionPts_body(:,1).^2 + intsectionPts_body(:,2).^2);

org_index = 1:number_of_rays;
% flip to fit with NED
polarIntSec = [magnitudes_body, flip(angle_array)', org_index', intsectionPts_body];

%% main part

blocked_angles = nan(number_of_rays,5);
not_blocked_angles = nan(number_of_rays,5);
index_blocked = 1;
index_notblocked = 1;
for i = 1:number_of_rays
    if isnan(polarIntSec(i,1)) == false
        blocked_angles(index_blocked,:) = polarIntSec(i,:);
        index_blocked = index_blocked + 1;
    else
        not_blocked_angles(index_notblocked,:) = polarIntSec(i,:);
        index_notblocked = index_notblocked +1;
    end
end


object_count = 1;

objects = nan(nr_obsicals,2); 
objects(1,1) = blocked_angles(1,3);
object_number = 1;
prev_index_blocked = blocked_angles(1,3);

collision_cones = nan(nr_obsicals, 4);

chi_path_new = nan;
mag = nan;

alterntive_path = 0;
if isnan(blocked_angles(1,1)) == false
%         object = nan(10,1);

        for i = 2:size(blocked_angles, 1)
            if isnan(blocked_angles(i,3)) == false
                if blocked_angles(i,3) == prev_index_blocked + 1
                    objects(object_number,2) = blocked_angles(i,3);
                    prev_index_blocked = blocked_angles(i,3);
                else
                    object_number = object_number + 1;
                    objects(object_number,1) = blocked_angles(i,3);
                    prev_index_blocked = blocked_angles(i,3);
                end
            end
        end
        
      
        
%     nr_obsicals = size(objects,1)


    for obstical_nr = 1:nr_obsicals
        if isnan(objects(obstical_nr,1)) == false
            R_angle_index = objects(obstical_nr,1);
            R_angle_comp = tan(d_expand/polarIntSec(R_angle_index,1));
            R_mag = sqrt(d_expand^2 + polarIntSec(R_angle_index,1)^2);
            if isnan(objects(obstical_nr,2))
                L_angle_index = R_angle_index;
            else
                L_angle_index = objects(obstical_nr,2);
            end
            L_angle_comp = tan(d_expand/polarIntSec(L_angle_index,1));
            L_mag = sqrt(d_expand^2 + polarIntSec(L_angle_index,1)^2);

            ccR = polarIntSec(R_angle_index,2)+R_angle_comp;
            ccL = polarIntSec(L_angle_index,2)-L_angle_comp;
            collision_cones(obstical_nr,:) = [ccR, ccL, R_mag, L_mag];
        end
    end
    

% 
    for obstical_nr = 1:nr_obsicals
        ccR = ssa(collision_cones(obstical_nr,1) + theta);
        ccL = ssa(collision_cones(obstical_nr,2) + theta);
        R_mag = collision_cones(obstical_nr,3);
        L_mag = collision_cones(obstical_nr,4);

        if ccL > ccR
%             if ccL < chi_path || chi_path < ccR % direction of path
            if ccL < chi_to_target || chi_to_target < ccR % direction to target
                alterntive_path = 1;
                if prev_path_state == -1
                    index_min = 0;
                elseif prev_path_state == 1
                    index_min = 1;
                else
                    [mm, index_min] = min([abs(ssa(ccR-chi_path)), abs(ssa(ccL-chi_path))]);
                end
                if index_min == 1
                    chi_path_new = ccR;
                    mag = R_mag;
                    prev_path_state = 1;
                else
                    chi_path_new = ccL;
                    mag = L_mag;
                    prev_path_state = -1;
                end
%                 next_GWP = (mag*[cos(chi_path_new); sin(chi_path_new)]);
%                 next_GWP = next_GWP + [x__n; y__n];
            end
        end
%         if ccL < chi_path && chi_path < ccR
        if ccL < chi_to_target && chi_to_target < ccR
            alterntive_path = 1;
            if prev_path_state == -1
                index_min = 0; % continue on left side
            elseif prev_path_state == 1
                index_min = 1; % continue on right side
            else
                [mm, index_min] = min([abs(ssa(ccR-chi_path)), abs(ssa(ccL-chi_path))]);
            end
            if index_min == 1
                chi_path_new = ccR;
                mag = R_mag;
                prev_path_state = 1;
            else
                chi_path_new = ccL;
                mag = L_mag;
                prev_path_state = -1;
            end
%             next_GWP = (mag*[cos(chi_path_new); sin(chi_path_new)]);
%             next_GWP = next_GWP + [x__n; y__n];
        end

    end
end

if alterntive_path == 0
    if blocked_count == nr_blocked % the path has been blocked before
        prev_GWP = [x__n; y__n];
    end
    prev_path_state = 0; % the path to target is not blocked
    blocked_count = 0;
else
    if blocked_count >= nr_blocked
        blocked_count = nr_blocked;
    else
        blocked_count = blocked_count + 1;
    end
end

% compensate by angle of prvoius path
chi_comp = atan(y__p_e_prev/x__p_e_prev); % Like LOS
% chi_comp = atan(y__p_e_prev/100);

if (alterntive_path == 1) && (blocked_count >= nr_blocked)
%     chi_commanded = chi_path_new - chi_comp;
    chi_commanded = chi_path_new + chi_comp;
    next_GWP = (mag*[cos(chi_path_new); sin(chi_path_new)]);
    next_GWP = next_GWP + [x__n; y__n];
    prev_GWP = next_GWP-100*[cos(chi_commanded); sin(chi_commanded)];
else
    chi_commanded = chi_path;
end



%% ploting
% prev_WP_occ = prev_GWP' + [gird_size/2, gird_size/2]; 
% next_WP_occ = next_GWP' + [gird_size/2, gird_size/2];
% % B = polarIntSec(~isnan(polarIntSec(:,1)))
% 
% 
% % %% Plot intersection with objects
% 
% figure(1)
% show(map);
% grid on
% hold on
% 
% % intersection
% intsectionPts_plot = intsectionPts_body + vehiclePose(1:2);
% 
% % figure(3)
% % hold on
% plot(intsectionPts_plot(:,1),intsectionPts_plot(:,2),'*r') % Intersection points
% plot(vehiclePose(1),vehiclePose(2),'ob') % Vehicle pose
% plot(next_WP_occ(2), next_WP_occ(1), 'og')
% plot(prev_WP_occ(2), prev_WP_occ(1), 'or')
% for i = 1:number_of_rays
%     if (ceil(number_of_rays/2) <= i)
%         plot([vehiclePose(1),intsectionPts_plot(i,1)],...
%         [vehiclePose(2),intsectionPts_plot(i,2)],'-b') % Plot intersecting rays
%     else
%         plot([vehiclePose(1),intsectionPts_plot(i,1)],...
%         [vehiclePose(2),intsectionPts_plot(i,2)],'-g') % Plot intersecting rays
%     end
%         
% end

