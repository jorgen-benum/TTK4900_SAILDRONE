clear all
close all
clc

vessel_l = 7
vessel_w = 1
vessel_h = 1

start_v = pi/5
end_v = 14*pi/16
f = start_v:0.2:end_v

% angles = [14*pi/16 12*pi/16 12*pi/16 8*pi/16 6*pi/16 4*pi/16 0 -4*pi/16 -6*pi/16 -8*pi/16 -12*pi/16 -12*pi/16 -14*pi/16]
angles = [flip(f) 0 -f]
angles = [0 f]
vessel_left_bot = [[(vessel_l/2)*cos(angles) (vessel_l/2)*cos(end_v) -1 1]; [(vessel_w/2)*sin(angles) 0 0 0 ]; [0*sin(angles) 0 -vessel_h/2 -vessel_h/2]]
vessel_right_bot = [[(vessel_l/2)*cos(angles) (vessel_l/2)*cos(end_v) -1 1]; [-(vessel_w/2)*sin(angles) 0 0 0 ]; [0*sin(angles) 0 -vessel_h/2 -vessel_h/2]]
vessel_right_top = [[(vessel_l/2)*cos(angles) (vessel_l/2)*cos(end_v) -1 1]; [-(vessel_w/2)*sin(angles) 0 0 0 ]; [0*sin(angles) 0 vessel_h/2 vessel_h/2]]
vessel_left_top = [[(vessel_l/2)*cos(angles) (vessel_l/2)*cos(end_v) -1 1]; [(vessel_w/2)*sin(angles) 0 0 0 ]; [0*sin(angles) 0 vessel_h/2 vessel_h/2]]

% y = [1 5 6 3];
% x = [1 1 3 2]
% area(x,y)

x2 = [5; 5; 8];
y2 = [0; 2; 0];
z2 = [0; 0; 0];
nr_points = length(x2)

figure(1)
patch(x2,y2,'green')
hold on

R = Rzyx(0,0,pi/4)

x3 = zeros(nr_points,1)
y3 = zeros(nr_points,1)
z3 = zeros(nr_points,1)
for point = 1:nr_points
    r_1 = R*[x2(point); y2(point); z2(point)]
    x3(point) = r_1(1)
    y3(point) = r_1(2)
    z3(point) = r_1(3)
    
end
patch(x3,y3,'r')
hold off

vessel_right_ = R*vessel_right_bot


figure(2)
patch(vessel_right_(1,:)',vessel_right_(2,:)','green')
xlim([-vessel_l/2-2, vessel_l/2+2])
ylim([-vessel_l/2-2, vessel_l/2+2])

figure(3)
patch(vessel_left_bot(1,:)',vessel_left_bot(2,:)',vessel_left_bot(3,:)','green')
patch(vessel_right_bot(1,:)',vessel_right_bot(2,:)',vessel_right_bot(3,:)','green')
patch(vessel_right_top(1,:)',vessel_right_top(2,:)',vessel_right_top(3,:)','green')
patch(vessel_left_top(1,:)',vessel_left_top(2,:)',vessel_left_top(3,:)','green')
xlim([-vessel_l/2-2, vessel_l/2+2])
ylim([-vessel_l/2-2, vessel_l/2+2])
zlim([-vessel_l/2-2, vessel_l/2+2])
view(3) 
