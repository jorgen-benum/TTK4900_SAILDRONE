function plot_saildrone_2D(theta, chi, U, beta_V_w, U_w, gamma_c, U_c, epsilon, delta_t, delta_r, position, fixed_window_on, velocities_on)

% clear all
% close all
% clc

%% Foil parameters
NACA0015 =   [1.0000     0.00158 0;
    0.9500     0.01008 0;
    0.9000     0.01810 0;
    0.8000     0.03279 0;
    0.7000     0.04580 0;
    0.6000     0.05704 0;
    0.5000     0.06617 0;
    0.4000     0.07254 0;
    0.3000     0.07502 0;
    0.2500     0.07427 0;
    0.2000     0.07172 0;
    0.1500     0.06682 0;
    0.1000     0.05853 0;
    0.0750     0.05250 0;
    0.0500     0.04443 0;
    0.0250     0.03268 0;
    0.0125     0.02367 0;
    0.0000     0.00000 0;
    0.0125     -0.02367 0;
    0.0250     -0.03268 0;
    0.0500     -0.04443 0;
    0.0750     -0.05250 0;
    0.1000     -0.05853 0;
    0.1500     -0.06682 0;
    0.2000     -0.07172 0;
    0.2500     -0.07427 0;
    0.3000     -0.07502 0;
    0.4000     -0.07254 0;
    0.5000     -0.06617 0;
    0.6000     -0.05704 0;
    0.7000     -0.04580 0;
    0.8000     -0.03279 0;
    0.9000     -0.01810 0;
    0.9500     -0.01008 0;
    1.0000     -0.00158 0]';


NACA0009 = ...
    [  1.00000  0.0;
      0.99572  0.00057;
      0.98296  0.00218;
      0.96194  0.00463;
      0.93301  0.00770;
      0.89668  0.01127;
      0.85355  0.01522;
      0.80438  0.01945;
      0.75000  0.02384;
      0.69134  0.02823;
      0.62941  0.03247;
      0.56526  0.03638;
      0.50000  0.03978;
      0.43474  0.04248;
      0.37059  0.04431;
      0.33928  0.04484;
      0.30866  0.04509;
      0.27886  0.04504;
      0.25000  0.04466;
      0.22221  0.04397;
      0.19562  0.04295;
      0.17033  0.04161;
      0.14645  0.03994;
      0.12408  0.03795;
      0.10332  0.03564;
      0.08427  0.03305;
      0.06699  0.03023;
      0.05156  0.02720;
      0.03806  0.02395;
      0.02653  0.02039;
      0.01704  0.01646;
      0.00961  0.01214;
      0.00428  0.00767;
      0.00107  0.00349;
      0.0      0.0;
      0.00107 -0.00349;
      0.00428 -0.00767;
      0.00961 -0.01214;
      0.01704 -0.01646;
      0.02653 -0.02039;
      0.03806 -0.02395;
      0.05156 -0.02720;
      0.06699 -0.03023;
      0.08427 -0.03305;
      0.10332 -0.03564;
      0.12408 -0.03795;
      0.14645 -0.03994;
      0.17033 -0.04161;
      0.19562 -0.04295;
      0.22221 -0.04397;
      0.25000 -0.04466;
      0.27886 -0.04504;
      0.30866 -0.04509;
      0.33928 -0.04484;
      0.37059 -0.04431;
      0.43474 -0.04248;
      0.50000 -0.03978;
      0.56526 -0.03638;
      0.62941 -0.03247;
      0.69134 -0.02823;
      0.75000 -0.02384;
      0.80438 -0.01945;
      0.85355 -0.01522;
      0.89668 -0.01127;
      0.93301 -0.00770;
      0.96194 -0.00463;
      0.98296 -0.00218;
      0.99572 -0.00057;
      1.00000  0.0]';
  
  NACA0009 = [NACA0009; zeros(1,length(NACA0009))];

  
%% plot switches
frame_on = true;
% velocities_on = true;
sail_body_frames_on = true;
sail_body_frames_on = false;
% fixed_window_on = true;


  
%% parameters 
% theta = pi/8;       % heading
% epsilon = pi/6;     % sail angle relative to heading
% delta_t = pi/7;     % tail angle relative to sail
% delta_r = pi/4;     % rudder angle relative to heading
% beta_V_w = pi;      % wind direction
% chi = pi/8;         % course
% gamma_c = 3*pi/4;   % current direction
% U = 5;
% U_c = 3;
% U_w = 7;

% vessel
vessel_l = 6;
vessel_w = 1;
vessel_h = 1;
r__b_hb = [0.5; 0; 0];

% keel
keel_l = 2;
% keel_w = 0.15
% keel_h = 2
r__k_kkC = [keel_l/4; 0 ;0]; % position of aerodynamic center and origin in keel 
r__b_hk = [0; 0; 0];         % position of keel in hull

% rudder
rudder_l = 1;
% rudder_w = 0.05
% rudder_h = 0.6
r__r_rrC = [rudder_l/4; 0; 0];   % position of aerodynamic center and pivot point in rudder
r__b_hr  = [-vessel_l/2+vessel_l/10; 0; 0];  % position of rudder in hull

% sail
sail_l = 1.5;
% sail_w = 0.1
% sail_h = 5
r__s_ssC = [sail_l/4; 0 ;0]; % position of rotation point for sail and sail system
r__b_hs = [1; 0; 0];         % position of sail system pivot point in hull

% tail
tail_l = 1;
% tail_w = 0.1;
% tail_h = 1;
r__t_ttC = [0; 0 ;0];
r__s_st = [-1.5; 0; 0];

% sailsystem fram and weight
r__s_sw = [-r__s_st(1); 0; 0];

% unit vector
e_x = [0 1; 0 0; 0 0];
e_y = [0 0; 0 1; 0 0];
e_z = [0 0; 0 0; 0 1];

%% Transforms and contours

% Vector calulation from body to keel, sail system and rudder center
r__b_bkC = r__b_hk - r__b_hb;
r__b_bsC = r__b_hs - r__b_hb;
r__b_brC = r__b_hr - r__b_hb;

% transforms
R__b_r = Rzyx(0,0,delta_r);
R__s_t = Rzyx(0,0,delta_t);
R__b_s = Rzyx(0,0,epsilon);
R__n_b = Rzyx(0,0,theta);
R__n_wind = Rzyx(0,0,beta_V_w);
R__n_COG  = Rzyx(0,0,chi);
R__n_current = Rzyx(0,0,gamma_c);

% contour of vessel
start_v_contour = pi/5;
end_v_contour = 14*pi/16;
vessel__contour = start_v_contour:0.2:end_v_contour;
angles = [flip(vessel__contour) 0 -vessel__contour];
vessel__h = [(vessel_l/2)*cos(angles); (vessel_w/2)*sin(angles); 0*sin(angles)]; 
vessel__b = vessel__h - r__b_hb;
vessel__n = R__n_b*vessel__b + position;
% vessel__h_coor = [e_x, e_y, e_z];
vessel__b_coor = [e_x, e_y, e_z];
vessel__n_coor = R__n_b*vessel__b_coor + position;

% contour of keel
keel__k = Rzyx(0,0,pi)*NACA0015*keel_l + r__k_kkC;  % move rotation center of rotation 
keel__b = keel__k + r__b_bkC;                       % move to position in body
keel__n = R__n_b*keel__b + position;

% coutour of rudder
rudder__r = Rzyx(0,0,pi)*NACA0015*rudder_l + r__r_rrC;
rudder__b_r = R__b_r*rudder__r;                     % in body frame at rudder center
rudder__b = rudder__b_r + r__b_brC;                 % in body frame and at body origin
rudder__n = R__n_b*rudder__b + position;

% contour of sail 
sail__s = Rzyx(0,0,pi)*NACA0015*sail_l + r__s_ssC;
sail__b_s = R__b_s*sail__s;          % rotate from sail to body
sail__b = sail__b_s + r__b_bsC;      % move to position in body
sail__n = R__n_b*sail__b + position;
sail__s_coor = [e_x, e_y, e_z];
sail__b_s_coor = R__b_s*sail__s_coor;
sail__b_coor = sail__b_s_coor + r__b_bsC; 
sail__n_coor = R__n_b*sail__b_coor + position;

% contour of tail
tail__t = Rzyx(0,0,pi)*NACA0015*tail_l + r__t_ttC;
tail__s_t = R__s_t*tail__t;
tail__s = tail__s_t + r__s_st;
tail__b_s = R__b_s*tail__s;
tail__b = tail__b_s + r__b_bsC;
tail__n = R__n_b*tail__b + position;

% contour of sail system frame and weight
frame__s = [r__s_st, r__s_sw];
frame__b_s = R__b_s*frame__s;          % rotate from sail to body
frame__b = frame__b_s + r__b_bsC;      % move to position in body
frame__n = R__n_b*frame__b + position;

%% Indication vectors
wind_vector__n = R__n_wind*U_w*([1; 0; 0]); % can use velocity directly instead
COG_vector__n = R__n_COG*U*([1; 0; 0]);
current__n = R__n_current*U_c*([1; 0; 0]);

%% Plot
% figure(1)
patch(vessel__n(2,:)',vessel__n(1,:)',[0.9290 0.6940 0.1250])
hold on
patch(keel__n(2,:)',keel__n(1,:)',[0.8500 0.3250 0.0980])
patch(rudder__n(2,:)',rudder__n(1,:)',[0.8500 0.3250 0.0980])
if frame_on == true
    plot(frame__n(2,:)',frame__n(1,:)', 'k','LineWidth',1.5)
end
patch(sail__n(2,:)',sail__n(1,:)',[0.3010 0.7450 0.9330])
patch(tail__n(2,:)',tail__n(1,:)',[0.3010 0.7450 0.9330])
if fixed_window_on == true
    xlim([-vessel_l/2-2+position(2), vessel_l/2+2+position(2)])
    ylim([-vessel_l/2-2+position(1), vessel_l/2+2++position(1)])
else
    axis equal
end

title('Saildrone pose and velocity')
ylabel('NORTH [m]')
xlabel('EAST [m]')
grid on

if velocities_on == true
    vector_scaling_w = 0.015;
    vector_scaling_c = 0.015*2.5;
    position_wind = [0.75, 0.75];
    position_hydr = [0.75, 0.25];
    vectorTriangle(position_wind,vector_scaling_w,COG_vector__n,wind_vector__n,'r','b','k','           U','U_{w}','U_{rw}')
    vectorTriangle(position_hydr,vector_scaling_c,COG_vector__n,current__n,'r','b','k','           U','U_{c}','U_{rc}')
end

if sail_body_frames_on == true
    plotCoordinateSystem2D(vessel__n_coor)
    plotCoordinateSystem2D(sail__n_coor)
end

hold off
% Ux = [position_wind(2),position_wind(2)+vector_scaling*COG_vector__n(2)];
% Uy = [position_wind(1),position_wind(1)+vector_scaling*COG_vector__n(1)];
% V_w_x = [position_wind(2)+vector_scaling*(COG_vector__n(2)-wind_vector__n_w(2)),position_wind(2)+vector_scaling*COG_vector__n(2)];
% V_w_y = [position_wind(1)+vector_scaling*(COG_vector__n(1)-wind_vector__n_w(1)),position_wind(1)+vector_scaling*COG_vector__n(1)];
% U_r_x = [position_wind(2),position_wind(2)+vector_scaling*(COG_vector__n(2)-wind_vector__n_w(2))];
% U_r_y = [position_wind(1),position_wind(1)+vector_scaling*(COG_vector__n(1)-wind_vector__n_w(1))];
% % a = annotation('textarrow',wind_vector__n(2,:)',wind_vector__n(1,:)','String','y = x ');
% a = annotation('textarrow',Ux,Uy,'String','            U');
% a.Color = 'r'
% b = annotation('textarrow',V_w_x,V_w_y,'String','V_w');
% b.Color = 'b'
% c = annotation('textarrow',U_r_x,U_r_y,'String','U_r');
% quiver(3,3,wind_vector__n_w(2),wind_vector__n_w(1))

% for ii = 1:3:length(X)-1
% 
%     headWidth = 200 * sqrt((X(ii+1)-X(ii)).^2 + (Y(ii+1)-Y(ii)).^2); % set the headWidth, function of length of arrow
%     angled = floor(atan2(Y(ii+1)-Y(ii),X(ii+1)-X(ii))*180/pi) + 1; %get the angle
%     ah = annotation('arrow',...
%         'Color', cmap(angled,:),...
%         'headStyle','cback1','HeadLength',50,'HeadWidth',headWidth);
%     set(ah,'parent',gca);
%     set(ah,'position',[X(ii) Y(ii) X(ii+1)-X(ii) Y(ii+1)-Y(ii)]);
% end

% ah = annotation('arrow','headStyle','cback1','HeadLength',5,'HeadWidth',5);
% set(ah,'parent',gca);
% p1 = [2 3];                         % First Point
% p2 = [9 8];                         % Second Point
% dp = p2-p1;                         % Difference
% figure(2)
% quiver(p1(1),p1(2),dp(1),dp(2),0)
% grid
% axis([0  10    0  10])
% text(p1(1),p1(2), sprintf('(%.0f,%.0f)',p1))
% text(p2(1),p2(2), sprintf('(%.0f,%.0f)',p2))


end