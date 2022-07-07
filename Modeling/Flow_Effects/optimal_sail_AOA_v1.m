%% definitions

deg2rad = pi/180;

%%
% angle specter
% optimalization parameters:
max_sail_angle = 50*deg2rad
alpha_vector = linspace(-max_sail_angle,max_sail_angle,1000);

%% Calulate and display the drag and lift coeffients and the lift coefficent bledning function


% sail
h_wing = 5;             % [m]  height 
l_wing = 1;             % [m]  lenght 
t_wing = l_wing*0.15;   % NACA0015

% foil lift parameters of symmetric rudder
S = h_wing*l_wing;
AR = (h_wing^2)/S
% rudder.C_L_0 = 0
% linear lift koefisients
C_L_0 = 0;
C_L_alpha = (pi*AR)/(1 + sqrt(1 + (AR/2)^2));
K_C_L = (1/2)*rho*S*C_L_alpha;

% foil drag paramters
e_OS = 0.9; % OSwald efficiency factor typical value is in the range 0.8-1.0, see Introduction to Aircraft Flight Mechanics
C_D_parasitic = 0.008; %0.01373; %0.00000001;% 0.01373; % drag at 0 deg angle of attack (C_D(alpha) = C_D_parasitic + 0) of NACA 0015 wing



% bleding function
alpha0 = 0.305%0.36%0.34 %0.32 %0.32 %0.4    %the cutoff angle
M = 120%40%30%40 %40 %15         %transition gain


% % Flat plate model
% C_L_fp = 2*sign(alpha)*((sin(alpha))^2)*cos(alpha)
% fp_gain = 1.3; %1.25 %1.2 % gain to adjust the amplitude of the flate plate model
% post stall max lift:
F1 = 1.190*(1-(t_wing/l_wing)^2)
F2 = 0.65 + 0.35*(exp(-(9/AR)^(2.3)))
max_post_lift = F1*F2

% Drag model
% the square model is similar to a sin(x)`2 model at small angles and the
% sin(x)^2 model rplecate the test data pattern and is periodic with pi
C_D_p = C_D_parasitic;
C_D_quad = (C_L_alpha^2)/(pi*0.9*AR) % same coefficient as original quadratic function with C_L_0=0

% post stall drag
G1 = 2.3*exp(-(0.65*(t_wing/l_wing))^(0.9))
G2 = 0.52 + 0.48*exp(-(6.5/AR)^(1.1))
peak_C_D = G1*G2; %1.75 drag coefficient peak of NACA0015 wing found in osti.gov/servlets/purl/6548367
peak_adj_gain = peak_C_D -  (C_D_p + C_D_quad*(sin(pi/2))^2)

% moment coeff
beta0 = 16*pi/180;
N = 2
moment_slope = 0.14
moment_gain = 0.27


blending_value = [];
C_L_curve = [];
C_D_curve = [];
C_D_quad_curve = [];
C_M_curve = []
C_M_curve2 = []
for i=1:length(alpha_vector)
    alpha = alpha_vector(i);
    
    
    % Blending
    sigma_blend_num = (1 + exp(-M*(alpha-alpha0))+ exp(M*(alpha+alpha0)));
    sigma_blend_den = (1 + exp(-M*(alpha-alpha0)))*(1 + exp(M*(alpha+alpha0)));
    sigma_blend = sigma_blend_num/sigma_blend_den;
    blending_value = [blending_value, sigma_blend];
    
%     [C_L, C_D] = liftDragModelNACA0015(alpha,foil_param)
    
    % Linear model
    C_L_lin = C_L_0 + alpha*C_L_alpha;
    
    % Flat plate model
%     C_L_fp = 2*sign(alpha)*((sin(alpha))^2)*cos(alpha);
    C_L_fp = sin(2*alpha);
    
    % Lift coefficient
%     C_L = (1-sigma_blend)*C_L_lin + fp_gain*sigma_blend*C_L_fp;
    C_L = (1-sigma_blend)*C_L_lin + max_post_lift*sigma_blend*C_L_fp;
    
    
    
    C_L_curve = [C_L_curve, C_L];
    
    % Drag coefficient
    C_D = C_D_p + C_D_quad*(sin(alpha))^2 + peak_adj_gain*(sin(alpha))^4;
    % test with adjusting witdh to better fit NACA0015 wingdata
%     C_D = C_L_p + C_D_quad*(sin(alpha))^2 + peak_adj_gain*sqrt(((sin(alpha))^2)^3.1);
    C_D_curve = [C_D_curve, C_D];
    
    C_Dquad = C_D_p + C_D_quad*alpha^2;
    C_D_quad_curve = [C_D_quad_curve, C_Dquad];
    
    
    % Blending
    beta_blend_num = (1 + exp(-N*(alpha-beta0))+ exp(N*(alpha+beta0)));
    beta_blend_den = (1 + exp(-N*(alpha-beta0)))*(1 + exp(N*(alpha+beta0)));
    beta_blend = beta_blend_num/beta_blend_den;
    
    C_M = (1-beta_blend)*moment_slope*alpha - beta_blend*moment_gain*sin(alpha)*alpha*sign(alpha);
    C_M_curve = [C_M_curve, C_M];
    
    CM_test = -0.18*alpha^3 + 0.01*alpha;
    C_M_curve2 = [C_M_curve2,   CM_test] ;

end




%% optimal AOA testplots

% iterate over all values within alpha = 0 to 55 select max and store 

f_AOA = sqrt(C_L_curve.^2 + C_D_curve.^2)

A_AOA_curve = [];
for i=1:length(C_L_curve)
    A_AOA = -atan2(C_D_curve(i),C_L_curve(i));
    A_AOA_curve = [A_AOA_curve  ,A_AOA];
end


% beta_rw = (-180+60)*pi/180
beta_rw = (180-63)*pi/180
psi = 0*pi/180

% fx_coef = f_AOA.*cos(beta_rw - psi - sign(alpha_vector)*(pi/2) + A_AOA_curve)
% fx_coef = sign(alpha_vector).*f_AOA.*cos(beta_rw - psi +(pi/2) + A_AOA_curve) 
fx_coef = f_AOA.*cos(beta_rw - psi - (pi/2) + A_AOA_curve) 
fy_coef = f_AOA.*sin(beta_rw - psi - (pi/2) + A_AOA_curve) 
% fx_coef = -sign(alpha_vector)*(pi/2) 

f_relation = fx_coef - abs(fy_coef)
f_ratio = fx_coef./abs(fy_coef)


% % optimalization parameters:
% max_sail_angle = 50*deg2rad
resolution = 1 % measuremtn / deg
nr_section = 360*resolution

step = 2*pi/(360*resolution)

sail_AOA_table = []
rel_sail_AOA_table = []

ratio = 0.9

some_values = []
for i = 0:nr_section
    gamma = -pi + i*step;
    
    % the functions
    fx_coef_loop = f_AOA.*cos(gamma - (pi/2) + A_AOA_curve)
    fy_coef_loop = f_AOA.*sin(gamma - (pi/2) + A_AOA_curve)
    
    % opt forward propulsion
    [opt_Fx_AOAopt, i_max] = max(fx_coef_loop)

    
    % store the angle to the reference vector gamma, the optimal AOA, the
    % force along the reference vector and the force normal to it following
    % the ned frame
    sail_AOA_table = [sail_AOA_table [gamma; alpha_vector(i_max); opt_Fx_AOAopt; fy_coef_loop(i_max); i_max]];
    
    % transition fase
    esv = []
    rel_max_index = ceil(nr_section/2);
    rel_maximum_value = 0.9;
    

    % under here is custom to setup with 1000 alpha points between -pi and pi 
    
    if (i_max == 1)||(i_max == 340)
        f_relative_loop1 = fx_coef_loop(1) - abs(fy_coef_loop(1));
        f_relative_loop2 = fx_coef_loop(340) - abs(fy_coef_loop(340))
        
        if (i_max == 1)
            if  (f_relative_loop1 < f_relative_loop2) && ((fx_coef_loop(340)/fx_coef_loop(1)) > ratio) %&&
                i_max_relative = 340;
            else
                i_max_relative = i_max;
          
            end
        elseif (i_max == 340)
            if ((f_relative_loop1 > f_relative_loop2)) && ((fx_coef_loop(1)/fx_coef_loop(340)) > ratio)
%                 gvs
%             end
%             if (fx_coef_loop(1)/fx_coef_loop(340)) > ratio
                i_max_relative = 1;
%                 hku
            else
                i_max_relative = i_max;
            end
        end        
    elseif ((i_max == 661)||(i_max == 1000))
        f_relative_loop4 = fx_coef_loop(661) - abs(fy_coef_loop(661));
        f_relative_loop3 = fx_coef_loop(1000) - abs(fy_coef_loop(1000))
        some_values = [some_values [f_relative_loop4; f_relative_loop3; i; i_max]]
        
        if (i_max == 661)
            if (f_relative_loop3 > f_relative_loop4) && (((fx_coef_loop(1000)/fx_coef_loop(660)) > ratio))
                i_max_relative = 1000;
%                 sdfsff
            else
                i_max_relative = i_max;
%                 zdvzd
            end
        elseif (i_max == 1000)
            if ((fx_coef_loop(661)/fx_coef_loop(1000)) > ratio) && (f_relative_loop4 > f_relative_loop3)
                i_max_relative = 661;
                hku
            else
                i_max_relative = i_max;
                i_max
%                 dcsv
            end
        end
        
    else
        i_max_relative = i_max;
        
    end
        
    
    rel_sail_AOA_table = [rel_sail_AOA_table [gamma; alpha_vector(i_max_relative); fx_coef_loop(i_max_relative); fy_coef_loop(i_max_relative); i_max_relative]];
    
end    




%% ploting
figure(92)
plot(sail_AOA_table(1,1:360).*(180/pi), sail_AOA_table(2,1:360).*(180/pi))
hold on
plot(rel_sail_AOA_table(1,1:360).*(180/pi), rel_sail_AOA_table(2,1:360).*(180/pi))
% xlim([-pi,pi])
xlim([-180,180])
% ylim([-1.2,1.2])
xlabel('$\gamma_{rw}$ [deg]','Interpreter','latex')
ylabel('Optimal sail angle of attack $\alpha_{s}$ [deg]','Interpreter','latex')
legend('Optimal $\alpha_{s}$', 'Optimal$^{*}$ $\alpha_{s}$','Interpreter','latex')
% ylabel('Force magnitude of coefficient f$(\alpha)$ along body x','Interpreter','latex')
grid on
x0=2000;
y0=400;
width=800;
height=400
set(gcf,'position',[x0,y0,width,height])



figure(80)
plot(alpha_vector.*(180/pi), fx_coef)
hold on
plot(alpha_vector.*(180/pi), fy_coef)
% xlim([-pi,pi])
xlim([-50,50])
% ylim([-1.2,1.2])
xlabel('Angle of attack $\alpha$ [deg] given $\gamma = 117^{\circ}$','Interpreter','latex')
ylabel('Force magnitude coefficients','Interpreter','latex')
legend('$C_{x}(\alpha_{s})$', '$C_{y}(\alpha_{s})$','Interpreter','latex')
grid on
x0=2000;
y0=400;
width=800;
height=400
set(gcf,'position',[x0,y0,width,height])


figure(96)
plot(alpha_vector.*(180/pi), fx_coef)
% xlim([-pi,pi])
xlim([-90,90])
% ylim([-1.2,1.2])
xlabel('Angle of attack $\alpha$ [deg]','Interpreter','latex')
ylabel('Force magnitude of coefficient f$(\alpha)$ along body x','Interpreter','latex')
grid on
x0=2000;
y0=400;
width=800;
height=400
set(gcf,'position',[x0,y0,width,height])

figure(95)
plot(alpha_vector.*(180/pi), fy_coef)
% xlim([-pi,pi])
xlim([-90,90])
% ylim([-1.2,1.2])
xlabel('Angle of attack $\alpha$ [deg]','Interpreter','latex')
ylabel('Force magnitude of coefficient f$(\alpha)$ along body y','Interpreter','latex')
grid on
x0=3000;
y0=400;
width=800;
height=400
set(gcf,'position',[x0,y0,width,height])

figure(95)
plot(alpha_vector.*(180/pi), fy_coef)
% xlim([-pi,pi])
xlim([-90,90])
% ylim([-1.2,1.2])
xlabel('Angle of attack $\alpha$ [deg]','Interpreter','latex')
ylabel('Force magnitude of coefficient f$(\alpha)$ along body y','Interpreter','latex')
grid on
x0=3000;
y0=400;
width=800;
height=400
set(gcf,'position',[x0,y0,width,height])


figure(94)
plot(alpha_vector.*(180/pi), f_relation)
% xlim([-pi,pi])
xlim([-90,90])
% ylim([-1.2,1.2])
xlabel('Angle of attack $\alpha$ [deg]','Interpreter','latex')
ylabel('Force magnitude of coefficient relation','Interpreter','latex')
grid on
x0=20;
y0=20;
width=800;
height=400
set(gcf,'position',[x0,y0,width,height])

figure(93)
plot(alpha_vector.*(180/pi), f_ratio)
% xlim([-pi,pi])
xlim([-90,90])
% ylim([-1.2,1.2])
xlabel('Angle of attack $\alpha$ [deg]','Interpreter','latex')
ylabel('Force magnitude of coefficient ratio','Interpreter','latex')
grid on
x0=20;
y0=20;
width=800;
height=400
set(gcf,'position',[x0,y0,width,height])

figure(97)
plot(alpha_vector.*(180/pi), A_AOA_curve)
% xlim([-pi,pi])
xlim([-90,90])
% ylim([-1.2,1.2])
xlabel('Angle of attack $\alpha$ [deg]','Interpreter','latex')
ylabel('Force magnitude of coefficient f$(\alpha)$','Interpreter','latex')
grid on
x0=20;
y0=20;
width=800;
height=400
set(gcf,'position',[x0,y0,width,height])




figure(99)
plot(alpha_vector.*(180/pi), blending_value)
% xlim([-pi,pi])
xlim([-180,180])
grid on

%
figure(100)
plot(alpha_vector.*(180/pi), C_L_curve)
% xlim([-pi,pi])
xlim([0,180])
ylim([-1.2,1.2])
xlabel('Angle of attack $\alpha$ [deg]','Interpreter','latex')
ylabel('Lift coefficient $C_{L}(\alpha)$','Interpreter','latex')
grid on
x0=20;
y0=20;
width=800;
height=400
set(gcf,'position',[x0,y0,width,height])

figure(101)
clf
plot(alpha_vector.*(180/pi), C_D_curve)
hold on
plot(alpha_vector.*(180/pi), C_D_quad_curve)
% xlim([-pi,pi])
xlim([0,180])
ylim([0, 2])
xlabel('Angle of attack $\alpha$ [deg]','Interpreter','latex')
ylabel('Drag coefficient $C_{D}(\alpha)$','Interpreter','latex')
legend('sin approximation', 'quadratic approximation')
grid on
hold off
x0=20;
y0=20;
width=800;
height=400
set(gcf,'position',[x0,y0,width,height])

figure(102)
plot(alpha_vector.*(180/pi), C_M_curve)
hold on
plot(alpha_vector.*(180/pi), C_M_curve2)
% xlim([-pi,pi])
xlim([0,180])
% ylim([-1.2,1.2])
xlabel('Angle of attack $\alpha$ [deg]','Interpreter','latex')
ylabel('Moment coefficient $C_{M}(\alpha)$','Interpreter','latex')
grid on
x0=20;
y0=20;
width=800;
height=400
set(gcf,'position',[x0,y0,width,height])





