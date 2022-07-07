%% Calulate and display the drag and lift coeffients and the lift coefficent bledning function

% foil_param = sail

alpha_vector = linspace(-pi,pi,500);

% bleding function
alpha0 = 0.3%0.36%0.34 %0.32 %0.32 %0.4    %the cutoff angle
M = 120%40%30%40 %40 %15         %transition gain

% linear lift koefisients
C_L_0 = 0;
C_L_alpha = 4.25;

% % Flat plate model
% C_L_fp = 2*sign(alpha)*((sin(alpha))^2)*cos(alpha)
fp_gain = 1.3; %1.25 %1.2 % gain to adjust the amplitude of the flate plate model

% Drag model
% the square model is similar to a sin(x)`2 model at small angles and the
% sin(x)^2 model rplecate the test data pattern and is periodic with pi
C_L_p = 0.0137;
C_D_quad = (C_L_alpha^2)/(pi*0.9*5) % same coefficient as original quadratic function with C_L_0=0

peak_C_D = 1.75; % drag coefficient peak of NACA0015 wing found in osti.gov/servlets/purl/6548367
peak_adj_gain = peak_C_D -  (C_L_p + C_D_quad*(sin(pi/2))^2)


blending_value = []
C_L_curve = []
C_D_curve = []
C_D_quad_curve = []
for i=1:length(alpha_vector)
    alpha_val = alpha_vector(i)
    
    if abs(alpha_val)> pi/2
        alpha = pi-alpha_val
    else
        alpha = alpha_val
    end
    
    % Blending
    sigma_blend_num = (1 + exp(-M*(alpha-alpha0))+ exp(M*(alpha+alpha0)));
    sigma_blend_den = (1 + exp(-M*(alpha-alpha0)))*(1 + exp(M*(alpha+alpha0)));
    sigma_blend = sigma_blend_num/sigma_blend_den;
    blending_value = [blending_value, sigma_blend]
    
%     [C_L, C_D] = liftDragModelNACA0015(alpha,foil_param)
    
    % Linear model
    C_L_lin = C_L_0 + alpha*C_L_alpha;
    
    % Flat plate model
%     C_L_fp = 2*sign(alpha)*((sin(alpha))^2)*cos(alpha);
    C_L_fp = sin(2*alpha);
    
    % Lift coefficient
    C_L = (1-sigma_blend)*C_L_lin + fp_gain*sigma_blend*C_L_fp;
    C_L = (1-sigma_blend)*C_L_lin + 1.05*sigma_blend*C_L_fp;
    
    if abs(alpha_val)> pi/2
        C_L = - 0.8*C_L
    end
    
    
    
    C_L_curve = [C_L_curve, C_L];
    
    % Drag coefficient
    C_D = C_L_p + C_D_quad*(sin(alpha))^2 + peak_adj_gain*(sin(alpha))^4;
    % test with adjusting witdh to better fit NACA0015 wingdata
%     C_D = C_L_p + C_D_quad*(sin(alpha))^2 + peak_adj_gain*sqrt(((sin(alpha))^2)^3.1);
    C_D_curve = [C_D_curve, C_D];
    
    C_Dquad = C_L_p + C_D_quad*alpha^2;
    C_D_quad_curve = [C_D_quad_curve, C_Dquad]

end

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
