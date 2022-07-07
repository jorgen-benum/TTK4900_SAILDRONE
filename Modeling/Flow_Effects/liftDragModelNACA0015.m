function [C_L, C_D] = liftDragModelNACA0015(alpha,foil_param)
% calculate the lift and drag coefficients based on the formulas found in
% the Small Unmanned aircraft book, modified to suit the NACA0015 data
% found in "Aerodynamic Characteristics of Seven Symmetrical Airfoil 
% Sections Through 180-Degree Angle of Attack for Use in Aerodynamic 
% Analysis of Vertical Axis Wind Turbines" 

%% setup paramteres lift
% Bleding function 
alpha0 = 0.34; %0.32 %0.32 %0.4      %the cutoff angle
M = 40; %40 %15                      %transition gain

% Flat plate model
% C_L_fp = 2*sign(alpha)*((sin(alpha))^2)*cos(alpha)
fp_gain = 1.3; %1.25 %1.2 % gain to adjust the amplitude of the flate plate model

%% setop paramters drag
peak_C_D = 1.75; % drag coefficient peak of NACA0015 wing found in osti.gov/servlets/purl/6548367

%% Lift coefficient
% Linear lift coefficients
C_L_0 = 0;                          % this assumpition must hold
C_L_alpha = foil_param.C_L_alpha;


% Blending
sigma_blend_num = (1 + exp(-M*(alpha-alpha0))+ exp(M*(alpha+alpha0)));
sigma_blend_den = (1 + exp(-M*(alpha-alpha0)))*(1 + exp(M*(alpha+alpha0)));
sigma_blend = sigma_blend_num/sigma_blend_den;

% Linear model
C_L_lin = C_L_0 + alpha*C_L_alpha;

% Flat plate model
% C_L_fp = 2*sign(alpha)*((sin(alpha))^2)*cos(alpha);
C_L_fp = sin(2*alpha); % alternative to flate plate model

% Lift coefficient
C_L = (1-sigma_blend)*C_L_lin + fp_gain*sigma_blend*C_L_fp;


%% Drag coefficient
% the square model is similar to a sin(x)^2 model at small angles and the
% sin(x)^2 model replecate the test data pattern and is periodic with pi

% Parasitic drag
C_L_p = foil_param.C_D_parasitic;

% Same coefficient as original quadratic function with C_L_0=0 (symmertic wing)
C_D_quad_term = (C_L_alpha^2)/(pi*foil_param.e_OS*foil_param.AR);

% adjustment to fit drag peak to testdata
peak_adj_gain = peak_C_D -  (C_L_p + C_D_quad_term*(sin(pi/2))^2);

% Drag coefficient
C_D = C_L_p + C_D_quad_term*(sin(alpha))^2 + peak_adj_gain*(sin(alpha))^4;
% test with adjusting witdh to better fit NACA0015 wingdata
%     C_D = C_L_p + C_D_quad*(sin(alpha))^2 + peak_adj_gain*sqrt(((sin(alpha))^2)^3.1);



end

