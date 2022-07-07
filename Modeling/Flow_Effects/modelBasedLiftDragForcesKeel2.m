function [tau_lift_and_drag, alpha, F_lift, F_drag] = modelBasedLiftDragForces(nu_linkAC, vel_flow, rho, foil_param)
    % nu_linkAC = the velocity of the link aerodynamic center relative to
    % the NED or inertial frame given in the link frame
% %     % nu_current = the current velocity in the same frame
    % vel_flow is the velocity of the fluid flow with respect to the same
    % frame as the link
    % foil_height = the span length
    % foil_length = the chord length
    % C_L = the lift coefficient
    % C_D = the drag coefficient
    % S = wind area

    % for a vessel the contribution from z-axis is minimal and hence
    % neglected. The velocities are given in the aerodynamic center hence 
    % the angular velocity compenents do not contribute 
    
    %%%%%%%%%%%%%% TODO %%%%%%%%%%%%%%%%%
%     implement input for e_Oswald and parasitic drag
    
    S = foil_param.S;
    AR = foil_param.AR; % can also use foil_heigth/foil_legth for rectangle
    
    %% Velocities and angle of attack AOA
    u_link = nu_linkAC(1);
    v_link = nu_linkAC(2);
    u_flow = vel_flow(1);
    v_flow = vel_flow(2);
    
    U = sqrt(u_link^2 + v_link^2);
    U_c = sqrt(u_flow^2 + v_flow^2);
    U_r = U - U_c;
    
    alpha = atan2(v_link-v_flow, u_link-u_flow);
    
    
    %% Linear lift model (se Fossen/beard Lecture 3 p31 chapter 2 and 4)
    [C_L, C_D] = liftDragModelNACA0015(alpha,foil_param);

%     if C_D < 0
%         C_D = 0 
%     end
    
%     C_L_0 = 0; % assuming symmertic wing, hence no lift when angle of attack is zero
%     C_L_alpha = (pi*AR)/(1 + sqrt(1 + (AR/2)^2));
%     
%     C_L = C_L_0 + C_L_alpha*alpha; % linear aerodynamic model % factors as function or rotational velocity are neglected as angular velocity will be small
%     
%     e_OS = 0.9; % OSwald efficiency factor typical value is in the range 0.8-1.0, see Introduction to Aircraft Flight Mechanics
%     C_D_parasitic = 0.01373; % drag at 0 deg angle of attack (C_D(alpha) = C_D_parasitic + 0) of NACA 0015 wing
% %     C_D_alpha = 0.25*C_L_alpha; % must be replaced by a formula ->>>>>>>>>>>>>>>>>>>>>>>>>>>>>blir negative???? ->>>>>>>>>> and nu_flow need a transfrom!!!
%     
%     C_D = C_D_parasitic + ((C_L_0 + C_L_alpha*alpha)^2)/(pi*e_OS*AR); % if this model in linear, the drag will be negative for negative angle of attack!!
    

    %% Forces
    F_lift = (1/2)*rho*(U_r^2)*S*C_L;
    F_drag = (1/2)*rho*(U_r^2)*S*C_D;
    % the foil is assumed to be symmetric hence it do not create moment
    % around AC
    
    % transfrom to link
    R__link_flow = [cos(alpha),  -sin(alpha);
                    sin(alpha),  cos(alpha)];
                
                
%     R__link_flow = [1,  0;
%                     0,  1];
    
    XY_link = R__link_flow*[-F_drag; -F_lift];
                  
    tau_lift_and_drag = [XY_link; 0; 0; 0; 0]; % 6DOF 
