function [x_dot, beta_U_rw] = sailsystem_dynamics_LIN(x, v__n_rw, delta_t, r__s_stAC, K_CL, I_zz_ss)
    % U__s_rw = relative velocity vector in sail frame with respect to wind
    % r__s_stAC = vector from sail pivot point to tail AC when delta_t is
    % zero
    % K_CL = the lift constant given by the density and lift coefficient
    % I_zz_ss = sailsystem inertia around pivot point
    
    r  = abs(r__s_stAC(1)); %0.5*abs(r__s_stAC(1)); %% length from sail pivot to tail AC along x-axis 
    
    
    % beta_U_r = direction of relative velocity
    % U_r = magnitude of relative wind
    beta_U_rw = atan2(v__n_rw(2), v__n_rw(1));
    U_rw = sqrt(v__n_rw(1)^2 + v__n_rw(2)^2);
    
    if U_rw < 0.0001
        beta_U_rw = 0;
    end
    
    u = [beta_U_rw; delta_t];
    
    % forgot the arm this is only the force induced by the tail
%     A = [0,                         1;
%         -((U_rw^2)*K_CL)/I_zz_ss,    -(U_rw*K_CL*r)/I_zz_ss];
%     
%     B = [0,                         0;
%          ((U_rw^2)*K_CL)/I_zz_ss,    -((U_rw^2)*K_CL)/I_zz_ss];

    A = [0,                         1;
        -((U_rw^2)*K_CL*r)/I_zz_ss,    -(U_rw*K_CL*r^2)/I_zz_ss];
    
    B = [0,                         0;
         ((U_rw^2)*K_CL*r)/I_zz_ss,    -((U_rw^2)*K_CL*r)/I_zz_ss];
     
    x_dot = A*x + B*u;
    
    

    

end