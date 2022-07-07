function [tau_tail] = tail_dynamics(eta_tail, nu_tail, nu_dot_tail, beta_V_rw)
%TAIL_DYNAMICS Summary of this function goes here
% eta_tail =  position and orientation
% nu_tail = spatial velocity
% nu_dot_tail = spatial acceleration
% beta_V_rw = relative wind direction in NED

%%
v__t_nt = nu_tail(1:3) 
omega_t_nt= nu_tail(4:6)

%% Tail configuration
m_tail = 5;             % tail mass
tail_l = 1;             % tail lenght
tail_w = 0.1;           % tail width
tail_h = 1;             % tail height
r__t_tjtC = [0; 0 ;0];  %vector from tail origin (pivot point/joint) to tail center of AC and CG in tail frame
% r__t_tjtC do not need to be equal to r__t_ttC in saildrone_plotting_2D

%% Inertia and system inertia matrix
I_tail_zz = m_tail*(l_tail^2+w_tail^2)/12;
I_tail_yy = m_tail*(l_tail^2+h_tail^2)/12;
I_tail_xx = m_tail*(h_tail^2+w_tail^2)/12;

I__CG_tail = diag([I_tail_xx, I_tail_yy, I_tail_zz]);

I3x3 = eye(3);
O3x3 = zeros(3,3);

MRB__CG_tail = [m_tail*I3x3,   O3x3;
                 O3x3,          I__CG_tail];

CRB__CG_tail = [m*Smtrx(omega_t_nt), O3x3;
                 O3x3,                 -Smtrx(I__CG_tail*omega_t_nt)];
       
H = Hmtrx(r__t_tjtC);

MRB_tail = H' * MRB__CG_tail * H
CRB_tail = H' * CRB__CG_tail * H

%% EoM
% tau_aero_AC = air_load(l_tail, h_tail, eta_tail, beta_V_rw)
% tau_g_tail = tauGravity = (m_tail, eta_tail, r__t_tjtC)

tau_tail = MRB_tail*nu_dot_tail + CRB_tail*nu_tail - tau_g_tail - tau_aero_tail; 

end

