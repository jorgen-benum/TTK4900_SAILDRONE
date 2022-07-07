init_saildrone_dynamics_v5

foil_param = sail

alpha_vector = linspace(-pi,pi,1000);

blending_value = [];
C_L_curve = [];
C_D_curve = [];
C_D_quad_curve = [];
C_M_curve = []
for i=1:length(alpha_vector)
    alpha = alpha_vector(i);

    [C_L, C_D, C_M] = liftDragModelNACA0015_AERODAS(alpha,foil_param)
    
    C_L_curve = [C_L_curve, C_L];
    C_D_curve = [C_D_curve, C_D];
    C_M_curve = [C_M_curve, C_M];

end

% figure(99)
% plot(alpha_vector.*(180/pi), blending_value)
% % xlim([-pi,pi])
% xlim([-180,180])
% grid on

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
% plot(alpha_vector.*(180/pi), C_D_quad_curve)
% xlim([-pi,pi])
xlim([0,180])
ylim([0, 2])
xlabel('Angle of attack $\alpha$ [deg]','Interpreter','latex')
ylabel('Drag coefficient $C_{D}(\alpha)$','Interpreter','latex')
% legend('sin approximation', 'quadratic approximation')
grid on
hold off
x0=20;
y0=20;
width=800;
height=400
set(gcf,'position',[x0,y0,width,height])

figure(102)
plot(alpha_vector.*(180/pi), C_M_curve)
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