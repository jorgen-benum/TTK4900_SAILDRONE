% 
% % title('Saildrone pose and velocity')
% % title('Sail angle of attack')
title(' ')
% % ylabel('Angle [deg]')
% % ylabel('Course [deg]')
% % ylabel('VMG velocity [m/s]')
% % ylabel('Actuator moment [Nm]')
% % ylabel('force [N]')
% ylabel('AOA [deg]','FontSize',12)
% xlabel('time [s]', 'FontSize',12)
ylabel('NORTH [m]', 'FontSize',12)
xlabel('EAST [m]', 'FontSize',12)
% ylabel('Angle [deg]','FontSize',12)
% ylabel('Velocity [m/s]','FontSize',12)
% ylabel('Desired path angle $\pi_p$ [deg]','Interpreter','latex','FontSize',12)
% xlabel('time [s]', 'FontSize',12)
% % ylabel('Sail angle of attack $\alpha_{s}$ [deg]','Interpreter','latex')
% 
% position_old = position_old
% speed = zeros(length(position_old),1)
% patch([position_old(2,:) nan],[position_old(1,:) nan],[speed' nan],[speed' nan],'FaceColor','k', 'LineStyle', '--');
% % % % pathSpeedPlotter(position_old + 10, zeros(length(position_old),1)); % plot path
% 
% 
% 
% % xlim([140 180])
% % ylim([-55 55])
% % xlim([0 12])
% % ylim([0 16])
% % xlim([70 100])
% % ylim([-50 50])
% % xlim([100 200])
% % ylim([-40 330])
% xlim([55 100])
% ylim([40 80])
% xlim([0.9 1.5])
% ylim([-1 16])
% ylim([-1 6])
% 
% xlim([32 42])
% ylim([-20 20])
% 
% xlim([137 147])
% ylim([243 267])
% 
% xlim([133 151])
% ylim([13 43])
% 
xlim([50 150])
ylim([-10 510])

xlim([50 150])
ylim([-10 300])

xlim([50 299.999])
ylim([-20 270])

xlim([190 300])
ylim([-41 15])

xlim([45 155])
ylim([330 540])


xlim([180 200])
ylim([-50 50])

xlim([150 200])
ylim([1.5 4])

xlim([0 5])
ylim([-20 1])

xlim([20 120])
ylim([-60 30])

xlim([75 135])
ylim([-25 35])

xlim([-60 200])
ylim([-20 520])

% xlim([0 600])
% ylim([-180 185])

xlim([0 600])
ylim([-180 190])

xlim([500 850])
ylim([-60 60])

xlim([550 1100])
ylim([-60 60])

xlim([-50 400])
ylim([-20 520])

% xlim([45 155])
% ylim([440 680])


% xlim([133 151])
% ylim([288 318])
% 
% xlim([110 140])
% ylim([58 88])
% 
% xlim([860 960])
% ylim([-65 100])
% 
% xlim([860 960])
% ylim([0.5 3.3])
% 
% xlim([860 960])
% ylim([-13 20])

% lgd = legend('$f_{ss,x}^{s\;*}$', '$f_{ss,x}^{s}$','$f_{ss,y}^{s\;*}$', '$f_{ss,y}^{s}$', 'Interpreter','latex')
% lgd = legend('$\alpha_{s}^{*}$', '$\alpha_{s}$', 'Interpreter','latex')
% lgd = legend('Saildrone pose and velocity', 'Interpreter','latex')
% lgd.FontSize = 12;
% lgd = legend('Reference AOA $\alpha_{s_{ref}}$',  'Simplified model','Simulation model', 'Interpreter','latex')
% lgd = legend('Reference AOA',  'Simplified model','Simulation model', 'Interpreter','latex')
% lgd = legend('Reference AOA',  'Simplified model','Simulation model')
% lgd = legend('Heading reference',  'Roll angle $\phi$','Sail AOA reference', 'Sail AOA $\alpha_{s}$','latex')
% lgd = legend('Heading reference',  'Roll angle $\phi$','Sail AOA reference', 'Sail AOA $\alpha_{s}$','latex')

% lgd = legend('Heading reference',  'Desired heading', 'Keel AOA $\alpha_{k}$', 'Heading $\psi$', 'Rudder angle $\delta_r$', 'Rudder AOA $\alpha_{r}$' , 'Interpreter','latex')
% lgd = legend('Desired rudder angle $-\delta_{r_d}$','Desired heading', 'Keel AOA $\alpha_{k}$', 'Heading $\psi$', 'Rudder angle $\delta_r$', 'Roll angle $\phi$' ,'Rudder AOA $\alpha_{r}$' , 'Interpreter','latex')
% lgd = legend('Desired heading', 'Keel AOA $\alpha_{k}$', 'Heading $\psi$', 'Rudder angle $\delta_r$' ,'Rudder AOA $\alpha_{r}$' , 'Interpreter','latex')


% lgd = legend('Tail angle $U_w = 4 m/s$',  'Tail angle reference', 'Tail angle $U_w = 12 m/s$', 'Desired tail angle', 'Rudder angle $\delta_r$', 'Rudder AOA $\alpha_{r}$' , 'Interpreter','latex')
% lgd = legend('$M^{t}_{z}$ at $U_w = 4 m/s$',  '$M^{t}_{z}$ at $U_w = 12 m/s$', 'Interpreter','latex')
% lgd = legend('$M^{t}_{z}$ at $U_w = 15 m/s$', 'Interpreter','latex')
% lgd = legend('Heading $\psi$', 'Tail angle $\delta_t$', 'Desired tail angle $\delta_{t_{d}}$', 'Sail AOA $\alpha_s$', 'Interpreter','latex')
% lgd = legend('LOS $U_c = 0$m/s', 'Course switch $U_c = 0$m/s', 'LOS $U_c = 0.5$m/s', 'Course switch $U_c = 0.5$m/s', 'Interpreter','latex')
% lgd = legend('Keel AOA $\alpha_k$', 'Rudder AOA $\alpha_r$', 'Interpreter','latex')
% lgd = legend('LOS Keel AOA', 'LOS Rudder AOA', 'Course switch Keel AOA', 'Course switch Rudder AOA','Interpreter','latex')
% lgd = legend('Keel with comp.', 'Rudder with comp.', 'Keel no comp.', 'Rudder no comp.','Interpreter','latex')
% lgd = legend('Heading $\psi$', 'Rudder AOA with heading ref.', 'Couse $\chi$', 'Rudder AOA with course ref.','Interpreter','latex')
% lgd = legend('VMG with $\psi$', 'Velocity $U$ with $\psi$', 'VMG with $\chi$', 'Velocity $U$ with $\chi$','Interpreter','latex')
% lgd = legend('$\delta_t$', '$\delta_t$ simplified', '$\alpha_s$', '$\alpha_s$ simplified','Interpreter','latex')
lgd = legend('$\psi$','$\delta_t$', '$\delta_r$', '$\phi$', '$\alpha_s$','Interpreter','latex')
lgd.FontSize = 12;

% lgd = legend('Path angle $\pi_p$','Interpreter','latex')
% lgd.FontSize = 12;

% ylabel('Sail angle of attack $\alpha_{s}$ [deg]','Interpreter','latex', 'FontSize',12)
% ylabel('Angle [deg]','Interpreter','latex', 'FontSize',12)
% xlabel('time [s]', 'FontSize',12)

x0=20;
y0=20;
% the standard
% width=800;
% height=400

% for square:
width=600;
height=400

% width=800;
% height=300

% set(gcf,'position',[x0,y0,width,height])

% width=300;
% height=350
% 
% width=300;
% height=300

% for square:
% width=400;
% height=600

% set(gca,'DataAspectRatio',[1 1 1])
set(gcf,'position',[x0,y0,width,height])
axis on
grid on