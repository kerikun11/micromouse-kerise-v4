%% Micromouse; Model Based Control
% Author Ryotaro Onuki
% Created_at 2019.02.16
%% cleaning
clear
% close all
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');
set(groot, 'DefaultAxesFontSize', 14);
set(groot, 'DefaultLineLineWidth', 1.5);
figindex = 1;
%% System Identificated Models (1st-order)
% PWM Duty Average [-1,1]　-> Translational Velocity [mm/s]
K1_tra = 6465;
T1_tra = 0.264;
% P1_tra = tf([K1_tra], [T1_tra 1]);
% PWM Duty Diff [-2,2]　-> Rotational Velocity [rad/s]
K1_rot = 54.96;
T1_rot = 0.08372;
% P1_rot = tf([K1_rot], [T1_rot 1]);

%% PID Controller Design
% Kpid_tra = [0.002, 0.1, 0];
% Kpid_rot = [0.008, 2.0, 0];
Kp_tra = 0.002;
Ki_tra = 0.1;
Kd_tra = 0;
Kp_rot = 0.002;
Ki_rot = 2.0;
Kd_rot = 0;

zeta = 1.0;
omega_n = 0.1;
K_x = omega_n * omega_n;
K_dx = 2 * zeta * omega_n;
zeta = 1.0;
omega_n = 0.1;
K_y = omega_n * omega_n;
K_dy = 2 * zeta * omega_n;
% PWM Duty Average [-1,1]　-> Translational Velocity [mm/s]
K1_tra = 6465;
T1_tra = 0.264;
P1_tra = tf([K1_tra], [T1_tra 1]);
% PWM Duty Diff [-2,2]　-> Rotational Velocity [rad/s]
K1_rot = 54.96;
T1_rot = 0.08372;
P1_rot = tf([K1_rot], [T1_rot 1]);

%% trajectory
dt = 0.001;
v_start = 600;
v_max = 1800 - v_start;
traj_omega = 9000 / v_max * 2;
t = 0:dt:2*pi/traj_omega;
ddx_rs = v_max / 2 * traj_omega * sin(traj_omega*t);
dx_rs = v_start + v_max / 2 * (1 - cos(traj_omega*t));
x_rs = v_start * t + v_max / 2 * (t - 1/traj_omega*sin(traj_omega*t));
ddy_rs = 0 * t;
dy_rs = 0 * t;
y_rs = 0 * t;
%% visualization
% %{
figure(figindex); figindex = figindex + 1;
ylabels= {'$a$ [mm/s/s]', '$v$ [mm/s]', '$x$ [mm]'};
titles= {'Acceleration', 'Velocity', 'Position'};
xlabelstr = '$t$ [s]';
X_r = {ddx_rs, dx_rs, x_rs};
for i = 1 : 3
    subplot(3, 1, i);
    hold off; plot(nan, nan); % clean
    hold on;
    ax = gca; ax.ColorOrderIndex = i;
    plot(t, X_r{i});
    grid on;
    xlabel(xlabelstr);
    ylabel(ylabels(i));
    title(titles(i));
end
%}
