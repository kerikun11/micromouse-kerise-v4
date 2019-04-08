%% Micromouse; Model Based Control
% Author Ryotaro Onuki
% Created_at 2019.02.16
%% cleaning
clear;
close all;
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');
set(groot, 'DefaultAxesFontSize', 14);
set(groot, 'DefaultLineLineWidth', 1.5);
figindex = 1;
%% System Identificated Models (1st-order)
% PWM Duty Average [-1,1]@-> Translational Velocity [mm/s]
K1_tra = 5789;
T1_tra = 0.2517;
% PWM Duty Diff [-2,2]@-> Rotational Velocity [rad/s]
K1_rot = 49.74;
T1_rot = 0.09089;
% Matrix Form
K1 = diag([K1_tra, K1_rot]);
T1 = diag([T1_tra, T1_rot]);

%% PID Controller Design
Kp = diag([0.002 0.04]);
Ki = diag([0.1 6.0]);
Kd = diag([0 0]);

%% Trajectory Tracker
zeta = 1.0;
omega_n = 5;
kx = omega_n * omega_n;
kdx = 2 * zeta * omega_n;
ky = kx;
kdy = kdx;

%% reference trajectory
dt = 0.001;
v_start = 600;
v_max = 1800;
v_diff = v_max - v_start;
traj_omega = 9000 / v_diff * 2;
t = 0:dt:2*pi/traj_omega;
dddx_rs = v_diff / 2 * cos(traj_omega*t);
ddx_rs = v_diff / 2 * traj_omega * sin(traj_omega*t);
dx_rs = v_start + v_diff / 2 * (1 - cos(traj_omega*t));
x_rs = v_start * t + v_diff / 2 * (t - 1/traj_omega*sin(traj_omega*t));
ddy_rs = 0 * t;
dy_rs = 0 * t;
y_rs = 0 * t;
%% visualization
%{
figure(figindex); figindex = figindex + 1;
ylabels= {'$j$ [mm/s/s/s]', '$a$ [mm/s/s]', '$v$ [mm/s]', '$x$ [mm]'};
titles= {'Jerk', 'Acceleration', 'Velocity', 'Position'};
xlabelstr = '$t$ [s]';
X_r = {dddx_rs, ddx_rs, dx_rs, x_rs};
for i = 1 : length(X_r)
    subplot(length(X_r), 1, i);
    hold off; plot(nan, nan); hold on; % claen
    ax = gca; ax.ColorOrderIndex = i;
    plot(t, X_r{i});
    grid on;
    title(titles(i));
    xlabel(xlabelstr);
    ylabel(ylabels(i));
end
%}

%% function
function [dv dw v w] = TrajTracker()

end
%% Motor Cotroller; FF + FB
function [u ds] = MotorController(dt, s, dr, r, dy, y)
    ff = (T1 * dr + r) / K1;
    fb = K_PID * [r-y; ir-iy, dr-dy];
end
