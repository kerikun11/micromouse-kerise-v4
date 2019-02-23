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
P1_tra = tf([K1_tra], [T1_tra 1]);
% PWM Duty Diff [-2,2]　-> Rotational Velocity [rad/s]
K1_rot = 54.96;
T1_rot = 0.08372;
P1_rot = tf([K1_rot], [T1_rot 1]);
% Combi
P1 = [P1_tra 0; 0 P1_rot];
%% visualiztion
%{
figure(figindex); figindex = figindex + 1;
subplot(2, 2, 1); step(P1_tra); grid on;
subplot(2, 2, 2); bode(P1_tra); grid on;
subplot(2, 2, 3); step(P1_rot); grid on;
subplot(2, 2, 4); bode(P1_rot); grid on;
%}
%% PID Controller Design
C_tra = pid(0.002, 0.1, 0);
C_rot = pid(0.008, 2.0, 0);
Ctrlr = [C_tra 0; 0 C_rot];
closed_loop = feedback(P1*Ctrlr, eye(2));
closed_tra = feedback(P1_tra * C_tra, 1);
closed_rot = feedback(P1_rot * C_rot, 1);
%% visualization
%{
figure(figindex); figindex = figindex + 1;
subplot(2, 2, 1); step(closed_tra); grid on;
subplot(2, 2, 2); bode(closed_tra); grid on;
subplot(2, 2, 3); step(closed_rot); grid on;
subplot(2, 2, 4); bode(closed_rot); grid on;
%}
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
%% simulation
ddx = 0;
dx = v_start;
x = 0;
ddy = 0;
dy = 0;
y = 0;
xi = v_start;
theta = 0;
e_v_int = 0;
e_w_int = 0;

result_x = zeros(length(t), 1);
result_y = zeros(length(t), 1);
clear t

[t, y] = ode45(@(t, y) sim(...
v_max / 2 * traj_omega * sin(traj_omega*t),...
v_start + v_max / 2 * (1 - cos(traj_omega*t)),...
v_start * t + v_max / 2 * (t - 1/traj_omega*sin(traj_omega*t)),...
0 * t,...
0 * t,...
0 * t), [0 10], zeros(6, 1));

function [dx, dy, dth, dxi, de_v_int, de_w_int] = sim(x_r, dx_r, ddx_r, y_r, dy_r, ddy_r)
    [u_x, u_y, du_x, du_y] = LFBC(ddx_r, ddx, dx_r, x_r, dx, x, ddy_r, ddy, dy_r, y_r, dy, y);
    [u_v, u_w, du_v, du_w, dxi] = Compensator(u_x, u_y, du_x, du_y, xi, theta);
    [u_m, de_v_int, de_w_int] = MotorController(u_v, u_w, du_v, du_w, x, y, dx, dy, e_v_int, e_w_int);
    u_tra = u_m(1); u_rot = u_m(2);
    [v, w] = P_motor(u_tra, u_rot);
    [dx, dy, dth] = P_theta(v, w, theta);
end
%% Traj FBC design
function [u_x, u_y, du_x, du_y] = LFBC(ddx_r, ddx, dx_r, x_r, dx, x, ddy_r, ddy, dy_r, y_r, dy, y)
    zeta = 1.0;
    omega_n = 0.1;
    K_x = omega_n * omega_n;
    K_dx = 2 * zeta * omega_n;
    u_x = ddx_r + K_dx * (dx_r - dx) + K_x * (x_r - x);
    du_x = 0 + K_dx * (ddx_r - ddx) + K_x * (dx_r - dx);
    zeta = 1.0;
    omega_n = 0.1;
    K_y = omega_n * omega_n;
    K_dy = 2 * zeta * omega_n;
    u_y = ddy_r + K_dy * (dy_r - dy) + K_y * (y_r - y);
    du_y = 0 + K_dy * (ddy_r - ddy) + K_y * (dy_r - dy);
end
%% Exact Linearizing Compensator
function [u_v, u_w, du_v, du_w, dxi] = Compensator(u_x, u_y, du_x, du_y, xi, theta)
    dxi = u_x * cos(theta) + u_y * sin(theta);
    u_v = xi;
    u_w = ( u_y * cos(theta) - u_x * sin(theta) ) / xi;
    du_v = dxi;
    du_w = -4*dxi*u_w/xi - sin(theta)/xi*du_x + cos(theta)/xi*du_y;
end
%% $u_v, u_\omega $ から $\dot{x}, \dot{y}, \dot{\theta}$ の運動学
function [dx, dy, dth] = P_theta(uv, uw, theta)
    dx = uv * cos(theta);
    dy = uw * cos(theta);
    dth = uw;
end
%% Motor And Inertia Model
function [v_y, w_y] = P_motor(u_tra, u_rot)
    % PWM Duty Average [-1,1]　-> Translational Velocity [mm/s]
    K1_tra = 6465;
    T1_tra = 0.264;
    P1_tra = tf([K1_tra], [T1_tra 1]);
    % PWM Duty Diff [-2,2]　-> Rotational Velocity [rad/s]
    K1_rot = 54.96;
    T1_rot = 0.08372;
    P1_rot = tf([K1_rot], [T1_rot 1]);
    % output
    v_y = P1_tra * u_tra;
    w_y = P1_rot * u_rot;
end
%% PI Controller
function [u_m, de_v_int, de_w_int] = MotorController(u_v, u_w, du_v, du_w, x, y, dx, dy, e_v_int, e_w_int)
    r = [u_v; u_w];
    dr = [du_v; du_w];
    y = [x; y];
    dy = [dx; dy];
    e_int = [e_v_int; e_w_int];
    T1 = diag([0.264, 0.08372]);
    K1 = diag([6465, 54.96]);
    u_ff = K1 \ (T1 * dr + r);
    Kp = diag([0.002, 0.008]);
    Ki = diag([0.1, 2.0]);
    Kd = diag([0, 0]);
    u_fb = Kp * (r-y) + Ki * e_int + Kd * (dr-dy);
    u_m = u_ff + u_fb;
    de_int = r - y;
    de_v_int = de_int(1);
    de_w_int = de_int(2);
end
% function [u_tra, u_rot] = PIDC(r_tra, y_tra, r_rot, y_rot)
%     % Translational Velocity [mm/s] -> PWM Duty Average [-1,1]
%     C_tra = pid(0.002, 0.1, 0);
%     u_tra = C_tra * (r_tra - y_tra);
%     % Rotational Velocity [rad/s] -> PWM Duty Diff [-2,2]
%     C_rot = pid(0.008, 2.0, 0);
%     u_rot = C_rot * (r_rot - y_rot);
% end
