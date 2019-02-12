%% cleaning
clear
% close all
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');
set(groot, 'DefaultAxesFontSize', 12);
set(groot, 'DefaultLineLineWidth', 1.5);
figindex = 1;
%% load data
rawdata = dlmread('1e-2.tab', '\t');
rawdata = rawdata';

%% extract
dt = 1e-3;
t = dt * 1:length(rawdata);
vs =    rawdata(1, :);
ws =    rawdata(2, :);
x =     rawdata(3, :);
y =     rawdata(4, :);
theta = rawdata(5, :);
accel = rawdata(6, :);
ang_accel = rawdata(7, :);
ada =   rawdata(8, :);
adv =   rawdata(9, :);
adx =   rawdata(10, :);
v =     rawdata(11, :);
w =     rawdata(12, :);
dv =    rawdata(13, :);
dw =    rawdata(14, :);

%% x, y, theta
figure(figindex); figindex = figindex + 1;
subplot(2, 1, 1); hold off;
plot(x, y); grid on;
title('$x-y$');
xlabel('$x$ [mm]');
ylabel('$y$ [mm]');
ylim([-5 5]);
axis equal;
subplot(2, 1, 2); hold off;
plot(t, theta); grid on;
title('Orientation');
xlabel('Time [ms]');
ylabel('$\theta$ [rad]');

%% velocity
figure(figindex); figindex = figindex + 1;
subplot(2, 1, 1); hold off;
plot(t, vs); hold on;
plot(t, v); hold on;
grid on;
title('$v$');
xlabel('Time $t$ [ms]');
ylabel('Traslational Velocity [mm/s]');
legend('Sensor', 'Reference');
subplot(2, 1, 2); hold off;
plot(t, ws); hold on;
plot(t, w); hold on;
grid on;
title('$\omega$');
xlabel('Time $t$ [ms]');
ylabel('Rotational Velocity [rad/s]');
legend('Sensor', 'Reference');

%% error
figure(figindex); figindex = figindex + 1;
subplot(3, 1, 1); hold off;
plot(t, adx-x); grid on;
title('$e_x$');
xlabel('Time $t$ [ms]');
ylabel('$e_x$ [mm]');
subplot(3, 1, 2); hold off;
plot(t, 0-y); grid on;
title('$e_y$');
xlabel('Time $t$ [ms]');
ylabel('$e_y$ [mm]');
subplot(3, 1, 3); hold off;
plot(t, 0-theta); grid on;
title('$e_\theta$');
xlabel('Time $t$ [ms]');
ylabel('$e_\theta$ [rad]');

%% feedforward term
figure(figindex); figindex = figindex + 1;
subplot(2, 1, 1); hold off;
plot(t, dv); hold on;
grid on;
title('$\dot{v}$');
xlabel('Time $t$ [ms]');
ylabel('Traslational Acceleration [mm/s/s]');
subplot(2, 1, 2); hold off;
plot(t, dw); hold on;
grid on;
title('$\dot{\omega}$');
xlabel('Time $t$ [ms]');
ylabel('Rotational Acceleration [rad/s/s]');
