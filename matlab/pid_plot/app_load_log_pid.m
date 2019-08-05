%% Micromouse; Log File Loader and Visualizer
% Author: Ryotaro Onuki
% Created_at: 2019.03.24

%% cleaning
clear;
% close all;
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');
set(groot, 'DefaultAxesFontSize', 14);
set(groot, 'DefaultLineLineWidth', 1.5);
figindex = 1;

%% Select a Log File
[filename, pathname] = uigetfile({'*'}, 'Select a Log File');
fprintf('Log File: %s\n', filename);

%% Load Data
rawdata = dlmread([pathname filename]);

%% Triming and Preprocess
rawdata = rawdata';
% rawdata = rawdata(:, 1:600);
% rawdata = rawdata(:, 1:1150);

%% extract
dt = 1e-3;
time = dt * 1:length(rawdata);
v_tra = rawdata(1:2, :);
a_tra = rawdata(3:4, :);
u_tra = rawdata(5:9, :);
v_rot = rawdata(10:11, :);
a_rot = rawdata(12:13, :);
u_rot = rawdata(14:18, :);

% log format
%{
        sc.ref_v.tra,
        sc.est_v.tra,
        sc.ref_a.tra,
        sc.est_a.tra,
        sc.ff.tra,
        sc.fb.tra,
        sc.fbp.tra,
        sc.fbi.tra,
        sc.pwm_value.tra,
        sc.ref_v.rot,
        sc.est_v.rot,
        sc.ref_a.rot,
        sc.est_a.rot,
        sc.ff.rot,
        sc.fb.rot,
        sc.fbp.rot,
        sc.fbi.rot,
        sc.pwm_value.rot,
%}

%% Visualization
%% Translational Velocity
figure(figindex); figindex = figindex + 1;

subplot(3, 1, 1); hold off;
plot(time, v_tra); grid on;
title('Translational Velocity');
xlabel('Time [ms]');
ylabel('Velocity [mm/s]');
legend({'Reference', 'Estimation'});

subplot(3, 1, 2); hold off;
plot(time, a_tra); grid on;
title('Translational Acceleration');
xlabel('Time [ms]');
ylabel('Acceleration [mm/s/s]');
legend({'Reference', 'Estimation'});

subplot(3, 1, 3); hold off;
plot(time, u_tra); grid on;
title('Translational Control Input');
xlabel('Time [ms]');
ylabel('Control Input');
legend({'FF', 'FB', 'FB p', 'FB i', 'FF+FB'});

%% Rotational Velocity
figure(figindex); figindex = figindex + 1;

subplot(3, 1, 1); hold off;
plot(time, v_rot); grid on;
title('Rotational Velocity');
xlabel('Time [ms]');
ylabel('Angular Velocity [rad/s]');
legend({'Reference', 'Estimation'});

subplot(3, 1, 2); hold off;
plot(time, a_rot); grid on;
title('Rotational Acceleration');
xlabel('Time [ms]');
ylabel('Angular Acceleration [rad/s/s]');
legend({'Reference', 'Estimation'});

subplot(3, 1, 3); hold off;
plot(time, u_rot); grid on;
title('Rotational Control Input');
xlabel('Time [ms]');
ylabel('Control Input');
legend({'FF', 'FB', 'FB p', 'FB i', 'FF+FB'});
