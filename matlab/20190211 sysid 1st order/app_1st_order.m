%% cleaning
clear
close all
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');
set(groot, 'DefaultAxesFontSize', 14);
set(groot, 'DefaultLineLineWidth', 1.5);

%% model
% PWM Duty Average [-1,1]@-> Translational Velocity [mm/s]
K1_tra = 6465;
T1_tra = 0.264;
P1_tra = tf([K1_tra], [T1_tra 1]);
% PWM Duty Diff [-2,2]@-> Rotational Velocity [rad/s]
K1_rot = 54.96;
T1_rot = 0.08372;
P1_rot = tf([K1_rot], [T1_rot 1]);

%% visualiztion
%{
subplot(2, 2, 1);
step(P1_tra); grid on;
subplot(2, 2, 2);
bode(P1_tra); grid on;
subplot(2, 2, 3);
step(P1_rot); grid on;
subplot(2, 2, 4);
bode(P1_rot); grid on;
%}
%% pidTuner
% pidTuner(P1_tra);
% pidTuner(P1_rot);

%% controller
Kp_tra = 0.0005033;
Ki_tra = 0.0036305;
Kd_tra = 0.0;

Kp_rot = 0.018919;
Ki_rot = 0.321020;
Kd_rot = 0.0;
