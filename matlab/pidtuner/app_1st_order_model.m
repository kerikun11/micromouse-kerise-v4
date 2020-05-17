%% Micromouse; Model Based Control
% Author Ryotaro Onuki
% Created_at 2019.02.16
%% cleaning
clear
% close all
figindex = 1;
%% System Identificated Models (1st-order)
% PWM Duty Average [-1,1]@-> Translational Velocity [mm/s]
K1_tra = 5833;
T1_tra = 0.3694;
P1_tra = tf(K1_tra, [T1_tra 1]);
% PWM Duty Difference [-2,2]@-> Rotational Velocity [rad/s]
K1_rot = 66.72;
T1_rot = 0.1499;
P1_rot = tf(K1_rot, [T1_rot 1]);
% Combi
P1 = [P1_tra 0; 0 P1_rot];
%% visualiztion
% %{
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
% %{
figure(figindex); figindex = figindex + 1;
subplot(2, 2, 1); step(closed_tra); grid on;
subplot(2, 2, 2); bode(closed_tra); grid on;
subplot(2, 2, 3); step(closed_rot); grid on;
subplot(2, 2, 4); bode(closed_rot); grid on;
%}
%% PID Tuner
% pidTuner(P1_tra)
Ts = 0.001;
% C_tra = pid(0.0006, 0.1, 0, 0, Ts);
pidTuner(c2d(P1_tra, Ts));

