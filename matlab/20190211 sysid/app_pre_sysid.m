%% cleaning
clear
close all
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');
set(groot, 'DefaultAxesFontSize', 14);
set(groot, 'DefaultLineLineWidth', 1.5);

%% model
% PWM Duty [0,1]Å@-> TransVelocity [mm/s]
K = 6465;
T1 = 0.264;
P1 = tf([K], [T1 1]);
step(P1); grid on;

clear;
N = 3;
M = 5;
R = randi([0 1], M,1);
nP = 2^M-1;
B = zeros(nP*30, 1);

% ê∂ê¨
for iP = 1:nP*30
    R = [xor(R(N),R(end)); R(1:end-1)];    
    B(iP) = R(1);
end

% åãâ 
figure();
subplot(2,1,1);
plot(-40:40,xcorr(B(1:1:end),40,'coeff'));
subplot(2,1,2);
plot(-40:40,xcorr(B(1:10:end),40,'coeff'));
