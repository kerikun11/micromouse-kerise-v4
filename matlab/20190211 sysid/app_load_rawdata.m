%% cleaning
clear
% close all
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');
set(groot, 'DefaultAxesFontSize', 14);
set(groot, 'DefaultLineLineWidth', 1.5);

%% load data
rawdata = dlmread('0.2-2.tab', '\t');
rawdata = rawdata';

%% Triming
% rawdata = rawdata(:, 1:500);
rawdata = rawdata(:, 1:1150);

%% extract
dt = 1e-3;
time = dt * 1:length(rawdata);
input = 1.0 * 1:length(rawdata) / 1024;
enc = rawdata(1:2, :);
gyro = rawdata(3, :);
accel = rawdata(4, :);
ang_accel = rawdata(5, :);
voltage = rawdata(6, :);

%% visualize
figure(1);
subNum = 5;
subplot(subNum, 1, 1); hold off;
plot(time, enc); grid on;
title('Encoder Position');
xlabel('Time [ms]');
ylabel('Position [mm]');
legend({'Encoder L', 'Encoder R'});

subplot(subNum, 1, 2); hold off;
plot(time, accel); grid on;
title('IMU Acceleration');
xlabel('Time [ms]');
ylabel('Acceleration [mm/s/s]');

subplot(subNum, 1, 3); hold off;
plot(time, gyro); grid on;
title('IMU gyro');
xlabel('Time [ms]');
ylabel('Angular Velocity [rad/s]');

subplot(subNum, 1, 4); hold off;
plot(time, ang_accel); grid on;
title('IMUs Angular Acceleration');
xlabel('Time [ms]');
ylabel('Angular Acceleration. [rad/s/s]');

subplot(subNum, 1, 5); hold off;
plot(time, voltage); grid on;
title('Battery Voltage');
xlabel('Time [ms]');
ylabel('Battery Voltage [V]');

%% filter design
diff_enc = (diff(enc(1, :))+diff(enc(2, :))) / dt / 2;
machine_rotation_radius = 15;
omega_enc = (diff(enc(2, :))-diff(enc(1, :))) / dt / 2 / (machine_rotation_radius*2);

% IIR Complementary Filtered
alpha = 0.75;
v_filtered = zeros(size(diff_enc));
v_filtered(1) = diff_enc(1);
omega_filtered = zeros(size(gyro));
omega_filtered(1) = gyro(1);
for i = 2 : length(diff_enc)
    v_filtered(i) = alpha * (v_filtered(i-1) + accel(i) * dt) + (1-alpha) * diff_enc(i);
end
k_enc  = 0.25;
k_gyro = 0.25;
for i = 2 : length(gyro)-1
%     omega_filtered(i) = alpha * (omega_filtered(i-1) + ang_accel(i) * dt) + (1-alpha) * gyro(i);
    omega_filtered(i) = (omega_filtered(i-1) + ang_accel(i) * dt)*(1-k_enc-k_gyro) + k_gyro * gyro(i) + k_enc*omega_enc(i);
end

%% Visualization of Velocity
figure(2);

subplot(2, 1, 1); hold off; clear legstr; legstr = {};
plot(time(1:end-1), diff_enc);
hold on; grid on;
legstr{length(legstr)+1} = 'Differential of Encoder';
plot(time(1:end-1), v_filtered);
legstr{length(legstr)+1} = 'Encoder + IMU-Accel, Complementary Filtered';
title('Translational Velocity');
xlabel('Time [ms]');
ylabel('Velocity [mm/s]');
legend(legstr, 'Location', 'NorthWest');

subplot(2, 1, 2); hold off; clear legstr; legstr = {};
plot(time, gyro); hold on;
legstr{length(legstr)+1} = 'Gyroscope';
% plot(time, omega_filtered); hold on;
% legstr{length(legstr)+1} = 'Encoder + Gyro + IMU-Accel, Complementary Filtered';
plot(time(1:end-1), omega_enc); hold on;
legstr{length(legstr)+1} = 'Differential of Encoder';
grid on;
title('Angular Velocity');
xlabel('Time [ms]');
ylabel('Angular Velocity [rad/s]');
legend(legstr, 'Location', 'NorthWest');
