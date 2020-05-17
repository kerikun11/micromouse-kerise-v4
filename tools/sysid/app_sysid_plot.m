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

%% Serial input
%{
sl = seriallist;
portname = sl{2};
s = serial(portname, 'Baudrate', 2000000);
s.Timeout = 10;

date_time_str = datestr(datetime('now'), 'yymmdd-HHMMSS');
pathname = './data/';
filename = [date_time_str '.tab'];
[~, ~] = mkdir(pathname);
fileID = fopen([pathname filename], 'w');

fopen(s);
disp('now waiting for serial input...');

while 1
    idn = fscanf(s);
    s.Timeout = 0.1;
    if idn == ""; break; end
    fwrite(fileID, idn);
end

fclose(fileID);
fclose(s);
delete(s);
clear s;
fprintf("Saved Log File: %s\n", filename);
%}

%% Select a Log File
[filename, pathname] = uigetfile({'*'}, 'Select a Log File');
fprintf('Log File: %s\n', filename);

%% Load Data
rawdata = dlmread([pathname filename]);

%% Triming and Preprocess
rawdata = rawdata';
% rawdata = rawdata(:, 1:600);
% rawdata = rawdata(:, 1:1200);

%% extract data
dt = 1e-3;
time = dt * 1:length(rawdata);
input = 1.0 * 1:length(rawdata) / 1024;
enc = rawdata(1:2, :);
gyro = rawdata(3, :);
accel = rawdata(4, :);
ang_accel = rawdata(5, :);
voltage = rawdata(6, :);

%% visualize
figure(figindex); figindex = figindex + 1;
subNum = 5; 
subIndex = 1; % for subplot increment

subplot(subNum, 1, subIndex); subIndex = subIndex + 1; hold off;
plot(time, enc); grid on;
title('Encoder Position');
xlabel('Time [ms]');
ylabel('Position [mm]');
legend({'Encoder L', 'Encoder R'});

subplot(subNum, 1, subIndex); subIndex = subIndex + 1; hold off;
plot(time, accel); grid on;
title('IMU Acceleration');
xlabel('Time [ms]');
ylabel('Acceleration [mm/s/s]');

subplot(subNum, 1, subIndex); subIndex = subIndex + 1; hold off;
plot(time, gyro); grid on;
title('IMU gyro');
xlabel('Time [ms]');
ylabel('Angular Velocity [rad/s]');

subplot(subNum, 1, subIndex); subIndex = subIndex + 1; hold off;
plot(time, ang_accel); grid on;
title('IMUs Angular Acceleration');
xlabel('Time [ms]');
ylabel('Angular Acceleration. [rad/s/s]');

subplot(subNum, 1, subIndex); subIndex = subIndex + 1; hold off;
plot(time, voltage); grid on;
title('Battery Voltage');
xlabel('Time [ms]');
ylabel('Battery Voltage [V]');

%% filter design
diff_enc = (diff(enc(1, :)) + diff(enc(2, :))) / dt / 2;
machine_rotation_radius = 17;
omega_enc = (diff(enc(2, :)) - diff(enc(1, :))) / dt / (machine_rotation_radius * 2);

% IIR Complementary Filtered
alpha = 0.75;
v_filtered = zeros(size(diff_enc));
v_filtered(1) = diff_enc(1);
omega_filtered = zeros(size(gyro));
omega_filtered(1) = gyro(1);

for i = 2:length(diff_enc)
    v_filtered(i) = alpha * (v_filtered(i - 1) + accel(i) * dt) + (1 - alpha) * diff_enc(i);
end

k_enc = 1;
k_gyro = 0;

for i = 2:length(gyro) - 1
    %     omega_filtered(i) = alpha * (omega_filtered(i-1) + ang_accel(i) * dt) + (1-alpha) * gyro(i);
    omega_filtered(i) = (omega_filtered(i - 1) + ang_accel(i) * dt) * (1 - k_enc - k_gyro) + k_gyro * gyro(i) + k_enc * omega_enc(i);
end

%% Visualization of Velocity
figure(figindex); figindex = figindex + 1;

subplot(2, 1, 1); hold off; clear legstr; legstr = {};
plot(time(1:end - 1), diff_enc);
hold on; grid on;
legstr{length(legstr) + 1} = 'Differential of Encoder';
plot(time(1:end - 1), v_filtered);
legstr{length(legstr) + 1} = 'Encoder + IMU-Accel, Complementary Filtered';
title('Translational Velocity');
xlabel('Time [ms]');
ylabel('Velocity [mm/s]');
legend(legstr, 'Location', 'SouthWest');

subplot(2, 1, 2); hold off; clear legstr; legstr = {};
plot(time, gyro); hold on;
legstr{length(legstr) + 1} = 'Gyroscope';
plot(time(1:end - 1), omega_enc); hold on;
legstr{length(legstr) + 1} = 'Differential of Encoder';
grid on;
title('Angular Velocity');
xlabel('Time [ms]');
ylabel('Angular Velocity [rad/s]');
legend(legstr, 'Location', 'SouthWest');
