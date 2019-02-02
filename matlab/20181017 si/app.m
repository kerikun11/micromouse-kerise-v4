%% load data
% rawdata = csvread('lamp500.csv');
% rawdata = csvread('accel.csv');
rawdata = dlmread('100.tab', '\t');
rawdata = rawdata';
rawdata = rawdata(:, 1:1600);

%% extract
dt = 1e-3;
time = dt * 1:length(rawdata);
input = 1.0 * 1:length(rawdata) / 1024;
enc = rawdata(1:2, :);
omega = rawdata(3, :);
accel = rawdata(4, :);
domega = rawdata(5, :);
voltage = rawdata(6, :);
voltage = filloutliers(voltage, 'linear');

%% visualize
LineWidth = 2;
FontSize = 12;
subNum = 5;
figure(1);
subplot(subNum, 1, 1); hold off;
plot(time, enc, 'LineWidth', LineWidth); grid on;
legend({'Encoder L', 'Encoder R'}, 'FontSize', FontSize);
title('Encoder Position', 'FontSize', FontSize);
xlabel('time [ms]', 'FontSize', FontSize);
ylabel('position [mm]', 'FontSize', FontSize);

subplot(subNum, 1, 2); hold off;
plot(time, accel, 'LineWidth', LineWidth); grid on;
legend({'acceleration'}, 'FontSize', FontSize);
title('Acceleration', 'FontSize', FontSize);
xlabel('time [ms]', 'FontSize', FontSize);
ylabel('acceleration [mm/s/s]', 'FontSize', FontSize);

subplot(subNum, 1, 3); hold off;
plot(time, omega, 'LineWidth', LineWidth); grid on;
title('angular velocity', 'FontSize', FontSize);
xlabel('time [ms]', 'FontSize', FontSize);
ylabel('angular velocity [rad/s]', 'FontSize', FontSize);

subplot(subNum, 1, 4); hold off;
plot(time, domega, 'LineWidth', LineWidth); grid on;
title('Angular Acceleration', 'FontSize', FontSize);
xlabel('time [ms]', 'FontSize', FontSize);
ylabel('angular accel. [rad/s/s]', 'FontSize', FontSize);

subplot(subNum, 1, 5); hold off;
plot(time, voltage, 'LineWidth', LineWidth); grid on;
title('Battery Voltage [V]', 'FontSize', FontSize);
xlabel('time [ms]', 'FontSize', FontSize);
ylabel('Battery Voltage [V]', 'FontSize', FontSize);

%% filter design
diff_enc = (diff(enc(1, :))+diff(enc(2, :))) / dt / 2;
int_accel = cumtrapz(accel) * dt;
int_domega= cumtrapz(domega) * dt;
diff_enc_mean = movmean(diff_enc, [16 0]);
omega_mean = movmean(omega, [16 0]);

% IIR Complementary Filtered
alpha = 0.75;
v_filtered = zeros(size(diff_enc));
v_filtered(1) = diff_enc(1);
omega_filtered = zeros(size(omega));
omega_filtered(1) = omega(1);
for i = 2 : length(diff_enc)
    v_filtered(i) = alpha * (v_filtered(i-1) + accel(i) * dt) + (1-alpha) * diff_enc(i);
end
for i = 2 : length(omega)
    omega_filtered(i) = alpha * (omega_filtered(i-1) + domega(i) * dt) + (1-alpha) * omega(i);
end

figure(2);
subplot(2, 1, 1); hold off;
plot(time(1:end-1), diff_enc, time, int_accel, time(1:end-1), diff_enc_mean, time(1:end-1), v_filtered, 'LineWidth', LineWidth); grid on;
legend({'differential of encoder', 'integral of accel', 'Meadian Fileted', 'Complementary Filtered'}, 'FontSize', FontSize, 'Location', 'SouthEast');
title('comparison of velocity', 'FontSize', FontSize);
xlabel('time [ms]', 'FontSize', FontSize);
ylabel('velocity [mm/s/s]', 'FontSize', FontSize);

subplot(2, 1, 2); hold off;
plot(time, omega, time, int_domega, time, omega_mean, time, omega_filtered, 'LineWidth', LineWidth); grid on;
legend({'angular velocity', 'integral of angular acceleration', 'Meadian Fileted', 'Complementary Filtered'}, 'FontSize', FontSize, 'Location', 'SouthEast');
title('comparison of angular velocity', 'FontSize', FontSize);
xlabel('time [ms]', 'FontSize', FontSize);
ylabel('angular velocity [rad/s]', 'FontSize', FontSize);
