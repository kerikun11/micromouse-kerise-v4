%% Micromouse; Turn Trajectory Generator
% Author Ryotaro Onuki
% Created_at 2017.12.13
%% cleaning
clear;
% close all;
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');
% set(groot, 'DefaultAxesFontSize', 16);
set(groot, 'DefaultLineLineWidth', 1.5);
figindex = 1;

%% ���̑傫�����` [mm]
seg_full = 90;
seg_half = seg_full / 2;

%% �ݒ���
% �_��̊Ԋu [mm]
dx = 1.0;
% �p���x�Ɗp�����x��ݒ�
omega_dot = 90 * pi;
omega_max = 4 * pi;

%% �p�^�[����I��
% adv_straight: �J�[�u�O�̒��������̒��� [mm]
% pos_start: �n�_�ʒu [x; y; theta]
% pos_start: �I�_�ʒu [x; y; theta]
switch 0
    case 0 % #0 search 90
        adv_straight = 1;
        pos_start = [0; 0; 0];
        pos_end = [seg_half; seg_half; pi/2];
        omega_dot = 240 * pi;
        omega_max = 3 * pi;
    case 1 % #1 �ŒZ 45
        adv_straight = 10;
        pos_start = [0; 0; 0];
        pos_end = [seg_full; seg_half; pi/4];
    case 2 % #2 �ŒZ 90
        adv_straight = 25;
        pos_start = [0; 0; 0];
        pos_end = [seg_full; seg_full; pi/2];
    case 3 % #3 �ŒZ 135
        adv_straight = 20;
        pos_start = [0; 0; 0];
        pos_end = [seg_half; seg_full; 3/4*pi];
    case 4 % #4 �ŒZ 180
        adv_straight = 26;
        pos_start = [0; 0; 0];
        pos_end = [0; seg_full; pi];
    case 5 % #5 �ŒZ �΂� 90
        adv_straight = 20;
        pos_start = [0; 0; pi/4];
        pos_end = [0; seg_full; 3/4*pi];
        omega_dot = 240 * pi;
    case 6 % #6 �ŒZ �����O �΂� 90
        adv_straight = 0;
        pos_start = [0; 0; pi/4];
        pos_end = [0; seg_full * 2; 3/4*pi];
    case 7 % #7 �ŒZ �����O 135
        adv_straight = 0;
        pos_start = [0; 0; 0];
        pos_end = [seg_half; seg_full * 2; 3/4*pi];
    case 8 % #8 �ŒZ �����O 180
        adv_straight = 0;
        pos_start = [0; 0; 0];
        pos_end = [seg_half; seg_full * 2; pi];
    case 9 % #8 �ŒZ �΂� 180
        adv_straight = 0;
        pos_start = [0; 0; pi/4];
        pos_end = [-seg_half*3; seg_half*3; pi*5/4];
    case 10 % #8 �ŒZ ����΂� 180
        adv_straight = 60;
        pos_start = [0; 0; pi/4];
        pos_end = [-seg_half*2; seg_half*2; pi*5/4];
    case 11 % #2' �ŒZ ����� 90
        adv_straight = 5;
        pos_start = [0; 0; 0];
        pos_end = [seg_half; seg_half; pi/2];
end

%% �K�v���̎Z�o
% �X�^�[�g�|�W�V�����̓����ϊ��s��𐶐�
Rot_start = [cos(pos_start(3)),-sin(pos_start(3)),0;sin(pos_start(3)),cos(pos_start(3)),0;0,0,1];
% �I�t�Z�b�g���������ڕW�ʒu���Z�o
pos_target = Rot_start \ (pos_end - pos_start) - [adv_straight; 0; 0];
% �����g������1�����̎��Ԃ��Z�o
T = omega_max / omega_dot * pi;
[t, theta] = ode45(@(t, theta) omega_max * sin(pi*t/T)^2, [0 T], 0); %#ok<ASGLU>

%% �ϕ����ʂ��ڕW�p�x�𒴂��Ă��邩�ǂ����ŏ�������
if pos_target(3) < theta(end)
    %% �ϕ����ʂ��ڕW�p�x�𒴂��Ă���ꍇ
    % �I�_�p�x���ڕW�p�x�ɂȂ�悤�ȃX�P�[�����O�W��
    theta_gain = sqrt(pos_target(3) / theta(end));
    % ���Ԃ��X�P�[�����O
    T = T * theta_gain;
    % ���l�ϕ��ŋO�Ղ𐶐�
    [t, x] = ode45(@(t, x) cos(theta_gain * ((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi))), [0 T], 0);
    [t, y] = ode45(@(t, x) sin(theta_gain * ((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi))), [0 T], 0);
    % �I�_�ʒu���ڕW�ʒu�ɂȂ�悤�ɕ��i���x���Z�o
    syms v;
    v = double(solve((pos_target(2)-v*y(end))*cos(pos_target(3))==(pos_target(1)-v*x(end))*sin(pos_target(3)), v));
    %% �O���̕\���C����
    dt = dx/v;
    x_end = x(end)*v; y_end = y(end)*v;
    % �p���x�̔z��𐶐�
    figure(1);
    omega = omega_max * sin(pi*[0:dt:T]/T).^2;
    subplot(6, 1, 1); hold off;
    plot(0:dt:T, omega, '.', 'MarkerSize', 12); grid on;
    % �p�x�̔z��𐶐�
    [t, theta] = ode45(@(t, theta) theta_gain * omega_max * sin(pi*t/T)^2, [0:dt:T], 0);
    subplot(6, 1, 2); hold off;
    plot(t, theta, '.', 'MarkerSize', 12); grid on;
    % �ʒu�̔z��𐶐�
    [t, x] = ode45(@(t, x) v * cos(theta_gain * ((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi))), [0:dt:T], 0);
    [t, y] = ode45(@(t, y) v * sin(theta_gain * ((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi))), [0:dt:T], 0);
    subplot(6, 1, [3 6]); hold off;
    plot(x, y, '.', 'MarkerSize', 12); grid on;
    % �o�̓f�[�^�𐶐�
    pos = [x, y, theta];
    pos = [pos; x_end, y_end, pos_target(3)];
else
    %% �ϕ����ʂ��ڕW�p�x�ɖ����Ȃ��ꍇ
    % �p���x�����̎��Ԃ�݂��ĖڕW�p�x�ɂȂ�悤�ɒ��߂���
    % �p���x ��������
    T1 = T / 2;
    % �p���x ��莞��
    T2 = T1 + (pos_target(3) - theta(end)) / omega_max;
    % �p���x ��������
    T3 = T2 + T / 2;
    % ���l�ϕ��ŋO�Ղ𐶐�
    [t, x1] = ode45(@(t, x1) cos((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi)), [0 T/2], 0);
    [t, x2] = ode45(@(t, x2) cos((omega_max*T/2)/2 - (T*omega_max*sin((2*pi*T/2)/T))/(4*pi) + omega_max*(t-T1)), [T1 T2], x1(end));
    [t, x3] = ode45(@(t, x3) cos(omega_max*(T2-T1) + (omega_max*(t-T2+T1))/2 - (T*omega_max*sin((2*pi*(t-T2+T1))/T))/(4*pi)), [T2 T3], x2(end));
    [t, y1] = ode45(@(t, y1) sin((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi)), [0 T/2], 0);
    [t, y2] = ode45(@(t, y2) sin((omega_max*T/2)/2 - (T*omega_max*sin((2*pi*T/2)/T))/(4*pi) + omega_max*(t-T1)), [T1 T2], y1(end));
    [t, y3] = ode45(@(t, y3) sin(omega_max*(T2-T1) + (omega_max*(t-T2+T1))/2 - (T*omega_max*sin((2*pi*(t-T2+T1))/T))/(4*pi)), [T2 T3], y2(end));
    % �I�_�ʒu���ڕW�ʒu�ɂȂ�悤�ɕ��i���x���Z�o
    syms v;
    v = double(solve((pos_target(2)-v*y3(end))*cos(pos_target(3))==(pos_target(1)-v*x3(end))*sin(pos_target(3)), v));
    %% �O���̕\���C����
    dt = dx/v;
    t1 = 0:dt:T1;
    t2 = t1(end):dt:T2;
    t3 = t2(end):dt:T3;
    x1_end = x1(end)*v; x2_end = x2(end)*v; x3_end = x3(end)*v;
    y1_end = x1(end)*v; y2_end = y2(end)*v; y3_end = y3(end)*v;
    omega = [omega_max * sin(pi*t1/T).^2, omega_max+t2*0, omega_max * sin(pi*(t3-T2+T1)/T).^2];
    % �p���x�̔z��𐶐�
    figure(1);
    subplot(6, 1, 1); hold off;
    plot(t1, omega(1:length(t1)), '.', 'MarkerSize', 12); grid on; hold on;
    plot(t2, omega(length(t1)+1:length(t1)+length(t2)), '.', 'MarkerSize', 12); grid on; hold on;
    plot(t3, omega(length(t1)+length(t2)+1:length(t1)+length(t2)+length(t3)), '.', 'MarkerSize', 12); grid on; hold on;
    % �p�x�̔z��𐶐�
    subplot(6, 1, 2); hold off;
    theta1 = (omega_max*t1)/2 - (T*omega_max*sin((2*pi*t1)/T))/(4*pi);
    theta2 = (omega_max*T/2)/2 - (T*omega_max*sin((2*pi*T/2)/T))/(4*pi) + omega_max*(t2-T1);
    theta3 = omega_max*(T2-T1) + (omega_max*(t3-T2+T1))/2 - (T*omega_max*sin((2*pi*(t3-T2+T1))/T))/(4*pi);
    plot(t1, theta1, '.', 'MarkerSize', 12); grid on; hold on;
    plot(t2, theta2, '.', 'MarkerSize', 12); grid on; hold on;
    plot(t3, theta3, '.', 'MarkerSize', 12); grid on; hold on;
    % �ʒu�̔z��𐶐�
    [t, x1] = ode45(@(t, x1) v*cos((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi)), t1, 0);
    [t, x2] = ode45(@(t, x2) v*cos((omega_max*T/2)/2 - (T*omega_max*sin((2*pi*T/2)/T))/(4*pi) + omega_max*(t-T1)), t2, x1(end));
    [t, x3] = ode45(@(t, x3) v*cos(omega_max*(T2-T1) + (omega_max*(t-T2+T1))/2 - (T*omega_max*sin((2*pi*(t-T2+T1))/T))/(4*pi)), t3, x2(end));
    [t, y1] = ode45(@(t, y1) v*sin((omega_max*t)/2 - (T*omega_max*sin((2*pi*t)/T))/(4*pi)), t1, 0);
    [t, y2] = ode45(@(t, y2) v*sin((omega_max*T/2)/2 - (T*omega_max*sin((2*pi*T/2)/T))/(4*pi) + omega_max*(t-T1)), t2, y1(end));
    [t, y3] = ode45(@(t, y3) v*sin(omega_max*(T2-T1) + (omega_max*(t-T2+T1))/2 - (T*omega_max*sin((2*pi*(t-T2+T1))/T))/(4*pi)), t3, y2(end));
    subplot(6, 1, [3 6]); hold off;
    plot(x1, y1, '.', 'MarkerSize', 12); hold on; grid on;
    plot(x2, y2, '.', 'MarkerSize', 12); hold on; grid on;
    plot(x3, y3, '.', 'MarkerSize', 12); hold on; grid on;

    % �o�̓f�[�^�𐶐�
    pos = [x1, y1, theta1'; x2(2:end), y2(2:end), theta2(2:end)'; x3(2:end), y3(2:end), theta3(2:end)'];
    pos = [pos; x3_end, y3_end, omega_max*(T2-T1) + (omega_max*(T3-T2+T1))/2 - (T*omega_max*sin((2*pi*(T3-T2+T1))/T))/(4*pi)];
end

%% �o�͏��
format long;
% ���i���x
velocity = v;
% ���߂��O�Ղ̔z��̒���
length = size(pos, 1);
% �J�[�u�I������I�_�ʒu�܂ł̒��������̒������Z�o
extra_straight = max([(pos_target(2)-pos(end, 2)) / sin(pos_target(3)),(pos_target(1)-pos(end, 1)) / cos(pos_target(3))]);

%% ��Ő��������O���t(�J�[�u�̂�)�𑕏�
subplot(6,1,1);
title(sprintf('$ \\dot{\\omega}_{max}: %.0f\\pi,\\ \\omega_{max}: %.0f\\pi $', omega_dot/pi, omega_max/pi));
xlabel('$t$');
ylabel('$\omega$');
xlim([0, dt*length]);
subplot(6,1,2);
title(sprintf('$ \\theta_{end}: %.2f\\pi $', pos_target(3)/pi));
xlabel('$t$');
ylabel('$\theta$');
xlim([0, dt*length]);
subplot(6,1,[3 6]);
title(sprintf('$ v_{max}: %.3f $', v));
xlabel('$x$');
ylabel('$y$');
axis equal;
xlim([min(pos(:,1)), max(pos(:,1))]);
ylim([min(pos(:,2)), max(pos(:,2))]);

%% �X�^�[�g�ʒu�ƒ����������������ăv���b�g
%%{
pos_disp = pos_start + Rot_start * [adv_straight; 0; 0]+ Rot_start * pos';
figure(2); hold off;
plot([0 pos_disp(1,1)], [0, pos_disp(2,1)], 'LineWidth', 4); hold on;
plot(pos_disp(1,:), pos_disp(2,:), 'LineWidth', 4);
plot([pos_disp(1,end), pos_disp(1,end)+extra_straight*cos(pos_end(3))], [pos_disp(2,end), pos_disp(2,end)+extra_straight*sin(pos_end(3))], 'LineWidth', 4);
axis equal;
xlim([floor(min(pos_disp(1,:))/seg_half)*seg_half, ceil(max(pos_disp(1,:)-1)/seg_half)*seg_half]);
ylim([floor(min(pos_disp(2,:))/seg_half)*seg_half, ceil(max(pos_disp(2,:)-1)/seg_half)*seg_half]);
xticks(-5*seg_half:seg_half/6:5*seg_half);
yticks(-5*seg_half:seg_half/6:5*seg_half);
grid on;
xlabel('$x$', 'Interpreter', 'Latex');
ylabel('$y$', 'Interpreter', 'Latex');
%}
%% ���̏o��
% x[mm], y[mm], theta[rad]��CSV�`���ŕۑ�
dlmwrite('data.csv', pos, 'precision', '%.10f');
length
velocity
extra_straight
