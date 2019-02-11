%%
clear all

%% load data
rawdata = dlmread('data4.tab', '\t');
rawdata = rawdata';

%% extract
dt = 1e-3;
t = dt * 1:length(rawdata);
vr =    rawdata(1, :);
xr =    rawdata(2, :);
yv =    rawdata(3, :);
yw =    rawdata(4, :);
x =     rawdata(5, :);
y =     rawdata(6, :);
theta = rawdata(7, :);
v =     rawdata(8, :);
w =     rawdata(9, :);

figure(1);
subplot(2, 1, 1);
plot(t, xr-x, t, 0-y);
title('xr-x, 0-y');
grid on
subplot(2, 1, 2);
plot(t, 0-theta);
title('0-theta');
grid on

figure(2);
subplot(2, 1, 1);
plot(t, yv-v);
title('yv-v');
grid on
subplot(2, 1, 2);
plot(t, yw-w);
title('yw-w');
grid on

