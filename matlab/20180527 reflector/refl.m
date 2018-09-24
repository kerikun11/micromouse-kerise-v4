data = csvread('reflector.csv');
data = data(1:40, :);

xdata = data(:, 2);
ydata = data(:, 1);

hold off;
plot(data(:, 2), data(:, 1),'x');
grid on;
hold on;
% x=0:3600;
% plot(x, 10*log(x)-70);

% f = fit(data(:,2), data(:,1), 'poly3');
% plot(f);

fun = @(k,x) k(1) * log(x) + k(2);

k0 = [0, 0];
k = lsqcurvefit(fun, k0, xdata, ydata);
plot(xdata, fun(k, xdata));
xlabel('reflector raw value');
ylabel('position [mm]');
title('Photo-Reflector Specific Graph');

figure(2);
hold off;
plot(-ydata+45-25,xdata,'x');
grid on;
xlim([0 45]);
ylim([0 4096]);
yticks(0:256:4096);
xlabel('Distance From Wall [mm]');
ylabel('ADC Value');
title('KERISE Photo-Reflector Specific Graph');
