clear;
data = load('step300.csv');
data = data(1:420, 2);
plot(data); grid on; hold on;

t = 1:length(data);
k = 2700;
a = 0.0064;
plot(t, k*(1 - exp(-a*t)));

hold off;

p = 300;
a = a * 1000;

P = k/p * tf([a], [1 a]);
step(P);

C = pidtune(P, 'PID');
pidTuner(P);
