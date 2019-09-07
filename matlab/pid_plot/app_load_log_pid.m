%% Micromouse; Log File Loader and Visualizer
% Author: Ryotaro Onuki
% Created_at: 2019.03.24

%% cleaning
clear;
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');
figindex = 1;

%% settings
save_fig = 0;

%% Select a Log File with GUI
%{
[filename, pathname] = uigetfile({'*'}, 'Select a Log File');
fprintf('Log File: %s\n', filename);
%}

%% Select a Log File directly
%{
pathname = './data/';
filename = '190811-140250.tab';
%}

%% Serial
% %{
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
%}

%% Parse Data
rawdata = dlmread([pathname filename]);

%% Triming
rawdata = rawdata';
% rawdata = rawdata(:, 1:600);

%% extract
dt = 1e-3;
time = dt * 1:length(rawdata);
v_tra = rawdata(1:2, :);
a_tra = rawdata(3:4, :);
u_tra = rawdata(5:9, :);
v_rot = rawdata(10:11, :);
a_rot = rawdata(12:13, :);
u_rot = rawdata(14:18, :);

% log format
%{
sc.ref_v.tra,
sc.est_v.tra,
sc.ref_a.tra,
sc.est_a.tra,
sc.ff.tra,
sc.fb.tra,
sc.fbp.tra,
sc.fbi.tra,
sc.pwm_value.tra,
sc.ref_v.rot,
sc.est_v.rot,
sc.ref_a.rot,
sc.est_a.rot,
sc.ff.rot,
sc.fb.rot,
sc.fbp.rot,
sc.fbi.rot,
sc.pwm_value.rot,
%}

%% Visualization
%% Translational Velocity
figure(figindex); figindex = figindex + 1;

subplot(3, 1, 1); hold off;
plot(time, v_tra); grid on;
title('Translational Velocity');
xlabel('Time [ms]');
ylabel('Velocity [mm/s]');
legend({'Reference', 'Estimation'});

subplot(3, 1, 2); hold off;
plot(time, a_tra); grid on;
title('Translational Acceleration');
xlabel('Time [ms]');
ylabel('Acceleration [mm/s/s]');
legend({'Reference', 'Estimation'});

subplot(3, 1, 3); hold off;
plot(time, u_tra); grid on;
title('Translational Control Input');
xlabel('Time [ms]');
ylabel('Control Input');
legend({'FF', 'FB', 'FB p', 'FB i', 'FF+FB'});

%% Rotational Velocity
figure(figindex); figindex = figindex + 1;

subplot(3, 1, 1); hold off;
plot(time, v_rot); grid on;
title('Rotational Velocity');
xlabel('Time [ms]');
ylabel('Angular Velocity [rad/s]');
legend({'Reference', 'Estimation'});

subplot(3, 1, 2); hold off;
plot(time, a_rot); grid on;
title('Rotational Acceleration');
xlabel('Time [ms]');
ylabel('Angular Acceleration [rad/s/s]');
legend({'Reference', 'Estimation'});

subplot(3, 1, 3); hold off;
plot(time, u_rot); grid on;
title('Rotational Control Input');
xlabel('Time [ms]');
ylabel('Control Input');
legend({'FF', 'FB', 'FB p', 'FB i', 'FF+FB'});

%% save the figure
if save_fig
    %%
    for i = 1:figindex - 1
        fig = figure(i);
        fig.Position(3) = 720; fig.Position(4) = 960;
        fig.PaperPositionMode = 'auto';
        fig.PaperSize = [fig.PaperPosition(3) fig.PaperPosition(4)];
        date_time_str = datestr(datetime('now'), 'yymmdd-HHMMSS');
        outdir = 'figs/';
        [~, ~] = mkdir(outdir);
        warning('off', 'all');
        filename_without_extension = [outdir filename '-' int2str(i)];
        print(fig, [filename_without_extension '.pdf'], '-dpdf');
        print(fig, [filename_without_extension '.png'], '-dpng');
        savefig([filename_without_extension '.fig']);
        warning('on', 'all');
    end

end
