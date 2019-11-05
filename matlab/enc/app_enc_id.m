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
filename = '191022-171343.tab';
%}

%% Serial input
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
fprintf("Saved Log File: %s\n", filename);
%}

%% Parse Data
rawdata = dlmread([pathname filename]);

%% Triming
rawdata = rawdata';
% rawdata = rawdata(:, 1:600);

%% extract
dt = 1e-3;
time = dt * 1:length(rawdata);
enc_pulses = rawdata(1:2, :);
angle = rawdata(3:4, :);
angular_velocity = rawdata(5:6, :);
control_input = rawdata(7:12, :);

%% Visualization
figure(figindex); figindex = figindex + 1;

subplot(3, 1, 1); hold off;
plot(time, angle(1, :) - angle(2,:)); grid on;
title('Traking Error of Angle');
xlabel('Time [ms]');
ylabel('Angle Error [rad]');

subplot(3, 1, 2); hold off;
plot(time, angular_velocity); grid on;
title('Angular Velocity');
xlabel('Time [ms]');
ylabel('Angular Velocity [rad/s]');
legend({'Reference', 'Estimation'});

subplot(3, 1, 3); hold off;
plot(time, control_input); grid on;
title('Translational Control Input');
xlabel('Time [ms]');
ylabel('Control Input');
legend({'FF', 'FB', 'FB p', 'FB i', 'FB d', 'FF+FB'});

%% Visualization
figure(figindex); figindex = figindex + 1;

% subplot(2, 1, 1); 
hold off;
plot(time, angle(1, :) - angle(2,:)); grid on; hold on;
plot(time(1:end-1), diff(enc_pulses(2, :))' / 29 * pi / 100);
title('Traking Error of Angle');
xlabel('Time [ms]');
ylabel('Angle Error [rad]');

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
