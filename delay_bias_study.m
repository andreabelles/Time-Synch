close all; clc
%% DELAY BIAS STUDY
initColumns()
global COL_TRAJECTORY_TIME COL_TRAJECTORY_VEL COL_TRAJECTORY_DELAY
global COL_EST_VEL COL_EST_DELAY

%% LOAD DATA
load('/Users/arnauochoa/Documents/Estudis/ASNAT/Semester 3/Sensor_Timig/Time-Synch/Results/results_0.mat')
err_d0 = trueTrajectory(:, COL_TRAJECTORY_DELAY) - estSkog(:, COL_EST_DELAY);
err_v0 = trueTrajectory(:, COL_TRAJECTORY_VEL) - estSkog(:, COL_EST_VEL);
load('/Users/arnauochoa/Documents/Estudis/ASNAT/Semester 3/Sensor_Timig/Time-Synch/Results/results_0-01.mat')
err_d001 = trueTrajectory(:, COL_TRAJECTORY_DELAY) - estSkog(:, COL_EST_DELAY);
err_v001 = trueTrajectory(:, COL_TRAJECTORY_VEL) - estSkog(:, COL_EST_VEL);
load('/Users/arnauochoa/Documents/Estudis/ASNAT/Semester 3/Sensor_Timig/Time-Synch/Results/results_0-1.mat')
err_d01 = trueTrajectory(:, COL_TRAJECTORY_DELAY) - estSkog(:, COL_EST_DELAY);
err_v01 = trueTrajectory(:, COL_TRAJECTORY_VEL) - estSkog(:, COL_EST_VEL);
load('/Users/arnauochoa/Documents/Estudis/ASNAT/Semester 3/Sensor_Timig/Time-Synch/Results/results_0-5.mat')
err_d05 = trueTrajectory(:, COL_TRAJECTORY_DELAY) - estSkog(:, COL_EST_DELAY);
err_v05 = trueTrajectory(:, COL_TRAJECTORY_VEL) - estSkog(:, COL_EST_VEL);

%% Initializations
g = 9.8; % m/s^2
a = 2*g;

%% Bias computation
d_0 = a * (err_d0.^2) / 2 - err_v0 .* err_d0;
d_001 = a * (err_d001.^2) / 2 - err_v001 .* err_d001;
d_01 = a * (err_d01.^2) / 2 - err_v01 .* err_d01;
d_05 = a * (err_d05.^2) / 2 - err_v05 .* err_d05;

%% Plots config
savePlots = 1;
lineWidth = 2;
plotFontSize = 22;
figSize = [0.8 0.8];
savePath = '../Figures_auto/delay_bias';
format = '.png'; %'.fig'
if ~exist(savePath, 'dir'), mkdir(savePath); end

%% PLOTS 
f = figure('DefaultAxesFontSize',plotFontSize , 'units','normalized','outerposition',[0 0 figSize]);
hold on;
plot(trueTrajectory(:, COL_TRAJECTORY_TIME), err_d0, 'Linewidth', lineWidth);
plot(trueTrajectory(:, COL_TRAJECTORY_TIME), err_d001, 'Linewidth', lineWidth);
plot(trueTrajectory(:, COL_TRAJECTORY_TIME), err_d01, 'Linewidth', lineWidth);
plot(trueTrajectory(:, COL_TRAJECTORY_TIME), err_d05, 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('\delta T_d (s)'); 
legend('Td = 0', 'Td = 0.01', 'Td = 0.1', 'Td = 0.5')
title('Time delay estimation errors')

FigName = 'delay_errors';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

f = figure('DefaultAxesFontSize',plotFontSize , 'units','normalized','outerposition',[0 0 figSize]);
hold on;
plot(trueTrajectory(:, COL_TRAJECTORY_TIME), d_0, 'Linewidth', lineWidth);
plot(trueTrajectory(:, COL_TRAJECTORY_TIME), d_001, 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('d (m)'); 
legend('Td = 0', 'Td = 0.01')
title('Bias due to delay')

FigName = 'bias_small';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

f = figure('DefaultAxesFontSize',plotFontSize , 'units','normalized','outerposition',[0 0 figSize]);
hold on;
plot(trueTrajectory(:, COL_TRAJECTORY_TIME), d_01, 'Linewidth', lineWidth);
plot(trueTrajectory(:, COL_TRAJECTORY_TIME), d_05, 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('d (m)'); 
legend('Td = 0.1', 'Td = 0.5')
title('Bias due to delay')

FigName = 'bias_large';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;