close all; clear; %clc; addpath(genpath('./'));
% Time Synchronization GNSS-aided IMU project
% (Description of the project here)

%% Initializations
initColumns()
Config = loadConfigFile();
% rng(1);

% tDelays     = [0.5  0.4  0.3  0.2  0.1  0.05  0.01  0];
% PDelayinits = [1.5    0.8  0.6  0.4  0.2  0.1   0.1   0.1];
% Rvalues     = [115  135  135  130  130  120   120   120];
% 
% for i = 1:length(tDelays)
%     Config.tDelay           = tDelays(i);
%     Config.sigmaInitDelay   = PDelayinits(i);
%     Config.varPosGNSS       = Rvalues(i);

    %% Generation of true trajectory
    [trueTrajectory] = generateTrajectory(Config);

    %% Generation of measurements and run estimations
    [estIMU, pGNSS, xEKF, PEKF, xSkog, PSkog, estEKF, estSkog, estSkogPresent, PSkogPresent, measAcc, measAccInt, measAccIntLag] = ...
                                    simulateEstimations(trueTrajectory, Config);

    %% Results and plots
    results(trueTrajectory, Config, estIMU, pGNSS, xEKF, PEKF, xSkog, PSkog, estEKF, estSkog, estSkogPresent, PSkogPresent);
% end
% plotFontSize = 22;
% figSize = [0.8 0.8];
% f = figure('DefaultAxesFontSize', plotFontSize, 'units','normalized','outerposition',[0 0 figSize]);
% histogram(measAccInt - measAccIntLag);
% xlabel('Acceleration difference (m/s^2)'); ylabel('Frequency');
% title('Distribution of the difference between Linear and Lagrange interpolators');