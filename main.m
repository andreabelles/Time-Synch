close all; clear; clc; addpath(genpath('./'));
% Time Synchronization GNSS-aided IMU project
% (Description of the project here)

%% Initializations
Config = loadConfigFile();
% rng(1);
initColumns()

%% Generation of true trajectory
[trueTrajectory] = generateTrajectory(Config);

%% Generation of measurements and run estimations
[estIMU, pGNSS, xEKF, PEKF, xSkog, PSkog, estEKF, estSkog, estSkogPresent, PSkogPresent] = ...
                                simulateEstimations(trueTrajectory, Config);
   
%% Results and plots
results(trueTrajectory, Config, estIMU, pGNSS, xEKF, PEKF, xSkog, PSkog, estEKF, estSkog, estSkogPresent, PSkogPresent);
