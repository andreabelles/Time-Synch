close all; clear; clc; addpath(genpath('./'));
% Time Synchronization GNSS-aided IMU project
% (Description of the project here)

global COL_TRAJECTORY_TIME COL_TRAJECTORY_POS COL_TRAJECTORY_VEL
global COL_EST_POS COL_EST_VEL COL_EST_ACCBIAS COL_EST_DELAY COL_IMU_MAX COL_EKF_MAX COL_SKOG_MAX

%% Initializations
Config = loadConfigFile();
% rng(1);
initColumns()

%% Generation of true trajectory
[trueTrajectory] = generateTrajectory(Config);

%% Generation of measurements and run estimations
[estIMU, pGNSS, xEKF, PEKF, xSkog, PSkog, estEKF, estSkog, estSkogPresent, PSkogPresent] = ...
                            simulateEstimations(trueTrajectory, Config);

%% Results
% Computation of errors
% IMU errors (INS)
errPosIMU   = trueTrajectory(:, COL_TRAJECTORY_POS) - estIMU(:, COL_EST_POS);
errVelIMU   = trueTrajectory(:, COL_TRAJECTORY_VEL) - estIMU(:, COL_EST_VEL);
% GNSS errors
errPosGNSS  = trueTrajectory(:, COL_TRAJECTORY_POS) - pGNSS;
% Standard EKF errors
errPosEKF   = trueTrajectory(:, COL_TRAJECTORY_POS) - estEKF(:, COL_EST_POS);
errVelEKF   = trueTrajectory(:, COL_TRAJECTORY_VEL) - estEKF(:, COL_EST_VEL);
% Skog EKF errors
errPosSkog   = trueTrajectory(:, COL_TRAJECTORY_POS) - estSkog(:, COL_EST_POS);
errVelSkog   = trueTrajectory(:, COL_TRAJECTORY_VEL) - estSkog(:, COL_EST_VEL);
errDelaySkog = Config.tDelay - estSkog(:, COL_EST_DELAY);
% Skog EKF at present errors
errPosSkogPresent   = trueTrajectory(:, COL_TRAJECTORY_POS) - estSkogPresent(:, COL_EST_POS);
errVelSkogPresent   = trueTrajectory(:, COL_TRAJECTORY_VEL) - estSkogPresent(:, COL_EST_VEL);
errDelaySkogPresent = Config.tDelay - estSkogPresent(:, COL_EST_DELAY);

% STD of Standard EKF estimations
sigmaPosEKF    = sqrt(permute(PEKF(1,1,:), [3 1 2]));
sigmaVelEKF    = sqrt(permute(PEKF(2,2,:), [3 1 2]));
% STD of Skog EKF estimations
sigmaPosSkog    = sqrt(permute(PSkog(1,1,:), [3 1 2]));
sigmaVelSkog    = sqrt(permute(PSkog(2,2,:), [3 1 2]));
% STD of Skog EKF to present estimations
sigmaPosSkogPresent    = sqrt(permute(PSkogPresent(1,1,:), [3 1 2]));
sigmaVelSkogPresent    = sqrt(permute(PSkogPresent(2,2,:), [3 1 2]));

rmsePosIMU = sqrt(mean(errPosIMU.^2));
rmseVelIMU = sqrt(mean(errVelIMU.^2));
rmsePosGNSS = sqrt(nanmean(errPosGNSS.^2));
rmsePosEKF = sqrt(mean(errPosEKF.^2));
rmseVelEKF = sqrt(mean(errVelEKF.^2));
rmsePosSkog = sqrt(mean(errPosSkog.^2));
rmseVelSkog = sqrt(mean(errVelSkog.^2));
rmsePosSkogPresent = sqrt(mean(errPosSkogPresent.^2));
rmseVelSkogPresent = sqrt(mean(errVelSkogPresent.^2));


fprintf('\n\t\t\t\t\t ==== RMSE ==== \n');
fprintf('\t\t IMU-only \t GNSS-only \t Standard EKF \t Skog EKF \t Skog EKF at present \n');
fprintf('Position: \t %.4f \t %.4f \t %.4f \t %.4f \t %.4f \n', ...
        rmsePosIMU, rmsePosGNSS, rmsePosEKF, rmsePosSkog, rmsePosSkogPresent);
fprintf('Velocity: \t %.4f \t N/A \t\t %.4f \t %.4f \t %.4f \n\n', ...
        rmseVelIMU, rmseVelEKF, rmseVelSkog, rmseVelSkogPresent);

%% Plots
tVec        = trueTrajectory(:, COL_TRAJECTORY_TIME);
kGNSS       = Config.M:Config.M:length(tVec); % samples with GNSS measurement

% True trajectory vs measurements
figure;
plot(tVec, trueTrajectory(:, COL_TRAJECTORY_POS), 'k-', 'Linewidth', 1.2); hold on;
plot(tVec, estIMU(:, COL_EST_POS), 'b-', 'Linewidth', 1.2); 
plot(tVec, pGNSS, 'c.');
plot(tVec, estEKF(:, COL_EST_POS), 'g-', 'Linewidth', 1.2);
plot(tVec, estSkog(:, COL_EST_POS), 'r-', 'Linewidth', 1.2);
plot(tVec, estSkogPresent(:, COL_EST_POS), 'm-', 'Linewidth', 1.2);
xlabel('Time (s)'); ylabel('Position (m)')
title('True trajectory vs estimated');
legend('True', 'IMU', 'GNSS', 'EKF', 'Skog', 'Skog present');

% True Velocity vs estimated velocity
figure;
plot(tVec, trueTrajectory(:, COL_TRAJECTORY_VEL), 'k-', 'Linewidth', 1); hold on;
plot(tVec, estEKF(:, COL_EST_VEL), 'b-');
plot(tVec, estSkog(:, COL_EST_VEL), 'r-');
plot(tVec, estSkogPresent(:, COL_EST_VEL), 'g-');
xlabel('Time (s)'); ylabel('Velocity (m/s)')
title('True velocity vs estimated');
legend('True', 'EKF', 'Skog', 'Skog present');

figure;
plot(tVec, estEKF(:, COL_EST_ACCBIAS), 'b-'); hold on;
plot(tVec, estSkog(:, COL_EST_ACCBIAS), 'r-'); 
xlabel('Time (s)'); ylabel('Bias (m/s^2)')
title('Accelerometer bias estimation');
legend('EKF', 'Skog');

figure;
plot(tVec(kGNSS), xEKF(COL_EST_ACCBIAS, kGNSS), 'b-'); hold on;
plot(tVec(kGNSS), xSkog(COL_EST_ACCBIAS, kGNSS), 'r-');
xlabel('Time (s)'); ylabel('Error Bias (m/s^2)')
title('Accelerometer bias error-state EKF');
legend('EKF', 'Skog');

% Position Estimation Error plot
figure
subplot(2,1,1)
p1 = plot(tVec, errPosEKF, 'b-'); hold on;
p2 = plot(tVec, errPosSkog, 'r-');
p21 = plot(tVec, errPosSkogPresent, 'g-');
% yline(mean(errPosEKF), 'k');
p3 = plot(tVec, 3*sigmaPosEKF, 'c-');
p4 = plot(tVec, -3*sigmaPosEKF, 'c-');
p5 = plot(tVec, 3*sigmaPosSkog, 'm-');
p6 = plot(tVec, -3*sigmaPosSkog, 'm-');
p7 = plot(tVec, 3*sigmaPosSkogPresent, 'Color', '#D95319');
p8 = plot(tVec, -3*sigmaPosSkogPresent, 'Color', '#D95319');
xlabel('Time (s)'); ylabel('Position error (m)')
title('Error in position estimations');
h = [p1 p2 p21 p3 p5 p7];
legend(h,'EKF', 'Skog', 'Skog present','3\sigma EKF', '3\sigma Skog', '3\sigma SkogPresent');
subplot(2,1,2)
plot(tVec, errPosIMU, 'b-'); hold on;
plot(tVec, errPosGNSS, 'm.');
xlabel('Time (s)'); ylabel('Position error (m)')
legend('IMU-only', 'GNSS-only')
title('IMU-only and GNSS-only Position error');

% Velocity Estimation Error plot
figure
subplot(2,1,1)
p1 = plot(tVec, errVelEKF, 'b-'); hold on;
p2 = plot(tVec, errVelSkog, 'r-');
p21 = plot(tVec, errVelSkogPresent, 'g-');
% yline(mean(errVelEKF), 'k');
p3 = plot(tVec, 3*sigmaVelEKF, 'c-');
p4 = plot(tVec, -3*sigmaVelEKF, 'c-');
p5 = plot(tVec, 3*sigmaVelSkog, 'm-');
p6 = plot(tVec, -3*sigmaVelSkog, 'm-');
p7 = plot(tVec, 3*sigmaVelSkogPresent, 'Color', '#D95319');
p8 = plot(tVec, -3*sigmaVelSkogPresent, 'Color', '#D95319');
xlabel('Time (s)'); ylabel('Velocity error (m)')
title('Error in velocity estimations');
h = [p1 p2 p21 p3 p5, p7];
legend(h,'EKF', 'Skog', 'Skog present','3\sigma EKF', '3\sigma Skog', '3\sigma SkogPresent');
subplot(2,1,2)
plot(tVec, errVelIMU, 'b-');
xlabel('Time (s)'); ylabel('Velocity error (m/s)')
title('IMU-only Velocity error');

figure;
subplot(2,1,1)
plot(tVec([1 end]), Config.tDelay*ones(1, 2), 'k-', 'Linewidth', 1); hold on;
plot(tVec, estSkog(:, COL_EST_DELAY), 'r-', 'Linewidth', 1);
xlabel('Time (s)'); ylabel('Time Delay (s)')
legend('True', 'Estimation')
title('Time Delay Skog Estimation');
subplot(2,1,2)
plot(tVec, errDelaySkog);
xlabel('Time (s)'); ylabel('Delay error (s)');
title('Time delay estimation error');


