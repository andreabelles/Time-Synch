clear; close all; clc;
addpath(genpath('./'));
% Time Synchronization GNSS-aided IMU project
% (Description of the project here)

Config = loadConfigFile();

%% Generation of true trajectory
[tspan, p, v, a, psi] = generateTrajectory(Config);

%% Generation of measurements and EKF
[pIMU, vIMU, psiIMU, pGNSS, xEKF, PEKF, xSkog, PSkog, vEKF, pEKF, psiEKF, biasAccEKF, pSkog, vSkog, psiSkog, biasAccSkog, timeDelaySkog] = ...
                            simulateEstimations(p, tspan, Config);

%% Results
tVec        = 0:Config.tIMU:Config.tEnd;
% Computation of errors
% IMU errors (INS)
errPosIMU       = p(:, 1:2) - pIMU;
errVelIMU       = v - vIMU;
errHeadingIMU   = psi - psiIMU;
% GNSS errors
errPosGNSS  = p(:, 1:2) - pGNSS;
% Standard EKF errors
errPosEKF   = p(:, 1:2) - pEKF;
errVelEKF   = v - vEKF;
% Skog EKF errors
% errPosSkog   = p(:, 1:2) - pSkog;
% errVelSkog   = v - vSkog;

% STD of Standard EKF estimations
sigmaPosEKF    = sqrt(permute(PEKF(1,1,:), [3 1 2]));
sigmaVelEKF    = sqrt(permute(PEKF(2,2,:), [3 1 2]));
% STD of Skog EKF estimations
% sigmaPosSkog    = sqrt(permute(PSkog(1,1,:), [3 1 2]));
% sigmaVelSkog    = sqrt(permute(PSkog(2,2,:), [3 1 2]));

rmsePosIMU = sqrt(mean(errPosIMU.^2));
rmseVelIMU = sqrt(mean(errVelIMU.^2));
rmsePosGNSS = sqrt(nanmean(errPosGNSS.^2));
rmsePosEKF = sqrt(mean(errPosEKF.^2));
rmseVelEKF = sqrt(mean(errVelEKF.^2));
% rmsePosSkog = sqrt(mean(errPosSkog.^2));
% rmseVelSkog = sqrt(mean(errVelSkog.^2));

fprintf('\t\t ==== RMSE ==== \n');
fprintf('\t\t\t IMU-only \t GNSS-only \t Standard EKF \n');% Skog EKF \n');
fprintf('Position: \t %.4f \t %.4f \t %.4f \t\t %.4f \n', rmsePosIMU, rmsePosGNSS, rmsePosEKF);%, rmsePosSkog);
fprintf('Velocity: \t %.4f \t \t\t \t %.4f \t\t %.4f \n', rmseVelIMU, rmseVelEKF);%, rmseVelSkog);

%% Plots
% True trajectory vs measurements
figure;
plot(p(:, 2), p(:, 1) , 'k-', 'Linewidth', 1.2); hold on;
plot(pIMU(:, 2), pIMU(:, 1), 'b-', 'Linewidth', 1.2); 
plot(pGNSS(:, 2), pGNSS(:, 1), 'c.');
plot(pEKF(:, 2), pEKF(:, 1), 'g-', 'Linewidth', 1.2);
% plot(tVec, pSkog, 'r-', 'Linewidth', 1.2);
xlabel('Time (s)'); ylabel('Position (m)')
title('True trajectory vs estimated');
legend('True', 'IMU', 'GNSS', 'EKF');%, 'Skog');

% IMU Position error plot
% figure
% plot(tVec, errPosIMU, 'b-'); hold on
% plot(tVec, errPosGNSS, 'r.');
% xlabel('Time (s)'); ylabel('Position error (m)')
% legend('IMU', 'GNSS');
% title('IMU-only & GNSS-only Position error');

% IMU Velocity error plot
% figure
% plot(tVec, errVelIMU, 'b-'); hold on
% xlabel('Time (s)'); ylabel('Velocity error (m/s)')
% title('IMU-only Velocity error');

% True trajectory vs estimated trajectory
% figure;
% plot(tVec, p, 'k-', 'Linewidth', 1); hold on;
% plot(tVec, pEKF, 'b-'); 
% % plot(tVec, pIntSkog, 'r-'); 
% xlabel('Time (s)'); ylabel('Position (m)')
% title('True position vs estimated');
% legend('True', 'EKF');

% True Velocity vs estimated velocity
figure;
plot(tVec, v, 'k-', 'Linewidth', 1.2); hold on;
plot(tVec, vEKF, 'b-',  'Linewidth', 1.2);
%plot(tVec, vSkog, 'r-',  'Linewidth', 1.2);
xlabel('Time (s)'); ylabel('Velocity (m/s)')
title('True velocity vs estimated');
legend('True', 'EKF');%, 'Skog');

% True heading vs estimated heading
figure;
plot(tVec, psi, 'k-', 'Linewidth', 1.2); hold on;
plot(tVec, psiEKF, 'b-',  'Linewidth', 1.2); 
% plot(tVec, psiSkog, 'r-', 'Linewidth', 1.2); 
xlabel('Time (s)'); ylabel('Heading (deg)')
title('True heading vs estimations');
legend('True', 'EKF');%, 'Skog');

% Accelerometer Bias
figure;
plot(tVec(Config.M:Config.M:end), biasAccEKF(Config.M:Config.M:end), 'b-'); hold on;
% plot(tVec(Config.M:Config.M:end), biasAccSkog(Config.M:Config.M:end), 'r-'); 
xlabel('Time (s)'); ylabel('Bias (m/s^2)')
title('Accelerometer bias estimation');
legend('EKF');%, 'Skog');

figure;
plot(tVec(Config.M:Config.M:end), xEKF(4,Config.M:Config.M:end), 'b-'); hold on;
% plot(tVec(Config.M:Config.M:end), xSkog(3,Config.M:Config.M:end), 'r-');
xlabel('Time (s)'); ylabel('Error Bias (m/s^2)')
title('Accelerometer bias error-state EKF');
legend('EKF');%, 'Skog');

% North Position Estimation Error plot
figure
subplot(2,1,1)
p1 = plot(tVec, errPosEKF(:, 1), 'b-'); hold on;
% p2 = plot(tVec, errPosSkog(:, 1), 'r-');
% yline(mean(errPosEKF), 'k');
p3 = plot(tVec, 3*sigmaErrPosEKF(:, 1), 'c-');
p4 = plot(tVec, -3*sigmaErrPosEKF(:, 1), 'c-');
% p5 = plot(tVec, 3*sigmaPosSkog(:, 1), 'm-');
% p6 = plot(tVec, -3*sigmaPosSkog(:, 1), 'm-');
xlabel('Time (s)'); ylabel('Position error (m)')
title('Error in North position estimations');
% h = [p1 p2 p3 p5];
h = [p1 p3];
% legend(h,'EKF', 'Skog','3\sigma EKF', '3\sigma Skog');
legend('EKF', '3\sigma EKF');
subplot(2,1,2)
plot(tVec, errPosIMU(:, 1), 'b-'); hold on
plot(tVec, errPosGNSS(:, 1), 'r.');
xlabel('Time (s)'); ylabel('Position error (m)')
legend('IMU', 'GNSS');
title('IMU-only & GNSS-only North Position error');

% East Position Estimation Error plot
figure
subplot(2,1,1)
p1 = plot(tVec, errPosEKF(:, 2), 'b-'); hold on;
% p2 = plot(tVec, errPosSkog(:, 2), 'r-');
% yline(mean(errPosEKF), 'k');
p3 = plot(tVec, 3*sigmaErrPosEKF(:, 2), 'c-');
p4 = plot(tVec, -3*sigmaErrPosEKF(:, 2), 'c-');
% p5 = plot(tVec, 3*sigmaPosSkog(:, 2), 'm-');
% p6 = plot(tVec, -3*sigmaPosSkog(:, 2), 'm-');
xlabel('Time (s)'); ylabel('Position error (m)')
title('Error in East position estimations');
% h = [p1 p2 p3 p5];
h = [p1 p3];
% legend(h,'EKF', 'Skog','3\sigma EKF', '3\sigma Skog');
legend('EKF', '3\sigma EKF');
subplot(2,1,2)
plot(tVec, errPosIMU(:, 2), 'b-'); hold on
plot(tVec, errPosGNSS(:, 2), 'r.');
xlabel('Time (s)'); ylabel('Position error (m)')
legend('IMU', 'GNSS');
title('IMU-only & GNSS-only East Position error');

% Velocity Estimation Error plot
figure
subplot(2,1,1)
p1 = plot(tVec, errVelEKF, 'b-'); hold on;
p2 = plot(tVec, errVelSkog, 'r-');
% yline(mean(errVelEKF), 'k');
p3 = plot(tVec, 3*sigmaVelEKF, 'c-');
p4 = plot(tVec, -3*sigmaVelEKF, 'c-');
p5 = plot(tVec, 3*sigmaVelSkog, 'm-');
p6 = plot(tVec, -3*sigmaVelSkog, 'm-');
xlabel('Time (s)'); ylabel('Velocity error (m)')
title('Error in velocity estimations');
h = [p1 p2 p3 p5];
legend(h,'EKF', 'Skog','3\sigma EKF', '3\sigma Skog');
subplot(2,1,2)
plot(tVec, errVelIMU, 'b-');
xlabel('Time (s)'); ylabel('Velocity error (m/s)')
title('IMU-only Velocity error');

% figure; plot(tVec./1e3, sqrt(permute(abs(PEKF(1, 1, :)), [3 1 2])));
% xlabel('Time (s)'); ylabel('Position std (m/s)');
% title('Standard deviation of the position estimation');
% 
% figure; plot(tVec./1e3, sqrt(permute(abs(PEKF(2, 2, :)), [3 1 2])));
% xlabel('Time (s)'); ylabel('Velocity std (m/s)');
% title('Standard deviation of the velocity estimation');
% 
% figure; plot(tVec./1e3, sqrt(permute(abs(PEKF(3, 3, :)), [3 1 2])));
% xlabel('Time (s)'); ylabel('Bias std (m/s)');
% title('Standard deviation of the bias estimation');

% figure;
% plot(tVec, Config.tDelay, 'k-', 'Linewidth', 1); hold on;
% plot(tVec, timeDelaySkog, 'r-', 'Linewidth', 1);
% xlabel('Time (s)'); ylabel('Time Delay (s)')
% legend('True', 'Estimation')
% title('Time Delay Skog Estimation');

