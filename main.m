clear; close all; clc;
addpath(genpath('./'));
% Time Synchronization GNSS-aided IMU project
% (Description of the project here)

Config = loadConfigFile();

%% Generation of true trajectory
[tspan, trueTrajectory] = generateTrajectory(Config); % trueTrajectory: [posN, posE, vel, acc, heading, headingRate]

%% Generation of measurements and EKF
[estIMURaw, pGNSS, xEKF, PEKF, estEKF, xSkog, PSkog, estSkog, estSkogAtDelay, PSkogAtDelay] = ...
            simulateEstimations(trueTrajectory, tspan, Config);
        
%% Results
tVec        = 0:Config.tIMU:Config.tEnd;

% Computation of errors
% IMU errors (INS)
errPosIMU   = trueTrajectory(:, 1:2) - estIMURaw.pos;        % errPosIMU: [errorNorth, errorEast]
errVelIMU   = trueTrajectory(:, 3) - estIMURaw.vel;
errHeadingIMU   = trueTrajectory(:, 5) - estIMURaw.heading;

% GNSS errors
errPosGNSS  = trueTrajectory(:, 1:2) - pGNSS;       % errPosGNSS: [errorNorth, errorEast]

% Standard EKF errors
errPosEKF   = trueTrajectory(:, 1:2) - estEKF.pos;    % errPosEKF: [errorNorth, errorEast]
errVelEKF   = trueTrajectory(:, 3) - estEKF.vel;
errHeadingEKF   = trueTrajectory(:, 5) - estEKF.heading;

% Skog EKF errors
% errPosSkog  = trueTrajectory(:, 1:2) - estSkog.pos;    % errPosEKF: [errorNorth, errorEast]
% errVelSkog  = trueTrajectory(:, 3) - estSkog.vel;
% errHeadingSkog   = trueTrajectory(:, 3) - estSkog.heading;

% Eucliedean errors
errEucliIMU = sqrt( errPosIMU(:, 1).^2 + errPosIMU(:, 2).^2 );
errEucliGNSS = sqrt( errPosGNSS(:, 1).^2 + errPosGNSS(:, 2).^2 );
errEucliEKF = sqrt( errPosEKF(:, 1).^2 + errPosEKF(:, 2).^2 );
% errEucliSkog = sqrt( errPosSkog(:, 1).^2 + errPosSkog(:, 2).^2 );

% STD of Standard EKF estimations
sigmaErrPosEKF         = sqrt([permute(PEKF(1, 1, :), [3 1 2]), permute(PEKF(1, 1, :), [3 1 2])]); % [sigmaNorth, sigmaEast]
sigmaErrVelEKF         = sqrt(permute(PEKF(3, 3, :), [3 1 2]));
% sigmaErrHeadingEKF     = sqrt(permute(PEKF(4, 4, :), [3 1 2]));
% sigmaErrBiasAccEKF     = sqrt(permute(PEKF(5, 5, :), [3 1 2]));
% sigmaErrBiasGyroEKF    = sqrt(permute(PEKF(6, 6, :), [3 1 2]));

% STD of Skog EKF estimations
% sigmasPosSkog     = sqrt([permute(PSkogHistoric(1, 1, :), [3 1 2]), permute(PSkogHistoric(1, 1, :), [3 1 2])]); % [sigmaNorth, sigmaEast]
% sigmaVelSkog      = sqrt(permute(PSkogHistoric(3, 3, :), [3 1 2]));
% sigmaHeadingSkog     = sqrt(permute(PSkogHistoric(4, 4, :), [3 1 2]));
% sigmaBiasAccSkog  = sqrt(permute(PSkogHistoric(5, 5, :), [3 1 2]));
% sigmaBiasGyroSkog = sqrt(permute(PSkogHistoric(6, 6, :), [3 1 2]));
% sigmaDelaySkog    = sqrt(permute(PSkogHistoric(7, 7, :), [3 1 2]));

% Euclidean STD of error estimations
sigmaErrEucliEKF = sqrt(sigmaErrPosEKF(:, 1).^2 + sigmaErrPosEKF(:, 2).^2);
% sigmaEucliSkog = sqrt( sigmasPosSkog(:, 1).^2 + sigmaPosSkog(:, 2)).^2);

% Euclidean distance of estimation
eucliEKF = sqrt(estEKF.pos(:, 1).^2 + estEKF.pos(:, 2).^2);
% eucliSkog = sqrt(estSkog.pos(:, 1).^2 + estSkog.pos(:, 2).^2);

%% Plots

% True trajectory vs measurements
% figure;
% plot(trueTrajectory(:, 2), trueTrajectory(:, 1), '.k') 
% xlabel('East (m)'); ylabel('North (m)')
% title('True trajectory');

% figure;
% plot(tVec, trueTrajectory(4, :), '.k') 
% xlabel('Time (s)'); ylabel('Heading (deg)')
% title('Heading');

% True trajectory vs measurements
figure;
plot(trueTrajectory(:, 2), trueTrajectory(:, 1), 'k-', 'Linewidth', 1.2); hold on;
plot(estIMURaw.pos(:, 2), estIMURaw.pos(:, 1), 'b-', 'Linewidth', 1.2); 
plot(pGNSS(:, 2), pGNSS(:, 1), 'c.');
plot(estEKF.pos(:, 2), estEKF.pos(:, 1), 'g-', 'Linewidth', 1.2);
% plot(estSkog.pos(:, 2), estSkog.pos(:, 1), 'r-', 'Linewidth', 1.2);
xlabel('East (m)'); ylabel('North (m)');
title('True trajectory vs estimated');
legend('True', 'IMU', 'GNSS', 'EKF');%, 'Skog');

% True Velocity vs estimated velocity
figure;
plot(tVec, trueTrajectory(:, 3), 'k-', 'Linewidth', 1.2); hold on;
plot(tVec, estEKF.vel, 'b-',  'Linewidth', 1.2);
%plot(tVec, estSkog.vel, 'r-',  'Linewidth', 1.2);
xlabel('Time (s)'); ylabel('Velocity (m/s)')
title('True velocity vs estimated');
legend('True', 'EKF');%, 'Skog');

% True heading vs estimated heading
figure;
plot(tVec, trueTrajectory(:, 5), 'k-', 'Linewidth', 1.2); hold on;
plot(tVec, estEKF.heading, 'b-',  'Linewidth', 1.2); 
% plot(tVec, estSkog.heading, 'r-', 'Linewidth', 1.2); 
xlabel('Time (s)'); ylabel('Heading (deg)')
title('True heading vs estimations');
legend('True', 'EKF');%, 'Skog');

% Accelerometer Bias
figure;
plot(tVec(Config.M:Config.M:end), estEKF.biasAcc(Config.M:Config.M:end), 'b-'); hold on;
% plot(tVec(Config.M:Config.M:end), estSkog.biasAcc(Config.M:Config.M:end), 'r-'); 
xlabel('Time (s)'); ylabel('Bias (m/s^2)')
title('Accelerometer bias estimation');
legend('EKF');%, 'Skog');

figure;
plot(tVec(Config.M:Config.M:end), xEKF(4,Config.M:Config.M:end), 'b-'); hold on;
% plot(tVec(Config.M:Config.M:end), xSkog(3,Config.M:Config.M:end), 'r-');
xlabel('Time (s)'); ylabel('Error Bias (m/s^2)')
title('Accelerometer bias error-state EKF');
legend('EKF');%, 'Skog');

% % Gyroscope Bias
% figure;
% plot(tVec(Config.M:Config.M:end), estEKF.biasGyro(Config.M:Config.M:end), 'b-'); hold on;
% % plot(tVec(Config.M:Config.M:end), estSkog.biasGyro(Config.M:Config.M:end), 'r-'); 
% xlabel('Time (s)'); ylabel('Bias (deg/s)')
% title('Gyroscope bias estimation');
% legend('EKF');%, 'Skog');


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
% legend(h,'EKF', 'Skog','3\sigma EKF', '3\sigma Skog');
legend('EKF', '3\sigma EKF');
subplot(2,1,2)
plot(tVec, errPosIMU(:, 2), 'b-'); hold on
plot(tVec, errPosGNSS(:, 2), 'r.');
xlabel('Time (s)'); ylabel('Position error (m)')
legend('IMU', 'GNSS');
title('IMU-only & GNSS-only East Position error');

% Euclidean error distance 
figure
subplot(2,1,1)
p1 = plot(tVec, errEucliEKF, 'b-'); hold on;
% p2 = plot(tVec, errEucliSkog, 'r-');
% yline(mean(errPosEKF), 'k');
p3 = plot(tVec, 3*sigmaErrEucliEKF, 'c-');
p4 = plot(tVec, -3*sigmaErrEucliEKF, 'c-');
% p5 = plot(tVec, 3*sigmaEucliSkog, 'm-');
% p6 = plot(tVec, -3*sigmaEucliSkog, 'm-');
xlabel('Time (s)'); ylabel('Position error (m)')
title('Error in Euclidean position estimations');
% h = [p1 p2 p3 p5];
% legend(h,'EKF', 'Skog','3\sigma EKF', '3\sigma Skog');
legend('EKF', '3\sigma EKF');
subplot(2,1,2)
plot(tVec, errEucliIMU, 'b-'); hold on
plot(tVec, errEucliGNSS, 'r.');
xlabel('Time (s)'); ylabel('Position error (m)')
legend('IMU', 'GNSS');
title('IMU-only & GNSS-only Euclidean Position error');

% Velocity Estimation Error plot
figure
subplot(2,1,1)
p1 = plot(tVec, errVelEKF, 'b-'); hold on;
% p2 = plot(tVec, errVelSkog, 'r-');
% yline(mean(errVelEKF), 'k');
p3 = plot(tVec, 3*sigmaErrVelEKF, 'c-');
p4 = plot(tVec, -3*sigmaErrVelEKF, 'c-');
% p5 = plot(tVec, 3*sigmaVelSkog, 'm-');
% p6 = plot(tVec, -3*sigmaVelSkog, 'm-');
xlabel('Time (s)'); ylabel('Velocity error (m)')
title('Error in velocity estimations');
% h = [p1 p2 p3 p5];
% legend(h,'EKF', 'Skog','3\sigma EKF', '3\sigma Skog');
legend('EKF', '3\sigma EKF');
subplot(2,1,2)
plot(tVec, errVelIMU, 'b-');
xlabel('Time (s)'); ylabel('Velocity error (m/s)')
title('IMU-only Velocity error');

% Heading Estimation Error plot
figure
subplot(2,1,1)
p1 = plot(tVec, errHeadingEKF, 'b-'); hold on;
% p2 = plot(tVec, errHeadingSkog, 'r-');
% yline(mean(errVelEKF), 'k');
% p3 = plot(tVec, 3*sigmaErrHeadingEKF, 'c-');
% p4 = plot(tVec, -3*sigmaErrHeadingEKF, 'c-');
% p5 = plot(tVec, 3*sigmaHeadingSkog, 'm-');
% p6 = plot(tVec, -3*sigmaHeadingSkog, 'm-');
xlabel('Time (s)'); ylabel('Heading error (deg)')
title('Error in heading estimations');
% h = [p1 p2 p3 p5];
% legend(h,'EKF', 'Skog','3\sigma EKF', '3\sigma Skog');
legend('EKF');%, '3\sigma EKF');
subplot(2,1,2)
plot(tVec, errHeadingIMU, 'b-');
xlabel('Time (s)'); ylabel('Heading error (deg)')
title('IMU-only Heading error');

% ------------------------------------------------------------------------
% IMU & GNSS Position error comparison
% figure
% plot(tVec, abs(errPosIMU(:, 1)), 'b-'); hold on
% plot(tVec, abs(errPosIMU(:, 2)), 'g-');
% plot(tVec, abs(errPosGNSS(:, 1)), 'r.');
% plot(tVec, abs(errPosGNSS(:, 2)), 'm.');
% xlabel('Time (s)'); ylabel('Absolute position error (m)')
% legend('IMU North', 'IMU East', 'GNSS North', 'GNSS East');
% title('IMU-only & GNSS-only Position error');

% IMU Velocity error plot
% figure
% plot(tVec, errVelIMU, 'b-'); hold on
% xlabel('Time (s)'); ylabel('Velocity error (m/s)')
% title('IMU-only Velocity error');


% True trajectory vs estimated trajectory
% figure;
% plot(trueTrajectory(:, 2), trueTrajectory(:, 1), '.k'); hold on;
% plot(estEKF.pos(:, 2), estEKF.pos(:, 1), 'b-'); 
% % plot(estIntSkogHistoric.pos(:, 2), estIntSkogHistoric.pos(:, 1), 'r-'); 
% xlabel('East (m)'); ylabel('North (m)');
% title('True position vs estimations');
% legend('True', 'EKF', 'Skog');

% EKF Position error
% figure
% plot(tVec, abs(errPosEKF(:, 1)), 'b-', 'Linewidth', 1.5); hold on
% plot(tVec, abs(errPosEKF(:, 2)), 'g-', 'Linewidth', 1.5);
% % plot(tVec, abs(errPosSkog(:, 1)), 'r-', 'Linewidth', 1.5);
% % plot(tVec, abs(errPosSkog(:, 2)), 'm-', 'Linewidth', 1.5);
% xlabel('Time (s)'); ylabel('Absolute position error (m)')
% legend('EKF North', 'EKF East', 'SKOG North', 'SKOG East');
% title('EKF Position error');

% figure
% plot(tVec, errEucliEKF, 'b-', 'Linewidth', 1.5); hold on
% % plot(tVec, errEucliSkog, 'r-', 'Linewidth', 1.5);
% xlabel('Time (s)'); ylabel('Absolute position error (m)')
% legend('EKF error', 'SKOG error');
% title('EKF Position error');

% figure
% plot(tVec, estIntSkogHistoric.timeDelay, 'b-', 'Linewidth', 1.5);
% xlabel('Time (s)'); ylabel('Time delay (s)')
% title('Time delay estimation');

% Position STD
% figure
% plot(tVec, sigmaEucliEKF, 'b-', 'Linewidth', 1.5); hold on
% % plot(tVec, stdEucliSkog, 'r-', 'Linewidth', 1.5);
% xlabel('Time (s)'); ylabel('Position STD (m)')
% legend('EKF \sigma_P', 'SKOG \sigma_P');
% title('EKF position STD');

% % Velocity STD
% figure
% plot(tVec, sqrt(abs(varVelEKF)), 'b-', 'Linewidth', 1.5); hold on
% % plot(tVec, sqrt(abs(varVelSkog)), 'r-', 'Linewidth', 1.5);
% xlabel('Time (s)'); ylabel('Velocity STD (m/s)')
% legend('EKF \sigma_V', 'SKOG \sigma_V');
% title('EKF velocity STD');

% Heading STD
% figure
% plot(tVec, sqrt(sigmaHeadingEKF), 'b-', 'Linewidth', 1.5); hold on
% % plot(tVec, sqrt(varHeadSkog), 'r-', 'Linewidth', 1.5);
% xlabel('Time (s)'); ylabel('Heading STD (deg)')
% legend('EKF \sigma_{\phi}', 'SKOG \sigma_{\phi}');
% title('EKF heading STD');

% Delay STD
% figure
% plot(tVec, sqrt(varDelaySkog), 'r-', 'Linewidth', 1.5);
% xlabel('Time (s)'); ylabel('Delay STD (s)')
% legend('EKF \sigma_{Td}');
% title('EKF Delay STD');

% figure;
% plot(tVec, bAccIntEKF, 'b-'); hold on;
% plot(tVec, bAccIntSkog, 'r-'); 
% xlabel('Time (s)'); ylabel('Acceleration bias (m/s^2)')
% title('Standard EKF method');
% legend('EKF', 'Skog');
% 
% % Estimation Error plot
% figure
% plot(tVec, errPosEKF, 'b-'); hold on;
% plot(tVec, errPosSkog, 'r-'); 
% xlabel('Time (s)'); ylabel('Position error (m)')
% title('Error in position estimations');
% legend('EKF', 'Skog');
% 
% figure
% plot(tVec, errVelEKF, 'b-'); hold on;
% plot(tVec, errVelSkog, 'r-');
% xlabel('Time (s)'); ylabel('Velocity error (m/s)')
% title('Error in velocity estimations');
% legend('EKF', 'Skog');

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
% plot(kVec./1e3, tDelay*ones(length(kVec)), 'k-', 'Linewidth', 1); hold on;
% plot(kVec./1e3, xSkog(3,:), 'b.'); 
% xlabel('Time (s)'); ylabel('Position (m)')
% title('Time Delay Evolution');