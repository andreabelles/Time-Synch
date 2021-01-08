clear; close all; clc;
addpath(genpath('./'));
% Time Synchronization GNSS-aided IMU project
% (Description of the project here)

[tEnd, tIMU, tGNSS, tDelay, M, sigmaAcc, sigmaGNSS, a0, v0, p0,         ...
rp0, rv0, rba0, rt0, sigmaPos, sigmaVel, sigmaAccBias, sigmaTd]         ...
    = loadConfigFile();

%% Generation of true trajectory
[tspan, p, v, a] = generateTrajectory(p0, v0, a0, tIMU, tEnd);

%% Generation of measurements and EKF
[pIMU, vIMU, pGNSS, xEKF, PEKF, xSkog, PSkog, vEKF, pEKF, biasAccEKF] = simulateEstimations(          ...
                                                        p, tspan, M, tIMU, tDelay,  ...
                                                        p0, v0, a0,                 ...
                                                        rp0, rv0, rba0, rt0,        ...
                                                        sigmaGNSS, sigmaAcc,        ...
                                                        sigmaPos, sigmaVel, sigmaAccBias, sigmaTd);

%% Results
tVec        = 0:tIMU:tEnd;
% Computation of errors
errPosIMU   = p - pIMU;
errVelIMU   = v - vIMU;
errPosGNSS  = p - pGNSS;
errPosEKF   = p - pEKF;
errVelEKF   = v - vEKF;
% errPosSkog  = abs(p - pIntSkog');
% errVelSkog  = v - vIntSkog';

sigmaPos    = sqrt(permute(PEKF(1,1,:), [3 1 2]));
sigmaVel    = sqrt(permute(PEKF(2,2,:), [3 1 2]));

%% Plots
% True trajectory vs measurements
figure;
plot(tVec, p, 'k-', 'Linewidth', 1); hold on;
plot(tVec, pIMU, 'b.'); 
plot(tVec, pGNSS, 'r.');
xlabel('Time (s)'); ylabel('Position (m)')
title('True trajectory');
legend('True', 'IMU', 'GNSS');

% IMU Position error plot
figure
plot(tVec, errPosIMU, 'b-'); hold on
plot(tVec, errPosGNSS, 'r.');
xlabel('Time (s)'); ylabel('Position error (m)')
legend('IMU', 'GNSS');
title('IMU-only & GNSS-only Position error');

% IMU Velocity error plot
figure
plot(tVec, errVelIMU, 'b-'); hold on
xlabel('Time (s)'); ylabel('Velocity error (m/s)')
title('IMU-only Velocity error');

% True trajectory vs estimated trajectory
figure;
plot(tVec, p, 'k-', 'Linewidth', 1); hold on;
plot(tVec, pEKF, 'b-'); 
% plot(tVec, pIntSkog, 'r-'); 
xlabel('Time (s)'); ylabel('Position (m)')
title('True position vs estimations');
legend('True', 'EKF');

% True Velocity vs estimated velocity
figure;
plot(tVec, v, 'k-', 'Linewidth', 1); hold on;
plot(tVec, vEKF, 'b-'); 
% plot(tVec, vIntSkog, 'r-'); 
xlabel('Time (s)'); ylabel('Velocity (m/s)')
title('True velocity vs estimations');
legend('True', 'EKF');

figure;
plot(tVec, biasAccEKF, 'b-'); hold on;
% plot(tVec, bIntSkog, 'r-'); 
xlabel('Time (s)'); ylabel('Acceleration bias (m/s^2)')
title('Standard EKF method');
legend('EKF', 'Skog');

% Estimation Error plot
figure
plot(tVec, errPosEKF, 'b-'); hold on;
yline(mean(errPosEKF), 'k');
plot(tVec, 3*sigmaPos, 'r-');
plot(tVec, -3*sigmaPos, 'r-');
xlabel('Time (s)'); ylabel('Position error (m)')
title('Error in position estimations');
legend('EKF', '\mu','3\sigma');

figure
plot(tVec, errVelEKF, 'b-'); hold on;
yline(mean(errVelEKF), 'k');
plot(tVec, 3*sigmaVel, 'r-');
plot(tVec, -3*sigmaVel, 'r-');
xlabel('Time (s)'); ylabel('Velocity error (m/s)')
title('Error in velocity estimations');
legend('Estimation error', '\mu', '3\sigma');

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