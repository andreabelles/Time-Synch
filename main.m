clear; close all; clc;
addpath(genpath('./'));
% Time Synchronization GNSS-aided IMU project
% (Description of the project here)

Config = loadConfigFile();

%% Generation of true trajectory
[tspan, trueTrajectory] = generateTrajectory(Config);

%% Generation of measurements and EKF
[pIMU, vIMU, pGNSS, xEKF, PEKF, xSkog, PSkog] = simulateEstimations(trueTrajectory, tspan, Config);

%% Results
tVec        = 0:Config.tIMU:Config.tEnd;
pIntEKF     = xEKF(1, :);
vIntEKF     = xEKF(2, :);
bIntEKF     = xEKF(3, :);
pIntSkog    = xSkog(1, :);
vIntSkog    = xSkog(2, :);
bIntSkog    = xSkog(3, :);
% Computation of errors
errPosIMU   = abs(p - pIMU);
errVelIMU   = v - vIMU;
errPosGNSS  = abs(p - pGNSS);
errPosEKF   = abs(p - pIntEKF');
errVelEKF   = v - vIntEKF';
errPosSkog  = abs(p - pIntSkog');
errVelSkog  = v - vIntSkog';


%% Plots

% True trajectory vs measurements
figure;
plot(pEast, pNorth, '.k') 
xlabel('East (m)'); ylabel('North (m)')
title('True trajectory');

figure;
plot(tVec, heading, '.k') 
xlabel('Time (s)'); ylabel('Heading (deg)')
title('Heading');

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
plot(tVec, pIntEKF, 'b-'); 
plot(tVec, pIntSkog, 'r-'); 
xlabel('Time (s)'); ylabel('Position (m)')
title('True position vs estimations');
legend('True', 'EKF', 'Skog');

% True Velocity vs estimated velocity
figure;
plot(tVec, v, 'k-', 'Linewidth', 1); hold on;
plot(tVec, vIntEKF, 'b-'); 
plot(tVec, vIntSkog, 'r-'); 
xlabel('Time (s)'); ylabel('Velocity (m/s)')
title('True velocity vs estimations');
legend('True', 'EKF', 'Skog');

figure;
plot(tVec, bIntEKF, 'b-'); hold on;
plot(tVec, bIntSkog, 'r-'); 
xlabel('Time (s)'); ylabel('Acceleration bias (m/s^2)')
title('Standard EKF method');
legend('EKF', 'Skog');

% Estimation Error plot
figure
plot(tVec, errPosEKF, 'b-'); hold on;
plot(tVec, errPosSkog, 'r-'); 
xlabel('Time (s)'); ylabel('Position error (m)')
title('Error in position estimations');
legend('EKF', 'Skog');

figure
plot(tVec, errVelEKF, 'b-'); hold on;
plot(tVec, errVelSkog, 'r-');
xlabel('Time (s)'); ylabel('Velocity error (m/s)')
title('Error in velocity estimations');
legend('EKF', 'Skog');

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