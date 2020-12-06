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
[pIMU, vIMU, pGNSS, rIntEKF, PEKF, rIntSkog, PSkog] = simulateEstimations(          ...
                                                        p, tspan, M, tIMU, tDelay,  ...
                                                        p0, v0, a0,                 ...
                                                        rp0, rv0, rba0, rt0,        ...
                                                        sigmaGNSS, sigmaAcc,        ...
                                                        sigmaPos, sigmaVel, sigmaAccBias, sigmaTd);

%% Results
tVec        = 0:tIMU:tEnd;
pIntEKF     = rIntEKF(1, :);
vIntEKF     = rIntEKF(2, :);
bIntEKF     = rIntEKF(3, :);
pIntSkog    = rIntSkog(1, :);
vIntSkog    = rIntSkog(2, :);
bIntSkog    = rIntSkog(3, :);
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
plot(tVec./1e3, p, 'k-', 'Linewidth', 1); hold on;
plot(tVec./1e3, pIMU, 'b.'); 
plot(tVec./1e3, pGNSS, 'r.');
xlabel('Time (s)'); ylabel('Position (m)')
title('True trajectory');
legend('True', 'IMU', 'GNSS');

% IMU Position error plot
figure
plot(tVec/1e3, errPosIMU, 'b-'); hold on
plot(tVec/1e3, errPosGNSS, 'r.');
xlabel('Time (s)'); ylabel('Position error (m)')
legend('IMU', 'GNSS');
title('IMU-only & GNSS-only Position error');

% IMU Velocity error plot
figure
plot(tVec/1e3, errVelIMU*1e3, 'b-'); hold on
xlabel('Time (s)'); ylabel('Velocity error (m)')
title('IMU-only Velocity error');

% True trajectory vs estimated trajectory
figure;
plot(tVec./1e3, p, 'k-', 'Linewidth', 1); hold on;
plot(tVec./1e3, pIntEKF, 'b-'); 
plot(tVec./1e3, pIntSkog, 'r-'); 
xlabel('Time (s)'); ylabel('Position (m)')
title('True position vs estimations');
legend('True', 'EKF', 'Skog');

% True Velocity vs estimated velocity
figure;
plot(tVec./1e3, v*1e3, 'k-', 'Linewidth', 1); hold on;
plot(tVec./1e3, vIntEKF*1e3, 'b-'); 
plot(tVec./1e3, vIntSkog*1e3, 'r-'); 
xlabel('Time (s)'); ylabel('Velocity (m/s)')
title('True velocity vs estimations');
legend('True', 'EKF', 'Skog');

figure;
plot(tVec./1e3, bIntEKF*1e6, 'b-'); hold on;
plot(tVec./1e3, bIntSkog*1e6, 'r-'); 
xlabel('Time (s)'); ylabel('Acceleration bias (m/s^2)')
title('Standard EKF method');
legend('EKF', 'Skog');

% Estimation Error plot
figure
plot(tVec./1e3, errPosEKF, 'b-'); hold on;
plot(tVec./1e3, errPosSkog, 'r-'); 
xlabel('Time (s)'); ylabel('Position error (m)')
title('Error in position estimations');
legend('EKF', 'Skog');

figure
plot(tVec./1e3, errVelEKF*1e3, 'b-'); hold on;
plot(tVec./1e3, errVelSkog*1e3, 'r-');
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