clear; close all; clc;
addpath(genpath('./'));
% Time Synchronization GNSS-aided IMU project
% (Description of the project here)

Config = loadConfigFile();

%% Generation of true trajectory
[tspan, trueTrajectory] = generateTrajectory(Config); % trueTrajectory: [posN, posE, vel, acc, heading, headingRate]

%% Generation of measurements and EKF
[estIMURaw, estIntEKF, pGNSS, PEKF, estIntSkogPresent, estIntSkogPast, PSkogPresent, PSkogPast] = ...
            simulateEstimations(trueTrajectory, tspan, Config);
        
%% Results
tVec        = 0:Config.tIMU:Config.tEnd;
% Computation of errors
errPosIMU   = trueTrajectory(:, 1:2) - estIMURaw.pos;        % errPosIMU: [errorNorth, errorEast]
errVelIMU   = trueTrajectory(:, 3) - estIMURaw.vel;             
errPosGNSS  = trueTrajectory(:, 1:2) - pGNSS;       % errPosGNSS: [errorNorth, errorEast]
errPosEKF   = trueTrajectory(:, 1:2) - estIntEKF.pos;    % errPosEKF: [errorNorth, errorEast]
errVelEKF   = trueTrajectory(:, 3) - estIntEKF.vel;
errPosSkog  = trueTrajectory(:, 1:2) - estIntSkogPresent.pos;    % errPosEKF: [errorNorth, errorEast]
errVelSkog  = trueTrajectory(:, 3) - estIntSkogPresent.vel;

errEucliEKF = sqrt( errPosEKF(:, 1).^2 + errPosEKF(:, 2).^2 );
errEucliSkog = sqrt( errPosSkog(:, 1).^2 + errPosSkog(:, 2).^2 );

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
plot(trueTrajectory(:, 2), trueTrajectory(:, 1), '.k'); hold on;
plot(estIMURaw.pos(:, 2), estIMURaw.pos(:, 1), 'b.'); 
plot(pGNSS(:, 2), pGNSS(:, 1), 'r.');
xlabel('East (m)'); ylabel('North (m)');
title('True trajectory');
legend('True', 'IMU', 'GNSS');

% IMU & GNSS Position error comparison
figure
plot(tVec, errPosIMU(:, 1), 'b-'); hold on
plot(tVec, errPosIMU(:, 2), 'g-');
plot(tVec, errPosGNSS(:, 1), 'r.');
plot(tVec, errPosGNSS(:, 2), 'm.');
xlabel('Time (s)'); ylabel('Position error (m)')
legend('IMU North', 'IMU East', 'GNSS North', 'GNSS East');
title('IMU-only & GNSS-only Position error');


% % IMU Velocity error plot
% figure
% plot(tVec, errVelIMU, 'b-'); hold on
% xlabel('Time (s)'); ylabel('Velocity error (m/s)')
% title('IMU-only Velocity error');


% True trajectory vs estimated trajectory
figure;
plot(trueTrajectory(:, 2), trueTrajectory(:, 1), '.k'); hold on;
plot(estIntEKF.pos(:, 2), estIntEKF.pos(:, 1), 'r-'); 
plot(estIntSkogPresent.pos(:, 2), estIntSkogPresent.pos(:, 1), 'b-'); 
xlabel('East (m)'); ylabel('North (m)');
title('True position vs estimations');
legend('True', 'EKF', 'Skog');

% EKF Position error
figure
plot(tVec, abs(errPosEKF(:, 1)), 'b-', 'Linewidth', 1.5); hold on
plot(tVec, abs(errPosEKF(:, 2)), 'g-', 'Linewidth', 1.5);
plot(tVec, abs(errPosSkog(:, 1)), 'r-', 'Linewidth', 1.5);
plot(tVec, abs(errPosSkog(:, 2)), 'm-', 'Linewidth', 1.5);
xlabel('Time (s)'); ylabel('Position error (m)')
legend('EKF North', 'EKF East', 'SKOG North', 'SKOG East');
title('EKF Position error');

figure
plot(tVec, errEucliEKF, 'b-', 'Linewidth', 1.5); hold on
plot(tVec, errEucliSkog, 'r-', 'Linewidth', 1.5);
xlabel('Time (s)'); ylabel('Position error (m)')
legend('EKF error', 'SKOG error');
title('EKF Position error');

figure
plot(tVec, estIntSkogPresent.timeDelay, 'b-', 'Linewidth', 1.5);
xlabel('Time (s)'); ylabel('Time delay (s)')
title('Time delay estimation');
% % True Velocity vs estimated velocity
% figure;
% plot(tVec, v, 'k-', 'Linewidth', 1); hold on;
% plot(tVec, vIntEKF, 'b-'); 
% plot(tVec, vIntSkog, 'r-'); 
% xlabel('Time (s)'); ylabel('Velocity (m/s)')
% title('True velocity vs estimations');
% legend('True', 'EKF', 'Skog');
% 
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



% ------------------------------------------------------------------------

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