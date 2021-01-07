clear; close all; clc;
addpath(genpath('./'));
% Time Synchronization GNSS-aided IMU project
% (Description of the project here)

Config = loadConfigFile();
load ('Reference.mat');

%% Generation of true trajectory
[tspan, trueTrajectory] = generateTrajectory(Config); % trueTrajectory: [posN, posE, vel, acc, heading, headingRate]

%% Generation of measurements and EKF
[estIMURaw, estIntEKF, pGNSS, PEKF, estIntSkogHistoric, estIntSkogAtDelay, PSkogHistoric, PSkogAtDelay, measAcc, measGyro, xEKF, estIntPredPos] = ...
            simulateEstimations(trueTrajectory, tspan, Config);
        
%% Results
tVec        = 0:Config.tIMU:Config.tEnd;
% Computation of errors
% errPosIMU   = trueTrajectory(:, 1:2) - estIMURaw.pos;        % errPosIMU: [errorNorth, errorEast]
% errVelIMU   = trueTrajectory(:, 3) - estIMURaw.vel;             
% errPosGNSS  = trueTrajectory(:, 1:2) - pGNSS;       % errPosGNSS: [errorNorth, errorEast]
% errPosEKF   = trueTrajectory(:, 1:2) - estIntEKF.pos;    % errPosEKF: [errorNorth, errorEast]
% errVelEKF   = trueTrajectory(:, 3) - estIntEKF.vel;
% errPosSkog  = trueTrajectory(:, 1:2) - estIntSkogHistoric.pos;    % errPosEKF: [errorNorth, errorEast]
% errVelSkog  = trueTrajectory(:, 3) - estIntSkogHistoric.vel;

errPosIMU   = ned_Ref(1:240, 1:2) - estIMURaw.pos(Config.M:Config.M:end,:);        % errPosIMU: [errorNorth, errorEast]
% errVelIMU   = trueTrajectory(:, 3) - estIMURaw.vel;             
errPosGNSS  = ned_Ref(1:240, 1:2) - pGNSS(Config.M:Config.M:end,:);       % errPosGNSS: [errorNorth, errorEast]
errPosEKF   = ned_Ref(1:240, 1:2) - estIntEKF.pos(Config.M:Config.M:end,:);    % errPosEKF: [errorNorth, errorEast]
% errVelEKF   = trueTrajectory(:, 3) - estIntEKF.vel;
errPosSkog  = ned_Ref(1:240, 1:2) - estIntSkogHistoric.pos(Config.M:Config.M:end,:);    % errPosEKF: [errorNorth, errorEast]
% errVelSkog  = trueTrajectory(:, 3) - estIntSkogHistoric.vel;
errEstIntPredPos = ned_Ref(1:240, 1:2) - estIntPredPos(Config.M:Config.M:end,:); 

errEucliEKF = sqrt( errPosEKF(:, 1).^2 + errPosEKF(:, 2).^2 );
errEucliSkog = sqrt( errPosSkog(:, 1).^2 + errPosSkog(:, 2).^2 );

% Variances from covariance matrices
varsPosEKF      = [permute(PEKF(1, 1, :), [3 1 2]), permute(PEKF(1, 1, :), [3 1 2])]; % [varNorth, varEast]
varVelEKF       = permute(PEKF(3, 3, :), [3 1 2]);
varHeadEKF      = permute(PEKF(4, 4, :), [3 1 2]);
varBiasAccEKF   = permute(PEKF(5, 5, :), [3 1 2]);
varBiasGyroEKF  = permute(PEKF(6, 6, :), [3 1 2]);

varsPosSkog     = [permute(PSkogHistoric(1, 1, :), [3 1 2]), permute(PSkogHistoric(1, 1, :), [3 1 2])]; % [varNorth, varEast]
varVelSkog      = permute(PSkogHistoric(3, 3, :), [3 1 2]);
varHeadSkog     = permute(PSkogHistoric(4, 4, :), [3 1 2]);
varBiasAccSkog  = permute(PSkogHistoric(5, 5, :), [3 1 2]);
varBiasGyroSkog = permute(PSkogHistoric(6, 6, :), [3 1 2]);
varDelaySkog    = permute(PSkogHistoric(7, 7, :), [3 1 2]);

stdEucliEKF = sqrt( abs(varsPosEKF(:, 1) + varsPosEKF(:, 2)));
stdEucliSkog = sqrt( abs(varsPosSkog(:, 1) + varsPosSkog(:, 2)) );

%% Plots
dir='../Figures/BE Inertial Data/Config24';
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
% figure;
% plot(trueTrajectory(:, 2), trueTrajectory(:, 1), '.k'); hold on;
% plot(estIMURaw.pos(:, 2), estIMURaw.pos(:, 1), 'b.'); 
% plot(pGNSS(:, 2), pGNSS(:, 1), 'r.');
% xlabel('East (m)'); ylabel('North (m)');
% title('True trajectory');
% legend('True', 'IMU', 'GNSS');

figure;
plot(ned_Ref(1:241, 2), ned_Ref(1:241, 1), '.k'); hold on;
plot(estIMURaw.pos(:, 2), estIMURaw.pos(:, 1), 'b.'); 
plot(pGNSS(:, 2), pGNSS(:, 1), 'r.');
xlabel('East (m)'); ylabel('North (m)');
title('True trajectory');
legend('True', 'IMU', 'GNSS');
% IMU & GNSS Position error comparison
% figure
% plot(tVec, abs(errPosIMU(:, 1)), 'b-'); hold on
% plot(tVec, abs(errPosIMU(:, 2)), 'g-');
% plot(tVec, abs(errPosGNSS(:, 1)), 'r.');
% plot(tVec, abs(errPosGNSS(:, 2)), 'm.');
% xlabel('Time (s)'); ylabel('Absolute position error (m)')
% legend('IMU North', 'IMU East', 'GNSS North', 'GNSS East');
% title('IMU-only & GNSS-only Position error');
figure;
plot(errPosIMU(:, 1), 'b-'); hold on
plot(errPosIMU(:, 2), 'g-');
%plot(abs(errPosGNSS(:, 1)), 'r.');
%plot(abs(errPosGNSS(:, 2)), 'm.');
xlabel('Time (s)'); ylabel('Position error (m)')
legend('IMU North', 'IMU East');%, 'GNSS North', 'GNSS East');
title('IMU-only Position error');
% filename = ['IMU_only_errors','.png'];
% dirFig = fullfile(dir, filename);
% saveas(2,dirFig)

figure;
plot(xEKF(1,:), 'b-'); hold on;
plot(xEKF(2,:), 'g-');
xlabel('Time (s)'); ylabel('Accumulated position error (m)')
legend('EKF North', 'EKF East');
title('EKF State Vector Accumulated Position error');
% filename = ['EKF_stateVector_posErrors_accumulated','.png'];
% dirFig = fullfile(dir, filename);
% saveas(3,dirFig)

% % IMU Velocity error plot
% figure
% plot(tVec, errVelIMU, 'b-'); hold on
% xlabel('Time (s)'); ylabel('Velocity error (m/s)')
% title('IMU-only Velocity error');


% True trajectory vs estimated trajectory
% figure;
% plot(trueTrajectory(:, 2), trueTrajectory(:, 1), '.k'); hold on;
% plot(estIntEKF.pos(:, 2), estIntEKF.pos(:, 1), 'b-'); 
% plot(estIntSkogHistoric.pos(:, 2), estIntSkogHistoric.pos(:, 1), 'r-'); 
% xlabel('East (m)'); ylabel('North (m)');
% title('True position vs estimations');
% legend('True', 'EKF', 'Skog');

figure;
plot(ned_Ref(1:241, 2), ned_Ref(1:241, 1), '.k'); hold on;
plot(estIntEKF.pos(:, 2), estIntEKF.pos(:, 1), 'b-'); 
plot(pGNSS(:, 2), pGNSS(:, 1), 'r.');
%plot(estIntSkogHistoric.pos(:, 2), estIntSkogHistoric.pos(:, 1), 'r-'); 
xlabel('East (m)'); ylabel('North (m)');
title('True position vs estimations');
legend('True', 'EKF')%;, 'Skog');
% filename = ['truePos_vs_estimatedPos','.png'];
% dirFig = fullfile(dir, filename);
% saveas(4,dirFig)

% EKF Position error
% figure
% plot(tVec, abs(errPosEKF(:, 1)), 'b-', 'Linewidth', 1.5); hold on
% plot(tVec, abs(errPosEKF(:, 2)), 'g-', 'Linewidth', 1.5);
% plot(tVec, abs(errPosSkog(:, 1)), 'r-', 'Linewidth', 1.5);
% plot(tVec, abs(errPosSkog(:, 2)), 'm-', 'Linewidth', 1.5);
% xlabel('Time (s)'); ylabel('Absolute position error (m)')
% legend('EKF North', 'EKF East', 'SKOG North', 'SKOG East');
% title('EKF Position error');

% EKF Position error
figure
plot(errPosEKF(:, 1), 'b-', 'Linewidth', 1.5); hold on
plot(errPosEKF(:, 2), 'g-', 'Linewidth', 1.5);
plot(errPosGNSS(:, 1), 'r-', 'Linewidth', 1.5);
plot(errPosGNSS(:, 2), 'm-', 'Linewidth', 1.5);
%plot(abs(errPosSkog(:, 1)), 'r-', 'Linewidth', 1.5);
%plot(abs(errPosSkog(:, 2)), 'm-', 'Linewidth', 1.5);
xlabel('Time (s)'); ylabel('Position error (m)')
legend('EKF North', 'EKF East', 'GNSS North', 'GNSS East');
title('EKF Position error');
% filename = ['EKF_estimatedPos_errors','.png'];
% dirFig = fullfile(dir, filename);
% saveas(5,dirFig)

figure; 
subplot(2,1,1), plot(errPosEKF(:,1), 'b', 'LineWidth',2), hold on, grid on, 
plot(3*sqrt(varsPosEKF(Config.M:Config.M:end,1)), 'r', 'LineWidth',2);
plot(-3*sqrt(varsPosEKF(Config.M:Config.M:end,1)), 'r', 'LineWidth',2);
xlabel('Time [s]'); ylabel('[m]'); legend('Estimation error','3\sigma envelop');
title('Estimation error (North axis)');
subplot(2,1,2), plot(errPosEKF(:,2), 'b', 'LineWidth',2), hold on, grid on, 
plot(3*sqrt(varsPosEKF(Config.M:Config.M:end,2)), 'r', 'LineWidth',2);
plot(-3*sqrt(varsPosEKF(Config.M:Config.M:end,2)), 'r', 'LineWidth',2);
xlabel('Time [s]'); ylabel('[m]'); legend('Estimation error','3\sigma envelop');
title('Estimation error (East axis)');
% filename = ['EKF_estimatedPos_errors_3sigmaCovEKF','.png'];
% dirFig = fullfile(dir, filename);
% saveas(6,dirFig)
% figure
% plot(xEKF(:, 1), 'b-'); hold on
% plot(xEKF(:, 2), 'g-');
% %plot(abs(errPosGNSS(:, 1)), 'r.');
% %plot(abs(errPosGNSS(:, 2)), 'm.');
% xlabel('Time (s)'); ylabel('EKF Position error (m)')
% legend('IMU North', 'IMU East');%, 'GNSS North', 'GNSS East');
% title('EKF Position error');
figure;
plot(errEucliEKF, 'b-', 'Linewidth', 1.5); hold on
plot(sqrt(errEstIntPredPos(:,1).^2 + errEstIntPredPos(:,2).^2), 'r-', 'Linewidth', 1.5);
plot(sqrt(xEKF(1,Config.M:Config.M:end).^2 + xEKF(2,Config.M:Config.M:end).^2), 'g-', 'Linewidth', 1.5);
%plot(tVec, errEucliSkog, 'r-', 'Linewidth', 1.5);
xlabel('Time (s)'); ylabel('Position error (m)')
legend('EKF estimation error', 'INS error before update', 'Error-state vector (pos norm)');%, 'SKOG error');
title('EKF Euclidean Position error');
% filename = ['EKF_euclidean_pos_error','.png'];
% dirFig = fullfile(dir, filename);
% saveas(7,dirFig)

% figure;
% plot(sqrt(xEKF(1,Config.M:Config.M:end).^2 + xEKF(2,Config.M:Config.M:end).^2), 'b-', 'Linewidth', 1.5);
% %plot(tVec, errEucliSkog, 'r-', 'Linewidth', 1.5);
% xlabel('Time (s)'); ylabel('Position error (m)')
% legend('EKF error');%, 'SKOG error');
% title('EKF Norm EKF Error State Vector (Position)');
% filename = ['EKF_norm_stateVectorErrors_GPSavailable','.png'];
% dirFig = fullfile(dir, filename);
% saveas(8,dirFig)

figure;
plot(sqrt(xEKF(1,:).^2 + xEKF(2,:).^2), 'b-', 'Linewidth', 1.5); hold on
%plot(tVec, errEucliSkog, 'r-', 'Linewidth', 1.5);
xlabel('Time (s)'); ylabel('Position error (m)')
legend('EKF error');%, 'SKOG error');
title('EKF Norm EKF Error State Vector (Position)');
% filename = ['EKF_norm_stateVectorErrors','.png'];
% dirFig = fullfile(dir, filename);
% saveas(9,dirFig)

% figure;
% plot(sqrt(errEstIntPredPos(:,1).^2 + errEstIntPredPos(:,2).^2), 'b-', 'Linewidth', 1.5);
% xlabel('Time (s)'); ylabel('Position error (m)')
% title('INS error before update');


% 
% figure
% plot(tVec, estIntSkogHistoric.timeDelay, 'b-', 'Linewidth', 1.5);
% xlabel('Time (s)'); ylabel('Time delay (s)')
% title('Time delay estimation');
% 
% % True Velocity vs estimated velocity
% % figure;
% % plot(tVec, trueTrajectory(:, 3), 'k-', 'Linewidth', 1); hold on;
% % plot(tVec, estIntEKF.vel, 'b-'); 
% % plot(tVec, estIntSkogHistoric.vel, 'r-'); 
% % xlabel('Time (s)'); ylabel('Velocity (m/s)')
% % title('True velocity vs estimations');
% % legend('True', 'EKF', 'Skog');
% 
% % True heading vs estimated heading
% % figure;
% % plot(tVec, rad2deg(trueTrajectory(:, 5)), 'k-', 'Linewidth', 1); hold on;
% % plot(tVec, rad2deg(estIntEKF.heading), 'b-'); 
% % plot(tVec, rad2deg(estIntSkogHistoric.heading), 'r-'); 
% % % xlabel('Time (s)'); ylabel('Heading (deg)')
% % % title('True heading vs estimations');
% % % legend('True', 'EKF', 'Skog');
% % 
% % % Position STD
% % figure
% % plot(tVec, stdEucliEKF, 'b-', 'Linewidth', 1.5); hold on
% % plot(tVec, stdEucliSkog, 'r-', 'Linewidth', 1.5);
% % xlabel('Time (s)'); ylabel('Position STD (m)')
% % legend('EKF \sigma_P', 'SKOG \sigma_P');
% % title('EKF position STD');
% % 
% % % Velocity STD
% % figure
% % plot(tVec, sqrt(abs(varVelEKF)), 'b-', 'Linewidth', 1.5); hold on
% % plot(tVec, sqrt(abs(varVelSkog)), 'r-', 'Linewidth', 1.5);
% % xlabel('Time (s)'); ylabel('Velocity STD (m/s)')
% % legend('EKF \sigma_V', 'SKOG \sigma_V');
% % title('EKF velocity STD');
% % 
% % % Heading STD
% % figure
% % plot(tVec, rad2deg(sqrt(varHeadEKF)), 'b-', 'Linewidth', 1.5); hold on
% % plot(tVec, sqrt(varHeadSkog), 'r-', 'Linewidth', 1.5);
% % xlabel('Time (s)'); ylabel('Heading STD (deg)')
% % legend('EKF \sigma_{\phi}', 'SKOG \sigma_{\phi}');
% % title('EKF heading STD');
% % 
% % % Delay STD
% % figure
% % plot(tVec, sqrt(varDelaySkog), 'r-', 'Linewidth', 1.5);
% % xlabel('Time (s)'); ylabel('Delay STD (s)')
% % legend('EKF \sigma_{Td}');
% % title('EKF Delay STD');
% 
% figure;
% plot(tVec, estIntEKF.biasAcc, 'b-'); hold on; 
% %plot(tVec, estIntSkogHistoric.biasAcc, 'r-')
% xlabel('Time (s)'); ylabel('Acceleration bias (m/s^2)')
% title('Accelerometer Bias estimation');
% %legend('EKF', 'Skog');
% 
% figure;
% plot(tVec, rad2deg(estIntEKF.biasGyro), 'b-'); hold on;
% plot(tVec, rad2deg(estIntSkogHistoric.biasGyro), 'r-')
% xlabel('Time (s)'); ylabel('Gyroscope bias (deg/s)')
% title('Gyroscope Bias estimation');
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