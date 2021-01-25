function results(trueTrajectory, Config, estIMU, pGNSS, xEKF, PEKF, xSkog, PSkog, estEKF, estSkog, estSkogPresent, PSkogPresent)
close all;
global COL_TRAJECTORY_TIME COL_TRAJECTORY_POS COL_TRAJECTORY_VEL
global COL_EST_POS COL_EST_VEL COL_EST_ACCBIAS COL_EST_DELAY

%% Plots configuration
lineWidth= 2;
figSize = [0.8 0.8];
save = 0;
savePath = '../Figures_auto/results_05';
format = '.png'; %'.fig'
if ~exist(savePath, 'dir'), mkdir(savePath); end

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
errBiasEKF  = Config.biasMeasAcc - estEKF(:, COL_EST_ACCBIAS);
% Skog EKF errors
errPosSkog   = trueTrajectory(:, COL_TRAJECTORY_POS) - estSkog(:, COL_EST_POS);
errVelSkog   = trueTrajectory(:, COL_TRAJECTORY_VEL) - estSkog(:, COL_EST_VEL);
errDelaySkog = Config.tDelay - estSkog(:, COL_EST_DELAY);
errBiasSkog  = Config.biasMeasAcc - estSkog(:, COL_EST_ACCBIAS);
% Skog EKF at present errors
errPosSkogPresent   = trueTrajectory(:, COL_TRAJECTORY_POS) - estSkogPresent(:, COL_EST_POS);
errVelSkogPresent   = trueTrajectory(:, COL_TRAJECTORY_VEL) - estSkogPresent(:, COL_EST_VEL);
errBiasSkogPresent  = Config.biasMeasAcc - estSkogPresent(:, COL_EST_ACCBIAS);
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

% RMSE
rmsePosIMU              = sqrt(mean(errPosIMU.^2));
rmseVelIMU              = sqrt(mean(errVelIMU.^2));

rmsePosGNSS             = sqrt(nanmean(errPosGNSS.^2));

rmsePosEKF              = sqrt(mean(errPosEKF.^2));
rmseVelEKF              = sqrt(mean(errVelEKF.^2));
rmseBiasEKF             = sqrt(mean(errBiasEKF.^2));

rmsePosSkog             = sqrt(mean(errPosSkog.^2));
rmseVelSkog             = sqrt(mean(errVelSkog.^2));
rmseBiasSkog            = sqrt(mean(errBiasSkog.^2));
rmseDelaySkog           = sqrt(mean(errDelaySkog.^2));

rmsePosSkogPresent      = sqrt(mean(errPosSkogPresent.^2));
rmseVelSkogPresent      = sqrt(mean(errVelSkogPresent.^2));
rmseBiasSkogPresent     = sqrt(mean(errBiasSkogPresent.^2));
rmseDelaySkogPresent    = sqrt(mean(errDelaySkogPresent.^2));


fprintf('\n\t\t\t\t\t ==== RMSE ==== \n');
fprintf('\t\t IMU-only \t GNSS-only \t Standard EKF \t AEKF \t\t AEKF at present \n');
fprintf('Position: \t %.4f \t %.4f \t %.4f \t %.4f \t\t %.4f \n', ...
        rmsePosIMU, rmsePosGNSS, rmsePosEKF, rmsePosSkog, rmsePosSkogPresent);
fprintf('Velocity: \t %.4f \t N/A \t\t %.4f \t %.4f \t\t %.4f \n', ...
        rmseVelIMU, rmseVelEKF, rmseVelSkog, rmseVelSkogPresent);
fprintf('Acc Bias: \t N/A \t\t N/A \t\t %.4f \t %.4f \t\t %.4f \n', ...
        rmseBiasEKF, rmseBiasSkog, rmseBiasSkogPresent);
fprintf('Delay: \t\t N/A \t\t N/A \t\t N/A \t\t %.4f \t\t %.4f \n', ...
        rmseDelaySkog, rmseDelaySkogPresent);

%% Plots
tVec        = trueTrajectory(:, COL_TRAJECTORY_TIME);
kGNSS       = Config.M:Config.M:length(tVec); % samples with GNSS measurement

% True trajectory vs measurements
f = figure('units','normalized','outerposition',[0 0 figSize]);
plot(tVec, trueTrajectory(:, COL_TRAJECTORY_POS), 'k-', 'Linewidth', lineWidth); hold on;
plot(tVec, estIMU(:, COL_EST_POS), 'm-', 'Linewidth', lineWidth); 
plot(tVec, pGNSS, 'c.');
plot(tVec, estEKF(:, COL_EST_POS), 'b-', 'Linewidth', lineWidth);
plot(tVec, estSkog(:, COL_EST_POS), 'r-', 'Linewidth', lineWidth);
plot(tVec, estSkogPresent(:, COL_EST_POS), 'g-', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Position (m)')
if ~save, title('True trajectory vs estimated'); end
legend({'True', 'IMU', 'GNSS', 'EKF', 'AEKF', 'AEKF at present'}, 'Location', 'northeastoutside');

FigName = 'trajectory';
if save, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

% True Velocity vs estimated velocity
f = figure('units','normalized','outerposition',[0 0 figSize]);
plot(tVec, trueTrajectory(:, COL_TRAJECTORY_VEL), 'k-', 'Linewidth', lineWidth); hold on;
plot(tVec, estEKF(:, COL_EST_VEL), 'b-', 'Linewidth', lineWidth);
plot(tVec, estSkog(:, COL_EST_VEL), 'r-', 'Linewidth', lineWidth);
plot(tVec, estSkogPresent(:, COL_EST_VEL), 'g-', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Velocity (m/s)')
if ~save, title('True velocity vs estimated'); end
legend({'True', 'EKF', 'AEKF', 'AEKF at present'}, 'Location', 'northeastoutside');

FigName = 'velocity';
if save, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

% Accelerometer bias estimation
f = figure('units','normalized','outerposition',[0 0 figSize]);
plot(tVec([1 end]), Config.biasMeasAcc*ones(1, 2), 'k-', 'Linewidth', lineWidth); hold on;
plot(tVec, estEKF(:, COL_EST_ACCBIAS), 'b-', 'Linewidth', lineWidth);
plot(tVec, estSkog(:, COL_EST_ACCBIAS), 'r-', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Bias (m/s^2)')
if ~save, title('Accelerometer bias estimation'); end
legend({'True', 'EKF', 'AEKF'}, 'Location', 'northeastoutside');

FigName = 'acc_bias';
if save, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

% Time Delay estimation
f = figure('units','normalized','outerposition',[0 0 figSize]);
% subplot(2,1,1)
plot(tVec([1 end]), Config.tDelay*ones(1, 2), 'k-', 'Linewidth', lineWidth); hold on;
plot(tVec, estSkog(:, COL_EST_DELAY), 'r-', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Time Delay (s)')
legend('True', 'AEKF', 'Location', 'northeastoutside');
if ~save, title('Time Delay AEKF Estimation'); end
% subplot(2,1,2)
% plot(tVec, errDelaySkog);
% xlabel('Time (s)'); ylabel('Delay error (s)');
% title('Time delay estimation error');

FigName = 'delay';
if save, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

strColor1 = '#4DBEEE';
color1 = sscanf(strColor1(2:end),'%2x%2x%2x',[1 3])/255;
strColor2 = '#D95319';
color2 = sscanf(strColor2(2:end),'%2x%2x%2x',[1 3])/255;
strColor3 = '#77AC30';
color3 = sscanf(strColor3(2:end),'%2x%2x%2x',[1 3])/255;

% Position Estimation Error plot
f = figure('units','normalized','outerposition',[0 0 figSize]);
subplot(2,1,1)
p1 = plot(tVec, errPosEKF, 'b-', 'Linewidth', lineWidth); hold on;
p2 = plot(tVec, errPosSkog, 'r-', 'Linewidth', lineWidth);
p21 = plot(tVec, errPosSkogPresent, 'g-', 'Linewidth', lineWidth);
% yline(mean(errPosEKF), 'k');
p3 = plot(tVec, 3*sigmaPosEKF, 'Color', color1, 'Linewidth', lineWidth);
p4 = plot(tVec, -3*sigmaPosEKF, 'Color', color1, 'Linewidth', lineWidth);
p5 = plot(tVec, 3*sigmaPosSkog, 'Color', color2, 'Linewidth', lineWidth);
p6 = plot(tVec, -3*sigmaPosSkog, 'Color', color2, 'Linewidth', lineWidth);
p7 = plot(tVec, 3*sigmaPosSkogPresent, 'Color', color3, 'Linewidth', lineWidth);
p8 = plot(tVec, -3*sigmaPosSkogPresent, 'Color', color3, 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Position error (m)')
if ~save, title('Error in position estimations'); end
h = [p1 p2 p21 p3 p5 p7];
hleg1 = legend(h,{'EKF', 'AEKF', 'AEKF at present','3\sigma EKF', '3\sigma AEKF', '3\sigma AEKF at present'}, 'Location', 'northeastoutside');
poshleg1 = get(hleg1, 'Position'); 
hsp1 = get(gca, 'Position');                    % Get 'Position' for (2,1,1)

subplot(2,1,2)
plot(tVec, errPosIMU, 'm-', 'Linewidth', lineWidth); hold on;
plot(tVec, errPosGNSS, 'c.', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Position error (m)')
if ~save, title('IMU-only and GNSS-only Position error'); end
hleg2 = legend({'IMU-only', 'GNSS-only'}, 'Location', 'northeastoutside');
poshleg2 = get(hleg2, 'Position'); 
set(hleg2, 'Position', [poshleg1(1) poshleg2(2) poshleg1(3) poshleg2(4)])
hsp2 = get(gca, 'Position');                    % Get 'Position' for (2,1,2)
set(gca, 'Position', [hsp2(1:2)  hsp1(3:4)])      % Use (2,1,1) size for (2,1,2)

FigName = 'pos_error';
if save, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

% Velocity Estimation Error plot
f = figure('units','normalized','outerposition',[0 0 figSize]);
subplot(2,1,1)
p1 = plot(tVec, errVelEKF, 'b-', 'Linewidth', lineWidth); hold on;
p2 = plot(tVec, errVelSkog, 'r-', 'Linewidth', lineWidth);
p21 = plot(tVec, errVelSkogPresent, 'g-', 'Linewidth', lineWidth);
% yline(mean(errVelEKF), 'k');
p3 = plot(tVec, 3*sigmaVelEKF, 'Color', color1, 'Linewidth', lineWidth);
p4 = plot(tVec, -3*sigmaVelEKF, 'Color', color1, 'Linewidth', lineWidth);
p5 = plot(tVec, 3*sigmaVelSkog, 'Color', color2, 'Linewidth', lineWidth);
p6 = plot(tVec, -3*sigmaVelSkog, 'Color', color2, 'Linewidth', lineWidth);
p7 = plot(tVec, 3*sigmaVelSkogPresent, 'Color', color3, 'Linewidth', lineWidth);
p8 = plot(tVec, -3*sigmaVelSkogPresent, 'Color', color3, 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Velocity error (m/s)')
if ~save, title('Error in velocity estimations'); end
h = [p1 p2 p21 p3 p5, p7];
hleg1 = legend(h,{'EKF', 'AEKF', 'AEKF at present','3\sigma EKF', '3\sigma AEKF', '3\sigma AEKF at present'}, 'Location', 'northeastoutside');
poshleg1 = get(hleg1, 'Position'); 
hsp1 = get(gca, 'Position');                    % Get 'Position' for (2,1,1)

subplot(2,1,2)
plot(tVec, errVelIMU, 'm-', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Velocity error (m/s)')
if ~save, title('IMU-only Velocity error'); end
hleg2 = legend({'IMU-only'}, 'Location', 'northeastoutside');
poshleg2 = get(hleg2, 'Position'); 
set(hleg2, 'Position', [poshleg1(1) poshleg2(2) poshleg1(3) poshleg2(4)]) 
hsp2 = get(gca, 'Position');                    % Get 'Position' for (2,1,2)
set(gca, 'Position', [hsp2(1:2)  hsp1(3:4)])    % Use (2,1,1) size for (2,1,2)

FigName = 'vel_error';
if save, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

% Position error CDF
c = ['b', 'r', 'g'];
f = figure('units','normalized','outerposition',[0 0 figSize]);
h(1) = cdfplot(abs(errPosEKF)); hold on;
h(2) = cdfplot(abs(errPosSkog));
h(3) = cdfplot(abs(errPosSkogPresent));
for i = 1:3, set(h(i), 'Linewidth', lineWidth, 'Color', c(i)); end
xlabel('Position error (m)'); ylabel('Cumulative frequency');
legend({'EKF', 'AEKF', 'AEKF at present'}, 'Location', 'northeastoutside');
title('');
FigName = 'pos_cdf';
if save, saveas(f, fullfile(savePath, [FigName, format])); end

% Velocity error CDF
f = figure('units','normalized','outerposition',[0 0 figSize]);
h(1) = cdfplot(abs(errVelEKF)); hold on;
h(2) = cdfplot(abs(errVelSkog));
h(3) = cdfplot(abs(errVelSkogPresent));
for i = 1:3, set(h(i), 'Linewidth', lineWidth, 'Color', c(i)); end
xlabel('Velocity error (m)'); ylabel('Cumulative frequency');
legend({'EKF', 'AEKF', 'AEKF at present'}, 'Location', 'northeastoutside');
title('');
FigName = 'vel_cdf';
if save, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

end


%% Figures not used

% figure;
% plot(tVec(kGNSS), xEKF(COL_EST_ACCBIAS, kGNSS), 'b-'); hold on;
% plot(tVec(kGNSS), xSkog(COL_EST_ACCBIAS, kGNSS), 'r-');
% xlabel('Time (s)'); ylabel('Error Bias (m/s^2)')
% title('Accelerometer bias error-state EKF');
% legend('EKF', 'AEKF', 'Location', 'northeastoutside');