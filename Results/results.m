function results(trueTrajectory, Config, estIMU, pGNSS, xEKF, PEKF, xSkog, PSkog, estEKF, estSkog, estSkogPresent, PSkogPresent)
close all;
global COL_TRAJECTORY_TIME COL_TRAJECTORY_POS COL_TRAJECTORY_VEL COL_TRAJECTORY_ACC COL_TRAJECTORY_BIASACC COL_TRAJECTORY_DELAY
global COL_EST_POS COL_EST_VEL COL_EST_ACCBIAS COL_EST_DELAY

%% Initializations
savePlots = 0; % boolean 1 to save
showTitle = 1; % boolean 1 to show

convergenceTime = 120; %s
kConv = convergenceTime/Config.tIMU;

%% Plots configuration
lineWidth = 2;
% saveName = 'results_0-1';
saveName = ['results_' strrep(num2str(Config.tDelay), '.', '-')];
plotFontSize = 22;
figSize = [0.8 0.8];
savePath = ['../Figures_auto/' saveName];
format = '.png'; %'.fig'
if ~exist(savePath, 'dir'), mkdir(savePath); end

%% Save data
if savePlots
    save(['Results/' saveName '.mat'],'trueTrajectory', 'Config', 'estIMU', 'pGNSS', 'xEKF', ...
        'PEKF', 'xSkog', 'PSkog', 'estEKF', 'estSkog', 'estSkogPresent', 'PSkogPresent');
end
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
errBiasEKF  = trueTrajectory(:, COL_TRAJECTORY_BIASACC) - estEKF(:, COL_EST_ACCBIAS);
% Skog EKF errors
errPosSkog   = trueTrajectory(:, COL_TRAJECTORY_POS) - estSkog(:, COL_EST_POS);
errVelSkog   = trueTrajectory(:, COL_TRAJECTORY_VEL) - estSkog(:, COL_EST_VEL);
errBiasSkog  = trueTrajectory(:, COL_TRAJECTORY_BIASACC) - estSkog(:, COL_EST_ACCBIAS);
errDelaySkog = trueTrajectory(:, COL_TRAJECTORY_DELAY) - estSkog(:, COL_EST_DELAY);
% Skog EKF at present errors
errPosSkogPresent   = trueTrajectory(:, COL_TRAJECTORY_POS) - estSkogPresent(:, COL_EST_POS);
errVelSkogPresent   = trueTrajectory(:, COL_TRAJECTORY_VEL) - estSkogPresent(:, COL_EST_VEL);
errBiasSkogPresent  = trueTrajectory(:, COL_TRAJECTORY_BIASACC) - estSkogPresent(:, COL_EST_ACCBIAS);
errDelaySkogPresent = trueTrajectory(:, COL_TRAJECTORY_DELAY) - estSkogPresent(:, COL_EST_DELAY);

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
rmsePosIMU              = sqrt(mean(errPosIMU(kConv:end).^2));
rmseVelIMU              = sqrt(mean(errVelIMU(kConv:end).^2));

rmsePosGNSS             = sqrt(nanmean(errPosGNSS(kConv:end).^2));

rmsePosEKF              = sqrt(mean(errPosEKF(kConv:end).^2));
rmseVelEKF              = sqrt(mean(errVelEKF(kConv:end).^2));
rmseBiasEKF             = sqrt(mean(errBiasEKF(kConv:end).^2));

rmsePosSkog             = sqrt(mean(errPosSkog(kConv:end).^2));
rmseVelSkog             = sqrt(mean(errVelSkog(kConv:end).^2));
rmseBiasSkog            = sqrt(mean(errBiasSkog(kConv:end).^2));
rmseDelaySkog           = sqrt(mean(errDelaySkog.^2));

rmsePosSkogPresent      = sqrt(mean(errPosSkogPresent(kConv:end).^2));
rmseVelSkogPresent      = sqrt(mean(errVelSkogPresent(kConv:end).^2));
rmseBiasSkogPresent     = sqrt(mean(errBiasSkogPresent(kConv:end).^2));
rmseDelaySkogPresent    = sqrt(mean(errDelaySkogPresent(kConv:end).^2));


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

f = figure('DefaultAxesFontSize',plotFontSize , 'units','normalized','outerposition',[0 0 figSize]);
subplot(3,1,1)
plot(tVec, trueTrajectory(:, COL_TRAJECTORY_POS), 'k-', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Position (m)')
if showTitle, title('True trajectory'); end
subplot(3,1,2)
plot(tVec, trueTrajectory(:, COL_TRAJECTORY_VEL), 'k-', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Velocity (m/s)')
if showTitle, title('True velocity'); end
subplot(3,1,3)
plot(tVec, trueTrajectory(:, COL_TRAJECTORY_ACC), 'k-', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)')
if showTitle, title('True acceleration'); end

FigName = 'true_trajectory';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

% True trajectory vs measurements
f = figure('DefaultAxesFontSize',plotFontSize , 'units','normalized','outerposition',[0 0 figSize]);
plot(tVec, trueTrajectory(:, COL_TRAJECTORY_POS), 'k-', 'Linewidth', lineWidth); hold on;
plot(tVec, estIMU(:, COL_EST_POS), 'm-', 'Linewidth', lineWidth); 
plot(tVec, pGNSS, 'c.');
plot(tVec, estEKF(:, COL_EST_POS), 'b-', 'Linewidth', lineWidth);
plot(tVec, estSkog(:, COL_EST_POS), 'r-', 'Linewidth', lineWidth);
plot(tVec, estSkogPresent(:, COL_EST_POS), 'g-', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Position (m)')
if showTitle, title('True trajectory vs estimated'); end
legend({'True', 'IMU', 'GNSS', 'EKF', 'AEKF', 'AEKF at present'}, 'Location', 'northeastoutside');

FigName = 'trajectory';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

% True Velocity vs estimated velocity
f = figure('DefaultAxesFontSize',plotFontSize , 'units','normalized','outerposition',[0 0 figSize]);
plot(tVec, trueTrajectory(:, COL_TRAJECTORY_VEL), 'k-', 'Linewidth', lineWidth); hold on;
plot(tVec, estEKF(:, COL_EST_VEL), 'b-', 'Linewidth', lineWidth);
plot(tVec, estSkog(:, COL_EST_VEL), 'r-', 'Linewidth', lineWidth);
plot(tVec, estSkogPresent(:, COL_EST_VEL), 'g-', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Velocity (m/s)')
if showTitle, title('True velocity vs estimated'); end
legend({'True', 'EKF', 'AEKF', 'AEKF at present'}, 'Location', 'northeastoutside');


FigName = 'velocity';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

% Accelerometer bias estimation
f = figure('DefaultAxesFontSize', plotFontSize, 'units','normalized','outerposition',[0 0 figSize]);
plot(tVec, trueTrajectory(:, COL_TRAJECTORY_BIASACC), 'k-', 'Linewidth', lineWidth); hold on;
plot(tVec, estEKF(:, COL_EST_ACCBIAS), 'b-', 'Linewidth', lineWidth);
plot(tVec, estSkog(:, COL_EST_ACCBIAS), 'r-', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Bias (m/s^2)')
if showTitle, title('Accelerometer bias estimation'); end
legend({'True', 'EKF', 'AEKF'}, 'Location', 'northeastoutside');


FigName = 'acc_bias';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

% Time Delay estimation
f = figure('DefaultAxesFontSize', plotFontSize, 'units','normalized','outerposition',[0 0 figSize]);
plot(tVec, trueTrajectory(:, COL_TRAJECTORY_DELAY), 'k-', 'Linewidth', lineWidth); hold on;
plot(tVec, estSkog(:, COL_EST_DELAY), 'r-', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Time Delay (s)')
legend('True', 'AEKF', 'Location', 'northeastoutside');
if showTitle, title('Time Delay AEKF Estimation'); end


FigName = 'delay';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

% IMU-only and GNSS-only position error plots
f = figure('DefaultAxesFontSize', plotFontSize, 'units','normalized','outerposition',[0 0 figSize]);
subplot(2,1,1)
plot(tVec, errPosIMU, 'm-', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Position error (m)')
if showTitle, title('IMU-only Position error'); end
subplot(2,1,2)
plot(tVec, errPosGNSS, 'b.', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Position error (m)')
if showTitle, title('GNSS-only Position error'); end

FigName = 'pos_error_gnss-imu';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;


% Position Estimation Error plot
f = figure('DefaultAxesFontSize', plotFontSize, 'units','normalized','outerposition',[0 0 figSize]);
p1 = plot(tVec, errPosEKF, 'b-', 'Linewidth', lineWidth); hold on;
p2 = plot(tVec, errPosSkog, 'r-', 'Linewidth', lineWidth);
p21 = plot(tVec, errPosSkogPresent, 'g-', 'Linewidth', lineWidth);
% yline(mean(errPosEKF), 'k');
p3 = plot(tVec, 3*sigmaPosEKF, 'Color', '#4DBEEE', 'Linewidth', lineWidth);
p4 = plot(tVec, -3*sigmaPosEKF, 'Color', '#4DBEEE', 'Linewidth', lineWidth);
p5 = plot(tVec, 3*sigmaPosSkog, 'Color', '#D95319', 'Linewidth', lineWidth);
p6 = plot(tVec, -3*sigmaPosSkog, 'Color', '#D95319', 'Linewidth', lineWidth);
p7 = plot(tVec, 3*sigmaPosSkogPresent, 'Color', '#77AC30', 'Linewidth', lineWidth);
p8 = plot(tVec, -3*sigmaPosSkogPresent, 'Color', '#77AC30', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Position error (m)')
if showTitle, title('Error in position estimations'); end
h = [p1 p2 p21 p3 p5 p7];
hleg1 = legend(h,{'EKF', 'AEKF', 'AEKF at present','3\sigma EKF', '3\sigma AEKF', '3\sigma AEKF at present'}, 'Location', 'northeastoutside');

FigName = 'pos_error';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

% IMU-only velocity error plots
f = figure('DefaultAxesFontSize', plotFontSize, 'units','normalized','outerposition',[0 0 figSize]);
plot(tVec, errVelIMU, 'm-', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Velocity error (m/s)')
if showTitle, title('IMU-only Velocity error'); end

FigName = 'vel_error_gnss-imu';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;


% Velocity Estimation Error plot
f = figure('DefaultAxesFontSize', plotFontSize, 'units','normalized','outerposition',[0 0 figSize]);
p1 = plot(tVec, errVelEKF, 'b-', 'Linewidth', lineWidth); hold on;
p2 = plot(tVec, errVelSkog, 'r-', 'Linewidth', lineWidth);
p21 = plot(tVec, errVelSkogPresent, 'g-', 'Linewidth', lineWidth);
% yline(mean(errVelEKF), 'k');
p3 = plot(tVec, 3*sigmaVelEKF, 'Color', '#4DBEEE', 'Linewidth', lineWidth);
p4 = plot(tVec, -3*sigmaVelEKF, 'Color', '#4DBEEE', 'Linewidth', lineWidth);
p5 = plot(tVec, 3*sigmaVelSkog, 'Color', '#D95319', 'Linewidth', lineWidth);
p6 = plot(tVec, -3*sigmaVelSkog, 'Color', '#D95319', 'Linewidth', lineWidth);
p7 = plot(tVec, 3*sigmaVelSkogPresent, 'Color', '#77AC30', 'Linewidth', lineWidth);
p8 = plot(tVec, -3*sigmaVelSkogPresent, 'Color', '#77AC30', 'Linewidth', lineWidth);
xlabel('Time (s)'); ylabel('Velocity error (m/s)')
if showTitle, title('Error in velocity estimations'); end
h = [p1 p2 p21 p3 p5, p7];
hleg1 = legend(h,{'EKF', 'AEKF', 'AEKF at present','3\sigma EKF', '3\sigma AEKF', '3\sigma AEKF at present'}, 'Location', 'northeastoutside');


FigName = 'vel_error';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

% Position error CDF
c = ['b', 'r', 'g'];
f = figure('DefaultAxesFontSize', plotFontSize, 'units','normalized','outerposition',[0 0 figSize]);
h(1) = cdfplot(abs(errPosEKF)); hold on;
h(2) = cdfplot(abs(errPosSkog));
h(3) = cdfplot(abs(errPosSkogPresent));
for i = 1:3, set(h(i), 'Linewidth', lineWidth, 'Color', c(i)); end
xlabel('Position error (m)'); ylabel('Cumulative frequency');
legend({'EKF', 'AEKF', 'AEKF at present'}, 'Location', 'northeastoutside');
title('');
if showTitle, title('Position error CDF'); end


FigName = 'pos_cdf';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end

% Velocity error CDF
f = figure('DefaultAxesFontSize', plotFontSize, 'units','normalized','outerposition',[0 0 figSize]);
h(1) = cdfplot(abs(errVelEKF)); hold on;
h(2) = cdfplot(abs(errVelSkog));
h(3) = cdfplot(abs(errVelSkogPresent));
for i = 1:3, set(h(i), 'Linewidth', lineWidth, 'Color', c(i)); end
xlabel('Velocity error (m/s)'); ylabel('Cumulative frequency');
legend({'EKF', 'AEKF', 'AEKF at present'}, 'Location', 'northeastoutside');
title('');
if showTitle, title('Velocity error CDF'); end


FigName = 'vel_cdf';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

% Position error CDF After Convergence
c = ['b', 'r', 'g'];
f = figure('DefaultAxesFontSize', plotFontSize, 'units','normalized','outerposition',[0 0 figSize]);
h(1) = cdfplot(abs(errPosEKF(kConv:end))); hold on;
h(2) = cdfplot(abs(errPosSkog(kConv:end)));
h(3) = cdfplot(abs(errPosSkogPresent(kConv:end)));
% yline(0.95)
for i = 1:3, set(h(i), 'Linewidth', lineWidth, 'Color', c(i)); end
xlabel('Position error (m)'); ylabel('Cumulative frequency');
legend({'EKF', 'AEKF', 'AEKF at present'}, 'Location', 'northeastoutside');
title('');
if showTitle, title(sprintf('Position error CDF after convergence (%d s)', convergenceTime)); end


FigName = 'pos_cdf_conv';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end

% Velocity error CDF After Convergence
f = figure('DefaultAxesFontSize', plotFontSize, 'units','normalized','outerposition',[0 0 figSize]);
h(1) = cdfplot(abs(errVelEKF(kConv:end))); hold on;
h(2) = cdfplot(abs(errVelSkog(kConv:end)));
h(3) = cdfplot(abs(errVelSkogPresent(kConv:end)));
% yline(0.95)
for i = 1:3, set(h(i), 'Linewidth', lineWidth, 'Color', c(i)); end
xlabel('Velocity error (m/s)'); ylabel('Cumulative frequency');
legend({'EKF', 'AEKF', 'AEKF at present'}, 'Location', 'northeastoutside');
title('');
if showTitle, title(sprintf('Velocity error CDF after convergence (%d s)', convergenceTime)); end


FigName = 'vel_cdf_conv';
if savePlots, saveas(f, fullfile(savePath, [FigName, format])); end
clear f;

end


%% Figures not used

% figure;
% plot(tVec(kGNSS), xEKF(COL_EST_ACCBIAS, kGNSS), 'b-'); hold on;
% plot(tVec(kGNSS), xSkog(COL_EST_ACCBIAS, kGNSS), 'r-');
% xlabel('Time (s)'); ylabel('Error Bias (m/s^2)')
% title('Accelerometer bias error-state EKF');
% legend('EKF', 'AEKF', 'Location', 'northeastoutside');