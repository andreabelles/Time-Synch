function [estIMU, pGNSS, xEKF, PEKF, xSkog, PSkog, estEKF, estSkog, estSkogPresent, PSkogPresent] = ...
                        simulateEstimations(trueTrajectory, Config)

                    
global COL_TRAJECTORY_TIME
global COL_EST_POS COL_EST_VEL COL_EST_ACCBIAS COL_EST_DELAY COL_IMU_MAX COL_EKF_MAX COL_SKOG_MAX

%% Initializations
nPts    = size(trueTrajectory, 1);

% IMU Sensor Measurements
measAcc         = zeros(nPts,1);
measAccCorrEKF  = zeros(nPts,1);
measAccCorrSkog = zeros(nPts,1);

% IMU-Only Navigation Solution
estIMU                  = nan(nPts, COL_IMU_MAX);
estIMU(1, COL_EST_POS)  = Config.rp0;
estIMU(1, COL_EST_VEL)  = Config.rv0;

% GNSS-Only Navigation Solution
pGNSS   = nan(nPts,1);

% Standard EKF:
% GNSS/INS Integration Navigation Solution
estEKF                      = nan(nPts, COL_EKF_MAX); 
estEKF(1, COL_EST_POS)      = Config.rp0;
estEKF(1, COL_EST_VEL)      = Config.rv0;
estEKF(1, COL_EST_ACCBIAS)  = Config.rba0;
% Error-State Vector
xEKF        = nan(COL_EKF_MAX, nPts);
% Covariance matrix
PEKF        = nan(COL_EKF_MAX, COL_EKF_MAX, nPts);
PEKF(:,:,1) = [Config.sigmaInitPos^2 0 0; ...
               0 Config.sigmaInitVel^2 0; ...
               0 0 Config.sigmaInitAccBias^2];

% Skog EKF: 
% GNSS/INS Integration Navigation Solution
estSkog                     = nan(nPts, COL_SKOG_MAX); 
estSkog(1, COL_EST_POS)     = Config.rp0;
estSkog(1, COL_EST_VEL)     = Config.rv0;
estSkog(1, COL_EST_ACCBIAS) = Config.rba0;
estSkog(1, COL_EST_DELAY)   = Config.rt0;
% Error-State Vector
xSkog        = nan(COL_SKOG_MAX,nPts);
% Covariance matrix
PSkog        = nan(COL_SKOG_MAX,COL_SKOG_MAX,nPts);
PSkog(:,:,1) = [Config.sigmaInitPos^2 0 0 0;    ...
                0 Config.sigmaInitVel^2 0 0;    ...
                0 0 Config.sigmaInitAccBias^2 0;...
                0 0 0 Config.sigmaInitDelay^2];
% Skog estimation moved to the present
estSkogPresent                      = nan(nPts, COL_SKOG_MAX); 
estSkogPresent(1, COL_EST_POS)      = Config.rp0;
estSkogPresent(1, COL_EST_VEL)      = Config.rv0;
estSkogPresent(1, COL_EST_ACCBIAS)  = Config.rba0;
estSkogPresent(1, COL_EST_DELAY)    = Config.rt0;
PSkogPresent     = nan(COL_SKOG_MAX,COL_SKOG_MAX,nPts);
PSkogPresent(:,:,1) = [ Config.sigmaInitPos^2 0 0 0;    ...
                        0 Config.sigmaInitVel^2 0 0;    ...
                        0 0 Config.sigmaInitAccBias^2 0;...
                        0 0 0 Config.sigmaInitDelay^2];
timeVec = trueTrajectory(:, COL_TRAJECTORY_TIME);

for k = 2:1:nPts
    % IMU and GNSS measurements generation
    [measAcc(k), pGNSS(k)] = generateMeasurements(trueTrajectory, k, Config);
    % Strapdown equations
    [estIMU(k, :)] = navigationEquations(estIMU(k-1, :), measAcc(k), measAcc(k-1), Config.tIMU);

    % Standard EKF method
    [xEKF(:,k), PEKF(:,:,k), estEKF(k,:), measAccCorrEKF(k)] =          ...
                                    standardEKF(xEKF(:,k-1),            ...
                                                PEKF(:,:,k-1),          ...
                                                estEKF(k-1, :),         ...
                                                pGNSS(k),               ...
                                                measAcc(k),                ...
                                                measAccCorrEKF(k-1),    ...              
                                                Config);
    % Augmented EKF from Skog and Händel
    [xSkog(:,k), PSkog(:,:,k), estSkog(k,:), measAccCorrSkog(k), estSkogPresent(k,:), PSkogPresent(:,:,k)] = ...
                                        skogEKF(xSkog(:,k-1),           ...
                                                PSkog(:,:,k-1),         ...
                                                estSkog(k-1,:),         ...
                                                pGNSS,                  ...
                                                measAcc,                ...
                                                measAccCorrSkog(k-1),   ...
                                                k,                      ...
                                                timeVec,                ...
                                                Config);
     
end


end