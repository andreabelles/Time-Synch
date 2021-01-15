function [estIMURaw, pGNSS, xEKF, PEKF, estEKF, xSkog, PSkog, estSkog] = ...
            simulateEstimations(trueTrajectory, tspan, Config)

%% Initializations
pTrue = trueTrajectory(:,1:2);
nPts    = length(tspan); 

% IMU Sensor Measurements
measAcc = zeros(nPts,1);
measAccCorrEKF = zeros(nPts,1);
measAccCorrSkog = zeros(nPts,1);
measGyro = zeros(nPts,1);
measGyroCorrEKF = zeros(nPts,1);
measGyroCorrSkog = zeros(nPts,1);

% Estimated Inertial Navigation Solution (INS) Raw (IMU Only No Corrected)
estIMURaw.headingRate       = zeros(nPts,1);    
estIMURaw.headingRate(1)    = Config.headingRate0;
estIMURaw.heading           = zeros(nPts,1);    
estIMURaw.heading(1)        = Config.heading0;
estIMURaw.acc               = zeros(nPts,1);    
estIMURaw.acc(1)            = Config.a0;
estIMURaw.vel               = zeros(nPts,1);    
estIMURaw.vel(1)            = Config.v0;
estIMURaw.pos               = zeros(nPts,2); 
estIMURaw.pos(1, :)         = [Config.pNorth0 Config.pEast0];

% Estimated GNSS Navigation Solution (GNSS Only)
pGNSS   = nan(nPts,2);

% Standard EKF:
% GNSS/INS Integration Navigation Solution (IMU corrected with GNSS)
estEKF.pos               = zeros(nPts,2);    
estEKF.pos(1, :)         = [Config.xpNorth0 Config.xpEast0];
estEKF.vel               = zeros(nPts,1);    
estEKF.vel(1)            = Config.xv0;
estEKF.acc               = zeros(nPts,1);
estEKF.acc(1)            = Config.xa0;
estEKF.heading           = zeros(nPts,1);    
estEKF.heading(1)        = Config.xheading0;
estEKF.headingRate       = zeros(nPts,1);
estEKF.headingRate(1)    = Config.xheadingRate0;
estEKF.biasAcc           = zeros(nPts,1);    
estEKF.biasAcc(1)        = Config.xba0;
estEKF.biasGyro           = zeros(nPts,1);    
estEKF.biasGyro(1)        = Config.xbg0;

% Error-State Vector
xEKF        = zeros(6,nPts);
% Covariance matrix
PEKF        = zeros(6,6,nPts);
PEKF(:,:,1) = diag([Config.sigmaInitNorthPos^2, ...
                    Config.sigmaInitEastPos^2,  ...
                    Config.sigmaInitVel^2,      ...
                    Config.sigmaInitHeading^2,  ...
                    Config.sigmaInitAccBias^2,  ...
                    Config.sigmaInitGyroBias^2]);

% Skog EKF : 
% GNSS/INS Integration Navigation Solution (IMU corrected with GNSS) at
% present (k)
estSkog.pos               = zeros(nPts,2);    
estSkog.pos(1, :)         = [Config.xpNorth0 Config.xpEast0];
estSkog.vel               = zeros(nPts,1);    
estSkog.vel(1)            = Config.xv0;
estSkog.acc               = zeros(nPts,1);
estSkog.acc(1)            = Config.xa0;
estSkog.heading           = zeros(nPts,1);    
estSkog.heading(1)        = Config.xheading0;
estSkog.headingRate       = zeros(nPts,1);
estSkog.headingRate(1)    = Config.xheadingRate0;
estSkog.biasAcc           = zeros(nPts,1);    
estSkog.biasAcc(1)        = Config.xba0;
estSkog.biasGyro          = zeros(nPts,1);    
estSkog.biasGyro(1)       = Config.xbg0;
estSkog.timeDelay         = zeros(nPts,1);    
estSkog.timeDelay(1)      = Config.xt0;

% Error-State Vector
xSkog        = zeros(7,nPts);
% Covariance matrix
PSkog        = zeros(7,7,nPts);
PSkog(:,:,1) = diag([Config.sigmaInitNorthPos^2,...
                    Config.sigmaInitEastPos^2,  ...
                    Config.sigmaInitVel^2,      ...
                    Config.sigmaInitHeading^2,  ...
                    Config.sigmaInitAccBias^2,  ...
                    Config.sigmaInitGyroBias^2, ...
                    Config.sigmaInitDelay^2]);
                

% GNSS/INS Integration Navigation Solution (IMU corrected with GNSS) at
% time delay (k-Td)
% estSkogAtDelay.pos               = nan(nPts,2);    
% estSkogAtDelay.vel               = nan(nPts,1);    
% estSkogAtDelay.heading           = nan(nPts,1);    
% estSkogAtDelay.biasAcc           = nan(nPts,1);    
% estSkogAtDelay.biasGyro           = nan(nPts,1);    
% estSkogAtDelay.timeDelay           = nan(nPts,1);    
% % Covariance matrix at time delay
% PSkogAtDelay        = nan(7,7,nPts);


for k = 2:1:nPts
%     [measAcc(k), measGyro(k), pGNSS(k, :)] = generateMeasurements(pTrue, Config, k);
    load('measWrongHeading.mat');
    
    % IMU Only: Navigation equations (Using raw measurements)
    
    [estIMURaw] = navigationEquations(measGyro(k), measAcc(k), estIMURaw, k, Config.tIMU);
    
    % Integration GNSS/INS: Standard EKF
    [xEKF(:,k), PEKF, estEKF, measAccCorrEKF, measGyroCorrEKF] =     ...
                                            standardEKF(xEKF(:,k-1), ...
                                                        PEKF, ...
                                                        estEKF,                ...
                                                        pGNSS(k,:),         ...
                                                        k, ...
                                                        measAcc,                 ...
                                                        measGyro,                ...
                                                        measAccCorrEKF, ...
                                                        measGyroCorrEKF,...    
                                                        Config);
    
%     [xSkog(:,k), PSkog, estSkog, measAccCorrSkog, measGyroCorrSkog] =         ...
%                                                 skogEKF(xSkog(:,k-1),  ...
%                                                         PSkog,       ...
%                                                         estSkog,     ...
%                                                         pGNSS(k,:),     ...
%                                                         k, ...
%                                                         measAcc,            ...
%                                                         measAccCorrSkog,    ...
%                                                         measGyro, ...
%                                                         measGyroCorrSkog,   ... 
%                                                         tspan,              ...
%                                                         Config);                                                
%     TODO: Lee and Johnson method
    
     
end


end