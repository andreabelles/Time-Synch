function [estIMURaw, estIntEKF, pGNSS, PEKF, estIntSkogPresent, PSkog] = simulateEstimations(trueTrajectory, tspan, Config)

%% Initializations
pTrue = trueTrajectory(:,1:2);
tIMU = Config.tIMU;
nPts    = length(tspan);        
measAcc = zeros(nPts,1);
measAccCorrStandard = zeros(nPts,1);
measAccCorrSkog = zeros(nPts,1);
measGyro = zeros(nPts,1);
measGyroCorrStandard = zeros(nPts,1);
measGyroCorrSkog = zeros(nPts,1);

% Estimated Inertial Navigation Solution Raw (IMU Only No Corrected): Output variable
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

% % Estimated Inertial Navigation Solution Corrected (IMU Only Corrected): Output variable
% estIMUCorrEKF.headingRate       = zeros(nPts,1);    
% estIMUCorrEKF.headingRate(1)    = Config.headingRate0;
% estIMUCorrEKF.heading           = zeros(nPts,1);    
% estIMUCorrEKF.heading(1)        = Config.heading0;
% estIMUCorrEKF.acc               = zeros(nPts,1);    
% estIMUCorrEKF.acc(1)            = Config.a0;
% estIMUCorrEKF.vel               = zeros(nPts,1);    
% estIMUCorrEKF.vel(1)            = Config.v0;
% estIMUCorrEKF.pos               = zeros(nPts,2);    
% estIMUCorrEKF.pos(1, :)         = [Config.pNorth0 Config.pEast0];

% Estimated GNSS Navigation Solution (GPS Only): Output variable
pGNSS   = nan(nPts,2);

% Standard EKF parameters and initializations
PEKF        = zeros(6,6,nPts); % Covariance matrix
PEKF(:,:,1) = diag([Config.sigmaInitNorthPos^2, ...
                    Config.sigmaInitEastPos^2,  ...
                    Config.sigmaInitVel^2,      ...
                    Config.sigmaInitHeading^2,  ...
                    Config.sigmaInitAccBias^2,  ...
                    Config.sigmaInitGyroBias^2]);

% Standard EKF method:                 
% Estimated GNSS/INS Integrated Navigation Solution (IMU and GPS): Output variable                
estIntEKF.pos               = zeros(nPts,2);    
estIntEKF.pos(1, :)         = [Config.xpNorth0 Config.xpEast0];
estIntEKF.vel               = zeros(nPts,1);    
estIntEKF.vel(1)            = Config.xv0;
estIntEKF.heading           = zeros(nPts,1);    
estIntEKF.heading(1)        = Config.xheading0;
estIntEKF.biasAcc           = zeros(nPts,1);    
estIntEKF.biasAcc(1)        = Config.xba0;
estIntEKF.biasGyro           = zeros(nPts,1);    
estIntEKF.biasGyro(1)        = Config.xbg0;

% Skog EKF method: 
% Parameters and initializations
PSkogPresent        = zeros(7,7,nPts);
PSkogPresent(:,:,1) = diag([Config.sigmaInitNorthPos^2,...
                    Config.sigmaInitEastPos^2,  ...
                    Config.sigmaInitVel^2,      ...
                    Config.sigmaInitHeading^2,  ...
                    Config.sigmaInitAccBias^2,  ...
                    Config.sigmaInitGyroBias^2, ...
                    Config.sigmaInitDelay^2]);
                
% Estimated GNSS/INS Integrated Navigation Solution (IMU and GPS): Output variable                
estIntSkogPresent.pos               = zeros(nPts,2);    
estIntSkogPresent.pos(1, :)         = [Config.xpNorth0 Config.xpEast0];
estIntSkogPresent.vel               = zeros(nPts,1);    
estIntSkogPresent.vel(1)            = Config.xv0;
estIntSkogPresent.heading           = zeros(nPts,1);    
estIntSkogPresent.heading(1)        = Config.xheading0;
estIntSkogPresent.biasAcc           = zeros(nPts,1);    
estIntSkogPresent.biasAcc(1)        = Config.xba0;
estIntSkogPresent.biasGyro           = zeros(nPts,1);    
estIntSkogPresent.biasGyro(1)        = Config.xbg0;
estIntSkogPresent.timeDelay           = zeros(nPts,1);    
estIntSkogPresent.timeDelay(1)        = Config.xt0;

estIntSkogPast.pos               = nan(nPts,2);    
estIntSkogPast.vel               = nan(nPts,1);    
estIntSkogPast.heading           = nan(nPts,1);    
estIntSkogPast.biasAcc           = nan(nPts,1);    
estIntSkogPast.biasGyro           = nan(nPts,1);    
estIntSkogPast.timeDelay           = nan(nPts,1);    

PSkogPast        = nan(7,7,nPts);


for epoch = 2:1:nPts
    [measAcc(epoch), measGyro(epoch), pGNSS(epoch, :)] = generateMeasurements(pTrue, Config, epoch);
    
    % IMU Only: Navigation equations (Using raw measurements)
    
    [estIMURaw] = navigationEquations(measGyro(epoch), measAcc(epoch), estIMURaw, epoch, tIMU);
    
    % Integration GNSS/INS: Standard EKF
    [estIntEKF, PEKF(:,:,epoch), measAccCorrStandard(epoch), measGyroCorrStandard(epoch)] =     ...
                                            standardEKF(estIntEKF,                ...
                                                        epoch, ...
                                                        PEKF(:,:,epoch-1),              ...
                                                        pGNSS(epoch,:),                   ...
                                                        measAcc(epoch),                 ...
                                                        measGyro(epoch),                ...
                                                        tIMU,                       ...
                                                        Config);
    
    [estIntSkogPresent, estIntSkogPast, PSkogPresent, PSkogPast, measAccCorrSkog, measGyroCorrSkog] =         ...
                                                skogEKF(estIntSkogPresent,              ...
                                                        estIntSkogPast, ...
                                                        PSkogPresent,        ...
                                                        PSkogPast, ...
                                                        pGNSS(epoch,:),           ...
                                                        measAcc,            ...
                                                        measAccCorrSkog,    ...
                                                        measGyro,           ...
                                                        measGyroCorrSkog,   ...
                                                        epoch,              ...
                                                        tspan,              ...
                                                        tIMU,               ...                      
                                                        Config);                                                
    % TODO: Lee and Johnson method
    
     
end


end