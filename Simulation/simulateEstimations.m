function [pIMU, vIMU, pGNSS, xEKF, PEKF, xSkog, PSkog] = simulateEstimations(trueTrajectory, tspan, Config)

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
vIMU    = zeros(nPts,1);    
vIMU(1) = Config.v0;
pIMU    = zeros(nPts,2);    
pIMU(1,:) = [Config.pNorth0 Config.pEast0];
headingIMU    = zeros(nPts,1);    
headingIMU(1) = Config.heading0;
pGNSS   = nan(nPts,2);

% Standard EKF parameters and initializations
xEKF         = zeros(6,nPts);
xEKF(:,1)    = [Config.xpNorth0,    ...
                Config.xpEast0,     ...
                Config.xv0,         ...
                Config.xheading0,   ...
                Config.xba0,        ...
                Config.xbg0];
            
PEKF        = zeros(6,6,nPts);
PEKF(:,:,1) = diag([Config.sigmaInitNorthPos^2, ...
                    Config.sigmaInitEastPos^2,  ...
                    Config.sigmaInitVel^2,      ...
                    Config.sigmaInitHeading^2,  ...
                    Config.sigmaInitAccBias^2,  ...
                    Config.sigmaInitGyroBias^2]);

% Skog EKF parameters and initializations
xSkog         = zeros(7,nPts);
xSkog(:,1)    = [Config.xpNorth0,   ...
                Config.xpEast0,     ...
                Config.xv0,         ...
                Config.xheading0,   ...
                Config.xba0,        ...
                Config.xbg0,        ...
                Config.xt0];
            
PSkog        = zeros(7,7,nPts);
PSkog(:,:,1) = diag([Config.sigmaInitNorthPos^2,...
                    Config.sigmaInitEastPos^2,  ...
                    Config.sigmaInitVel^2,      ...
                    Config.sigmaInitHeading^2,  ...
                    Config.sigmaInitAccBias^2,  ...
                    Config.sigmaInitGyroBias^2, ...
                    Config.sigmaInitDelay^2]);

for k = 2:1:nPts
    % IMU measurements generation
    measAcc(k) = Config.a0 + normrnd(0,sqrt(Config.varAccNoise));
    measGyro(k) = Config.heading0 + normrnd(0,sqrt(Config.varGyroNoise));
    
    % GNSS measurements generation
    if mod(k,Config.M) == 0 && (k - Config.tDelay/tIMU)>0
        pGNSS(k) = pTrue(k - Config.tDelay/tIMU,:) + ...        % true position at t-delay
                    normrnd(0,sqrt(Config.varPosGNSS),[1 2]);   % Gaussian noise for 2D position
    end
    
    % Strapdown equations
    vIMU(k) = vIMU(k-1) + 0.5 * (measAcc(k) + measAcc(k-1)) * tIMU;
    headingIMU(k) = headingIMU(k-1) + 0.5 * (measGyro(k) + measGyro(k-1)) * tIMU;
    pIMU(k, 1) = pIMU(k-1, 1) + 0.5 * (vIMU(k) + vIMU(k-1)) * cosd(headingIMU(k)) * tIMU;
    pIMU(k, 2) = pIMU(k-1, 2) + 0.5 * (vIMU(k) + vIMU(k-1)) * sind(headingIMU(k)) * tIMU;
    
    %EKF
    [xEKF(:,k), PEKF(:,:,k), measAccCorrStandard(k), measGyroCorrStandard(k)] =     ...
                                            standardEKF(xEKF(:,k-1),                ...
                                                        PEKF(:,:,k-1),              ...
                                                        pGNSS(k),                   ...
                                                        measAcc(k),                 ...
                                                        measAccCorrStandard(k-1),   ...
                                                        measGyro,                   ...
                                                        measGyroCorrStandard(k-1),  ...
                                                        tIMU,                       ...
                                                        Config);
    
    [xSkog(:,k), PSkog(:,:,k), measAccCorrSkog, measGyroCorrSkog] =         ...
                                                skogEKF(xSkog,              ...
                                                        PSkog,              ...
                                                        pGNSS(k),           ...
                                                        measAcc,            ...
                                                        measAccCorrSkog,    ...
                                                        measGyro,           ...
                                                        measGyroCorrSkog,   ...
                                                        k,                  ...
                                                        tspan,              ...
                                                        tIMU,               ...                                                
                                                        Config);                                                
    % TODO: Lee and Johnson method
    
     
end


end