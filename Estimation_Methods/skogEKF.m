function [xPredicted, PPredicted, measAccCorr] = ...
    skogEKF(xHistoric, PHistoric, pGNSS, measAcc, measAccCorr, k, tspan, tIMU, Config)
% skogEKF:  This function estimates the position, velocity, bias and time delay based on state-augmented KF
%           from [Skog, Händel, 2011] Time Synchronization Errors in Loosely CoupledGPS-Aided Inertial Navigation Systems.
%           This algorithm has been modified to estimate a total-state instead of an error-state KF and also to 
%           propagate the estimation from previous delays to the present. 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

% --> TODO: define indexes of state vector as variables

% From all the previous states and covariance matrix, we select the
% previous one
xOld       = xHistoric(:, k-1);
POld       = PHistoric(:, :, k-1); 

% Sensor error compensation
measAccCorr(k) = measAcc(k) + xOld(3);

if(~isnan(pGNSS)) % If GNSS position is available
    %% Interpolate accelerometer measurement from k to k-Td
    timeAtDelay = tspan(k-1) - xOld(4);
    % Extracted from Eq. (24) and reference paper [22]. 
    measAccInt = lagrangeInterp(tspan(1:k), measAccCorr(1:k), timeAtDelay); % TODO: check constraint for Td
    
    %% Find xEst and PEst before k-Td
    % Determine index of IMU sample right before k-Td in time vector tspan
    [~, closestIndex] = min(abs(timeAtDelay - tspan)); % Index of tspan closest to delay
    % Condition to keep always the previous sample to k-Td
    if tspan(closestIndex) > timeAtDelay, prevIndex = closestIndex - 1;
    else, prevIndex = closestIndex; end
    
    % Select the state vector and covariance matrix at instant previous to k-Td
    xPrevDelay = xHistoric(:, prevIndex);
    PPrevDelay = PHistoric(:,:, prevIndex);
    
    %% Predict rIMU from one sample before k-Td of the IMU to k-Td
    timeToDelay = timeAtDelay - tspan(prevIndex); % Time from previous sample of k-Td until k-Td
    
    % Strapdown equations to obtain state vector prediction at time delay (k-Td)
    xPredAtDelay = zeros(4,1);
    xPredAtDelay(2) = xPrevDelay(2) + 0.5 * (measAccInt + measAccCorr(prevIndex)) * timeToDelay;
    xPredAtDelay(1) = xPrevDelay(1) + 0.5 * (xPrevDelay(2) + xPredAtDelay(2)) * timeToDelay;
    % Sensor error update
    xPredAtDelay(3) = xPrevDelay(3);
    % Delay error update
    xPredAtDelay(4) = xPrevDelay(4);
    
    % Covariance matrix prediction at time delay (k-Td)
    FtimeToDelay = [1 timeToDelay 0 0; 0 1 timeToDelay 0; 0 0 1 0; 0 0 0 1];
    QtimeToDelay = [0 0 0 0; 0 timeToDelay*Config.varAccNoise 0 0; 0 0 timeToDelay*Config.varAccBiasNoise 0; 0 0 0 0];

    PPredAtDelay = FtimeToDelay*PPrevDelay*FtimeToDelay' + QtimeToDelay; % P at k-Td
 
%     %% Interpolate rIMU from k-1 to k-Td
%     rIMUtd = zeros(4, 1);
%     for i=1:4
%         rIMUtd(i) = lagrangeInterp(tspan(1:k-1), rIMU(i, 1:k-1), tspan(k-1) - rIMUPrev(4), 30);
%     end
    
    %% Update at k-Td using GNSS measurements
    H = [1 0 0 -xPredAtDelay(2)]; % Eq. (21)
    R = [Config.varPosGNSS]; 
    K = (PPredAtDelay*H')/(H*PPredAtDelay*H' + R); % From Table 1
    z = pGNSS - H * xPredAtDelay(:); % Eq. (20)
    Kp = FtimeToDelay*K; % From eq. (5) and (6)
    d = measAccInt * (1/2) * (xPredAtDelay(4)^2); % Eq. (23)
    expX = pinv(eye(size(FtimeToDelay))-FtimeToDelay) * (-(1/2) * Kp * measAccInt * PPredAtDelay(4,4)); % Eq. (29)
    piFactor = (1/4)*(measAccInt^2)*(3*(PPredAtDelay(4,4)^2) - 2*(expX(4)^4)); % Eq. (27)
    gammaFactor = (PPredAtDelay(4,4)*expX + 2*expX(4)*(PPredAtDelay(1:4,4) - expX(4)*expX))*(measAccInt/2); % Eq. (28)
    
    % Update state vector and covariance matrix at time delay (k-Td)
    xUpdatedAtDelay = xPredAtDelay + Kp*(z - d); % Eq. (25)
    PUpdatedAtDelay = PPredAtDelay + Kp*R*Kp' + Kp*piFactor*Kp' - FtimeToDelay*gammaFactor*Kp' - Kp*gammaFactor'*FtimeToDelay'; % Eq. (26)
    
    %% Move updated state vector and covariance matrix from k-Td to k-1
    FDelaytoOld     = [ 1 xOld(4) 0 0;              ...
                    0 1 xOld(4) 0;              ...
                    0 0 1 0;                        ...
                    0 0 0 1];
                
    QDelaytoOld     = [ 0 0 0 0;                        ...
                    0 xOld(4)*Config.varAccNoise 0 0;   ...
                    0 0 tIMU*Config.varAccBiasNoise 0;                        ...
                    0 0 0 tIMU*Config.varDelayProcessNoise];
                
    xOld        = FDelaytoOld * xUpdatedAtDelay;
    POld        = FDelaytoOld*PUpdatedAtDelay*FDelaytoOld' + QDelaytoOld;
end

%% Predict state vector and covariance matrix from k-1 to k
% Strapdown equations for new prediction from Table 1. 
xPredicted(2) = xOld(2) + 0.5 * (measAccCorr(k) + measAccCorr(k-1)) * tIMU;
xPredicted(1) = xOld(1) + 0.5 * (xOld(2) + xPredicted(2)) * tIMU;
% Sensor error update
xPredicted(3) = xOld(3);
% Delay error update
xPredicted(4) = xOld(4);

% Initialization matrices
F = [1 tIMU 0 0; 0 1 tIMU 0; 0 0 1 0; 0 0 0 1];
Q = [0 0 0 0; 0 tIMU*Config.varAccNoise 0 0; 0 0 tIMU*Config.varAccBiasNoise 0; 0 0 0 tIMU*Config.varDelayProcessNoise]; 
PPredicted = F*POld*F' + Q; % Covariance prediction from Table 1

end