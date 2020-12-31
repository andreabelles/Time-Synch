function [xEst, PEst, measAccCorr] = standardEKF(xOld, POld, pGNSS, measAcc, measAccCorrOld, measGyro, measGyroCorrOld, tIMU, Config)

% EKF:  This function estimates the position, velocity and bias based on state-augmented KF
%           from [Skog, Händel, 2011] Time Synchronization Errors in Loosely CoupledGPS-Aided 
%           Inertial Navigation Systems (see Table 1). 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

if (~isnan(pGNSS)) % If GNSS position is available
    H = [1 0 0 0 0 0;   ...
         0 1 0 0 0 0];
    R = diag([Config.varPosGNSS Config.varPosGNSS]);
    K = (POld*H')/(H*POld*H' + R);
    z = pGNSS - H*xOld;
    xOld = xOld + K*z;
    POld = POld - K*H*POld;
end

% Initialization
% F = [1 tIMU 0; 0 1 tIMU; 0 0 1];
% F = [1 0 tIMU*cos(xOld(4)) 0 0 0;   ...   %%%% TODO: find correct F %%%%
%      0 1 tIMU*sin(xOld(4)) 0 0 0;   ...
%      0 0 1 0 0 0;                   ...
%      0 0 0 1 0 0;                   ...
%      0 0 0 0 1 0;                   ...
%      0 0 0 0 0 1];
 
Q = [0 0 0; 0 tIMU*Config.varAccNoise 0; 0 0 tIMU*Config.varAccBiasNoise];
    
% Sensor error compensation
measAccCorr = measAcc + xOld(3);
% Strapdown equations updated
xEst(2) = xOld(2) + 0.5 * (measAccCorr + measAccCorrOld) * tIMU;
xEst(1) = xOld(1) + 0.5 * (xEst(2) + xOld(2)) * tIMU;
% Sensor error update
xEst(3) = xOld(3);

% Covariance prediction
PEst = F*POld*F' + Q;

end