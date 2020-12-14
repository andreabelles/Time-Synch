function [xEst, PEst] = standardEKF(xOld, POld, pGNSS, measAcc, tIMU, sigmaAcc, sigmaGNSS)
% EKF:  This function estimates the position, velocity and bias based on state-augmented KF
%           from [Skog, Händel, 2011] Time Synchronization Errors in Loosely CoupledGPS-Aided 
%           Inertial Navigation Systems (see Table 1). 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

if (~isnan(pGNSS)) % If GNSS position is available
    H = [1 0 0];
    R = [(sigmaGNSS)^2];
    K = (POld*H')/(H*POld*H' + R);
    z = pGNSS - H*xOld;
    xOld = xOld + K*z;
    POld = POld - K*H*POld;
end

% Initialization
F = [1 tIMU 0; 0 1 tIMU; 0 0 1];
Q = [0 0 0; 0 (tIMU)*sigmaAcc^2 0; 0 0 (tIMU)*sigmaAcc^2];
    
% Sensor error compensation
measAccCorr = measAcc + xOld(3);
% Strapdown equations updated
xEst(2) = xOld(2) + measAccCorr * tIMU;
xEst(1) = xOld(1) + xOld(2) * tIMU;
% Sensor error update
xEst(3) = xOld(3);

% Covariance prediction
PEst = F*POld*F' + Q;

end