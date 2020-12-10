function [rIMU, PEst] = standardEKF(rIMU, PEst, pGNSS, measAcc, tIMU, sigmaAcc, sigmaGNSS)
% EKF:  This function estimates the position and velocity using an EKF. 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

if (~isnan(pGNSS)) % If GNSS position is available
    H = [1 0 0];
    R = [(sigmaGNSS)^2];
    K = (PEst*H')/(H*PEst*H' + R);
    z = pGNSS - H*rIMU;
    xEst = K*z;
    rIMU = rIMU + xEst;
    PEst = PEst - K*H*PEst;
end

% Initialization
F = [1 tIMU 0; 0 1 tIMU; 0 0 1];
Q = [0 0 0; 0 tIMU*sigmaAcc^2 0; 0 0 tIMU*sigmaAcc^2];
    
% Sensor error compensation
measAccCorr = measAcc + rIMU(3);
% Strapdown equations updated
rIMU(2) = rIMU(2) + measAccCorr * tIMU;
rIMU(1) = rIMU(1) + rIMU(2) * tIMU;
% Sensor error update
rIMU(3) = rIMU(3);

% Covariance prediction
PEst = F*PEst*F' + Q;

end