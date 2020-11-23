function [xEst, PEst] = standardEKF(vIMU, pIMU, pGNSS, tIMU, xOld, POld, sigmaAcc, sigmaGNSS)
% EKF:  This function estimates the position and velocity using an EKF. 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

% Initialization
% deltaK = ~isnan(pGNSS);
F = [1 tIMU; 0 1];
Q = [sigmaAcc^2 0; 0 sigmaAcc^2];

% State prediction
xEst = F*xOld;
PEst = F*POld*F' + Q;

% State Update
if(~isnan(pGNSS))
    H = [1 0];
    R = [sigmaGNSS^2];
    z = pGNSS - H * xEst;
    K = (POld*H')/(H*POld*H' + R);
    xEst = xEst + K*z;
    PEst = PEst - K*H*PEst;
end

end