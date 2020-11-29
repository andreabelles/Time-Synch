function [rInt, xEst, PEst] = standardEKFbis(rInt, pGNSS, measAcc, tIMU, xOld, POld, sigmaAcc, sigmaGNSS)
% EKF:  This function estimates the position and velocity using an EKF. 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation


% Strapdown equations
rInt(2) = rInt(2) + measAcc * tIMU;
rInt(1) = rInt(1) + rInt(2) * tIMU;

% Initialization
F = [1 tIMU; 0 1];
Q = [sigmaAcc^2 0; 0 sigmaAcc^2];

% State prediction
xEst = F*xOld;
PEst = F*POld*F' + Q;

% State Update
if(~isnan(pGNSS))
    H = [1 0];
    R = [sigmaGNSS^2];
    z = pGNSS - H*rInt;
    K = (POld*H')/(H*POld*H' + R);
    xEst = K*z;
    PEst = PEst - K*H*PEst;
end

rInt = rInt + xEst;

end