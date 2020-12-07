function [rInt, xEst, PEst] = standardEKFtest(rInt, pGNSS, measAcc, tIMU, xOld, POld, sigmaAcc, sigmaGNSS)
% EKF:  This function estimates the position and velocity using an EKF. 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

% Sensor error compensation
measAccCorr = measAcc + rInt(3);

% Strapdown equations updated
rInt(2) = rInt(2) + measAccCorr * tIMU;
rInt(1) = rInt(1) + rInt(2) * tIMU;

% Sensor error update
rInt(3) = rInt(3);

% Initialization
F = [1 tIMU 0; 0 1 tIMU; 0 0 1];
Q = [0 0 0; 0 tIMU*sigmaAcc^2 0; 0 0 0];

% State prediction
%xEst = F*xOld;
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