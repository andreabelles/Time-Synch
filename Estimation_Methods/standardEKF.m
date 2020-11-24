function [pInt, vInt, PEst] = standardEKF(pInt, vInt, pGNSS, measAcc, tIMU, xOld, POld, sigmaAcc, sigmaGNSS)
% EKF:  This function estimates the position and velocity using an EKF. 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation


% Strapdown equations
vInt = vInt + measAcc * tIMU;
pInt = pInt + vInt * tIMU;
rInt = [pInt; vInt];

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
    z = pGNSS - H*rInt;
    K = (POld*H')/(H*POld*H' + R);
    xEst = xEst + K*z;
    PEst = PEst - K*H*PEst;
end

pInt = pInt + xEst(1);
vInt = vInt + xEst(2);

end