function [rEst, PEst] = standardEKF(pGNSS, tIMU, rOld, POld, sigmaAcc, sigmaGNSS, measAcc)
% EKF:  This function estimates the position and velocity using an EKF. 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

% Initialization
% deltaK = ~isnan(pGNSS);

F = [1 tIMU 0; 0 1 tIMU; 0 0 measAcc];
Q = [sigmaAcc 0; 0 sigmaAcc];

% pIMU(k) = pIMU(k-1) + v(k)*tIMU + (measAcc(k)*tIMU^2);
% State prediction
rEst = F*rOld;
PEst = F*POld*F' + Q;

% State Update
if(~isnan(pGNSS))
    H = [1 0];
    R = [sigmaGNSS];
    z = pGNSS - H * rEst;
    K = (POld*H')/(H*POld*H' + R);
    xEst = K*z;
    rEst = rEst + xEst;
    PEst = PEst - K*H*PEst;
end

end