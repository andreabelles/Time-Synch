function [xEst, PEst, vEKF, pEKF, biasAccEKF, vIMUPred, pIMUPred, measAccCorr] = standardEKF(xOld, ...
                                                                                                POld, ...
                                                                                                pGNSS, ...
                                                                                                k, ...
                                                                                                measAcc, ...
                                                                                                measAccCorr, ...
                                                                                                vEKF, ...
                                                                                                pEKF, ...
                                                                                                biasAccEKF, ...
                                                                                                vIMUPred, ...
                                                                                                pIMUPred, ...
                                                                                                tIMU, ...
                                                                                                sigmaAcc, ...
                                                                                                sigmaGNSS, ...
                                                                                                sigmaAccBias)

% EKF:  This function estimates the position, velocity and bias based on state-augmented KF
%           from [Skog, Händel, 2011] Time Synchronization Errors in Loosely CoupledGPS-Aided 
%           Inertial Navigation Systems (see Table 1). 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation
  
% Sensor error compensation
measAccCorr(k) = measAcc(k) - biasAccEKF(k-1);
% Strapdown equations updated
vIMUPred(k) = vEKF(k-1) + 0.5 +(measAccCorr(k) + measAccCorr(k-1)) * tIMU;
pIMUPred(k) = pEKF(k-1) + 0.5 +(vIMUPred(k) + vEKF(k-1)) * tIMU;

% vEKF(k) = vIMUPred(k);
% pEKF(k) = pIMUPred(k);
% Initialization
F = [0 1 0; 0 0 1; 0 0 0];
Q = [0 0 0; 0 sigmaAcc^2 0; 0 0 sigmaAccBias^2];  

% Discrete transition model
Fk = eye(size(F)) + tIMU*F;
Qk = tIMU*Q;
xOld  = zeros(3,1);

% System model transition
xEst = Fk * xOld;
% Covariance prediction
PEst = Fk*POld*Fk' + Qk;

if (~isnan(pGNSS)) % If GNSS position is available
    H = [1 0 0];
    R = sigmaGNSS^2;
    K = (PEst*H')/(H*PEst*H' + R);
    z = pGNSS - pIMUPred(k);
    dz = z - H*xEst;
    xEst = xEst + K*dz;
    PEst = PEst - K*H*PEst;
end

pEKF(k) = pIMUPred(k) + xEst(1);
vEKF(k) = vIMUPred(k) + xEst(2);
biasAccEKF(k) = xEst(2);
end