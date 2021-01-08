function [xEst, PEst, vEKF, pEKF, biasAccEKF, measAccCorr] = standardEKF(xOld, ...
                                                                        POld, ...
                                                                        pGNSS, ...
                                                                        k, ...
                                                                        measAcc, ...
                                                                        measAccCorr, ...
                                                                        pEKF, ...
                                                                        vEKF, ...
                                                                        biasAccEKF, ...
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
vEKF(k) = vEKF(k-1) + 0.5 * (measAccCorr(k) + measAccCorr(k-1)) * tIMU;
pEKF(k) = pEKF(k-1) + 0.5 * (vEKF(k) + vEKF(k-1)) * tIMU;

% Initialization
F = [0 1 0; 0 0 1; 0 0 0];
sigmaAccBiasMeas = 0.0001;       % TO BE SET IN CONFIG FILE
Q = [0 0 0; 0 sigmaAcc^2 0; 0 0 sigmaAccBiasMeas^2];  

% Discrete transition model
Fk = eye(size(F)) + tIMU*F;
Qk = tIMU*Q;

% Initialize state for close loop
xOld(1:end)  = 0; % xOld(1:2)  = 0;

% System model transition
xEst = Fk * xOld;
% Covariance prediction
PEst = Fk*POld*Fk' + Qk;

if (~isnan(pGNSS)) % If GNSS position is available
    H = [1 0 0];
    R = sigmaGNSS^2;
    K = (PEst*H')/(H*PEst*H' + R);
    z = pGNSS - pEKF(k);
    dz = z - H*xEst;
    xEst = xEst + K*dz;
    PEst = PEst - K*H*PEst;
end

pEKF(k) = pEKF(k) + xEst(1);
vEKF(k) = vEKF(k) + xEst(2);
biasAccEKF(k) = xEst(3);
end