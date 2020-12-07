function [rIMU, PEst, measAccCorr] = skogEKF(rIMU, PEst, pGNSS, measAcc, measAccCorr, k, t, tIMU, sigmaAcc, sigmaGNSS)
% skogEKF:  This function estimates the position, velocity and time delay error using the Skog and Handel
%           method (augmented error EKF). 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

% Sensor error compensation
measAccCorr(k) = measAcc(k) + rIMU(3);

% Interpolation of accelerometer measurement at t-Td
measAccInt = interp1(t, measAccCorr(1:k), t(end) - rIMU(4)); % TODO: check constraint for Td

% Initialization
F = [1 tIMU 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Q = [0 0 0 0; 0 tIMU*sigmaAcc^2 0 0; 0 0 0 0; 0 0 0 0];

if(~isnan(pGNSS)) % If GNSS position is available  
    H = [1 0 0 -rIMU(2)];
    R = [sigmaGNSS^2];
    K = (PEst*H')/(H*PEst*H' + R);
    z = pGNSS - H * rIMU;
    Kp = F*K;
    d = measAccInt * (1/2) * (rIMU(4)^2);
    expX = pinv(eye(size(F))-F) * (-(1/2) * Kp * measAccInt * PEst(4,4));
    piFactor = (1/4)*(measAccInt^2)*(3*(PEst(4,4)^2) - 2*(expX(4)^4));
    gammaFactor = (PEst(4,4)*expX + 2*expX(4)*(PEst(1:4,4) - expX(4)*expX))*(measAccInt/2);
    
    xEst = Kp*(z - d);
    rIMU = rIMU + xEst;
    PEst = PEst + Kp*R*Kp' + Kp*piFactor*Kp' - F*gammaFactor*Kp' - Kp*gammaFactor'*F';
end

% Sensor error compensation
measAccCorr(k) = measAccInt + rIMU(3);
% Strapdown equations updated
rIMU(2) = rIMU(2) + measAccCorr(k) * tIMU;
rIMU(1) = rIMU(1) + rIMU(2) * tIMU;
% Sensor error update
rIMU(3) = rIMU(3);

% Covariance prediction
PEst = F*PEst*F' + Q;
    
%     % Correction from t-Td to t
%     F = [1 rInt(4) 0; 0 1 0; 0 0 1];
%     xEst = F*xEst;
%     PEst = F*PEst*F' + Q;

end