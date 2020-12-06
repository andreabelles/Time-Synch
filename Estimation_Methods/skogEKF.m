function [rInt, PEst, measAccCorr] = skogEKF(rInt, pGNSS, measAcc, measAccCorr, k, t, tIMU, POld, sigmaAcc, sigmaGNSS)
% skogEKF:  This function estimates the position, velocity and time delay error using the Skog and Handel
%           method (augmented error EKF). 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

% Initialization
F = [1 tIMU 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Q = [0 0 0 0; 0 tIMU*sigmaAcc^2 0 0; 0 0 0 0; 0 0 0 0];

% Sensor error compensation
measAccCorr(k) = measAcc(k) + rInt(3);

% Interpolation of accelerometer measurement at t-Td
measAccInt = interp1(t, measAccCorr(1:k), t(end) - rInt(4)); % TODO: check constraint for Td

if(isnan(pGNSS)) % No GNSS observations
    
    % Strapdown equations
    rInt(2) = rInt(2) + measAccCorr(k) * tIMU;
    rInt(1) = rInt(1) + rInt(2) * tIMU;

    % Sensor error update
    rInt(3) = rInt(3);

    % State prediction
    PEst = F*POld*F' + Q;

else % GNSS observations
    
    H = [1 0 0 -rInt(2)];
    R = [sigmaGNSS^2];
    K = (POld*H')/(H*POld*H' + R);
    z = pGNSS - H * rInt;
    Kp = F*K;
    d = measAccInt * (1/2) * (rInt(4)^2);
    expX = pinv(eye(size(F))-F) * (-(1/2) * Kp * measAccInt * POld(4,4));
    piFactor = (1/4)*(measAccInt^2)*(3*(POld(4,4)^2) - 2*(expX(4)^4));
    gammaFactor = (POld(4,4)*expX + 2*expX(4)*(POld(1:4,4) - expX(4)*expX))*(measAccInt/2);
    
    xEst = Kp*(z - d);
    rInt = rInt + xEst;
    PEst = POld + Kp*R*Kp' + Kp*piFactor*Kp' - F*gammaFactor*Kp' - Kp*gammaFactor'*F';
    
    % Sensor error compensation
    measAccCorr(k) = measAccInt + rInt(4);
    % Strapdown equations updated
    rInt(2) = rInt(2) + measAccCorr(k) * tIMU;
    rInt(1) = rInt(1) + rInt(2) * tIMU;
    % Sensor error update
    rInt(3) = rInt(3);
    
%     % Correction from t-Td to t
%     F = [1 rInt(3) 0; 0 1 0; 0 0 1];
%     xEst = F*xEst;
%     PEst = F*PEst*F' + Q;
end

end