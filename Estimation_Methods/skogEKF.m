function [rIMUEst, PEst, measAccCorr] = ...
    skogEKF(rIMU, PEst, pGNSS, measAcc, measAccCorr, k, tspan, tIMU, sigmaAcc, sigmaGNSS)
% skogEKF:  This function estimates the position, velocity and time delay error using the Skog and Handel
%           method (augmented error EKF). 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

% --> TODO: define indexes of state vector as variables

rIMUPrev = rIMU(:, k-1);

% Sensor error compensation
measAccCorr(k) = measAcc(k) + rIMUPrev(3);

% Interpolation of accelerometer measurement at t-Td
measAccInt = lagrangeInterp(tspan(1:k), measAccCorr(1:k), tspan(k) - rIMUPrev(4), 30); % TODO: check constraint for Td

% Initialization
F = [1 tIMU 0 0; 0 1 tIMU 0; 0 0 1 0; 0 0 0 1];
Q = [0 0 0 0; 0 (tIMU^2)*sigmaAcc^2 0 0; 0 0 sigmaAcc^2 0; 0 0 0 0];

if(~isnan(pGNSS)) % If GNSS position is available
    %% Estimate GNSS update at k-Td
    H = [1 0 0 -rIMUPrev(2)];
    R = [sigmaGNSS^2];
    K = (PEst*H')/(H*PEst*H' + R);
    z = pGNSS - H * rIMUPrev(:);
    Kp = F*K;
    d = measAccInt * (1/2) * (rIMUPrev(4)^2);
    expX = pinv(eye(size(F))-F) * (-(1/2) * Kp * measAccInt * PEst(4,4));
    piFactor = (1/4)*(measAccInt^2)*(3*(PEst(4,4)^2) - 2*(expX(4)^4));
    gammaFactor = (PEst(4,4)*expX + 2*expX(4)*(PEst(1:4,4) - expX(4)*expX))*(measAccInt/2);
    
    %% Move rIMU from k-1 to k-Td
    rIMUtd = zeros(4, 1);
    for i=1:4
        rIMUtd(i) = lagrangeInterp(tspan(1:k-1), rIMU(i, 1:k-1), tspan(k-1) - rIMUPrev(4), 30);
    end
    
    %% Update rIMU with xEst at k-Td
    xEst = Kp*(z - d);
    rIMUtd = rIMUtd + xEst;
    PEst = PEst + Kp*R*Kp' + Kp*piFactor*Kp' - F*gammaFactor*Kp' - Kp*gammaFactor'*F';
    
    %% Move updated rIMU from k-Td to k-1
    Ftd         = [ 1 rIMUPrev(4) 0 0;              ...
                    0 1 rIMUPrev(4) 0;              ...
                    0 0 1 0;                        ...
                    0 0 0 1];
                
    Qtd         = [ 0 0 0 0;                        ...
                    0 rIMUPrev(4)*sigmaAcc^2 0 0;   ...
                    0 0 0 0;                        ...
                    0 0 0 0];
                
    rIMUPrev    = Ftd * rIMUtd;
    PEst        = Ftd*PEst*Ftd' + Qtd;
end

%% Predict rIMU from k-1 to k
% Sensor error compensation
measAccCorr(k) = measAccInt + rIMUPrev(3);
% Strapdown equations updated
rIMUEst(2) = rIMUPrev(2) + measAccCorr(k) * tIMU;
rIMUEst(1) = rIMUPrev(1) + rIMUPrev(2) * tIMU;
% Sensor error update
rIMUEst(3) = rIMUPrev(3);
% Delay error update
rIMUEst(4) = rIMUPrev(4);

% Covariance prediction
PEst = F*PEst*F' + Q;

end