function [xNew, PNew] = skogEKF(pIMU, pGNSS, tIMU, rOld, POld, sigmaAcc, sigmaGNSS, measAcc)
% skogEKF:  This function estimates the position, velocity and time delay error using the Skog and Handel
%           method (augmented error EKF). 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

% Initialization
deltaK = ~isnan(pGNSS);

F = [1 tIMU 0; 0 1 0; 0 0 1];
Q = [sigmaAcc 0 0; 0 sigmaAcc 0; 0 0 0];

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


z = deltaK*(pGNSS - pIMU);
d = measAcc*(1/2)*(xOld(3)^2);
H = deltaK*[1 0 -xOld(2)];
R = [sigmaGNSS];
K = (H*POld*H' + R)\(POld*H'); 
Kp = F*K;
expX = pinv(eye(size(F))-F)*(-(1/2)*Kp*measAcc*POld(3,3));
piFactor = (1/4)*(measAcc^2)*(3*(POld(3,3)^2) - 2*(expX(3)^4));
gammaFactor = (POld(3,3)*expX + 2*expX(3)*(POld(1:3,3) - expX(3)*expX))*(measAcc/2);
%xEst = F*xOld;
%PEst = F*POLd*F' + Q;
%K = (H*PEst*H' + R)\(PEst*H');
%xPred = xOld + K*z;
%PPred = PEst - K*H*PEst;

xNew = (F - Kp*H)*xOld - Kp*d;
PNew = F*POld*F' + Kp*R*Kp' + Q + Kp*piFactor*Kp' - F*gammaFactor*Kp' - Kp*gammaFactor'*F';

end