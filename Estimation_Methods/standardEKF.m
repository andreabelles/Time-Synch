function [rInt, PEst] = standardEKF(rInt, pGNSS, measAcc, tIMU, POld, sigmaAcc, sigmaGNSS, sigmaBiasAcc)
% EKF:  This function estimates the position and velocity using an EKF. 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

if (isnan(pGNSS)) % No GNSS observations
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
    PEst = F*POld*F' + Q;
    
else % GNSS observations
    H = [1 0 0];
    R = [sigmaGNSS^2];
    K = (POld*H')/(H*POld*H' + R);
    z = pGNSS - H*rInt;
    xEst = K*z;
    rInt = rInt + xEst;
    PEst = POld - K*H*POld;
    
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
    PEst = F*PEst*F' + Q;
end

end