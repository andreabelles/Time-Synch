function [xEst, PEst, measAccCorr] = standardEKF(xOld, POld, pGNSS, measAcc, measAccCorrOld, measGyro, measGyroCorrOld, tIMU, Config)

% EKF:  This function estimates the position, velocity and bias based on state-augmented KF
%           from [Skog, Händel, 2011] Time Synchronization Errors in Loosely CoupledGPS-Aided 
%           Inertial Navigation Systems (see Table 1). 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

if (~isnan(pGNSS)) % If GNSS position is available
    H = [1 0 0 0 0 0;   ...
         0 1 0 0 0 0];
    R = diag([Config.varPosGNSS Config.varPosGNSS]);
    K = (POld*H')/(H*POld*H' + R);
    z = pGNSS - H*xOld;
    xOld = xOld + K*z;
    POld = POld - K*H*POld;
end

% Initialization
% F = [1 tIMU 0; 0 1 tIMU; 0 0 1];  %%%% TODO: find correct F %%%%
F       = eye(6);
F(1, 3) = tIMU*cosd(xOld(4));               % d PNorth / d vAT
F(1, 4) = -tIMU*xOld(3)*sind(xOld(4));      % d PNorth / d heading
F(2, 3) = tIMU*sind(xOld(4));               % d PEast / d vAT
F(2, 4) = tIMU*xOld(3)*cosd(xOld(4));       % d PEast / d heading
F(3, 5) = tIMU;                             % d vAT / d biasAcc      %% Check this one: using dev for bias, not d bias
F(4, 6) = tIMU;                             % d heading / d biasGyro %% Check this one: using dev for bias, not d bias
 
Q       = zeros(6);
Q(3, 3) = Config.varAccNoise * tIMU;        % Along-Track Velocity
Q(4, 4) = Config.varGyroNoise * tIMU;       % Heading
Q(5, 5) = Config.varAccBiasNoise * tIMU;    % Accelerometer bias (error)
Q(5, 5) = Config.varGyroBiasNoise * tIMU;   % Gyroscope bias (error)
    
% Sensor error compensation
measAccCorr = measAcc + xOld(5);
measGyroCorr = measGyro + xOld(6);
% Strapdown equations updated
xEst(3) = xOld(3) + 0.5 * (measAccCorr + measAccCorrOld) * tIMU;
xEst(4) = xOld(4) + 0.5 * (measGyroCorr + measGyroCorrOld) * tIMU;
xEst(1) = xOld(1) + 0.5 * (xEst(3) + xOld(3)) * cosd(xOld(4)) * tIMU;
xEst(2) = xOld(2) + 0.5 * (xEst(3) + xOld(3)) * sind(xOld(4)) * tIMU;      % TODO: check this, average heading?
% % Sensor error update
xEst(5) = xOld(5);
xEst(6) = xOld(6);

% Covariance prediction
PEst = F*POld*F' + Q;

end