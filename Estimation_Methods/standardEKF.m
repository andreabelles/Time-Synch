function [estInt, P, estIMUCorr, measAccCorr, measGyroCorr] = ...
    standardEKF(estInt, epoch, POld, estIMUCorr, pGNSS, measAcc, measGyro, tIMU, Config)

% EKF:  This function estimates the position, velocity and bias based on state-augmented KF
%           from [Skog, Händel, 2011] Time Synchronization Errors in Loosely CoupledGPS-Aided 
%           Inertial Navigation Systems (see Table 1). 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

% Sensor error compensation: Get IMU measurements estimations from IMU measures
measAccCorr = measAcc - estInt.biasAcc(epoch-1); 
measGyroCorr = measGyro - estInt.biasGyro(epoch-1);

% Navigation equations computation: Update corrected inertial navigation solution
[estIMUCorr] = navigationEquations(measGyroCorr, measAccCorr, estIMUCorr, epoch, tIMU);

% Initialization
x         = zeros(6,1); % Error-state vector: [dpNorth dpEast dV dHeading dBiasAcc dBiasGyro]
% z            = zeros(size(pGNSS)); % Innovation vector (= 0 if no GNSS measurements available)     

% Continuous-time state transition model
F       = zeros(6);
F(1, 3) = cosd(estIMUCorr.heading(epoch));                              % d PNorth / d vAT
F(1, 4) = -estIMUCorr.vel(epoch)*sind(estIMUCorr.heading(epoch));       % d PNorth / d heading
F(2, 3) = sind(estIMUCorr.heading(epoch));                              % d PEast / d vAT
F(2, 4) = estIMUCorr.vel(epoch)*cosd(estIMUCorr.heading(epoch));        % d PEast / d heading
F(3, 5) = 1;                                                            % d vAT / d biasAcc
F(4, 6) = 1;                                                            % d heading / d biasGyro
 
Q       = zeros(6);
Q(3, 3) = Config.varAccNoise;        % Along-Track Velocity
Q(4, 4) = Config.varGyroNoise;       % Heading
Q(5, 5) = Config.varAccBiasNoise;    % Accelerometer bias (error)
Q(6, 6) = Config.varGyroBiasNoise;   % Gyroscope bias (error)

% Discrete-time state transition model
Fk       = eye(6) + tIMU*F; % Taylor expansion 1st order
Qk       = tIMU*Q;

% Time propagation (state prediction) - X_k|k-1 and cov(X_k|k-1)
x = Fk * x; % X_k|k-1 = F_k*X_k-1|k-1
% Covariance prediction
P = Fk * POld * Fk' + Qk;

if (~isnan(pGNSS)) % If GNSS position is available
    
    % Measurement model
    z = pGNSS' - estIMUCorr.pos(epoch,:)'; % Observation vector: GPS - prediction INS 
    
    H = [1 0 0 0 0 0;   ...
         0 1 0 0 0 0];
    R = diag([Config.varPosGNSS Config.varPosGNSS]);
    
    % Kalman filter gain computation
    K = (P*H')/(H*P*H' + R);
    
    % Innovation vector computation 
    dz =  z - H * x;
    
    % Update state prediction (state estimation)
    x = x + K*dz; % X_k|k = X_k|k-1 + Kdz 
    P = P - K*H*P;
end

% CLOSED-LOOP CORRECTION
% GNSS/INS Integration navigation solution
estInt.pos(epoch,:) = estIMUCorr.pos(epoch,:) + x(1:2)';
estInt.vel(epoch) = estIMUCorr.vel(epoch) + x(3);
estInt.heading(epoch) = estIMUCorr.heading(epoch) + x(4);
estInt.biasAcc(epoch) = estInt.biasAcc(epoch-1) + x(5);
estInt.biasGyro(epoch) = estInt.biasGyro(epoch-1) + x(6);

end