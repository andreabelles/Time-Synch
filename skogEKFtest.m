function [x, PHistoric, vSkog, pSkog, biasAccSkog, timeDelaySkog, measAccCorr] = ...
               skogEKFtest(xOld, PHistoric, pGNSS, k, measAcc, measAccCorr, pSkog, vSkog, biasAccSkog, timeDelaySkog, tspan, Config)
% EKF:  This function estimates the position, velocity and bias based on state-augmented KF
%           from [Skog, Händel, 2011] Time Synchronization Errors in Loosely CoupledGPS-Aided 
%           Inertial Navigation Systems (see Table 1). 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

% From all the previous covariance matrix, we select the previous one
POld           = PHistoric(:, :, k-1); 

% Sensor error compensation: Get IMU measurements estimations from IMU
% measurements
% f_tilda = f_true + bias_f_true
% delta_b_f = b_f_hat - b_f_true -> b_f_hat = b_f_true + delta_b_f
% f_hat = f_tilda - bias_f_hat = (f_true + bias_f_true) - (b_f_true +
% delta_b_f); if delta_b_f tends to 0, then: f_hat = f_true + bias_f_true -
% b_f_true -> f_hat = f_true so the estimation tends to the true value

if all(isnan(pGNSS)) % we need to wait tGNSS to find the first GNSS measurement available, otherwise we only have IMU
  measAccCorr(k) = measAcc(k) - biasAccSkog(k-1);
else
  timeAtDelay = tspan(k) - timeDelaySkog(k-1);
  measAccInt = interp1(tspan(1:k), measAcc(1:k), timeAtDelay); % TODO: check constraint for Td
  measAccCorr(k) = measAccInt - biasAccSkog(k-1);
end
% Navigation equations computation: Update corrected inertial navigation solution
vSkog(k) = vSkog(k-1) + measAccCorr(k) * Config.tIMU;
pSkog(k) = pSkog(k-1) + vSkog(k) * Config.tIMU;
biasAccSkog(k) = biasAccSkog(k-1);
timeDelaySkog(k) = timeDelaySkog(k-1);

% Initialization
F = [0 1 0 0; ...
     0 0 1 0; ...
     0 0 0 0; ...
     0 0 0 0];
Q = [0 0 0 0; ... 
     0 Config.varAccNoise 0 0; ...
     0 0 Config.varAccBiasNoise 0; ...
     0 0 0 Config.varDelayNoise]; 
 
% Discrete transition model
Fk = eye(size(F)) + Config.tIMU*F;
Qk = Config.tIMU*Q;

% Initialize state to 0 for close loop
xOld(1:end)  = 0; % xOld(1:2)  = 0;

% Time propagation (state prediction) - X_k|k-1 and cov(X_k|k-1)
x = Fk * xOld;
% Covariance prediction
PPred = Fk*POld*Fk' + Qk;
PHistoric(:,:,k) = PPred; % Save in case no GNSS measurements

if (~isnan(pGNSS(k))) % If GNSS position is available
    
    % Measurement model
    z = pGNSS(k) - pSkog(k);  % Observation vector: GPS - prediction INS  
    
    H = [1 0 0 -vSkog(k)]; % Eq. (21)
    R = Config.varPosGNSS;
    
    % Kalman filter gain computation
    K = (PPred*H')/(H*PPred*H' + R); % From Table 1
    
    % Innovation vector computation 
    dz = z - H * x; % Eq. (20)
    
    % Update state vector and covariance matrix
    x = x + K*dz;
    PUpdated = PPred - K*H*PPred;
    PHistoric(:,:,k) = PUpdated;
end
%% CLOSED LOOP CORRECTION
% GNSS/INS Integration navigation solution at epoch
% Output variables in the present
pSkog(k) = pSkog(k) + x(1); % Position correction
vSkog(k) = vSkog(k) + x(2); % Velocity correction
biasAccSkog(k) = biasAccSkog(k) - x(3); % Bias Acc correction
timeDelaySkog(k) = timeDelaySkog(k) + x(4); % Time delay correction
timeDelaySkog(k) = max(0, timeDelaySkog(k));% Time delay can't be negative

end