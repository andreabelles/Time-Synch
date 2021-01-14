function [x, PHistoric, vEKF, pEKF, psiEKF, biasAccEKF, measAccCorr] = standardEKF(xOld, ...
                                                                        PHistoric, ...
                                                                        pGNSS, ...
                                                                        k, ...
                                                                        measAcc, ...
                                                                        measAccCorr, ...
                                                                        pEKF, ...
                                                                        vEKF, ...
                                                                        psiEKF, ...
                                                                        biasAccEKF, ...
                                                                        Config)

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
measAccCorr(k) = measAcc(k) - biasAccEKF(k-1);  

% Navigation equations computation: Update corrected inertial navigation solution
psiEKF(k) = psiEKF(k-1);
vEKF(k) = vEKF(k-1) + 0.5 * (measAccCorr(k) + measAccCorr(k-1)) * Config.tIMU;
pEKF(k,1) = pEKF(k-1,1) + 0.5 * (vEKF(k) + vEKF(k-1))*cos(psiEKF(k)) * Config.tIMU;
pEKF(k,2) = pEKF(k-1,2) + 0.5 * (vEKF(k) + vEKF(k-1))*sin(psiEKF(k)) * Config.tIMU;
biasAccEKF(k) = biasAccEKF(k-1);
% Initialization
F = [0 0 cos(psiEKF(k)) 0; 0 0 sin(psiEKF(k)) 0; 0 0 0 1; 0 0 0 0];
Q = [0 0 0 0; 0 0 0 0; 0 0 Config.varAccNoise 0; 0 0 0 Config.varAccBiasNoise];  

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

if (~isnan(pGNSS)) % If GNSS position is available
    
    % Measurement model
    z = pGNSS' - pEKF(k,:)';  % Observation vector: GPS - prediction INS  
    
    H = [1 0 0 0; ...
         0 1 0 0]; % Eq. (21)
    R = diag([Config.varPosGNSS Config.varPosGNSS]);
    
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
psiEKF(k) = psiEKF(k-1);
pEKF(k,1) = pEKF(k,1) + x(1); % Position correction
pEKF(k,2) = pEKF(k,2) + x(2); % Position correction
vEKF(k) = vEKF(k) + x(3); % Velocity correction
biasAccEKF(k) = biasAccEKF(k) - x(4); % Bias Acc correction
end