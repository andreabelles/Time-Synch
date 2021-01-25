function [xNew, PNew, estNew, measAccCorr] = standardEKF(xOld, POld, estOld, pGNSS, measAcc, measAccCorrPrev, Config)
% EKF:  This function estimates the position, velocity and bias based on state-augmented KF
%           from [Skog, Händel, 2011] Time Synchronization Errors in Loosely CoupledGPS-Aided 
%           Inertial Navigation Systems (see Table 1). 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

global COL_EST_POS COL_EST_VEL COL_EST_ACCBIAS

% Sensor error compensation: Get IMU measurements estimations from IMU
% measurements
% f_tilda = f_true + bias_f_true
% delta_b_f = b_f_hat - b_f_true -> b_f_hat = b_f_true + delta_b_f
% f_hat = f_tilda - bias_f_hat = (f_true + bias_f_true) - (b_f_true +
% delta_b_f); if delta_b_f tends to 0, then: f_hat = f_true + bias_f_true -
% b_f_true -> f_hat = f_true so the estimation tends to the true value
measAccCorr = measAcc - estOld(COL_EST_ACCBIAS);  

% Navigation equations computation: Update corrected inertial navigation solution
estPred = navigationEquations(estOld, measAccCorr, measAccCorrPrev, Config.tIMU);

% Initialization
F = [0 1 0; 0 0 1; 0 0 0];
Q = [0 0 0; 0 Config.varAccNoise 0; 0 0 Config.varAccBiasNoise];  

% Discrete transition model
Fk = eye(size(F)) + Config.tIMU*F;
Qk = Config.tIMU*Q;

% Initialize state to 0 for close loop
xOld(1:end)  = 0;

% Time propagation (state prediction) - X_k|k-1 and cov(X_k|k-1)
xNew = Fk * xOld;
% Covariance prediction
PNew = Fk*POld*Fk' + Qk; % Save in case no GNSS measurements available

if (~isnan(pGNSS)) % If GNSS position is available
    
    % Measurement model
    z = pGNSS - estPred(COL_EST_POS);  % Observation vector: GPS - prediction INS  
    
    H = [1 0 0]; % Eq. (21)
    R = Config.varPosGNSS;
    
    % Kalman filter gain computation
    K = (PNew*H')/(H*PNew*H' + R); % From Table 1
    
    % Innovation vector computation 
    dz = z - H * xNew; % Eq. (20)
    
    % Update state vector and covariance matrix
    xNew = xNew + K*dz;
    PNew = PNew - K*H*PNew;
end
%% CLOSED LOOP CORRECTION
% Estimate update
estNew(COL_EST_POS)      = estPred(COL_EST_POS) + xNew(1); % Position correction
estNew(COL_EST_VEL)      = estPred(COL_EST_VEL) + xNew(2); % Velocity correction
estNew(COL_EST_ACCBIAS)  = estPred(COL_EST_ACCBIAS) - xNew(3); % Bias Acc correction
end