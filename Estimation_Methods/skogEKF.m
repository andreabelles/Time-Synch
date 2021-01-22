function [xNew, PNew, estNew, measAccCorr, estPresent, PPresent] = ...
                                            skogEKF(xOld,       ...
                                                    POld,       ...
                                                    estOld,     ...
                                                    pGNSSHist,  ...
                                                    measAccHist,...
                                                    k,          ...
                                                    tspan,      ...
                                                    Config)
% EKF:  This function estimates the position, velocity and bias based on state-augmented KF
%           from [Skog, Händel, 2011] Time Synchronization Errors in Loosely CoupledGPS-Aided 
%           Inertial Navigation Systems (see Table 1). 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

global COL_EST_POS COL_EST_VEL COL_EST_ACCBIAS COL_EST_DELAY

% Sensor error compensation: Get IMU measurements estimations from IMU
% measurements
% f_tilda = f_true + bias_f_true
% delta_b_f = b_f_hat - b_f_true -> b_f_hat = b_f_true + delta_b_f
% f_hat = f_tilda - bias_f_hat = (f_true + bias_f_true) - (b_f_true +
% delta_b_f); if delta_b_f tends to 0, then: f_hat = f_true + bias_f_true -
% b_f_true -> f_hat = f_true so the estimation tends to the true value

if all(isnan(pGNSSHist)) % we need to wait tGNSS to find the first GNSS measurement available, otherwise we only have IMU
    measAccCorr = measAccHist(k) - estOld(COL_EST_ACCBIAS);
else
    timeAtDelay = tspan(k) - estOld(COL_EST_DELAY);
    measAccInt = interp1(tspan(1:k), measAccHist(1:k), timeAtDelay);
    measAccCorr = measAccInt - estOld(COL_EST_ACCBIAS);
end
% Navigation equations computation: Update corrected inertial navigation solution
estPred = navigationEquations(estOld, measAccCorr, Config.tIMU);

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
xOld(1:end)  = 0;

% Time propagation (state prediction) - X_k|k-1 and cov(X_k|k-1)
xNew = Fk * xOld;
% Covariance prediction
PNew = Fk*POld*Fk' + Qk; % Save in case no GNSS measurements

if (~isnan(pGNSSHist(k))) % If GNSS position is available
    
    % Measurement model
    z = pGNSSHist(k) - estPred(COL_EST_POS);  % Observation vector: GPS - prediction INS  
    
    H = [1 0 0 -estPred(COL_EST_VEL)]; % Eq. (21)
    R = Config.varPosGNSS;
    
    % Kalman filter gain computation
    K = (PNew*H')/(H*PNew*H' + R); % From Table 1
    
    % Innovation vector computation 
    dz = z - H * xNew;
    
    % Update state vector and covariance matrix
    xNew = xNew + K*dz;
    PNew = PNew - K*H*PNew;
end
%% CLOSED LOOP CORRECTION
% Estimate update at delayed time
estNew(COL_EST_POS)     = estPred(COL_EST_POS) + xNew(1); % Position correction
estNew(COL_EST_VEL)     = estPred(COL_EST_VEL) + xNew(2); % Velocity correction
estNew(COL_EST_ACCBIAS) = estPred(COL_EST_ACCBIAS) - xNew(3); % Bias Acc correction
estNew(COL_EST_DELAY)   = estPred(COL_EST_DELAY) + xNew(4); % Time delay correction
% Constraint delay
estNew(COL_EST_DELAY)   = max(0, estNew(COL_EST_DELAY));% Time delay can't be negative
estNew(COL_EST_DELAY)   = min(tspan(k), estNew(COL_EST_DELAY));% Time delay can't be greater than the actual time

%% ESTIMATION AT PRESENT
% Navigation equations to move estimation to the present
estPresent = navigationEquations(estNew, measAccHist(k), estNew(COL_EST_DELAY));

% Prediction of covariance at present
FkPresent = eye(size(F)) + estNew(COL_EST_DELAY)*F;
QkPresent = estNew(COL_EST_DELAY)*Q;
PPresent  = FkPresent*PNew*FkPresent' + QkPresent;

end