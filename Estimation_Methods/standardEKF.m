function [x, PHistoric, estEKF, measAccCorr, measGyroCorr] = ...
    standardEKF(xOld, PHistoric, estEKF, pGNSS, k, measAcc, measGyro, measAccCorr, measGyroCorr, Config)

% EKF:  This function estimates the position, velocity and bias based on state-augmented KF
%           from [Skog, H�ndel, 2011] Time Synchronization Errors in Loosely CoupledGPS-Aided 
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
measAccCorr(k) = measAcc(k) - estEKF.biasAcc(k-1); 
measGyroCorr(k) = measGyro(k) - estEKF.biasGyro(k-1);

% Navigation equations computation: Update corrected inertial navigation solution
%[estEKF] = navigationEquations(measGyroCorr(k), measAccCorr(k), estEKF, k, Config.tIMU);

estEKF.heading(k) = estEKF.heading(k-1);                                                                         % Acceleration
estEKF.vel(k) = estEKF.vel(k-1) + 0.5 * (measAccCorr(k) + measAccCorr(k-1)) * Config.tIMU;                            % Velocity
estEKF.pos(k,1) = estEKF.pos(k-1,1) + 0.5 * (estEKF.vel(k) + estEKF.vel(k-1)) * cos(estEKF.heading(k)) * Config.tIMU;     % North position
estEKF.pos(k,2) = estEKF.pos(k-1,2) + 0.5 * (estEKF.vel(k) + estEKF.vel(k-1)) * sin(estEKF.heading(k)) * Config.tIMU;     % East position
estEKF.biasAcc(k) = estEKF.biasAcc(k-1);

% Continuous-time state transition model
% F       = zeros(6);
% F(1, 3) = cos(estEKF.heading(k));                              % d PNorth / d vAT
% F(1, 4) = -estEKF.vel(k)*sin(estEKF.heading(k));       % d PNorth / d heading
% F(2, 3) = sin(estEKF.heading(k));                              % d PEast / d vAT
% F(2, 4) = estEKF.vel(k)*cos(estEKF.heading(k));        % d PEast / d heading
% F(3, 5) = 1;                                                            % d vAT / biasAcc
% F(4, 6) = 1;                                              % d heading / biasGyro

% Q       = zeros(6);
% Q(3, 3) = Config.varAccNoise;        % Velocity
% Q(4, 4) = Config.varGyroNoise;       % Heading
% Q(5, 5) = Config.varAccBiasNoise;    % Accelerometer bias
% Q(6, 6) = Config.varGyroBiasNoise;   % Gyroscope bias

F       = zeros(4);
F(1, 3) = cos(estEKF.heading(k));                              % d PNorth / d vAT
F(2, 3) = sin(estEKF.heading(k));                              % d PEast / d vAT

Q       = zeros(4);
Q(3, 3) = Config.varAccNoise;        % Velocity
Q(4, 4) = Config.varAccBiasNoise;   % Acc bias
% Discrete-time state transition model
Fk       = eye(size(F)) + Config.tIMU*F; % Taylor expansion 1st order
Qk       = Config.tIMU*Q;

% Initialization
% xOld         = zeros(6,1); % Error-state vector: [dpNorth dpEast dV dHeading BiasAcc BiasGyro]
xOld         = zeros(4,1); % Error-state vector: [dpNorth dpEast dV dHeading BiasAcc BiasGyro]

% Time propagation (state prediction) - X_k|k-1 and cov(X_k|k-1)
x = Fk * xOld; % X_k|k-1 = F_k*X_k-1|k-1
% Covariance prediction
PPred = Fk*POld*Fk' + Qk;
PHistoric(:,:,k) = PPred; % Save in case no GNSS measurements

if (~isnan(pGNSS)) % If GNSS position is available
    
    % Measurement model
    z = pGNSS' - estEKF.pos(k,:)'; % Observation vector: GPS - prediction INS 
    
%     H = [1 0 0 0 0 0;   ...
%          0 1 0 0 0 0]; % Eq. (21)
    H = [1 0 0 0;   ...
         0 1 0 0];
    R = diag([Config.varPosGNSS Config.varPosGNSS]);
    
    % Kalman filter gain computation
    K = (PPred*H')/(H*PPred*H' + R); % From Table 1
    
    % Innovation vector computation 
    dz =  z - H * x; % Eq. (20)
    
    % Update state prediction (state estimation)
    x = x + K*dz; % X_k|k = X_k|k-1 + Kdz 
    PUpdated = PPred - K*H*PPred;
    PHistoric(:,:,k) = PUpdated; 
end

%% CLOSED-LOOP CORRECTION
% GNSS/INS Integration navigation solution
% Output variables in the present
estEKF.pos(k,:) = estEKF.pos(k,:) + x(1:2)'; % Position correction
estEKF.vel(k) = estEKF.vel(k) + x(3); % Velocity correction
%estEKF.heading(k) = estEKF.heading(k) + x(4); % Heading correction
estEKF.biasAcc(k) = estEKF.biasAcc(k) - x(4); % Bias Acc correction
%estEKF.biasGyro(k) = x(6); % Bias Gyro estimation 

end