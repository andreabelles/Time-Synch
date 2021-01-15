function [x, PHistoric, estSkog, measAccCorr, measGyroCorr] = ...
    skogEKF(xOld, PHistoric, estSkog, pGNSS, k, measAcc, measAccCorr, ...
            measGyro, measGyroCorr, tspan, Config)

% skogEKF:  This function estimates the position, velocity, bias and time delay based on state-augmented KF
%           from [Skog, Händel, 2011] Time Synchronization Errors in Loosely CoupledGPS-Aided Inertial Navigation Systems.
%           This algorithm has been modified to estimate a total-state instead of an error-state KF and also to 
%           propagate the estimation from previous delays to the present. 
%
% Inputs:   pIMU:   Position estimated by the IMU
%           pIMU:   Position estimated by the IMU
%
% Outputs:  delta x:      Error state vector estimation

% --> TODO: define indexes of state vector as variables

% From all the previous covariance matrix, we select the previous one
POld           = PHistoric(:, :, k-1); 

% Sensor error compensation: Get IMU measurements estimations from IMU
% measurements
% f_tilda = f_true + bias_f_true
% delta_b_f = b_f_hat - b_f_true -> b_f_hat = b_f_true + delta_b_f
% f_hat = f_tilda - bias_f_hat = (f_true + bias_f_true) - (b_f_true +
% delta_b_f); if delta_b_f tends to 0, then: f_hat = f_true + bias_f_true -
% b_f_true -> f_hat = f_true so the estimation tends to the true value
measAccCorr(k) = measAcc(k) - estSkog.biasAcc(k-1); 
measGyroCorr(k) = measGyro(k) - estSkog.biasGyro(k-1);

% Navigation equations computation: Update corrected inertial navigation solution
[estSkog] = navigationEquations(measGyroCorr(k), measAccCorr(k), estSkog, k, Config.tIMU);
estSkog.biasAcc(k) = estSkog.biasAcc(k-1);
estSkog.biasGyro(k) = estSkog.biasGyro(k-1);
estSkog.timeDelay(k) = estSkog.timeDelay(k-1); % Time delay modeled as constant

% Continuous-time state transition model
F       = zeros(7);
F(1, 3) = cos(estSkog.heading(k));                              % d PNorth / d vAT
F(1, 4) = -estSkog.vel(k)*sin(estSkog.heading(k));       % d PNorth / d heading
F(2, 3) = sin(estSkog.heading(k));                              % d PEast / d vAT
F(2, 4) = estSkog.vel(k)*cos(estSkog.heading(k));        % d PEast / d heading
F(3, 5) = 1;                                                            % d vAT / biasAcc
F(4, 6) = 1;                                                            % d heading / biasGyro
 
Q       = zeros(7);
Q(3, 3) = Config.varAccNoise;        % Velocity
Q(4, 4) = Config.varGyroNoise;       % Heading
Q(5, 5) = Config.varAccBiasNoise;    % Accelerometer bias
Q(6, 6) = Config.varGyroBiasNoise;   % Gyroscope bias 
Q(7, 7) = Config.varDelayProcessNoise;  % Time Delay

% Discrete-time state transition model
Fk       = eye(size(F)) + Config.tIMU*F; % Taylor expansion 1st order
Qk       = Config.tIMU*Q;

% Initialization
x0         = zeros(7,1); % Error-state vector: [dpNorth dpEast dV dHeading BiasAcc BiasGyro timeDelay]

% Time propagation (state prediction) - X_k|k-1 and cov(X_k|k-1)
x = Fk * x0; % X_k|k-1 = F_k*X_k-1|k-1
% Covariance prediction
PPred = Fk * POld * Fk' + Qk;
PHistoric(:,:, k) = PPred; % Save in case no GNSS measurements

if ~isnan(pGNSS) % If GNSS position is available
    %% Interpolate accelerometer measurement from k to k-Td
    timeAtDelay = tspan(k) - estSkog.timeDelay(k);
    timeAtDelay = max(0,timeAtDelay);
    % Extracted from Eq. (24) and reference paper [22]. 
    measAccInt = lagrangeInterp(tspan(1:k), measAccCorr(1:k), timeAtDelay); % TODO: check constraint for Td
    measGyroInt = lagrangeInterp(tspan(1:k), measGyroCorr(1:k), timeAtDelay); % TODO: check constraint for Td
    
    %% Find xEst and PEst before k-Td
    % Determine index of IMU sample right before k-Td in time vector tspan
    [~, closestIndex] = min(abs(timeAtDelay - tspan)); % Index of tspan closest to delay
    % Condition to keep always the previous sample to k-Td
    if tspan(closestIndex) > timeAtDelay, prevIndex = closestIndex - 1;
    else, prevIndex = closestIndex; end
    
    % Select the navigation solution and covariance matrix at sample previous to k-Td
    estPrevDelay.pos            = estSkog.pos(prevIndex,:);
    estPrevDelay.vel            = estSkog.vel(prevIndex);
    estPrevDelay.acc            = measAccCorr(prevIndex);  
    estPrevDelay.heading        = estSkog.heading(prevIndex);
    estPrevDelay.headingRate    = measGyroCorr(prevIndex);
    estPrevDelay.biasAcc        = estSkog.biasAcc(prevIndex);
    estPrevDelay.biasGyro       = estSkog.biasGyro(prevIndex);
    estPrevDelay.timeDelay      = estSkog.timeDelay(prevIndex);
    
    xPrevDelay = zeros(7,1); % Error-state vector at sample previous delay: [dpNorth dpEast dV dHeading BiasAcc BiasGyro dtimeDelay]
    PPrevDelay = PHistoric(:,:, prevIndex);
    
    %% Predict estimation from one sample before k-Td of the IMU to k-Td
    timeToDelay = timeAtDelay - tspan(prevIndex); % Time from previous sample of k-Td until k-Td
    
    % Strapdown equations to obtain state vector prediction at time delay (k-Td)
    estPredAtDelay.headingRate = measGyroInt;          % Heading Rate
    estPredAtDelay.heading = estPrevDelay.heading + ...% Heanding angle
        0.5 * (estPredAtDelay.headingRate + estPrevDelay.headingRate) * timeToDelay;   
    estPredAtDelay.acc = measAccInt;                   % Acceleration
    estPredAtDelay.vel = estPrevDelay.vel + 0.5 * (estPredAtDelay.acc + estPrevDelay.acc) * timeToDelay;  % Velocity
    estPredAtDelay.pos(1) = estPrevDelay.pos(1) + ...  % North position
        0.5 * (estPredAtDelay.vel + estPrevDelay.vel) * cos(estPredAtDelay.heading) * timeToDelay;  
    estPredAtDelay.pos(2) = estPrevDelay.pos(2) + ...  % East position
        0.5 * (estPredAtDelay.vel + estPrevDelay.vel) * sin(estPredAtDelay.heading) * timeToDelay;
    estPredAtDelay.biasAcc   = estPrevDelay.biasAcc;
    estPredAtDelay.biasGyro  = estPrevDelay.biasGyro;
    estPredAtDelay.timeDelay = estPrevDelay.timeDelay;
    
    % Continuous-time state transition modelat time delay (k-Td)
    FtimeToDelay       = zeros(7);
    FtimeToDelay(1, 3) = cos(estPredAtDelay.heading);                              % d PNorth / d vAT
    FtimeToDelay(1, 4) = -estPredAtDelay.vel*sin(estPredAtDelay.heading);       % d PNorth / d heading
    FtimeToDelay(2, 3) = sin(estPredAtDelay.heading);                              % d PEast / d vAT
    FtimeToDelay(2, 4) = estPredAtDelay.vel*cos(estPredAtDelay.heading);        % d PEast / d heading
    FtimeToDelay(3, 5) = 1;                                                            % d vAT / biasAcc
    FtimeToDelay(4, 6) = 1;                                                            % d heading / biasGyro

    % Discrete-time state transition model
    FktimeToDelay       = eye(7) + timeToDelay*FtimeToDelay; % Taylor expansion 1st order
    QktimeToDelay       = timeToDelay*Q;

    % Time propagation (state prediction) - X_k|k-1 and cov(X_k|k-1)
    xPredAtDelay = FktimeToDelay * xPrevDelay; % X_k|k-1 = F_k*X_k-1|k-1
    % Covariance prediction
    PPredAtDelay = FktimeToDelay*PPrevDelay*FktimeToDelay' + QktimeToDelay; % P at k-Td
    
    %% Update at k-Td using GNSS measurements
    % Measurement model
    z = pGNSS' - estPredAtDelay.pos'; % Observation vector: GPS - prediction INS at k-Td
    
    H = [1 0 0 0 0 0 0;   ...
         0 1 0 0 0 0 0]; % Eq. (21)
    R = diag([Config.varPosGNSS Config.varPosGNSS]);
    
    % Kalman filter gain computation
    K = (PPredAtDelay*H')/(H*PPredAtDelay*H' + R); % From Table 1
    
    % Innovation vector computation 
    dz = z - H * xPredAtDelay; % Eq. (20)
    
    % Update state vector and covariance matrix at time delay (epoch-Td)
    xUpdatedAtDelay = xPredAtDelay + K*dz; % A posteriori error-state 
    PUpdatedAtDelay = PPredAtDelay - K*H*PPredAtDelay; % A posteriori covariance
        
    % Constrained state estimation: Estimate projection
%     d = - estPredAtDelay.timeDelay;
%     D = zeros(1,7);
%     D(1, 7) = 1;
%     xConstrainedAtDelay = xUpdatedAtDelay - PUpdatedAtDelay*D'*pinv(D*PUpdatedAtDelay*D')*(D*xUpdatedAtDelay - d);
      
    % Constrained state estimation: PDF truncation
%     D = zeros(7,1); %Phi_k from Lee and Johnson 2017
%     D(1,1) = 1;  
%     lowerBound = - estPredAtDelay.timeDelay;
%     upperBound = timeAtDelay - estPredAtDelay.timeDelay;
%     [T, W] = jordan(PUpdatedAtDelay);
%     rho = zeros(7,7);
%     rho(1,:) = (D'*T*sqrt(W))/sqrt(D'*PUpdatedAtDelay*D);

    xConstrainedAtDelay = xUpdatedAtDelay;
    %% CLOSED LOOP CORRECTION
    % GNSS/INS Integration navigation solution at time delay (k-Td)
    estUpdatedAtDelay.pos = estPredAtDelay.pos + xConstrainedAtDelay(1:2)';
    estUpdatedAtDelay.vel = estPredAtDelay.vel + xConstrainedAtDelay(3);
    estUpdatedAtDelay.heading = estPredAtDelay.heading + xConstrainedAtDelay(4);
    estUpdatedAtDelay.biasAcc = estPredAtDelay.biasAcc - xConstrainedAtDelay(5); % Bias Acc correction
    estUpdatedAtDelay.biasGyro = estPredAtDelay.biasGyro - xConstrainedAtDelay(6); % Bias Gyro correction
    estUpdatedAtDelay.timeDelay = estPredAtDelay.timeDelay + xConstrainedAtDelay(7); % Time Delay correction 

    %% Move updated state vector and covariance matrix from k-Td to k
    % Continuous-time state transition model
    FDelaytoEpoch       = zeros(7);
    FDelaytoEpoch(1, 3) = cos(estUpdatedAtDelay.heading);                              % d PNorth / d vAT
    FDelaytoEpoch(1, 4) = -estUpdatedAtDelay.vel*sin(estUpdatedAtDelay.heading);       % d PNorth / d heading
    FDelaytoEpoch(2, 3) = sin(estUpdatedAtDelay.heading);                              % d PEast / d vAT
    FDelaytoEpoch(2, 4) = estUpdatedAtDelay.vel*cos(estUpdatedAtDelay.heading);        % d PEast / d heading
    FDelaytoEpoch(3, 5) = 1;                                                            % d vAT / biasAcc
    FDelaytoEpoch(4, 6) = 1;                                                            % d heading / biasGyro
    
    % Discrete-time state transition model
    FkDelaytoEpoch       = eye(7) + estSkog.timeDelay(k)*FDelaytoEpoch; % Taylor expansion 1st order
    QkDelaytoEpoch       = estSkog.timeDelay(k)*Q;
    
    % Time propagation (state prediction) - X_k|k-1 and cov(X_k|k-1) 
    x = FkDelaytoEpoch * xConstrainedAtDelay; % X_k|k-1 = F_k*X_k-1|k-1 % state vector updated at k
    % Covariance prediction - covariance matrix updated at k
    PUpdated = FkDelaytoEpoch * PUpdatedAtDelay * FkDelaytoEpoch' + QkDelaytoEpoch; 
    PHistoric(:,:, k) = PUpdated;
    
    %% Output variables in the past
%     estIntSkogAtDelay.pos(k,:) = estUpdatedAtDelay.pos;
%     estIntSkogAtDelay.vel(k) = estUpdatedAtDelay.vel;
%     estIntSkogAtDelay.heading(k) = estUpdatedAtDelay.heading;
%     estIntSkogAtDelay.biasAcc(k) = estUpdatedAtDelay.biasAcc; % Bias Acc estimation
%     estIntSkogAtDelay.biasGyro(k) = estUpdatedAtDelay.biasGyro; % Bias Gyro estimation
%     estIntSkogAtDelay.timeDelay(k) = estUpdatedAtDelay.timeDelay; % Time Delay estimation 
%     covIntSkogAtDelay(:,:,k) = PUpdatedAtDelay;
end

%% CLOSED LOOP CORRECTION
% GNSS/INS Integration navigation solution at k
% Output variables in the present
estSkog.pos(k,:) = estSkog.pos(k,:) + x(1:2)';
estSkog.vel(k) = estSkog.vel(k) + x(3);
estSkog.heading(k) = estSkog.heading(k) + x(4);
estSkog.biasAcc(k) = estSkog.biasAcc(k) - x(5); % Bias Acc correction
estSkog.biasGyro(k) = estSkog.biasGyro(k) - x(6); % Bias Gyro correction
estSkog.timeDelay(k) = estSkog.timeDelay(k) + x(7); % Time Delay correction
% estSkog.timeDelay(k) = abs(estSkog.timeDelay(k)); % Bound so that delay is not < 0
estSkog.timeDelay(k) = max(0, estSkog.timeDelay(k)); % Bound so that delay is not < 0
% covIntSkogPresent(:,:,k) = P;   
end