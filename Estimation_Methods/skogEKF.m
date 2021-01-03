function [estIntSkogPresent, estIntSkogPast, covIntSkogPresent, covIntSkogPast, measAccCorr, measGyroCorr] = ...
    skogEKF(estIntHistoric, estIntSkogPast, PHistoric, covIntSkogPast, pGNSS, measAcc, measAccCorr, measGyro, measGyroCorr, epoch, tspan, tIMU, Config)
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

% From all the previous estimated navigation solution from GNSS/INS integration and covariance matrix, we select the
% previous one
estIntOld.pos       = estIntHistoric.pos(epoch-1,:);
estIntOld.vel       = estIntHistoric.vel(epoch-1);
estIntOld.heading   = estIntHistoric.heading(epoch-1);
estIntOld.biasAcc   = estIntHistoric.biasAcc(epoch-1);
estIntOld.biasGyro  = estIntHistoric.biasGyro(epoch-1);
estIntOld.timeDelay = estIntHistoric.timeDelay(epoch-1);

POld       = PHistoric(:, :, epoch-1); 

% Sensor error compensation: Get IMU measurements estimations from IMU measures
measAccCorr(epoch) = measAcc(epoch) + estIntOld.biasAcc;
measGyroCorr(epoch) = measGyro(epoch) + estIntOld.biasGyro;

% Navigation equations computation: Update corrected inertial navigation solution
[estIntSkogPresent] = navigationEquations(measGyroCorr(epoch), measAccCorr(epoch), estIntHistoric, epoch, tIMU);
estIntSkogPresent.timeDelay(epoch) = estIntOld.timeDelay;

% Initialization
x0         = zeros(7,1); % Error-state vector: [dpNorth dpEast dV dHeading BiasAcc BiasGyro timeDelay]
% z            = zeros(size(pGNSS)); % Innovation vector (= 0 if no GNSS measurements available)     

% Continuous-time state transition model
F       = zeros(7);
F(1, 3) = cosd(estIntSkogPresent.heading(epoch));                              % d PNorth / d vAT
F(1, 4) = -estIntSkogPresent.vel(epoch)*sind(estIntSkogPresent.heading(epoch));       % d PNorth / d heading
F(2, 3) = sind(estIntSkogPresent.heading(epoch));                              % d PEast / d vAT
F(2, 4) = estIntSkogPresent.vel(epoch)*cosd(estIntSkogPresent.heading(epoch));        % d PEast / d heading
F(3, 5) = 1;                                                            % d vAT / biasAcc
F(4, 6) = 1;                                                            % d heading / biasGyro
 
Q       = zeros(7);
Q(3, 3) = Config.varAccNoise;        % Velocity
Q(4, 4) = Config.varGyroNoise;       % Heading
Q(5, 5) = Config.varAccBiasNoise;    % Accelerometer bias
Q(6, 6) = Config.varGyroBiasNoise;   % Gyroscope bias 
Q(7, 7) = Config.varDelayProcessNoise;  % Delay

% Discrete-time state transition model
Fk       = eye(7) + tIMU*F; % Taylor expansion 1st order
Qk       = tIMU*Q;

% Time propagation (state prediction) - X_k|k-1 and cov(X_k|k-1)
x = Fk * x0; % X_k|k-1 = F_k*X_k-1|k-1
% Covariance prediction
P = Fk * POld * Fk' + Qk;


if(~isnan(pGNSS)) % If GNSS position is available
    %% Interpolate accelerometer measurement from epoch to epoch-Td
    timeAtDelay = tspan(epoch-1) - estIntSkogPresent.timeDelay(epoch);
    % Extracted from Eq. (24) and reference paper [22]. 
    measAccInt = lagrangeInterp(tspan(1:epoch), measAccCorr(1:epoch), timeAtDelay); % TODO: check constraint for Td
    measGyroInt = lagrangeInterp(tspan(1:epoch), measGyroCorr(1:epoch), timeAtDelay); % TODO: check constraint for Td
    
    %% Find xEst and PEst before epoch-Td
    % Determine index of IMU sample right before epoch-Td in time vector tspan
    [~, closestIndex] = min(abs(timeAtDelay - tspan)); % Index of tspan closest to delay
    % Condition to keep always the previous sample to epoch-Td
    if tspan(closestIndex) > timeAtDelay, prevIndex = closestIndex - 1;
    else, prevIndex = closestIndex; end
    
    % Select the navigation solution and covariance matrix at instant previous to epoch-Td
    estPrevDelay.pos       = estIntHistoric.pos(prevIndex,:);
    estPrevDelay.vel       = estIntHistoric.vel(prevIndex);
    estPrevDelay.acc        = measAccCorr(prevIndex);  
    estPrevDelay.heading   = estIntHistoric.heading(prevIndex);
    estPrevDelay.headingRate   = measGyroCorr(prevIndex);
    estPrevDelay.biasAcc   = estIntHistoric.biasAcc(prevIndex);
    estPrevDelay.biasGyro  = estIntHistoric.biasGyro(prevIndex);
    estPrevDelay.timeDelay     = estIntHistoric.timeDelay(prevIndex);
    
    xPrevDelay = zeros(7,1); % Error-state vector at sample previous delay: [dpNorth dpEast dV dHeading BiasAcc BiasGyro dtimeDelay]
    PPrevDelay = PHistoric(:,:, prevIndex);
    
    %% Predict rIMU from one sample before epoch-Td of the IMU to epoch-Td
    timeToDelay = timeAtDelay - tspan(prevIndex); % Time from previous sample of epoch-Td until epoch-Td
    
    % Strapdown equations to obtain state vector prediction at time delay (epoch-Td)
    estPredAtDelay.headingRate = measGyroInt;                                                                         % Heading Rate
    estPredAtDelay.heading = estPrevDelay.heading + 0.5 * (estPredAtDelay.headingRate + estPrevDelay.headingRate) * timeToDelay;   % Heanding angle
    estPredAtDelay.acc = measAccInt;                                                                          % Acceleration
    estPredAtDelay.vel = estPrevDelay.vel + 0.5 * (estPredAtDelay.acc + estPrevDelay.acc) * timeToDelay;                            % Velocity
    estPredAtDelay.pos(1) = estPrevDelay.pos(1) + 0.5 * (estPredAtDelay.vel + estPrevDelay.vel) * cosd(estPredAtDelay.heading) * timeToDelay;  % North position
    estPredAtDelay.pos(2) = estPrevDelay.pos(2) + 0.5 * (estPredAtDelay.vel + estPrevDelay.vel) * sind(estPredAtDelay.heading) * timeToDelay;  % East position
    estPredAtDelay.biasAcc   = estPrevDelay.biasAcc;
    estPredAtDelay.biasGyro  = estPrevDelay.biasGyro;
    estPredAtDelay.timeDelay  = estPrevDelay.timeDelay;
    
    % Covariance matrix prediction at time delay (epoch-Td)
    % Continuous-time state transition model
    F       = zeros(7);
    F(1, 3) = cosd(estPredAtDelay.heading);                              % d PNorth / d vAT
    F(1, 4) = -estPredAtDelay.vel*sind(estPredAtDelay.heading);       % d PNorth / d heading
    F(2, 3) = sind(estPredAtDelay.heading);                              % d PEast / d vAT
    F(2, 4) = estPredAtDelay.vel*cosd(estPredAtDelay.heading);        % d PEast / d heading
    F(3, 5) = 1;                                                            % d vAT / biasAcc
    F(4, 6) = 1;                                                            % d heading / biasGyro

    % Discrete-time state transition model
    FktimeToDelay       = eye(7) + timeToDelay*F; % Taylor expansion 1st order
    QktimeToDelay       = timeToDelay*Q;

    % Time propagation (state prediction) - X_k|k-1 and cov(X_k|k-1)
    xPredAtDelay = FktimeToDelay * xPrevDelay; % X_k|k-1 = F_k*X_k-1|k-1
    % Covariance prediction
    PPredAtDelay = FktimeToDelay*PPrevDelay*FktimeToDelay' + QktimeToDelay; % P at epoch-Td
    
    %% Update at epoch-Td using GNSS measurements
    % Measurement model
    z = pGNSS' - estPredAtDelay.pos'; % Observation vector: GPS - prediction INS 
    
    H = [1 0 0 0 0 0 -estPredAtDelay.vel;   ...
         0 1 0 0 0 0 -estPredAtDelay.vel]; % Eq. (21)
    R = diag([Config.varPosGNSS Config.varPosGNSS]);
    
    % Kalman filter gain computation
    K = (PPredAtDelay*H')/(H*PPredAtDelay*H' + R); % From Table 1
    
    % Innovation vector computation 
    dz = z - H * xPredAtDelay; % Eq. (20)
    
    % One-step-ahead Kalman prediction gain
    Kp = FktimeToDelay*K; % From eq. (5) and (6)
    
    % Innovation vector bias
    d = estPredAtDelay.acc * (1/2) * (estPredAtDelay.timeDelay^2); % Eq. (23)
    
    % Factors to compute the covariance matrix of the augmented system
    expX = pinv(eye(size(FktimeToDelay))-FktimeToDelay) * (-(1/2) * Kp * estPredAtDelay.acc * PPredAtDelay(7,7)); % Eq. (29)
    piFactor = (1/4)*(estPredAtDelay.acc^2)*(3*(PPredAtDelay(7,7)^2) - 2*(expX(7)^4)); % Eq. (27)
    gammaFactor = (PPredAtDelay(7,7)*expX + 2*expX(7)*(PPredAtDelay(1:7,7) - expX(7)*expX))*(estPredAtDelay.acc/2); % Eq. (28)
    
    % Update state vector and covariance matrix at time delay (epoch-Td)
    xUpdatedAtDelay = xPredAtDelay + Kp*(dz - d); % Eq. (25)
    PUpdatedAtDelay = PPredAtDelay + Kp*R*Kp' + Kp*piFactor*Kp' - FktimeToDelay*gammaFactor*Kp' - Kp*gammaFactor'*FktimeToDelay'; % Eq. (26)
    
    %% CLOSED LOOP CORRECTION
    % GNSS/INS Integration navigation solution at time delay (epoch-Td)
    estUpdatedAtDelay.pos = estPredAtDelay.pos + xUpdatedAtDelay(1:2)';
    estUpdatedAtDelay.vel = estPredAtDelay.vel + xUpdatedAtDelay(3);
    estUpdatedAtDelay.heading = estPredAtDelay.heading + xUpdatedAtDelay(4);
    estUpdatedAtDelay.biasAcc = xUpdatedAtDelay(5); % Bias Acc estimation
    estUpdatedAtDelay.biasGyro = xUpdatedAtDelay(6); % Bias Gyro estimation
    estUpdatedAtDelay.timeDelay = estPredAtDelay.timeDelay + xUpdatedAtDelay(7); % Time Delay estimation 

    %% Move updated state vector and covariance matrix from epoch-Td to epoch
    % Covariance matrix prediction from time delay (epoch-Td) to epoch
    % Continuous-time state transition model
    FDelaytoEpoch       = zeros(7);
    FDelaytoEpoch(1, 3) = cosd(estUpdatedAtDelay.heading);                              % d PNorth / d vAT
    FDelaytoEpoch(1, 4) = -estUpdatedAtDelay.vel*sind(estUpdatedAtDelay.heading);       % d PNorth / d heading
    FDelaytoEpoch(2, 3) = sind(estUpdatedAtDelay.heading);                              % d PEast / d vAT
    FDelaytoEpoch(2, 4) = estUpdatedAtDelay.vel*cosd(estUpdatedAtDelay.heading);        % d PEast / d heading
    FDelaytoEpoch(3, 5) = 1;                                                            % d vAT / biasAcc
    FDelaytoEpoch(4, 6) = 1;                                                            % d heading / biasGyro
    
    % Discrete-time state transition model
    FkDelaytoEpoch       = eye(7) + estUpdatedAtDelay.timeDelay*FDelaytoEpoch; % Taylor expansion 1st order
    QkDelaytoEpoch       = estUpdatedAtDelay.timeDelay*Q;

    % Time propagation (state prediction) - X_k|k-1 and cov(X_k|k-1) 
    x = FkDelaytoEpoch * xUpdatedAtDelay; % X_k|k-1 = F_k*X_k-1|k-1 % state vector updated at epoch
    % Covariance prediction
    P = FkDelaytoEpoch * PUpdatedAtDelay * FkDelaytoEpoch' + QkDelaytoEpoch; % covariance matrix updated at epoch
    
    %% Output variables in the past
    estIntSkogPast.pos(epoch,:) = estUpdatedAtDelay.pos;
    estIntSkogPast.vel(epoch) = estUpdatedAtDelay.vel;
    estIntSkogPast.heading(epoch) = estUpdatedAtDelay.heading;
    estIntSkogPast.biasAcc(epoch) = estUpdatedAtDelay.biasAcc; % Bias Acc estimation
    estIntSkogPast.biasGyro(epoch) = estUpdatedAtDelay.biasGyro; % Bias Gyro estimation
    estIntSkogPast.timeDelay(epoch) = estUpdatedAtDelay.timeDelay; % Time Delay estimation 
    covIntSkogPast(:,:,epoch) = PUpdatedAtDelay;
end

%% CLOSED LOOP CORRECTION
% GNSS/INS Integration navigation solution at epoch
% Output variables in the present
estIntSkogPresent.pos(epoch,:) = estIntSkogPresent.pos(epoch,:) + x(1:2)';
estIntSkogPresent.vel(epoch) = estIntSkogPresent.vel(epoch) + x(3);
estIntSkogPresent.heading(epoch) = estIntSkogPresent.heading(epoch) + x(4);
estIntSkogPresent.biasAcc(epoch) = x(5); % Bias Acc estimation
estIntSkogPresent.biasGyro(epoch) = x(6); % Bias Gyro estimation
estIntSkogPresent.timeDelay(epoch) = estIntSkogPresent.timeDelay(epoch) + x(7); % Time Delay estimation
covIntSkogPresent(:,:,epoch) = P;   
end