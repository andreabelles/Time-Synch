function [x, PHistoric, vSkog, pSkog, biasAccSkog, timeDelaySkog, measAccCorr] = ...
skogEKF(vSkog, pSkog, biasAccSkog, timeDelaySkog, xOld, PHistoric, pGNSS, measAcc, measAccCorr, k, tspan, Config)
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
measAccCorr(k) = measAcc(k) + biasAccSkog(k-1);

% Navigation equations computation: Update corrected inertial navigation solution
vSkog(k) = vSkog(k-1) + 0.5 * (measAccCorr(k) + measAccCorr(k-1)) * Config.tIMU;
pSkog(k) = pSkog(k-1) + 0.5 * (vSkog(k) + vSkog(k-1)) * Config.tIMU;
biasAccSkog(k) = biasAccSkog(k-1);
timeDelaySkog(k) = timeDelaySkog(k-1);

% Initialization
F = [0 1 0 0; 0 0 1 0; 0 0 0 0; 0 0 0 0];
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


if(~isnan(pGNSS)) % If GNSS position is available
    %% Interpolate accelerometer measurement from k to k-Td
    timeAtDelay = tspan(k) - timeDelaySkog(k);
    timeAtDelay = max(0,timeAtDelay);
    % Extracted from Eq. (24) and reference paper [22]. 
    measAccInt = lagrangeInterp(tspan(1:k), measAccCorr(1:k), timeAtDelay); % TODO: check constraint for Td
    
    %% Find xEst and PEst before k-Td
    % Determine index of IMU sample right before k-Td in time vector tspan
    [~, closestIndex] = min(abs(timeAtDelay - tspan)); % Index of tspan closest to delay
    % Condition to keep always the previous sample to k-Td
    if tspan(closestIndex) > timeAtDelay, prevIndex = closestIndex - 1;
    else, prevIndex = closestIndex; end
    
    % Select the state vector and covariance matrix at instant previous to k-Td
    pSkogPrevDelay = pSkog(prevIndex);
    vSkogPrevDelay = vSkog(prevIndex);
    biasAccSkogPrevDelay = biasAccSkog(prevIndex);
    timeDelayPrevDelay = timeDelaySkog(prevIndex);
    
    xPrevDelay = zeros(4,1); % Error-state vector at sample previous delay: [dp dV BiasAcc dtimeDelay]
    PPrevDelay = PHistoric(:,:, prevIndex);
    
    %% Predict rIMU from one sample before k-Td of the IMU to k-Td
    timeToDelay = timeAtDelay - tspan(prevIndex); % Time from previous sample of k-Td until k-Td
    
    % Strapdown equations to obtain state vector prediction at time delay (k-Td) solution
    vSkogPredAtDelay = vSkogPrevDelay + 0.5 * (measAccInt + measAccCorr(prevIndex)) * Config.tIMU;
    pSkogPredAtDelay = pSkogPrevDelay + 0.5 * (vSkogPredAtDelay + vSkogPrevDelay) * Config.tIMU;
    
    % Sensor error update
    biasAccSkogPredAtDelay = biasAccSkogPrevDelay;
    % Delay error update
    timeDelayPredAtDelay = timeDelayPrevDelay;
    
    % Covariance matrix prediction at time delay (k-Td) 
    % Discrete-time state transition model
    FktimeToDelay       = eye(size(F)) + timeToDelay*F; % Taylor expansion 1st order
    QktimeToDelay       = timeToDelay*Q;   

    % Time propagation (state prediction) - X_k|k-1 and cov(X_k|k-1)
    xPredAtDelay = FktimeToDelay * xPrevDelay; % X_k|k-1 = F_k*X_k-1|k-1
    % Covariance prediction
    PPredAtDelay = FktimeToDelay*PPrevDelay*FktimeToDelay' + QktimeToDelay; % P at epoch-Td
        
    %% Update at epoch-Td using GNSS measurements
    % Measurement model
    z = pGNSS - pSkogPredAtDelay; % Observation vector: GPS - prediction INS at epoch-Td
    
    H = [1 0 0 0]; % Eq. (21)
    R = Config.varPosGNSS; 
    
    % Kalman filter gain computation
    K = (PPredAtDelay*H')/(H*PPredAtDelay*H' + R); % From Table 1
    
    % Innovation vector computation 
    dz = z - H * xPredAtDelay; % Eq. (20)
    
    % One-step-ahead Kalman prediction gain
    %Kp = FktimeToDelay*K; % From eq. (5) and (6)
    
    % Innovation vector bias
    %d = measAccInt * (1/2) * (timeDelayPredAtDelay^2); % Eq. (23)
   
%     % Factors to compute the covariance matrix of the augmented system
    %expX = pinv(eye(size(FktimeToDelay))-FktimeToDelay) * (-(1/2) * Kp * measAccInt * PPredAtDelay(4,4)); % Eq. (29)
    %piFactor = (1/4)*(measAccInt^2)*(3*(PPredAtDelay(4,4)^2) - 2*(expX(4)^4)); % Eq. (27)
    %gammaFactor = (PPredAtDelay(4,4)*expX + 2*expX(4)*(PPredAtDelay(1:4,4) - expX(4)*expX))*(measAccInt/2); % Eq. (28)
    
    % Update state vector and covariance matrix at time delay (epoch-Td)
    xUpdatedAtDelay = xPredAtDelay + K*dz; % Eq. (25)
    PUpdatedAtDelay = PPredAtDelay - K*H*PPredAtDelay;%*(Kp*H)' + Kp*R*Kp' ...   % Eq. (26)
                    %+ Kp*piFactor*Kp' - FktimeToDelay*gammaFactor*Kp' - Kp*gammaFactor'*FktimeToDelay';
    
    % Constrained state estimation: Estimate projection
    % TODO 
    % xConstrainedAtDelay = xUpdatedAtDelay;
    % Quadratic programming
    % H = [2 0 0 0; 0 2 0 0; 0 0 2 0; 0 0 0 2];
%     H = pinv(PUpdatedAtDelay);
%     f = (2*xUpdatedAtDelay'*H)';
     A = [0 0 0 -1; 0 0 0 1]; % Constraints: (1) -dTd <= Td equivalent to dT >= -Td
% %                              %              (2) dTd <= tspan - Td            
     b = [timeDelaySkog(k) tspan(k)-timeDelaySkog(k)];
% %     % b = [1; 1];
%     lb = [-100 -100 -100 -100];
%     ub = [100 100 100 100];
%     [xConstrainedAtDelay,fval] = quadprog(H,f,A,b,[],[],lb,ub);
%     
    % Linear programming
    % [xErrConstrainedAtDelay,fval] = linprog(f,A,b,[],[],lb,ub);
    % [xErrConstrainedAtDelay,fval] = fmincon(@(x)0.5*(x(1)^2 + x(2)^2 + x(3)^2 + x(4)^2),xUpdatedAtDelay,A,b,[],[],lb,ub);
    f = @(x) (x - xUpdatedAtDelay)'*pinv(PUpdatedAtDelay)*(x - xUpdatedAtDelay);
    X0 = 0.1*ones(4,1);
    [xConstrainedAtDelay,fval] = fmincon(f,X0,A,b,[],[],[],[]);
    % xConstrainedAtDelay = xUpdatedAtDelay + xErrConstrainedAtDelay;
    % LS: Simon 2006, eq. (7.149)
    % xConstrainedAtDelay = xPredAtDelay - A'*pinv(A*A')*(A*xUpdatedAtDelay - timeDelaySkog(k));
    % MP: Simon 2006, eq. (7.150)
    % xConstrainedAtDelay = xUpdatedAtDelay - PUpdatedAtDelay*A'*pinv(A*PUpdatedAtDelay*A')*(A*xUpdatedAtDelay - timeDelaySkog(k));
    
    % Ansara, Molaei, et al., 2017
%     C = [0 0 0 1];
%     N = zeros(size(xPredAtDelay));
%     N(:,4) = C';
%     alpha = min(-C(4))
%     B = alpha*(eye(size(xPredAtDelay)) - N*pinv((N'*N))*N;
%     xConstrainedAtDelay = xPredAtDelay + B*K*dz; % Kp o K?
    %% CLOSED LOOP CORRECTION
    % GNSS/INS Integration navigation solution at time delay (epoch-Td)
    % Discrete-time state transition model
    FkDelaytoEpoch       = eye(size(F)) + timeDelaySkog(k)*F; % Taylor expansion 1st order
    QkDelaytoEpoch       = timeDelaySkog(k)*Q;
    
    % Time propagation (state prediction) - X_k|k-1 and cov(X_k|k-1) 
    x = FkDelaytoEpoch * xConstrainedAtDelay; % X_k|k-1 = F_k*X_k-1|k-1 % state vector updated at epoch
    % Covariance prediction - covariance matrix updated at epoch
    PUpdated = FkDelaytoEpoch * PUpdatedAtDelay * FkDelaytoEpoch' + QkDelaytoEpoch;
    PHistoric(:,:,k) = PUpdated;
    
    %% Output variables in the past
    % TODO: Implement in case we want to save and plot estimations in the
    % past (at delay)
%     pSkogAtDelay(k) = pSkog(k) + xConstrainedAtDelay(1); % Position correction
%     vSkogAtDelay(k) = vSkogPredAtDelay + xConstrainedAtDelay(2); % Velocity correction
%     biasAccSkogAtDelay(k) = xConstrainedAtDelay(3);  % Bias Acc estimation
%     timeDelaySkogAtDelay(k) = timeDelaySkog(k) + xConstrainedAtDelay(4); % Time Delay correction
end
%% CLOSED LOOP CORRECTION
% GNSS/INS Integration navigation solution at epoch
% Output variables in the present
pSkog(k) = pSkog(k) + x(1); % Position correction
vSkog(k) = vSkog(k) + x(2); % Velocity correction
biasAccSkog(k) = x(3);  % Bias Acc estimation
timeDelaySkog(k) = timeDelaySkog(k) + x(4); % Time Delay correction

end