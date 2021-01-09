function [pIMU, vIMU, pGNSS, xEKF, PEKF, xSkog, PSkog, vEKF, pEKF, biasAccEKF, pSkog, vSkog, biasAccSkog, timeDelaySkog] = ...
                        simulateEstimations(p, tspan, Config)

%% Initializations

nPts    = length(tspan);

% IMU Sensor Measurements
measAcc = zeros(nPts,1);
measAccCorr = zeros(nPts,1);
% IMU-Only Navigation Solution
vIMU    = zeros(nPts,1);    
vIMU(1) = Config.v0;
pIMU    = zeros(nPts,1);    
pIMU(1) = Config.p0;
% GNSS-Only Navigation Solution
pGNSS   = nan(nPts,1);

% Standard EKF: 
% GNSS/INS Integration Navigation Solution
pEKF    = zeros(nPts,1);
pEKF(1) = Config.p0;
vEKF    = zeros(nPts,1);
vEKF(1) = Config.v0;
biasAccEKF = zeros(nPts,1);
% Error-State Vector
xEKF        = zeros(3,nPts);
% Covariance matrix
PEKF        = zeros(3,3,nPts);
PEKF(:,:,1) = [Config.sigmaInitPos^2 0 0; ...
               0 Config.sigmaInitVel^2 0; ...
               0 0 Config.sigmaInitAccBias^2];

% Skog EKF: 
% GNSS/INS Integration Navigation Solution
pSkog    = zeros(nPts,1);
pSkog(1) = Config.p0;
vSkog    = zeros(nPts,1);
vSkog(1) = Config.v0;
biasAccSkog = zeros(nPts,1);
timeDelaySkog = zeros(nPts,1);
% Error-State Vector
xSkog        = zeros(4,nPts);
% Covariance matrix
PSkog        = zeros(4,4,nPts);
PSkog(:,:,1) = [Config.sigmaInitPos^2 0 0 0; ...
                0 Config.sigmaInitVel^2 0 0; ...
                0 0 Config.sigmaInitAccBias^2 0; ...
                0 0 0 Config.sigmaInitDelay^2];

for k = 2:1:nPts
    % IMU measurements generation
    measAcc(k) = Config.a0 + Config.biasAcc + normrnd(0,sqrt(Config.varMeasAcc));
    % GNSS measurements generation
    if mod(k,Config.M) == 0 && (k - Config.tDelay/Config.tIMU)>0
        pGNSS(k) = p(k - Config.tDelay/Config.tIMU) + normrnd(0,sqrt(Config.varMeasPosGNSS));
    end
    % Strapdown equations
    vIMU(k) = vIMU(k-1) + measAcc(k)*Config.tIMU;
    pIMU(k) = pIMU(k-1) + vIMU(k)*Config.tIMU;

    %EKF
    [xEKF(:,k), PEKF(:,:,k), vEKF, pEKF, biasAccEKF, measAccCorr] = ...
                                  standardEKF(  xEKF(:,k-1),    ...
                                                PEKF(:,:,k-1),  ...
                                                pGNSS(k),       ...
                                                k,              ...
                                                measAcc,        ...
                                                measAccCorr,    ...
                                                pEKF, ...    
                                                vEKF, ...
                                                biasAccEKF,...
                                                Config);
    
    [xSkog(:,k), PSkog, vSkog, pSkog, biasAccSkog, timeDelaySkog, measAccCorr] = skogEKF(               ...
                                                    vSkog,           ...
                                                    pSkog,           ...
                                                    biasAccSkog,     ...
                                                    timeDelaySkog,   ...
                                                    xSkog(:,k-1),           ...
                                                    PSkog,              ...
                                                    pGNSS(k),           ...
                                                    measAcc,            ...
                                                    measAccCorr,        ...
                                                    k,                  ...
                                                    tspan,              ...
                                                    Config);                                                
    % TODO: Lee and Johnson method
    
     
end


end