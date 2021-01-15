function [pIMU, vIMU, pGNSS, xEKF, PEKF, xSkog, PSkog, vEKF, pEKF, biasAccEKF, pSkog, vSkog, biasAccSkog, timeDelaySkog, biasGPSSkog, driftGPSSkog] = ...
                        simulateEstimations(p, tspan, Config)

%% Initializations

nPts    = length(tspan);

% IMU Sensor Measurements
measAcc = zeros(nPts,1);
measAccCorrEKF = zeros(nPts,1);
measAccCorrSkog = zeros(nPts,1);

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
pEKF(1) = Config.rp0;
vEKF    = zeros(nPts,1);
vEKF(1) = Config.rv0;
biasAccEKF = zeros(nPts,1);
biasAccEKF(1) = Config.rba0;
% Error-State Vector
xEKF        = zeros(3,nPts);
% Covariance matrix
PEKF        = zeros(3,3,nPts);
PEKF(:,:,1) = diag([Config.sigmaInitPos^2; ...
                    Config.sigmaInitVel^2; ...
                    Config.sigmaInitAccBias^2]);

% Skog EKF: 
% GNSS/INS Integration Navigation Solution
pSkog    = zeros(nPts,1);
pSkog(1) = Config.rp0;
vSkog    = zeros(nPts,1);
vSkog(1) = Config.rv0;
biasAccSkog = zeros(nPts,1);
biasAccSkog(1) = Config.rba0;
timeDelaySkog = zeros(nPts,1);
timeDelaySkog(1) = Config.rt0;
biasGPSSkog    = zeros(nPts,1);
biasGPSSkog(1) = Config.biasGPS0;
driftGPSSkog = zeros(nPts,1);
driftGPSSkog(1) = Config.driftGPS0;
% Error-State Vector
xSkog        = zeros(5,nPts);
% Covariance matrix
PSkog        = zeros(5,5,nPts);
PSkog(:,:,1) = diag([Config.sigmaInitPos^2; ...
                    Config.sigmaInitVel^2; ...
                    Config.sigmaInitAccBias^2; ...
                    Config.sigmaInitBiasGPS^2; ...
                    Config.sigmaInitDriftGPS^2]);

for k = 2:1:nPts
    % IMU measurements generation
    measAcc(k) = Config.a0 + Config.biasMeasAcc + normrnd(0,sqrt(Config.varMeasAcc));
    % GNSS measurements generation
    if mod(k,Config.M) == 0 && (k - Config.tDelay/Config.tIMU)>0
        pGNSS(k) = p(k - Config.tDelay/Config.tIMU) + normrnd(0,sqrt(Config.varMeasPosGNSS));
    end
    % Strapdown equations
    vIMU(k) = vIMU(k-1) + measAcc(k)*Config.tIMU;
    pIMU(k) = pIMU(k-1) + vIMU(k)*Config.tIMU;

    %EKF
%     [xEKF(:,k), PEKF, vEKF, pEKF, biasAccEKF, measAccCorrEKF] = ...
%                                     standardEKF(xEKF(:,k-1),    ...
%                                                 PEKF,           ...
%                                                 pGNSS(k),       ...
%                                                 k,              ...
%                                                 measAcc,        ...
%                                                 measAccCorrEKF,    ...
%                                                 pEKF, ...    
%                                                 vEKF, ...
%                                                 biasAccEKF,...
%                                                 Config);
    
    [xSkog(:,k), PSkog, vSkog, pSkog, biasAccSkog, timeDelaySkog, biasGPSSkog, driftGPSSkog, measAccCorrSkog] = ...
                                            skogEKF(xSkog(:,k-1),           ...
                                                    PSkog,              ...
                                                    pGNSS(k),           ...
                                                    k,                  ...
                                                    measAcc,            ...
                                                    measAccCorrSkog,        ...
                                                    pSkog,           ...
                                                    vSkog,           ...
                                                    biasAccSkog,     ...
                                                    timeDelaySkog,   ...
                                                    biasGPSSkog, ...
                                                    driftGPSSkog, ...
                                                    tspan,           ...
                                                    Config);                                                
    % TODO: Lee and Johnson method
    
     
end


end