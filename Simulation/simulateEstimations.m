function [pIMU, vIMU, psiIMU, pGNSS, xEKF, PEKF, xSkog, PSkog, vEKF, pEKF, psiEKF, biasAccEKF, pSkog, vSkog, psiSkog, biasAccSkog, timeDelaySkog] = ...
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
pIMU    = zeros(nPts,2);    
pIMU(1,:) = [Config.pNorth0 Config.pEast0];
psiIMU    = zeros(nPts,1);    
psiIMU(1) = Config.psi0;

% GNSS-Only Navigation Solution
pGNSS   = nan(nPts,2);

% Standard EKF:
% GNSS/INS Integration Navigation Solution
pEKF    = zeros(nPts,2);
pEKF(1,:) = [Config.rpNorth0 Config.rpEast0];
vEKF    = zeros(nPts,1);
vEKF(1) = Config.rv0;
biasAccEKF = zeros(nPts,1);
biasAccEKF(1) = Config.rba0;
psiEKF    = zeros(nPts,1);    
psiEKF(1) = Config.rpsi0;

% Error-State Vector
xEKF        = zeros(4,nPts);
% Covariance matrix
PEKF        = zeros(4,4,nPts);
PEKF(:,:,1) = [Config.sigmaInitNorthPos^2 0 0 0; ...
               0 Config.sigmaInitEastPos^2 0 0; ...
               0 0 Config.sigmaInitVel^2 0; ...
               0 0 0 Config.sigmaInitAccBias^2];

% Skog EKF: 
% GNSS/INS Integration Navigation Solution
pSkog    = zeros(nPts,2);
pSkog(1,:) = [Config.rpNorth0 Config.rpEast0];
vSkog    = zeros(nPts,1);
vSkog(1) = Config.rv0;
biasAccSkog = zeros(nPts,1);
biasAccSkog(1) = Config.rba0;
timeDelaySkog = zeros(nPts,1);
timeDelaySkog(1) = Config.rt0;
psiSkog = zeros(nPts,1);    
psiSkog(1) = Config.rpsi0;
% Error-State Vector
xSkog        = zeros(4,nPts);
% Covariance matrix
PSkog        = zeros(4,4,nPts);
PSkog(:,:,1) = [Config.sigmaInitNorthPos^2 0 0 0; ...
               0 Config.sigmaInitEastPos^2 0 0; ...
               0 0 Config.sigmaInitVel^2 0; ...
               0 0 0 Config.sigmaInitAccBias^2];

for k = 2:1:nPts
    % IMU measurements generation
    measAcc(k) = Config.a0 + Config.biasMeasAcc + normrnd(0,sqrt(Config.varMeasAcc));
    % GNSS measurements generation
    if mod(k,Config.M) == 0 && (k - Config.tDelay/Config.tIMU)>0
        pGNSS(k,:) = p(k - Config.tDelay/Config.tIMU,:) + normrnd(0,sqrt(Config.varMeasPosGNSS),[1 2]);
    end
    % Strapdown equations
    psiIMU(k) = psiIMU(k-1);
    vIMU(k) = vIMU(k-1) + measAcc(k)*Config.tIMU;
    pIMU(k,1) = pIMU(k-1,1) + vIMU(k)*cos(psiIMU(k))*Config.tIMU;
    pIMU(k,2) = pIMU(k-1,2) + vIMU(k)*sin(psiIMU(k))*Config.tIMU;
%     vIMU(k) = vIMU(k-1) + 0.5 * (measAcc(k) + measAcc(k-1)) * Config.tIMU;
%     pIMU(k,1) = pIMU(k-1,1) + 0.5 * (vIMU(k) + vIMU(k-1))*cos(psiIMU(k)) * Config.tIMU;
%     pIMU(k,2) = pIMU(k-1,2) + 0.5 * (vIMU(k) + vIMU(k-1))*sin(psiIMU(k)) * Config.tIMU;

    %EKF
    [xEKF(:,k), PEKF, vEKF, pEKF, psiEKF, biasAccEKF, measAccCorrEKF] = ...
                                  standardEKF(  xEKF(:,k-1),    ...
                                                PEKF,  ...
                                                pGNSS(k,:),       ...
                                                k,              ...
                                                measAcc,        ...
                                                measAccCorrEKF,    ...
                                                pEKF, ...    
                                                vEKF, ...
                                                psiEKF, ...
                                                biasAccEKF,...
                                                Config);
    
%     [xSkog(:,k), PSkog, vSkog, pSkog, biasAccSkog, timeDelaySkog, measAccCorrSkog] = skogEKF(               ...
%                                                     vSkog,           ...
%                                                     pSkog,           ...
%                                                     biasAccSkog,     ...
%                                                     timeDelaySkog,   ...
%                                                     xSkog(:,k-1),           ...
%                                                     PSkog,              ...
%                                                     pGNSS(k),           ...
%                                                     measAcc,            ...
%                                                     measAccCorrSkog,        ...
%                                                     k,                  ...
%                                                     tspan,              ...
%                                                     Config);                                                
    % TODO: Lee and Johnson method
    
     
end


end