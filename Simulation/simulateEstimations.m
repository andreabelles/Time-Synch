function [pIMU, vIMU, pGNSS, xEKF, PEKF, xSkog, PSkog] = simulateEstimations( ...
                                                        p, tspan, M, tIMU, tDelay,  ...
                                                        p0, v0, a0,                 ...
                                                        rp0, rv0, rba0, rt0,        ...
                                                        sigmaGNSS, sigmaAcc,        ...
                                                        sigmaPos, sigmaVel, sigmaAccBias, sigmaTd)

%% Initializations

nPts    = length(tspan);        
measAcc = zeros(nPts,1);
measAccCorr = zeros(nPts,1);
vIMU    = zeros(nPts,1);    
vIMU(1) = v0;
pIMU    = zeros(nPts,1);    
pIMU(1) = p0;
pGNSS   = nan(nPts,1);

% EKF parameters and initializations
PEKF        = zeros(3,3,nPts);
PEKF(:,:,1) = [sigmaPos^2 0 0; 0 sigmaVel^2 0; 0 0 sigmaAccBias^2];

% Integration IMU/GNSS parameters and initializations
xEKF         = zeros(3,nPts);
xEKF(1, 1)   = rp0;
xEKF(2, 1)   = rv0;
xEKF(3, 1)   = rba0;

PSkog           = zeros(4,4,nPts);
PSkog(:,:,1)    = [sigmaPos^2 0 0 0; 0 sigmaVel^2 0 0; 0 0 sigmaAccBias^2 0; 0 0 0 sigmaTd^2];
xSkog        = zeros(4,nPts);
xSkog(1, 1)  = rp0;
xSkog(2, 1)  = rv0;
xSkog(3, 1)  = rba0;
xSkog(4, 1)  = rt0;

for k = 2:1:nPts
    % IMU measurements generation
    measAcc(k) = a0 + normrnd(0,sigmaAcc);
    % GNSS measurements generation
    if mod(k,M) == 0 && (k - tDelay/tIMU)>0
        pGNSS(k) = p(k - tDelay/tIMU) + normrnd(0,sigmaGNSS);
    end
    % Strapdown equations
    vIMU(k) = vIMU(k-1) + measAcc(k)*tIMU;
    pIMU(k) = pIMU(k-1) + vIMU(k)*tIMU;
        
    %EKF
    [xEKF(:,k), PEKF(:,:,k)] = standardEKF(  xEKF(:,k-1), ...
                                                PEKF(:,:,k-1),  ...
                                                pGNSS(k),       ...
                                                measAcc(k),     ...
                                                tIMU,           ...
                                                sigmaAcc,       ...
                                                sigmaGNSS);
    
%     [xSkog(:,k), PSkog(:,:,k), measAccCorr] = skogEKF(               ...
%                                                     xSkog,           ...
%                                                     PSkog,              ...
%                                                     pGNSS(k),           ...
%                                                     measAcc,            ...
%                                                     measAccCorr,        ...
%                                                     k,                  ...
%                                                     tspan,              ...
%                                                     tIMU,               ...                                                
%                                                     sigmaAcc,           ...
%                                                     sigmaGNSS);                                                
    % TODO: Lee and Johnson method
    
     
end


end