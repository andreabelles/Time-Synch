clear; close all; clc;
addpath(genpath('./'));
% Time Synchronization GNSS-aided IMU project
% (Description of the project here)

%% Definition of variables
tEnd    = 60e3; % ms
tIMU    = 10;   % ms
tGNSS   = 5000;  % ms
tDelay  = 1000;  % ms
M       = tGNSS/tIMU;
sigmaAcc = 0.2; % m/s^2
sigmaGNSS = 3/sqrt(3); % m
sigmaV = 0.1; %m/s
sigmaTd = 13/1200; %s
p0 = 0; % m
v0 = 1; %m/s
a0 = 0;

% Conversion from s to ms
sigmaAcc = sigmaAcc/1e6;
sigmaV = sigmaV/1e3;
sigmaTd = sigmaTd/1e3;
v0 = v0/1e3;
a0 = a0/1e6;

%% Generation of true trajectory
p = zeros(tEnd,1);
v = zeros(tEnd,1);
p(1) = p0;
v(1) = v0;
for t = 2:1:tEnd
    p(t) = p(t-1) + v(t-1);
    v(t) = v(t-1);
end

%% Generation of measurements and EKF
nPts    = tEnd/tIMU;
measAcc = zeros(nPts,1);
vIMU    = zeros(nPts,1);
pIMU    = zeros(nPts,1);
vIMU(1) = v0;
pIMU(1) = p0;
pGNSS   = nan(nPts,1);

% EKF parameters and initializations
xEKF        = zeros(2,nPts); % Pos, Vel
xEKF(:, 1)  = [0; 0];
PEKF        = zeros(2,2,nPts);
PEKF(:,:,1) = [sigmaGNSS^2 0; 0 sigmaV^2];

% Integration IMU/GNSS parameters and initializations
vIntEKF         = zeros(1,nPts);
vIntEKF(1)   = vIMU(1);
pIntEKF         = zeros(1,nPts);
pIntEKF(1)   = pIMU(1);

xSkog   = zeros(3,nPts);
xSkog(:, 1) = [0; 0; 0];
PSkog   = zeros(3,3,nPts);
PSkog(:,:,1) = [sigmaGNSS^2 0 0; 0 sigmaV^2 0; 0 0 sigmaTd^2];
% xLee   = zeros(nPts,1);
% xLee(1) = [p0; v0];
% xJulier   = zeros(nPts,1);
% xJulier(1) = [p0; v0];

for k = 2:1:nPts
    % IMU measurements generation
    measAcc(k) = a0 + normrnd(0,sigmaAcc);
    % GNSS measurements generation
    if mod(k,M) == 0
        pGNSS(k) = p(k*tIMU - tDelay) + normrnd(0,sigmaGNSS);
    end
    % Strapdown equations
    vIMU(k) = vIMU(k-1) + measAcc(k)*tIMU;
    pIMU(k) = pIMU(k-1) + vIMU(k)*tIMU;
        
    %EKF
    [pIntEKF(:,k), vIntEKF(:,k), PEKF(:,:,k)] = standardEKF(pIntEKF(:,k-1), ...
                                                            vIntEKF(:,k-1), ...
                                                            pGNSS(k),       ...
                                                            measAcc(k),     ...
                                                            tIMU,           ...
                                                            xEKF(:,k-1),    ...
                                                            PEKF(:,:,k-1),  ...
                                                            sigmaAcc,       ...
                                                            sigmaGNSS);
    
    %[xSkog(:,k), PSkog(:,:,k)] = skogEKF(pIMU(k), pGNSS(k), tIMU, xSkog(:,k-1), PSkog(:,:,k-1), sigmaAcc, sigmaGNSS, measAcc);
    %[xLee(k)] = leeEKF(pGNSS(k), tIMU, xLee(k-1), sigmaAcc, sigmaGNSS);
    %[xJulier(k)] = julierCU(pGNSS(k), tIMU, xJulier(k-1), sigmaAcc, sigmaGNSS);
end

%% Results
% Computation of errors
tVec = 1:tEnd;
kVec = 1:tIMU:tEnd;
errPosIMU = p(kVec)-pIMU;
errVelIMU = v(kVec)-vIMU;
errPosGNSS = p(kVec)-pGNSS;
errPosEKF = p(kVec) - pIntEKF';
errVelEKF = v(kVec) - vIntEKF';
%% Plots
% True trajectory vs measurements
figure;
plot(tVec./1e3, p, 'k-', 'Linewidth', 1); hold on;
plot(kVec./1e3, pIMU, 'b.'); 
plot(kVec./1e3, pGNSS, 'r.');
xlabel('Time (s)'); ylabel('Position (m)')
title('True trajectory');
legend('True', 'IMU', 'GNSS');

% IMU Position error plot
figure
plot(kVec/1e3, errPosIMU, 'b-'); hold on
plot(kVec/1e3, errPosGNSS, 'r.');
xlabel('Time (s)'); ylabel('Position error (m)')
legend('IMU', 'GNSS');
title('IMU-only & GNSS-only Position error');

% IMU Velocity error plot
figure
plot(kVec/1e3, errVelIMU*1e3, 'b-'); hold on
xlabel('Time (s)'); ylabel('Velocity error (m)')
title('IMU-only Velocity error');

% True trajectory vs estimated trajectory
figure;
plot(tVec./1e3, p, 'k-', 'Linewidth', 1); hold on;
plot(kVec./1e3, pIntEKF, 'b-'); 
xlabel('Time (s)'); ylabel('Position (m)')
title('Standard EKF method');
legend('True', 'Estimated');

% True Velocity vs estimated velocity
figure;
plot(tVec./1e3, v*1e3, 'k-', 'Linewidth', 1); hold on;
plot(kVec./1e3, vIntEKF*1e3, 'b-'); 
xlabel('Time (s)'); ylabel('Velocity (m/s)')
title('Standard EKF method');
legend('True', 'Estimated');

% Estimation Error plot
figure
plot(kVec./1e3, errPosEKF, 'b-'); 
xlabel('Time (s)'); ylabel('Position error (m)')
title('Standard EKF method');

figure
plot(kVec./1e3, errVelEKF*1e3, 'b-'); 
xlabel('Time (s)'); ylabel('Velocity error (m/s)')
title('Standard EKF method');


% figure;
% plot(kVec./1e3, tDelay*ones(length(kVec)), 'k-', 'Linewidth', 1); hold on;
% plot(kVec./1e3, xSkog(3,:), 'b.'); 
% xlabel('Time (s)'); ylabel('Position (m)')
% title('Time Delay Evolution');