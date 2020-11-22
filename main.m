clear; %close all; clc;
% Time Synchronization GNSS-aided IMU project
% (Description of the project here)

%% Definition of variables
tEnd    = 120e3; % ms
tIMU    = 10;   % ms
tGNSS   = 10000;  % ms
tDelay  = 100;  % ms
M       = tGNSS/tIMU;
sigmaAcc = 400; % m/s^2
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
pIMU    = zeros(nPts,1);
pGNSS   = nan(nPts,1);
pIMU(1) = p0;
measAcc = zeros(nPts,1);

% EKF parameters and initializations
rEKF   = zeros(3,nPts);
rEKF(:, 1) = [0; 0; 0]; % Pos, Vel, bAcc
PEKF   = zeros(3,3,nPts);
PEKF(:,:,1) = [sigmaGNSS^2 0 0; 0 sigmaV^2 0; 0 0 sigmaAcc^2];

xSkog   = zeros(3,nPts);
xSkog(:, 1) = [0; 0; 0];
PSkog   = zeros(3,3,nPts);
PSkog(:,:,1) = [sigmaGNSS^2 0 0; 0 sigmaV^2 0; 0 0 sigmaTd^2];
% xLee   = zeros(nPts,1);
% xLee(1) = [p0; v0];
% xJulier   = zeros(nPts,1);
% xJulier(1) = [p0; v0];

for k = 2:1:nPts
%     pIMU(k) = p(k*tIMU) + normrnd(0,sigmaIMU);
    measAcc(k) = a0 + normrnd(0,sigmaAcc);
    pIMU(k) = pIMU(k-1) + v(k)*tIMU + (measAcc(k)*tIMU^2);
    if mod(k,M) == 0
        pGNSS(k) = p(k*tIMU - tDelay) + normrnd(0,sigmaGNSS);
    end    
    %EKF
    [rEKF(:,k), PEKF(:,:,k)] = standardEKF(pGNSS(k), tIMU, rEKF(:,k-1), PEKF(:,:,k-1), sigmaAcc, sigmaGNSS, measAcc);
    [xSkog(:,k), PSkog(:,:,k)] = skogEKF(pIMU(k), pGNSS(k), tIMU, xSkog(:,k-1), PSkog(:,:,k-1), sigmaAcc, sigmaGNSS, measAcc);
    %[xLee(k)] = leeEKF(pGNSS(k), tIMU, xLee(k-1), sigmaAcc, sigmaGNSS);
    %[xJulier(k)] = julierCU(pGNSS(k), tIMU, xJulier(k-1), sigmaAcc, sigmaGNSS);
end

%% Results
% Computation of errors
tVec = 1:tEnd;
kVec = 1:tIMU:tEnd;
errorIMU = p(kVec)-pIMU;
errorGNSS = p(kVec)-pGNSS;
errorEKF = [p(kVec), v(kVec)]' - rEKF;

%% Plots
% True trajectory vs measurements
figure;
plot(tVec./1e3, p, 'k-', 'Linewidth', 1); hold on;
plot(kVec./1e3, pIMU, 'b.'); 
plot(kVec./1e3, pGNSS, 'r.');
xlabel('Time (s)'); ylabel('Position (m)')
title('True trajectory');
legend('True', 'IMU', 'GNSS');

% Error plot
figure
plot(kVec, errorIMU, 'b-'); hold on
plot(kVec, errorGNSS, 'r.');
xlabel('Samples'); ylabel('Position error (m)')
legend('IMU', 'GNSS');

% True trajectory vs estimated trajectory
figure;
plot(tVec./1e3, p, 'k-', 'Linewidth', 1); hold on;
plot(kVec./1e3, rEKF(1,:), 'b-'); 
xlabel('Time (s)'); ylabel('Position (m)')
title('Standard EKF method');
legend('True', 'Estimated');

% Estimation Error plot
figure
plot(kVec./1e3, errorEKF(1, :), 'b-'); 
xlabel('Samples'); ylabel('Position error (m)')
legend('IMU');
figure
plot(kVec./1e3, errorEKF(2, :)*1e3, 'b-'); 
xlabel('Samples'); ylabel('Velocity error (m/s)')
legend('IMU');

% figure;
% plot(kVec./1e3, tDelay*ones(length(kVec)), 'k-', 'Linewidth', 1); hold on;
% plot(kVec./1e3, xSkog(3,:), 'b.'); 
% xlabel('Time (s)'); ylabel('Position (m)')
% title('Time Delay Evolution');