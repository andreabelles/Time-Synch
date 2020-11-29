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
sigmaBiasAcc = (0.1^2)/12;
p0 = 0; % m
v0 = 1; % m/s
a0 = 0; % m/s^2

% Conversion from s to ms
sigmaAcc = sigmaAcc/1e6;
sigmaV = sigmaV/1e3;
sigmaTd = sigmaTd/1e3;
sigmaBiasAcc = sigmaBiasAcc/1e6;
v0 = v0/1e3;
a0 = a0/1e6;

%% Generation of true trajectory
y0      = [p0; v0];
tspan   = 0:tIMU:tEnd;
dydt    = @(t, y) [y(2); a0];
[~, y]  = ode45(dydt,tspan,y0);
p       = y(:, 1);
v       = y(:, 2);

% p = zeros(tEnd,1);
% v = zeros(tEnd,1);
% p(1) = p0;
% v(1) = v0;
% for t = 2:1:tEnd
%     p(t) = p(t-1) + v(t-1);
%     v(t) = v(t-1);
% end

%% Generation of measurements and EKF
nPts    = length(tspan);
measAcc = zeros(nPts,1);
measAccCorr = zeros(nPts,1);
vIMU    = zeros(nPts,1);
pIMU    = zeros(nPts,1);
vIMU(1) = v0;
pIMU(1) = p0;
pGNSS   = nan(nPts,1);

% EKF parameters and initializations
xEKF        = zeros(3,nPts); % Pos, Vel
PEKF        = zeros(3,3,nPts);
PEKF(:,:,1) = [sigmaGNSS^2 0 0; 0 sigmaV^2 0; 0 0 sigmaBiasAcc^2];

% Integration IMU/GNSS parameters and initializations
rIntEKF         = zeros(3,nPts);
rIntEKF(2, 1)   = vIMU(1);
rIntEKF(1, 1)   = pIMU(1);

xSkog           = zeros(4,nPts);
PSkog           = zeros(4,4,nPts);
PSkog(:,:,1)    = [sigmaGNSS^2 0 0 0; 0 sigmaV^2 0 0; 0 0 sigmaBiasAcc^2 0; 0 0 0 sigmaTd^2];
rIntSkog        = zeros(4,nPts);
rIntSkog(2, 1)  = vIMU(1);
rIntSkog(1, 1)  = pIMU(1);
% xLee   = zeros(nPts,1);
% xLee(1) = [p0; v0];
% xJulier   = zeros(nPts,1);
% xJulier(1) = [p0; v0];

for k = 2:1:nPts
    % IMU measurements generation
    measAcc(k) = a0 + normrnd(0,sigmaAcc);
    % GNSS measurements generation
    if mod(k,M) == 0
        pGNSS(k) = p(k - tDelay/tIMU) + normrnd(0,sigmaGNSS);
    end
    % Strapdown equations
    vIMU(k) = vIMU(k-1) + measAcc(k)*tIMU;
    pIMU(k) = pIMU(k-1) + vIMU(k)*tIMU;
        
    %EKF
    [rIntEKF(:,k), PEKF(:,:,k)] = standardEKF(  rIntEKF(:,k-1), ...
                                                pGNSS(k),       ...
                                                measAcc(k),     ...
                                                tIMU,           ...
                                                PEKF(:,:,k-1),  ...
                                                sigmaAcc,       ...
                                                sigmaGNSS,      ...
                                                sigmaBiasAcc);
    
    tNow = 0:tIMU:(k-1)*tIMU;
    [rIntSkog(:,k), PSkog(:,:,k), ] = skogEKF(  rIntSkog(:,k-1),    ...
                                                pGNSS(k),           ...
                                                measAcc,            ...
                                                measAccCorr,        ...
                                                k,                  ...
                                                tNow,               ...
                                                tIMU,               ...
                                                PSkog(:,:,k-1),      ...
                                                sigmaAcc,           ...
                                                sigmaGNSS);    
    % TODO: Julier and Uhlman method                                            
    % TODO: Lee and Johnson method
end

%% Results
tVec        = 0:tIMU:tEnd;
pIntEKF     = rIntEKF(1, :);
vIntEKF     = rIntEKF(2, :);
bIntEKF     = rIntEKF(3, :);
pIntSkog    = rIntSkog(1, :);
vIntSkog    = rIntSkog(2, :);
bIntSkog    = rIntSkog(3, :);
% Computation of errors
errPosIMU   = abs(p - pIMU);
errVelIMU   = v - vIMU;
errPosGNSS  = abs(p - pGNSS);
errPosEKF   = abs(p - pIntEKF');
errVelEKF   = v - vIntEKF';
errPosSkog  = abs(p - pIntSkog');
errVelSkog  = v - vIntSkog';
%% Plots
% True trajectory vs measurements
figure;
plot(tVec./1e3, p, 'k-', 'Linewidth', 1); hold on;
plot(tVec./1e3, pIMU, 'b.'); 
plot(tVec./1e3, pGNSS, 'r.');
xlabel('Time (s)'); ylabel('Position (m)')
title('True trajectory');
legend('True', 'IMU', 'GNSS');

% IMU Position error plot
figure
plot(tVec/1e3, errPosIMU, 'b-'); hold on
plot(tVec/1e3, errPosGNSS, 'r.');
xlabel('Time (s)'); ylabel('Position error (m)')
legend('IMU', 'GNSS');
title('IMU-only & GNSS-only Position error');

% IMU Velocity error plot
figure
plot(tVec/1e3, errVelIMU*1e3, 'b-'); hold on
xlabel('Time (s)'); ylabel('Velocity error (m)')
title('IMU-only Velocity error');

% True trajectory vs estimated trajectory
figure;
plot(tVec./1e3, p, 'k-', 'Linewidth', 1); hold on;
plot(tVec./1e3, pIntEKF, 'b-'); 
plot(tVec./1e3, pIntSkog, 'r-'); 
xlabel('Time (s)'); ylabel('Position (m)')
title('True position vs estimations');
legend('True', 'EKF', 'Skog');

% True Velocity vs estimated velocity
figure;
plot(tVec./1e3, v*1e3, 'k-', 'Linewidth', 1); hold on;
plot(tVec./1e3, vIntEKF*1e3, 'b-'); 
plot(tVec./1e3, vIntSkog*1e3, 'r-'); 
xlabel('Time (s)'); ylabel('Velocity (m/s)')
title('True velocity vs estimations');
legend('True', 'EKF', 'Skog');

figure;
plot(tVec./1e3, bIntEKF*1e6, 'b-'); hold on;
plot(tVec./1e3, bIntSkog*1e6, 'r-'); 
xlabel('Time (s)'); ylabel('Acceleration bias (m/s^2)')
title('Standard EKF method');
legend('EKF', 'Skog');

% Estimation Error plot
figure
plot(tVec./1e3, errPosEKF, 'b-'); hold on;
plot(tVec./1e3, errPosSkog, 'r-'); 
xlabel('Time (s)'); ylabel('Position error (m)')
title('Error in position estimations');
legend('EKF', 'Skog');

figure
plot(tVec./1e3, errVelEKF*1e3, 'b-'); hold on;
plot(tVec./1e3, errVelSkog*1e3, 'r-');
xlabel('Time (s)'); ylabel('Velocity error (m/s)')
title('Error in velocity estimations');
legend('EKF', 'Skog');

% figure; plot(tVec./1e3, sqrt(permute(abs(PEKF(1, 1, :)), [3 1 2])));
% xlabel('Time (s)'); ylabel('Position std (m/s)');
% title('Standard deviation of the position estimation');
% 
% figure; plot(tVec./1e3, sqrt(permute(abs(PEKF(2, 2, :)), [3 1 2])));
% xlabel('Time (s)'); ylabel('Velocity std (m/s)');
% title('Standard deviation of the velocity estimation');
% 
% figure; plot(tVec./1e3, sqrt(permute(abs(PEKF(3, 3, :)), [3 1 2])));
% xlabel('Time (s)'); ylabel('Bias std (m/s)');
% title('Standard deviation of the bias estimation');

% figure;
% plot(kVec./1e3, tDelay*ones(length(kVec)), 'k-', 'Linewidth', 1); hold on;
% plot(kVec./1e3, xSkog(3,:), 'b.'); 
% xlabel('Time (s)'); ylabel('Position (m)')
% title('Time Delay Evolution');