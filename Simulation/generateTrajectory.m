function [trajectory] = generateTrajectory(Config)

global COL_TRAJECTORY_TIME COL_TRAJECTORY_POS COL_TRAJECTORY_VEL COL_TRAJECTORY_ACC ...
        COL_TRAJECTORY_BIASACC COL_TRAJECTORY_DELAY COL_TRAJECTORY_MAX

% Initializations
nPts = Config.tEnd/Config.tIMU + 1; % from 0 to tEnd
trajectory = nan(nPts, COL_TRAJECTORY_MAX);

% Simulation time vector
trajectory(:, COL_TRAJECTORY_TIME) = 0:Config.tIMU:Config.tEnd;

% Evolution of the accelerometer bias
trajectory(:, COL_TRAJECTORY_BIASACC) = Config.biasMeasAcc;
% Evolution of the measurement delay
if Config.delayJumpAmp == 0
    trajectory(:, COL_TRAJECTORY_DELAY) = Config.tDelay;
elseif Config.delayJumpTime > 0 && Config.delayJumpTime < Config.tEnd
    kJump = ceil(Config.delayJumpTime/Config.tIMU);
    trajectory(1:kJump, COL_TRAJECTORY_DELAY) = Config.tDelay;
    trajectory(kJump+1:nPts, COL_TRAJECTORY_DELAY) = Config.tDelay + Config.delayJumpAmp;
else
    error('Invalid Config.delayJumpTime');
end

% Initial values
y0      = [Config.p0; Config.v0; Config.a0]; 

% Differential equations for the trajectory
switch Config.accFunction
    case 'constant'
        dydt    = @(t, y) [y(2); y(3); 0]; % d/dt pos, d/dt vel, d/dt acc
    case 'logistic'
        dydt    = @(t, y) [y(2); y(3); ...
                                    (Config.aLogSteep * Config.aAmp *               ...
                                    exp(Config.aLogSteep*(Config.aLogT0-t))) /      ...
                                    (exp(Config.aLogSteep*(Config.aLogT0-t)) + 1)^2];
    case 'sine'
        dydt    = @(t, y) [y(2); y(3); Config.aAmp*2*pi*Config.aFreq*cos(2*pi*Config.aFreq*t)];
    otherwise
        error('Invalid accFunction');
end

% Generation of the trajectory
[~, y]  = ode45(dydt, trajectory(:, COL_TRAJECTORY_TIME), y0);

trajectory(:, COL_TRAJECTORY_POS)   = y(:, 1);
trajectory(:, COL_TRAJECTORY_VEL)   = y(:, 2);
trajectory(:, COL_TRAJECTORY_ACC)   = y(:, 3);

end