function [trajectory] = generateTrajectory(Config)

global COL_TRAJECTORY_TIME COL_TRAJECTORY_POS COL_TRAJECTORY_VEL COL_TRAJECTORY_ACC COL_TRAJECTORY_MAX

% Initializations
nPts = Config.tEnd/Config.tIMU + 1; % from 0 to tEnd
trajectory = nan(nPts, COL_TRAJECTORY_MAX);

% Simulation time vector
trajectory(:, COL_TRAJECTORY_TIME) = 0:Config.tIMU:Config.tEnd;

% Initial values
y0      = [Config.p0; Config.v0; Config.a0]; 
% Differential equation for the trajectory
dydt    = @(t, y) [y(2); y(3); Config.aAmp*2*pi*Config.aFreq*cos(2*pi*Config.aFreq*t)];
% Generation of the trajectory
[~, y]  = ode45(dydt, trajectory(:, COL_TRAJECTORY_TIME), y0);

trajectory(:, COL_TRAJECTORY_POS)   = y(:, 1);
trajectory(:, COL_TRAJECTORY_VEL)   = y(:, 2);
trajectory(:, COL_TRAJECTORY_ACC)   = y(:, 3);

end