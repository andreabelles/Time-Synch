function initColumns()

% Trajectory
global COL_TRAJECTORY_TIME COL_TRAJECTORY_POS COL_TRAJECTORY_VEL COL_TRAJECTORY_ACC ...
        COL_TRAJECTORY_DELAY COL_TRAJECTORY_BIASACC COL_TRAJECTORY_MAX
% Estimations
global COL_EST_POS COL_EST_VEL COL_EST_ACCBIAS COL_EST_DELAY COL_IMU_MAX COL_EKF_MAX COL_SKOG_MAX

% Trajectory
COL_TRAJECTORY_TIME = 1;
COL_TRAJECTORY_POS  = 2;
COL_TRAJECTORY_VEL  = 3; 
COL_TRAJECTORY_ACC  = 4;
COL_TRAJECTORY_BIASACC  = 5;
COL_TRAJECTORY_DELAY  = 6;
COL_TRAJECTORY_MAX  = 6;

% Estimations
COL_EST_POS     = 1;
COL_EST_VEL     = 2;
COL_EST_ACCBIAS = 3;
COL_EST_DELAY   = 4;
COL_IMU_MAX     = 2;
COL_EKF_MAX     = 3;
COL_SKOG_MAX    = 4;

end