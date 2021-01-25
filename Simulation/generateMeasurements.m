function [measAcc, pGNSS] = generateMeasurements(trueTrajectory, k, Config)
%FUNCTION_NAME - One line description of what the function or script performs (H1 line)
%Optional file header info (to give more details about the function than in the H1 line)
%
% Syntax:  [output1,output2] = function_name(input1,input2,input3)
%
% Inputs:
%    input1 - Description
%    input2 - Description
%    input3 - Description
%
% Outputs:
%    output1 - Description
%    output2 - Description
%
% Example: 
%    Line 1 of example
%    Line 2 of example
%    Line 3 of example
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% Author: Andrea Belles Ferreres, Arnau Ochoa Bañuelos 
%------------- BEGIN CODE --------------

global COL_TRAJECTORY_TIME COL_TRAJECTORY_POS COL_TRAJECTORY_ACC COL_TRAJECTORY_BIASACC COL_TRAJECTORY_DELAY

%% IMU measurements generation
measAcc = trueTrajectory(k, COL_TRAJECTORY_ACC) + trueTrajectory(k, COL_TRAJECTORY_BIASACC) + normrnd(0,sqrt(Config.varMeasAcc));

%% GNSS measurements generation
% Get current delay
tDelay = trueTrajectory(k, COL_TRAJECTORY_DELAY);
% If there is gnss measurement at time k
if mod(k,Config.M) == 0 && (k - tDelay/Config.tIMU)>0
    % If no true position at delay, interpolate
    if mod(tDelay,Config.tIMU) == 0
        kAtDelay = k - tDelay/Config.tIMU;
        pAtDelay = trueTrajectory(kAtDelay, COL_TRAJECTORY_POS);
    else
        pAtDelay = interp1( trueTrajectory(:, COL_TRAJECTORY_TIME), ...             % X axis
                            trueTrajectory(:, COL_TRAJECTORY_POS), ...              % Y axis
                            trueTrajectory(k, COL_TRAJECTORY_TIME) - tDelay);% Time at delay
    end
    pGNSS = pAtDelay + normrnd(0,sqrt(Config.varMeasPosGNSS));
else
    pGNSS = nan;
end

%------------- END OF CODE --------------
end