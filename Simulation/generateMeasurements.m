function [measAcc, pGNSS] = generateMeasurements(a, p, k, tspan, Config)
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

%% IMU measurements generation
measAcc = a(k) + Config.biasMeasAcc + normrnd(0,sqrt(Config.varMeasAcc));

%% GNSS measurements generation
% If there is gnss measurement at time k
if mod(k,Config.M) == 0 && (k - Config.tDelay/Config.tIMU)>0
    % If no true position at delay, interpolate
    if mod(Config.tDelay,Config.tIMU) == 0
        pAtDelay = p(k - Config.tDelay/Config.tIMU);
    else
        pAtDelay = interp1(tspan, p, tspan(k) - Config.tDelay);
    end
    pGNSS = pAtDelay + normrnd(0,sqrt(Config.varMeasPosGNSS));
else
    pGNSS = nan;
end

%------------- END OF CODE --------------
end