function [tEnd, tIMU, tGNSS, tDelay, M, sigmaAcc, sigmaGNSS, a0, v0, p0,    ...
    rp0, rv0, rba0, rt0, sigmaPos, sigmaVel, sigmaAccBias, sigmaTd]         ...
    = loadConfigFile()


%% Load config file
configFile  = 'config.yaml';
Config      = ReadYaml(configFile);


%% Transformations
tEnd            = Config.tEnd;           % s
tIMU            = 1/Config.fIMU;         % s
tGNSS           = 1/Config.fGNSS;        % s
tDelay          = Config.tDelay;         % s

M               = tGNSS/tIMU;

sigmaAcc        = Config.sigmaAcc;       % m/s^2
sigmaGNSS       = Config.sigmaGNSS;      % m

a0              = Config.a0;             % m/s^2
v0              = Config.v0;             % m/s
p0              = Config.p0;             % m

rp0             = Config.rp0;            % m
rv0             = Config.rv0;            % m/s
rba0            = Config.rba0;           % m/s^2
rt0             = Config.rt0;            % s

sigmaPos        = Config.sigmaPos;       % m
sigmaVel        = Config.sigmaVel;       % m/s
sigmaAccBias    = Config.sigmaAccBias;   % m/s^2
sigmaTd         = Config.sigmaTd;        % s


end