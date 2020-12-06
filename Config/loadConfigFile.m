function [tEnd, tIMU, tGNSS, tDelay, M, sigmaAcc, sigmaGNSS, a0, v0, p0,    ...
    rp0, rv0, rba0, rt0, sigmaPos, sigmaVel, sigmaAccBias, sigmaTd]         ...
    = loadConfigFile()


%% Load config file
configFile  = 'config.yaml';
Config      = ReadYaml(configFile);


%% Transformations
tEnd            = Config.tEnd           * 1e3;      % s to ms
tIMU            = 1/Config.fIMU         * 1e3;      % s to ms
tGNSS           = 1/Config.fGNSS        * 1e3;      % s to ms
tDelay          = Config.tDelay         * 1e3;      % s to ms

M               = tGNSS/tIMU;

sigmaAcc        = Config.sigmaAcc       / 1e6;      % m/s^2 to m/ms^2
sigmaGNSS       = Config.sigmaGNSS;                 % m

a0              = Config.a0             / 1e6;      % m/s^2 to m/ms^2
v0              = Config.v0             / 1e3;      % m/s to m/ms
p0              = Config.p0;                        % m

rp0             = Config.rp0;                       % m
rv0             = Config.rv0            / 1e3;      % m/s to m/ms
rba0            = Config.rba0           / 1e6;      % m/s^2 to m/ms^2
rt0             = Config.rt0            * 1e3;      % s to ms

sigmaPos        = Config.sigmaPos;                  % m
sigmaVel        = Config.sigmaVel       / 1e3;      % m/s to m/ms
sigmaAccBias    = Config.sigmaAccBias   / 1e6;      % m/s^2 to m/ms^2
sigmaTd         = Config.sigmaTd        * 1e3;      % s to ms


end