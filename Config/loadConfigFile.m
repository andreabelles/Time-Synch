function Config = loadConfigFile()


%% Load config file
configFile  = 'config.yaml';
Config      = ReadYaml(configFile);


%% Transformations
% tEnd            = Config.tEnd;           % s
Config.tIMU            = 1/Config.fIMU;         % s
Config.tGNSS           = 1/Config.fGNSS;        % s
% tDelay          = Config.tDelay;         % s
% 
Config.M               = Config.tGNSS/Config.tIMU;
% 
% varAccNoise        = Config.varAccNoise;       % m^2/s^3
% varAccBiasNoise    = Config.varAccBiasNoise;   % m^2/s^5
% varPosGNSS       = Config.sigmaGNSS;           % m^2
% 
% a0              = Config.a0;             % m/s^2
% v0              = Config.v0;             % m/s
% p0              = Config.p0;             % m
% 
% rp0             = Config.rp0;            % m
% rv0             = Config.rv0;            % m/s
% rba0            = Config.rba0;           % m/s^2
% rt0             = Config.rt0;            % s
% 
% sigmaInitPos        = Config.sigmaPos;       % m
% sigmaInitVel        = Config.sigmaVel;       % m/s
% sigmaInitAccBias    = Config.sigmaAccBias;   % m/s^2
% sigmaInitDelay      = Config.sigmaTd;     % s


end