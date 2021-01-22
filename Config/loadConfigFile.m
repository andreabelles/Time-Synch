function Config = loadConfigFile()


%% Load config file
configFile  = 'config.yaml';
Config      = ReadYaml(configFile);


%% Transformations
Config.tIMU            = 1/Config.fIMU;         % s
Config.tGNSS           = 1/Config.fGNSS;        % s

Config.M               = Config.tGNSS/Config.tIMU; % Rate between GNSS and IMU

end