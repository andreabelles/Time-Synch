function [measAcc, measGyro, pGNSS] = generateMeasurements(pTrue, Config, k)

% IMU measurements generation
measAcc = Config.a0 + Config.biasMeasAcc + normrnd(0,sqrt(Config.varMeasAcc));
measGyro = Config.headingRate0 + normrnd(0,sqrt(Config.varMeasGyro));

% GNSS measurements generation
if mod(k,Config.M) == 0 && (k - Config.tDelay/Config.tIMU)>0
    pGNSS = pTrue(k - Config.tDelay/Config.tIMU,:) + ...        % true position at t-delay
                normrnd(0,sqrt(Config.varMeasPosGNSS),[1 2]);   % Gaussian noise for 2D position
else
    pGNSS = nan;
end

end