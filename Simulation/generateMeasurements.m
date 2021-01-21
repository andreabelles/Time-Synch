function [measAcc, pGNSS] = generateMeasurements(a, p, k, Config)

    % IMU measurements generation
    measAcc = a(k) + Config.biasMeasAcc + normrnd(0,sqrt(Config.varMeasAcc));
    % GNSS measurements generation
    if mod(k,Config.M) == 0 && (k - Config.tDelay/Config.tIMU)>0
        pGNSS = p(k - Config.tDelay/Config.tIMU) + normrnd(0,sqrt(Config.varMeasPosGNSS));
    else
        pGNSS = nan;
    end
end