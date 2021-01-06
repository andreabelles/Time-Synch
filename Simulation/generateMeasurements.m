function [measAcc, measGyro, pGNSS] = generateMeasurements(pTrue, Config, k, f_bi_b, w_bi_b, ned_GPS)

measAcc = f_bi_b(k,1);
measGyro = w_bi_b(k,3);

% IMU measurements generation
%measAcc = Config.a0 + normrnd(0,sqrt(Config.varMeasAcc));
%measGyro = Config.headingRate0 + normrnd(0,sqrt(Config.varMeasGyro));

% GNSS measurements generation
if mod(k,Config.M) == 0 && (k - Config.tDelay/Config.tIMU)>0
    %pGNSS = pTrue(k - Config.tDelay/Config.tIMU,:) + ...        % true position at t-delay
     %           normrnd(0,sqrt(Config.varMeasPosGNSS),[1 2]);   % Gaussian noise for 2D position
    pGNSS = ned_GPS(k/Config.M, 1:2);
else
    pGNSS = nan;
end

end