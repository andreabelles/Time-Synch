function [estIMU] = navigationEquations(measGyro, measAcc, estIMU, k, tIMU)

estIMU.headingRate(k) = measGyro;                                                                         % Heading Rate
estIMU.heading(k) = estIMU.heading(k-1) + 0.5 * (estIMU.headingRate(k) + estIMU.headingRate(k-1)) * tIMU;                      % Heanding angle
estIMU.acc(k) = measAcc;                                                                          % Acceleration
estIMU.vel(k) = estIMU.vel(k-1) + 0.5 * (estIMU.acc(k) + estIMU.acc(k-1)) * tIMU;                            % Velocity
estIMU.pos(k,1) = estIMU.pos(k-1,1) + 0.5 * (estIMU.vel(k) + estIMU.vel(k-1)) * cosd(estIMU.heading(k)) * tIMU;     % North position
estIMU.pos(k,2) = estIMU.pos(k-1,2) + 0.5 * (estIMU.vel(k) + estIMU.vel(k-1)) * sind(estIMU.heading(k)) * tIMU;     % East position
 
end