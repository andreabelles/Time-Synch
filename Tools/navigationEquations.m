function [estNow] = navigationEquations(estPrev, acc, dt)

global COL_EST_POS COL_EST_VEL COL_EST_ACCBIAS COL_EST_DELAY COL_EKF_MAX COL_SKOG_MAX

% Strapdown equations
estNow(COL_EST_VEL) = estPrev(COL_EST_VEL) + acc * dt;
estNow(COL_EST_POS) = estPrev(COL_EST_POS) + estNow(COL_EST_VEL) * dt;

if size(estPrev, 2) >= COL_EKF_MAX
    estNow(COL_EST_ACCBIAS) = estPrev(COL_EST_ACCBIAS);
end

if size(estPrev, 2) == COL_SKOG_MAX
    estNow(COL_EST_DELAY) = estPrev(COL_EST_DELAY);
end    

end