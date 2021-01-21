function [pNow, vNow, biasNow, tDelayNow] = navigationEquations(pPrev, vPrev, acc, dt, biasPrev, tDelayPrev)


% Strapdown equations
vNow = vPrev + acc * dt;
pNow = pPrev + vNow * dt;

if nargin > 4
    biasNow = biasPrev;
end

if nargin > 5
    tDelayNow = tDelayPrev;
end    

end