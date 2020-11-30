function [rNew, PEst] = julierCU(rOld, pGNSS, measAcc, tIMU, POld, sigmaAcc, sigmaGNSS)
% julierCU: This function estimates the position and velocity using the 
%           method described by Julier and Uhlman 
%
% Inputs:   
%
% Outputs:  

F = [1 tIMU; 0 1];
Q = (1e-3) * [tIMU^3/3 tIMU^2/2; tIMU^2/2 tIMU];

rNew = F * rOld;

H = [1 0];
R = 1e-2;

K = (POld*H')/(H*POld*H' + R);
z = pGNSS - H*rNew;
xEst = K*z;
rInt = rInt + xEst;
PEst = POld - K*H*POld;





end