function [tspan, trueTrajectory] = generateTrajectory(Config)

diffEq0          = [Config.pNorth0; Config.pEast0; Config.v0; Config.a0; Config.heading0; Config.headingRate0];
tspan           = 0:Config.tIMU:Config.tEnd;
dEqdt       = @(t, diffEq) [diffEq(3)*cosd(diffEq(5));  ... % dpNorth/dt = v*cos(heading)
                            diffEq(3)*sind(diffEq(5));  ... % dpEast/dt = v*sin(heading)
                            diffEq(4);                  ... % dv/dt = a
                            0;                          ... % da/dt = 0
                            diffEq(6);                  ... % dHeading/dt = w
                            0];                             % dw/dt = 0
[~, trueTrajectory] = ode45(dEqdt, tspan, diffEq0);

end