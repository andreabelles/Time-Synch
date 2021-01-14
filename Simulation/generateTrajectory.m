function [tspan, p, v, a, psi] = generateTrajectory(Config)

y0      = [Config.pNorth0; Config.pEast0; Config.v0; Config.a0; Config.psi0];
tspan   = 0:Config.tIMU:Config.tEnd;
dydt    = @(t, y) [y(3)*cos(y(5)); y(3)*sin(y(5)); y(4); 0; 0];
[~, y]  = ode45(dydt, tspan, y0);
p       = y(:, 1:2);
v       = y(:, 3);
a       = y(:, 4);
psi     = y(:, 5);

end