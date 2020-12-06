function [tspan, p, v, a] = generateTrajectory(p0, v0, a0, tIMU, tEnd)

y0      = [p0; v0; a0];
tspan   = 0:tIMU:tEnd;
dydt    = @(t, y) [y(2); y(3); 0];
[~, y]  = ode45(dydt, tspan, y0);
p       = y(:, 1);
v       = y(:, 2);
a       = y(:, 3);

end