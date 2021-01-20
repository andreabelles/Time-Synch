function [tspan, p, v, a] = generateTrajectory(Config)

y0      = [Config.p0; Config.v0; Config.a0];
tspan   = 0:Config.tIMU:Config.tEnd;
dydt    = @(t, y) [y(2); y(3); Config.aAmp*2*pi*Config.aFreq*cos(2*pi*Config.aFreq*t)];
[~, y]  = ode45(dydt, tspan, y0);
p       = y(:, 1);
v       = y(:, 2);
a       = y(:, 3);

end