function v = lagrangeInterp(t, y, D)

%% Find index in t closest to delay D
[~, closestIndex] = min(abs(D - t.'));

%% Window size computation
dEnd    = length(t) - closestIndex; % Distance from delay to last time
dBegin  = closestIndex - 1; % Distance from delay to first time

% Deal with extreme values
dEnd    = max(dEnd, 1); 
dBegin  = max(dBegin, 1);

N       = 2 * min(dEnd, dBegin); % Time window - 1

% if N > maxN, N = maxN; end

% Indexes of y inside window
kWindow = max(closestIndex-(N/2), 1) : min(closestIndex+(N/2), length(t)); 

%% Weights of the FIR
h   = zeros(1, length(kWindow));
% Adding an appropiate number of unit delays before the fractional delay FIR filter
n   = t(kWindow);
for i = 1:length(kWindow)
    k = n;
    k(i) = [];
    h(i) = prod( (D-k) ./ (n(i)-k) );
end
% Flip h so h(0) multiplies last y
% h = flip(h);

h(isinf(h)) = 0;

%% Interpolation
v = sum(dot(y(kWindow), h));

% figure; 
% plot(n, h, 'o-');
% title('Weights');
% xline(D)
% 
% figure;
% plot(t, y, 'o-'); hold on;
% plot(D, v, 'x');
% xline(D)

end