function prevIndex = findPrevIndex(v, x)
% Find index of the largest value in v smaller than x. Assuming v is
% sorted.

% Determine index of value right before x in vector v
[~, closestIndex] = min(abs(x - v)); % Index of v closest to x
% Condition to keep always the previous sample to x
if v(closestIndex) > x, prevIndex = closestIndex - 1;
else, prevIndex = closestIndex; end



end