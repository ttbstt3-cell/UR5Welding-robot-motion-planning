function inside = jointWithinLimits(q,limits,tolerance)
%jointWithinLimits Check a 1-by-6 state without clamping it.

if nargin < 3 || isempty(tolerance), tolerance = 1e-10; end
q = reshape(double(q),1,[]);
limits = double(limits);
if ~isequal(size(limits),[6 2]) || numel(q) ~= 6
    error('q must be 1-by-6 and limits must be 6-by-2.');
end
inside = all(isfinite(q)) && all(q >= limits(:,1).'-tolerance) && ...
    all(q <= limits(:,2).'+tolerance);
end
