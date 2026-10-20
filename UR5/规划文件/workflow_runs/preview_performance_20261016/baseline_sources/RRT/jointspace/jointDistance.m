function distance = jointDistance(q1,q2,weights)
%jointDistance Weighted Euclidean distance using shortest revolute deltas.

if nargin < 3 || isempty(weights), weights = ones(1,6); end
q1 = reshape(double(q1),1,[]);
q2 = reshape(double(q2),1,[]);
weights = reshape(double(weights),1,[]);
if numel(q1) ~= 6 || numel(q2) ~= 6 || numel(weights) ~= 6
    error('UR5 joint distance expects three 1-by-6 inputs.');
end
if any(~isfinite(weights)) || any(weights < 0)
    error('Joint distance weights must be finite and non-negative.');
end
dq = jointWrapDelta(q1,q2);
distance = sqrt(sum(weights.*(dq.^2)));
end
