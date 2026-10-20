function dq = jointWrapDelta(q1,q2)
%jointWrapDelta Shortest revolute-joint displacement q1 -> q2 in radians.

q1 = reshape(double(q1),1,[]);
q2 = reshape(double(q2),1,[]);
if numel(q1) ~= numel(q2)
    error('q1 and q2 must have the same number of joints.');
end
if any(~isfinite([q1 q2]))
    error('Joint states must be finite.');
end
dq = atan2(sin(q2-q1),cos(q2-q1));
end
