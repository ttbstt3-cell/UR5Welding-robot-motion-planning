function frame = computeGaussianFrame(startPoint,goalPoint)
% computeGaussianFrame  Build the local frame used by GD-RRT* sampling.
%
% The local origin is startPoint.  Local y points from start to goal and
% local x is the perpendicular direction [ey(2), -ey(1)].

startPoint = reshape(startPoint,1,[]);
goalPoint = reshape(goalPoint,1,[]);
if numel(startPoint) ~= 2 || numel(goalPoint) ~= 2 || ...
        any(~isfinite([startPoint goalPoint]))
    error('startPoint and goalPoint must be finite 1-by-2 points.');
end

direction = goalPoint-startPoint;
distance = norm(direction);
if distance <= eps
    error('startPoint and goalPoint must be distinct.');
end

ey = direction/distance;
ex = [ey(2),-ey(1)];

frame.origin = startPoint(:);
frame.goal = goalPoint(:);
frame.ex = ex(:);
frame.ey = ey(:);
frame.R = [frame.ex,frame.ey];
frame.distance = distance;

end
