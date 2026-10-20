function [distance,nearestPoint,repulsiveDirection] = ...
    distanceToRectangleObstacle(point,rectangle,clearance)
% distanceToRectangleObstacle  Distance from a point to an expanded rectangle.

point = reshape(point,1,[]);
rectangle = reshape(rectangle,1,[]);
if numel(point) ~= 2 || numel(rectangle) ~= 4
    error('point must be 1-by-2 and rectangle must be 1-by-4.');
end
if nargin < 3 || isempty(clearance)
    clearance = 0;
end
if ~isscalar(clearance) || ~isfinite(clearance) || clearance < 0
    error('clearance must be a finite non-negative scalar.');
end

xMin = rectangle(1)-clearance;
xMax = rectangle(1)+rectangle(3)+clearance;
yMin = rectangle(2)-clearance;
yMax = rectangle(2)+rectangle(4)+clearance;

nearestPoint = [min(max(point(1),xMin),xMax), ...
    min(max(point(2),yMin),yMax)];
delta = point-nearestPoint;
distance = norm(delta);

if point(1) >= xMin && point(1) <= xMax && ...
        point(2) >= yMin && point(2) <= yMax
    distancesToSides = [point(1)-xMin,xMax-point(1), ...
        point(2)-yMin,yMax-point(2)];
    [distanceToSide,side] = min(distancesToSides);
    switch side
        case 1
            nearestPoint = [xMin,point(2)];
        case 2
            nearestPoint = [xMax,point(2)];
        case 3
            nearestPoint = [point(1),yMin];
        otherwise
            nearestPoint = [point(1),yMax];
    end
    delta = point-nearestPoint;
    distance = distanceToSide;
end

if distance > 1e-12
    repulsiveDirection = delta/distance;
else
    repulsiveDirection = [1 0];
end

end
