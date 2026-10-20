function [newPoint,reached] = steer(fromPoint,toPoint,stepSize)
% steer  Move from a point toward a target by at most stepSize.

fromPoint = reshape(fromPoint,1,[]);
toPoint = reshape(toPoint,1,[]);

if numel(fromPoint) ~= 2 || numel(toPoint) ~= 2
    error('fromPoint and toPoint must each contain two coordinates.');
end

direction = toPoint-fromPoint;
distance = norm(direction);

if distance < 1e-12
    newPoint = fromPoint;
    reached = true;
    return;
end

moveDistance = min(stepSize,distance);
newPoint = fromPoint+(moveDistance/distance)*direction;
reached = distance <= stepSize+1e-12;

end
