function [point,usedInformed,rejectCount] = sampleInformedEllipse( ...
    startPoint,goalPoint,cBest,mapBounds,maxRetries,tolerance)
% sampleInformedEllipse  Directly sample the 2-D Informed RRT* ellipsoid.
%
% The sample is drawn uniformly from the area of the ellipse by sampling a
% uniform point in the unit disk and transforming it with the ellipse
% rotation and semi-axis matrix.  Points outside mapBounds are resampled;
% this is not whole-map rejection sampling.

if nargin < 5 || isempty(maxRetries)
    maxRetries = 100;
end
if nargin < 6 || isempty(tolerance)
    tolerance = 1e-10;
end

startPoint = reshape(startPoint,1,[]);
goalPoint = reshape(goalPoint,1,[]);
mapBounds = reshape(mapBounds,1,[]);

if numel(startPoint) ~= 2 || numel(goalPoint) ~= 2
    error('startPoint and goalPoint must each contain two coordinates.');
end
if numel(mapBounds) ~= 4 || mapBounds(1) >= mapBounds(2) || ...
        mapBounds(3) >= mapBounds(4)
    error('mapBounds must be [xmin xmax ymin ymax].');
end
if ~isscalar(cBest) || ~isfinite(cBest) || cBest < 0
    error('cBest must be a finite non-negative scalar.');
end
if ~isscalar(maxRetries) || maxRetries < 1 || maxRetries ~= floor(maxRetries)
    error('maxRetries must be a positive integer.');
end

cMin = norm(goalPoint-startPoint);
center = (startPoint+goalPoint)/2;

% A zero-length start-goal segment or a path at the theoretical lower
% bound produces a degenerate ellipse.  Return its center rather than
% dividing by cMin or trying to sample a zero-area region.
if cMin <= tolerance || cBest <= cMin+tolerance
    point = center;
    usedInformed = false;
    rejectCount = 0;
    return;
end

a = cBest/2;
minorTerm = max(cBest^2-cMin^2,0);
b = sqrt(minorTerm)/2;
direction = (goalPoint-startPoint)/cMin;

% Columns of C are the major-axis and perpendicular unit vectors.
C = [direction(:),[-direction(2);direction(1)]];
L = diag([a,b]);

rejectCount = 0;
for attempt = 1:maxRetries
    theta = 2*pi*rand;
    radius = sqrt(rand);
    xBall = radius*[cos(theta);sin(theta)];
    point = (C*L*xBall+center(:)).';

    if point(1) >= mapBounds(1) && point(1) <= mapBounds(2) && ...
            point(2) >= mapBounds(3) && point(2) <= mapBounds(4)
        usedInformed = true;
        return;
    end

    rejectCount = rejectCount+1;
end

% The midpoint is inside the ellipse and, for valid start/goal points in a
% rectangular map, inside the map.  This deterministic fallback prevents a
% pathological map/ellipse combination from creating an infinite loop.
point = center;
usedInformed = true;

end
