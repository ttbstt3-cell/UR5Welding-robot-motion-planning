function free = collisionFree(p1,p2,obstacles,clearance)
%% collisionFree  Continuous 2-D line-segment collision check.
%
%   free = collisionFree(p1,p2,obstacles,clearance) checks whether the
%   closed line segment p1--p2 intersects any axis-aligned rectangular
%   obstacle.  A non-negative clearance expands every obstacle on all
%   sides before the check.
%
%   p1, p2       1-by-2 points [x y]
%   obstacles    N-by-4 rectangles [x y width height]
%   clearance    safety distance; defaults to zero
%
%   Boundary contact is treated as a collision.  The test is continuous,
%   so it does not depend on a sampling resolution along the segment.

if nargin < 4 || isempty(clearance)
    clearance = 0;
end

if ~isscalar(clearance) || ~isfinite(clearance) || clearance < 0
    error('clearance must be a finite non-negative scalar.');
end

p1 = reshape(p1,1,[]);
p2 = reshape(p2,1,[]);

if numel(p1) ~= 2 || numel(p2) ~= 2
    error('p1 and p2 must each contain exactly two coordinates.');
end

if isempty(obstacles)
    free = true;
    return;
end

if size(obstacles,2) ~= 4
    error('obstacles must be an N-by-4 matrix [x y width height].');
end

if any(~isfinite(obstacles(:))) || any(obstacles(:,3:4) < 0,'all')
    error('obstacles must contain finite coordinates and non-negative sizes.');
end

d = p2-p1;
free = true;

for i = 1:size(obstacles,1)
    obsMin = obstacles(i,1:2)-clearance;
    obsMax = obstacles(i,1:2)+obstacles(i,3:4)+clearance;

    % Slab intersection for t in [0, 1].
    tEnter = 0;
    tExit = 1;
    intersected = true;

    for j = 1:2
        if abs(d(j)) < 1e-12
            if p1(j) < obsMin(j) || p1(j) > obsMax(j)
                intersected = false;
                break;
            end
        else
            t1 = (obsMin(j)-p1(j))/d(j);
            t2 = (obsMax(j)-p1(j))/d(j);

            tEnter = max(tEnter,min(t1,t2));
            tExit = min(tExit,max(t1,t2));

            if tEnter > tExit
                intersected = false;
                break;
            end
        end
    end

    if intersected
        free = false;
        return;
    end
end

end
