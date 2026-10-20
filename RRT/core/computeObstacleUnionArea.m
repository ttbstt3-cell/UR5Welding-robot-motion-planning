function [obstacleMeasure,workspaceMeasure] = computeObstacleUnionArea( ...
    obstacles,mapBounds)
% computeObstacleUnionArea  Exact 2-D union area of axis-aligned rectangles.
%
% The APF paper uses obstacle volume/workspace volume.  In this 2-D project
% those measures are interpreted as obstacle area/workspace area.  Overlaps
% are counted once, and rectangles are clipped to mapBounds.

if isempty(obstacles)
    obstacles = zeros(0,4);
end
mapBounds = reshape(mapBounds,1,4);
workspaceMeasure = (mapBounds(2)-mapBounds(1))* ...
    (mapBounds(4)-mapBounds(3));
if isempty(obstacles)
    obstacleMeasure = 0;
    return;
end

clipped = zeros(size(obstacles,1),4);
validCount = 0;
for i = 1:size(obstacles,1)
    x1 = max(mapBounds(1),obstacles(i,1));
    x2 = min(mapBounds(2),obstacles(i,1)+obstacles(i,3));
    y1 = max(mapBounds(3),obstacles(i,2));
    y2 = min(mapBounds(4),obstacles(i,2)+obstacles(i,4));
    if x2 > x1 && y2 > y1
        validCount = validCount+1;
        clipped(validCount,:) = [x1 y1 x2-x1 y2-y1];
    end
end
clipped = clipped(1:validCount,:);
if isempty(clipped)
    obstacleMeasure = 0;
    return;
end

xBreaks = unique([clipped(:,1);clipped(:,1)+clipped(:,3)]);
obstacleMeasure = 0;
for strip = 1:numel(xBreaks)-1
    xLeft = xBreaks(strip);
    xRight = xBreaks(strip+1);
    stripWidth = xRight-xLeft;
    if stripWidth <= 0
        continue;
    end
    active = clipped(clipped(:,1) <= xLeft & ...
        clipped(:,1)+clipped(:,3) >= xRight,:);
    if isempty(active)
        continue;
    end
    intervals = sortrows([active(:,2),active(:,2)+active(:,4)],1);
    coveredY = 0;
    currentStart = intervals(1,1);
    currentEnd = intervals(1,2);
    for interval = 2:size(intervals,1)
        if intervals(interval,1) <= currentEnd
            currentEnd = max(currentEnd,intervals(interval,2));
        else
            coveredY = coveredY+(currentEnd-currentStart);
            currentStart = intervals(interval,1);
            currentEnd = intervals(interval,2);
        end
    end
    coveredY = coveredY+(currentEnd-currentStart);
    obstacleMeasure = obstacleMeasure+stripWidth*coveredY;
end

end
