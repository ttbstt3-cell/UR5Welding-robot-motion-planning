function [valid,collisionChecks] = validateSmoothedPath( ...
    path,config,collisionFcn)
% validateSmoothedPath  Validate endpoints and every sampled curve segment.

valid = false;
collisionChecks = 0;
if isempty(path) || size(path,2) ~= 2 || any(~isfinite(path(:)))
    return;
end
if norm(path(1,:)-config.start) > 1e-8 || ...
        norm(path(end,:)-config.goal) > 1e-8
    return;
end
if any(path(:,1) < config.mapBounds(1)-1e-10) || ...
        any(path(:,1) > config.mapBounds(2)+1e-10) || ...
        any(path(:,2) < config.mapBounds(3)-1e-10) || ...
        any(path(:,2) > config.mapBounds(4)+1e-10)
    return;
end

if nargin < 3 || isempty(collisionFcn)
    collisionFcn = @(p1,p2) collisionFree( ...
        p1,p2,config.obstacles,config.clearance);
end
for i = 1:size(path,1)-1
    collisionChecks = collisionChecks+1;
    if ~collisionFcn(path(i,:),path(i+1,:))
        return;
    end
end
valid = true;

end
