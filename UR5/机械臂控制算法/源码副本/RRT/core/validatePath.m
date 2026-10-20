function valid = validatePath(path,config)
% validatePath  Validate endpoints and every path edge with collisionFree.

valid = false;

if isempty(path) || size(path,2) ~= 2
    return;
end

if norm(path(1,:)-config.start) > 1e-8 || ...
        norm(path(end,:)-config.goal) > 1e-8
    return;
end

for i = 1:size(path,1)-1
    if ~collisionFree(path(i,:),path(i+1,:), ...
            config.obstacles,config.clearance)
        return;
    end
end

valid = true;

end
