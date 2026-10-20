function [simplifiedPath,stats] = greedySimplifyPath(path,collisionFcn)
% greedySimplifyPath  Remove redundant path nodes from the goal end.

path = reshape(path,[],2);
if size(path,1) < 2
    simplifiedPath = path;
    stats.collisionChecks = 0;
    return;
end
if ~isa(collisionFcn,'function_handle')
    error('collisionFcn must be a function handle.');
end

stats.collisionChecks = 0;
keptIndices = size(path,1);
currentIndex = size(path,1);

while currentIndex > 1
    selectedIndex = currentIndex-1;
    for candidateIndex = currentIndex-2:-1:1
        stats.collisionChecks = stats.collisionChecks+1;
        if collisionFcn(path(candidateIndex,:),path(currentIndex,:))
            selectedIndex = candidateIndex;
            break;
        end
    end
    keptIndices = [selectedIndex,keptIndices]; %#ok<AGROW>
    currentIndex = selectedIndex;
end

simplifiedPath = path(keptIndices,:);

end
