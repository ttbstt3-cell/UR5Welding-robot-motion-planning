function [bestParent,bestCost,stats] = chooseParentQRRTStar( ...
    tree,nearestIndex,nearIndices,newPoint,currentDepth,collisionFcn)
% chooseParentQRRTStar  Quick-RRT* parent selection with ancestors.

ancestorIndices = zeros(0,1);
for nearIndex = reshape(nearIndices,1,[])
    ancestorIndices = [ancestorIndices; ...
        getAncestorNodes(tree,nearIndex,currentDepth)]; %#ok<AGROW>
end
ancestorIndices = unique(ancestorIndices);

candidateIndices = unique([nearestIndex;nearIndices(:);ancestorIndices]);
ancestorMask = ismember(candidateIndices,ancestorIndices);

stats.ancestorCandidateCount = numel(ancestorIndices);
stats.ancestorChecks = 0;
stats.ancestorAcceptedCount = 0;

bestParent = nearestIndex;
bestCost = tree.cost(nearestIndex)+ ...
    norm(tree.nodes(nearestIndex,:)-newPoint);

for candidatePosition = 1:numel(candidateIndices)
    candidateIndex = candidateIndices(candidatePosition);
    candidateCost = tree.cost(candidateIndex)+ ...
        norm(tree.nodes(candidateIndex,:)-newPoint);

    if ancestorMask(candidatePosition)
        % Ancestor links are always collision-tested, even when their cost
        % is not lower, so ancestorChecks reflects actual geometry checks.
        stats.ancestorChecks = stats.ancestorChecks+1;
        free = collisionFcn(tree.nodes(candidateIndex,:),newPoint);
    elseif candidateCost < bestCost-1e-12
        free = collisionFcn(tree.nodes(candidateIndex,:),newPoint);
    else
        free = false;
    end

    if free && candidateCost < bestCost-1e-12
        bestParent = candidateIndex;
        bestCost = candidateCost;
    end
end

% Count the final selected parent once, rather than counting intermediate
% best-parent replacements while scanning candidates.
stats.ancestorAcceptedCount = ismember(bestParent,ancestorIndices);

end
