function [bestParent,bestCost,stats] = chooseParentCRRTStar( ...
    tree,nearestIndex,nearIndices,newPoint,backtrackDepth,collisionFcn)
% chooseParentCRRTStar  Algorithm 4's two-stage parent selection.
%
% Stage 1 selects xmin from Nearest+Near using cost-improving collision
% checks only.  Stage 2 checks xmin's ancestors in reverse order, from the
% farthest requested depth down to xmin itself, and accepts the first free
% edge.  This is deliberately different from Q-RRT*'s nearest-first union.

validateIndex(nearestIndex,size(tree.nodes,1),'nearestIndex');
nearIndices = reshape(unique(nearIndices),[],1);
for i = 1:numel(nearIndices)
    validateIndex(nearIndices(i),size(tree.nodes,1),'nearIndex');
end
if ~isscalar(backtrackDepth) || ~isfinite(backtrackDepth) || ...
        backtrackDepth < 0 || backtrackDepth ~= floor(backtrackDepth)
    error('backtrackDepth must be a non-negative integer.');
end

stats.backtrackChecks = 0;
stats.backtrackAcceptedCount = 0;
stats.backtrackAcceptedDepth = NaN;

bestParent = nearestIndex;
bestCost = tree.cost(nearestIndex)+ ...
    norm(tree.nodes(nearestIndex,:)-newPoint);

stageOneCandidates = unique([nearestIndex;nearIndices]);
xmin = nearestIndex;
cmin = bestCost;
for candidateIndex = reshape(stageOneCandidates,1,[])
    candidateCost = tree.cost(candidateIndex)+ ...
        norm(tree.nodes(candidateIndex,:)-newPoint);
    if candidateCost < cmin-1e-12 && ...
            collisionFcn(tree.nodes(candidateIndex,:),newPoint)
        xmin = candidateIndex;
        cmin = candidateCost;
    end
end

% Reverse-backtracking order: dbmax, ..., 1, 0.  The actual available
% depth is handled by getBacktrackAncestor returning zero at the root.
for depth = backtrackDepth:-1:0
    candidateIndex = getBacktrackAncestor(tree,xmin,depth);
    if candidateIndex == 0
        continue;
    end

    stats.backtrackChecks = stats.backtrackChecks+1;
    if collisionFcn(tree.nodes(candidateIndex,:),newPoint)
        bestParent = candidateIndex;
        bestCost = tree.cost(candidateIndex)+ ...
            norm(tree.nodes(candidateIndex,:)-newPoint);
        stats.backtrackAcceptedCount = 1;
        stats.backtrackAcceptedDepth = depth;
        return;
    end
end

% xmin was collision-checked in Stage 1 when it was a non-nearest winner;
% the nearest edge was checked by the caller.  Keeping xmin as a fallback
% prevents a failed reverse scan from losing a valid base extension.
bestParent = xmin;
bestCost = cmin;

end

function validateIndex(index,numberOfNodes,name)
if ~isscalar(index) || index < 1 || index > numberOfNodes || ...
        index ~= floor(index)
    error('%s is not a valid tree node index.',name);
end
end
