function [tree,rewireCount] = rewireGDRRTStar( ...
    tree,newIndex,nearIndices,collisionFcn)
% rewireGDRRTStar  GD-RRT* paper-order Quick-Rewire.
%
% For every xNear, first try Parent(xNew).  Only when that candidate does
% not improve the cost or its edge is blocked, try xNew itself.  This is
% intentionally separate from rewireQRRTStar, whose candidate ordering is
% cost-based.

rewireCount = 0;
newPoint = tree.nodes(newIndex,:);
newParent = tree.parent(newIndex);

if newParent == 0
    return;
end

parentPoint = tree.nodes(newParent,:);

for nearIndex = reshape(unique(nearIndices),1,[])
    if nearIndex == newIndex || nearIndex == newParent
        continue;
    end

    % Parent(xNew) or xNew must not be below xNear in the current tree.
    if isDescendant(tree,nearIndex,newParent) || ...
            isDescendant(tree,nearIndex,newIndex)
        continue;
    end

    candidateParents = [newParent,newIndex];
    candidatePoints = [parentPoint;newPoint];
    candidateCosts = [
        tree.cost(newParent)+norm(parentPoint-tree.nodes(nearIndex,:)), ...
        tree.cost(newIndex)+norm(newPoint-tree.nodes(nearIndex,:))];

    % The first candidate is always Parent(xNew), as required by the
    % GD-RRT* paper.  xNew is considered only after it fails.
    for candidatePosition = 1:2
        candidate = candidateParents(candidatePosition);
        candidateCost = candidateCosts(candidatePosition);

        if candidate == nearIndex || ...
                candidateCost >= tree.cost(nearIndex)-1e-12
            continue;
        end

        if collisionFcn(candidatePoints(candidatePosition,:), ...
                tree.nodes(nearIndex,:))
            tree.parent(nearIndex) = candidate;
            tree.cost(nearIndex) = candidateCost;
            tree = propagateDescendantCosts(tree,nearIndex);
            rewireCount = rewireCount+1;
            break;
        end
    end
end

end
