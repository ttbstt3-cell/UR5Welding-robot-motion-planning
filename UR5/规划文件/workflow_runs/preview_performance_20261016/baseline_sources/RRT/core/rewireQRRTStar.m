function [tree,rewireCount] = rewireQRRTStar( ...
    tree,newIndex,nearIndices,collisionFcn)
% rewireQRRTStar  Quick-RRT* rewire using xNew and parent(xNew).

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

    % xNew -> xNear is invalid if xNew is already below xNear.
    if isDescendant(tree,nearIndex,newIndex)
        continue;
    end

    % parent(xNew) -> xNear is invalid if that parent is below xNear.
    if isDescendant(tree,nearIndex,newParent)
        continue;
    end

    candidateParents = [newIndex,newParent];
    candidateCosts = [
        tree.cost(newIndex)+norm(newPoint-tree.nodes(nearIndex,:)), ...
        tree.cost(newParent)+norm(parentPoint-tree.nodes(nearIndex,:))
    ];
    [~,order] = sort(candidateCosts,'ascend');

    for candidatePosition = reshape(order,1,[])
        candidate = candidateParents(candidatePosition);
        candidateCost = candidateCosts(candidatePosition);

        if candidate == nearIndex || candidateCost >= tree.cost(nearIndex)-1e-12
            continue;
        end

        if candidate == newIndex
            candidatePoint = newPoint;
        else
            candidatePoint = parentPoint;
        end

        if collisionFcn(candidatePoint,tree.nodes(nearIndex,:))
            tree.parent(nearIndex) = candidate;
            tree.cost(nearIndex) = candidateCost;
            tree = propagateDescendantCosts(tree,nearIndex);
            rewireCount = rewireCount+1;
            break;
        end
    end
end

end
