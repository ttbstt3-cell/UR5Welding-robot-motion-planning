function [tree,rewireCount] = rewireRRTStar( ...
    tree,newIndex,candidateIndices,collisionFcn)
% rewireRRTStar  Shared RRT* sequential Rewire operation.

candidateIndices = unique(reshape(candidateIndices,[],1));
rewireCount = 0;
newPoint = tree.nodes(newIndex,:);

for candidateIndex = reshape(candidateIndices,1,[])
    if candidateIndex == newIndex || candidateIndex == tree.parent(newIndex)
        continue;
    end

    candidateCost = tree.cost(newIndex)+ ...
        norm(tree.nodes(candidateIndex,:)-newPoint);

    if candidateCost < tree.cost(candidateIndex)-1e-12 && ...
            collisionFcn(newPoint,tree.nodes(candidateIndex,:))
        tree.parent(candidateIndex) = newIndex;
        tree.cost(candidateIndex) = candidateCost;
        tree = propagateDescendantCosts(tree,candidateIndex);
        rewireCount = rewireCount+1;
    end
end

end
