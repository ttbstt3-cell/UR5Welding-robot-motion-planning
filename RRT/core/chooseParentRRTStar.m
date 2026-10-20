function [bestParent,bestCost] = chooseParentRRTStar( ...
    tree,candidateIndices,newPoint,defaultParent,collisionFcn)
% chooseParentRRTStar  Shared RRT* ChooseParent operation.
%
% collisionFcn must have the signature free = collisionFcn(p1,p2).  The
% callback lets each planner count collision checks without duplicating the
% geometric collision implementation.

candidateIndices = unique(reshape(candidateIndices,[],1));

bestParent = defaultParent;
bestCost = tree.cost(defaultParent)+ ...
    norm(tree.nodes(defaultParent,:)-newPoint);

for candidateIndex = reshape(candidateIndices,1,[])
    candidateCost = tree.cost(candidateIndex)+ ...
        norm(tree.nodes(candidateIndex,:)-newPoint);

    if candidateCost < bestCost-1e-12 && ...
            collisionFcn(tree.nodes(candidateIndex,:),newPoint)
        bestParent = candidateIndex;
        bestCost = candidateCost;
    end
end

end
