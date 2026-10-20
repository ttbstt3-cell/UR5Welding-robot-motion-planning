function [tree,rewireCount] = rewireCRRTStar( ...
    tree,newIndex,nearIndices,collisionFcn)
% rewireCRRTStar  Sequential Rewire-C-RRT* from Algorithm 5.
%
% Each xNear is tested in the fixed order Parent(xNew), xNew, and
% Parent(Parent(xNear)).  The next candidate is considered when the current
% candidate fails either the cost-improvement or collision-free condition.

rewireCount = 0;
newParent = tree.parent(newIndex);
if newParent == 0
    return;
end

newPoint = tree.nodes(newIndex,:);
for nearIndex = reshape(unique(nearIndices),1,[])
    if nearIndex == newIndex
        continue;
    end

    candidateParents = [newParent,newIndex];
    nearParent = tree.parent(nearIndex);
    if nearParent ~= 0
        nearGrandparent = tree.parent(nearParent);
        candidateParents(end+1) = nearGrandparent;
    end

    testedCandidates = zeros(0,1);
    for candidate = reshape(candidateParents,1,[])
        if candidate == 0 || candidate == nearIndex || ...
                any(testedCandidates == candidate)
            continue;
        end
        testedCandidates(end+1,1) = candidate; %#ok<AGROW>

        % A candidate below xNear would make xNear its ancestor and create
        % a cycle.  The candidate index is already valid by construction.
        if isDescendant(tree,nearIndex,candidate)
            continue;
        end

        candidateCost = tree.cost(candidate)+ ...
            norm(tree.nodes(candidate,:)-tree.nodes(nearIndex,:));
        if candidateCost >= tree.cost(nearIndex)-1e-12
            continue;
        end

        if collisionFcn(tree.nodes(candidate,:),tree.nodes(nearIndex,:))
            tree.parent(nearIndex) = candidate;
            tree.cost(nearIndex) = candidateCost;
            tree = propagateDescendantCosts(tree,nearIndex);
            rewireCount = rewireCount+1;
            break;
        end
    end
end

end
