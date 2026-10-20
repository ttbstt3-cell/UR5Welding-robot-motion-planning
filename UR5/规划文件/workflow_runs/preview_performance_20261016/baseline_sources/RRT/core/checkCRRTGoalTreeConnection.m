function [bestGoalNode,bestCost,stats] = checkCRRTGoalTreeConnection( ...
    tree,goalTree,newIndex,collisionFcn)
% checkCRRTGoalTreeConnection  Connect Type 2/4 nodes to goal-tree branches.

bestGoalNode = NaN;
bestCost = Inf;
stats.connectionChecks = 0;
stats.connectionSuccesses = 0;

if size(goalTree.nodes,1) <= 1
    return;
end

newPoint = tree.nodes(newIndex,:);
for goalNode = 2:size(goalTree.nodes,1)
    candidateCost = tree.cost(newIndex)+ ...
        norm(newPoint-goalTree.nodes(goalNode,:))+ ...
        goalTree.costToGoal(goalNode);

    stats.connectionChecks = stats.connectionChecks+1;
    if collisionFcn(newPoint,goalTree.nodes(goalNode,:))
        stats.connectionSuccesses = stats.connectionSuccesses+1;
        if candidateCost < bestCost-1e-12
            bestGoalNode = goalNode;
            bestCost = candidateCost;
        end
    end
end

end
