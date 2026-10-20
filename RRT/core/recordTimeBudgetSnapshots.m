function [snapshots,nextIndex] = recordTimeBudgetSnapshots( ...
        snapshots,nextIndex,budgetsMs,actualTimeMs,bestPath, ...
        firstSolutionTime,nodeCount,collisionChecks,rewireCount)
% recordTimeBudgetSnapshots  Fill all checkpoints crossed by one iteration.

while nextIndex <= numel(budgetsMs) && ...
        actualTimeMs >= budgetsMs(nextIndex)
    snapshot = snapshots(nextIndex);
    snapshot.successByBudget = ~isempty(bestPath);
    snapshot.firstSolutionFound = ~isempty(bestPath);
    if ~isempty(bestPath)
        snapshot.firstSolutionTime_ms = firstSolutionTime*1000;
        snapshot.bestPathLength = pathLength(bestPath);
        snapshot.bestPath = bestPath;
    else
        snapshot.bestPath = [];
    end
    snapshot.nodeCount = nodeCount;
    snapshot.collisionChecks = collisionChecks;
    snapshot.rewireCount = rewireCount;
    snapshot.actualSnapshotTime_ms = actualTimeMs;
    snapshot.budgetOvershoot_ms = actualTimeMs-snapshot.budget_ms;
    snapshots(nextIndex) = snapshot;
    nextIndex = nextIndex+1;
end
end
