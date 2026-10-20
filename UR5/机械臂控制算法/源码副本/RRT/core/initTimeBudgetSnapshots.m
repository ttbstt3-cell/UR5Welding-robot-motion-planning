function snapshots = initTimeBudgetSnapshots(budgetsMs)
% initTimeBudgetSnapshots  Allocate fixed wall-clock checkpoint records.

budgetsMs = reshape(double(budgetsMs),1,[]);
template = struct('budget_ms',NaN, 'successByBudget',false, ...
    'firstSolutionFound',false, 'firstSolutionTime_ms',NaN, ...
    'bestPathLength',NaN, 'nodeCount',NaN, 'collisionChecks',NaN, ...
    'rewireCount',NaN, 'bestPath',[], 'actualSnapshotTime_ms',NaN, ...
    'budgetOvershoot_ms',NaN);
snapshots = repmat(template,numel(budgetsMs),1);
for i = 1:numel(budgetsMs)
    snapshots(i).budget_ms = budgetsMs(i);
end
end
