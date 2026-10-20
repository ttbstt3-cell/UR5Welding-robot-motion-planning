function test_phase4_bi_rrt_star()
% test_phase4_bi_rrt_star  Verification for the Phase 4 Bi-RRT* planner.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

assert(exist(fullfile(projectRoot,'Bi_RRT_star.m'),'file') == 2);
assert(exist(fullfile(projectRoot,'planners','Bi_RRT_star_impl.m'),'file') == 2);

config = defaultRRTConfig();
config.showAnimation = false;
config.maxIterations = 5000;
config.seed = 1;

budgets = [0 100 500 1000];
results = cell(size(budgets));

for budgetIndex = 1:numel(budgets)
    config.optimizeIterations = budgets(budgetIndex);
    result = Bi_RRT_star(config);
    results{budgetIndex} = result;

    assert(result.success);
    assert(norm(result.path(1,:)-config.start) < 1e-8);
    assert(norm(result.path(end,:)-config.goal) < 1e-8);
    assert(validatePath(result.path,config));
    assert(validatePath(result.firstPath,config));
    assert(result.iterations-result.firstSolutionIteration == budgets(budgetIndex));
    assert(result.optimizationIterationsRequested == budgets(budgetIndex));
    assert(result.optimizationIterationsExecuted == budgets(budgetIndex));
    assert(result.pathLength <= result.firstPathLength+1e-9);
    assert(result.startTreeNodeCount+result.goalTreeNodeCount == result.nodeCount);
    assert(numel(result.iterationHistory) == result.iterations);
    assert(numel(result.bestCostHistory) == result.iterations);
end

% Rewire descendant cost consistency on the longest run.
result = results{4};
assert(result.rewireCount > 0);
assertTreeCosts(result.startTree);
assertTreeCosts(result.goalTree);
assert(abs(result.pathLength-result.bestCost) < 1e-8);

fprintf('Bi-RRT* optimization budgets 0/100/500/1000 passed.\n');
fprintf('Bi-RRT* cost consistency and path optimization passed.\n');

end

function assertTreeCosts(tree)
tolerance = 1e-9;
assert(size(tree.nodes,2) == 2);
assert(size(tree.parent,2) == 1);
assert(size(tree.cost,2) == 1);
assert(numel(tree.parent) == size(tree.nodes,1));
assert(numel(tree.cost) == size(tree.nodes,1));

assert(tree.parent(1) == 0);
assert(abs(tree.cost(1)) < tolerance);

for nodeIndex = 2:size(tree.nodes,1)
    parentIndex = tree.parent(nodeIndex);
    expectedCost = tree.cost(parentIndex)+ ...
        norm(tree.nodes(nodeIndex,:)-tree.nodes(parentIndex,:));
    assert(abs(tree.cost(nodeIndex)-expectedCost) < tolerance);
end
end
