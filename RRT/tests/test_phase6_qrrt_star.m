function test_phase6_qrrt_star()
% test_phase6_qrrt_star  Verification and convergence plot for Phase 6.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

assert(exist(fullfile(projectRoot,'Q_RRT_star.m'),'file') == 2);
assert(exist(fullfile(projectRoot,'planners','Q_RRT_star_impl.m'),'file') == 2);

% Ancestor unit test: 1 <- 2 <- 3 <- 4 <- 5.
unitTree.nodes = [(1:5).' zeros(5,1)];
unitTree.parent = [0;1;2;3;4];
assert(isequal(getAncestorNodes(unitTree,5,1),4));
assert(isequal(getAncestorNodes(unitTree,5,2),[4;3]));
assert(isequal(getAncestorNodes(unitTree,5,3),[4;3;2]));
assert(isequal(getAncestorNodes(unitTree,5,10),[4;3;2;1]));
assert(isempty(getAncestorNodes(unitTree,1,10)));

config = defaultRRTConfig();
config.showAnimation = false;
config.maxIterations = 5000;
config.seed = 1;
config.qDepth = 2;

% Q-RRT* must not use the shared goal-bias setting.
config.optimizeIterations = 0;
config.goalBias = 0;
noBiasResult = Q_RRT_star(config);
config.goalBias = 0.9;
biasedConfigResult = Q_RRT_star(config);
assert(isequal(noBiasResult.firstPath,biasedConfigResult.firstPath));
assert(noBiasResult.firstSolutionIteration == ...
    biasedConfigResult.firstSolutionIteration);

% Optimization budget semantics and path validation.
config.goalBias = 0.05;
budgets = [0 100 500 1000];
budgetResults = cell(size(budgets));
for budgetIndex = 1:numel(budgets)
    config.optimizeIterations = budgets(budgetIndex);
    result = Q_RRT_star(config);
    budgetResults{budgetIndex} = result;

    assert(result.success);
    assert(validatePath(result.path,config));
    assert(validatePath(result.firstPath,config));
    assert(result.iterations-result.firstSolutionIteration == budgets(budgetIndex));
    assert(result.optimizationIterationsRequested == budgets(budgetIndex));
    assert(result.optimizationIterationsExecuted == budgets(budgetIndex));
    assert(result.pathLength <= result.firstPathLength+1e-9);
end

% The ancestor mechanism must have actually executed for seed 1.
result = budgetResults{4};
assert(result.ancestorCandidateCount > 0);
assert(result.ancestorChecks > 0);
assert(result.ancestorAcceptedCount > 0);
assert(result.rewireCount > 0);
assertTreeValid(result.tree);

% Depth comparison required by the Phase 6 acceptance criteria.
depthResults = cell(1,3);
for depth = 1:3
    config.qDepth = depth;
    config.optimizeIterations = 1000;
    depthResult = Q_RRT_star(config);
    depthResults{depth} = depthResult;
    assert(depthResult.success);
    assert(validatePath(depthResult.path,config));
    assert(depthResult.qDepth == depth);
    assertTreeValid(depthResult.tree);
end

% RRT* versus Q-RRT* under the same seed and optimization budget.
config.qDepth = 2;
config.optimizeIterations = 1000;
standardResult = RRT_star(config);
qResult = Q_RRT_star(config);
assert(standardResult.success && qResult.success);
assert(validatePath(standardResult.path,config));
assert(validatePath(qResult.path,config));
assert(standardResult.pathLength <= standardResult.firstPathLength+1e-9);
assert(qResult.pathLength <= qResult.firstPathLength+1e-9);

% Build the requested three-way convergence figure.
informedResult = Informed_RRT_star(config);
assert(informedResult.success);

resultsDirectory = fullfile(projectRoot,'results');
if ~exist(resultsDirectory,'dir')
    mkdir(resultsDirectory);
end

figureHandle = figure('Visible','off','Color','w');
hold on;
grid on;
box on;

plotConvergence(standardResult,'RRT*',[0.00 0.45 0.74]);
plotConvergence(informedResult,'Informed RRT*',[0.85 0.33 0.10]);
plotConvergence(qResult,'Q-RRT*',[0.47 0.67 0.19]);

xlabel('Iterations after first solution');
ylabel('Best path length');
title('RRT* vs Informed RRT* vs Q-RRT* convergence');
legend('Location','best');
exportgraphics(figureHandle, ...
    fullfile(resultsDirectory,'phase6_rrtstar_informed_qrrt_convergence.png'), ...
    'Resolution',300);
close(figureHandle);

fprintf('Q-RRT* ancestor unit test passed.\n');
fprintf('Q-RRT* optimization budgets 0/100/500/1000 passed.\n');
fprintf('Q-RRT* depth 1/2/3 tree and path tests passed.\n');
fprintf('Phase 6 convergence figure written to results/phase6_rrtstar_informed_qrrt_convergence.png.\n');

for depth = 1:3
    depthResult = depthResults{depth};
    fprintf(['Depth=%d FirstIteration=%d FirstTime_ms=%.6f ', ...
        'FirstLength=%.6f FinalLength=%.6f Planning_ms=%.6f ', ...
        'Nodes=%d Checks=%d AncestorChecks=%d AncestorAccepted=%d ', ...
        'Rewires=%d\n'], ...
        depth,depthResult.firstSolutionIteration, ...
        depthResult.firstSolutionTime*1000,depthResult.firstPathLength, ...
        depthResult.pathLength,depthResult.planningTime*1000, ...
        depthResult.nodeCount,depthResult.collisionChecks, ...
        depthResult.ancestorChecks,depthResult.ancestorAcceptedCount, ...
        depthResult.rewireCount);
end

fprintf(['RRT*: FirstIteration=%d FirstTime_ms=%.6f FirstLength=%.6f ', ...
    'FinalLength=%.6f Planning_ms=%.6f Nodes=%d Checks=%d Rewires=%d\n'], ...
    standardResult.firstSolutionIteration,standardResult.firstSolutionTime*1000, ...
    standardResult.firstPathLength,standardResult.pathLength, ...
    standardResult.planningTime*1000,standardResult.nodeCount, ...
    standardResult.collisionChecks,standardResult.rewireCount);

fprintf(['Q-RRT*: FirstIteration=%d FirstTime_ms=%.6f FirstLength=%.6f ', ...
    'FinalLength=%.6f Planning_ms=%.6f Nodes=%d Checks=%d Rewires=%d\n'], ...
    qResult.firstSolutionIteration,qResult.firstSolutionTime*1000, ...
    qResult.firstPathLength,qResult.pathLength,qResult.planningTime*1000, ...
    qResult.nodeCount,qResult.collisionChecks,qResult.rewireCount);

    function plotConvergence(result,labelName,color)
        valid = ~isnan(result.bestCostHistory) & ...
            ~isinf(result.bestCostHistory);
        x = result.iterationHistory(valid)-result.firstSolutionIteration;
        y = result.bestCostHistory(valid);
        plot(x,y,'LineWidth',1.5,'Color',color,'DisplayName',labelName);
    end

end

function assertTreeValid(tree)
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
    assert(parentIndex >= 1 && parentIndex <= size(tree.nodes,1));
    assert(parentIndex ~= nodeIndex);
    assert(abs(tree.cost(nodeIndex)- ...
        (tree.cost(parentIndex)+ ...
        norm(tree.nodes(nodeIndex,:)-tree.nodes(parentIndex,:)))) < tolerance);

    % Walking the parent chain must terminate at root without cycles.
    visited = false(size(tree.parent));
    currentIndex = nodeIndex;
    while currentIndex ~= 0
        assert(~visited(currentIndex));
        visited(currentIndex) = true;
        currentIndex = tree.parent(currentIndex);
        if currentIndex ~= 0
            assert(currentIndex >= 1 && currentIndex <= size(tree.nodes,1));
        end
    end
end
end
