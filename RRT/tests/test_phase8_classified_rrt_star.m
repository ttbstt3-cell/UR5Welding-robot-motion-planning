function test_phase8_classified_rrt_star()
% test_phase8_classified_rrt_star  Verification suite for Phase 8 C-RRT*.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

assert(exist(fullfile(projectRoot,'Classified_RRT_star.m'),'file') == 2);
assert(exist(fullfile(projectRoot,'planners','Classified_RRT_star_impl.m'), ...
    'file') == 2);

%% Four-class unit test with manually constructed points and maps.
startPoint = [10 10];
goalPoint = [90 90];
classificationCases = {
    [50 50], zeros(0,4), 1; ...
    [30 70], [45 70 10 10], 2; ...
    [40 50], [20 20 20 20], 3; ...
    [50 80], [30 0 10 70; 60 60 10 40], 4};
for caseIndex = 1:size(classificationCases,1)
    point = classificationCases{caseIndex,1};
    obstacles = classificationCases{caseIndex,2};
    expectedType = classificationCases{caseIndex,3};
    [nodeType,canStart,canGoal] = classifyCRRTNode(point,startPoint, ...
        goalPoint,@(p1,p2) collisionFree(p1,p2,obstacles,0));
    assert(nodeType == expectedType);
    assert(canStart == (expectedType == 1 || expectedType == 2));
    assert(canGoal == (expectedType == 1 || expectedType == 3));
end

%% Open map: Type 1 bypasses ChooseParent and still executes Rewire.
openConfig = defaultRRTConfig();
openConfig.start = startPoint;
openConfig.goal = goalPoint;
openConfig.obstacles = zeros(0,4);
openConfig.showAnimation = false;
openConfig.crrtPaperMode = true;
openConfig.crrtPaperTotalIterations = 1000;
openConfig.maxIterations = 1000;
openConfig.seed = 1;
openResult = Classified_RRT_star(openConfig);
assert(openResult.success);
assert(openResult.nodeTypeCount(1) > 0);
assert(openResult.chooseParentCountByType(1) == 0);
assert(openResult.chooseParentCountByType(2) == 0);
assert(openResult.rewireInvocationCountByType(1) > 0);

%% U-shaped dense map: all types, offset recovery, goal-tree connections.
denseConfig = defaultRRTConfig();
denseConfig.showAnimation = false;
denseConfig.crrtPaperMode = true;
denseConfig.crrtPaperTotalIterations = 1000;
denseConfig.maxIterations = 5000;
denseConfig.seed = 1;
denseConfig.obstacles = [
    25 20 50  8
    25 20  8 55
    67 20  8 55];
denseResult = Classified_RRT_star(denseConfig);
assert(denseResult.success);
assert(all(denseResult.nodeTypeCount > 0));
assert(denseResult.chooseParentCountByType(1) == 0);
assert(denseResult.chooseParentCountByType(2) == 0);
assert(denseResult.chooseParentCountByType(3) > 0);
assert(denseResult.chooseParentCountByType(4) > 0);
assert(all(denseResult.rewireInvocationCountByType > 0));
assert(denseResult.goalTreeNodeCount > 1);
assert(denseResult.goalTreeConnectionChecks > 0);
assert(denseResult.goalTreeConnectionSuccesses > 0);
assert(denseResult.offsetTriggerCount > 0);
assert(denseResult.offsetAttemptCount > 0);
assert(denseResult.offsetSuccessCount > 0);
assert(abs(denseResult.offsetSuccessRate- ...
    denseResult.offsetSuccessCount/denseResult.offsetAttemptCount) < 1e-12);
assert(abs(sum(denseResult.nodeTypeFraction)-1) < 1e-12);

%% Goal-tree branch correctness and full tree/path validation.
assertTreeValid(denseResult.tree);
assertGoalTreeValid(denseResult.goalTree,denseConfig);
assert(validatePath(denseResult.path,denseConfig));
assert(validatePath(denseResult.firstPath,denseConfig));
assert(denseResult.iterations == denseConfig.crrtPaperTotalIterations);
assert(denseResult.iterations > denseResult.firstSolutionIteration);
assert(denseResult.pathLength <= denseResult.firstPathLength+1e-9);

%% Reverse-backtracking depth sensitivity: dbmax=0..4.
depths = 0:4;
depthResults = cell(size(depths));
for depthIndex = 1:numel(depths)
    depthConfig = defaultRRTConfig();
    depthConfig.showAnimation = false;
    depthConfig.crrtPaperMode = true;
    depthConfig.crrtPaperTotalIterations = 1000;
    depthConfig.maxIterations = 5000;
    depthConfig.crrtBacktrackDepth = depths(depthIndex);
    depthConfig.seed = 1;
    depthResults{depthIndex} = Classified_RRT_star(depthConfig);
    depthResult = depthResults{depthIndex};
    assert(depthResult.success);
    assert(depthResult.crrtBacktrackDepth == depths(depthIndex));
    assert(validatePath(depthResult.path,depthConfig));
    assertTreeValid(depthResult.tree);
end

%% Extended mode: first solution plus exactly optimizeIterations.
extendedConfig = defaultRRTConfig();
extendedConfig.showAnimation = false;
extendedConfig.crrtPaperMode = false;
extendedConfig.optimizeIterations = 100;
extendedConfig.maxIterations = 5000;
extendedConfig.seed = 1;
extendedResult = Classified_RRT_star(extendedConfig);
assert(extendedResult.success);
assert(extendedResult.iterations-extendedResult.firstSolutionIteration == 100);
assert(extendedResult.optimizationIterationsExecuted == 100);
assert(validatePath(extendedResult.path,extendedConfig));

%% Initial feasible-path comparison.
comparisonConfig = defaultRRTConfig();
comparisonConfig.showAnimation = false;
comparisonConfig.optimizeIterations = 0;
comparisonConfig.maxIterations = 5000;
comparisonConfig.seed = 1;
comparisonConfig.crrtPaperMode = false;
rrtResult = RRT_star(comparisonConfig);
informedResult = Informed_RRT_star(comparisonConfig);
qResult = Q_RRT_star(comparisonConfig);
gdConfig = comparisonConfig;
gdConfig.gdPaperMode = true;
gdResult = GD_RRT_star(gdConfig);
crrtResult = Classified_RRT_star(comparisonConfig);
assert(rrtResult.success && informedResult.success && qResult.success && ...
    gdResult.success && crrtResult.success);
assert(validatePath(rrtResult.path,comparisonConfig));
assert(validatePath(informedResult.path,comparisonConfig));
assert(validatePath(qResult.path,comparisonConfig));
assert(validatePath(gdResult.path,comparisonConfig));
assert(validatePath(crrtResult.path,comparisonConfig));

%% Fixed-budget extended comparison.
benchmarkConfig = defaultRRTConfig();
benchmarkConfig.showAnimation = false;
benchmarkConfig.optimizeIterations = 1000;
benchmarkConfig.maxIterations = 5000;
benchmarkConfig.seed = 1;
benchmarkConfig.crrtPaperMode = false;
benchmarkRRT = RRT_star(benchmarkConfig);
benchmarkInformed = Informed_RRT_star(benchmarkConfig);
benchmarkQ = Q_RRT_star(benchmarkConfig);
benchmarkC = Classified_RRT_star(benchmarkConfig);
assert(benchmarkRRT.success && benchmarkInformed.success && ...
    benchmarkQ.success && benchmarkC.success);
assert(benchmarkRRT.iterations == benchmarkRRT.firstSolutionIteration+1000);
assert(benchmarkInformed.iterations == ...
    benchmarkInformed.firstSolutionIteration+1000);
assert(benchmarkQ.iterations == benchmarkQ.firstSolutionIteration+1000);
assert(benchmarkC.iterations == benchmarkC.firstSolutionIteration+1000);
assert(validatePath(benchmarkC.path,benchmarkConfig));

%% Convergence figure: iterations after first solution.
resultsDirectory = fullfile(projectRoot,'results');
if ~exist(resultsDirectory,'dir')
    mkdir(resultsDirectory);
end
figureHandle = figure('Visible','off','Color','w');
hold on; grid on; box on;
plotConvergence(benchmarkRRT,'RRT*',[0.00 0.45 0.74]);
plotConvergence(benchmarkInformed,'Informed RRT*',[0.85 0.33 0.10]);
plotConvergence(benchmarkQ,'Q-RRT*',[0.47 0.67 0.19]);
plotConvergence(benchmarkC,'Classified RRT*',[0.49 0.18 0.56]);
xlabel('Iterations after first solution');
ylabel('Best path length');
title('RRT* family extended optimization convergence');
legend('Location','best');
exportgraphics(figureHandle,fullfile(resultsDirectory, ...
    'phase8_rrtstar_informed_qrrt_classified_convergence.png'), ...
    'Resolution',300);
close(figureHandle);

%% Paper total-iteration view: every algorithm is capped at 1000 absolute.
paperConfig = defaultRRTConfig();
paperConfig.showAnimation = false;
paperConfig.maxIterations = 1000;
paperConfig.optimizeIterations = 1000;
paperConfig.seed = 1;
paperRRT = RRT_star(paperConfig);
paperInformed = Informed_RRT_star(paperConfig);
paperQ = Q_RRT_star(paperConfig);
paperConfig.crrtPaperMode = true;
paperConfig.crrtPaperTotalIterations = 1000;
paperC = Classified_RRT_star(paperConfig);
figureHandle = figure('Visible','off','Color','w');
hold on; grid on; box on;
plotAbsolute(paperRRT,'RRT*',[0.00 0.45 0.74]);
plotAbsolute(paperInformed,'Informed RRT*',[0.85 0.33 0.10]);
plotAbsolute(paperQ,'Q-RRT*',[0.47 0.67 0.19]);
plotAbsolute(paperC,'Classified RRT*',[0.49 0.18 0.56]);
xlabel('Absolute iteration');
ylabel('Best path length');
title('First 1000 absolute iterations');
legend('Location','best');
exportgraphics(figureHandle,fullfile(resultsDirectory, ...
    'phase8_crrt_paper_total1000_convergence.png'),'Resolution',300);
close(figureHandle);

fprintf('C-RRT* four-class, goal-tree, offset, and tree tests passed.\n');
fprintf('C-RRT* dbmax=0/1/2/3/4 tests passed.\n');
fprintf('C-RRT* paper and extended termination tests passed.\n');
fprintf(['Initial comparison: RRT*=%d/%d %.6f, Informed=%d/%d %.6f, ', ...
    'Q=%d/%d %.6f, GD=%d/%d %.6f, C=%d/%d %.6f\n'], ...
    rrtResult.success,rrtResult.firstSolutionIteration,rrtResult.firstPathLength, ...
    informedResult.success,informedResult.firstSolutionIteration,informedResult.firstPathLength, ...
    qResult.success,qResult.firstSolutionIteration,qResult.firstPathLength, ...
    gdResult.success,gdResult.firstSolutionIteration,gdResult.firstPathLength, ...
    crrtResult.success,crrtResult.firstSolutionIteration,crrtResult.firstPathLength);

for depthIndex = 1:numel(depths)
    depthResult = depthResults{depthIndex};
    fprintf(['dbmax=%d First=%d FirstLength=%.6f FinalLength=%.6f ', ...
        'Planning_ms=%.6f Checks=%d BacktrackChecks=%d ', ...
        'BacktrackAccepted=%d Rewires=%d\n'], ...
        depths(depthIndex),depthResult.firstSolutionIteration, ...
        depthResult.firstPathLength,depthResult.pathLength, ...
        depthResult.planningTime*1000,depthResult.collisionChecks, ...
        depthResult.backtrackChecks,depthResult.backtrackAcceptedCount, ...
        depthResult.rewireCount);
end

fprintf(['Extended final: RRT*=%.6f Informed=%.6f Q=%.6f C=%.6f\n'], ...
    benchmarkRRT.pathLength,benchmarkInformed.pathLength, ...
    benchmarkQ.pathLength,benchmarkC.pathLength);
fprintf('Figures written to results/phase8_*.png.\n');

    function plotConvergence(result,labelName,color)
        valid = ~isnan(result.bestCostHistory) & ...
            ~isinf(result.bestCostHistory);
        x = result.iterationHistory(valid)-result.firstSolutionIteration;
        y = result.bestCostHistory(valid);
        plot(x,y,'LineWidth',1.5,'Color',color,'DisplayName',labelName);
    end

    function plotAbsolute(result,labelName,color)
        valid = ~isnan(result.bestCostHistory) & ...
            ~isinf(result.bestCostHistory);
        plot(result.iterationHistory(valid),result.bestCostHistory(valid), ...
            'LineWidth',1.5,'Color',color,'DisplayName',labelName);
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
    expectedCost = tree.cost(parentIndex)+ ...
        norm(tree.nodes(nodeIndex,:)-tree.nodes(parentIndex,:));
    assert(abs(tree.cost(nodeIndex)-expectedCost) < tolerance);

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

function assertGoalTreeValid(goalTree,config)
assert(size(goalTree.nodes,2) == 2);
assert(all(goalTree.nodes(1,:) == config.goal));
assert(goalTree.sourceTreeIndex(1) == 0);
assert(abs(goalTree.costToGoal(1)) < 1e-12);
assert(numel(goalTree.sourceTreeIndex) == size(goalTree.nodes,1));
assert(numel(goalTree.costToGoal) == size(goalTree.nodes,1));
for goalNode = 2:size(goalTree.nodes,1)
    assert(goalTree.sourceTreeIndex(goalNode) >= 2);
    assert(collisionFree(goalTree.nodes(goalNode,:),config.goal, ...
        config.obstacles,config.clearance));
    assert(abs(goalTree.costToGoal(goalNode)- ...
        norm(goalTree.nodes(goalNode,:)-config.goal)) < 1e-9);
end
end
