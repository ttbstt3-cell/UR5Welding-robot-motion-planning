function test_phase5_informed_rrt_star()
% test_phase5_informed_rrt_star  Verification and convergence plot for Phase 5.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

assert(exist(fullfile(projectRoot,'Informed_RRT_star.m'),'file') == 2);
assert(exist(fullfile(projectRoot,'planners','Informed_RRT_star_impl.m'),'file') == 2);
assert(exist(fullfile(projectRoot,'core','sampleInformedEllipse.m'),'file') == 2);

config = defaultRRTConfig();
config.showAnimation = false;
config.maxIterations = 5000;
config.seed = 1;

% Before the first solution, Informed RRT* must follow the same random
% sequence and RRT* tree operations as standard RRT*.
config.optimizeIterations = 0;
standardFirst = RRT_star(config);
informedFirst = Informed_RRT_star(config);
assert(standardFirst.success && informedFirst.success);
assert(standardFirst.firstSolutionIteration == informedFirst.firstSolutionIteration);
assert(abs(standardFirst.firstPathLength-informedFirst.firstPathLength) < 1e-10);
assert(isequal(standardFirst.firstPath,informedFirst.firstPath));
assert(informedFirst.uniformSampleCount == informedFirst.firstSolutionIteration);
assert(informedFirst.informedSampleCount == 0);

% Optimization budget semantics and path validity.
budgets = [0 100 500 1000];
budgetResults = cell(size(budgets));
for budgetIndex = 1:numel(budgets)
    config.optimizeIterations = budgets(budgetIndex);
    result = Informed_RRT_star(config);
    budgetResults{budgetIndex} = result;

    assert(result.success);
    assert(validatePath(result.path,config));
    assert(validatePath(result.firstPath,config));
    assert(result.iterations-result.firstSolutionIteration == budgets(budgetIndex));
    assert(result.optimizationIterationsRequested == budgets(budgetIndex));
    assert(result.optimizationIterationsExecuted == budgets(budgetIndex));
    assert(result.pathLength <= result.firstPathLength+1e-9);
    assert(result.finalCBest == result.pathLength);
end

% Every directly generated informed sample must satisfy the ellipse
% inequality and map bounds.
startPoint = config.start;
goalPoint = config.goal;
cBestForTest = 200;
for sampleIndex = 1:1000
    [sample,usedInformed,~] = sampleInformedEllipse( ...
        startPoint,goalPoint,cBestForTest,config.mapBounds,100,1e-10);
    assert(usedInformed);
    assert(norm(sample-startPoint)+norm(sample-goalPoint) <= ...
        cBestForTest+1e-9);
    assert(sample(1) >= config.mapBounds(1) && ...
        sample(1) <= config.mapBounds(2));
    assert(sample(2) >= config.mapBounds(3) && ...
        sample(2) <= config.mapBounds(4));
end

% Ellipse area must shrink when cBest decreases toward cMin.
cMin = norm(goalPoint-startPoint);
cBest1 = 200;
cBest2 = 150;
a1 = cBest1/2;
b1 = sqrt(max(cBest1^2-cMin^2,0))/2;
a2 = cBest2/2;
b2 = sqrt(max(cBest2^2-cMin^2,0))/2;
assert(a2 < a1 && b2 < b1);
assert(pi*a2*b2 < pi*a1*b1);

% Compare convergence against standard RRT* under the same seed and budget.
config.optimizeIterations = 1000;
standardResult = RRT_star(config);
informedResult = Informed_RRT_star(config);
assert(standardResult.success && informedResult.success);
assert(standardResult.pathLength <= standardResult.firstPathLength+1e-9);
assert(informedResult.pathLength <= informedResult.firstPathLength+1e-9);

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

xlabel('Iterations after first solution');
ylabel('Best path length');
title('RRT* vs Informed RRT* convergence');
legend('Location','best');
exportgraphics(figureHandle, ...
    fullfile(resultsDirectory,'phase5_rrtstar_vs_informed_convergence.png'), ...
    'Resolution',300);
close(figureHandle);

fprintf('Informed RRT* first-solution consistency passed.\n');
fprintf('Informed ellipse 1000-sample and shrink tests passed.\n');
fprintf('Informed RRT* optimization budgets 0/100/500/1000 passed.\n');
fprintf('Convergence figure written to results/phase5_rrtstar_vs_informed_convergence.png.\n');
fprintf('RRT*: first %.6f, final %.6f. Informed: first %.6f, final %.6f.\n', ...
    standardResult.firstPathLength,standardResult.pathLength, ...
    informedResult.firstPathLength,informedResult.pathLength);

    function plotConvergence(result,labelName,color)
        valid = ~isnan(result.bestCostHistory) & ...
            ~isinf(result.bestCostHistory);
        x = result.iterationHistory(valid)-result.firstSolutionIteration;
        y = result.bestCostHistory(valid);
        plot(x,y,'LineWidth',1.5,'Color',color,'DisplayName',labelName);
    end

end
