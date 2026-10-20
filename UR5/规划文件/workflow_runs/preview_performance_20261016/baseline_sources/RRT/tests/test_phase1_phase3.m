function test_phase1_phase3()
% test_phase1_phase3  Verification for the existing baselines and Phases 2-3.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

assert(exist(fullfile(projectRoot,'RRT_basic_animation.m'),'file') == 2);
assert(exist(fullfile(projectRoot,'Bi_RRT_animation.m'),'file') == 2);
assert(exist(fullfile(projectRoot,'collisionFree.m'),'file') == 2);
assert(exist(fullfile(projectRoot,'collisionFreeJointEdge.m'),'file') == 2);
assert(exist(fullfile(projectRoot,'RRT_connect.m'),'file') == 2);
assert(exist(fullfile(projectRoot,'RRT_star.m'),'file') == 2);

config = defaultRRTConfig();
config.showAnimation = false;
config.maxIterations = 5000;
config.optimizeIterations = 300;
config.seed = 1;

% Continuous shared collision checks: crossing, tangent, and clearance.
assert(~collisionFree([10 30],[50 30],config.obstacles,0));
assert(~collisionFree([10 55],[50 55],config.obstacles,0));
assert(collisionFree([10 58],[50 58],config.obstacles,2));

connectResult = RRT_connect(config);
assert(connectResult.success);
assert(validatePath(connectResult.path,config));
assert(connectResult.firstSolutionIteration == connectResult.iterations);
assert(connectResult.firstPathLength == connectResult.pathLength);
assert(connectResult.rewireCount == 0);

optimizationBudgets = [0 100 500 1000];
starResults = cell(size(optimizationBudgets));
for budgetIndex = 1:numel(optimizationBudgets)
    config.optimizeIterations = optimizationBudgets(budgetIndex);
    starResult = RRT_star(config);
    starResults{budgetIndex} = starResult;

    assert(starResult.success);
    assert(validatePath(starResult.path,config));
    assert(validatePath(starResult.firstPath,config));
    assert(starResult.rewireCount >= 0);
    assert(starResult.firstSolutionIteration <= starResult.iterations);
    assert(starResult.optimizationIterationsRequested == optimizationBudgets(budgetIndex));
    assert(starResult.optimizationIterationsExecuted == optimizationBudgets(budgetIndex));
    assert(starResult.iterations-starResult.firstSolutionIteration == ...
        optimizationBudgets(budgetIndex));
end

% Reproducibility check uses the 100-iteration result.
assert(starResults{4}.rewireCount > 0);
config.optimizeIterations = 100;
starResult = starResults{2};

repeatConnect = RRT_connect(config);
repeatStar = RRT_star(config);
assert(isequal(connectResult.path,repeatConnect.path));
assert(isequal(starResult.path,repeatStar.path));
assert(connectResult.collisionChecks == repeatConnect.collisionChecks);
assert(starResult.collisionChecks == repeatStar.collisionChecks);

fprintf('RRT* optimization budgets 0/100/500/1000 passed.\n');
fprintf('Phase 1-3 tests passed.\n');

end
