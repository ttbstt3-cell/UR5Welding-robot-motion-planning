function test_phase9_apf_bi_rrt_star()
% test_phase9_apf_bi_rrt_star  Verification suite for Phase 9.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

assert(exist(fullfile(projectRoot,'APF_Bi_RRT_star.m'),'file') == 2);
assert(exist(fullfile(projectRoot,'planners','APF_Bi_RRT_star_impl.m'), ...
    'file') == 2);

%% Dynamic bias and exact union-area occupancy.
areaBounds = [0 20 0 10];
overlapObstacles = [0 0 10 10; 5 0 10 10];
[obstacleArea,workspaceArea] = computeObstacleUnionArea( ...
    overlapObstacles,areaBounds);
assert(abs(obstacleArea-150) < 1e-12);
assert(abs(workspaceArea-200) < 1e-12);

config = defaultRRTConfig();
config.mapBounds = [0 100 0 100];
config.obstacles = zeros(0,4);
[emptyBias,emptyDetails] = computeDynamicGoalBias(config);
config.obstacles = [0 0 100 100];
[fullBias,fullDetails] = computeDynamicGoalBias(config);
assert(abs(emptyBias-config.apfGoalBiasMax) < 1e-12);
assert(abs(fullBias-config.apfGoalBiasMin) < 1e-12);
assert(emptyDetails.occupancy == 0);
assert(fullDetails.occupancy == 1);

rng(9,'twister');
sampleCount = 10000;
sampledGoalFraction = mean(rand(sampleCount,1) <= emptyBias);
assert(abs(sampledGoalFraction-emptyBias) < 0.03);

%% Dynamic attraction, nearest rectangle distance, and influence range.
config.obstacles = [40 40 10 10];
distances = [1 10 50];
attractionMagnitudes = zeros(size(distances));
for i = 1:numel(distances)
    [~,details] = computeImprovedAPFDirection( ...
        [10 10],[10+distances(i) 10],zeros(0,4),config);
    attractionMagnitudes(i) = norm(details.attractiveForce);
end
assert(all(diff(attractionMagnitudes) > 0));

[distance,nearestPoint,awayDirection] = distanceToRectangleObstacle( ...
    [30 45],config.obstacles(1,:),0);
assert(abs(distance-10) < 1e-12);
assert(norm(nearestPoint-[40 45]) < 1e-12);
assert(norm(awayDirection-[-1 0]) < 1e-12);
[insideDistance,insidePoint,insideDirection] = ...
    distanceToRectangleObstacle([45 45],config.obstacles(1,:),0);
assert(abs(insideDistance-5) < 1e-12);
assert(norm(insidePoint-[40 45]) < 1e-12);
assert(norm(insideDirection-[1 0]) < 1e-12);

[~,farDetails] = computeImprovedAPFDirection( ...
    [10 10],[90 90],config.obstacles,config);
assert(farDetails.activeObstacleCount == 0);

%% Analytic repulsive force against a finite-difference gradient.
finiteDifferencePoints = [30 50; 60 60; 20 70];
h = 1e-6;
maximumGradientError = 0;
for i = 1:size(finiteDifferencePoints,1)
    point = finiteDifferencePoints(i,:);
    [~,details] = computeImprovedAPFDirection( ...
        point,[90 90],config.obstacles,config);
    numericalGradient = zeros(1,2);
    for dimension = 1:2
        offset = zeros(1,2);
        offset(dimension) = h;
        plus = computeImprovedAPFRepulsivePotential( ...
            point+offset,[90 90],config.obstacles,config);
        minus = computeImprovedAPFRepulsivePotential( ...
            point-offset,[90 90],config.obstacles,config);
        numericalGradient(dimension) = (plus-minus)/(2*h);
    end
    maximumGradientError = max(maximumGradientError, ...
        norm(details.repulsiveForce+numericalGradient));
end
assert(maximumGradientError < 1e-4);

%% Both trees must use opposite APF targets.
[~,startTargetDetails] = computeImprovedAPFDirection( ...
    [50 50],[90 90],zeros(0,4),config);
[~,goalTargetDetails] = computeImprovedAPFDirection( ...
    [50 50],[10 10],zeros(0,4),config);
assert(dot(startTargetDetails.attractiveForce,[40 40]) > 0);
assert(dot(goalTargetDetails.attractiveForce,[-40 -40]) > 0);
assert(startTargetDetails.attractiveForce(1) > 0 && ...
    goalTargetDetails.attractiveForce(1) < 0);

%% Greedy simplification and B-spline endpoint/safety checks.
pathWithRedundancy = [
    10 50
    30 50
    30 10
    70 10
    70 50
    90 50];
simplifyObstacles = [40 20 20 40];
simpleConfig = defaultRRTConfig();
simpleConfig.start = pathWithRedundancy(1,:);
simpleConfig.goal = pathWithRedundancy(end,:);
simpleConfig.obstacles = simplifyObstacles;
[simplifiedPath,simplifyStats] = greedySimplifyPath( ...
    pathWithRedundancy,@(p1,p2) collisionFree( ...
    p1,p2,simplifyObstacles,0));
assert(size(simplifiedPath,1) < size(pathWithRedundancy,1));
assert(simplifyStats.collisionChecks > 0);
assert(validatePath(simplifiedPath,simpleConfig));

openConfig = defaultRRTConfig();
openConfig.obstacles = zeros(0,4);
controlPoints = [10 10; 35 20; 65 75; 90 90];
[smoothedPath,usedDegree] = generateBSplinePath(controlPoints,3,200);
[smoothedValid,smoothingChecks] = validateSmoothedPath( ...
    smoothedPath,openConfig);
assert(usedDegree == 3);
assert(all(isfinite(smoothedPath(:))));
assert(norm(smoothedPath(1,:)-controlPoints(1,:)) < 1e-12);
assert(norm(smoothedPath(end,:)-controlPoints(end,:)) < 1e-12);
assert(smoothedValid && smoothingChecks == 199);

%% Main paper-mode run on the shared 2-D map.
runConfig = defaultRRTConfig();
runConfig.showAnimation = false;
runConfig.apfPaperMode = true;
runConfig.optimizeIterations = 0;
runConfig.stepSize = 1;
runConfig.maxIterations = 2000;
runResult = APF_Bi_RRT_star(runConfig);
assert(runResult.success);
assert(runResult.iterations == runResult.firstSolutionIteration);
assert(runResult.optimizationIterationsExecuted == 0);
assert(strcmp(char(runResult.apfActualConnectionMode),'nearest'));
assert(validatePath(runResult.rawPath,runConfig));
assert(validatePath(runResult.simplifiedPath,runConfig));
assert(runResult.smoothedPathValid);
assert(validatePath(runResult.path,runConfig));
assert(runResult.rawPathLength >= runResult.simplifiedPathLength-1e-8);
assert(runResult.searchCollisionChecks > 0);
assert(runResult.postProcessCollisionChecks > 0);
assert(runResult.startTreeNodeCount > 1 && runResult.goalTreeNodeCount > 1);
assertTreeValid(runResult.startTree,runConfig);
assertTreeValid(runResult.goalTree,runConfig);

%% Fair raw-path comparison against Bi-RRT* with the same seed/config.
comparisonConfig = runConfig;
comparisonConfig.optimizeIterations = 0;
biResult = Bi_RRT_star(comparisonConfig);
assert(biResult.success);
assert(validatePath(biResult.path,comparisonConfig));
assert(validatePath(runResult.rawPath,comparisonConfig));
% The APF-smoothed result is intentionally not compared to Bi-RRT* raw.
assert(isfinite(runResult.rawPathLength) && isfinite(biResult.pathLength));

%% Extended mode: exactly the requested extra iterations after first path.
extendedConfig = runConfig;
extendedConfig.apfPaperMode = false;
extendedConfig.apfPaperConnectionMode = 'best_near';
extendedConfig.optimizeIterations = 100;
extendedResult = APF_Bi_RRT_star(extendedConfig);
assert(extendedResult.success);
assert(extendedResult.iterations-extendedResult.firstSolutionIteration == 100);
assert(extendedResult.optimizationIterationsRequested == 100);
assert(extendedResult.optimizationIterationsExecuted == 100);
assert(strcmp(extendedResult.apfActualConnectionMode,'best_near'));
assert(validatePath(extendedResult.path,extendedConfig));

%% Paper-scale and project-normalized configurations remain callable.
paperConfig = defaultRRTConfig();
paperConfig.showAnimation = false;
paperConfig.apfPaperMode = true;
paperConfig.mapBounds = [0 150 0 150];
paperConfig.start = [20 20];
paperConfig.goal = [120 120];
paperConfig.obstacles = [55 45 10 10];
paperConfig.stepSize = 8;
paperConfig.nearRadiusMax = 10;
paperConfig.maxIterations = 2000;
paperResult = APF_Bi_RRT_star(paperConfig);
assert(paperResult.success);
assert(paperResult.iterations == paperResult.firstSolutionIteration);
assert(validatePath(paperResult.path,paperConfig));

fprintf(['APF-Bi-RRT* helper, gradient, simplification, smoothing, ', ...
    'tree, paper-mode, extended-mode, and scale tests passed.\n']);
fprintf(['Project-normalized: first=%d raw=%.6f simplified=%.6f ', ...
    'smoothed=%.6f final=%.6f checks=%d+%d\n'], ...
    runResult.firstSolutionIteration,runResult.rawPathLength, ...
    runResult.simplifiedPathLength,runResult.smoothedPathLength, ...
    runResult.pathLength,runResult.searchCollisionChecks, ...
    runResult.postProcessCollisionChecks);
fprintf(['Fair raw comparison: APF raw=%.6f, Bi-RRT* raw=%.6f; ', ...
    'extended final=%.6f\n'],runResult.rawPathLength,biResult.pathLength, ...
    extendedResult.pathLength);
fprintf('Paper-scale test: first=%d path=%.6f\n', ...
    paperResult.firstSolutionIteration,paperResult.pathLength);

end

function assertTreeValid(tree,config)
assert(size(tree.nodes,2) == 2);
assert(numel(tree.parent) == size(tree.nodes,1));
assert(numel(tree.cost) == size(tree.nodes,1));
assert(tree.parent(1) == 0);
assert(abs(tree.cost(1)) < 1e-12);
for nodeIndex = 2:size(tree.nodes,1)
    parentIndex = tree.parent(nodeIndex);
    assert(parentIndex >= 1 && parentIndex < nodeIndex);
    expectedCost = tree.cost(parentIndex)+ ...
        norm(tree.nodes(nodeIndex,:)-tree.nodes(parentIndex,:));
    assert(abs(tree.cost(nodeIndex)-expectedCost) < 1e-8);
    assert(collisionFree(tree.nodes(parentIndex,:),tree.nodes(nodeIndex,:), ...
        config.obstacles,config.clearance));
end
end
