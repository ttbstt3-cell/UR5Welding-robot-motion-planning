function test_phase10_wrappers()
% test_phase10_wrappers  Verify Basic/Bi-RRT benchmark adapters.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

config = defaultRRTConfig();
config.showAnimation = false;
config.stepSize = 4;
config.optimizeIterations = 0;

basicResult = RRT_basic(config);
assert(basicResult.success);
assert(basicResult.firstSolutionIteration == 646);
assert(abs(basicResult.pathLength-264.808321284) < 1e-9);
assert(basicResult.nodeCount == 470);
assert(isfield(basicResult,'collisionChecks'));
assert(isfield(basicResult,'firstSolutionTime'));
assert(isfield(basicResult,'firstPathLength'));
assert(validatePath(basicResult.path,config));

biResult = Bi_RRT(config);
assert(biResult.success);
assert(biResult.firstSolutionIteration == 87);
assert(abs(biResult.pathLength-194.326188895) < 1e-9);
assert(biResult.nodeCount == 64);
assert(isfield(biResult,'collisionChecks'));
assert(isfield(biResult,'firstSolutionTime'));
assert(isfield(biResult,'firstPathLength'));
assert(validatePath(biResult.path,config));

fprintf('Basic RRT and Bi-RRT adapters preserve seed=1 path behavior.\n');
end
