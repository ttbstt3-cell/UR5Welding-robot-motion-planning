function result = initRRTResult(algorithm,config)
% initRRTResult  Create the common result structure for 2-D planners.

result = struct();
result.algorithm = algorithm;
result.success = false;
result.path = [];
result.firstPath = [];
result.pathLength = NaN;
result.firstPathLength = NaN;
result.planningTime = NaN;
result.firstSolutionTime = NaN;
result.iterations = 0;
result.firstSolutionIteration = NaN;
result.optimizationIterationsRequested = config.optimizeIterations;
result.optimizationIterationsExecuted = 0;
result.nodeCount = 0;
result.collisionChecks = 0;
% Optional benchmark diagnostic: total collision calls observed when the
% first feasible path was found.  Planners that expose this field set it
% immediately after first-path validation; otherwise it remains NaN.
result.collisionChecksAtFirstSolution = NaN;
result.rewireCount = 0;
result.seed = config.seed;
result.config = config;

end
