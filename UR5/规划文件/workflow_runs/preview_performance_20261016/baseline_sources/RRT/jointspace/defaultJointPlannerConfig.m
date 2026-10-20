function config = defaultJointPlannerConfig()
%defaultJointPlannerConfig Phase 12A offline UR5 planner defaults.

config = struct();
config.start = zeros(1,6);
config.goal = zeros(1,6);
config.robot = [];
config.environment = [];
config.selfCollisionSkipPairs = {};
config.environmentCollisionPolicy = [];
config.jointLimits = [];
config.jointWeights = ones(1,6);
config.maxJointStepRad = deg2rad(12);
config.jointEdgeResolutionRad = deg2rad(2);
config.maxIterations = 3000;
config.maxPlanningTimeSec = Inf;
config.seed = 1;
config.goalBias = 0.20;
config.anytime = false;
config.continueAfterFirstSolution = false;
config.checkpointTimesSec = zeros(1,0);
config.goalThresholdRad = deg2rad(12);
config.nearGamma = 2.0;
config.nearRadiusMaxRad = deg2rad(60);
config.rewire = true;
config.classificationGoalTree = true;
config.useOffset = false;
config.useBacktracking = false;
config.useSequentialRewire = false;
config.offsetAttempts = 8;
config.offsetRadiusScale = 1.0;
config.backtrackDepth = 2;
% Optional strict validator called for every internally valid candidate.
% Signature: [pass,detail] = f(path,abortFcn).
config.candidateValidatorFcn = [];
end
