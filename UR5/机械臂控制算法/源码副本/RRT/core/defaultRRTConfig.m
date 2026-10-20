function config = defaultRRTConfig()
% defaultRRTConfig  Shared 2-D benchmark defaults for all planners.

config = struct();

config.start = [10 10];
config.goal = [90 90];
config.obstacles = [
    25  0  15 55
    55 45  15 55
];
config.mapBounds = [0 100 0 100];

config.clearance = 0;
config.stepSize = 4;
config.maxIterations = 5000;
config.seed = 1;
config.showAnimation = false;
config.animationPause = 0.003;
config.optimizeIterations = 2000;

config.goalThreshold = 5;
config.goalBias = 0.05;
config.qDepth = 2;

% GD-RRT* reproduction controls.
% Paper mode returns immediately after the first feasible path, as in
% Algorithm 1.  Extended mode is only for project-internal optimization.
config.gdPaperMode = true;
config.gdSamplingInterpretation = 'paper_literal';
config.gdGaussianMaxRetries = 100;

% Classified-RRT* controls.
config.crrtBacktrackDepth = 2;
config.crrtOffsetAttempts = 1;
config.crrtPaperMode = false;
config.crrtPaperTotalIterations = 1000;
config.crrtStoreOffsetPoints = false;

% Improved APF-Bi-RRT* controls. The supplied thesis gives
% katt_min=0.2, lambda=0.05, krep=0.1, and d0=15 in Table 3.1.
% pMax/pMin, the nonlinear bias exponent, the repulsive exponent a, and
% the B-spline degree are not numerically specified in the extracted text,
% so they remain explicit engineering parameters.
config.apfPaperMode = true;
config.apfGoalBiasMax = 0.30;
config.apfGoalBiasMin = 0.05;
config.apfGoalBiasExponent = 1.0;
config.apfKattMin = 0.2;
config.apfLambda = 0.05;
config.apfKrep = 0.1;
config.apfInfluenceDistance = 15;
config.apfRepulsiveExponent = 2.0;
config.apfForceTolerance = 1e-12;
config.apfPaperConnectionMode = 'nearest';
config.bsplineDegree = 3;
config.bsplineSamples = 200;

% RRT* near-neighbour radius:
% r_n = min(nearGamma*sqrt(log(n)/n), nearRadiusMax)
config.nearGamma = 30;
config.nearRadiusMax = 25;

end
