function test_phase7_gd_rrt_star()
% test_phase7_gd_rrt_star  Verification suite for Phase 7 GD-RRT*.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

assert(exist(fullfile(projectRoot,'GD_RRT_star.m'),'file') == 2);
assert(exist(fullfile(projectRoot,'planners','GD_RRT_star_impl.m'),'file') == 2);

%% A. ParamUpdate thresholds.
baseStep = 4;
state = updateGDRRTParameters(0.0,baseStep);
assert(~state.alpha && state.currentStep == 2*baseStep && state.currentDepth == 1);
state = updateGDRRTParameters(0.2,baseStep);
assert(~state.alpha && state.currentStep == 2*baseStep && state.currentDepth == 1);
state = updateGDRRTParameters(0.3,baseStep);
assert(~state.alpha && state.currentStep == baseStep && state.currentDepth == 2);
state = updateGDRRTParameters(0.5,baseStep);
assert(~state.alpha && state.currentStep == baseStep && state.currentDepth == 2);
state = updateGDRRTParameters(0.6,baseStep);
assert(state.alpha && state.currentStep == baseStep && state.currentDepth == 3);
state = updateGDRRTParameters(1.0,baseStep);
assert(state.alpha && state.currentStep == baseStep && state.currentDepth == 3);

%% B. pcf is collision frequency: pcf=(10-p)/10.
windows = {
    true(1,10), ...
    [true(1,8),false(1,2)], ...
    [true(1,5),false(1,5)], ...
    [true(1,4),false(1,6)], ...
    false(1,10)};
expectedPCF = [0.0 0.2 0.5 0.6 1.0];
for i = 1:numel(windows)
    assert(abs(computeGDRRTPCF(windows{i})-expectedPCF(i)) < 1e-12);
end

%% C. Eq.(4)--Eq.(14) Gaussian parameters.
localNodes = [-2 1; 3 4; 1 8; -1 5];
pcf = 0.6;
currentStep = 4;
params = computeGDRRTGaussianParameters(localNodes,currentStep,pcf);
sigmaX = 0.5*currentStep*exp(pcf);
sigmaY = currentStep*exp(pcf);
assert(abs(params.xgmax-3) < 1e-12);
assert(abs(params.xgmin+2) < 1e-12);
assert(abs(params.ygmax-8) < 1e-12);
assert(abs(params.mu11-(3+sigmaX)) < 1e-12);
assert(abs(params.mu12-(-2-sigmaX)) < 1e-12);
assert(abs(params.sigma11-sigmaX) < 1e-12);
assert(abs(params.sigma12-sigmaX) < 1e-12);
assert(abs(params.mu21-(8+sigmaY)) < 1e-12);
assert(abs(params.mu22-(8-sigmaY)) < 1e-12);
assert(abs(params.sigma21-sigmaY) < 1e-12);
assert(abs(params.sigma22-sigmaY) < 1e-12);
assert(abs(params.l1-(1-pcf^2)) < 1e-12);
assert(abs(params.l2-pcf^2) < 1e-12);
assert(abs(params.l1+params.l2-1) < 1e-12);

%% D. Gaussian frame round trip.
frame = computeGaussianFrame([3 7],[91 83]);
rng(17,'twister');
testLocal = randn(100,2)*12;
testWorld = gaussianLocalToWorld(frame,testLocal);
roundTrip = gaussianWorldToLocal(frame,testWorld);
assert(max(abs(roundTrip-testLocal),[],'all') < 1e-10);

%% E. Gaussian mixture statistics.
rng(23,'twister');
for pcf = [0.6 0.8 1.0]
    params = computeGDRRTGaussianParameters(localNodes,4,pcf);
    forwardCount = 0;
    backwardCount = 0;
    xFirstCount = 0;
    sampleCount = 10000;
    for sampleIndex = 1:sampleCount
        [~,sampleInfo] = sampleGDRRTGaussian(frame,params);
        if sampleInfo.yComponent == 1
            forwardCount = forwardCount+1;
        else
            backwardCount = backwardCount+1;
        end
        if sampleInfo.xComponent == 1
            xFirstCount = xFirstCount+1;
        end
    end
    assert(abs(backwardCount/sampleCount-pcf^2) < 0.03);
    assert(abs(xFirstCount/sampleCount-0.5) < 0.03);
    assert(forwardCount+backwardCount == sampleCount);
end

%% F. Dynamic state switching and real Gaussian activation.
config = defaultRRTConfig();
config.showAnimation = false;
config.gdPaperMode = true;
config.optimizeIterations = 1000;
config.maxIterations = 5000;
config.seed = 1;
gdResult = GD_RRT_star(config);
assert(gdResult.success);
assert(gdResult.gdPaperMode);
assert(gdResult.iterations == gdResult.firstSolutionIteration);
assert(gdResult.optimizationIterationsExecuted == 0);
assert(strcmp(gdResult.gdSamplingInterpretation,'paper_literal'));
assert(gdResult.gaussianSampleCount > 0);
assert(any(gdResult.alphaHistory));
assert(numel(unique(gdResult.depthHistory)) >= 2);
assert(numel(gdResult.pcfHistory) == numel(gdResult.depthHistory));
assert(numel(gdResult.pcfHistory) == numel(gdResult.stepHistory));
assert(numel(gdResult.pcfHistory) == numel(gdResult.alphaHistory));
assert(all(gdResult.pcfHistory >= 0 & gdResult.pcfHistory <= 1));

% The paper-literal TargetSample branch must ignore goalBias entirely.
noBiasConfig = config;
noBiasConfig.goalBias = 0;
noBiasResult = GD_RRT_star(noBiasConfig);
highBiasConfig = config;
highBiasConfig.goalBias = 0.9;
highBiasResult = GD_RRT_star(highBiasConfig);
assert(isequal(noBiasResult.firstPath,highBiasResult.firstPath));
assert(noBiasResult.firstSolutionIteration == ...
    highBiasResult.firstSolutionIteration);

%% Extended mode is explicitly separate from paper mode.
extendedConfig = config;
extendedConfig.gdPaperMode = false;
extendedConfig.optimizeIterations = 100;
extendedResult = GD_RRT_star(extendedConfig);
assert(extendedResult.success);
assert(extendedResult.iterations-extendedResult.firstSolutionIteration == 100);
assert(extendedResult.optimizationIterationsExecuted == 100);

%% G. Initial-path comparison under the paper-literal stopping rule.
comparisonConfig = defaultRRTConfig();
comparisonConfig.showAnimation = false;
comparisonConfig.gdPaperMode = true;
comparisonConfig.optimizeIterations = 0;
comparisonConfig.seed = 1;
comparisonConfig.maxIterations = 5000;
connectResult = RRT_connect(comparisonConfig);
starResult = RRT_star(comparisonConfig);
qResult = Q_RRT_star(comparisonConfig);
gdComparisonResult = GD_RRT_star(comparisonConfig);
assert(connectResult.success && starResult.success && qResult.success && gdComparisonResult.success);
assert(validatePath(connectResult.path,comparisonConfig));
assert(validatePath(starResult.path,comparisonConfig));
assert(validatePath(qResult.path,comparisonConfig));
assert(validatePath(gdComparisonResult.path,comparisonConfig));

%% H. Four maps: functional environment variation, not a benchmark sweep.
mapConfigs = cell(1,4);
mapConfigs{1} = comparisonConfig;
mapConfigs{2} = comparisonConfig;
mapConfigs{2}.obstacles = [
    18  0  8 38
    18 62  8 38
    42 20  8 80
    68  0  8 38
    68 62  8 38];
mapConfigs{3} = comparisonConfig;
mapConfigs{3}.obstacles = [
    22  0 12 46
    22 64 12 36
    46 18 12 82
    70  0 12 46
    70 64 12 36];
mapConfigs{4} = comparisonConfig;
mapConfigs{4}.obstacles = [
    25 20 50  8
    25 20  8 55
    67 20  8 55];

mapResults = cell(1,4);
for mapIndex = 1:4
    mapConfig = mapConfigs{mapIndex};
    mapConfig.maxIterations = 6000;
    mapConfig.seed = mapIndex;
    mapResults{mapIndex} = GD_RRT_star(mapConfig);
    assert(mapResults{mapIndex}.success);
    assert(validatePath(mapResults{mapIndex}.path,mapConfig));
end

%% I. Tree, cost, cycle, path, and descendant consistency.
assertTreeValid(gdResult.tree);
assert(validatePath(gdResult.path,config));
assert(abs(gdResult.pathLength-pathLength(gdResult.path)) < 1e-9);

fprintf('GD-RRT* ParamUpdate and pcf tests passed.\n');
fprintf('GD-RRT* Gaussian parameter, frame, and mixture tests passed.\n');
fprintf('GD-RRT* dynamic state and extended-mode tests passed.\n');
fprintf(['Initial-path comparison: RRT-Connect=%d/%d %.6f, ', ...
    'RRT*=%d/%d %.6f, Q-RRT*=%d/%d %.6f, GD-RRT*=%d/%d %.6f\n'], ...
    connectResult.success,connectResult.firstSolutionIteration, ...
    connectResult.firstPathLength,starResult.success, ...
    starResult.firstSolutionIteration,starResult.firstPathLength, ...
    qResult.success,qResult.firstSolutionIteration,qResult.firstPathLength, ...
    gdComparisonResult.success,gdComparisonResult.firstSolutionIteration, ...
    gdComparisonResult.firstPathLength);

for mapIndex = 1:4
    mapResult = mapResults{mapIndex};
    fprintf(['Map %d: success=%d first=%d firstLength=%.6f ', ...
        'gaussian=%d fallback=%d states=%d\n'], ...
        mapIndex,mapResult.success,mapResult.firstSolutionIteration, ...
        mapResult.firstPathLength,mapResult.gaussianSampleCount, ...
        mapResult.gaussianFallbackCount,numel(mapResult.pcfHistory));
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
