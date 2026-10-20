function outputs = run_phase10a_pilot(runOptimizationPilot,seedsInput, ...
        outputPrefix)
% run_phase10a_pilot  Phase 10A fixed-map, 20-seed pilot benchmark.
%
% By default this runs the explicitly scoped Phase 10A Pilot:
% Experiment A, 4 maps x 20 seeds x 10 planners = 800 runs, plus the
% separate Experiment C post-processing check.  Pass true explicitly to
% append the six-planner Experiment B optimization pilot.

if nargin < 1
    runOptimizationPilot = false;
end
if nargin < 2 || isempty(seedsInput)
    seedsInput = 1:20;
end
if nargin < 3 || isempty(outputPrefix)
    outputPrefix = 'pilot';
end

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

maps = phase10Maps();
seeds = seedsInput;
maxIterations = 5000;
commonStepSize = 1;
commonNearRadiusMax = 25;

allAlgorithms = algorithmDefinitions();
optimizationAlgorithms = allAlgorithms([4 5 6 7 9 10]);

resultsDirectory = fullfile(projectRoot,'results');
if strcmp(outputPrefix,'pilot')
    figureDirectory = fullfile(resultsDirectory,'benchmark_figures');
else
    figureDirectory = fullfile(resultsDirectory,'benchmark_figures',outputPrefix);
end
if ~exist(resultsDirectory,'dir'), mkdir(resultsDirectory); end
if ~exist(figureDirectory,'dir'), mkdir(figureDirectory); end

if strcmp(outputPrefix,'pilot')
    rawResultsFile = fullfile(resultsDirectory,'pilot_raw_results.csv');
    summaryFile = fullfile(resultsDirectory,'benchmark_summary.csv');
    postResultsFile = fullfile(resultsDirectory, ...
        'phase10a_postprocess_results.csv');
    postSummaryFile = fullfile(resultsDirectory, ...
        'phase10a_postprocess_summary.csv');
    convergenceFile = fullfile(resultsDirectory, ...
        'phase10a_pilot_convergence.mat');
else
    rawResultsFile = fullfile(resultsDirectory, ...
        [outputPrefix '_raw_results.csv']);
    summaryFile = fullfile(resultsDirectory,[outputPrefix '_summary.csv']);
    postResultsFile = fullfile(resultsDirectory, ...
        [outputPrefix '_postprocess_results.csv']);
    postSummaryFile = fullfile(resultsDirectory, ...
        [outputPrefix '_postprocess_summary.csv']);
    convergenceFile = fullfile(resultsDirectory, ...
        [outputPrefix '_convergence.mat']);
end

% Warm-up results are excluded from every output file.
warmupConfig = makeConfig(maps(1),1,100,commonStepSize, ...
    commonNearRadiusMax,'Warmup');
for algorithmIndex = 1:numel(allAlgorithms)
    try
        allAlgorithms(algorithmIndex).handle(warmupConfig);
    catch warmupError
        warning('Phase10:WarmupFailed','%s warm-up failed: %s', ...
            allAlgorithms(algorithmIndex).name,warmupError.message);
    end
end

totalRunCount = numel(maps)*numel(seeds)*numel(allAlgorithms);
if runOptimizationPilot
    totalRunCount = totalRunCount + numel(maps)*numel(seeds)* ...
        numel(optimizationAlgorithms);
end
rawRows = repmat(emptyRawRow(),totalRunCount,1);
rowIndex = 0;
convergenceRecords = repmat(emptyConvergenceRecord(), ...
    numel(maps)*numel(seeds)*numel(optimizationAlgorithms),1);
convergenceIndex = 0;
% APF native + one common Bi-RRT*/APF post-processing row per success.
postRows = repmat(emptyPostRow(),numel(maps)*numel(seeds)*4,1);
postIndex = 0;

pilotTimer = tic;
[rawRows,rowIndex,convergenceRecords,convergenceIndex,postRows,postIndex] = ...
    runExperiment(allAlgorithms,'FirstSolution',maps,seeds,maxIterations, ...
    commonStepSize,commonNearRadiusMax,rawRows,rowIndex, ...
    convergenceRecords,convergenceIndex,postRows,postIndex);
if runOptimizationPilot
    [rawRows,rowIndex,convergenceRecords,convergenceIndex,postRows,postIndex] = ...
        runExperiment(optimizationAlgorithms,'FixedOptimization1000', ...
        maps,seeds,maxIterations,commonStepSize,commonNearRadiusMax, ...
        rawRows,rowIndex,convergenceRecords,convergenceIndex,postRows,postIndex);
end
pilotElapsed = toc(pilotTimer);

rawRows = rawRows(1:rowIndex);
convergenceRecords = convergenceRecords(1:convergenceIndex);
postRows = postRows(1:postIndex);
rawTable = struct2table(rawRows);
summaryTable = buildSummaryTable(rawTable);
postTable = struct2table(postRows);
postSummaryTable = buildPostSummaryTable(postTable);

writetable(rawTable,rawResultsFile);
writetable(summaryTable,summaryFile);
writetable(postTable,postResultsFile);
writetable(postSummaryTable,postSummaryFile);
save(convergenceFile, ...
    'convergenceRecords','maps','seeds');
figureResults = createBenchmarkFigures(rawTable,summaryTable, ...
    postTable,convergenceRecords,figureDirectory);

outputs = struct('rawTable',rawTable,'summaryTable',summaryTable, ...
    'postTable',postTable,'postSummaryTable',postSummaryTable, ...
    'convergenceRecords',convergenceRecords,'maps',maps,'seeds',seeds, ...
    'figureResults',{figureResults},'optimizationPilotRun', ...
    logical(runOptimizationPilot),'totalRunCount',height(rawTable), ...
    'pilotElapsedSeconds',pilotElapsed,'outputPrefix',outputPrefix);

fprintf('\n%s first-solution benchmark finished: %d planner runs in %.3f s.\n', ...
    outputPrefix,height(rawTable),pilotElapsed);
fprintf('Raw results: %s\n',rawResultsFile);
fprintf('Summary: %s\n',summaryFile);
fprintf('Post-processing: %s\n',postSummaryFile);
printPilotAcceptance(rawTable,pilotElapsed);
end

function algorithms = algorithmDefinitions()
algorithms = struct('name',{},'handle',{});
algorithms(end+1) = struct('name','Basic RRT','handle',@RRT_basic);
algorithms(end+1) = struct('name','Bi-RRT','handle',@Bi_RRT);
algorithms(end+1) = struct('name','RRT-Connect','handle',@RRT_connect);
algorithms(end+1) = struct('name','RRT*','handle',@RRT_star);
algorithms(end+1) = struct('name','Bi-RRT*','handle',@Bi_RRT_star);
algorithms(end+1) = struct('name','Informed RRT*','handle',@Informed_RRT_star);
algorithms(end+1) = struct('name','Q-RRT*','handle',@Q_RRT_star);
algorithms(end+1) = struct('name','GD-RRT*','handle',@GD_RRT_star);
algorithms(end+1) = struct('name','Classified-RRT*','handle',@Classified_RRT_star);
algorithms(end+1) = struct('name','APF-Bi-RRT*','handle',@APF_Bi_RRT_star);
end

function config = makeConfig(map,seed,maxIterations,stepSize,nearRadiusMax,experiment)
config = defaultRRTConfig();
config.mapBounds = map.mapBounds;
config.start = map.start;
config.goal = map.goal;
config.obstacles = map.obstacles;
config.clearance = map.clearance;
config.seed = seed;
config.showAnimation = false;
config.animationPause = 0;
config.maxIterations = maxIterations;
config.stepSize = stepSize;
config.nearRadiusMax = nearRadiusMax;
config.optimizeIterations = 0;
config.gdPaperMode = true;
config.crrtPaperMode = false;
config.apfPaperMode = true;
config.apfPaperConnectionMode = 'nearest';
config.benchmarkExperiment = experiment;
end

function [rawRows,rowIndex,convergenceRecords,convergenceIndex, ...
        postRows,postIndex] = runExperiment(algorithms,experiment,maps,seeds, ...
        maxIterations,stepSize,nearRadiusMax,rawRows,rowIndex, ...
        convergenceRecords,convergenceIndex,postRows,postIndex)
fprintf('Starting %s: %d maps x %d seeds x %d planners.\n', ...
    experiment,numel(maps),numel(seeds),numel(algorithms));
for mapIndex = 1:numel(maps)
    for seedIndex = 1:numel(seeds)
        seed = seeds(seedIndex);
        for algorithmIndex = 1:numel(algorithms)
            algorithm = algorithms(algorithmIndex);
            config = makeConfig(maps(mapIndex),seed,maxIterations, ...
                stepSize,nearRadiusMax,experiment);
            if strcmp(experiment,'FixedOptimization1000')
                config.optimizeIterations = 1000;
                if strcmp(algorithm.name,'APF-Bi-RRT*')
                    config.apfPaperMode = false;
                    config.apfPaperConnectionMode = 'best_near';
                end
            end
            runTimer = tic;
            try
                result = algorithm.handle(config);
                runError = '';
            catch plannerError
                result = makeFailureResult(algorithm.name,seed,config, ...
                    plannerError.message);
                runError = plannerError.message;
            end
            wallElapsed = toc(runTimer);
            rowIndex = rowIndex+1;
            rawRows(rowIndex) = makeRawRow(result,algorithm.name, ...
                experiment,maps(mapIndex),seed,config,wallElapsed,runError);
            if strcmp(experiment,'FixedOptimization1000')
                convergenceIndex = convergenceIndex+1;
                convergenceRecords(convergenceIndex) = ...
                    makeConvergenceRecord(result,algorithm.name, ...
                    maps(mapIndex).id,seed);
            end
            if strcmp(experiment,'FirstSolution') && ...
                    getLogical(result,'success',false)
                if strcmp(algorithm.name,'APF-Bi-RRT*')
                    postIndex = postIndex+1;
                    postRows(postIndex) = makePostRowFromResult( ...
                        result,algorithm.name,maps(mapIndex),seed,'APF-native');
                end
                if strcmp(algorithm.name,'Bi-RRT*') || ...
                        strcmp(algorithm.name,'APF-Bi-RRT*')
                    % Use the first feasible polyline for both planners so
                    % the common post-processing comparison is independent
                    % of each planner's own final-path packaging.
                    commonPost = runCommonPostprocessing( ...
                        result.firstPath,config);
                    postIndex = postIndex+1;
                    postRows(postIndex) = makePostRowFromCommon(commonPost, ...
                        algorithm.name,maps(mapIndex),seed, ...
                        'Common Greedy+B-spline');
                end
            end
        end
        fprintf('%s map %d/%d seed %d/%d complete (%d rows).\n', ...
            experiment,mapIndex,numel(maps),seedIndex,numel(seeds),rowIndex);
    end
end
end

function result = makeFailureResult(algorithm,seed,config,errorMessage)
result = initRRTResult(algorithm,config);
result.success = false;
result.seed = seed;
result.errorMessage = errorMessage;
result.iterations = 0;
result.nodeCount = NaN;
result.collisionChecks = NaN;
result.rewireCount = NaN;
end

function row = emptyRawRow()
row = struct('algorithm','', 'experiment','', 'map',NaN, 'mapName','', ...
    'seed',NaN, 'success',false, 'pathCollisionFree',false, ...
    'firstSolutionIteration',NaN, 'firstSolutionTime_ms',NaN, ...
    'firstPathLength',NaN, 'finalIteration',NaN, ...
    'planningTime_ms',NaN, 'finalPathLength',NaN, 'nodeCount',NaN, ...
    'collisionChecks',NaN, 'coreCollisionChecks',NaN, ...
    'finalValidationCollisionChecks',NaN, 'rewireCount',NaN, ...
    'optimizationIterationsRequested',NaN, ...
    'optimizationIterationsExecuted',NaN, 'maxIterationsTruncated',false, ...
    'qDepth',NaN, 'ancestorChecks',NaN, 'ancestorAcceptedCount',NaN, ...
    'gdGaussianSampleCount',NaN, 'crrtOffsetSuccessCount',NaN, ...
    'crrtBacktrackChecks',NaN, 'crrtGoalTreeConnectionChecks',NaN, ...
    'apfGoalBiasedSampleCount',NaN, 'apfZeroForceCount',NaN, ...
    'apfPostProcessTime_ms',NaN, 'apfRawPathLength',NaN, ...
    'apfSimplifiedPathLength',NaN, 'apfSmoothedPathLength',NaN, ...
    'apfSmoothedPathValid',false, 'wallTime_ms',NaN, 'errorMessage','');
end

function row = makeRawRow(result,algorithm,experiment,map,seed,config, ...
        wallElapsed,runError)
row = emptyRawRow();
row.algorithm = algorithm;
row.experiment = experiment;
row.map = map.id;
row.mapName = map.name;
row.seed = seed;
row.wallTime_ms = wallElapsed*1000;
row.errorMessage = runError;
row.success = getLogical(result,'success',false);
row.firstSolutionIteration = getNumeric(result,'firstSolutionIteration',NaN);
row.firstSolutionTime_ms = getNumeric(result,'firstSolutionTime',NaN)*1000;
row.firstPathLength = getNumeric(result,'firstPathLength',NaN);
row.finalIteration = getNumeric(result,'iterations',NaN);
row.finalPathLength = getNumeric(result,'pathLength',NaN);
row.nodeCount = getNumeric(result,'nodeCount',NaN);
row.rewireCount = getNumeric(result,'rewireCount',0);
row.optimizationIterationsRequested = getNumeric(result, ...
    'optimizationIterationsRequested',config.optimizeIterations);
row.optimizationIterationsExecuted = getNumeric(result, ...
    'optimizationIterationsExecuted',0);
if isfield(result,'searchPlanningTime')
    planningTime = result.searchPlanningTime;
else
    planningTime = getNumeric(result,'planningTime',NaN);
end
row.planningTime_ms = planningTime*1000;
row.collisionChecks = getNumeric(result,'collisionChecks',NaN);
if isfield(result,'searchCollisionChecks')
    row.coreCollisionChecks = result.searchCollisionChecks;
else
    row.coreCollisionChecks = row.collisionChecks;
end
row.finalValidationCollisionChecks = getNumeric(result, ...
    'finalValidationCollisionChecks',0);
row.qDepth = getNumeric(result,'qDepth',NaN);
row.ancestorChecks = getNumeric(result,'ancestorChecks',NaN);
row.ancestorAcceptedCount = getNumeric(result,'ancestorAcceptedCount',NaN);
row.gdGaussianSampleCount = getNumeric(result,'gaussianSampleCount',NaN);
row.crrtOffsetSuccessCount = getNumeric(result,'offsetSuccessCount',NaN);
row.crrtBacktrackChecks = getNumeric(result,'backtrackChecks',NaN);
row.crrtGoalTreeConnectionChecks = getNumeric(result, ...
    'goalTreeConnectionChecks',NaN);
row.apfGoalBiasedSampleCount = getNumeric(result,'goalBiasedSampleCount',NaN);
row.apfZeroForceCount = getNumeric(result,'apfZeroForceCount',NaN);
row.apfPostProcessTime_ms = getNumeric(result,'postProcessTime',0)*1000;
row.apfRawPathLength = getNumeric(result,'rawPathLength',NaN);
row.apfSimplifiedPathLength = getNumeric(result,'simplifiedPathLength',NaN);
row.apfSmoothedPathLength = getNumeric(result,'smoothedPathLength',NaN);
row.apfSmoothedPathValid = getLogical(result,'smoothedPathValid',false);
if row.success && isfield(result,'path') && ~isempty(result.path)
    row.pathCollisionFree = validatePath(result.path,config);
end
row.maxIterationsTruncated = (~row.success && ...
    row.finalIteration >= config.maxIterations) || ...
    (row.success && strcmp(experiment,'FixedOptimization1000') && ...
    row.optimizationIterationsExecuted < row.optimizationIterationsRequested && ...
    row.finalIteration >= config.maxIterations);
end

function value = getNumeric(result,fieldName,defaultValue)
if isfield(result,fieldName) && ~isempty(result.(fieldName)) && ...
        isscalar(result.(fieldName)) && isnumeric(result.(fieldName))
    value = double(result.(fieldName));
else
    value = defaultValue;
end
end

function value = getLogical(result,fieldName,defaultValue)
if isfield(result,fieldName) && ~isempty(result.(fieldName)) && ...
        isscalar(result.(fieldName))
    value = logical(result.(fieldName));
else
    value = defaultValue;
end
end

function record = emptyConvergenceRecord()
record = struct('algorithm','', 'map',NaN, 'seed',NaN, ...
    'firstSolutionIteration',NaN, 'firstPathLength',NaN, ...
    'bestCostHistory',zeros(0,1));
end

function record = makeConvergenceRecord(result,algorithm,mapID,seed)
record = emptyConvergenceRecord();
record.algorithm = algorithm;
record.map = mapID;
record.seed = seed;
record.firstSolutionIteration = getNumeric(result,'firstSolutionIteration',NaN);
record.firstPathLength = getNumeric(result,'firstPathLength',NaN);
if isfield(result,'bestCostHistory')
    record.bestCostHistory = result.bestCostHistory;
end
end

function row = emptyPostRow()
row = struct('algorithm','', 'source','', 'map',NaN, 'mapName','', ...
    'seed',NaN, 'rawPathLength',NaN, 'rawNodeCount',NaN, ...
    'greedyPathLength',NaN, 'greedyNodeCount',NaN, ...
    'bsplinePathLength',NaN, 'bsplineSampleCount',NaN, ...
    'turningAngleSum_rad',NaN, 'maxTurningAngle_rad',NaN, ...
    'collisionFree',false, 'postProcessTime_ms',NaN, ...
    'postProcessCollisionChecks',NaN, 'bsplineDegreeUsed',NaN);
end

function row = makePostRowFromResult(result,algorithm,map,seed,source)
row = emptyPostRow();
row.algorithm = algorithm;
row.source = source;
row.map = map.id;
row.mapName = map.name;
row.seed = seed;
row.rawPathLength = getNumeric(result,'rawPathLength',NaN);
row.rawNodeCount = getNumeric(result,'rawNodeCount',NaN);
row.greedyPathLength = getNumeric(result,'simplifiedPathLength',NaN);
row.greedyNodeCount = getNumeric(result,'simplifiedNodeCount',NaN);
row.bsplinePathLength = getNumeric(result,'smoothedPathLength',NaN);
row.bsplineSampleCount = getNumeric(result,'smoothedSampleCount',NaN);
row.collisionFree = getLogical(result,'smoothedPathValid',false);
row.postProcessTime_ms = getNumeric(result,'postProcessTime',NaN)*1000;
row.postProcessCollisionChecks = getNumeric(result, ...
    'postProcessCollisionChecks',NaN);
row.bsplineDegreeUsed = getNumeric(result,'bsplineDegreeUsed',NaN);
[row.turningAngleSum_rad,row.maxTurningAngle_rad] = ...
    turningMetrics(result.rawPath);
end

function post = runCommonPostprocessing(rawPath,config)
post = struct('rawPathLength',pathLength(rawPath), ...
    'rawNodeCount',size(rawPath,1),'collisionChecks',0);
timer = tic;
[post.greedyPath,~] = greedySimplifyPath(rawPath,@checkedCollision);
post.greedyPathLength = pathLength(post.greedyPath);
post.greedyNodeCount = size(post.greedyPath,1);
[post.bsplinePath,post.bsplineDegreeUsed] = generateBSplinePath( ...
    post.greedyPath,config.bsplineDegree,config.bsplineSamples);
[post.collisionFree,~] = validateSmoothedPath( ...
    post.bsplinePath,config,@checkedCollision);
post.bsplinePathLength = pathLength(post.bsplinePath);
post.bsplineSampleCount = size(post.bsplinePath,1);
post.postProcessTime_ms = toc(timer)*1000;
[post.turningAngleSum_rad,post.maxTurningAngle_rad] = ...
    turningMetrics(post.bsplinePath);
    function free = checkedCollision(p1,p2)
        post.collisionChecks = post.collisionChecks+1;
        free = collisionFree(p1,p2,config.obstacles,config.clearance);
    end
end

function row = makePostRowFromCommon(post,algorithm,map,seed,source)
row = emptyPostRow();
row.algorithm = algorithm;
row.source = source;
row.map = map.id;
row.mapName = map.name;
row.seed = seed;
row.rawPathLength = post.rawPathLength;
row.rawNodeCount = post.rawNodeCount;
row.greedyPathLength = post.greedyPathLength;
row.greedyNodeCount = post.greedyNodeCount;
row.bsplinePathLength = post.bsplinePathLength;
row.bsplineSampleCount = post.bsplineSampleCount;
row.turningAngleSum_rad = post.turningAngleSum_rad;
row.maxTurningAngle_rad = post.maxTurningAngle_rad;
row.collisionFree = post.collisionFree;
row.postProcessTime_ms = post.postProcessTime_ms;
row.postProcessCollisionChecks = post.collisionChecks;
row.bsplineDegreeUsed = post.bsplineDegreeUsed;
end

function [sumAngles,maxAngle] = turningMetrics(path)
sumAngles = 0;
maxAngle = 0;
if size(path,1) < 3, return; end
directions = diff(path,1,1);
for i = 1:size(directions,1)-1
    firstNorm = norm(directions(i,:));
    secondNorm = norm(directions(i+1,:));
    if firstNorm < 1e-12 || secondNorm < 1e-12, continue; end
    cosine = dot(directions(i,:),directions(i+1,:))/(firstNorm*secondNorm);
    angle = acos(min(1,max(-1,cosine)));
    sumAngles = sumAngles+angle;
    maxAngle = max(maxAngle,angle);
end
end

function summaryTable = buildSummaryTable(rawTable)
algorithms = unique(rawTable.algorithm,'stable');
experiments = unique(rawTable.experiment,'stable');
mapIDs = unique(rawTable.map,'stable');
summaryRows = repmat(emptySummaryRow(), ...
    numel(algorithms)*numel(experiments)*numel(mapIDs),1);
rowIndex = 0;
for algorithmIndex = 1:numel(algorithms)
    for experimentIndex = 1:numel(experiments)
        for mapIndex = 1:numel(mapIDs)
            mask = strcmp(rawTable.algorithm,algorithms{algorithmIndex}) & ...
                strcmp(rawTable.experiment,experiments{experimentIndex}) & ...
                rawTable.map == mapIDs(mapIndex);
            rowIndex = rowIndex+1;
            summaryRows(rowIndex) = summarizeSubset(rawTable(mask,:), ...
                algorithms{algorithmIndex},experiments{experimentIndex}, ...
                mapIDs(mapIndex));
        end
    end
end
summaryTable = struct2table(summaryRows(1:rowIndex));
end

function row = emptySummaryRow()
row = struct('algorithm','', 'experiment','', 'map',NaN, ...
    'mapName','', 'nRuns',NaN, 'nSuccess',NaN, 'nFailure',NaN, ...
    'successRate',NaN, 'successRateCI95Low',NaN, ...
    'successRateCI95High',NaN, ...
    'firstSolutionTimeMean_ms',NaN, 'firstSolutionTimeStd_ms',NaN, ...
    'firstSolutionTimeMedian_ms',NaN, 'firstSolutionTimeIQR_ms',NaN, ...
    'firstSolutionTimeCI95Low_ms',NaN, ...
    'firstSolutionTimeCI95High_ms',NaN, ...
    'firstPathLengthMean',NaN, 'firstPathLengthStd',NaN, ...
    'firstPathLengthMedian',NaN, 'firstPathLengthIQR',NaN, ...
    'firstPathLengthCI95Low',NaN, 'firstPathLengthCI95High',NaN, ...
    'finalPathLengthMean',NaN, 'finalPathLengthStd',NaN, ...
    'finalPathLengthMedian',NaN, 'finalPathLengthIQR',NaN, ...
    'finalPathLengthCI95Low',NaN, 'finalPathLengthCI95High',NaN, ...
    'nodeCountMean',NaN, 'nodeCountMedian',NaN, ...
    'collisionChecksMean',NaN, 'collisionChecksMedian',NaN, ...
    'maxIterationsTruncatedCount',NaN);
end

function row = summarizeSubset(subset,algorithm,experiment,mapID)
row = emptySummaryRow();
row.algorithm = algorithm;
row.experiment = experiment;
row.map = mapID;
if isempty(subset), return; end
row.mapName = subset.mapName{1};
row.nRuns = height(subset);
row.nSuccess = sum(subset.success);
row.nFailure = row.nRuns-row.nSuccess;
[row.successRate,row.successRateCI95Low,row.successRateCI95High] = ...
    binomialSummary(row.nSuccess,row.nRuns);
[row.firstSolutionTimeMean_ms,row.firstSolutionTimeStd_ms, ...
    row.firstSolutionTimeMedian_ms,row.firstSolutionTimeIQR_ms, ...
    row.firstSolutionTimeCI95Low_ms,row.firstSolutionTimeCI95High_ms] = ...
    continuousSummary(subset.firstSolutionTime_ms(subset.success));
[row.firstPathLengthMean,row.firstPathLengthStd, ...
    row.firstPathLengthMedian,row.firstPathLengthIQR, ...
    row.firstPathLengthCI95Low,row.firstPathLengthCI95High] = ...
    continuousSummary(subset.firstPathLength(subset.success));
[row.finalPathLengthMean,row.finalPathLengthStd, ...
    row.finalPathLengthMedian,row.finalPathLengthIQR, ...
    row.finalPathLengthCI95Low,row.finalPathLengthCI95High] = ...
    continuousSummary(subset.finalPathLength(subset.success));
row.nodeCountMean = meanFinite(subset.nodeCount(subset.success));
row.nodeCountMedian = medianFinite(subset.nodeCount(subset.success));
row.collisionChecksMean = meanFinite(subset.collisionChecks(subset.success));
row.collisionChecksMedian = medianFinite(subset.collisionChecks(subset.success));
row.maxIterationsTruncatedCount = sum(subset.maxIterationsTruncated);
end

function [meanValue,stdValue,medianValue,iqrValue,ciLow,ciHigh] = ...
        continuousSummary(values)
values = values(isfinite(values));
if isempty(values)
    [meanValue,stdValue,medianValue,iqrValue,ciLow,ciHigh] = deal(NaN);
    return;
end
meanValue = mean(values);
stdValue = std(values,0);
medianValue = median(values);
quartiles = prctile(values,[25 75]);
iqrValue = quartiles(2)-quartiles(1);
rng(20260924,'twister');
bootstrapIndex = randi(numel(values),1000,numel(values));
bootstrapMeans = mean(values(bootstrapIndex),2);
ci = prctile(bootstrapMeans,[2.5 97.5]);
ciLow = ci(1);
ciHigh = ci(2);
end

function [rate,ciLow,ciHigh] = binomialSummary(successCount,totalCount)
if totalCount <= 0
    [rate,ciLow,ciHigh] = deal(NaN);
    return;
end
rate = successCount/totalCount;
z = 1.95996398454005;
denominator = 1+z^2/totalCount;
center = (rate+z^2/(2*totalCount))/denominator;
halfWidth = z*sqrt(rate*(1-rate)/totalCount+z^2/(4*totalCount^2))/denominator;
ciLow = max(0,center-halfWidth);
ciHigh = min(1,center+halfWidth);
end

function value = meanFinite(values)
values = values(isfinite(values));
if isempty(values), value = NaN; else, value = mean(values); end
end

function value = medianFinite(values)
values = values(isfinite(values));
if isempty(values), value = NaN; else, value = median(values); end
end

function postSummary = buildPostSummaryTable(postTable)
if isempty(postTable)
    postSummary = table();
    return;
end
groupKey = strcat(string(postTable.algorithm),"|",string(postTable.source));
groupNames = unique(groupKey,'stable');
summaryRows = repmat(emptyPostSummaryRow(),numel(groupNames),1);
for i = 1:numel(groupNames)
    separator = strfind(char(groupNames(i)),'|');
    algorithm = char(groupNames(i));
    algorithm = algorithm(1:separator-1);
    source = char(groupNames(i));
    source = source(separator+1:end);
    mask = groupKey == groupNames(i);
    summaryRows(i) = summarizePostSubset(postTable(mask,:),algorithm,source);
end
postSummary = struct2table(summaryRows);
end

function row = emptyPostSummaryRow()
row = struct('algorithm','', 'source','', 'nRuns',NaN, ...
    'nCollisionFree',NaN, 'collisionFreeRate',NaN, ...
    'rawPathLengthMedian',NaN, 'greedyPathLengthMedian',NaN, ...
    'bsplinePathLengthMedian',NaN, 'turningAngleSumMedian_rad',NaN, ...
    'maxTurningAngleMedian_rad',NaN, 'postProcessTimeMedian_ms',NaN, ...
    'postProcessCollisionChecksMedian',NaN);
end

function row = summarizePostSubset(subset,algorithm,source)
row = emptyPostSummaryRow();
row.algorithm = algorithm;
row.source = source;
row.nRuns = height(subset);
row.nCollisionFree = sum(subset.collisionFree);
row.collisionFreeRate = row.nCollisionFree/row.nRuns;
row.rawPathLengthMedian = medianFinite(subset.rawPathLength);
row.greedyPathLengthMedian = medianFinite(subset.greedyPathLength);
valid = subset.collisionFree;
row.bsplinePathLengthMedian = medianFinite(subset.bsplinePathLength(valid));
row.turningAngleSumMedian_rad = medianFinite(subset.turningAngleSum_rad(valid));
row.maxTurningAngleMedian_rad = medianFinite(subset.maxTurningAngle_rad(valid));
row.postProcessTimeMedian_ms = medianFinite(subset.postProcessTime_ms);
row.postProcessCollisionChecksMedian = medianFinite( ...
    subset.postProcessCollisionChecks);
end

function figureResults = createBenchmarkFigures(rawTable,summaryTable, ...
        postTable,convergenceRecords,figureDirectory)
figureResults = {};
experimentA = rawTable(strcmp(rawTable.experiment,'FirstSolution'),:);
figureResults{end+1} = makeSuccessFigure(experimentA,figureDirectory);
figureResults{end+1} = makeBoxFigure(experimentA,'firstSolutionTime_ms', ...
    'First solution time (ms)','FirstSolutionTime_boxplot.png', ...
    figureDirectory,true);
figureResults{end+1} = makeBoxFigure(experimentA,'firstPathLength', ...
    'First path length','FirstPathLength_boxplot.png', ...
    figureDirectory,false);
experimentB = rawTable(strcmp(rawTable.experiment,'FixedOptimization1000'),:);
figureResults{end+1} = makeBoxFigure(experimentB,'finalPathLength', ...
    'Final path length after 1000 extra iterations', ...
    'FinalPathLength_boxplot.png',figureDirectory,false);
figureResults{end+1} = makeBoxFigure(experimentA,'nodeCount', ...
    'Node count','NodeCount_boxplot.png',figureDirectory,true);
figureResults{end+1} = makeBoxFigure(experimentA,'collisionChecks', ...
    'Collision checks (log scale)','CollisionChecks_boxplot.png', ...
    figureDirectory,true);
figureResults{end+1} = makeScatterFigure(experimentA,figureDirectory);
figureResults{end+1} = makeCollisionPathScatterFigure(experimentA,figureDirectory);
figureResults{end+1} = makeConvergenceFigure(convergenceRecords,figureDirectory);
figureResults{end+1} = makePostprocessFigure(postTable,figureDirectory);
if isempty(summaryTable), return; end
end

function outputPath = makeSuccessFigure(data,figureDirectory)
figureHandle = figure('Visible','off','Color','w');
algorithms = unique(data.algorithm,'stable');
successRates = zeros(numel(algorithms),1);
for i = 1:numel(algorithms)
    successRates(i) = mean(data.success(strcmp(data.algorithm,algorithms{i})));
end
bar(successRates);
set(gca,'XTick',1:numel(algorithms),'XTickLabel',algorithms);
xtickangle(35); ylim([0 1.05]); grid on; box on;
ylabel('Success rate'); title('Experiment A success rate');
outputPath = fullfile(figureDirectory,'SuccessRate_bar.png');
exportgraphics(figureHandle,outputPath,'Resolution',300);
close(figureHandle);
end

function outputPath = makeBoxFigure(data,fieldName,yLabel,fileName, ...
        figureDirectory,logScale)
figureHandle = figure('Visible','off','Color','w');
if isempty(data) || height(data) == 0
    axis off;
    title([yLabel ' (no Experiment B data in this pilot)']);
    outputPath = fullfile(figureDirectory,fileName);
    exportgraphics(figureHandle,outputPath,'Resolution',300);
    close(figureHandle);
    return;
end
algorithms = unique(data.algorithm,'stable');
values = [];
groups = [];
for i = 1:numel(algorithms)
    subset = data.(fieldName)(strcmp(data.algorithm,algorithms{i}));
    subset = subset(isfinite(subset));
    values = [values;subset]; %#ok<AGROW>
    groups = [groups;repmat(i,numel(subset),1)]; %#ok<AGROW>
end
if isempty(values), values = NaN; groups = 1; end
boxplot(values,groups,'Labels',algorithms);
xtickangle(35); grid on; box on; ylabel(yLabel);
if logScale, set(gca,'YScale','log'); end
title(yLabel);
outputPath = fullfile(figureDirectory,fileName);
exportgraphics(figureHandle,outputPath,'Resolution',300);
close(figureHandle);
end

function outputPath = makeScatterFigure(data,figureDirectory)
figureHandle = figure('Visible','off','Color','w');
hold on; grid on; box on;
algorithms = unique(data.algorithm,'stable');
for i = 1:numel(algorithms)
    mask = strcmp(data.algorithm,algorithms{i}) & data.success;
    scatter(data.firstSolutionTime_ms(mask),data.firstPathLength(mask), ...
        18,'DisplayName',algorithms{i});
end
xlabel('First solution time (ms)'); ylabel('First path length');
title('Experiment A speed-quality scatter'); legend('Location','best');
outputPath = fullfile(figureDirectory,'FirstTime_vs_FirstLength_scatter.png');
exportgraphics(figureHandle,outputPath,'Resolution',300);
close(figureHandle);
end

function outputPath = makeCollisionPathScatterFigure(data,figureDirectory)
figureHandle = figure('Visible','off','Color','w');
hold on; grid on; box on;
algorithms = unique(data.algorithm,'stable');
for i = 1:numel(algorithms)
    mask = strcmp(data.algorithm,algorithms{i}) & data.success & ...
        isfinite(data.collisionChecks) & isfinite(data.firstPathLength);
    scatter(data.collisionChecks(mask),data.firstPathLength(mask), ...
        18,'DisplayName',algorithms{i});
end
set(gca,'XScale','log');
xlabel('Collision checks (log scale)'); ylabel('First path length');
title('Experiment A collision-cost versus path-quality');
legend('Location','best');
outputPath = fullfile(figureDirectory,'CollisionChecks_vs_FirstLength_scatter.png');
exportgraphics(figureHandle,outputPath,'Resolution',300);
close(figureHandle);
end

function outputPath = makeConvergenceFigure(records,figureDirectory)
figureHandle = figure('Visible','off','Color','w');
hold on; grid on; box on;
algorithms = unique({records.algorithm},'stable');
offsets = 0:100:1000;
for algorithmIndex = 1:numel(algorithms)
    values = NaN(numel(records),numel(offsets));
    selectedRecords = records(strcmp({records.algorithm}, ...
        algorithms{algorithmIndex}));
    for i = 1:numel(selectedRecords)
        firstIteration = selectedRecords(i).firstSolutionIteration;
        history = selectedRecords(i).bestCostHistory;
        if ~isfinite(firstIteration) || isempty(history), continue; end
        for offsetIndex = 1:numel(offsets)
            historyIndex = firstIteration+offsets(offsetIndex);
            if historyIndex >= 1 && historyIndex <= numel(history)
                values(i,offsetIndex) = history(historyIndex);
            end
        end
    end
    medianValues = NaN(1,numel(offsets));
    lowerValues = medianValues;
    upperValues = medianValues;
    for offsetIndex = 1:numel(offsets)
        valid = values(:,offsetIndex);
        valid = valid(isfinite(valid));
        if ~isempty(valid)
            medianValues(offsetIndex) = median(valid);
            limits = prctile(valid,[25 75]);
            lowerValues(offsetIndex) = limits(1);
            upperValues(offsetIndex) = limits(2);
        end
    end
    plot(offsets,medianValues,'LineWidth',1.7, ...
        'DisplayName',algorithms{algorithmIndex});
    plot(offsets,lowerValues,':','HandleVisibility','off');
    plot(offsets,upperValues,':','HandleVisibility','off');
end
xlabel('Iterations after first solution'); ylabel('Best-known path cost');
title('Optimization convergence median with IQR bounds');
legend('Location','best');
outputPath = fullfile(figureDirectory,'Optimization_convergence.png');
exportgraphics(figureHandle,outputPath,'Resolution',300);
close(figureHandle);
end

function outputPath = makePostprocessFigure(postTable,figureDirectory)
figureHandle = figure('Visible','off','Color','w');
valid = postTable.collisionFree == 1;
stageData = [postTable.rawPathLength(valid), ...
    postTable.greedyPathLength(valid),postTable.bsplinePathLength(valid)];
if isempty(stageData)
    axis off;
    title('APF post-processing (no valid smoothed samples)');
    outputPath = fullfile(figureDirectory,'APF_postprocess_stages.png');
    exportgraphics(figureHandle,outputPath,'Resolution',300);
    close(figureHandle);
    return;
end
subplot(1,3,1);
boxplot(stageData,'Labels',{'Raw','Greedy','B-spline'});
grid on; box on; ylabel('Path length');
title('Length');
subplot(1,3,2);
turningData = [postTable.turningAngleSum_rad(valid), ...
    postTable.maxTurningAngle_rad(valid)];
boxplot(turningData,'Labels',{'Angle sum','Max angle'});
grid on; box on; ylabel('Radians');
title('Turning');
subplot(1,3,3);
timeData = postTable.postProcessTime_ms(valid);
boxplot(timeData,'Labels',{'Post-process'});
grid on; box on; ylabel('ms');
title('Time');
sgtitle('APF-Bi-RRT* post-processing (valid smoothing only)');
outputPath = fullfile(figureDirectory,'APF_postprocess_stages.png');
exportgraphics(figureHandle,outputPath,'Resolution',300);
close(figureHandle);
end

function printPilotAcceptance(rawTable,pilotElapsed)
fprintf('\nPilot acceptance summary:\n');
algorithms = unique(rawTable.algorithm,'stable');
for i = 1:numel(algorithms)
    mask = strcmp(rawTable.algorithm,algorithms{i}) & ...
        strcmp(rawTable.experiment,'FirstSolution');
    successCount = sum(rawTable.success(mask));
    checks = rawTable.collisionChecks(mask);
    checks = checks(isfinite(checks));
    if isempty(checks), maxChecks = NaN; else, maxChecks = max(checks); end
    fprintf('%-24s success=%3d/%3d maxChecks=%g\n',algorithms{i}, ...
        successCount,sum(mask),maxChecks);
end
failed = rawTable(~rawTable.success,:);
errors = rawTable(~cellfun(@isempty,rawTable.errorMessage),:);
fprintf('Failures: %d, planner errors: %d, path-validity failures among successes: %d\n', ...
    height(failed),height(errors),sum(rawTable.success & ~rawTable.pathCollisionFree));
fprintf('Max-iteration truncations: %d\n',sum(rawTable.maxIterationsTruncated));
fprintf('Total pilot wall time: %.3f s\n',pilotElapsed);
end
