function outputs = run_phase10a5_optimization_pilot()
% run_phase10a5_optimization_pilot  Phase 10A.5 fixed-budget pilot.
%
% Runs the six main optimization planners plus GD-RRT* Extended on the
% same four maps and seeds used by Experiment A.  The search core is not
% changed here: this file only supplies benchmark configuration, records
% first-to-final diagnostics, and writes per-map summaries/figures.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

maps = phase10Maps();
seeds = 1:20;
optimizationBudget = 1000;
maxIterations = 6000;
stepSize = 1;
nearRadiusMax = 25;
lengthTolerance = 1e-8;
relativeIterations = 0:100:optimizationBudget;

algorithms = optimizationAlgorithmDefinitions();
resultsDirectory = fullfile(projectRoot,'results');
figureDirectory = fullfile(resultsDirectory,'benchmark_figures');
if ~exist(resultsDirectory,'dir'), mkdir(resultsDirectory); end
if ~exist(figureDirectory,'dir'), mkdir(figureDirectory); end

% Warm up every exact extended configuration.  Warm-up runs are excluded.
for algorithmIndex = 1:numel(algorithms)
    warmupConfig = makeOptimizationConfig(maps(1),1,100,stepSize, ...
        nearRadiusMax,optimizationBudget);
    try
        algorithms(algorithmIndex).handle(warmupConfig);
    catch warmupError
        warning('Phase10A5:WarmupFailed','%s warm-up failed: %s', ...
            algorithms(algorithmIndex).label,warmupError.message);
    end
end

runCount = numel(maps)*numel(seeds)*numel(algorithms);
rawRows = repmat(emptyOptimizationRow(),runCount,1);
convergenceRows = repmat(emptyConvergenceRow(), ...
    runCount*numel(relativeIterations),1);
anomalyRows = repmat(emptyAnomalyRow(),runCount,1);
rowIndex = 0;
convergenceIndex = 0;
anomalyIndex = 0;
pilotTimer = tic;

checkpointFile = fullfile(resultsDirectory, ...
    'phase10a5_optimization_checkpoint.mat');

for mapIndex = 1:numel(maps)
    for seedIndex = 1:numel(seeds)
        seed = seeds(seedIndex);
        for algorithmIndex = 1:numel(algorithms)
            algorithm = algorithms(algorithmIndex);
            config = makeOptimizationConfig(maps(mapIndex),seed, ...
                maxIterations,stepSize,nearRadiusMax,optimizationBudget);
            runTimer = tic;
            runError = '';
            try
                result = algorithm.handle(config);
            catch plannerError
                result = makeFailureResult(algorithm.label,config, ...
                    plannerError.message);
                runError = plannerError.message;
            end
            wallElapsed = toc(runTimer);

            rowIndex = rowIndex+1;
            rawRows(rowIndex) = makeOptimizationRow(result,algorithm, ...
                maps(mapIndex),seed,config,wallElapsed,runError, ...
                lengthTolerance);

            if rawRows(rowIndex).success
                [convergenceRows,convergenceIndex] = addConvergenceRows( ...
                    convergenceRows,convergenceIndex,result,algorithm, ...
                    maps(mapIndex),seed,relativeIterations);
            end

            if ~isempty(rawRows(rowIndex).errorMessage)
                anomalyIndex = anomalyIndex+1;
                anomalyRows(anomalyIndex) = makeAnomalyRow( ...
                    rawRows(rowIndex),'planner exception');
            elseif ~isempty(rawRows(rowIndex).anomaly)
                anomalyIndex = anomalyIndex+1;
                anomalyRows(anomalyIndex) = makeAnomalyRow( ...
                    rawRows(rowIndex),rawRows(rowIndex).anomaly);
            end
        end
        fprintf('OptimizationPilot map %d/%d seed %d/%d complete (%d/%d runs).\n', ...
            mapIndex,numel(maps),seedIndex,numel(seeds),rowIndex,runCount);
    end
    % Keep a recoverable checkpoint after every map.
    save(checkpointFile,'rawRows','rowIndex','convergenceRows', ...
        'convergenceIndex','anomalyRows','anomalyIndex','maps','seeds', ...
        'algorithms','-v7.3');
end

pilotElapsed = toc(pilotTimer);
rawRows = rawRows(1:rowIndex);
convergenceRows = convergenceRows(1:convergenceIndex);
anomalyRows = anomalyRows(1:anomalyIndex);
rawTable = struct2table(rawRows);
convergenceTable = struct2table(convergenceRows);
summaryTable = buildOptimizationSummary(rawTable);
pooledTable = buildPooledDiagnostic(rawTable);
anomalyTable = struct2table(anomalyRows);

writetable(rawTable,fullfile(resultsDirectory, ...
    'pilot_optimization_raw_results.csv'));
writetable(summaryTable,fullfile(resultsDirectory, ...
    'pilot_optimization_summary.csv'));
writetable(pooledTable,fullfile(resultsDirectory, ...
    'pilot_optimization_pooled_diagnostic.csv'));
writetable(convergenceTable,fullfile(resultsDirectory, ...
    'pilot_optimization_convergence.csv'));
writetable(anomalyTable,fullfile(resultsDirectory, ...
    'pilot_optimization_anomalies.csv'));
save(fullfile(resultsDirectory,'phase10a5_optimization_pilot.mat'), ...
    'rawTable','summaryTable','pooledTable','convergenceTable', ...
    'anomalyTable','maps','seeds','relativeIterations','pilotElapsed');

figureResults = cell(numel(maps),1);
for mapIndex = 1:numel(maps)
    figureResults{mapIndex} = makeOptimizationConvergenceFigure( ...
        convergenceTable,maps(mapIndex),figureDirectory);
end

outputs = struct('rawTable',rawTable,'summaryTable',summaryTable, ...
    'pooledTable',pooledTable,'convergenceTable',convergenceTable, ...
    'anomalyTable',anomalyTable,'maps',maps,'seeds',seeds, ...
    'relativeIterations',relativeIterations,'figureResults',{figureResults}, ...
    'totalRunCount',height(rawTable),'pilotElapsedSeconds',pilotElapsed);

fprintf('\nPhase 10A.5 optimization pilot finished: %d runs in %.3f s.\n', ...
    height(rawTable),pilotElapsed);
printOptimizationAcceptance(rawTable,pilotElapsed);
end

function algorithms = optimizationAlgorithmDefinitions()
algorithms = struct('label',{},'handle',{});
algorithms(end+1) = struct('label','RRT*','handle',@RRT_star);
algorithms(end+1) = struct('label','Bi-RRT*','handle',@Bi_RRT_star);
algorithms(end+1) = struct('label','Informed RRT*','handle',@Informed_RRT_star);
algorithms(end+1) = struct('label','Q-RRT*','handle',@Q_RRT_star);
algorithms(end+1) = struct('label','Classified-RRT*','handle',@Classified_RRT_star);
algorithms(end+1) = struct('label','APF-Bi-RRT*','handle',@APF_Bi_RRT_star);
algorithms(end+1) = struct('label','GD-RRT* Extended','handle',@GD_RRT_star);
end

function config = makeOptimizationConfig(map,seed,maxIterations, ...
        stepSize,nearRadiusMax,optimizationBudget)
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
config.optimizeIterations = optimizationBudget;
config.gdPaperMode = false;
config.crrtPaperMode = false;
config.apfPaperMode = false;
config.apfPaperConnectionMode = 'best_near';
config.benchmarkExperiment = 'FixedOptimization1000';
end

function result = makeFailureResult(algorithm,config,errorMessage)
result = initRRTResult(algorithm,config);
result.success = false;
result.errorMessage = errorMessage;
result.iterations = 0;
result.nodeCount = NaN;
result.collisionChecks = NaN;
result.rewireCount = NaN;
end

function row = emptyOptimizationRow()
row = struct('algorithm','', 'experiment','FixedOptimization1000', ...
    'map',NaN, 'mapName','', 'seed',NaN, 'success',false, ...
    'pathCollisionFree',false, 'isPaperMode',false, ...
    'firstSolutionIteration',NaN, 'firstSolutionTime_ms',NaN, ...
    'firstPathLength',NaN, 'finalIteration',NaN, ...
    'optimizationIterationsRequested',NaN, ...
    'optimizationIterationsExecuted',NaN, 'optimizationTime_ms',NaN, ...
    'planningTime_ms',NaN, 'finalPathLength',NaN, ...
    'pathImprovementAbsolute',NaN, 'pathImprovementPercent',NaN, ...
    'collisionChecksAfterFirstSolution',NaN, ...
    'improvementPer1000Checks',NaN, 'improvementPerMillisecond',NaN, ...
    'nodeCount',NaN, 'collisionChecks',NaN, 'coreCollisionChecks',NaN, ...
    'collisionChecksAtFirstSolution',NaN, 'finalValidationCollisionChecks',NaN, ...
    'rewireCount',NaN, 'qDepth',NaN, 'ancestorChecks',NaN, ...
    'ancestorAcceptedCount',NaN, 'gaussianSampleCount',NaN, ...
    'offsetSuccessCount',NaN, 'backtrackChecks',NaN, ...
    'goalTreeConnectionChecks',NaN, 'apfZeroForceCount',NaN, ...
    'maxIterationsTruncated',false, 'finalLengthError',false, ...
    'budgetError',false, 'anomaly','NONE', 'wallTime_ms',NaN, ...
    'errorMessage','');
end

function row = makeOptimizationRow(result,algorithm,map,seed,config, ...
        wallElapsed,runError,lengthTolerance)
row = emptyOptimizationRow();
row.algorithm = algorithm.label;
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
row.optimizationIterationsRequested = getNumeric(result, ...
    'optimizationIterationsRequested',config.optimizeIterations);
row.optimizationIterationsExecuted = getNumeric(result, ...
    'optimizationIterationsExecuted',0);
row.isPaperMode = getLogical(result,'gdPaperMode',false) || ...
    getLogical(result,'crrtPaperMode',false) || ...
    getLogical(result,'apfPaperMode',false);

if isfield(result,'searchPlanningTime')
    planningSeconds = result.searchPlanningTime;
else
    planningSeconds = getNumeric(result,'planningTime',NaN);
end
row.planningTime_ms = planningSeconds*1000;
row.optimizationTime_ms = max(0,row.planningTime_ms- ...
    row.firstSolutionTime_ms);

if strcmp(algorithm.label,'APF-Bi-RRT*') && ...
        isfield(result,'rawPathLength')
    row.finalPathLength = getNumeric(result,'rawPathLength',NaN);
    finalPath = getFieldOrEmpty(result,'rawPath');
else
    row.finalPathLength = getNumeric(result,'pathLength',NaN);
    finalPath = getFieldOrEmpty(result,'path');
end
row.nodeCount = getNumeric(result,'nodeCount',NaN);
row.rewireCount = getNumeric(result,'rewireCount',0);
if isfield(result,'searchCollisionChecks') && ...
        strcmp(algorithm.label,'APF-Bi-RRT*')
    row.collisionChecks = getNumeric(result,'searchCollisionChecks',NaN);
else
    row.collisionChecks = getNumeric(result,'collisionChecks',NaN);
end
if isfield(result,'searchCollisionChecks')
    row.coreCollisionChecks = getNumeric(result,'searchCollisionChecks',NaN);
else
    row.coreCollisionChecks = row.collisionChecks;
end
row.collisionChecksAtFirstSolution = getNumeric(result, ...
    'collisionChecksAtFirstSolution',NaN);
row.collisionChecksAfterFirstSolution = row.collisionChecks- ...
    row.collisionChecksAtFirstSolution;
row.finalValidationCollisionChecks = getNumeric(result, ...
    'finalValidationCollisionChecks',0);

if row.success && ~isempty(finalPath)
    row.pathCollisionFree = validatePath(finalPath,config);
end
if row.success && isfinite(row.firstPathLength) && ...
        isfinite(row.finalPathLength)
    row.pathImprovementAbsolute = row.firstPathLength-row.finalPathLength;
    row.pathImprovementPercent = 100*row.pathImprovementAbsolute/ ...
        row.firstPathLength;
    if isfinite(row.collisionChecksAfterFirstSolution) && ...
            row.collisionChecksAfterFirstSolution > 0
        row.improvementPer1000Checks = 1000*row.pathImprovementAbsolute/ ...
            row.collisionChecksAfterFirstSolution;
    end
    if isfinite(row.optimizationTime_ms) && row.optimizationTime_ms > 0
        row.improvementPerMillisecond = row.pathImprovementAbsolute/ ...
            row.optimizationTime_ms;
    end
end

row.maxIterationsTruncated = (~row.success && ...
    row.finalIteration >= config.maxIterations) || ...
    (row.success && row.optimizationIterationsExecuted < ...
    row.optimizationIterationsRequested && ...
    row.finalIteration >= config.maxIterations);
row.budgetError = row.success && ...
    row.optimizationIterationsExecuted ~= config.optimizeIterations;
row.finalLengthError = row.success && ...
    row.finalPathLength > row.firstPathLength+lengthTolerance;

row.qDepth = getNumeric(result,'qDepth',NaN);
row.ancestorChecks = getNumeric(result,'ancestorChecks',NaN);
row.ancestorAcceptedCount = getNumeric(result,'ancestorAcceptedCount',NaN);
row.gaussianSampleCount = getNumeric(result,'gaussianSampleCount',NaN);
row.offsetSuccessCount = getNumeric(result,'offsetSuccessCount',NaN);
row.backtrackChecks = getNumeric(result,'backtrackChecks',NaN);
row.goalTreeConnectionChecks = getNumeric(result, ...
    'goalTreeConnectionChecks',NaN);
row.apfZeroForceCount = getNumeric(result,'apfZeroForceCount',NaN);

issues = {};
if ~isempty(runError), issues{end+1} = 'PLANNER_EXCEPTION'; end %#ok<AGROW>
if ~row.success, issues{end+1} = 'FAILURE'; end %#ok<AGROW>
if row.success && ~row.pathCollisionFree
    issues{end+1} = 'PATH_COLLISION'; %#ok<AGROW>
end
if row.finalLengthError, issues{end+1} = 'FINAL_LONGER_THAN_FIRST'; end %#ok<AGROW>
if row.budgetError, issues{end+1} = 'OPTIMIZATION_BUDGET_MISMATCH'; end %#ok<AGROW>
if row.maxIterationsTruncated, issues{end+1} = 'MAX_ITERATIONS_TRUNCATED'; end %#ok<AGROW>
required = [row.firstSolutionIteration,row.firstSolutionTime_ms, ...
    row.firstPathLength,row.finalIteration,row.finalPathLength, ...
    row.planningTime_ms,row.optimizationTime_ms,row.nodeCount, ...
    row.collisionChecks];
if row.success && any(~isfinite(required))
    issues{end+1} = 'REQUIRED_NAN_OR_INF'; %#ok<AGROW>
end
if isempty(issues)
    row.anomaly = 'NONE';
else
    row.anomaly = strjoin(issues,';');
end
end

function [rows,index] = addConvergenceRows(rows,index,result,algorithm, ...
        map,seed,relativeIterations)
history = getFieldOrEmpty(result,'bestCostHistory');
firstIteration = getNumeric(result,'firstSolutionIteration',NaN);
for i = 1:numel(relativeIterations)
    index = index+1;
    row = emptyConvergenceRow();
    row.algorithm = algorithm.label;
    row.map = map.id;
    row.mapName = map.name;
    row.seed = seed;
    row.relativeIteration = relativeIterations(i);
    historyIndex = firstIteration+relativeIterations(i);
    if isfinite(historyIndex) && historyIndex >= 1 && ...
            historyIndex <= numel(history)
        row.bestRawPathLength = history(historyIndex);
    end
    rows(index) = row;
end
end

function row = emptyConvergenceRow()
row = struct('algorithm','', 'map',NaN, 'mapName','', 'seed',NaN, ...
    'relativeIteration',NaN, 'bestRawPathLength',NaN);
end

function row = emptyAnomalyRow()
row = struct('algorithm','', 'map',NaN, 'mapName','', 'seed',NaN, ...
    'anomaly','', 'success',false, 'firstSolutionIteration',NaN, ...
    'finalIteration',NaN, 'optimizationIterationsExecuted',NaN, ...
    'firstPathLength',NaN, 'finalPathLength',NaN, ...
    'collisionChecks',NaN, 'goalTreeConnectionChecks',NaN);
end

function row = makeAnomalyRow(source,reason)
row = emptyAnomalyRow();
row.algorithm = source.algorithm;
row.map = source.map;
row.mapName = source.mapName;
row.seed = source.seed;
if strcmp(source.anomaly,'NONE'), row.anomaly = reason; ...
else, row.anomaly = source.anomaly; end
row.success = source.success;
row.firstSolutionIteration = source.firstSolutionIteration;
row.finalIteration = source.finalIteration;
row.optimizationIterationsExecuted = source.optimizationIterationsExecuted;
row.firstPathLength = source.firstPathLength;
row.finalPathLength = source.finalPathLength;
row.collisionChecks = source.collisionChecks;
row.goalTreeConnectionChecks = source.goalTreeConnectionChecks;
end

function summary = buildOptimizationSummary(rawTable)
algorithms = unique(rawTable.algorithm,'stable');
mapIDs = unique(rawTable.map,'stable');
rows = repmat(emptySummaryRow(),numel(algorithms)*numel(mapIDs),1);
index = 0;
for algorithmIndex = 1:numel(algorithms)
    for mapIndex = 1:numel(mapIDs)
        mask = strcmp(rawTable.algorithm,algorithms{algorithmIndex}) & ...
            rawTable.map == mapIDs(mapIndex);
        index = index+1;
        rows(index) = summarizeSubset(rawTable(mask,:), ...
            algorithms{algorithmIndex},mapIDs(mapIndex));
    end
end
summary = struct2table(rows(1:index));
end

function row = emptySummaryRow()
row = struct('algorithm','', 'map',NaN, 'mapName','', 'nRuns',NaN, ...
    'nSuccess',NaN, 'nFailure',NaN, 'successRate',NaN, ...
    'firstPathLengthMedian',NaN, 'finalPathLengthMedian',NaN, ...
    'improvementAbsoluteMedian',NaN, 'improvementPercentMedian',NaN, ...
    'firstSolutionTimeMedian_ms',NaN, 'optimizationTimeMedian_ms',NaN, ...
    'totalPlanningTimeMedian_ms',NaN, 'nodeCountMedian',NaN, ...
    'collisionChecksMedian',NaN, 'rewireCountMedian',NaN, ...
    'goalTreeConnectionChecksMedian',NaN, ...
    'maxIterationsTruncatedCount',NaN, 'budgetErrorCount',NaN, ...
    'finalLengthErrorCount',NaN);
end

function row = summarizeSubset(subset,algorithm,mapID)
row = emptySummaryRow();
row.algorithm = algorithm;
row.map = mapID;
row.mapName = subset.mapName{1};
row.nRuns = height(subset);
row.nSuccess = sum(subset.success);
row.nFailure = row.nRuns-row.nSuccess;
row.successRate = row.nSuccess/row.nRuns;
success = subset.success == 1;
row.firstPathLengthMedian = medianFinite(subset.firstPathLength(success));
row.finalPathLengthMedian = medianFinite(subset.finalPathLength(success));
row.improvementAbsoluteMedian = medianFinite( ...
    subset.pathImprovementAbsolute(success));
row.improvementPercentMedian = medianFinite( ...
    subset.pathImprovementPercent(success));
row.firstSolutionTimeMedian_ms = medianFinite( ...
    subset.firstSolutionTime_ms(success));
row.optimizationTimeMedian_ms = medianFinite( ...
    subset.optimizationTime_ms(success));
row.totalPlanningTimeMedian_ms = medianFinite( ...
    subset.planningTime_ms(success));
row.nodeCountMedian = medianFinite(subset.nodeCount(success));
row.collisionChecksMedian = medianFinite(subset.collisionChecks(success));
row.rewireCountMedian = medianFinite(subset.rewireCount(success));
row.goalTreeConnectionChecksMedian = medianFinite( ...
    subset.goalTreeConnectionChecks(success));
row.maxIterationsTruncatedCount = sum(subset.maxIterationsTruncated);
row.budgetErrorCount = sum(subset.budgetError);
row.finalLengthErrorCount = sum(subset.finalLengthError);
end

function pooled = buildPooledDiagnostic(rawTable)
algorithms = unique(rawTable.algorithm,'stable');
rows = repmat(emptyPooledRow(),numel(algorithms),1);
for i = 1:numel(algorithms)
    subset = rawTable(strcmp(rawTable.algorithm,algorithms{i}),:);
    rows(i) = summarizePooledSubset(subset,algorithms{i});
end
pooled = struct2table(rows);
end

function row = emptyPooledRow()
row = struct('scope','pooled diagnostic only', 'algorithm','', ...
    'nRuns',NaN, 'nSuccess',NaN, 'successRate',NaN, ...
    'firstPathLengthMedian',NaN, 'finalPathLengthMedian',NaN, ...
    'improvementAbsoluteMedian',NaN, 'improvementPercentMedian',NaN, ...
    'firstSolutionTimeMedian_ms',NaN, 'optimizationTimeMedian_ms',NaN, ...
    'totalPlanningTimeMedian_ms',NaN, 'nodeCountMedian',NaN, ...
    'collisionChecksMedian',NaN, 'rewireCountMedian',NaN);
end

function row = summarizePooledSubset(subset,algorithm)
row = emptyPooledRow();
row.algorithm = algorithm;
row.nRuns = height(subset);
row.nSuccess = sum(subset.success);
row.successRate = row.nSuccess/row.nRuns;
success = subset.success == 1;
row.finalPathLengthMedian = medianFinite(subset.finalPathLength(success));
row.firstPathLengthMedian = medianFinite(subset.firstPathLength(success));
row.improvementAbsoluteMedian = medianFinite( ...
    subset.pathImprovementAbsolute(success));
row.improvementPercentMedian = medianFinite( ...
    subset.pathImprovementPercent(success));
row.firstSolutionTimeMedian_ms = medianFinite( ...
    subset.firstSolutionTime_ms(success));
row.optimizationTimeMedian_ms = medianFinite( ...
    subset.optimizationTime_ms(success));
row.totalPlanningTimeMedian_ms = medianFinite( ...
    subset.planningTime_ms(success));
row.nodeCountMedian = medianFinite(subset.nodeCount(success));
row.collisionChecksMedian = medianFinite(subset.collisionChecks(success));
row.rewireCountMedian = medianFinite(subset.rewireCount(success));
end

function outputPath = makeOptimizationConvergenceFigure(data,map,figureDirectory)
figureHandle = figure('Visible','off','Color','w');
hold on; grid on; box on;
mapData = data(data.map == map.id,:);
algorithms = unique(mapData.algorithm,'stable');
for algorithmIndex = 1:numel(algorithms)
    selected = mapData(strcmp(mapData.algorithm,algorithms{algorithmIndex}),:);
    x = unique(selected.relativeIteration,'stable');
    medianValues = NaN(size(x));
    lowerValues = medianValues;
    upperValues = medianValues;
    for i = 1:numel(x)
        values = selected.bestRawPathLength( ...
            selected.relativeIteration == x(i));
        values = values(isfinite(values));
        if ~isempty(values)
            medianValues(i) = median(values);
            limits = prctile(values,[25 75]);
            lowerValues(i) = limits(1);
            upperValues(i) = limits(2);
        end
    end
    plot(x,medianValues,'LineWidth',1.7, ...
        'DisplayName',algorithms{algorithmIndex});
    plot(x,lowerValues,':','HandleVisibility','off');
    plot(x,upperValues,':','HandleVisibility','off');
end
xlabel('Iterations after first solution');
ylabel('Best raw path length');
title(sprintf('Map %d: optimization convergence median and IQR',map.id));
if ~isempty(algorithms), legend('Location','best'); end
outputPath = fullfile(figureDirectory, ...
    sprintf('Map%d_optimization_convergence.png',map.id));
exportgraphics(figureHandle,outputPath,'Resolution',300);
close(figureHandle);
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

function value = getFieldOrEmpty(result,fieldName)
if isfield(result,fieldName)
    value = result.(fieldName);
else
    value = [];
end
end

function value = medianFinite(values)
values = values(isfinite(values));
if isempty(values), value = NaN; else, value = median(values); end
end

function printOptimizationAcceptance(rawTable,pilotElapsed)
fprintf('\nOptimization Pilot acceptance summary:\n');
algorithms = unique(rawTable.algorithm,'stable');
for i = 1:numel(algorithms)
    subset = rawTable(strcmp(rawTable.algorithm,algorithms{i}),:);
    fprintf('%-24s success=%3d/%3d budgetMismatch=%d truncation=%d\n', ...
        algorithms{i},sum(subset.success),height(subset), ...
        sum(subset.budgetError),sum(subset.maxIterationsTruncated));
end
fprintf('Planner exceptions: %d\n', ...
    sum(~cellfun(@isempty,rawTable.errorMessage)));
fprintf('Path-validity failures: %d\n', ...
    sum(rawTable.success & ~rawTable.pathCollisionFree));
fprintf('Final-longer-than-first errors: %d\n',sum(rawTable.finalLengthError));
fprintf('Required NaN/Inf or other anomalies: %d\n', ...
    sum(~strcmp(rawTable.anomaly,'NONE')));
fprintf('Total optimization pilot wall time: %.3f s\n',pilotElapsed);
end
