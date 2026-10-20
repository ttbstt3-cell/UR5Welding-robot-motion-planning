function outputs = run_phase11b_time_budget(seedsInput)
% run_phase11b_time_budget  Phase 11B fixed wall-clock budget benchmark.
%
% Each planner run is executed once through the largest budget and records
% best-so-far snapshots at 25/50/100/200 ms.  The time-budget branch is
% opt-in through config.timeBudgetMs; default planner behavior is unchanged.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

if nargin < 1 || isempty(seedsInput)
    seedsInput = 1:100;
end
seedsInput = reshape(seedsInput,1,[]);
if isempty(seedsInput) || any(seedsInput < 0) || ...
        any(seedsInput ~= floor(seedsInput))
    error('seedsInput must contain non-negative integer seeds.');
end

maps = phase10Maps();
budgetsMs = [25 50 100 200];
variants = timeBudgetDefinitions();
maxIterations = 100000;
stepSize = 1;
nearRadiusMax = 25;

resultsDirectory = fullfile(projectRoot,'results');
figureDirectory = fullfile(resultsDirectory,'benchmark_figures', ...
    'phase11b_time_budget');
if ~exist(resultsDirectory,'dir'), mkdir(resultsDirectory); end
if ~exist(figureDirectory,'dir'), mkdir(figureDirectory); end

for variantIndex = 1:numel(variants)
    warmupConfig = makeTimeBudgetConfig(maps(1),seedsInput(1), ...
        maxIterations,stepSize,nearRadiusMax,variants(variantIndex),budgetsMs);
    try
        variants(variantIndex).handle(warmupConfig);
    catch warmupError
        warning('Phase11B:WarmupFailed','%s warm-up failed: %s', ...
            variants(variantIndex).label,warmupError.message);
    end
end

runCount = numel(maps)*numel(seedsInput)*numel(variants);
rowCount = runCount*numel(budgetsMs);
rawRows = repmat(emptyTimeBudgetRow(),rowCount,1);
bestPaths = cell(rowCount,1);
rowIndex = 0;
runTimer = tic;
checkpointFile = fullfile(resultsDirectory, ...
    'phase11b_time_budget_checkpoint.mat');

for mapIndex = 1:numel(maps)
    for seedIndex = 1:numel(seedsInput)
        seed = seedsInput(seedIndex);
        for variantIndex = 1:numel(variants)
            variant = variants(variantIndex);
            config = makeTimeBudgetConfig(maps(mapIndex),seed, ...
                maxIterations,stepSize,nearRadiusMax,variant,budgetsMs);
            plannerTimer = tic;
            runError = '';
            try
                result = variant.handle(config);
            catch plannerError
                result = [];
                runError = plannerError.message;
            end
            plannerWallTime = toc(plannerTimer)*1000;

            for budgetIndex = 1:numel(budgetsMs)
                rowIndex = rowIndex+1;
                if isempty(result)
                    snapshot = emptyTimeBudgetSnapshot(budgetsMs(budgetIndex));
                    firstSolutionTime = NaN;
                    firstPathLength = NaN;
                else
                    snapshots = result.timeBudgetSnapshots;
                    if budgetIndex <= numel(snapshots)
                        snapshot = snapshots(budgetIndex);
                    else
                        snapshot = emptyTimeBudgetSnapshot( ...
                            budgetsMs(budgetIndex));
                    end
                    firstSolutionTime = getNumeric(result, ...
                        'firstSolutionTime',NaN)*1000;
                    firstPathLength = getNumeric(result, ...
                        'firstPathLength',NaN);
                end
                if isfield(snapshot,'bestPath')
                    bestPaths{rowIndex} = snapshot.bestPath;
                else
                    bestPaths{rowIndex} = [];
                end
                rawRows(rowIndex) = makeTimeBudgetRow( ...
                    result,variant,maps(mapIndex),seed,snapshot, ...
                    firstSolutionTime,firstPathLength,plannerWallTime, ...
                    config,runError);
            end
        end
        fprintf('Phase11B map %d/%d seed %d/%d complete (%d/%d planner runs).\n', ...
            mapIndex,numel(maps),seed,numel(seedsInput), ...
            (mapIndex-1)*numel(seedsInput)*numel(variants)+ ...
            seedIndex*numel(variants),runCount);
    end
    save(checkpointFile,'rawRows','bestPaths','rowIndex','maps', ...
        'budgetsMs','variants','seedsInput','runTimer','-v7.3');
end

rawRows = rawRows(1:rowIndex);
bestPaths = bestPaths(1:rowIndex);
rawTable = struct2table(rawRows);
summaryTable = buildTimeBudgetSummary(rawTable);
pairedTable = buildTimeBudgetPairedStatistics(rawTable);
paretoTable = buildTimeBudgetPareto(summaryTable);
anomalyTable = rawTable(string(rawTable.anomaly) ~= "NONE",:);

rawFile = fullfile(resultsDirectory,'phase11b_time_budget_raw.csv');
summaryFile = fullfile(resultsDirectory,'phase11b_time_budget_summary.csv');
pairedFile = fullfile(resultsDirectory,'phase11b_time_budget_paired_statistics.csv');
paretoFile = fullfile(resultsDirectory,'phase11b_time_budget_pareto.csv');
anomalyFile = fullfile(resultsDirectory,'phase11b_time_budget_anomalies.csv');
writetable(rawTable,rawFile);
writetable(summaryTable,summaryFile);
writetable(pairedTable,pairedFile);
writetable(paretoTable,paretoFile);
writetable(anomalyTable,anomalyFile);

figureResults = cell(1,numel(maps));
for mapIndex = 1:numel(maps)
    figureResults{mapIndex} = makeTimeBudgetFigure(rawTable, ...
        maps(mapIndex),variants,figureDirectory);
end
paretoFigures = cell(1,2);
for mapIndex = 3:4
    paretoFigures{mapIndex-2} = makeParetoFigure(summaryTable, ...
        maps(mapIndex),figureDirectory);
end

pilotElapsed = toc(runTimer);
save(fullfile(resultsDirectory,'phase11b_time_budget.mat'), ...
    'rawTable','summaryTable','pairedTable','paretoTable','anomalyTable', ...
    'bestPaths','maps','budgetsMs','variants','seedsInput','pilotElapsed', ...
    '-v7.3');

outputs = struct('rawTable',rawTable,'summaryTable',summaryTable, ...
    'pairedTable',pairedTable,'paretoTable',paretoTable, ...
    'anomalyTable',anomalyTable,'bestPaths',{bestPaths}, ...
    'maps',maps,'budgetsMs',budgetsMs,'variants',variants, ...
    'seedsInput',seedsInput,'figureResults',{figureResults}, ...
    'paretoFigures',{paretoFigures},'totalRunCount',runCount, ...
    'totalSnapshotCount',height(rawTable),'pilotElapsedSeconds',pilotElapsed);

fprintf('\nPhase 11B finished: %d planner runs, %d snapshots in %.3f s.\n', ...
    runCount,height(rawTable),pilotElapsed);
printTimeBudgetAcceptance(rawTable,runCount,pilotElapsed);
end

function variants = timeBudgetDefinitions()
variants = struct('label',{},'handle',{},'family',{}, ...
    'useDynamicGoalBias',{},'useAPFForce',{},'useOffset',{}, ...
    'useClassificationGoalTree',{},'useBacktracking',{}, ...
    'useSequentialRewire',{});
variants(end+1) = makeVariant('RRT-Connect',@RRT_connect,'baseline', ...
    NaN,NaN,NaN,NaN,NaN,NaN);
variants(end+1) = makeVariant('Bi-RRT*',@Bi_RRT_star,'baseline', ...
    NaN,NaN,NaN,NaN,NaN,NaN);
variants(end+1) = makeVariant('Full Classified-RRT*', ...
    @Classified_RRT_star,'Classified',NaN,NaN,true,true,true,true);
variants(end+1) = makeVariant('ClassGoal-Lite',@Classified_RRT_star, ...
    'Classified',NaN,NaN,false,true,false,false);
variants(end+1) = makeVariant('APF-only Bi-RRT*',@APF_Bi_RRT_star, ...
    'APF',false,true,NaN,NaN,NaN,NaN);
variants(end+1) = makeVariant('Full APF-Bi-RRT*',@APF_Bi_RRT_star, ...
    'APF',true,true,NaN,NaN,NaN,NaN);
end

function variant = makeVariant(label,handle,family,useBias,useForce, ...
        useOffset,useClassification,useBacktracking,useSequential)
variant = struct('label',label,'handle',handle,'family',family, ...
    'useDynamicGoalBias',useBias,'useAPFForce',useForce, ...
    'useOffset',useOffset,'useClassificationGoalTree',useClassification, ...
    'useBacktracking',useBacktracking,'useSequentialRewire',useSequential);
end

function config = makeTimeBudgetConfig(map,seed,maxIterations,stepSize, ...
        nearRadiusMax,variant,budgetsMs)
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
config.timeBudgetMs = budgetsMs;
config.crrtPaperMode = false;
config.apfPaperMode = false;
config.apfPaperConnectionMode = 'best_near';

if strcmp(variant.family,'APF')
    config.ablationUseDynamicGoalBias = variant.useDynamicGoalBias;
    config.ablationUseAPFForce = variant.useAPFForce;
elseif strcmp(variant.family,'Classified')
    config.ablationUseOffset = variant.useOffset;
    config.ablationUseClassificationGoalTree = ...
        variant.useClassificationGoalTree;
    config.ablationUseBacktracking = variant.useBacktracking;
    config.ablationUseSequentialRewire = variant.useSequentialRewire;
end
end

function row = emptyTimeBudgetRow()
row = struct('variant','', 'family','', 'map',NaN, 'mapName','', ...
    'seed',NaN, 'budget_ms',NaN, 'successByBudget',false, ...
    'firstSolutionFound',false, 'firstSolutionTime_ms',NaN, ...
    'firstPathLength',NaN, 'bestPathLength',NaN, 'nodeCount',NaN, ...
    'collisionChecks',NaN, 'rewireCount',NaN, ...
    'actualSnapshotTime_ms',NaN, 'budgetOvershoot_ms',NaN, ...
    'pathCollisionFree',false, 'anomaly','NONE', ...
    'plannerWallTime_ms',NaN, 'errorMessage','');
end

function snapshot = emptyTimeBudgetSnapshot(budgetMs)
snapshot = initTimeBudgetSnapshots(budgetMs);
snapshot = snapshot(1);
end

function row = makeTimeBudgetRow(result,variant,map,seed,snapshot, ...
        firstSolutionTime,firstPathLength,plannerWallTime,config,runError)
row = emptyTimeBudgetRow();
row.variant = variant.label;
row.family = variant.family;
row.map = map.id;
row.mapName = map.name;
row.seed = seed;
row.budget_ms = snapshot.budget_ms;
row.successByBudget = logical(snapshot.successByBudget);
row.firstSolutionFound = logical(snapshot.firstSolutionFound);
row.firstSolutionTime_ms = firstSolutionTime;
row.firstPathLength = firstPathLength;
row.bestPathLength = getNumeric(snapshot,'bestPathLength',NaN);
row.nodeCount = getNumeric(snapshot,'nodeCount',NaN);
row.collisionChecks = getNumeric(snapshot,'collisionChecks',NaN);
row.rewireCount = getNumeric(snapshot,'rewireCount',NaN);
row.actualSnapshotTime_ms = getNumeric(snapshot, ...
    'actualSnapshotTime_ms',NaN);
row.budgetOvershoot_ms = getNumeric(snapshot,'budgetOvershoot_ms',NaN);
row.plannerWallTime_ms = plannerWallTime;
row.errorMessage = runError;

if row.successByBudget && isfield(snapshot,'bestPath') && ...
        ~isempty(snapshot.bestPath)
    row.pathCollisionFree = validatePath(snapshot.bestPath,config);
end

issues = {};
if ~isempty(runError), issues{end+1} = 'PLANNER_EXCEPTION'; end %#ok<AGROW>
if row.successByBudget && ~row.pathCollisionFree
    issues{end+1} = 'PATH_COLLISION'; %#ok<AGROW>
end
if row.successByBudget && ~isfinite(row.bestPathLength)
    issues{end+1} = 'BEST_PATH_NAN_OR_INF'; %#ok<AGROW>
end
if isfinite(row.actualSnapshotTime_ms) && ...
        row.actualSnapshotTime_ms+1e-9 < row.budget_ms
    issues{end+1} = 'SNAPSHOT_BEFORE_BUDGET'; %#ok<AGROW>
end
if ~isfinite(row.actualSnapshotTime_ms)
    issues{end+1} = 'MISSING_CHECKPOINT'; %#ok<AGROW>
end
if isempty(issues)
    row.anomaly = 'NONE';
else
    row.anomaly = strjoin(issues,';');
end
if isempty(result)
    row.firstSolutionFound = false;
end
end

function summary = buildTimeBudgetSummary(rawTable)
variants = unique(rawTable.variant,'stable');
mapIDs = unique(rawTable.map,'stable');
budgets = unique(rawTable.budget_ms,'stable');
rows = repmat(emptyTimeBudgetSummaryRow(), ...
    numel(variants)*numel(mapIDs)*numel(budgets),1);
index = 0;
for v = 1:numel(variants)
    for m = 1:numel(mapIDs)
        for b = 1:numel(budgets)
            mask = strcmp(rawTable.variant,variants{v}) & ...
                rawTable.map == mapIDs(m) & rawTable.budget_ms == budgets(b);
            if ~any(mask), continue; end
            index = index+1;
            rows(index) = summarizeTimeBudgetSubset(rawTable(mask,:), ...
                variants{v},mapIDs(m),budgets(b));
        end
    end
end
summary = struct2table(rows(1:index));
end

function row = emptyTimeBudgetSummaryRow()
row = struct('variant','', 'map',NaN, 'mapName','', 'budget_ms',NaN, ...
    'nRuns',NaN, 'nSuccess',NaN, 'successRateByBudget',NaN, ...
    'firstSolutionFoundRate',NaN, 'bestPathLengthMedian',NaN, ...
    'bestPathLengthIQRLow',NaN, 'bestPathLengthIQRHigh',NaN, ...
    'bestPathLengthBootstrap95CILow',NaN, ...
    'bestPathLengthBootstrap95CIHigh',NaN, ...
    'collisionChecksMedian',NaN, 'collisionChecksIQRLow',NaN, ...
    'collisionChecksIQRHigh',NaN, 'nodeCountMedian',NaN, ...
    'actualPlanningTimeMedian_ms',NaN, 'budgetOvershootMedian_ms',NaN, ...
    'rewireCountMedian',NaN, 'anomalyCount',NaN);
end

function row = summarizeTimeBudgetSubset(subset,variant,mapID,budget)
row = emptyTimeBudgetSummaryRow();
row.variant = variant;
row.map = mapID;
row.mapName = subset.mapName{1};
row.budget_ms = budget;
row.nRuns = height(subset);
row.nSuccess = sum(subset.successByBudget);
row.successRateByBudget = row.nSuccess/row.nRuns;
row.firstSolutionFoundRate = mean(subset.firstSolutionFound);
pathValues = subset.bestPathLength(subset.successByBudget);
row.bestPathLengthMedian = medianFinite(pathValues);
if ~isempty(pathValues)
    quartiles = prctile(pathValues,[25 75]);
    row.bestPathLengthIQRLow = quartiles(1);
    row.bestPathLengthIQRHigh = quartiles(2);
    ci = bootstrapMedianCI(pathValues,1000, ...
        11000+mapID*100+round(budget));
    row.bestPathLengthBootstrap95CILow = ci(1);
    row.bestPathLengthBootstrap95CIHigh = ci(2);
end
row.collisionChecksMedian = medianFinite( ...
    subset.collisionChecks(subset.successByBudget));
collisionValues = subset.collisionChecks(subset.successByBudget);
if ~isempty(collisionValues)
    quartiles = prctile(collisionValues,[25 75]);
    row.collisionChecksIQRLow = quartiles(1);
    row.collisionChecksIQRHigh = quartiles(2);
end
row.nodeCountMedian = medianFinite(subset.nodeCount);
row.actualPlanningTimeMedian_ms = medianFinite( ...
    subset.actualSnapshotTime_ms);
row.budgetOvershootMedian_ms = medianFinite( ...
    subset.budgetOvershoot_ms);
row.rewireCountMedian = medianFinite(subset.rewireCount);
row.anomalyCount = sum(string(subset.anomaly) ~= "NONE");
end

function paired = buildTimeBudgetPairedStatistics(rawTable)
comparisons = {
    'RRT-Connect','Bi-RRT*';
    'Bi-RRT*','APF-only Bi-RRT*';
    'APF-only Bi-RRT*','Full APF-Bi-RRT*';
    'Full Classified-RRT*','ClassGoal-Lite';
    'Bi-RRT*','ClassGoal-Lite'};
metrics = {'BestPathLength','CollisionChecks','NodeCount', ...
    'ActualSnapshotTime_ms','SuccessByBudget'};
rows = repmat(emptyTimeBudgetPairedRow(),0,1);
for c = 1:size(comparisons,1)
    left = comparisons{c,1};
    right = comparisons{c,2};
    for mapID = unique(rawTable.map).'
        for budget = unique(rawTable.budget_ms).'
            leftData = rawTable(strcmp(rawTable.variant,left) & ...
                rawTable.map == mapID & rawTable.budget_ms == budget,:);
            rightData = rawTable(strcmp(rawTable.variant,right) & ...
                rawTable.map == mapID & rawTable.budget_ms == budget,:);
            [~,leftIndex,rightIndex] = intersect(leftData.seed,rightData.seed);
            if isempty(leftIndex), continue; end
            leftData = leftData(leftIndex,:);
            rightData = rightData(rightIndex,:);
            for metricIndex = 1:numel(metrics)
                rows(end+1) = pairedTimeBudgetMetric(leftData,rightData, ...
                    left,right,mapID,budget,metrics{metricIndex}); %#ok<AGROW>
            end
        end
    end
end
paired = struct2table(rows);
end

function row = emptyTimeBudgetPairedRow()
row = struct('comparison','', 'map',NaN, 'budget_ms',NaN, 'metric','', ...
    'nPairs',NaN, 'nBothSuccess',NaN, 'medianPairedDifference',NaN, ...
    'meanPairedDifference',NaN, 'wilcoxonP',NaN, ...
    'rankBiserialEffect',NaN, 'mcnemarP',NaN, ...
    'discordantLeftSuccessRightFailure',NaN, ...
    'discordantLeftFailureRightSuccess',NaN, ...
    'successDifference',NaN);
end

function row = pairedTimeBudgetMetric(leftData,rightData,left,right, ...
        mapID,budget,metric)
row = emptyTimeBudgetPairedRow();
row.comparison = [right ' minus ' left];
row.map = mapID;
row.budget_ms = budget;
row.metric = metric;
row.nPairs = height(leftData);
bothSuccess = leftData.successByBudget & rightData.successByBudget;
row.nBothSuccess = sum(bothSuccess);
if strcmp(metric,'SuccessByBudget')
    leftSuccess = sum(leftData.successByBudget);
    rightSuccess = sum(rightData.successByBudget);
    row.successDifference = rightSuccess-leftSuccess;
    leftFailRightSuccess = sum(~leftData.successByBudget & ...
        rightData.successByBudget);
    leftSuccessRightFail = sum(leftData.successByBudget & ...
        ~rightData.successByBudget);
    row.discordantLeftSuccessRightFailure = leftSuccessRightFail;
    row.discordantLeftFailureRightSuccess = leftFailRightSuccess;
    discordant = leftFailRightSuccess+leftSuccessRightFail;
    if discordant > 0
        row.mcnemarP = exactMcNemarP(leftFailRightSuccess, ...
            leftSuccessRightFail);
        row.rankBiserialEffect = ...
            (leftFailRightSuccess-leftSuccessRightFail)/discordant;
    end
    return;
end

switch metric
    case 'BestPathLength'
        leftValues = leftData.bestPathLength(bothSuccess);
        rightValues = rightData.bestPathLength(bothSuccess);
    case 'CollisionChecks'
        leftValues = leftData.collisionChecks(bothSuccess);
        rightValues = rightData.collisionChecks(bothSuccess);
    case 'NodeCount'
        leftValues = leftData.nodeCount(bothSuccess);
        rightValues = rightData.nodeCount(bothSuccess);
    otherwise
        leftValues = leftData.actualSnapshotTime_ms(bothSuccess);
        rightValues = rightData.actualSnapshotTime_ms(bothSuccess);
end
differences = rightValues-leftValues;
differences = differences(isfinite(differences));
if isempty(differences), return; end
row.medianPairedDifference = median(differences);
row.meanPairedDifference = mean(differences);
nonzero = differences(abs(differences) > 1e-12);
if isempty(nonzero), return; end
try
    row.wilcoxonP = signrank(nonzero,0,'method','approximate');
catch
    row.wilcoxonP = signrank(nonzero,0);
end
ranks = averageRanks(abs(nonzero));
row.rankBiserialEffect = ...
    (sum(ranks(nonzero > 0))-sum(ranks(nonzero < 0)))/sum(ranks);
end

function pareto = buildTimeBudgetPareto(summary)
rows = repmat(emptyTimeBudgetParetoRow(),0,1);
for mapID = unique(summary.map).'
    for budget = unique(summary.budget_ms).'
        subset = summary(summary.map == mapID & ...
            summary.budget_ms == budget,:);
        for i = 1:height(subset)
            dominated = false;
            for j = 1:height(subset)
                if i == j, continue; end
                iPath = subset.bestPathLengthMedian(i);
                jPath = subset.bestPathLengthMedian(j);
                iChecks = subset.collisionChecksMedian(i);
                jChecks = subset.collisionChecksMedian(j);
                if ~isfinite(jPath) || ~isfinite(jChecks), continue; end
                if ~isfinite(iPath)
                    if subset.successRateByBudget(j) > ...
                            subset.successRateByBudget(i)
                        dominated = true;
                        break;
                    end
                    continue;
                end
                successBetterOrEqual = subset.successRateByBudget(j) >= ...
                    subset.successRateByBudget(i);
                qualityBetterOrEqual = (isfinite(iPath) && ...
                    jPath <= iPath && jChecks <= iChecks);
                oneStrict = subset.successRateByBudget(j) > ...
                    subset.successRateByBudget(i) || ...
                    (isfinite(iPath) && jPath < iPath) || jChecks < iChecks;
                if successBetterOrEqual && qualityBetterOrEqual && oneStrict
                    dominated = true;
                    break;
                end
            end
            rows(end+1) = struct('variant',subset.variant{i}, ...
                'map',mapID,'mapName',subset.mapName{i}, ...
                'budget_ms',budget,'successRateByBudget', ...
                subset.successRateByBudget(i), ...
                'bestPathLengthMedian',subset.bestPathLengthMedian(i), ...
                'collisionChecksMedian',subset.collisionChecksMedian(i), ...
                'isPareto',~dominated); %#ok<AGROW>
        end
    end
end
pareto = struct2table(rows);
end

function row = emptyTimeBudgetParetoRow()
row = struct('variant','', 'map',NaN, 'mapName','', 'budget_ms',NaN, ...
    'successRateByBudget',NaN, 'bestPathLengthMedian',NaN, ...
    'collisionChecksMedian',NaN, 'isPareto',false);
end

function outputPath = makeTimeBudgetFigure(rawTable,map,variants,figureDirectory)
figureHandle = figure('Visible','off','Color','w');
budgets = unique(rawTable.budget_ms).';
subplot(1,3,1); hold on;
for i = 1:numel(variants)
    values = nan(size(budgets));
    for b = 1:numel(budgets)
        mask = strcmp(rawTable.variant,variants(i).label) & ...
            rawTable.map == map.id & rawTable.budget_ms == budgets(b) & ...
            rawTable.successByBudget;
        values(b) = medianFinite(rawTable.bestPathLength(mask));
    end
    plot(budgets,values,'-o','LineWidth',1.2,'DisplayName',variants(i).label);
end
grid on; box on; xlabel('Time budget (ms)'); ylabel('Best path length');
title('Best path vs budget');
subplot(1,3,2); hold on;
for i = 1:numel(variants)
    values = nan(size(budgets));
    for b = 1:numel(budgets)
        mask = strcmp(rawTable.variant,variants(i).label) & ...
            rawTable.map == map.id & rawTable.budget_ms == budgets(b);
        values(b) = mean(rawTable.successByBudget(mask));
    end
    plot(budgets,values,'-o','LineWidth',1.2,'DisplayName',variants(i).label);
end
grid on; box on; xlabel('Time budget (ms)'); ylabel('Success rate');
ylim([-0.02 1.02]); title('Success rate vs budget');
subplot(1,3,3); hold on;
for i = 1:numel(variants)
    values = nan(size(budgets));
    for b = 1:numel(budgets)
        mask = strcmp(rawTable.variant,variants(i).label) & ...
            rawTable.map == map.id & rawTable.budget_ms == budgets(b) & ...
            rawTable.successByBudget;
        values(b) = medianFinite(rawTable.collisionChecks(mask));
    end
    plot(budgets,values,'-o','LineWidth',1.2,'DisplayName',variants(i).label);
end
grid on; box on; xlabel('Time budget (ms)'); ylabel('Collision checks');
title('Collision checks vs budget');
legend('Location','bestoutside');
sgtitle(sprintf('Phase 11B fixed time budget - Map %d',map.id));
outputPath = fullfile(figureDirectory,sprintf('Map%d_time_budget.png',map.id));
exportgraphics(figureHandle,outputPath,'Resolution',300);
close(figureHandle);
end

function outputPath = makeParetoFigure(summary,map,figureDirectory)
figureHandle = figure('Visible','off','Color','w');
budgets = unique(summary.budget_ms).';
colors = lines(numel(budgets));
hold on;
for b = 1:numel(budgets)
    subset = summary(summary.map == map.id & ...
        summary.budget_ms == budgets(b),:);
    scatter(subset.bestPathLengthMedian,subset.collisionChecksMedian, ...
        55,colors(b,:),'filled','DisplayName',sprintf('%d ms',budgets(b)));
    for i = 1:height(subset)
        if isfinite(subset.bestPathLengthMedian(i)) && ...
                isfinite(subset.collisionChecksMedian(i))
            text(subset.bestPathLengthMedian(i), ...
                subset.collisionChecksMedian(i),['  ' subset.variant{i}], ...
                'FontSize',7,'Interpreter','none');
        end
    end
end
grid on; box on; xlabel('Median best path length');
ylabel('Median collision checks');
title(sprintf('Time-path-cost Pareto view - Map %d',map.id));
legend('Location','bestoutside');
outputPath = fullfile(figureDirectory, ...
    sprintf('Map%d_TimePath_Pareto.png',map.id));
exportgraphics(figureHandle,outputPath,'Resolution',300);
close(figureHandle);
end

function printTimeBudgetAcceptance(rawTable,runCount,pilotElapsed)
fprintf('\nPhase 11B acceptance summary:\n');
fprintf('Planner runs: %d; snapshot rows: %d\n',runCount,height(rawTable));
fprintf('Success rows: %d/%d\n',sum(rawTable.successByBudget),height(rawTable));
fprintf('Path-validity failures: %d\n', ...
    sum(rawTable.successByBudget & ~rawTable.pathCollisionFree));
fprintf('Anomaly rows: %d\n',sum(string(rawTable.anomaly) ~= "NONE"));
fprintf('Max budget overshoot median: %.3f ms\n', ...
    medianFinite(rawTable.budgetOvershoot_ms));
fprintf('Total Phase 11B wall time: %.3f s\n',pilotElapsed);
for mapID = unique(rawTable.map).'
    for budget = unique(rawTable.budget_ms).'
        mask = rawTable.map == mapID & rawTable.budget_ms == budget;
        fprintf('Map%d @ %g ms: success=%d/%d\n',mapID,budget, ...
            sum(rawTable.successByBudget(mask)),sum(mask));
    end
end
end

function p = exactMcNemarP(wins,losses)
n = wins+losses;
if n == 0
    p = NaN;
    return;
end
k = min(wins,losses);
probability = 0;
for i = 0:k
    probability = probability+nchoosek(n,i)*0.5^n;
end
p = min(1,2*probability);
end

function ranks = averageRanks(values)
[sorted,order] = sort(values(:));
ranksSorted = zeros(size(sorted));
startIndex = 1;
while startIndex <= numel(sorted)
    endIndex = startIndex;
    while endIndex < numel(sorted) && ...
            sorted(endIndex+1) == sorted(startIndex)
        endIndex = endIndex+1;
    end
    ranksSorted(startIndex:endIndex) = mean(startIndex:endIndex);
    startIndex = endIndex+1;
end
ranks = zeros(size(ranksSorted));
ranks(order) = ranksSorted;
end

function ci = bootstrapMedianCI(values,repetitions,seed)
values = values(isfinite(values));
if isempty(values)
    ci = [NaN NaN];
    return;
end
if numel(values) == 1
    ci = [values values];
    return;
end
oldState = rng;
rng(seed,'twister');
indices = randi(numel(values),numel(values),repetitions);
bootstrapMedians = median(values(indices),1);
rng(oldState);
ci = prctile(bootstrapMedians,[2.5 97.5]);
end

function value = getNumeric(record,fieldName,defaultValue)
if isfield(record,fieldName) && ~isempty(record.(fieldName)) && ...
        isscalar(record.(fieldName)) && isnumeric(record.(fieldName))
    value = double(record.(fieldName));
else
    value = defaultValue;
end
end

function value = medianFinite(values)
values = values(isfinite(values));
if isempty(values), value = NaN; else, value = median(values); end
end
