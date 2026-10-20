function outputs = run_phase11a_ablation()
% run_phase11a_ablation  Phase 11A first-solution mechanism ablation.
%
% Map 3/4 use 100 seeds and Map 1/2 use 20-seed sanity checks.  Every
% variant stops at its first feasible path; no optimization or smoothing is
% included in the primary metrics.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

maps = phase10Maps();
seedsByMap = {1:20,1:20,1:100,1:100};
maxIterations = 5000;
stepSize = 1;
nearRadiusMax = 25;

variants = ablationDefinitions();
resultsDirectory = fullfile(projectRoot,'results');
figureDirectory = fullfile(resultsDirectory,'benchmark_figures', ...
    'phase11a_ablation');
if ~exist(resultsDirectory,'dir'), mkdir(resultsDirectory); end
if ~exist(figureDirectory,'dir'), mkdir(figureDirectory); end

for variantIndex = 1:numel(variants)
    warmupConfig = makeAblationConfig(maps(1),1,100,stepSize, ...
        nearRadiusMax,variants(variantIndex));
    try
        variants(variantIndex).handle(warmupConfig);
    catch warmupError
        warning('Phase11A:WarmupFailed','%s warm-up failed: %s', ...
            variants(variantIndex).label,warmupError.message);
    end
end

runCount = sum(cellfun(@numel,seedsByMap))*numel(variants);
rawRows = repmat(emptyAblationRow(),runCount,1);
rowIndex = 0;
pilotTimer = tic;

for mapIndex = 1:numel(maps)
    seeds = seedsByMap{mapIndex};
    for seedIndex = 1:numel(seeds)
        seed = seeds(seedIndex);
        for variantIndex = 1:numel(variants)
            variant = variants(variantIndex);
            config = makeAblationConfig(maps(mapIndex),seed, ...
                maxIterations,stepSize,nearRadiusMax,variant);
            runTimer = tic;
            runError = '';
            try
                result = variant.handle(config);
            catch plannerError
                result = makeFailureResult(variant.label,config, ...
                    plannerError.message);
                runError = plannerError.message;
            end
            rowIndex = rowIndex+1;
            rawRows(rowIndex) = makeAblationRow(result,variant, ...
                maps(mapIndex),seed,config,toc(runTimer),runError);
        end
        if mod(seedIndex,10) == 0 || seedIndex == numel(seeds)
            fprintf('Phase11A map %d/%d seed %d/%d complete (%d/%d runs).\n', ...
                mapIndex,numel(maps),seedIndex,numel(seeds),rowIndex,runCount);
        end
    end
end

pilotElapsed = toc(pilotTimer);
rawRows = rawRows(1:rowIndex);
rawTable = struct2table(rawRows);
summaryTable = buildAblationSummary(rawTable);
pairedTable = buildPairedStatistics(rawTable);
anomalyTable = rawTable(string(rawTable.anomaly) ~= "NONE",:);

rawFile = fullfile(resultsDirectory,'phase11a_ablation_raw_results.csv');
summaryFile = fullfile(resultsDirectory,'phase11a_ablation_summary.csv');
pairedFile = fullfile(resultsDirectory,'phase11a_paired_statistics.csv');
anomalyFile = fullfile(resultsDirectory,'phase11a_ablation_anomalies.csv');
writetable(rawTable,rawFile);
writetable(summaryTable,summaryFile);
writetable(pairedTable,pairedFile);
writetable(anomalyTable,anomalyFile);
save(fullfile(resultsDirectory,'phase11a_ablation.mat'), ...
    'rawTable','summaryTable','pairedTable','anomalyTable', ...
    'maps','seedsByMap','pilotElapsed');

figureResults = {};
for mapIndex = 3:4
    figureResults{end+1} = makeAblationFigure(rawTable,maps(mapIndex), ...
        variants,figureDirectory); %#ok<AGROW>
end

outputs = struct('rawTable',rawTable,'summaryTable',summaryTable, ...
    'pairedTable',pairedTable,'anomalyTable',anomalyTable, ...
    'maps',maps,'seedsByMap',{seedsByMap}, ...
    'figureResults',{figureResults},'totalRunCount',height(rawTable), ...
    'pilotElapsedSeconds',pilotElapsed);

fprintf('\nPhase 11A ablation finished: %d runs in %.3f s.\n', ...
    height(rawTable),pilotElapsed);
printAcceptance(rawTable,pilotElapsed);
end

function variants = ablationDefinitions()
variants = struct('family',{},'label',{},'handle',{}, ...
    'useDynamicGoalBias',{},'useAPFForce',{},'useOffset',{}, ...
    'useClassificationGoalTree',{},'useBacktracking',{}, ...
    'useSequentialRewire',{});

variants(end+1) = makeVariant('APF','Bi-RRT*',@Bi_RRT_star, ...
    NaN,NaN,NaN,NaN,NaN,NaN);
variants(end+1) = makeVariant('APF','Bias-only Bi-RRT*',@APF_Bi_RRT_star, ...
    true,false,NaN,NaN,NaN,NaN);
variants(end+1) = makeVariant('APF','APF-only Bi-RRT*',@APF_Bi_RRT_star, ...
    false,true,NaN,NaN,NaN,NaN);
variants(end+1) = makeVariant('APF','Bias+APF Bi-RRT*',@APF_Bi_RRT_star, ...
    true,true,NaN,NaN,NaN,NaN);

variants(end+1) = makeVariant('Classified','Full',@Classified_RRT_star, ...
    NaN,NaN,true,true,true,true);
variants(end+1) = makeVariant('Classified','No Offset',@Classified_RRT_star, ...
    NaN,NaN,false,true,true,true);
variants(end+1) = makeVariant('Classified', ...
    'No Classification+GoalTree',@Classified_RRT_star, ...
    NaN,NaN,true,false,true,true);
variants(end+1) = makeVariant('Classified','No Backtracking', ...
    @Classified_RRT_star,NaN,NaN,true,true,false,true);
variants(end+1) = makeVariant('Classified','No Sequential Rewire', ...
    @Classified_RRT_star,NaN,NaN,true,true,true,false);
end

function variant = makeVariant(family,label,handle,useBias,useForce, ...
        useOffset,useClassification,useBacktracking,useSequential)
variant = struct('family',family,'label',label,'handle',handle, ...
    'useDynamicGoalBias',useBias,'useAPFForce',useForce, ...
    'useOffset',useOffset,'useClassificationGoalTree',useClassification, ...
    'useBacktracking',useBacktracking,'useSequentialRewire',useSequential);
end

function config = makeAblationConfig(map,seed,maxIterations,stepSize, ...
        nearRadiusMax,variant)
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
config.crrtPaperMode = false;
config.apfPaperMode = false;
config.apfPaperConnectionMode = 'best_near';

if strcmp(variant.family,'APF') && ~isnan(variant.useDynamicGoalBias)
    config.ablationUseDynamicGoalBias = variant.useDynamicGoalBias;
    config.ablationUseAPFForce = variant.useAPFForce;
end
if strcmp(variant.family,'Classified')
    config.ablationUseOffset = variant.useOffset;
    config.ablationUseClassificationGoalTree = ...
        variant.useClassificationGoalTree;
    config.ablationUseBacktracking = variant.useBacktracking;
    config.ablationUseSequentialRewire = variant.useSequentialRewire;
end
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

function row = emptyAblationRow()
row = struct('family','', 'variant','', 'map',NaN, 'mapName','', ...
    'seed',NaN, 'success',false, 'pathCollisionFree',false, ...
    'firstSolutionIteration',NaN, 'firstSolutionTime_ms',NaN, ...
    'firstPathLength',NaN, 'planningTime_ms',NaN, 'nodeCount',NaN, ...
    'collisionChecks',NaN, 'rewireCount',NaN, ...
    'goalTreeConnectionChecks',NaN, 'offsetSuccessCount',NaN, ...
    'backtrackChecks',NaN, 'dynamicGoalBiasUsed',false, ...
    'apfForceUsed',false, 'offsetUsed',false, ...
    'classificationGoalTreeUsed',false, 'backtrackingUsed',false, ...
    'sequentialRewireUsed',false, 'maxIterationsTruncated',false, ...
    'anomaly','NONE', 'wallTime_ms',NaN, 'errorMessage','');
end

function row = makeAblationRow(result,variant,map,seed,config, ...
        wallElapsed,runError)
row = emptyAblationRow();
row.family = variant.family;
row.variant = variant.label;
row.map = map.id;
row.mapName = map.name;
row.seed = seed;
row.wallTime_ms = wallElapsed*1000;
row.errorMessage = runError;
row.success = getLogical(result,'success',false);
row.firstSolutionIteration = getNumeric(result,'firstSolutionIteration',NaN);
row.firstSolutionTime_ms = getNumeric(result,'firstSolutionTime',NaN)*1000;
row.firstPathLength = getNumeric(result,'firstPathLength',NaN);
if isfield(result,'searchPlanningTime')
    row.planningTime_ms = result.searchPlanningTime*1000;
else
    row.planningTime_ms = getNumeric(result,'planningTime',NaN)*1000;
end
row.nodeCount = getNumeric(result,'nodeCount',NaN);
if strcmp(variant.family,'APF') && isfield(result,'searchCollisionChecks')
    row.collisionChecks = getNumeric(result,'searchCollisionChecks',NaN);
else
    row.collisionChecks = getNumeric(result,'collisionChecks',NaN);
end
row.rewireCount = getNumeric(result,'rewireCount',0);
row.goalTreeConnectionChecks = getNumeric(result, ...
    'goalTreeConnectionChecks',NaN);
row.offsetSuccessCount = getNumeric(result,'offsetSuccessCount',NaN);
row.backtrackChecks = getNumeric(result,'backtrackChecks',NaN);
row.dynamicGoalBiasUsed = logical(~isnan(variant.useDynamicGoalBias) && ...
    variant.useDynamicGoalBias);
row.apfForceUsed = logical(~isnan(variant.useAPFForce) && ...
    variant.useAPFForce);
row.offsetUsed = logical(~isnan(variant.useOffset) && variant.useOffset);
row.classificationGoalTreeUsed = logical(~isnan( ...
    variant.useClassificationGoalTree) && variant.useClassificationGoalTree);
row.backtrackingUsed = logical(~isnan(variant.useBacktracking) && ...
    variant.useBacktracking);
row.sequentialRewireUsed = logical(~isnan( ...
    variant.useSequentialRewire) && variant.useSequentialRewire);
if row.success && isfield(result,'firstPath') && ~isempty(result.firstPath)
    row.pathCollisionFree = validatePath(result.firstPath,config);
end
row.maxIterationsTruncated = ~row.success && ...
    getNumeric(result,'iterations',NaN) >= config.maxIterations;

issues = {};
if ~isempty(runError), issues{end+1} = 'PLANNER_EXCEPTION'; end %#ok<AGROW>
if ~row.success, issues{end+1} = 'FAILURE'; end %#ok<AGROW>
if row.success && ~row.pathCollisionFree
    issues{end+1} = 'PATH_COLLISION'; %#ok<AGROW>
end
required = [row.firstSolutionIteration,row.firstSolutionTime_ms, ...
    row.firstPathLength,row.planningTime_ms,row.nodeCount, ...
    row.collisionChecks];
if row.success && any(~isfinite(required))
    issues{end+1} = 'REQUIRED_NAN_OR_INF'; %#ok<AGROW>
end
if row.maxIterationsTruncated
    issues{end+1} = 'MAX_ITERATIONS_TRUNCATED'; %#ok<AGROW>
end
if isempty(issues), row.anomaly = 'NONE';
else, row.anomaly = strjoin(issues,';');
end
end

function summary = buildAblationSummary(rawTable)
families = unique(rawTable.family,'stable');
variants = unique(rawTable.variant,'stable');
mapIDs = unique(rawTable.map,'stable');
rows = repmat(emptySummaryRow(),numel(families)*numel(variants)* ...
    numel(mapIDs),1);
index = 0;
for familyIndex = 1:numel(families)
    for variantIndex = 1:numel(variants)
        for mapIndex = 1:numel(mapIDs)
            mask = strcmp(rawTable.family,families{familyIndex}) & ...
                strcmp(rawTable.variant,variants{variantIndex}) & ...
                rawTable.map == mapIDs(mapIndex);
            if ~any(mask), continue; end
            index = index+1;
            rows(index) = summarizeSubset(rawTable(mask,:), ...
                families{familyIndex},variants{variantIndex},mapIDs(mapIndex));
        end
    end
end
summary = struct2table(rows(1:index));
end

function row = emptySummaryRow()
row = struct('family','', 'variant','', 'map',NaN, 'mapName','', ...
    'nRuns',NaN, 'nSuccess',NaN, 'nFailure',NaN, 'successRate',NaN, ...
    'firstSolutionTimeMedian_ms',NaN, 'firstPathLengthMedian',NaN, ...
    'nodeCountMedian',NaN, 'collisionChecksMedian',NaN, ...
    'firstSolutionIterationMedian',NaN, 'rewireCountMedian',NaN, ...
    'goalTreeConnectionChecksMedian',NaN, 'offsetSuccessCountMedian',NaN, ...
    'backtrackChecksMedian',NaN, 'maxIterationsTruncatedCount',NaN);
end

function row = summarizeSubset(subset,family,variant,mapID)
row = emptySummaryRow();
row.family = family;
row.variant = variant;
row.map = mapID;
row.mapName = subset.mapName{1};
row.nRuns = height(subset);
row.nSuccess = sum(subset.success);
row.nFailure = row.nRuns-row.nSuccess;
row.successRate = row.nSuccess/row.nRuns;
success = subset.success == 1;
row.firstSolutionTimeMedian_ms = medianFinite( ...
    subset.firstSolutionTime_ms(success));
row.firstPathLengthMedian = medianFinite(subset.firstPathLength(success));
row.nodeCountMedian = medianFinite(subset.nodeCount(success));
row.collisionChecksMedian = medianFinite(subset.collisionChecks(success));
row.firstSolutionIterationMedian = medianFinite( ...
    subset.firstSolutionIteration(success));
row.rewireCountMedian = medianFinite(subset.rewireCount(success));
row.goalTreeConnectionChecksMedian = medianFinite( ...
    subset.goalTreeConnectionChecks(success));
row.offsetSuccessCountMedian = medianFinite( ...
    subset.offsetSuccessCount(success));
row.backtrackChecksMedian = medianFinite(subset.backtrackChecks(success));
row.maxIterationsTruncatedCount = sum(subset.maxIterationsTruncated);
end

function paired = buildPairedStatistics(rawTable)
families = {'APF','Classified'};
rows = repmat(emptyPairedRow(),0,1);
for familyIndex = 1:numel(families)
    family = families{familyIndex};
    familyData = rawTable(strcmp(rawTable.family,family),:);
    if strcmp(family,'APF')
        fullLabel = 'Bi-RRT*';
    else
        fullLabel = 'Full';
    end
    fullData = familyData(strcmp(familyData.variant,fullLabel),:);
    variants = unique(familyData.variant,'stable');
    variants = variants(~strcmp(variants,fullLabel));
    for variantIndex = 1:numel(variants)
        variantLabel = variants{variantIndex};
        variantData = familyData(strcmp(familyData.variant,variantLabel),:);
        for mapID = unique(familyData.map).'
            fullMap = fullData(fullData.map == mapID,:);
            variantMap = variantData(variantData.map == mapID,:);
            [commonSeeds,fullIndex,variantIndex2] = intersect( ...
                fullMap.seed,variantMap.seed);
            if isempty(commonSeeds), continue; end
            fullMap = fullMap(fullIndex,:);
            variantMap = variantMap(variantIndex2,:);
            successPair = fullMap.success & variantMap.success;
            rows(end+1) = pairedMetric(fullMap,variantMap,successPair, ...
                family,variantLabel,mapID,'FirstSolutionTime_ms'); %#ok<AGROW>
            rows(end+1) = pairedMetric(fullMap,variantMap,successPair, ...
                family,variantLabel,mapID,'FirstPathLength'); %#ok<AGROW>
        end
    end
end
paired = struct2table(rows);
end

function row = emptyPairedRow()
row = struct('family','', 'comparison','', 'map',NaN, 'metric','', ...
    'nPairs',NaN, 'nBothSuccess',NaN, 'medianPairedDifference',NaN, ...
    'meanPairedDifference',NaN, 'wilcoxonP',NaN, ...
    'rankBiserialEffect',NaN);
end

function row = pairedMetric(fullData,variantData,successPair, ...
        family,variant,mapID,metric)
row = emptyPairedRow();
row.family = family;
row.comparison = [variant ' minus full'];
row.map = mapID;
row.metric = metric;
row.nPairs = height(fullData);
row.nBothSuccess = sum(successPair);
if strcmp(metric,'FirstSolutionTime_ms')
    fullValues = fullData.firstSolutionTime_ms(successPair);
    variantValues = variantData.firstSolutionTime_ms(successPair);
else
    fullValues = fullData.firstPathLength(successPair);
    variantValues = variantData.firstPathLength(successPair);
end
differences = variantValues-fullValues;
differences = differences(isfinite(differences));
if isempty(differences), return; end
row.medianPairedDifference = median(differences);
row.meanPairedDifference = mean(differences);
nonzero = differences(abs(differences) > 1e-12);
if ~isempty(nonzero)
    try
        row.wilcoxonP = signrank(nonzero,0,'method','approximate');
    catch
        row.wilcoxonP = signrank(nonzero,0);
    end
    ranks = averageRanks(abs(nonzero));
    row.rankBiserialEffect = ...
        (sum(ranks(nonzero > 0))-sum(ranks(nonzero < 0)))/sum(ranks);
end
end

function ranks = averageRanks(values)
[sorted,order] = sort(values(:)); %#ok<ASGLU>
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

function outputPath = makeAblationFigure(rawTable,map,variants,figureDirectory)
figureHandle = figure('Visible','off','Color','w');
mapData = rawTable(rawTable.map == map.id,:);
familyNames = {'APF','Classified'};
for familyIndex = 1:2
    subplot(2,2,(familyIndex-1)*2+1);
    familyData = mapData(strcmp(mapData.family,familyNames{familyIndex}),:);
    labels = unique(familyData.variant,'stable');
    values = [];
    groups = [];
    for i = 1:numel(labels)
        subset = familyData.firstSolutionTime_ms( ...
            strcmp(familyData.variant,labels{i}));
        values = [values;subset]; %#ok<AGROW>
        groups = [groups;repmat(i,numel(subset),1)]; %#ok<AGROW>
    end
    boxplot(values,groups,'Labels',labels);
    xtickangle(25); grid on; box on;
    ylabel('First solution time (ms)');
    title([familyNames{familyIndex} ' time']);

    subplot(2,2,(familyIndex-1)*2+2);
    values = [];
    groups = [];
    for i = 1:numel(labels)
        subset = familyData.firstPathLength( ...
            strcmp(familyData.variant,labels{i}));
        values = [values;subset]; %#ok<AGROW>
        groups = [groups;repmat(i,numel(subset),1)]; %#ok<AGROW>
    end
    boxplot(values,groups,'Labels',labels);
    xtickangle(25); grid on; box on;
    ylabel('First path length');
    title([familyNames{familyIndex} ' path']);
end
sgtitle(sprintf('Phase 11A mechanism ablation - Map %d',map.id));
outputPath = fullfile(figureDirectory,sprintf('Map%d_ablation.png',map.id));
exportgraphics(figureHandle,outputPath,'Resolution',300);
close(figureHandle);
end

function printAcceptance(rawTable,pilotElapsed)
fprintf('\nPhase 11A acceptance summary:\n');
groups = unique(strcat(string(rawTable.family),"|", ...
    string(rawTable.variant)),'stable');
for i = 1:numel(groups)
    mask = strcat(string(rawTable.family),"|", ...
        string(rawTable.variant)) == groups(i);
    fprintf('%-38s success=%d/%d anomalies=%d\n',char(groups(i)), ...
        sum(rawTable.success(mask)),sum(mask), ...
        sum(string(rawTable.anomaly(mask)) ~= "NONE"));
end
fprintf('Path-validity failures: %d\n', ...
    sum(rawTable.success & ~rawTable.pathCollisionFree));
fprintf('Max-iteration truncations: %d\n',sum(rawTable.maxIterationsTruncated));
fprintf('Total Phase 11A wall time: %.3f s\n',pilotElapsed);
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

function value = medianFinite(values)
values = values(isfinite(values));
if isempty(values), value = NaN; else, value = median(values); end
end
