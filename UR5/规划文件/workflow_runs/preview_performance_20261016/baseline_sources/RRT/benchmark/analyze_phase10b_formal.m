function outputs = analyze_phase10b_formal()
% analyze_phase10b_formal  Analyze completed Phase 10B first-solution data.
%
% The primary result remains the existing Map x Algorithm summary.  This
% helper adds a pooled diagnostic table and a per-map Pareto flag; it does
% not rerun planners or alter any algorithm configuration.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
resultsDirectory = fullfile(projectRoot,'results');
figureDirectory = fullfile(resultsDirectory,'benchmark_figures', ...
    'formal_first_solution');
rawTable = readtable(fullfile(resultsDirectory, ...
    'formal_first_solution_raw_results.csv'));
summaryTable = readtable(fullfile(resultsDirectory, ...
    'formal_first_solution_summary.csv'));

pooledTable = buildPooledTable(rawTable);
paretoTable = buildParetoTable(summaryTable);
writetable(pooledTable,fullfile(resultsDirectory, ...
    'formal_first_solution_pooled_diagnostic.csv'));
writetable(paretoTable,fullfile(resultsDirectory, ...
    'formal_first_solution_pareto.csv'));
paretoFigure = makeParetoFigure(paretoTable,figureDirectory);

outputs = struct('pooledTable',pooledTable,'paretoTable',paretoTable, ...
    'paretoFigure',paretoFigure);

fprintf('Formal pooled diagnostic written: %s\n', ...
    fullfile(resultsDirectory,'formal_first_solution_pooled_diagnostic.csv'));
fprintf('Formal Pareto table written: %s\n', ...
    fullfile(resultsDirectory,'formal_first_solution_pareto.csv'));
fprintf('Formal Pareto figure written: %s\n',paretoFigure);
fprintf('Pareto points by map: ');
for mapID = unique(paretoTable.map).'
    fprintf('Map%d=%d ',mapID, ...
        sum(paretoTable.map == mapID & paretoTable.paretoOptimal));
end
fprintf('\n');
end

function pooled = buildPooledTable(rawTable)
algorithms = unique(rawTable.algorithm,'stable');
rows = repmat(emptyPooledRow(),numel(algorithms),1);
for i = 1:numel(algorithms)
    subset = rawTable(strcmp(rawTable.algorithm,algorithms{i}),:);
    rows(i) = summarizePooledSubset(subset,algorithms{i});
end
pooled = struct2table(rows);
end

function row = emptyPooledRow()
row = struct('algorithm','', 'nRuns',NaN, 'nSuccess',NaN, ...
    'nFailure',NaN, 'successRate',NaN, ...
    'firstSolutionTimeMean_ms',NaN, 'firstSolutionTimeStd_ms',NaN, ...
    'firstSolutionTimeMedian_ms',NaN, 'firstSolutionTimeIQR_ms',NaN, ...
    'firstSolutionTimeCI95Low_ms',NaN, ...
    'firstSolutionTimeCI95High_ms',NaN, ...
    'firstPathLengthMedian',NaN, 'nodeCountMedian',NaN, ...
    'collisionChecksMedian',NaN);
end

function row = summarizePooledSubset(subset,algorithm)
row = emptyPooledRow();
row.algorithm = algorithm;
row.nRuns = height(subset);
row.nSuccess = sum(subset.success);
row.nFailure = row.nRuns-row.nSuccess;
row.successRate = row.nSuccess/row.nRuns;
success = subset.success == 1;
timeValues = subset.firstSolutionTime_ms(success);
timeValues = timeValues(isfinite(timeValues));
row.firstSolutionTimeMean_ms = mean(timeValues);
row.firstSolutionTimeStd_ms = std(timeValues,0);
row.firstSolutionTimeMedian_ms = median(timeValues);
quartiles = prctile(timeValues,[25 75]);
row.firstSolutionTimeIQR_ms = quartiles(2)-quartiles(1);
rng(20260924,'twister');
bootstrapIndex = randi(numel(timeValues),1000,numel(timeValues));
bootstrapMeans = mean(timeValues(bootstrapIndex),2);
limits = prctile(bootstrapMeans,[2.5 97.5]);
row.firstSolutionTimeCI95Low_ms = limits(1);
row.firstSolutionTimeCI95High_ms = limits(2);
row.firstPathLengthMedian = medianFinite(subset.firstPathLength(success));
row.nodeCountMedian = medianFinite(subset.nodeCount(success));
row.collisionChecksMedian = medianFinite(subset.collisionChecks(success));
end

function paretoTable = buildParetoTable(summaryTable)
rowCount = height(summaryTable);
rows = repmat(emptyParetoRow(),rowCount,1);
for i = 1:rowCount
    rows(i).algorithm = summaryTable.algorithm{i};
    rows(i).map = summaryTable.map(i);
    rows(i).mapName = summaryTable.mapName{i};
    rows(i).successRate = summaryTable.successRate(i);
    rows(i).firstSolutionTimeMedian_ms = ...
        summaryTable.firstSolutionTimeMedian_ms(i);
    rows(i).firstPathLengthMedian = summaryTable.firstPathLengthMedian(i);
    rows(i).nodeCountMedian = summaryTable.nodeCountMedian(i);
    rows(i).collisionChecksMedian = summaryTable.collisionChecksMedian(i);
end

for mapID = unique(summaryTable.map).'
    indices = find(summaryTable.map == mapID);
    for localIndex = 1:numel(indices)
        candidateIndex = indices(localIndex);
        candidate = rows(candidateIndex);
        dominated = false;
        for otherLocalIndex = 1:numel(indices)
            otherIndex = indices(otherLocalIndex);
            if otherIndex == candidateIndex, continue; end
            other = rows(otherIndex);
            noWorse = other.successRate >= candidate.successRate && ...
                other.firstSolutionTimeMedian_ms <= ...
                candidate.firstSolutionTimeMedian_ms && ...
                other.firstPathLengthMedian <= candidate.firstPathLengthMedian && ...
                other.nodeCountMedian <= candidate.nodeCountMedian && ...
                other.collisionChecksMedian <= candidate.collisionChecksMedian;
            strictlyBetter = other.successRate > candidate.successRate || ...
                other.firstSolutionTimeMedian_ms < ...
                candidate.firstSolutionTimeMedian_ms || ...
                other.firstPathLengthMedian < candidate.firstPathLengthMedian || ...
                other.nodeCountMedian < candidate.nodeCountMedian || ...
                other.collisionChecksMedian < candidate.collisionChecksMedian;
            if noWorse && strictlyBetter
                dominated = true;
                break;
            end
        end
        rows(candidateIndex).paretoOptimal = ~dominated;
    end
end
paretoTable = struct2table(rows);
end

function row = emptyParetoRow()
row = struct('algorithm','', 'map',NaN, 'mapName','', ...
    'successRate',NaN, 'firstSolutionTimeMedian_ms',NaN, ...
    'firstPathLengthMedian',NaN, 'nodeCountMedian',NaN, ...
    'collisionChecksMedian',NaN, 'paretoOptimal',false);
end

function outputPath = makeParetoFigure(paretoTable,figureDirectory)
figureHandle = figure('Visible','off','Color','w');
mapIDs = unique(paretoTable.map).';
for plotIndex = 1:numel(mapIDs)
    subplot(2,2,plotIndex);
    subset = paretoTable(paretoTable.map == mapIDs(plotIndex),:);
    hold on; grid on; box on;
    dominated = ~subset.paretoOptimal;
    scatter(subset.firstSolutionTimeMedian_ms(dominated), ...
        subset.firstPathLengthMedian(dominated),45,[0.60 0.60 0.60],'filled');
    scatter(subset.firstSolutionTimeMedian_ms(~dominated), ...
        subset.firstPathLengthMedian(~dominated),75,[0.85 0.20 0.10],'filled');
    for i = 1:height(subset)
        text(subset.firstSolutionTimeMedian_ms(i), ...
            subset.firstPathLengthMedian(i),['  ' subset.algorithm{i}], ...
            'FontSize',7,'Interpreter','none');
    end
    xlabel('Median first solution time (ms)');
    ylabel('Median first path length');
    title(sprintf('Map %d',mapIDs(plotIndex)));
    legend({'Dominated','Pareto'},'Location','best');
end
sgtitle('Phase 10B first-solution Pareto diagnostics');
outputPath = fullfile(figureDirectory,'formal_first_solution_pareto.png');
exportgraphics(figureHandle,outputPath,'Resolution',300);
close(figureHandle);
end

function value = medianFinite(values)
values = values(isfinite(values));
if isempty(values), value = NaN; else, value = median(values); end
end
