function report = summarize_fair_weld_benchmark(csvFile, outputDir)
%SUMMARIZE_FAIR_WELD_BENCHMARK Summarize fair cold-start planner runs.
%
% This is an analysis-only script. It does not run or modify either planner.
% It reads the CSV produced by benchmark_weld_planners_fair and writes:
%   - a text summary;
%   - success-rate, first-solution-time and IK-count figures;
%   - a failure-point histogram.

if nargin < 1 || isempty(csvFile)
    csvFile = fullfile(fileparts(mfilename('fullpath')), ...
        'fair_benchmark_30.csv');
end
if nargin < 2 || isempty(outputDir)
    outputDir = fullfile(fileparts(csvFile), 'fair_benchmark_30', 'plots');
end

if ~isfile(csvFile)
    error('CSV file not found: %s', csvFile);
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

T = readtable(csvFile, 'TextType', 'string');
algorithms = unique(T.algorithm, 'stable');
budgets = [10 20 30 60];

report = struct();
report.csvFile = csvFile;
report.outputDir = outputDir;
report.algorithms = algorithms;
report.budgets = budgets;

summaryRows = strings(0, 9);
fprintf('\nFAIR WELD BENCHMARK SUMMARY\n');
fprintf('CSV: %s\n\n', csvFile);

for ia = 1:numel(algorithms)
    alg = algorithms(ia);
    mask = T.algorithm == alg;
    complete = mask & logical(T.complete);
    first = T.firstFeasibleTimeSec(complete);
    first = first(isfinite(first));

    successCounts = zeros(1, numel(budgets));
    for ib = 1:numel(budgets)
        successCounts(ib) = sum(complete & ...
            isfinite(T.firstFeasibleTimeSec) & ...
            T.firstFeasibleTimeSec <= budgets(ib));
    end

    ik = T.ikCalls(mask);
    edge = T.edgeTests(mask);
    ikRate = T.ikSuccessRatePct(mask);
    failPoints = T.failurePoint(mask & ~logical(T.complete));
    failPoints = failPoints(isfinite(failPoints));

    if isempty(first)
        medTime = NaN; p90Time = NaN; meanTime = NaN;
    else
        medTime = median(first);
        p90Time = prctile(first, 90);
        meanTime = mean(first);
    end

    passRows = mask & logical(T.complete);
    maxJump = T.maxJointJumpDeg(passRows);
    segTcp = T.maxSegmentTcpErrorMm(passRows);
    nodeTcp = T.maxNodeTcpErrorMm(passRows);
    maxJump = maxJump(isfinite(maxJump));
    segTcp = segTcp(isfinite(segTcp));
    nodeTcp = nodeTcp(isfinite(nodeTcp));

    fprintf('%s\n', alg);
    fprintf('  runs/pass                 : %d/%d (%.1f%%)\n', ...
        sum(complete), sum(mask), 100*sum(complete)/sum(mask));
    fprintf('  success @ 10/20/30/60 s   : %s\n', ...
        strjoin(string(successCounts), '/'));
    fprintf('  first solution median/P90 : %.4f / %.4f s\n', medTime, p90Time);
    fprintf('  first solution mean       : %.4f s\n', meanTime);
    fprintf('  IK calls mean/median      : %.1f / %.1f\n', mean(ik), median(ik));
    fprintf('  edge tests mean/median    : %.1f / %.1f\n', mean(edge), median(edge));
    fprintf('  IK success rate mean      : %.2f%%\n', mean(ikRate, 'omitnan'));
    fprintf('  pass max jump median/max  : %.4f / %.4f deg\n', ...
        median(maxJump), max(maxJump));
    fprintf('  pass segment TCP med/max  : %.4f / %.4f mm\n', ...
        median(segTcp), max(segTcp));
    fprintf('  pass node TCP med/max     : %.6f / %.6f mm\n', ...
        median(nodeTcp), max(nodeTcp));
    if isempty(failPoints)
        fprintf('  failure points            : none\n\n');
    else
        fprintf('  failure points            : %s\n\n', ...
            strjoin(string(failPoints.'), ', '));
    end

    summaryRows(end+1,:) = [alg, string(sum(mask)), string(sum(complete)), ...
        string(successCounts(1)), string(successCounts(2)), ...
        string(successCounts(3)), string(successCounts(4)), ...
        string(medTime), string(p90Time)]; %#ok<AGROW>

    report.(matlab.lang.makeValidName(alg)).successCounts = successCounts;
    report.(matlab.lang.makeValidName(alg)).firstSolutionTimes = first;
    report.(matlab.lang.makeValidName(alg)).failurePoints = failPoints;
end

summaryTable = array2table(summaryRows, 'VariableNames', {
    'algorithm','runs','passes','success10s','success20s', ...
    'success30s','success60s','medianFirstSolutionSec','p90FirstSolutionSec'});
report.summaryTable = summaryTable;

txtFile = fullfile(fileparts(outputDir), 'fair_benchmark_30_summary.txt');
fid = fopen(txtFile, 'w');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'FAIR WELD BENCHMARK SUMMARY\nCSV: %s\n\n', csvFile);
for ia = 1:height(summaryTable)
    fprintf(fid, '%s | runs=%s pass=%s | 10/20/30/60=%s/%s/%s/%s | median=%.6f s | P90=%.6f s\n', ...
        summaryTable.algorithm(ia), summaryTable.runs(ia), summaryTable.passes(ia), ...
        summaryTable.success10s(ia), summaryTable.success20s(ia), ...
        summaryTable.success30s(ia), summaryTable.success60s(ia), ...
        str2double(summaryTable.medianFirstSolutionSec(ia)), ...
        str2double(summaryTable.p90FirstSolutionSec(ia)));
end

% Figure 1: success probability versus time budget.
f1 = figure('Visible','off','Color','w'); hold on; grid on;
for ia = 1:numel(algorithms)
    mask = T.algorithm == algorithms(ia);
    y = zeros(size(budgets));
    for ib = 1:numel(budgets)
        y(ib) = 100*sum(mask & logical(T.complete) & ...
            isfinite(T.firstFeasibleTimeSec) & ...
            T.firstFeasibleTimeSec <= budgets(ib))/sum(mask);
    end
    plot(budgets, y, '-o', 'LineWidth', 1.8, 'DisplayName', algorithms(ia));
end
xlabel('Time budget (s)'); ylabel('Complete-path success rate (%)');
title('Fair cold-start success rate'); legend('Location','best');
ylim([0 100]);
exportgraphics(f1, fullfile(outputDir, 'success_rate_curve.png'), 'Resolution', 180);
close(f1);

% Figure 2: first feasible solution time for successful runs.
f2 = figure('Visible','off','Color','w');
groups = strings(0,1); vals = [];
for ia = 1:numel(algorithms)
    mask = T.algorithm == algorithms(ia) & logical(T.complete) & ...
        isfinite(T.firstFeasibleTimeSec);
    v = T.firstFeasibleTimeSec(mask);
    vals = [vals; v]; %#ok<AGROW>
    groups = [groups; repmat(algorithms(ia), numel(v), 1)]; %#ok<AGROW>
end
if ~isempty(vals)
    boxplot(vals, groups);
end
grid on; ylabel('First complete-path time (s)');
title('Successful runs: first feasible solution time');
exportgraphics(f2, fullfile(outputDir, 'first_solution_time_boxplot.png'), 'Resolution', 180);
close(f2);

% Figure 3: IK calls across all runs.
f3 = figure('Visible','off','Color','w');
groups = strings(0,1); vals = [];
for ia = 1:numel(algorithms)
    mask = T.algorithm == algorithms(ia);
    v = T.ikCalls(mask);
    vals = [vals; v]; %#ok<AGROW>
    groups = [groups; repmat(algorithms(ia), numel(v), 1)]; %#ok<AGROW>
end
boxplot(vals, groups); grid on;
ylabel('IK calls'); title('IK calls per cold-start run');
exportgraphics(f3, fullfile(outputDir, 'ik_calls_boxplot.png'), 'Resolution', 180);
close(f3);

% Figure 4: failure-point distribution.
f4 = figure('Visible','off','Color','w'); hold on; grid on;
hasFailure = false;
for ia = 1:numel(algorithms)
    mask = T.algorithm == algorithms(ia) & ~logical(T.complete) & ...
        isfinite(T.failurePoint);
    v = T.failurePoint(mask);
    if ~isempty(v)
        histogram(v, 1:81, 'DisplayStyle','stairs', 'LineWidth',1.8, ...
            'DisplayName', algorithms(ia));
        hasFailure = true;
    end
end
xlabel('Failure point'); ylabel('Number of runs');
title('Failure-point distribution');
if hasFailure, legend('Location','best'); end
exportgraphics(f4, fullfile(outputDir, 'failure_point_distribution.png'), 'Resolution', 180);
close(f4);

save(fullfile(fileparts(outputDir), 'fair_benchmark_30_analysis.mat'), 'report');
fprintf('Saved text summary: %s\n', txtFile);
fprintf('Saved plots       : %s\n', outputDir);
end
