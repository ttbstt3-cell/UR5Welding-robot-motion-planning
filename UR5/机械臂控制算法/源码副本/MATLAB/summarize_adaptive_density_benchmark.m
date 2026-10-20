function report = summarize_adaptive_density_benchmark(csvFile, outputDir)
%SUMMARIZE_ADAPTIVE_DENSITY_BENCHMARK Summarize sparse seam experiments.
% Analysis only: this function does not rerun the planner.

if nargin < 1 || isempty(csvFile)
    csvFile = fullfile(fileparts(mfilename('fullpath')), ...
        'adaptive_density_benchmark.csv');
end
if nargin < 2 || isempty(outputDir)
    outputDir = fullfile(fileparts(csvFile), 'adaptive_density_runs', 'plots');
end
if ~isfile(csvFile), error('CSV file not found: %s', csvFile); end
if ~isfolder(outputDir), mkdir(outputDir); end

T = readtable(csvFile, 'TextType','string');
counts = unique(T.numPoints, 'sorted');
summaryRows = repmat(localEmptySummary(), numel(counts), 1);

fprintf('\nADAPTIVE SEAM DENSITY SUMMARY\n');
for i = 1:numel(counts)
    n = counts(i);
    m = T.numPoints == n;
    pass = m & logical(T.complete);
    t = T.algorithmTimeSec(pass & isfinite(T.algorithmTimeSec));
    ik = T.ikCalls(m & isfinite(T.ikCalls));
    seg = T.maxSegmentTcpErrorMm(pass & isfinite(T.maxSegmentTcpErrorMm));
    jump = T.maxJointJumpDeg(pass & isfinite(T.maxJointJumpDeg));
    s = localEmptySummary();
    s.numPoints = n; s.runs = sum(m); s.passes = sum(pass);
    s.successRatePct = 100*s.passes/max(1,s.runs);
    if isempty(t)
        s.timeMedianSec = NaN; s.timeMeanSec = NaN; s.timeStdSec = NaN;
    else
        s.timeMedianSec = median(t); s.timeMeanSec = mean(t);
        s.timeStdSec = std(t, 0);
    end
    if isempty(ik), s.ikMedian = NaN; s.ikMean = NaN;
    else, s.ikMedian = median(ik); s.ikMean = mean(ik); end
    if isempty(seg), s.segmentTcpMaxMm = NaN; else, s.segmentTcpMaxMm = max(seg); end
    if isempty(jump), s.maxJointJumpDeg = NaN; else, s.maxJointJumpDeg = max(jump); end
    summaryRows(i) = s;
    fprintf('%3d points: PASS %d/%d (%.1f%%), time med/mean %.4f/%.4f s, IK med %.1f, max segment TCP %.4f mm\n', ...
        s.numPoints, s.passes, s.runs, s.successRatePct, ...
        s.timeMedianSec, s.timeMeanSec, s.ikMedian, s.segmentTcpMaxMm);
end

report = struct('csvFile',csvFile,'outputDir',outputDir,'summaryRows',summaryRows);
rootDir = fileparts(outputDir);
save(fullfile(rootDir,'adaptive_density_analysis.mat'),'report');
writetable(struct2table(summaryRows), fullfile(rootDir,'adaptive_density_summary.csv'));

f = figure('Visible','off','Color','w');
plot([summaryRows.numPoints], [summaryRows.timeMedianSec], '-o', 'LineWidth',1.8);
grid on; xlabel('Planning points'); ylabel('Median planning time (s)');
title('Sparse seam planning time');
exportgraphics(f, fullfile(outputDir,'density_time.png'),'Resolution',180); close(f);

f = figure('Visible','off','Color','w');
plot([summaryRows.numPoints], [summaryRows.ikMedian], '-o', 'LineWidth',1.8);
grid on; xlabel('Planning points'); ylabel('Median IK calls');
title('Sparse seam IK calls');
exportgraphics(f, fullfile(outputDir,'density_ik_calls.png'),'Resolution',180); close(f);

f = figure('Visible','off','Color','w');
bar([summaryRows.numPoints], [summaryRows.successRatePct]);
grid on; xlabel('Planning points'); ylabel('PASS rate (%)'); ylim([0 100]);
title('Sparse seam complete-path success rate');
exportgraphics(f, fullfile(outputDir,'density_success_rate.png'),'Resolution',180); close(f);

f = figure('Visible','off','Color','w');
plot([summaryRows.numPoints], [summaryRows.segmentTcpMaxMm], '-o', 'LineWidth',1.8);
grid on; xlabel('Planning points'); ylabel('Max segment TCP error (mm)');
title('Sparse seam segment TCP error');
exportgraphics(f, fullfile(outputDir,'density_segment_tcp_error.png'),'Resolution',180); close(f);
fprintf('Saved density analysis under: %s\n', rootDir);
end

function s = localEmptySummary()
s = struct('numPoints',NaN,'runs',0,'passes',0,'successRatePct',NaN, ...
    'timeMedianSec',NaN,'timeMeanSec',NaN,'timeStdSec',NaN, ...
    'ikMedian',NaN,'ikMean',NaN,'segmentTcpMaxMm',NaN,'maxJointJumpDeg',NaN);
end
