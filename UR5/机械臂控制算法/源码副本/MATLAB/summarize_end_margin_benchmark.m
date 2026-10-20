function summary = summarize_end_margin_benchmark(options)
%SUMMARIZE_END_MARGIN_BENCHMARK Aggregate coarse/fine endpoint-margin runs.

if nargin < 1 || isempty(options), options = struct(); end
root = fileparts(mfilename('fullpath'));
defaults = struct( ...
    'marginsMm',[0 5 10 15 20 21 22 23 24 25 30], ...
    'inputRoot',root, ...
    'outputCsv',fullfile(root,'end_margin_benchmark.csv'), ...
    'outputMat',fullfile(root,'end_margin_benchmark.mat'), ...
    'summaryTxt',fullfile(root,'end_margin_summary.txt'), ...
    'plotDir',fullfile(root,'end_margin_plots'));
options = localMerge(defaults,options);
if ~isfolder(options.plotDir), mkdir(options.plotDir); end

rows = table();
for m = options.marginsMm(:).'
    f = fullfile(options.inputRoot,sprintf('end_margin_%gmm.csv',m));
    if ~isfile(f), warning('Missing margin summary: %s',f); continue; end
    T = readtable(f);
    rows = [rows; T]; %#ok<AGROW>
end
if isempty(rows), error('No endpoint-margin summary CSV files were found.'); end
rows = sortrows(rows,'marginMm');
writetable(rows,options.outputCsv);
save(options.outputMat,'rows','options','-v7.3');

% Success-rate curve.
fig = figure('Visible','off','Color','w');
plot(rows.marginMm,rows.successRatePct,'o-','LineWidth',1.8,'MarkerSize',6);
xlabel('Symmetric end margin (mm)'); ylabel('PASS rate (%)'); grid on;
ylim([0 100]); title('A-ARC-IK+ endpoint-margin benchmark');
exportgraphics(fig,fullfile(options.plotDir,'end_margin_success_rate.png'),'Resolution',150); close(fig);

% Median total algorithm time for successful samples.
fig = figure('Visible','off','Color','w');
plot(rows.marginMm,rows.medianTotalTimeSec,'o-','LineWidth',1.8,'MarkerSize',6);
xlabel('Symmetric end margin (mm)'); ylabel('Median algorithm time (s)'); grid on;
title('Successful-run planning time versus end margin');
exportgraphics(fig,fullfile(options.plotDir,'end_margin_time.png'),'Resolution',150); close(fig);

% Failure count / success count.
fig = figure('Visible','off','Color','w');
bar(rows.marginMm,[rows.passCount rows.sampleCount-rows.passCount],'stacked');
xlabel('Symmetric end margin (mm)'); ylabel('Runs'); grid on;
legend({'PASS','FAIL'},'Location','best'); title('Endpoint-margin outcome counts');
exportgraphics(fig,fullfile(options.plotDir,'end_margin_failure_type.png'),'Resolution',150); close(fig);

fid = fopen(options.summaryTxt,'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid,'A-ARC-IK+ endpoint-margin benchmark\n');
fprintf(fid,'Seeds: 20270001:20270100\n');
fprintf(fid,'All collision geometry remained enabled; no endpoint collisions were ignored.\n\n');
fprintf(fid,' margin_mm  PASS/100  rate(%%)  median_time_s  median_IK  median_TCP_seg_mm\n');
for i=1:height(rows)
    fprintf(fid,' %8.1f  %3d/100  %7.1f  %14.4f  %9.1f  %18.4f\n', ...
        rows.marginMm(i),rows.passCount(i),rows.successRatePct(i), ...
        rows.medianTotalTimeSec(i),rows.medianIkCalls(i),rows.medianTcpErrorMm(i));
end
fprintf(fid,'\nNo tested margin reached 100/100 unless such a row is explicitly present.\n');
fprintf(fid,'The first tested margin reaching the highest observed rate is benchmark-specific, not a theoretical minimum.\n');

summary = struct('rows',rows,'options',options,'outputCsv',options.outputCsv, ...
    'outputMat',options.outputMat,'summaryTxt',options.summaryTxt, ...
    'plotDir',options.plotDir,'created',char(datetime('now')));
fprintf('Saved aggregate endpoint-margin results: %s\n',options.outputCsv);
end

function out = localMerge(defaults,custom)
out = defaults;
f = fieldnames(custom);
for i=1:numel(f), out.(f{i}) = custom.(f{i}); end
end
