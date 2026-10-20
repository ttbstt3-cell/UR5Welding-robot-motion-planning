function summary = summarize_adaptive_arc_ik_benchmark(csvFile,outDir)
%SUMMARIZE_ADAPTIVE_ARC_IK_BENCHMARK Create compact tables and figures.

if nargin < 1 || isempty(csvFile)
    csvFile = fullfile(fileparts(mfilename("fullpath")), ...
        "adaptive_arc_ik_benchmark.csv");
end
if nargin < 2 || isempty(outDir)
    outDir = fullfile(fileparts(csvFile),"adaptive_arc_ik_benchmark_plots");
end
if ~isfolder(outDir), mkdir(outDir); end

T = readtable(csvFile,"TextType","string");
names = ["Fixed-80","Fixed-16","A-ARC-IK"];
keys = ["fixed80","fixed16","A-ARC-IK"];
S = table('Size',[numel(keys),10], ...
    'VariableTypes',["string","double","double","double","double", ...
        "double","double","double","double","double"], ...
    'VariableNames',{'algorithm','passCount','runCount','successRatePct', ...
        'medianSuccessTimeSec','medianIKCalls','medianFinalPoints', ...
        'medianExecutionPoints','medianTcpErrorMm','medianJumpDeg'});
for k=1:numel(keys)
    m=T.algorithm==keys(k); pass=m & T.complete==1;
    S.algorithm(k)=names(k); S.passCount(k)=sum(pass); S.runCount(k)=sum(m);
    S.successRatePct(k)=100*S.passCount(k)/max(1,S.runCount(k));
    S.medianSuccessTimeSec(k)=median(T.algorithmTimeSec(pass),'omitnan');
    S.medianIKCalls(k)=median(T.ikCalls(pass),'omitnan');
    S.medianFinalPoints(k)=median(T.finalPointCount(pass),'omitnan');
    S.medianExecutionPoints(k)=median(T.executionPointCount(pass),'omitnan');
    S.medianTcpErrorMm(k)=median(T.maxSegmentTcpErrorMm(pass),'omitnan');
    S.medianJumpDeg(k)=median(T.maxJointJumpDeg(pass),'omitnan');
end
summary = S;
outCsv=fullfile(fileparts(csvFile),"adaptive_arc_ik_summary.csv");
writetable(S,outCsv);

f=figure('Visible','off','Color','w');
bar(S.successRatePct); ylim([0 100]); grid on;
set(gca,'XTick',1:height(S),'XTickLabel',S.algorithm);
ylabel('PASS rate (%)'); title('Adaptive ARC-IK paired success rate');
exportgraphics(f,fullfile(outDir,'success_rate.png'),'Resolution',160); close(f);

f=figure('Visible','off','Color','w'); hold on;
for k=1:numel(keys)
    m=T.algorithm==keys(k) & T.complete==1;
    x=k*ones(sum(m),1); boxchart(x,T.algorithmTimeSec(m));
end
grid on; xlim([0.4 numel(keys)+0.6]);
set(gca,'XTick',1:numel(keys),'XTickLabel',names);
ylabel('Planning time (s)'); title('Successful-run planning time');
exportgraphics(f,fullfile(outDir,'planning_time_boxplot.png'),'Resolution',160); close(f);

f=figure('Visible','off','Color','w');
bar(S.medianTcpErrorMm); grid on;
set(gca,'XTick',1:height(S),'XTickLabel',S.algorithm);
ylabel('Max segment TCP error (mm)'); title('Successful-run TCP segment error');
exportgraphics(f,fullfile(outDir,'tcp_error.png'),'Resolution',160); close(f);

fprintf('Summary CSV: %s\n',outCsv);
fprintf('Plots      : %s\n',outDir);
disp(S);
end
