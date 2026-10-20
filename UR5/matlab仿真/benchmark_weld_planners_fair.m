function summary = benchmark_weld_planners_fair(options)
%BENCHMARK_WELD_PLANNERS_FAIR
% Same-scene cold-start comparison for Continuation and Gao-AR-RRT*.
%
% Every run reconstructs the MATLAB planning scene and solves IK from
% scratch.  No previous qPath, checkpoint, or state graph is loaded.
% Defaults are deliberately the full experiment requested for the paper:
% 30 seeds at 10/20/30/60 s budgets.  Use a smaller runCount/budget list for
% a smoke test before launching the full batch.

if nargin < 1 || isempty(options), options=struct(); end
defaults=struct( ...
    "runCount",30,"timeBudgetsSec",[10 20 30 60], ...
    "singleRunPerSeed",true,"maxBudgetSec",60, ...
    "baseSeed",20260924,"runContinuation",true,"runGao",true, ...
    "gaoSamplesPerLayer",12,"gaoMaxStatesPerLayer",12, ...
    "outputDir","fair_benchmark_runs", ...
    "summaryFile","fair_weld_planner_benchmark.mat", ...
    "csvFile","fair_weld_planner_benchmark.csv");
options=localMergeOptions(defaults,options);
matlabDir=fileparts(mfilename("fullpath"));
outputDirRel=char(options.outputDir);
if startsWith(outputDirRel,[matlabDir filesep])
    outputDirRel=extractAfter(outputDirRel,[matlabDir filesep]);
elseif isabsolute(outputDirRel)
    error("outputDir 必须是 MATLAB 仿真目录下的相对路径。当前：%s",outputDirRel);
end
outDir=fullfile(matlabDir,outputDirRel);
if ~isfolder(outDir), mkdir(outDir); end

algorithms=string.empty(1,0);
if options.runContinuation, algorithms(end+1)="Continuation"; end
if options.runGao, algorithms(end+1)="Gao-AR-RRT*"; end
if isempty(algorithms), error("至少启用一个算法。"); end

fprintf("\nFAIR COLD-START BENCHMARK\n");
if options.singleRunPerSeed
    runBudgets=options.maxBudgetSec;
else
    runBudgets=options.timeBudgetsSec;
end
fprintf("Runs=%d, run budgets=[%s] s, report budgets=[%s] s, algorithms=%s\n",options.runCount, ...
    sprintf("%g ",runBudgets),sprintf("%g ",options.timeBudgetsSec),strjoin(algorithms,", "));

nRows=options.runCount*numel(runBudgets)*numel(algorithms);
template=localEmptyRow(); rows=repmat(template,nRows,1); row=0;

for ib=1:numel(runBudgets)
    runBudget=runBudgets(ib);
    for ir=1:options.runCount
        seed=options.baseSeed+1000*ib+ir;
        if options.runContinuation
            row=row+1;
            fprintf("[%d/%d] Continuation seed=%d budget=%gs\n",row,nRows,seed,runBudget);
            opts=localContinuationOptions(seed,fullfile(outputDirRel,sprintf("continuation_b%d_r%02d.mat",runBudget,ir)));
            t=tic; result=replan_weld_actual_axis_continuation(opts); wall=toc(t);
            rows(row)=localRowFromResult(rows(row),result,"Continuation",seed,runBudget,wall);
        end
        if options.runGao
            row=row+1;
            fprintf("[%d/%d] Gao-AR-RRT* seed=%d budget=%gs\n",row,nRows,seed,runBudget);
            opts=localGaoOptions(seed,runBudget,options,fullfile(outputDirRel,sprintf("gao_b%d_r%02d.mat",runBudget,ir)));
            t=tic; result=replan_weld_actual_axis_gao_ar_rrtstar(opts); wall=toc(t);
            rows(row)=localRowFromResult(rows(row),result,"Gao-AR-RRT*",seed,runBudget,wall);
        end
    end
end

rows=rows(1:row);
summary=struct(); summary.options=options; summary.rows=rows;
summary.algorithms=algorithms; summary.created=char(datetime("now"));
summary.outputDir=outDir;
summary.successCurves=localSuccessCurves(rows,algorithms,options.timeBudgetsSec);
summaryFile=options.summaryFile; if ~isfile(summaryFile), summaryFile=fullfile(matlabDir,summaryFile); end
save(summaryFile,"summary","-v7.3");
csvFile=options.csvFile; if ~isfile(csvFile), csvFile=fullfile(matlabDir,csvFile); end
writetable(struct2table(rows),csvFile);

fprintf("\nSaved summary: %s\n",summaryFile);
fprintf("Saved CSV    : %s\n",csvFile);
for a=1:numel(algorithms)
    mask=strcmp(string({rows.algorithm}),algorithms(a));
    fprintf("\n%s\n",algorithms(a));
    for b=1:numel(options.timeBudgetsSec)
        m=mask & [rows.firstFeasibleTimeSec]<=options.timeBudgetsSec(b) & [rows.complete];
        fprintf("  %g s: success=%d/%d (%.1f%%)\n",options.timeBudgetsSec(b), ...
            sum(m),sum(mask),100*sum(m)/max(1,sum(mask)));
    end
end
end

function opts=localContinuationOptions(seed,file)
opts=struct("animate",false,"publish",false,"enableFallback",false, ...
    "beamWidth",1,"firstPointMaxStates",2,"maxPoseSeeds",3, ...
    "maxPoseNeighbors",9,"numRandomSeedsFirstPoint",3, ...
    "coarseTravelDeg",0,"coarseRollDeg",0:45:315, ...
    "progressivePoseNeighbors",true,"progressiveInitialPoseNeighbors",5, ...
    "progressiveMaxPoseNeighbors",9,"randomSeed",seed, ...
    "outputFile",file);
end

function opts=localGaoOptions(seed,budget,options,file)
opts=struct("randomSeed",seed,"timeBudgetSec",budget,"maxIterations",100000, ...
    "samplesPerLayer",options.gaoSamplesPerLayer, ...
    "maxStatesPerLayer",options.gaoMaxStatesPerLayer, ...
    "maxIKSeeds",8,"neighborParents",8,"goalBias",0.20, ...
    "animate",false,"outputFile",file);
end

function row=localRowFromResult(row,r,algorithm,seed,budget,wall)
row.algorithm=char(algorithm); row.seed=seed; row.timeBudgetSec=budget;
row.algorithmTimeSec=r.algorithmElapsedSec; row.wallTimeSec=wall;
row.firstFeasibleTimeSec=localGet(r,"firstFeasibleTimeSec",r.algorithmElapsedSec);
row.complete=logical(r.complete); row.withinBudgetAndPass=row.complete && ...
    isfinite(row.firstFeasibleTimeSec) && row.firstFeasibleTimeSec<=budget;
plannerStats=localPlannerStats(r);
row.ikCalls=localGet(plannerStats,"ikCalls",NaN);
row.ikPoseSuccess=localGet(plannerStats,"ikPoseSuccess",NaN);
row.ikSuccessRatePct=100*row.ikPoseSuccess/max(1,row.ikCalls);
row.edgeTests=localGet(plannerStats,"edgeTests",NaN);
row.edgeCollisionFree=localGet(plannerStats,"edgeCollisionFree",NaN);
row.nodeCollisionFree=logical(localGet(r,"nodeCollisionFree",false));
row.edgeCollisionFreePass=logical(localGet(r,"edgeCollisionFree",false));
row.maxJointJumpDeg=localGet(r,"maxJointJumpDeg",NaN);
row.maxNodeTcpErrorMm=localGetNested(r,"tcpMetrics","maxNodePositionErrorMm",NaN);
row.maxSegmentTcpErrorMm=localGetNested(r,"tcpMetrics","maxSegmentPositionErrorMm",NaN);
row.maxSegmentTcpOrientationDeg=localGetNested(r,"tcpMetrics","maxSegmentOrientationErrorDeg",NaN);
row.failurePoint=localFailurePoint(r);
row.furthestPoint=localFurthestPoint(r);
end

function s=localPlannerStats(r)
if isfield(r,"stats")
    s=r.stats;
elseif isfield(r,"continuationStats")
    s=r.continuationStats;
else
    s=struct();
end
end

function p=localFailurePoint(r)
p=NaN;
if isfield(r,"stateBank")
    x=cellfun(@isempty,r.stateBank); k=find(x,1); if ~isempty(k),p=k;else,p=numel(r.stateBank);end
elseif isfield(r,"layers")
    x=cellfun(@isempty,r.layers); k=find(x,1); if ~isempty(k),p=k;else,p=numel(r.layers);end
end
end

function p=localFurthestPoint(r)
p=NaN;
if isfield(r,"stateBank")
    p=find(~cellfun(@isempty,r.stateBank),1,"last");
elseif isfield(r,"layers")
    p=find(~cellfun(@isempty,r.layers),1,"last");
end
if isempty(p), p=0; end
end

function x=localGet(s,f,default)
if isfield(s,f),x=s.(f);else,x=default;end
end
function x=localGetNested(s,f1,f2,default)
if isfield(s,f1) && isfield(s.(f1),f2),x=s.(f1).(f2);else,x=default;end
end

function c=localSuccessCurves(rows,algorithms,budgets)
c=repmat(struct("algorithm","","budgetSec",0,"successCount",0,"runCount",0,"successRate",0),numel(algorithms)*numel(budgets),1); k=0;
for a=1:numel(algorithms), for b=1:numel(budgets), k=k+1; m=strcmp(string({rows.algorithm}),algorithms(a)); pass=m & [rows.complete] & [rows.firstFeasibleTimeSec]<=budgets(b); c(k).algorithm=char(algorithms(a)); c(k).budgetSec=budgets(b); c(k).successCount=sum(pass); c(k).runCount=sum(m); c(k).successRate=c(k).successCount/max(1,c(k).runCount); end,end
end

function row=localEmptyRow()
row=struct("algorithm","","seed",0,"timeBudgetSec",NaN,"algorithmTimeSec",NaN,"wallTimeSec",NaN,"firstFeasibleTimeSec",NaN,"complete",false,"withinBudgetAndPass",false,"ikCalls",NaN,"ikPoseSuccess",NaN,"ikSuccessRatePct",NaN,"edgeTests",NaN,"edgeCollisionFree",NaN,"nodeCollisionFree",false,"edgeCollisionFreePass",false,"maxJointJumpDeg",NaN,"maxNodeTcpErrorMm",NaN,"maxSegmentTcpErrorMm",NaN,"maxSegmentTcpOrientationDeg",NaN,"failurePoint",NaN,"furthestPoint",NaN);
end
function out=localMergeOptions(d,in)
out=d; for f=fieldnames(d).', if isfield(in,f{1}) && ~isempty(in.(f{1})),out.(f{1})=in.(f{1});end,end
end
function tf=isabsolute(p)
tf=startsWith(string(p),filesep) || (strlength(string(p))>=2 && extractBetween(string(p),2,2)==":");
end
