function summary = benchmark_adaptive_arc_ik(options)
%BENCHMARK_ADAPTIVE_ARC_IK Paired cold-start benchmark.
%
% Compares the unchanged fixed-80 and fixed-16 continuation baselines with
% the independent feasibility-first A-ARC-IK implementation.  Every method
% receives the same seed and the same current MATLAB planning scene.  No
% previous qPath or state graph is loaded.

if nargin < 1 || isempty(options), options = struct(); end
defaults = struct( ...
    "baseSeed",20269000,"runCount",30, ...
    "outputDir","adaptive_arc_ik_benchmark", ...
    "csvFile","adaptive_arc_ik_benchmark.csv", ...
    "matFile","adaptive_arc_ik_benchmark.mat", ...
    "initialPoints",16, ...
    "maxRefinementRounds",4,"maxPoints",40, ...
    "maxSegmentTcpErrorMm",0.5,"maxIntermediatePoints",4);
options = localMergeOptions(defaults,options);

here = fileparts(mfilename("fullpath"));
outDir = options.outputDir;
if ~isfolder(outDir), outDir = fullfile(here,outDir); end
if ~isfolder(outDir), mkdir(outDir); end

algorithms = ["fixed80","fixed16","A-ARC-IK"];
rows = repmat(localEmptyRow(),0,1);
total = numel(algorithms)*options.runCount;
row = 0;
fprintf("\n========================================\n");
fprintf("PAIRED A-ARC-IK BENCHMARK\n");
fprintf("========================================\n");
fprintf("Runs per algorithm: %d, seeds: %d..%d\n",options.runCount, ...
    options.baseSeed+1,options.baseSeed+options.runCount);

for ir = 1:options.runCount
    seed = options.baseSeed + ir;
    for ia = 1:numel(algorithms)
        algorithm = algorithms(ia);
        row = row+1;
        fprintf("[%d/%d] %s seed=%d ... ",row,total,algorithm,seed);
        t = tic;
        try
            switch algorithm
                case "fixed80"
                    opts = localStableOptions(seed,80, ...
                        sprintf("adaptive_arc_benchmark_fixed80_seed%05d.mat",seed));
                    capture = evalc("r = replan_weld_actual_axis_continuation(opts);"); %#ok<NASGU>
                    wall = toc(t);
                    rows(row) = localRowFromFixed(r,algorithm,seed,wall);
                    localMoveResultFile(r.outputFile,outDir);
                case "fixed16"
                    opts = localStableOptions(seed,16, ...
                        sprintf("adaptive_arc_benchmark_fixed16_seed%05d.mat",seed));
                    capture = evalc("r = replan_weld_actual_axis_continuation(opts);"); %#ok<NASGU>
                    wall = toc(t);
                    rows(row) = localRowFromFixed(r,algorithm,seed,wall);
                    localMoveResultFile(r.outputFile,outDir);
                case "A-ARC-IK"
                    opts = localAdaptiveOptions(options,seed,fullfile(outDir, ...
                        sprintf("adaptive_seed%05d.mat",seed)),fullfile(outDir, ...
                        sprintf("adaptive_seed%05d_round",seed)));
                    capture = evalc("r = replan_weld_actual_axis_adaptive_feasibility(opts);"); %#ok<NASGU>
                    wall = toc(t);
                    rows(row) = localRowFromAdaptive(r,algorithm,seed,wall);
            end
            fprintf("%s, points=%d, time=%.3fs\n", ...
                ternary(rows(row).complete,"PASS","FAIL"), ...
                rows(row).finalPointCount,rows(row).algorithmTimeSec);
        catch err
            wall = toc(t);
            rows(row) = localErrorRow(algorithm,seed,wall,string(err.message));
            fprintf("ERROR: %s\n",err.message);
        end
    end
end

rows = rows(1:row);
csvPath = options.csvFile;
if ~isfile(csvPath) && ~startsWith(string(csvPath),filesep), csvPath=fullfile(here,csvPath); end
writetable(struct2table(rows),csvPath);
matPath = options.matFile;
if ~isfile(matPath) && ~startsWith(string(matPath),filesep), matPath=fullfile(here,matPath); end
summary = struct("options",options,"rows",rows,"csvFile",csvPath, ...
    "matFile",matPath,"created",char(datetime("now")));
save(matPath,"summary","-v7.3");

fprintf("\nSaved CSV: %s\n",csvPath);
fprintf("Saved MAT: %s\n",matPath);
localPrintSummary(rows);
end

function opts = localStableOptions(seed,nPoints,outputFile)
opts = struct("numPoints",nPoints,"animate",false,"publish",false, ...
    "enableFallback",false,"beamWidth",1,"firstPointMaxStates",2, ...
    "maxPoseSeeds",3,"maxPoseNeighbors",9,"numRandomSeedsFirstPoint",3, ...
    "coarseTravelDeg",0,"coarseRollDeg",0:45:315, ...
    "progressivePoseNeighbors",true,"progressiveInitialPoseNeighbors",5, ...
    "progressiveMaxPoseNeighbors",9,"randomSeed",seed, ...
    "outputFile",outputFile);
end

function opts = localAdaptiveOptions(options,seed,outputFile,roundPrefix)
opts = localStableOptions(seed,options.initialPoints,outputFile);
opts.maxRefinementRounds=options.maxRefinementRounds;
opts.maxPoints=options.maxPoints;
opts.maxSegmentTcpErrorMm=options.maxSegmentTcpErrorMm;
opts.maxIntermediatePoints=options.maxIntermediatePoints;
opts.roundOutputPrefix=roundPrefix;
end

function row = localRowFromFixed(r,algorithm,seed,wall)
row = localEmptyRow();
row.algorithm=string(algorithm); row.seed=seed; row.complete=logical(r.complete);
row.algorithmTimeSec=r.algorithmElapsedSec; row.wallTimeSec=wall;
row.finalPointCount=r.seam.numPoints; row.executionPointCount=r.seam.numPoints;
row.ikCalls=r.continuationStats.ikCalls; row.ikPoseSuccess=r.continuationStats.ikPoseSuccess;
row.maxJointJumpDeg=r.maxJointJumpDeg;
row.maxSegmentTcpErrorMm=r.tcpMetrics.maxSegmentPositionErrorMm;
row.nodeCollisionFree=logical(r.nodeCollisionFree); row.edgeCollisionFree=logical(r.edgeCollisionFree);
row.furthestPoint=localFurthestPoint(r.stateBank); row.insertedPoints=0;
row.refinementRounds=0; row.precisionPass=logical(r.complete);
end

function row = localRowFromAdaptive(r,algorithm,seed,wall)
row = localEmptyRow();
row.algorithm=string(algorithm); row.seed=seed; row.complete=logical(r.complete);
row.algorithmTimeSec=r.feasibilityAdaptiveAlgorithmTimeSec+r.precisionAlgorithmTimeSec;
row.wallTimeSec=wall; row.finalPointCount=r.finalPointCount;
if isfield(r,"qPathExecution"), row.executionPointCount=size(r.qPathExecution,1); end
row.ikCalls=r.feasibilityAdaptiveIkCalls;
if isfield(r,"adaptiveIkCalls"), row.ikCalls=row.ikCalls+r.adaptiveIkCalls; end
row.ikPoseSuccess=NaN; row.maxJointJumpDeg=r.maxJointJumpDeg;
if isfield(r,"finalTcpMetrics")
    row.maxSegmentTcpErrorMm=max(r.finalTcpMetrics.segmentPositionErrorMm);
else
    row.maxSegmentTcpErrorMm=NaN;
end
row.nodeCollisionFree=logical(r.nodeCollisionFree); row.edgeCollisionFree=logical(r.edgeCollisionFree);
row.furthestPoint=localFurthestPoint(r.stateBank);
row.insertedPoints=r.finalPointCount-r.initialPoints+r.precisionInsertedPoints;
row.refinementRounds=r.refinementRoundCount; row.precisionPass=logical(r.adaptiveCorrectionComplete);
end

function row = localErrorRow(algorithm,seed,wall,msg)
row=localEmptyRow(); row.algorithm=string(algorithm); row.seed=seed; row.wallTimeSec=wall;
row.errorMessage=string(msg);
end

function row = localEmptyRow()
row=struct("algorithm","","seed",NaN,"complete",false, ...
    "algorithmTimeSec",NaN,"wallTimeSec",NaN,"finalPointCount",NaN, ...
    "executionPointCount",NaN,"ikCalls",NaN,"ikPoseSuccess",NaN, ...
    "maxJointJumpDeg",NaN,"maxSegmentTcpErrorMm",NaN, ...
    "nodeCollisionFree",false,"edgeCollisionFree",false,"furthestPoint",NaN, ...
    "insertedPoints",0,"refinementRounds",0,"precisionPass",false, ...
    "errorMessage","");
end

function idx = localFurthestPoint(bank)
idx=find(~cellfun(@isempty,bank),1,"last"); if isempty(idx), idx=0; end
end

function localPrintSummary(rows)
names=unique(string({rows.algorithm}));
fprintf("\nSummary:\n");
for i=1:numel(names)
    m=string({rows.algorithm})==names(i); rr=rows(m);
    fprintf("  %-10s PASS %d/%d, median time %.3fs, median points %.1f\n", ...
        names(i),sum([rr.complete]),numel(rr),median([rr.algorithmTimeSec],"omitnan"), ...
        median([rr.finalPointCount],"omitnan"));
end
end

function out=localMergeOptions(defaults,user)
out=defaults; names=fieldnames(user);
for i=1:numel(names), out.(names{i})=user.(names{i}); end
end

function out=ternary(cond,a,b)
if cond, out=a; else, out=b; end
end

function localMoveResultFile(source,outDir)
% The legacy stable continuation resolves relative output names against its
% own MATLAB directory.  Move those per-run artifacts into the benchmark
% directory after the call without modifying that stable file.
if isfile(source)
    [~,name,ext]=fileparts(source);
    target=fullfile(outDir,[name ext]);
    if ~strcmp(source,target), movefile(source,target,'f'); end
end
end
