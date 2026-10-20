function summary = benchmark_end_margin(options)
%BENCHMARK_END_MARGIN
% Independent endpoint-margin experiment for the frozen A-ARC-IK+ planner.
%
% The planner and its collision rules are not changed.  Only endMarginM is
% varied, which clips the *weld target interval* symmetrically along the
% physical seam.  The complete robot/tool/wire/workpiece collision model is
% still used for every node and interpolated edge.
%
% This script is intentionally separate from benchmark_adaptive_arc_ik_new100
% so the existing benchmark wrapper remains untouched.

if nargin < 1 || isempty(options), options = struct(); end
root = fileparts(mfilename('fullpath'));
defaults = struct( ...
    'marginsMm',0:5:30, ...
    'seeds',20270001:20270100, ...
    'outputRoot',fullfile(root,'end_margin_benchmark_runs'), ...
    'summaryCsv',fullfile(root,'end_margin_benchmark.csv'), ...
    'summaryMat',fullfile(root,'end_margin_benchmark.mat'), ...
    'maxRefinementRounds',4, ...
    'maxPoints',40, ...
    'initialPoints',16, ...
    'maxSegmentTcpErrorMm',0.5, ...
    'maxIntermediatePoints',4, ...
    'resume',true, ...
    'stopOnError',false);
options = localMerge(defaults,options);
options.marginsMm = double(options.marginsMm(:).');
options.seeds = double(options.seeds(:).');
if ~isfolder(options.outputRoot), mkdir(options.outputRoot); end

rows = repmat(localMarginRow(),0,1);
for im = 1:numel(options.marginsMm)
    marginMm = options.marginsMm(im);
    marginTag = sprintf('%gmm',marginMm);
    outDir = fullfile(options.outputRoot,marginTag);
    if ~isfolder(outDir), mkdir(outDir); end
    csvFile = fullfile(outDir,sprintf('end_margin_%s.csv',marginTag));
    matFile = fullfile(outDir,sprintf('end_margin_%s.mat',marginTag));
    fprintf('\n====================================================\n');
    fprintf('END-MARGIN BENCHMARK: %.3f mm (%d seeds)\n',marginMm,numel(options.seeds));
    fprintf('====================================================\n');

    runRows = repmat(localRunRow(),0,1);
    for k = 1:numel(options.seeds)
        seed = options.seeds(k);
        outputFile = fullfile(outDir,sprintf('seed%d.mat',seed));
        roundPrefix = fullfile(outDir,sprintf('seed%d_round',seed));
        if options.resume && isfile(outputFile)
            cached = load(outputFile,'result');
            if isfield(cached,'result') && isstruct(cached.result)
                row = localRunRowFromResult(cached.result,seed,NaN,marginMm);
                runRows(end+1,1) = row; %#ok<AGROW>
                fprintf('[%3d/%3d] margin=%g mm seed=%d ... CACHED %s\n', ...
                    k,numel(options.seeds),marginMm,seed,ternary(row.complete,'PASS','FAIL'));
                continue;
            end
        end
        plannerOptions = struct( ...
            'initialPoints',options.initialPoints, ...
            'maxRefinementRounds',options.maxRefinementRounds, ...
            'maxPoints',options.maxPoints, ...
            'maxSegmentTcpErrorMm',options.maxSegmentTcpErrorMm, ...
            'maxIntermediatePoints',options.maxIntermediatePoints, ...
            'endMarginM',marginMm/1000, ...
            'randomSeed',seed, ...
            'enablePostRecovery',true, ...
            'outputFile',outputFile, ...
            'roundOutputPrefix',roundPrefix, ...
            'animate',false,'publish',false);
        fprintf('[%3d/%3d] margin=%g mm seed=%d ... ',k,numel(options.seeds),marginMm,seed);
        t = tic;
        try
            capture = evalc('result = replan_weld_actual_axis_adaptive_feasibility(plannerOptions);'); %#ok<NASGU>
            wall = toc(t);
            row = localRunRowFromResult(result,seed,wall,marginMm);
            fprintf('%s wall=%.3fs points=%d IK=%d stage=%s\n', ...
                ternary(row.complete,'PASS','FAIL'),wall,row.finalPointCount,row.ikCalls,char(row.failureStage));
        catch err
            wall = toc(t);
            row = localRunRow();
            row.marginMm = marginMm; row.seed = seed; row.wallTimeSec = wall;
            row.failureStage = "runtime_error"; row.errorMessage = string(err.message);
            fprintf('ERROR wall=%.3fs %s\n',wall,err.message);
            if options.stopOnError, rethrow(err); end
        end
        runRows(end+1,1) = row; %#ok<AGROW>
    end
    writetable(struct2table(runRows),csvFile);
    save(matFile,'runRows','marginMm','options','-v7.3');
    rows(end+1,1) = localSummarizeMargin(runRows,marginMm); %#ok<AGROW>
    fprintf('MARGIN %.3f mm: PASS %d/%d (%.1f%%)\n',marginMm, ...
        rows(end).passCount,rows(end).sampleCount,rows(end).successRatePct);
end

summary = struct('name','A-ARC-IK+ endpoint-margin benchmark', ...
    'marginsMm',options.marginsMm,'seeds',options.seeds,'options',options, ...
    'rows',rows,'summaryCsv',options.summaryCsv,'summaryMat',options.summaryMat, ...
    'created',char(datetime('now')));
writetable(struct2table(rows),options.summaryCsv);
save(options.summaryMat,'summary','-v7.3');
fprintf('\nSaved summary CSV: %s\nSaved summary MAT: %s\n',options.summaryCsv,options.summaryMat);
end

function row = localRunRowFromResult(r,seed,wall,marginMm)
row = localRunRow();
row.marginMm = marginMm; row.seed = seed; row.complete = logical(r.complete); row.wallTimeSec = wall;
row.algorithmTimeSec = localNum(r,'feasibilityAdaptiveAlgorithmTimeSec',NaN) + localNum(r,'precisionAlgorithmTimeSec',0);
row.firstSolutionTimeSec = localFirstTime(r);
row.finalPointCount = localNum(r,'finalPointCount',NaN);
row.ikCalls = localNum(r,'feasibilityAdaptiveIkCalls',0) + localNum(r,'adaptiveIkCalls',0);
row.postRecoveryUsed = logical(localNum(r,'postRecoveryUsed',false));
row.furthestPoint = localFurthest(r);
row.maxJointJumpDeg = localNum(r,'maxJointJumpDeg',NaN);
row.maxTcpSegmentErrorMm = NaN;
if isfield(r,'finalTcpMetrics') && isfield(r.finalTcpMetrics,'segmentPositionErrorMm') && ~isempty(r.finalTcpMetrics.segmentPositionErrorMm)
    row.maxTcpSegmentErrorMm = max(r.finalTcpMetrics.segmentPositionErrorMm);
elseif isfield(r,'tcpMetrics') && isfield(r.tcpMetrics,'maxSegmentPositionErrorMm')
    row.maxTcpSegmentErrorMm = r.tcpMetrics.maxSegmentPositionErrorMm;
end
row.nodeCollisionFree = logical(localNum(r,'nodeCollisionFree',false));
row.edgeCollisionFree = logical(localNum(r,'edgeCollisionFree',false));
row.failureStage = localFailureStage(r);
if row.complete, row.failureStage = "PASS"; end
end

function row = localRunRow()
row = struct('marginMm',NaN,'seed',NaN,'complete',false,'algorithmTimeSec',NaN, ...
    'firstSolutionTimeSec',NaN,'wallTimeSec',NaN,'finalPointCount',NaN,'ikCalls',NaN, ...
    'postRecoveryUsed',false,'furthestPoint',NaN,'maxJointJumpDeg',NaN, ...
    'maxTcpSegmentErrorMm',NaN,'nodeCollisionFree',false,'edgeCollisionFree',false, ...
    'failureStage',"",'errorMessage',"");
end

function row = localMarginRow()
row = struct('marginMm',NaN,'sampleCount',0,'passCount',0,'successRatePct',NaN, ...
    'medianFirstSolutionTimeSec',NaN,'p90FirstSolutionTimeSec',NaN, ...
    'medianTotalTimeSec',NaN,'p90TotalTimeSec',NaN,'medianIkCalls',NaN, ...
    'p90IkCalls',NaN,'medianTcpErrorMm',NaN,'p90TcpErrorMm',NaN, ...
    'medianMaxJointJumpDeg',NaN,'collisionFailureCount',0,'noNodeFailureCount',0, ...
    'connectivityFailureCount',0,'runtimeErrorCount',0);
end

function row = localSummarizeMargin(runRows,marginMm)
row = struct('marginMm',marginMm,'sampleCount',numel(runRows),'passCount',sum([runRows.complete]), ...
    'successRatePct',100*mean([runRows.complete]),'medianFirstSolutionTimeSec',NaN, ...
    'p90FirstSolutionTimeSec',NaN,'medianTotalTimeSec',NaN,'p90TotalTimeSec',NaN, ...
    'medianIkCalls',NaN,'p90IkCalls',NaN,'medianTcpErrorMm',NaN,'p90TcpErrorMm',NaN, ...
    'medianMaxJointJumpDeg',NaN,'collisionFailureCount',0,'noNodeFailureCount',0, ...
    'connectivityFailureCount',0,'runtimeErrorCount',sum(string({runRows.failureStage})=="runtime_error"));
ok = [runRows.complete];
if any(ok)
    row.medianFirstSolutionTimeSec = median([runRows(ok).firstSolutionTimeSec],'omitnan');
    row.p90FirstSolutionTimeSec = prctile([runRows(ok).firstSolutionTimeSec],90);
    row.medianTotalTimeSec = median([runRows(ok).algorithmTimeSec],'omitnan');
    row.p90TotalTimeSec = prctile([runRows(ok).algorithmTimeSec],90);
    row.medianIkCalls = median([runRows(ok).ikCalls],'omitnan');
    row.p90IkCalls = prctile([runRows(ok).ikCalls],90);
    row.medianTcpErrorMm = median([runRows(ok).maxTcpSegmentErrorMm],'omitnan');
    row.p90TcpErrorMm = prctile([runRows(ok).maxTcpSegmentErrorMm],90);
    row.medianMaxJointJumpDeg = median([runRows(ok).maxJointJumpDeg],'omitnan');
end
st = string({runRows.failureStage});
row.collisionFailureCount = sum(contains(st,"collision"));
row.noNodeFailureCount = sum(contains(st,"failed_after_point") | contains(st,"no_bisection"));
row.connectivityFailureCount = sum(~ok & ~contains(st,"collision") & ~contains(st,"runtime_error"));
end

function t = localFirstTime(r)
t = NaN;
if ~isfield(r,'refinementRounds') || isempty(r.refinementRounds), return; end
rr = r.refinementRounds;
for i = 1:numel(rr)
    if isfield(rr(i),'complete') && rr(i).complete
        v = [rr(1:i).algorithmTimeSec]; v = v(~isnan(v));
        if ~isempty(v), t = sum(v); end
        return;
    end
end
end

function idx = localFurthest(r)
idx = NaN;
if isfield(r,'stateBank')
    idx = find(~cellfun(@isempty,r.stateBank),1,'last');
    if isempty(idx), idx = 0; end
end
end

function stage = localFailureStage(r)
stage = "failed";
if isfield(r,'postRecoveryUsed') && r.postRecoveryUsed, stage = "post_recovery"; end
if isfield(r,'refinementRounds') && ~isempty(r.refinementRounds)
    rr = r.refinementRounds;
    for i = numel(rr):-1:1
        if isfield(rr(i),'recovery') && strlength(string(rr(i).recovery)) > 0
            stage = string(rr(i).recovery); return;
        end
    end
end
if isfield(r,'stateBank')
    idx = localFurthest(r);
    if idx > 0 && idx < numel(r.stateBank), stage = "failed_after_point_" + string(idx); end
end
end

function v = localNum(s,name,default)
if isfield(s,name) && ~isempty(s.(name)), v = double(s.(name)); else, v = default; end
end

function out = localMerge(defaults,custom)
out = defaults;
f = fieldnames(custom);
for i = 1:numel(f), out.(f{i}) = custom.(f{i}); end
end

function y = ternary(cond,a,b)
if cond, y = a; else, y = b; end
end
