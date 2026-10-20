function summary = benchmark_adaptive_arc_ik_new100(options)
%BENCHMARK_ADAPTIVE_ARC_IK_NEW100
% Frozen, independent 100-seed cold-start validation for A-ARC-IK+.
%
% This benchmark deliberately uses seeds disjoint from the historical
% development set 20269001:20269030.  It never loads a previous qPath or
% state graph.  The planner implementation and recovery parameters are not
% changed by this wrapper.

if nargin < 1 || isempty(options), options=struct(); end
root=fileparts(mfilename('fullpath'));
defaults=struct( ...
    'seeds',20270001:20270100, ...
    'outputDir',fullfile(root,'adaptive_arc_ik_new100_runs'), ...
    'csvFile',fullfile(root,'adaptive_arc_ik_new100.csv'), ...
    'matFile',fullfile(root,'adaptive_arc_ik_new100.mat'), ...
    'maxRefinementRounds',4, ...
    'maxPoints',40, ...
    'initialPoints',16, ...
    'maxSegmentTcpErrorMm',0.5, ...
    'maxIntermediatePoints',4);
options=localMergeOptions(defaults,options);
options.seeds=double(options.seeds(:).');
if ~isfolder(options.outputDir), mkdir(options.outputDir); end

rows=repmat(localEmptyRow(),0,1);
fprintf('\n====================================================\n');
fprintf('A-ARC-IK+ NEW 100-SEED COLD-START BENCHMARK\n');
fprintf('Frozen implementation; no historical paths or graphs\n');
fprintf('Seeds %d ... %d (%d runs)\n',options.seeds(1),options.seeds(end),numel(options.seeds));
fprintf('====================================================\n');

for k=1:numel(options.seeds)
    seed=options.seeds(k);
    outputFile=fullfile(options.outputDir,sprintf('adaptive_new100_seed%d.mat',seed));
    roundPrefix=fullfile(options.outputDir,sprintf('adaptive_new100_seed%d_round',seed));
    opts=struct( ...
        'initialPoints',options.initialPoints, ...
        'maxRefinementRounds',options.maxRefinementRounds, ...
        'maxPoints',options.maxPoints, ...
        'maxSegmentTcpErrorMm',options.maxSegmentTcpErrorMm, ...
        'maxIntermediatePoints',options.maxIntermediatePoints, ...
        'randomSeed',seed, ...
        'enablePostRecovery',true, ...
        'outputFile',outputFile, ...
        'roundOutputPrefix',roundPrefix, ...
        'animate',false,'publish',false);
    fprintf('[%3d/%3d] seed=%d ... ',k,numel(options.seeds),seed);
    t=tic;
    try
        capture=evalc('r=replan_weld_actual_axis_adaptive_feasibility(opts);'); %#ok<NASGU>
        wall=toc(t);
        row=localRow(r,seed,wall);
        rows(end+1,1)=row; %#ok<AGROW>
        fprintf('%s first=%.3fs wall=%.3fs points=%d IK=%d stage=%s\n', ...
            ternary(row.complete,'PASS','FAIL'),row.firstSolutionTimeSec,wall, ...
            row.finalPointCount,row.ikCalls,char(row.failureStage));
    catch err
        wall=toc(t);
        row=localEmptyRow(); row.seed=seed; row.wallTimeSec=wall;
        row.errorMessage=string(err.message); row.failureStage="runtime_error";
        rows(end+1,1)=row; %#ok<AGROW>
        fprintf('ERROR wall=%.3fs %s\n',wall,err.message);
    end
end

rows=rows(:);
writetable(struct2table(rows),options.csvFile);
summary=struct('name','A-ARC-IK+ new100 cold-start', ...
    'seeds',options.seeds,'options',options,'rows',rows, ...
    'csvFile',options.csvFile,'matFile',options.matFile, ...
    'created',char(datetime('now')));
save(options.matFile,'summary','-v7.3');
localPrintSummary(rows);
fprintf('Saved CSV: %s\nSaved MAT: %s\n',options.csvFile,options.matFile);
end

function row=localRow(r,seed,wall)
row=localEmptyRow();
row.seed=seed; row.complete=logical(r.complete);
row.wallTimeSec=wall;
row.algorithmTimeSec=localNumber(r,'feasibilityAdaptiveAlgorithmTimeSec',NaN)+ ...
    localNumber(r,'precisionAlgorithmTimeSec',0);
row.firstSolutionTimeSec=localFirstSolutionTime(r);
row.finalPointCount=localNumber(r,'finalPointCount',NaN);
row.executionPointCount=0;
if isfield(r,'qPathExecution'), row.executionPointCount=size(r.qPathExecution,1); end
row.ikCalls=localNumber(r,'feasibilityAdaptiveIkCalls',0)+localNumber(r,'adaptiveIkCalls',0);
row.postRecoveryUsed=logical(localNumber(r,'postRecoveryUsed',false));
row.postRecoveryAttempts=localNumber(r,'postRecoveryAttempts',0);
row.postRecoveryTimeSec=localNumber(r,'postRecoveryTimeSec',0);
row.postRecoveryIkCalls=localNumber(r,'postRecoveryIkCalls',0);
row.furthestPoint=localFurthestPoint(r);
row.maxJointJumpDeg=localNumber(r,'maxJointJumpDeg',NaN);
row.maxTcpSegmentErrorMm=NaN;
if isfield(r,'finalTcpMetrics') && isfield(r.finalTcpMetrics,'segmentPositionErrorMm') && ...
        ~isempty(r.finalTcpMetrics.segmentPositionErrorMm)
    row.maxTcpSegmentErrorMm=max(r.finalTcpMetrics.segmentPositionErrorMm);
elseif isfield(r,'tcpMetrics') && isfield(r.tcpMetrics,'maxSegmentPositionErrorMm')
    row.maxTcpSegmentErrorMm=r.tcpMetrics.maxSegmentPositionErrorMm;
end
row.nodeCollisionFree=logical(localNumber(r,'nodeCollisionFree',false));
row.edgeCollisionFree=logical(localNumber(r,'edgeCollisionFree',false));
row.refinementRounds=localNumber(r,'refinementRoundCount',0);
row.insertedPoints=localNumber(r,'precisionInsertedPoints',0)+ ...
    max(0,row.finalPointCount-localNumber(r,'initialPoints',16));
row.failureStage=localFailureStage(r);
if row.complete, row.failureStage="PASS"; end
end

function t=localFirstSolutionTime(r)
% Algorithm time through the first complete planning round.  This is an
% algorithm-clock quantity; wall time remains separately reported.
t=NaN;
if ~isfield(r,'refinementRounds') || isempty(r.refinementRounds), return; end
rr=r.refinementRounds;
for i=1:numel(rr)
    if isfield(rr(i),'complete') && rr(i).complete
        vals=[rr(1:i).algorithmTimeSec];
        t=sum(vals(~isnan(vals)));
        return;
    end
end
end

function stage=localFailureStage(r)
stage="failed";
if isfield(r,'postRecoveryUsed') && r.postRecoveryUsed, stage="post_recovery"; end
if isfield(r,'refinementRounds') && ~isempty(r.refinementRounds)
    rr=r.refinementRounds;
    for i=numel(rr):-1:1
        if isfield(rr(i),'recovery') && strlength(string(rr(i).recovery))>0
            stage=string(rr(i).recovery); return;
        end
    end
end
if isfield(r,'stateBank')
    idx=localFurthestPoint(r);
    if idx>0 && idx<numel(r.stateBank), stage="failed_after_point_"+string(idx); end
end
end

function idx=localFurthestPoint(r)
idx=NaN;
if isfield(r,'stateBank')
    idx=find(~cellfun(@isempty,r.stateBank),1,'last');
    if isempty(idx), idx=0; end
end
end

function v=localNumber(s,name,default)
if isfield(s,name) && ~isempty(s.(name)), v=double(s.(name)); else, v=default; end
end

function row=localEmptyRow()
row=struct( ...
    'seed',NaN,'complete',false,'algorithmTimeSec',NaN,'firstSolutionTimeSec',NaN, ...
    'wallTimeSec',NaN,'finalPointCount',NaN,'executionPointCount',NaN, ...
    'ikCalls',NaN,'postRecoveryUsed',false,'postRecoveryAttempts',0, ...
    'postRecoveryTimeSec',0,'postRecoveryIkCalls',0,'furthestPoint',NaN, ...
    'maxJointJumpDeg',NaN,'maxTcpSegmentErrorMm',NaN, ...
    'nodeCollisionFree',false,'edgeCollisionFree',false, ...
    'refinementRounds',0,'insertedPoints',0,'failureStage',"", ...
    'errorMessage',"");
end

function localPrintSummary(rows)
if isempty(rows), return; end
pass=[rows.complete];
fprintf('\nSummary: PASS %d/%d (%.1f%%), FAIL %d\n',sum(pass),numel(rows), ...
    100*mean(pass),sum(~pass));
for st=unique(string({rows.failureStage}))
    fprintf('  stage %-32s : %d\n',st,sum(string({rows.failureStage})==st));
end
ok=pass & ~isnan([rows.firstSolutionTimeSec]);
if any(ok)
    fprintf('Successful first-solution algorithm time: median %.3fs, P90 %.3fs\n', ...
        median([rows(ok).firstSolutionTimeSec]),prctile([rows(ok).firstSolutionTimeSec],90));
end
end

function out=localMergeOptions(d,u)
out=d; names=fieldnames(u);
for i=1:numel(names), out.(names{i})=u.(names{i}); end
end

function out=ternary(c,a,b)
if c, out=a; else, out=b; end
end
