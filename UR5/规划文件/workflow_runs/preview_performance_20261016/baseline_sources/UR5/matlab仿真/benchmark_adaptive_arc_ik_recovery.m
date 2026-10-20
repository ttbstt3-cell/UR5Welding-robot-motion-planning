function rows = benchmark_adaptive_arc_ik_recovery(options)
%BENCHMARK_ADAPTIVE_ARC_IK_RECOVERY Test post-refinement recovery only.
% The seed list defaults to the six A-ARC-IK failures from the 30-run study.

if nargin < 1 || isempty(options), options=struct(); end
defaults=struct( ...
    "seeds",[20269012 20269018 20269019 20269022 20269024 20269027], ...
    "outputDir","adaptive_arc_ik_recovery_runs", ...
    "csvFile","adaptive_arc_ik_recovery.csv", ...
    "maxRefinementRounds",4,"maxPoints",40);
options=localMergeOptions(defaults,options);
here=fileparts(mfilename("fullpath")); outDir=options.outputDir;
if ~isfolder(outDir), outDir=fullfile(here,outDir); end
if ~isfolder(outDir), mkdir(outDir); end
rows=repmat(localEmptyRow(),0,1);
for k=1:numel(options.seeds)
    seed=options.seeds(k); fprintf('[%d/%d] recovery seed=%d ... ',k,numel(options.seeds),seed);
    opts=struct('initialPoints',16,'maxRefinementRounds',options.maxRefinementRounds, ...
        'maxPoints',options.maxPoints,'randomSeed',seed,'enablePostRecovery',true, ...
        'outputFile',fullfile(outDir,sprintf('adaptive_recovery_seed%05d.mat',seed)), ...
        'roundOutputPrefix',fullfile(outDir,sprintf('adaptive_recovery_seed%05d_round',seed)), ...
        'animate',false,'publish',false);
    t=tic;
    try
        capture=evalc('r=replan_weld_actual_axis_adaptive_feasibility(opts);'); %#ok<NASGU>
        row=localRow(r,seed,toc(t)); rows(end+1,1)=row; %#ok<AGROW>
        fprintf('%s, attempts=%d, time=%.3fs\n',ternary(row.complete,'PASS','FAIL'), ...
            row.postRecoveryAttempts,row.algorithmTimeSec);
    catch err
        row=localEmptyRow(); row.seed=seed; row.wallTimeSec=toc(t); row.errorMessage=string(err.message);
        rows(end+1,1)=row; fprintf('ERROR: %s\n',err.message);
    end
end
rows=rows(:); csvPath=options.csvFile;
if ~isfile(csvPath)&&~startsWith(string(csvPath),filesep), csvPath=fullfile(here,csvPath); end
writetable(struct2table(rows),csvPath);
fprintf('Saved: %s\n',csvPath); disp(struct2table(rows));
end

function row=localRow(r,seed,wall)
row=localEmptyRow(); row.seed=seed; row.complete=logical(r.complete);
row.algorithmTimeSec=r.feasibilityAdaptiveAlgorithmTimeSec;
if isfield(r,'precisionAlgorithmTimeSec'), row.algorithmTimeSec=row.algorithmTimeSec+r.precisionAlgorithmTimeSec; end
row.wallTimeSec=wall; row.finalPointCount=r.finalPointCount;
row.executionPointCount=0; if isfield(r,'qPathExecution'), row.executionPointCount=size(r.qPathExecution,1); end
row.postRecoveryUsed=r.postRecoveryUsed; row.postRecoveryAttempts=r.postRecoveryAttempts;
row.postRecoveryTimeSec=r.postRecoveryTimeSec; row.postRecoveryIkCalls=r.postRecoveryIkCalls;
row.furthestPoint=localFurthest(r.stateBank); row.maxJointJumpDeg=r.maxJointJumpDeg;
if isfield(r,'finalTcpMetrics'), row.maxSegmentTcpErrorMm=max(r.finalTcpMetrics.segmentPositionErrorMm); end
end

function row=localEmptyRow()
row=struct('seed',NaN,'complete',false,'algorithmTimeSec',NaN,'wallTimeSec',NaN, ...
    'finalPointCount',NaN,'executionPointCount',NaN,'postRecoveryUsed',false, ...
    'postRecoveryAttempts',0,'postRecoveryTimeSec',0,'postRecoveryIkCalls',0, ...
    'furthestPoint',NaN,'maxJointJumpDeg',NaN,'maxSegmentTcpErrorMm',NaN, ...
    'errorMessage',"");
end

function idx=localFurthest(bank)
idx=find(~cellfun(@isempty,bank),1,'last'); if isempty(idx), idx=0; end
end

function out=localMergeOptions(d,u)
out=d; n=fieldnames(u); for i=1:numel(n), out.(n{i})=u.(n{i}); end
end

function out=ternary(c,a,b)
if c, out=a; else, out=b; end
end
