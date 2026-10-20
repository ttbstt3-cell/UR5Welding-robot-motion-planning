function summary = benchmark_direction_cascade(options)
%BENCHMARK_DIRECTION_CASCADE Frozen Forward -> Reverse ACCP cascade.
%
% A new, held-out seed set is required for this experiment.  Each seed is
% first planned in the physical forward direction.  Reverse planning is
% attempted only when the forward result fails.  This is a benchmark
% harness; it does not modify the planner, ROS2, CoppeliaSim or the robot.

if nargin < 1 || isempty(options), options = struct(); end
root = fileparts(mfilename('fullpath'));
defaults = struct( ...
    'seeds',20270101:20270200, ...
    'marginMm',25, ...
    'forwardDir',fullfile(root,'direction_cascade_forward_runs'), ...
    'reverseDir',fullfile(root,'direction_cascade_reverse_runs'), ...
    'csvFile',fullfile(root,'direction_cascade_heldout.csv'), ...
    'matFile',fullfile(root,'direction_cascade_heldout.mat'), ...
    'initialPoints',16,'maxRefinementRounds',4,'maxPoints',40, ...
    'maxSegmentTcpErrorMm',0.5,'maxIntermediatePoints',4,'resume',true);
options = localMerge(defaults,options);
options.seeds = double(options.seeds(:).');
if ~isfolder(options.forwardDir), mkdir(options.forwardDir); end
if ~isfolder(options.reverseDir), mkdir(options.reverseDir); end

% The reverse experiment uses the same geometric anchors as the validated
% 25 mm forward benchmark, only in the opposite order.
reverseAnchors = localLoadForwardAnchors(root,options.initialPoints);
reverseAnchors = flipud(reverseAnchors);

rows = repmat(localCompareRow(),0,1);
for k = 1:numel(options.seeds)
    seed = options.seeds(k);
    fFile = fullfile(options.forwardDir,sprintf('forward_seed%d.mat',seed));
    rFile = fullfile(options.reverseDir,sprintf('reverse_seed%d.mat',seed));
    fprintf('[%3d/%3d] cascade seed=%d: forward ... ',k,numel(options.seeds),seed);

    [f,fWall,fErr] = localRunOrLoad(fFile,seed,'forward',options,@(outFile) ...
        localForwardOptions(options,seed,outFile));
    fprintf('%s',ternary(f.complete,'PASS','FAIL'));
    reverseAttempted = ~f.complete;
    r = localEmptyRun(); r.seed=seed; r.direction="reverse";
    rWall = NaN; rErr = "";
    if reverseAttempted
        fprintf(' -> reverse ... ');
        [r,rWall,rErr] = localRunOrLoad(rFile,seed,'reverse',options,@(outFile) ...
            localReverseOptions(options,seed,outFile,reverseAnchors));
        fprintf('%s',ternary(r.complete,'PASS','FAIL'));
    else
        fprintf(' -> reverse skipped');
    end

    row = localCompareRow();
    row.seed = seed;
    row.forwardPass = f.complete;
    row.reverseAttempted = reverseAttempted;
    row.reversePass = r.complete;
    row.cascadePass = f.complete || r.complete;
    row.cascadeDirection = ternary(f.complete,"forward",ternary(r.complete,"reverse","none"));
    row.forwardAlgorithmTimeSec = f.algorithmTimeSec;
    row.reverseAlgorithmTimeSec = r.algorithmTimeSec;
    row.forwardWallTimeSec = fWall;
    row.reverseWallTimeSec = rWall;
    row.cascadeWallTimeSec = fWall + ternary(reverseAttempted,rWall,0);
    row.forwardFailureStage = f.failureStage;
    row.reverseFailureStage = r.failureStage;
    row.forwardFurthestPoint = f.furthestPoint;
    row.reverseFurthestPoint = r.furthestPoint;
    row.forwardIkCalls = f.ikCalls;
    row.reverseIkCalls = r.ikCalls;
    row.forwardError = fErr;
    row.reverseError = rErr;
    rows(end+1,1) = row; %#ok<AGROW>
    fprintf(' | cascade=%s\n',ternary(row.cascadePass,'PASS','FAIL'));
end

T = struct2table(rows);
writetable(T,options.csvFile);
summary = struct('name','Held-out Forward -> Reverse ACCP cascade', ...
    'seeds',options.seeds,'marginMm',options.marginMm, ...
    'forwardAnchors',flipud(reverseAnchors),'reverseAnchors',reverseAnchors, ...
    'rows',rows,'csvFile',options.csvFile,'matFile',options.matFile, ...
    'created',char(datetime('now')));
save(options.matFile,'summary','-v7.3');

fprintf('\nCascade summary (%d held-out seeds):\n',numel(rows));
fprintf('Forward       %d/%d\n',sum([rows.forwardPass]),numel(rows));
fprintf('Reverse tried %d/%d\n',sum([rows.reverseAttempted]),numel(rows));
fprintf('Reverse PASS  %d/%d attempted\n',sum([rows.reversePass]),sum([rows.reverseAttempted]));
fprintf('Cascade PASS  %d/%d\n',sum([rows.cascadePass]),numel(rows));
fprintf('Saved: %s\n',options.csvFile);
end

function [row,wall,errText] = localRunOrLoad(file,seed,direction,options,makeOptions)
errText = "";
if options.resume && isfile(file)
    A = load(file,'result');
    if isfield(A,'result')
        row = localRowFromResult(A.result,seed,direction);
        wall = localNum(A.result,'feasibilityAdaptiveWallTimeSec',NaN);
        return;
    end
end
row = localEmptyRun(); row.seed=seed; row.direction=direction;
t = tic;
try
    opts = makeOptions(file);
    result = replan_weld_actual_axis_adaptive_feasibility(opts);
    wall = toc(t);
    row = localRowFromResult(result,seed,direction);
catch ex
    wall = toc(t); errText = string(ex.message);
    row.failureStage = "runtime_error"; row.errorMessage = errText;
end
end

function o = localForwardOptions(options,seed,outFile)
o = localCommonOptions(options,seed,outFile);
end

function o = localReverseOptions(options,seed,outFile,anchors)
o = localCommonOptions(options,seed,outFile);
o.initialSeamPositionsWP = anchors;
end

function o = localCommonOptions(options,seed,outFile)
root = fileparts(mfilename('fullpath'));
roundDir = fullfile(root,'direction_cascade_rounds');
if ~isfolder(roundDir), mkdir(roundDir); end
o = struct('initialPoints',options.initialPoints, ...
    'maxRefinementRounds',options.maxRefinementRounds, ...
    'maxPoints',options.maxPoints, ...
    'maxSegmentTcpErrorMm',options.maxSegmentTcpErrorMm, ...
    'maxIntermediatePoints',options.maxIntermediatePoints, ...
    'endMarginM',options.marginMm/1000, ...
    'randomSeed',seed,'enablePostRecovery',true, ...
    'outputFile',outFile, ...
    'roundOutputPrefix',fullfile(roundDir,sprintf('%s_seed%d',"run",seed)), ...
    'animate',false,'publish',false);
end

function anchors = localLoadForwardAnchors(root,n)
f = fullfile(root,'direction_sensitivity.mat');
if isfile(f)
    A = load(f,'summary');
    if isfield(A,'summary') && isfield(A.summary,'forwardAnchors')
        anchors = A.summary.forwardAnchors;
    end
end
if ~exist('anchors','var') || size(anchors,1) ~= n
    f = fullfile(root,'end_margin_benchmark_runs','25mm','seed20270001.mat');
    if ~isfile(f), error('Cannot locate validated 25 mm forward anchor file.'); end
    A = load(f,'result'); anchors = A.result.seam.positionsWP;
end
if size(anchors,2) ~= 3 || size(anchors,1) ~= n
    error('Forward anchor matrix must be %d-by-3.',n);
end
end

function row = localRowFromResult(r,seed,direction)
row = localEmptyRun(); row.seed=seed; row.direction=direction;
if isfield(r,'complete'), row.complete=logical(r.complete); end
row.algorithmTimeSec = localNum(r,'feasibilityAdaptiveAlgorithmTimeSec',0) + localNum(r,'precisionAlgorithmTimeSec',0);
row.finalPointCount = localNum(r,'finalPointCount',NaN);
row.ikCalls = localNum(r,'feasibilityAdaptiveIkCalls',0) + localNum(r,'adaptiveIkCalls',0);
row.furthestPoint = localFurthest(r);
row.maxJointJumpDeg = localNum(r,'maxJointJumpDeg',NaN);
row.failureStage = localFailureStage(r);
if row.complete, row.failureStage="PASS"; end
end

function row=localEmptyRun()
row=struct('seed',NaN,'direction',"",'complete',false, ...
    'algorithmTimeSec',NaN,'finalPointCount',NaN,'ikCalls',NaN, ...
    'furthestPoint',NaN,'maxJointJumpDeg',NaN,'failureStage',"", ...
    'errorMessage',"");
end

function row=localCompareRow()
row=struct('seed',NaN,'forwardPass',false,'reverseAttempted',false, ...
    'reversePass',false,'cascadePass',false,'cascadeDirection',"", ...
    'forwardAlgorithmTimeSec',NaN,'reverseAlgorithmTimeSec',NaN, ...
    'forwardWallTimeSec',NaN,'reverseWallTimeSec',NaN,'cascadeWallTimeSec',NaN, ...
    'forwardFailureStage',"",'reverseFailureStage',"", ...
    'forwardFurthestPoint',NaN,'reverseFurthestPoint',NaN, ...
    'forwardIkCalls',NaN,'reverseIkCalls',NaN, ...
    'forwardError',"",'reverseError',"");
end

function idx=localFurthest(r)
idx=NaN;
if isfield(r,'stateBank')
    idx=find(~cellfun(@isempty,r.stateBank),1,'last');
    if isempty(idx), idx=0; end
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
end

function v=localNum(s,name,d)
if isfield(s,name)&&~isempty(s.(name)), v=double(s.(name)); else, v=d; end
end
function out=localMerge(d,c)
out=d; f=fieldnames(c); for i=1:numel(f), out.(f{i})=c.(f{i}); end
end
function y=ternary(c,a,b)
if c, y=a; else, y=b; end
end
