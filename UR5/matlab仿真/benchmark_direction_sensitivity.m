function summary = benchmark_direction_sensitivity(options)
%BENCHMARK_DIRECTION_SENSITIVITY Forward versus reverse seam traversal.
%
% Forward rows are read from the frozen 25 mm endpoint-margin benchmark.
% Reverse rows are cold-start runs with the same 16 initial seam anchors in
% reverse order.  The custom seam builder recomputes the tangent, so travel
% direction and the physical tool pose frame reverse consistently with the
% seam.  No ROS2/CoppeliaSim/real-robot action is performed.

if nargin < 1 || isempty(options), options = struct(); end
root = fileparts(mfilename('fullpath'));
defaults = struct( ...
    'seeds',20270001:20270100, ...
    'marginMm',25, ...
    'forwardDir',fullfile(root,'end_margin_benchmark_runs','25mm'), ...
    'outputDir',fullfile(root,'direction_reverse_runs'), ...
    'csvFile',fullfile(root,'direction_sensitivity.csv'), ...
    'matFile',fullfile(root,'direction_sensitivity.mat'), ...
    'initialPoints',16,'maxRefinementRounds',4,'maxPoints',40, ...
    'maxSegmentTcpErrorMm',0.5,'maxIntermediatePoints',4,'resume',true);
options = localMerge(defaults,options);
options.seeds = double(options.seeds(:).');
if ~isfolder(options.outputDir), mkdir(options.outputDir); end

% The 25 mm forward baseline has already been run independently.  Use one
% saved result only to obtain the exact 16-point seam anchor positions.
anchorFile = fullfile(options.forwardDir,sprintf('seed%d.mat',options.seeds(1)));
if ~isfile(anchorFile), error('Missing forward anchor file: %s',anchorFile); end
A = load(anchorFile,'result');
if ~isfield(A,'result') || ~isfield(A.result,'seam')
    error('Forward anchor file does not contain result.seam.');
end
forwardAnchors = A.result.seam.positionsWP;
if size(forwardAnchors,1) ~= options.initialPoints
    error('Expected %d forward anchors, got %d.',options.initialPoints,size(forwardAnchors,1));
end
reverseAnchors = flipud(forwardAnchors);

reverseRows = repmat(localRunRow(),0,1);
for k=1:numel(options.seeds)
    seed=options.seeds(k);
    outFile=fullfile(options.outputDir,sprintf('reverse_seed%d.mat',seed));
    prefix=fullfile(options.outputDir,sprintf('reverse_seed%d_round',seed));
    if options.resume && isfile(outFile)
        C=load(outFile,'result');
        if isfield(C,'result')
            row=localRowFromResult(C.result,seed,NaN,"reverse");
            reverseRows(end+1,1)=row; %#ok<AGROW>
            fprintf('[%3d/%3d] reverse seed=%d CACHED %s\n',k,numel(options.seeds),seed,ternary(row.complete,'PASS','FAIL'));
            continue;
        end
    end
    opts=struct('initialPoints',options.initialPoints, ...
        'initialSeamPositionsWP',reverseAnchors, ...
        'maxRefinementRounds',options.maxRefinementRounds, ...
        'maxPoints',options.maxPoints, ...
        'maxSegmentTcpErrorMm',options.maxSegmentTcpErrorMm, ...
        'maxIntermediatePoints',options.maxIntermediatePoints, ...
        'endMarginM',options.marginMm/1000, ...
        'randomSeed',seed,'enablePostRecovery',true, ...
        'outputFile',outFile,'roundOutputPrefix',prefix, ...
        'animate',false,'publish',false);
    fprintf('[%3d/%3d] reverse seed=%d ... ',k,numel(options.seeds),seed);
    t=tic;
    try
        capture=evalc('r=replan_weld_actual_axis_adaptive_feasibility(opts);'); %#ok<NASGU>
        wall=toc(t);
        row=localRowFromResult(r,seed,wall,"reverse");
        fprintf('%s wall=%.3fs points=%d IK=%d\n',ternary(row.complete,'PASS','FAIL'),wall,row.finalPointCount,row.ikCalls);
    catch err
        wall=toc(t); row=localRunRow(); row.seed=seed; row.direction="reverse";
        row.wallTimeSec=wall; row.failureStage="runtime_error"; row.errorMessage=string(err.message);
        fprintf('ERROR wall=%.3fs %s\n',wall,err.message);
    end
    reverseRows(end+1,1)=row; %#ok<AGROW>
end

% Load forward result rows from the already completed endpoint-margin run.
forwardRows=repmat(localRunRow(),0,1);
for k=1:numel(options.seeds)
    seed=options.seeds(k); f=fullfile(options.forwardDir,sprintf('seed%d.mat',seed));
    if isfile(f)
        C=load(f,'result');
        if isfield(C,'result'), forwardRows(end+1,1)=localRowFromResult(C.result,seed,NaN,"forward"); continue; end
    end
    row=localRunRow(); row.seed=seed; row.direction="forward"; row.failureStage="missing_forward";
    forwardRows(end+1,1)=row; %#ok<AGROW>
end

rows=repmat(localCompareRow(),0,1);
for k=1:numel(options.seeds)
    f=forwardRows(k); r=reverseRows(k); row=localCompareRow();
    row.seed=f.seed; row.forwardPass=f.complete; row.reversePass=r.complete;
    row.unionPass=f.complete || r.complete;
    row.forwardTimeSec=f.algorithmTimeSec; row.reverseTimeSec=r.algorithmTimeSec;
    row.forwardFailureStage=f.failureStage; row.reverseFailureStage=r.failureStage;
    row.forwardFurthestPoint=f.furthestPoint; row.reverseFurthestPoint=r.furthestPoint;
    row.forwardIkCalls=f.ikCalls; row.reverseIkCalls=r.ikCalls;
    rows(end+1,1)=row; %#ok<AGROW>
end

writetable(struct2table(rows),options.csvFile);
summary=struct('name','Forward/reverse ACCP direction sensitivity', ...
    'marginMm',options.marginMm,'seeds',options.seeds, ...
    'forwardAnchors',forwardAnchors,'reverseAnchors',reverseAnchors, ...
    'forwardRows',forwardRows,'reverseRows',reverseRows,'rows',rows, ...
    'csvFile',options.csvFile,'matFile',options.matFile,'created',char(datetime('now')));
save(options.matFile,'summary','-v7.3');
fprintf('\nDirection summary: forward %d/%d, reverse %d/%d, union %d/%d\n', ...
    sum([rows.forwardPass]),numel(rows),sum([rows.reversePass]),numel(rows),sum([rows.unionPass]),numel(rows));
fprintf('Saved: %s\n',options.csvFile);
end

function row=localRowFromResult(r,seed,wall,direction)
row=localRunRow(); row.seed=seed; row.direction=direction; row.complete=logical(r.complete); row.wallTimeSec=wall;
row.algorithmTimeSec=localNum(r,'feasibilityAdaptiveAlgorithmTimeSec',0)+localNum(r,'precisionAlgorithmTimeSec',0);
row.finalPointCount=localNum(r,'finalPointCount',NaN); row.ikCalls=localNum(r,'feasibilityAdaptiveIkCalls',0)+localNum(r,'adaptiveIkCalls',0);
row.postRecoveryUsed=logical(localNum(r,'postRecoveryUsed',false)); row.furthestPoint=localFurthest(r); row.maxJointJumpDeg=localNum(r,'maxJointJumpDeg',NaN);
row.failureStage=localFailureStage(r); if row.complete, row.failureStage="PASS"; end
if isfield(r,'finalTcpMetrics') && isfield(r.finalTcpMetrics,'segmentPositionErrorMm') && ~isempty(r.finalTcpMetrics.segmentPositionErrorMm)
    row.maxTcpSegmentErrorMm=max(r.finalTcpMetrics.segmentPositionErrorMm);
elseif isfield(r,'tcpMetrics') && isfield(r.tcpMetrics,'maxSegmentPositionErrorMm')
    row.maxTcpSegmentErrorMm=r.tcpMetrics.maxSegmentPositionErrorMm;
end
end

function row=localRunRow()
row=struct('seed',NaN,'direction',"",'complete',false,'algorithmTimeSec',NaN,'wallTimeSec',NaN,'finalPointCount',NaN,'ikCalls',NaN,'postRecoveryUsed',false,'furthestPoint',NaN,'maxJointJumpDeg',NaN,'maxTcpSegmentErrorMm',NaN,'failureStage',"",'errorMessage',"");
end
function row=localCompareRow()
row=struct('seed',NaN,'forwardPass',false,'reversePass',false,'unionPass',false,'forwardTimeSec',NaN,'reverseTimeSec',NaN,'forwardFailureStage',"",'reverseFailureStage',"",'forwardFurthestPoint',NaN,'reverseFurthestPoint',NaN,'forwardIkCalls',NaN,'reverseIkCalls',NaN);
end
function idx=localFurthest(r)
idx=NaN; if isfield(r,'stateBank'), idx=find(~cellfun(@isempty,r.stateBank),1,'last'); if isempty(idx), idx=0; end, end
end
function stage=localFailureStage(r)
stage="failed"; if isfield(r,'postRecoveryUsed') && r.postRecoveryUsed, stage="post_recovery"; end
if isfield(r,'refinementRounds') && ~isempty(r.refinementRounds), rr=r.refinementRounds; for i=numel(rr):-1:1, if isfield(rr(i),'recovery') && strlength(string(rr(i).recovery))>0, stage=string(rr(i).recovery); return; end, end, end
end
function v=localNum(s,name,d), if isfield(s,name)&&~isempty(s.(name)), v=double(s.(name)); else, v=d; end, end
function out=localMerge(d,c), out=d; f=fieldnames(c); for i=1:numel(f), out.(f{i})=c.(f{i}); end, end
function y=ternary(c,a,b), if c,y=a;else,y=b;end,end
