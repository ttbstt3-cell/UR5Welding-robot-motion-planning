function summary = time_aarc_ik_plus_coldstart(options)
%TIME_AARC_IK_PLUS_COLDSTART Real wall-clock timing for the frozen planner.
%
% Measures the FINAL production weld planner, which is A-ARC-IK+:
%   replan_weld_actual_axis_adaptive_feasibility
%     16-point sparse start -> adaptive local bisection
%     -> conditional beam/angular post-recovery
%     -> selective TCP execution-waypoint correction
%
% It does NOT measure the old 80-point continuation planner and does NOT
% measure the replan_weld_actual_axis_adaptive prototype.
%
% Every run is a genuine cold start: the validated branch reference
% (weld_anchor_points.json qWeldStart) is the only prior information, no
% previous qPath, state graph or checkpoint is read.  Scene construction and
% model loading are reported separately from the planner's own algorithm time,
% and both are also reported as the end-to-end wall clock.
%
% Two configurations are compared:
%   branchReferenceOn   qWeldStart used as initial seed / branch reference
%   branchReferenceOff  historical default (scene.qMat0)
%
% This script plans nothing that is sent anywhere.  It writes only its own
% result MAT/CSV and sends no ROS2 command, no CoppeliaSim command and no
% robot motion.

if nargin < 1 || isempty(options), options = struct(); end
here = fileparts(mfilename("fullpath"));
rootDir = fileparts(here);
addpath(genpath(rootDir));
addpath(here);
setenv('RRT_HEADLESS','1');

defaults = struct( ...
    'seeds',20260917, ...
    'configurations',{{'branchReferenceOn','branchReferenceOff'}}, ...
    'initialPoints',16, ...
    'maxRefinementRounds',4, ...
    'maxPoints',40, ...
    'maxSegmentTcpErrorMm',0.5, ...
    'maxIntermediatePoints',4, ...
    'endMarginM',0.025, ...
    'runDir',fullfile(rootDir,'aarc_ik_plus_timing_runs'), ...
    'matFile',fullfile(rootDir,'aarc_ik_plus_coldstart_timing.mat'), ...
    'csvFile',fullfile(rootDir,'aarc_ik_plus_coldstart_timing.csv'), ...
    'saveRuns',true);
options = localMerge(defaults,options);
options.seeds = double(options.seeds(:).');
if ~isfolder(options.runDir), mkdir(options.runDir); end

A = localLoadBranchReference(rootDir);
fprintf('\n========================================================\n');
fprintf('A-ARC-IK+ COLD-START WALL-CLOCK TIMING\n');
fprintf('Planner : replan_weld_actual_axis_adaptive_feasibility.m\n');
fprintf('Config  : %d points, <=%d points, TCP<=%.2f mm, post-recovery ON\n', ...
    options.initialPoints,options.maxPoints,options.maxSegmentTcpErrorMm);
fprintf('Seeds   : %d..%d (%d runs per configuration)\n', ...
    options.seeds(1),options.seeds(end),numel(options.seeds));
fprintf('========================================================\n');

rows = repmat(localEmptyRow(),0,1);
for ci = 1:numel(options.configurations)
    cfgName = options.configurations{ci};
    for k = 1:numel(options.seeds)
        seed = options.seeds(k);
        tag = sprintf('%s_seed%d',cfgName,seed);
        outFile = fullfile(options.runDir,[tag '.mat']);
        roundPrefix = [tag '_round'];

        opts = struct( ...
            'initialPoints',options.initialPoints, ...
            'maxRefinementRounds',options.maxRefinementRounds, ...
            'maxPoints',options.maxPoints, ...
            'maxSegmentTcpErrorMm',options.maxSegmentTcpErrorMm, ...
            'maxIntermediatePoints',options.maxIntermediatePoints, ...
            'endMarginM',options.endMarginM, ...
            'randomSeed',seed, ...
            'enablePostRecovery',true, ...
            'animate',false,'publish',false, ...
            'outputFile',outFile,'roundOutputPrefix',roundPrefix);
        if strcmp(cfgName,'branchReferenceOn')
            opts.branchReferenceQ = A.qWeldStart;
            opts.branchReferenceSource = 'weld_anchor_points.json qWeldStart';
        elseif ~strcmp(cfgName,'branchReferenceOff')
            error('Unknown configuration: %s',cfgName);
        end

        fprintf('[%s] seed=%d ... ',cfgName,seed);
        row = localEmptyRow();
        row.configuration = cfgName;
        row.seed = seed;
        tCold = tic;
        result = []; %#ok<NASGU>  % assigned inside evalc below
        try
            capture = evalc('result = replan_weld_actual_axis_adaptive_feasibility(opts);'); %#ok<NASGU>
            row.wallTimeSec = toc(tCold);
            row.complete = logical(result.complete);
            row.feasible = logical(result.feasibilityAdaptiveComplete);
            row.precisionComplete = logical(result.adaptiveCorrectionComplete);
            row.algorithmTimeSec = localNum(result,'feasibilityAdaptiveAlgorithmTimeSec',NaN) ...
                + localNum(result,'precisionAlgorithmTimeSec',NaN);
            row.plannerWallTimeSec = localNum(result,'feasibilityAdaptiveWallTimeSec',NaN);
            row.ikCalls = localNum(result,'feasibilityAdaptiveIkCalls',0) ...
                + localNum(result,'adaptiveIkCalls',0);
            row.finalPointCount = localNum(result,'finalPointCount',NaN);
            row.executionPointCount = size(result.qPathExecution,1);
            row.insertedPrecisionPoints = localNum(result,'precisionInsertedPoints',NaN);
            row.refinementRounds = localNum(result,'refinementRoundCount',NaN);
            row.postRecoveryUsed = logical(localNum(result,'postRecoveryUsed',0));
            row.maxJointJumpDeg = localNum(result,'maxJointJumpDeg',NaN);
            row.maxSegmentTcpErrorMm = localMaxSegTcp(result);
            row.failureStage = localFailureStage(result);
            if options.saveRuns
                save(fullfile(options.runDir,[tag '_result.mat']),'result','-v7.3');
            end
        catch err
            row.wallTimeSec = toc(tCold);
            row.complete = false;
            row.errorMessage = string(err.message);
            row.failureStage = "runtime_error";
        end
        rows(end+1,1) = row; %#ok<AGROW>
        fprintf('%s wall=%.3fs alg=%.3fs pts=%d IK=%d\n', ...
            localPassWord(row.complete),row.wallTimeSec,row.algorithmTimeSec, ...
            row.finalPointCount,row.ikCalls);
    end
end

rows = rows(:);
writetable(struct2table(rows),options.csvFile);

summary = struct();
summary.planner = 'A-ARC-IK+ (replan_weld_actual_axis_adaptive_feasibility.m)';
summary.created = char(datetime('now'));
summary.options = options;
summary.branchReferenceRad = A.qWeldStart;
summary.rows = rows;
summary.csvFile = options.csvFile;
summary.matFile = options.matFile;
summary.perConfiguration = localAggregate(rows,options.configurations);
save(options.matFile,'summary','-v7.3');

fprintf('\n================ TIMING SUMMARY ================\n');
for ci = 1:numel(options.configurations)
    cfgName = options.configurations{ci};
    a = summary.perConfiguration.(matlab.lang.makeValidName(cfgName));
    fprintf('%-20s runs=%d pass=%d/%d\n',cfgName,a.count,a.pass,a.count);
    fprintf('  cold-start wall  (incl. scene load): median %.3f  P90 %.3f  max %.3f s\n', ...
        a.wallMedian,a.wallP90,a.wallMax);
    fprintf('  planner algorithm time             : median %.3f  P90 %.3f  max %.3f s\n', ...
        a.algMedian,a.algP90,a.algMax);
    fprintf('  IK calls                           : median %.0f\n',a.ikMedian);
    fprintf('  final planning points              : median %.0f  max %.0f\n', ...
        a.pointsMedian,a.pointsMax);
    fprintf('  execution points                   : median %.0f\n',a.execMedian);
    fprintf('  max segment TCP error              : max %.4f mm\n',a.tcpMax);
    fprintf('  max joint jump                     : max %.3f deg\n',a.jumpMax);
end
fprintf('==============================================\n');
fprintf('Saved CSV: %s\nSaved MAT: %s\n',options.csvFile,options.matFile);
fprintf('No ROS2 command, no CoppeliaSim command and no robot motion was issued.\n');
end

function A = localAggregate(rows,names)
for ci = 1:numel(names)
    n = matlab.lang.makeValidName(names{ci});
    idx = strcmp({rows.configuration},names{ci});
    r = rows(idx);
    a = struct();
    a.count = numel(r);
    a.pass = sum([r.complete]);
    a.feasible = sum([r.feasible]);
    a.wall = [r.wallTimeSec];
    a.alg = [r.algorithmTimeSec];
    a.ik = [r.ikCalls];
    a.points = [r.finalPointCount];
    a.exec = [r.executionPointCount];
    a.wallMedian = localMedian(a.wall);
    a.wallP90 = localPct(a.wall,90);
    a.wallMax = localFiniteMax(a.wall);
    a.algMedian = localMedian(a.alg);
    a.algP90 = localPct(a.alg,90);
    a.algMax = localFiniteMax(a.alg);
    a.ikMedian = localMedian(a.ik);
    a.pointsMedian = localMedian(a.points);
    a.pointsMax = localFiniteMax(a.points);
    a.execMedian = localMedian(a.exec);
    a.tcpMax = localFiniteMax([r.maxSegmentTcpErrorMm]);
    a.jumpMax = localFiniteMax([r.maxJointJumpDeg]);
    a.postRecoveryRuns = sum([r.postRecoveryUsed]);
    A.(n) = a;
end
end

function A = localLoadBranchReference(rootDir)
f = fullfile(rootDir,'规划文件','weld_anchor_points.json');
if ~isfile(f), error('Branch reference file not found: %s',f); end
d = jsondecode(fileread(f));
A.qWeldStart = double(d.qWeldStart(:).');
A.source = 'weld_anchor_points.json';
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
end

function row = localEmptyRow()
row = struct('configuration',"",'seed',NaN,'complete',false,'feasible',false, ...
    'precisionComplete',false,'wallTimeSec',NaN,'plannerWallTimeSec',NaN, ...
    'algorithmTimeSec',NaN,'ikCalls',NaN,'finalPointCount',NaN, ...
    'executionPointCount',NaN,'insertedPrecisionPoints',NaN,'refinementRounds',NaN, ...
    'postRecoveryUsed',false,'maxJointJumpDeg',NaN,'maxSegmentTcpErrorMm',NaN, ...
    'failureStage',"",'errorMessage',"");
end

function v = localMaxSegTcp(r)
v = NaN;
if isfield(r,'finalTcpMetrics') && isstruct(r.finalTcpMetrics) && ...
        isfield(r.finalTcpMetrics,'segmentPositionErrorMm')
    x = r.finalTcpMetrics.segmentPositionErrorMm;
    if ~isempty(x), v = max(double(x(:))); end
end
end
function v = localNum(s,name,d)
if isfield(s,name) && ~isempty(s.(name)), v = double(s.(name)); else, v = d; end
end
function v = localMedian(x)
x = x(isfinite(x));
if isempty(x), v = NaN; else, v = median(x); end
end
function v = localPct(x,p)
x = sort(x(isfinite(x)));
if isempty(x), v = NaN; return; end
i = max(1,min(numel(x),ceil(p/100*numel(x))));
v = x(i);
end
function v = localFiniteMax(x)
x = x(isfinite(x));
if isempty(x), v = NaN; else, v = max(x); end
end
function w = localPassWord(c)
if c, w = 'PASS'; else, w = 'FAIL'; end
end
function out = localMerge(d,c)
out = d; f = fieldnames(c);
for i = 1:numel(f), out.(f{i}) = c.(f{i}); end
end