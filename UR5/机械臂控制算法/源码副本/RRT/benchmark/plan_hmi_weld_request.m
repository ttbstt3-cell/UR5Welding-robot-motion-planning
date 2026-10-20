function report = plan_hmi_weld_request(request)
%PLAN_HMI_WELD_REQUEST HMI entry point for seam/full welding tasks.
%
% Seam planner (frozen 2026-10-08): ACCP =
%     replan_weld_actual_axis_adaptive_feasibility
%       16-point sparse start
%       -> adaptive local bisection
%       -> conditional beam/angular post-recovery
%       -> selective TCP execution-waypoint correction
%     wrapped in a Forward -> Reverse direction cascade (Reverse only runs
%     after Forward fails).
%
% The old 80-point continuation planner and the _adaptive prototype are NOT
% used here.  replan_weld_seam_generalization_core.m stays reserved for the
% Seam B/C generalization experiments.
%
% Branch control: weld_anchor_points.json qWeldStart is passed to the planner
% as branchReferenceQ, so the seam stays in the validated
% PreWeld/Weld/PostWeld IK branch.  This only steers branch selection; every
% hard constraint (IK tolerance, joint limits, gun-axis/welding-side, node and
% interpolated-edge collision, self collision) is unchanged.
%
% This wrapper only adapts planner output to the HMI schema and performs the
% final common validation.  It does not implement another IK or seam planner.

if nargin ~= 1 || ~isstruct(request)
    error('plan_hmi_weld_request expects one request struct.');
end
here = fileparts(mfilename('fullpath'));
rrtRoot = fileparts(here);
matlabDir = fullfile(fileparts(rrtRoot),'UR5','matlab仿真');
addpath(genpath(rrtRoot));
addpath(matlabDir);
setenv('RRT_HEADLESS','1');

task = localTask(request);
algorithm = localAlgorithm(request);
seed = localSeed(request);
outputFile = localPath(request,'outputFile',fullfile(pwd,'trajectory.mat'));
reportFile = localPath(request,'reportFile',fullfile(pwd,'report.json'));
requestFile = localPath(request,'requestFile','');
localEnsureDir(outputFile);
localEnsureDir(reportFile);

sceneScript = localGet(request,'sceneScript',fullfile(matlabDir,'sync_ur5_joint_states.m'));
anchorFile = localGet(request,'anchorFile',fullfile(fileparts(matlabDir),'规划文件','weld_anchor_points.json'));
anchors = localLoadAnchors(anchorFile,matlabDir);

% D4 fix: qHome must come from the HMI measured real_home, never from a
% CoppeliaSim-mapped value and never from silent zeros(1,6).
qHomeInput = localGet(request,'qStart',[]);
if isempty(qHomeInput)
    if ~isfield(anchors,'qHome') || isempty(anchors.qHome)
        msg = sprintf('HMI_WELD_PLAN_FAIL task=%s: no qStart in the request and no qHome in %s. Refusing to default to zeros(1,6).', task, anchorFile);
        error('%s',msg);
    end
    qHomeInput = anchors.qHome;
    fprintf('HMI_WELD_QHOME_FROM_ANCHORS source=%s\n', ...
        localFieldText(anchors,'qHomeSource','unknown'));
end
qHome = localQ(qHomeInput,'qStart/qHome');

% Task-specific start/goal resolution.  qGoal is NOT silently defaulted to
% qStart: for RealHome_to_PreWeld the goal is the validated PreWeld anchor,
% and for PostWeld_to_RealHome it is the real-robot Home.  Defaulting qGoal to
% qStart would silently plan a zero-length motion and report it as success.
switch task
    case 'RealHome_to_PreWeld'
        taskStart = qHome;
        taskGoal = localQ(localGet(request,'qGoal',anchors.qPreWeld),'qGoal');
    case 'PostWeld_to_RealHome'
        taskStart = localQ(anchors.qPostWeld,'qPostWeld');
        taskGoal = localQ(localGet(request,'qGoal',qHome),'qGoal');
    otherwise
        taskStart = qHome;
        taskGoal = localQ(localGet(request,'qGoal',qHome),'qGoal');
end
qGoalInput = taskGoal;

fprintf('HMI_WELD_PLAN_START task=%s algorithm=%s seed=%d\n',task,algorithm,seed);
fprintf('HMI_WELD_STAGE stage=PREPARING\n');
tTotal = tic;

% One calibrated scene is used for the seam result and common final checks.
scene = loadUR5JointScene(sceneScript);
sceneProvenance = localSceneProvenance(scene,sceneScript);
anchorDiagnostics = localAnchorDiagnostics(anchors,scene,anchorFile);
planningScene = upper(char(string(localGet(request,'planning_scene','FROZEN_SIMULATION'))));
request.planning_scene = planningScene;
% Carry the same diagnostics into free-space sub-planner reports.  This is
% especially important when the RRT fails before the wrapper can assemble a
% final qPath report.
request.anchor_diagnostics = anchorDiagnostics;
request.scene_provenance = sceneProvenance;
fprintf('HMI_WELD_ANCHOR_CHECK qPreWeldLimits=%d qPreWeldFree=%d staleVsRealWorkpiece=%d deltaMm=%.3f\n', ...
    anchorDiagnostics.qPreWeld.within_limits,anchorDiagnostics.qPreWeld.collision_free, ...
    anchorDiagnostics.real_workpiece_mismatch,anchorDiagnostics.real_workpiece_delta_norm_mm);
if strcmp(planningScene,'REAL_CALIBRATION') && ...
        (~localGet(sceneProvenance,'real_tcp_calibration_loaded_into_scene',false) || ...
         ~localGet(sceneProvenance,'real_workpiece_pose_loaded_into_scene',false) || ...
         anchorDiagnostics.real_workpiece_mismatch)
    reason='REAL_CALIBRATION scene requested but real TCP/workpiece transforms are not loaded into the MATLAB scene, or qPreWeld is stale';
    localWriteFailureReport(reportFile,struct('task',task,'planning_scene',planningScene, ...
        'success',false,'failure_reason',['PLAN_FAILED: ' reason], ...
        'anchor_diagnostics',anchorDiagnostics,'scene_provenance',sceneProvenance));
    error('HMI_WELD_PLAN_FAILED planning_scene=REAL_CALIBRATION: %s; report=%s',reason,reportFile);
end

% weldInfo is only produced by the seam stage; initialize it so the
% free-space tasks can still report the same fields safely.
weldInfo = struct();
freeInfo = [];
% Per-stage point counts, filled in from the actual part sizes so the HMI
% never has to re-derive the approach / weld / exit split.
stageCounts = struct('approach',NaN,'weld',NaN,'exit',NaN,'total',NaN);

switch task
    case 'RealHome_to_PreWeld'
        fprintf('HMI_WELD_STAGE stage=APPROACH\n');
        [qPath,stageReports,freeInfo] = localFreeSpaceStage( ...
            sceneScript,taskStart,taskGoal,algorithm,seed,request,outputFile,reportFile, ...
            'approach');
        tcpPath = localTcpPath(scene,qPath);
        seamInfo = localEmptySeamInfo();
        stageCounts.approach = size(qPath,1);
    case 'PostWeld_to_RealHome'
        fprintf('HMI_WELD_STAGE stage=EXIT\n');
        [qPath,stageReports,freeInfo] = localFreeSpaceStage( ...
            sceneScript,anchors.qPostWeld,qGoalInput,algorithm,seed,request, ...
            outputFile,reportFile,'exit');
        tcpPath = localTcpPath(scene,qPath);
        seamInfo = localEmptySeamInfo();
        stageCounts.exit = size(qPath,1);
    case 'Weld_Seam'
        fprintf('HMI_WELD_STAGE stage=WELD\n');
        [qPath,seamInfo,stageReports,weldInfo] = localWeldStage( ...
            scene,matlabDir,anchors,seed,request,outputFile);
        tcpPath = localTcpPath(scene,qPath);
        stageCounts.weld = size(qPath,1);
    case 'Full_Welding_Task'
        approachDir = fullfile(fileparts(outputFile),'full_approach');
        exitDir = fullfile(fileparts(outputFile),'full_exit');
        if ~isfolder(approachDir), mkdir(approachDir); end
        if ~isfolder(exitDir), mkdir(exitDir); end

        % Stage order is APPROACH -> WELD -> EXIT so the reported stage
        % sequence matches the physical task order.
        fprintf('HMI_WELD_STAGE stage=APPROACH\n');
        approachOk = true; exitOk = true; freeInfo = struct();
        try
            qApproach = localFreeSpacePath(sceneScript,taskStart,anchors.qPreWeld, ...
                algorithm,seed,request, ...
                fullfile(approachDir,'trajectory.mat'),fullfile(approachDir,'report.json'));
            freeInfo.approach_report = localReadReport(fullfile(approachDir,'report.json'));
        catch err
            approachOk = false;
            freeInfo.approachError = string(err.message);
            fprintf('HMI_WELD_STAGE_FAIL stage=approach error=%s\n',err.message);
            localWriteFailureReport(reportFile,struct('task',task,'stage','approach', ...
                'success',false,'failure_reason',['PLAN_FAILED approach: ' err.message], ...
                'anchor_diagnostics',anchorDiagnostics,'scene_provenance',sceneProvenance));
            qApproach = zeros(0,6);
        end
        if ~approachOk
            error('HMI_WELD_PLAN_FAILED task=%s free-space approach stage failed; report=%s.',task,reportFile);
        end

        fprintf('HMI_WELD_STAGE stage=WELD\n');
        [qWeld,seamInfo,~,weldInfo] = localWeldStage( ...
            scene,matlabDir,anchors,seed,request,outputFile);

        fprintf('HMI_WELD_STAGE stage=EXIT\n');
        try
            qExit = localFreeSpacePath(sceneScript,anchors.qPostWeld,taskGoal, ...
                algorithm,seed+1,request, ...
                fullfile(exitDir,'trajectory.mat'),fullfile(exitDir,'report.json'));
            freeInfo.exit_report = localReadReport(fullfile(exitDir,'report.json'));
        catch err
            exitOk = false;
            freeInfo.exitError = string(err.message);
            fprintf('HMI_WELD_STAGE_FAIL stage=exit error=%s\n',err.message);
            localWriteFailureReport(reportFile,struct('task',task,'stage','exit', ...
                'success',false,'failure_reason',['PLAN_FAILED exit: ' err.message], ...
                'anchor_diagnostics',anchorDiagnostics,'scene_provenance',sceneProvenance));
            qExit = zeros(0,6);
        end

        if ~approachOk || ~exitOk
            error('HMI_WELD_PLAN_FAILED task=%s free-space stage failed (approach=%d exit=%d); report=%s.', ...
                task,approachOk,exitOk,reportFile);
        end

        % D0 fix: qPath is the single authoritative splice.  tcpPath is
        % derived from the SAME rows, so the row counts cannot diverge.
        qPath = localConcatPaths({qApproach,qWeld,qExit});
        tcpPath = localTcpPath(scene,qPath);
        % Row accounting.  Both junctions are de-duplicated, so attribute the
        % shared qPreWeld row to approach and the shared qPostWeld row to the
        % weld block.  The three numbers then sum exactly to total.
        stageCounts.approach = size(qApproach,1);
        stageCounts.weld = size(qWeld,1) - 1;
        stageCounts.exit = size(qExit,1) - 1;
        stageReports = struct('approach',true, ...
            'weld_seam',logical(weldInfo.collisionCheckPass), ...
            'exit',true);
    otherwise
        error('Unsupported weld task: %s',task);
end

fprintf('HMI_WELD_STAGE stage=VALIDATING\n');
% Never allow an empty or malformed stage result to reach endpoint checking.
% The stage planner must have produced at least the requested endpoints.
if isempty(qPath) || size(qPath,2) ~= 6 || any(~isfinite(qPath(:)))
    localWriteFailureReport(reportFile,struct('task',task,'algorithm',algorithm, ...
        'success',false,'failure_reason','PLAN_FAILED: qPath is empty or malformed before validation', ...
        'anchor_diagnostics',anchorDiagnostics,'scene_provenance',sceneProvenance));
    error('HMI_WELD_PLAN_FAILED task=%s: qPath is empty or malformed before endpoint validation; report=%s', ...
        task,reportFile);
end
% Splice continuity can leave a joint sitting one turn above/below the branch
% the HMI expects, which then trips the +-2*pi software limit even though the
% physical pose is identical.  Bring the whole path into the canonical
% [-pi, pi] branch before validating and exporting.
[canonical,branchTurns] = localCanonicalizeBranch(qPath);
branchCanonicalized = any(branchTurns ~= 0);
qPath = canonical;
continuity = localContinuity(scene,qPath,tcpPath);
dense = localDenseValidate(scene,qPath, ...
    deg2rad(localGet(request,'validationResolutionDeg',0.5)));
collisionPass = dense.pass;
continuityPass = continuity.pass;

% D0 regression guard: the two representations must have identical rows.
rowsConsistent = (size(qPath,1) == size(tcpPath,1));
if ~rowsConsistent
    error('HMI_WELD_PLAN_FAIL task=%s: qPath rows=%d != tcpPath rows=%d.', ...
        task,size(qPath,1),size(tcpPath,1));
end

% Endpoint guard.  A planner can return a self-consistent, collision-free
% path that never reaches the requested goal (for example a zero-length
% Home->Home motion when qGoal is missing).  Verify the executed path really
% starts at the task start and ends at the task goal, comparing 2*pi
% equivalent representations, before any success is reported.
endpointErr = localEndpointErrors(qPath,taskStart,taskGoal);
if endpointErr.startRad > 1e-6 || endpointErr.goalRad > 1e-6
    msg = sprintf('HMI_WELD_PLAN_FAIL task=%s path does not span the requested endpoints (start error %.3e rad, goal error %.3e rad).', task, endpointErr.startRad, endpointErr.goalRad);
    error('%s',msg);
end

% D5 fix: a failed common validation must abort instead of exporting an
% invalid trajectory into the HMI MAT file.
if ~(collisionPass && continuityPass)
    errorMessage = sprintf('common validation failed: collision=%d continuity=%d (maxJointJump=%.3f deg, maxTcpError=%.3f mm)', collisionPass, continuityPass, continuity.maxJointJumpDeg, continuity.maxTcpErrorMm);
    error('HMI_WELD_PLAN_FAIL task=%s %s',task,errorMessage);
end
errorMessage = '';

[timeFromStart,timeDetails] = parameterize_joint_path(qPath, ...
    'MaxVelocityDegPerSec',localGet(request,'maxVelocityDegPerSec',5), ...
    'MaxAccelerationDegPerSec2',localGet(request,'maxAccelerationDegPerSec2',15));
fprintf('HMI_WELD_STAGE stage=EXPORTING\n');

exportReport = export_hmi_trajectory(qPath,outputFile, ...
    'CoordinateSpace','ros_real','TimeFromStart',timeFromStart, ...
    'TrajectoryName',sprintf('HMI_%s',task), ...
    'Source','MATLAB_HMI_WELD_PLANNER_ACCP', ...
    'Description','ACCP seam planner with Forward->Reverse cascade; offline only.', ...
    'CollisionVerified',collisionPass,'CollisionCheckPass',collisionPass, ...
    'CollisionSampleCount',dense.collision_configuration_checks, ...
    'MinimumClearanceM',NaN,'ReportFile',reportFile);

% D1 fix: export_hmi_trajectory saves metadata into the MAT but does not
% return it in its report struct.  Read it back instead of assuming a field.
saved = load(outputFile,'metadata');
if ~isfield(saved,'metadata')
    error('HMI_WELD_PLAN_FAIL task=%s: metadata missing from %s.',task,outputFile);
end
metadata = saved.metadata;
metadata.task = task; metadata.algorithm = algorithm; metadata.seed = seed;
metadata.collision_check_pass = collisionPass;
metadata.continuity_check_pass = continuityPass;
metadata.seam_point_count = seamInfo.pointCount;
metadata.branch_canonicalized = branchCanonicalized;
metadata.branch_turns = branchTurns;
metadata.stage_reports = stageReports;
% The HMI supplies immutable calibration fingerprints with each request.
% Preserve them in the exported trajectory so the GUI can mark a path STALE
% after a TCP/workpiece/URDF change.  This is metadata only; it does not
% alter the planner's geometry or bypass any collision gate.
if isfield(request,'calibration_binding') && isstruct(request.calibration_binding)
    metadata.calibration_binding = request.calibration_binding;
end
if isfield(request,'planning_scene')
    metadata.planning_scene = char(string(request.planning_scene));
end
if isfield(weldInfo,'branchReferenceRad')
    metadata.branch_reference_rad = weldInfo.branchReferenceRad;
    metadata.branch_reference_source = weldInfo.branchReferenceSource;
end
if isfield(weldInfo,'direction')
    metadata.seam_direction = weldInfo.direction;
    metadata.seam_direction_cascade = weldInfo.cascade;
end
save(outputFile,'tcpPath','metadata','-append');

report = exportReport;
report.task = task; report.algorithm = algorithm; report.seed = seed;
report.planning_scene = planningScene;
report.success = collisionPass && continuityPass;
report.first_solution_time = NaN;
report.planning_time = toc(tTotal);
report.wall_time_sec = toc(tTotal);
report.path_length_rad = localPathLength(qPath);
report.raw_point_count = size(qPath,1);
report.execution_point_count = size(qPath,1);
report.collision_check_pass = collisionPass;
report.continuity_check_pass = continuityPass;
report.duration = timeFromStart(end);
report.duration_sec = timeFromStart(end);
report.time_parameterization = timeDetails;
report.tcp_point_count = size(tcpPath,1);
report.qpath_tcpath_rows_match = rowsConsistent;
report.branch_canonicalized = branchCanonicalized;
report.branch_turns = branchTurns;
report.seam_point_count = seamInfo.pointCount;
report.seam_execution_point_count = seamInfo.executionPointCount;
report.seam_planner = seamInfo.planner;
report.seam_direction = localFieldText(seamInfo,'direction','');
report.seam_cascade = localFieldText(seamInfo,'cascade','');
report.stage_point_counts = stageCounts;
report.stage_point_counts.total = size(qPath,1);
report.branch_reference_rad = localFieldNumeric(weldInfo,'branchReferenceRad',NaN(1,6));
report.branch_reference_source = localFieldText(weldInfo,'branchReferenceSource','');
report.seam_planner_time_sec = localFieldNumeric(weldInfo,'plannerWallTimeSec',NaN);
report.seam_algorithm_time_sec = localFieldNumeric(weldInfo,'plannerAlgorithmTimeSec',NaN);
report.max_joint_jump_deg = continuity.maxJointJumpDeg;
report.max_tcp_error_mm = continuity.maxTcpErrorMm;
report.max_tcp_step_mm = continuity.maxTcpStepMm;
report.max_orientation_change_deg = continuity.maxOrientationChangeDeg;
report.dense_validation = dense;
report.stage_reports = stageReports;
report.anchor_diagnostics = anchorDiagnostics;
report.scene_provenance = sceneProvenance;
report.output_file = outputFile;
report.report_file = reportFile;
report.request_file = requestFile;
report.q_start_rad = qPath(1,:);
report.q_end_rad = qPath(end,:);
report.endpoint_start_error_rad = endpointErr.startRad;
report.endpoint_goal_error_rad = endpointErr.goalRad;
report.note = errorMessage;
if ~isempty(freeInfo)
    report.free_space_info = freeInfo;
end
localWriteJson(report,reportFile);
fmt = 'HMI_WELD_PLAN_DONE task=%s success=%d points=%d seam=%d seamExec=%d collisionPASS=%d continuityPASS=%d rowsMatch=%d maxJump=%.3f maxTcpErr=%.4fmm wall=%.3f\n';
fprintf(fmt, ...
    task,report.success,size(qPath,1),seamInfo.pointCount,seamInfo.executionPointCount, ...
    collisionPass,continuityPass,rowsConsistent,continuity.maxJointJumpDeg, ...
    continuity.maxTcpErrorMm,report.wall_time_sec);
end

function [qPath,seamInfo,stageReports,weldInfo] = localWeldStage(scene,matlabDir,anchors,seed,request,outputFile)
% The sole seam-planning call.  ACCP with a Forward -> Reverse cascade.
% Absolute paths are passed for the wrapper outputs; the planner's own
% round/recovery files use bare names next to the planner directory, so their
% internal fullfile(here,...) never rebuilds a nonexistent parent path.
stageReports = struct('approach',false,'weld_seam',false,'exit',false);
weldInfo = struct();

runDir = fileparts(outputFile);
branchRef = localQ(localGet(anchors,'branchReferenceQ',anchors.qWeldStart),'branchReferenceQ');

baseOpts = struct( ...
    'initialPoints',16, ...
    'maxRefinementRounds',4, ...
    'maxPoints',40, ...
    'maxSegmentTcpErrorMm',0.5, ...
    'maxIntermediatePoints',4, ...
    'endMarginM',0.025, ...
    'enablePostRecovery',true, ...
    'branchReferenceQ',branchRef, ...
    'branchReferenceSource','weld_anchor_points.json qWeldStart', ...
    'animate',false,'publish',false);

% D7 fix: propagate the requested time budget.  The planner checks it
% cooperatively between rounds; a single in-progress round may still overrun
% and the true elapsed time is always reported.
budget = localGet(request,'maxPlanningTimeSec',[]);
if ~isempty(budget) && isfinite(budget) && budget > 0
    baseOpts.maxPlanningTimeSec = double(budget);
end

% Reverse uses the same geometry in reverse order, recomputed by the seam
% builder so travel/roll follow the reversed traversal direction.
[forward,forwardTime] = localRunArcIk(baseOpts,seed,runDir,'forward',[]);
if forward.result.complete
    raw = forward.result;
    direction = 'forward';
    cascade = 'forward_first_pass';
    plannerWall = forwardTime;
else
    fprintf('HMI_WELD_SEAM_FORWARD_FAIL -> trying Reverse ACCP.\n');
    forwardAnchors = localSeamAnchors(scene,baseOpts.initialPoints,baseOpts.endMarginM);
    reverseAnchors = flipud(forwardAnchors);
    [reverse,reverseTime] = localRunArcIk(baseOpts,seed,runDir,'reverse',reverseAnchors);
    if ~reverse.result.complete
        msg = sprintf('HMI_WELD_PLAN_FAIL: ACCP Forward and Reverse both failed (forward furthest point %s, reverse furthest point %s).', mat2str(localFurthest(forward.result)), mat2str(localFurthest(reverse.result)));
        error('%s',msg);
    end
    raw = reverse.result;
    direction = 'reverse';
    cascade = 'forward_failed_then_reverse';
    plannerWall = forwardTime + reverseTime;
    weldInfo.forwardWallTimeSec = forwardTime;
    weldInfo.reverseWallTimeSec = reverseTime;
end

if isfield(raw,'qPathExecution') && ~isempty(raw.qPathExecution)
    qSeam = double(raw.qPathExecution);
else
    qSeam = double(raw.qPath);
end
if isempty(qSeam) || size(qSeam,2) ~= 6 || any(~isfinite(qSeam(:)))
    error('HMI_WELD_PLAN_FAILED: ACCP returned an empty or malformed seam qPath.');
end
qSeam = localLiftTo(qSeam,anchors.qWeldStart);

% Splice on the joint level only; tcpPath is derived from the final qPath in
% the caller, so the two never diverge (D0 fix).
qPath = localConcatPaths({anchors.qPreWeld,qSeam,anchors.qPostWeld});

weldInfo.direction = direction;
weldInfo.cascade = cascade;
weldInfo.plannerWallTimeSec = plannerWall;
weldInfo.plannerAlgorithmTimeSec = localFieldNumeric(raw,'feasibilityAdaptiveAlgorithmTimeSec',NaN) ...
    + localFieldNumeric(raw,'precisionAlgorithmTimeSec',NaN);
weldInfo.branchReferenceRad = localFieldNumeric(raw,'branchReferenceRad',NaN(1,6));
weldInfo.branchReferenceSource = localFieldText(raw,'branchReferenceSource','');
weldInfo.timeBudgetSec = localFieldNumeric(raw,'timeBudgetSec',NaN);
weldInfo.timeBudgetExceeded = logical(localFieldNumeric(raw,'timeBudgetExceeded',0));
weldInfo.finalPointCount = localFieldNumeric(raw,'finalPointCount',NaN);
weldInfo.collisionCheckPass = ...
    logical(localFieldNumeric(raw,'nodeCollisionFree',0)) && ...
    logical(localFieldNumeric(raw,'edgeCollisionFree',0));

stageReports = struct('approach',false,'weld_seam',weldInfo.collisionCheckPass,'exit',false);
seamInfo = struct('pointCount',localFieldNumeric(raw,'finalPointCount',NaN), ...
    'executionPointCount',size(qSeam,1), ...
    'planner','ACCP (replan_weld_actual_axis_adaptive_feasibility.m)', ...
    'direction',direction,'cascade',cascade, ...
    'resultFile',localFieldText(raw,'outputFile',''));
end

function [out,wallSec] = localRunArcIk(baseOpts,seed,runDir,direction,initialSeamPositionsWP)
tag = sprintf('aarc_ik_%s_seed%d',direction,seed);
outFile = fullfile(runDir,[tag '.mat']);
roundPrefix = [tag '_round'];
opts = baseOpts;
opts.randomSeed = seed;
opts.outputFile = outFile;
opts.roundOutputPrefix = roundPrefix;
if ~isempty(initialSeamPositionsWP)
    opts.initialSeamPositionsWP = initialSeamPositionsWP;
end
t = tic;
result = replan_weld_actual_axis_adaptive_feasibility(opts);
wallSec = toc(t);
out = struct('result',result);
end

function P = localSeamAnchors(scene,numPoints,endMarginM)
% Recreate the forward sparse seam anchors, in Workpiece coordinates, so the
% reverse run uses identical geometry with reversed ordering.
info = scene.workpieceProxyInfo;
bottom = localFindProxy(info,'WorkpieceCollision_bottom');
wallX0 = localFindProxy(info,'WorkpieceCollision_wall_x0');
wallX1 = localFindProxy(info,'WorkpieceCollision_wall_x1');
wallY1 = localFindProxy(info,'WorkpieceCollision_wall_y1');
bottomMin = bottom.T_workpiece_proxy(1:3,4)-bottom.size_m(:)/2;
bottomMax = bottom.T_workpiece_proxy(1:3,4)+bottom.size_m(:)/2;
xMin = max(bottomMin(1),wallX0.T_workpiece_proxy(1,4)+wallX0.size_m(1)/2);
xMax = min(bottomMax(1),wallX1.T_workpiece_proxy(1,4)-wallX1.size_m(1)/2);
ySeam = wallY1.T_workpiece_proxy(2,4)-wallY1.size_m(2)/2;
zSeam = bottomMax(3);
p0 = [xMin+endMarginM;ySeam;zSeam];
p1 = [xMax-endMarginM;ySeam;zSeam];
if p1(1)<=p0(1), error('end margin %.1f mm is too large.',endMarginM*1000); end
s = linspace(0,1,numPoints).';
P = (1-s).*p0.'+s.*p1.';
end

function s = localFindProxy(info,name)
idx = find(strcmp(string({info.name}),string(name)),1);
if isempty(idx), error('Workpiece proxy not found: %s',name); end
s = info(idx);
end

function q = localLiftTo(q,reference)
% Keep the continuous 2*pi branch nearest the anchor.  Never wraps into
% [-pi,pi] and never changes the physical configuration.
q = double(q);
if isempty(q) || isempty(reference), return; end
ref = double(reference(:).');
if size(q,2) ~= numel(ref), error('joint dimension mismatch in localLiftTo.'); end
for i = 1:size(q,1)
    q(i,:) = q(i,:) - 2*pi*round((q(i,:)-ref)/(2*pi));
end
end

function [qPath,stageReports,freeInfo] = localFreeSpaceStage(sceneScript,qStart,qGoal,algorithm,seed,request,outputFile,reportFile,stageName)
try
    qPath = localFreeSpacePath(sceneScript,qStart,qGoal,algorithm,seed,request,outputFile,reportFile);
    ok = true; errText = "";
catch err
    qPath = zeros(0,6); ok = false; errText = string(err.message);
    fprintf('HMI_WELD_STAGE_FAIL stage=%s error=%s\n',stageName,err.message);
    localWriteFailureReport(reportFile,struct('success',false, ...
        'failure_reason',sprintf('PLAN_FAILED stage=%s: %s',stageName,err.message), ...
        'stage',stageName,'stage_output_file',outputFile, ...
        'anchor_diagnostics',localGet(request,'anchor_diagnostics',struct()), ...
        'scene_provenance',localGet(request,'scene_provenance',struct())));
    error('HMI_WELD_PLAN_FAILED stage=%s: %s; report=%s',stageName,err.message,reportFile);
end
stageReports = struct('approach',strcmp(stageName,'approach') && ok, ...
    'weld_seam',false,'exit',strcmp(stageName,'exit') && ok);
freeInfo = struct('stage',stageName,'success',ok);
if ~isempty(errText), freeInfo.error = errText; end
freeInfo.report_file = reportFile;
if ok && isfile(reportFile)
    try
        stageReport = jsondecode(fileread(reportFile));
        freeInfo.planner = localGet(stageReport,'planner',struct());
        freeInfo.planning_time_sec = localGet(stageReport,'planning_time',NaN);
        freeInfo.collision_configuration_checks = localGet(stageReport,'collision_sample_count',NaN);
        freeInfo.edge_collision_checks = localGet(localGet(stageReport,'planner',struct()),'edgeCollisionChecks',NaN);
        freeInfo.failure_reason = localGet(stageReport,'failure_reason','');
    catch err
        freeInfo.report_read_error = char(string(err.message));
    end
end
end

function qPath = localFreeSpacePath(sceneScript,qStart,qGoal,algorithm,seed,request,outputFile,reportFile)
req = struct('qStart',qStart,'qGoal',qGoal,'algorithm',algorithm,'seed',seed, ...
    'request_id',localGet(request,'request_id','weld_stage'), ...
    'outputFile',outputFile,'reportFile',reportFile,'sceneScript',sceneScript, ...
    'requestFile','');
req.maxPlanningTimeSec = localGet(request,'maxPlanningTimeSec',30);
if isfield(request,'anchor_diagnostics'), req.anchor_diagnostics=request.anchor_diagnostics; end
if isfield(request,'scene_provenance'), req.scene_provenance=request.scene_provenance; end
plan_hmi_request(req);
if ~isfile(outputFile)
    error('RRT stage reported success but did not create trajectory file: %s',outputFile);
end
s = load(outputFile,'qPath');
if ~isfield(s,'qPath') || isempty(s.qPath)
    error('RRT stage returned an empty qPath (interface/output parsing failure).');
end
qPath = double(s.qPath);
if size(qPath,2) ~= 6 || any(~isfinite(qPath(:)))
    error('RRT stage returned malformed qPath with size %s.',mat2str(size(qPath)));
end
end

function P = localTcpPath(scene,q)
% Derived strictly from q rows, so P always has exactly size(q,1) rows.
P = zeros(size(q,1),3);
for i = 1:size(q,1)
    T = getTransform(scene.robot,q(i,:),scene.tcpBodyName);
    P(i,:) = double(T(1:3,4)).';
end
end

function q = localConcatPaths(parts)
% Splice on the joint level only.  Two things happen per part:
%   1) the whole part is shifted by a per-joint multiple of 2*pi so its first
%      row is the nearest continuous branch to the accumulated end.  This is
%      required because free-space RRT paths and the fixed weld anchors may
%      sit on opposite sides of a 2*pi boundary (a raw 360 deg jump).
%      Only whole turns are added/subtracted, so the physical configuration
%      and every internal step are untouched, and nothing is wrapped to
%      [-pi,pi];
%   2) duplicate endpoints are removed with a 1e-8 rad joint tolerance.
% The Cartesian path is derived afterwards from the resulting rows, so the
% two representations cannot diverge.
q = zeros(0,6);
for i = 1:numel(parts)
    x = double(parts{i});
    if isempty(x), continue; end
    if size(x,2) ~= 6, error('each path part must be N-by-6.'); end
    if isempty(q)
        q = x;
    else
        turns = round((x(1,:)-q(end,:))/(2*pi));
        x = x - turns*(2*pi);
        if max(abs(x(1,:)-q(end,:))) <= 1e-8, x = x(2:end,:); end
        q = [q;x]; %#ok<AGROW>
    end
end
if size(q,1) < 2, error('concatenated path has fewer than two points.'); end
if max(max(abs(rad2deg(atan2(sin(diff(q,1,1)),cos(diff(q,1,1))))))) >= 180
    error('localConcatPaths left a non-continuous junction (>=180 deg jump).');
end
end

function [q,turns] = localCanonicalizeBranch(q)
%LOCALCANONICALIZEBRANCH Shift each joint by one constant turn into [-pi,pi].
%
% A single multiple of 2*pi is applied per joint to the whole column, so:
%   * every adjacent difference is unchanged, hence the interpolated path and
%     all continuity metrics are identical;
%   * the physical configuration is unchanged, because a revolute joint at
%     theta and theta + 2*pi*k is the same pose.
% This is deliberately NOT wrapToPi: no row is ever re-represented on its own,
% which would create artificial full-turn jumps between neighbours.
%
% Joints whose total span exceeds 2*pi cannot fit in the window and are left
% untouched rather than being folded.
q = double(q);
turns = zeros(1,size(q,2));
if isempty(q), return; end
for j = 1:size(q,2)
    col = q(:,j);
    lo = min(col); hi = max(col);
    kmin = ceil((-pi-lo)/(2*pi));
    kmax = floor((pi-hi)/(2*pi));
    if kmin > kmax
        fprintf("[WARN] joint %d spans %.3f rad (>2pi); branch left unchanged.\n", ...
            j,hi-lo);
        continue;
    end
    k = kmin;
    q(:,j) = col + 2*pi*k;
    turns(j) = k;
end
end

function e = localEndpointErrors(q,qStart,qGoal)
e = struct('startRad',NaN,'goalRad',NaN);
if isempty(q)
    error('HMI_WELD_PLAN_FAILED: cannot validate endpoints because qPath is empty.');
end
e.startRad = max(abs(localPeriodic(q(1,:)-qStart)));
e.goalRad = max(abs(localPeriodic(q(end,:)-qGoal)));
end

function d = localPeriodic(x)
d = x - 2*pi*round(x/(2*pi));
end

function c = localContinuity(scene,q,P)
c = struct('pass',true,'maxJointJumpDeg',0,'maxTcpErrorMm',0, ...
    'maxTcpStepMm',0,'maxOrientationChangeDeg',0);
if size(q,1) < 1 || size(P,1) ~= size(q,1), c.pass = false; return; end
for i = 1:size(q,1)
    T = getTransform(scene.robot,q(i,:),scene.tcpBodyName);
    c.maxTcpErrorMm = max(c.maxTcpErrorMm,norm(T(1:3,4)-P(i,:).')*1000);
    if i > 1
        dq = q(i,:)-q(i-1,:);
        c.maxJointJumpDeg = max(c.maxJointJumpDeg,max(abs(rad2deg(dq))));
        c.maxTcpStepMm = max(c.maxTcpStepMm,norm(P(i,:)-P(i-1,:))*1000);
        T0 = getTransform(scene.robot,q(i-1,:),scene.tcpBodyName);
        cc = (trace(T0(1:3,1:3).'*T(1:3,1:3))-1)/2;
        c.maxOrientationChangeDeg = max(c.maxOrientationChangeDeg, ...
            rad2deg(acos(max(-1,min(1,cc)))));
    end
end
% 45 deg matches the validated RRT-Connect edge step used for the free-space
% stages; the weld seam itself normally stays below 15 deg.
c.pass = isfinite(c.maxTcpErrorMm) && c.maxJointJumpDeg <= 45 && c.maxTcpErrorMm <= 1.0;
end

function dense = localDenseValidate(scene,q,resolutionRad)
dense = struct('pass',false,'node_count',size(q,1),'edge_count',max(0,size(q,1)-1), ...
    'node_collision_free',false,'edge_collision_free',false, ...
    'collision_configuration_checks',0,'edge_collision_checks',0, ...
    'collision_nodes',[],'collision_edges',[]);
if isempty(q) || size(q,2) ~= 6 || any(~isfinite(q(:))), return; end
for i = 1:size(q,1)
    [free,~] = collisionFreeJointState(scene.robot,q(i,:),scene.environment, ...
        scene.selfCollisionSkipPairs);
    dense.collision_configuration_checks = dense.collision_configuration_checks+1;
    if ~free, dense.collision_nodes(end+1) = i; return; end %#ok<AGROW>
end
dense.node_collision_free = true;
for i = 1:size(q,1)-1
    % Assign fields individually: struct('f', cellArray) would build a
    % non-scalar struct array and collisionFreeJointEdge rejects it.
    opts = struct();
    opts.wrapJoints = false;
    opts.selfCollisionSkipPairs = scene.selfCollisionSkipPairs;
    [free,~,n,timedOut] = collisionFreeJointEdge(scene.robot,q(i,:),q(i+1,:), ...
        scene.environment,resolutionRad,opts);
    dense.edge_collision_checks = dense.edge_collision_checks+1;
    dense.collision_configuration_checks = dense.collision_configuration_checks+n;
    if timedOut || ~free, dense.collision_edges(end+1) = i; return; end %#ok<AGROW>
end
dense.edge_collision_free = true; dense.pass = true;
end

function anchors = localLoadAnchors(file,matlabDir)
if isfile(file)
    anchors = jsondecode(fileread(file));
else
    source = fullfile(matlabDir,'weld_rrt_connect_approach_exit.mat');
    if ~isfile(source), error('Weld anchor file not found: %s',file); end
    s = load(source,'result');
    anchors.qPreWeld = s.result.preWeld.q;
    anchors.qPostWeld = s.result.postWeld.q;
    anchors.qWeldStart = s.result.qWeld(1,:);
    anchors.qWeldEnd = s.result.qWeld(end,:);
end

anchors.qPreWeld = localQ(anchors.qPreWeld,'qPreWeld');
anchors.qPostWeld = localQ(anchors.qPostWeld,'qPostWeld');
if isfield(anchors,'qWeldStart') && ~isempty(anchors.qWeldStart)
    anchors.qWeldStart = localQ(anchors.qWeldStart,'qWeldStart');
end
end

function d=localAnchorDiagnostics(anchors,scene,anchorFile)
% Check the frozen anchor states against the scene that will actually be
% passed to the RRT.  This is diagnostic only; it never changes a branch.
d=struct('anchor_file',char(string(anchorFile)), ...
    'qPreWeld',struct('q_rad',anchors.qPreWeld,'q_deg',rad2deg(anchors.qPreWeld), ...
        'within_limits',false,'collision_free',false,'valid',false), ...
    'qPostWeld',struct('q_rad',anchors.qPostWeld,'q_deg',rad2deg(anchors.qPostWeld), ...
        'within_limits',false,'collision_free',false,'valid',false), ...
    'branch_policy','continuous radians preserved; no wrapToPi', ...
    'real_workpiece_mismatch',false,'real_workpiece_delta_norm_mm',NaN);
for name={'qPreWeld','qPostWeld'}
    q=anchors.(name{1});
    d.(name{1}).within_limits=jointWithinLimits(q,scene.jointLimits);
    if d.(name{1}).within_limits
        [d.(name{1}).collision_free,detail]=collisionFreeJointState(scene.robot,q,scene.environment,scene.selfCollisionSkipPairs);
        d.(name{1}).collision_detail=struct('environmentCollision',logical(detail.environmentCollision), ...
            'selfCollision',logical(detail.selfCollision));
    else
        d.(name{1}).collision_detail=struct('environmentCollision',false,'selfCollision',false);
    end
    d.(name{1}).valid=logical(d.(name{1}).within_limits && d.(name{1}).collision_free);
end
if isfield(scene,'T_MatBase_Workpiece')
    d.nominal_workpiece_origin_mm=double(scene.T_MatBase_Workpiece(1:3,4).')*1000;
end
realFile=fullfile(fileparts(fileparts(char(string(anchorFile)))),'workpiece_pose_real.json');
% The usual anchor file is UR5/规划文件; the parent of its parent is UR5.
if isfile(realFile)
    try
        r=jsondecode(fileread(realFile));
        if isfield(r,'position') && isfield(d,'nominal_workpiece_origin_mm')
            d.real_workpiece_origin_mm=double(r.position(:).')*1000;
            delta=d.real_workpiece_origin_mm-d.nominal_workpiece_origin_mm;
            d.real_workpiece_delta_mm=delta;
            d.real_workpiece_delta_norm_mm=norm(delta);
            d.real_workpiece_mismatch=d.real_workpiece_delta_norm_mm > 1.0;
            d.real_workpiece_file=realFile;
            d.real_workpiece_valid=logical(localField(r,'valid',false));
        end
    catch err
        d.real_workpiece_read_error=char(string(err.message));
    end
end
% qPreWeld was generated for the frozen scene.  A large measured workpiece
% displacement is evidence that it is stale for the real-calibration scene.
d.qPreWeld_target_status=ternary(d.real_workpiece_mismatch, ...
    'STALE_FOR_REAL_WORKPIECE_CALIBRATION','matches_frozen_scene_only');
end

function p=localSceneProvenance(scene,sceneScript)
sceneScript=char(string(sceneScript));
rootDir=fileparts(fileparts(sceneScript));
frozenFile=fullfile(rootDir,'ur5_tool_calibration.mat');
realTcpFile=fullfile(rootDir,'real_tcp_calibration.json');
appliedTcpFile=fullfile(rootDir,'real_tcp_calibration_applied.json');
realWpFile=fullfile(rootDir,'workpiece_pose_real.json');
p=struct('scene_script',sceneScript,'scene_tcp_body',char(string(scene.tcpBodyName)), ...
    'wire_collision_enabled',logical(localField(scene,'wireCollisionEnabled',false)), ...
    'frozen_tool_calibration_file',frozenFile,'frozen_tool_calibration_used',isfile(frozenFile), ...
    'real_tcp_calibration_file',realTcpFile,'real_tcp_calibration_exists',isfile(realTcpFile), ...
    'real_tcp_calibration_loaded_into_scene',false, ...
    'real_tcp_calibration_applied_file',appliedTcpFile,'real_tcp_applied_record_exists',isfile(appliedTcpFile), ...
    'real_workpiece_calibration_file',realWpFile,'real_workpiece_calibration_exists',isfile(realWpFile), ...
    'real_workpiece_pose_loaded_into_scene',false, ...
    'coordinate_frame_note','planning scene uses frozen MATLAB base/tool calibration; real JSON files are not implicitly applied');
if isfield(scene,'T_tool0_mount'), p.scene_T_tool0_mount=double(scene.T_tool0_mount); end
if isfield(scene,'T_mount_tcp'), p.scene_T_mount_tcp_mm=double(scene.T_mount_tcp(1:3,4).')*1000; end
if isfield(scene,'T_MatBase_Workpiece'), p.scene_workpiece_origin_mm=double(scene.T_MatBase_Workpiece(1:3,4).')*1000; end
if isfile(realTcpFile)
    try
        r=jsondecode(fileread(realTcpFile));
        if isfield(r,'position_flange_tcp_mm'), p.real_tcp_position_flange_mm=double(r.position_flange_tcp_mm(:).'); end
        p.real_tcp_valid=logical(localField(r,'valid',false));
    catch err, p.real_tcp_read_error=char(string(err.message)); end
end
if isfile(realWpFile)
    try
        r=jsondecode(fileread(realWpFile));
        if isfield(r,'position'), p.real_workpiece_origin_mm=double(r.position(:).')*1000; end
        if isfield(p,'scene_workpiece_origin_mm') && isfield(p,'real_workpiece_origin_mm')
            delta=p.real_workpiece_origin_mm-p.scene_workpiece_origin_mm;
            p.workpiece_origin_delta_mm=delta;
            p.workpiece_origin_delta_norm_mm=norm(delta);
            p.workpiece_pose_match=(p.workpiece_origin_delta_norm_mm < 1.0);
        end
        p.real_workpiece_valid=logical(localField(r,'valid',false));
    catch err, p.real_workpiece_read_error=char(string(err.message)); end
end
end

function v=localField(s,name,default)
if isstruct(s) && isfield(s,name) && ~isempty(s.(name)), v=s.(name); else, v=default; end
end

function out=ternary(condition,a,b)
if condition, out=a; else, out=b; end
end

function task = localTask(request)
raw = lower(strrep(strrep(char(string(localGet(request,'task','Weld_Seam'))),'-','_'),' ','_'));
if contains(raw,'full'), task = 'Full_Welding_Task'; return; end
if contains(raw,'realhome') && contains(raw,'preweld'), task = 'RealHome_to_PreWeld'; return; end
if contains(raw,'postweld') && contains(raw,'realhome'), task = 'PostWeld_to_RealHome'; return; end
if contains(raw,'seam') || contains(raw,'weld'), task = 'Weld_Seam'; return; end
error('Unknown weld task: %s',raw);
end

function name = localAlgorithm(request)
name = char(string(localGet(request,'algorithm','V1 FastConnectAnytimeRRTStar')));
end

function seed = localSeed(request)
seed = double(localGet(request,'seed',20260917));
if ~isscalar(seed) || ~isfinite(seed) || seed < 1, error('Invalid seed.'); end
seed = round(seed);
end

function q = localQ(v,label)
q = double(v(:).');
if numel(q) ~= 6 || any(~isfinite(q)), error('%s must be finite 1x6.',label); end
end

function value = localGet(s,name,default)
if isstruct(s) && isfield(s,name) && ~isempty(s.(name))
    value = s.(name);
else
    value = default;
end
end

function path = localPath(s,name,default)
path = char(string(localGet(s,name,default)));
if isempty(path), path = default; end
end

function localEnsureDir(p)
d = fileparts(p);
if ~isempty(d) && ~isfolder(d), mkdir(d); end
end

function txt = localFieldText(s,name,default)
if isstruct(s) && isfield(s,name) && ~isempty(s.(name))
    txt = char(string(s.(name)));
else
    txt = char(string(default));
end
end

function v = localFieldNumeric(s,name,default)
if isstruct(s) && isfield(s,name) && ~isempty(s.(name))
    v = double(s.(name));
else
    v = default;
end
end

function idx = localFurthest(r)
idx = NaN;
if isfield(r,'stateBank')
    idx = find(~cellfun(@isempty,r.stateBank),1,'last');
    if isempty(idx), idx = 0; end
end
end

function info = localEmptySeamInfo()
info = struct('pointCount',0,'executionPointCount',0,'planner','', ...
    'direction','','cascade','','resultFile','');
end

function n = localPathLength(q)
if size(q,1) < 2, n = 0; else, n = sum(sqrt(sum(diff(q,1,1).^2,2))); end
end

function localWriteJson(report,file)
fid = fopen(file,'w');
if fid < 0, error('Cannot write report: %s',file); end
cleanup = onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(report),'char');
end

function localWriteFailureReport(file,extra)
% Preserve the detailed stage report produced by plan_hmi_request, while
% making the wrapper-level failure explicit for the HMI/QProcess caller.
base=struct('success',false,'failure_reason','PLAN_FAILED');
if isfile(file)
    try
        decoded=jsondecode(fileread(file));
        if isstruct(decoded) && isscalar(decoded), base=decoded; end
    catch
        % Keep a clean failure report if the stage was interrupted before a
        % valid JSON document could be written.
    end
end
base.success=false;
if isstruct(extra)
    fields=fieldnames(extra);
    for i=1:numel(fields), base.(fields{i})=extra.(fields{i}); end
end
localWriteJson(base,file);
end

function report=localReadReport(file)
report=struct();
if ~isfile(file), return; end
try
    report=jsondecode(fileread(file));
catch err
    report=struct('read_error',char(string(err.message)),'report_file',file);
end
end
