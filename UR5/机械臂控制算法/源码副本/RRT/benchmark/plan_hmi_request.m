function report = plan_hmi_request(qStart,qGoal,algorithm,seed,options)
%PLAN_HMI_REQUEST Unified MATLAB entry point for HMI offline planning.
% Accepts either (qStart,qGoal,algorithm,seed,options) or one request struct.
if nargin == 1 && isstruct(qStart)
    request = qStart;
    qStart = request.qStart; qGoal = request.qGoal;
    algorithm = request.algorithm;
    if isfield(request,'seed'), seed = request.seed; else, seed = 20260917; end
    options = request;
elseif nargin < 5 || isempty(options)
    options = struct();
end
qStart = localQ(qStart,'qStart'); qGoal = localQ(qGoal,'qGoal');
algorithm = localAlgorithmName(algorithm); seed = double(seed);
if ~isscalar(seed) || ~isfinite(seed) || seed < 1 || seed ~= floor(seed)
    error('seed must be a positive integer.');
end
defaults = struct('sceneScript','/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m', ...
    'outputFile','','reportFile','','requestFile','','requestId','', ...
    'maxPlanningTimeSec',30.0,'maxIterations',1000000, ...
    'plannerResolutionDeg',2.0,'validationResolutionDeg',0.5, ...
    'maxJointStepDeg',12.0,'executionMaxStepDeg',30.0,'goalBias',0.20, ...
    'maxVelocityDegPerSec',5.0,'maxAccelerationDegPerSec2',15.0);
options = localMerge(defaults,options);
if isempty(options.requestId)
    options.requestId=['manual_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))];
end
if isempty(options.outputFile) || isempty(options.reportFile)
    runDir=fullfile('/home/ttbstt/jixiebi/UR5/planning_runs',options.requestId);
    if isempty(options.outputFile), options.outputFile=fullfile(runDir,'trajectory.mat'); end
    if isempty(options.reportFile), options.reportFile=fullfile(runDir,'report.json'); end
end
if ~isempty(options.requestFile) && isfile(options.requestFile)
    request = jsondecode(fileread(options.requestFile));
else
    request = struct('qStart',qStart,'qGoal',qGoal,'algorithm',algorithm, ...
        'seed',seed,'request_id',char(string(options.requestId)));
end
request.qStart=qStart; request.qGoal=qGoal; request.algorithm=algorithm;
request.seed=seed; request.created=char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
if ~isfolder(fileparts(options.outputFile)), mkdir(fileparts(options.outputFile)); end
if ~isfolder(fileparts(options.reportFile)), mkdir(fileparts(options.reportFile)); end

here=fileparts(mfilename('fullpath')); rrtRoot=fileparts(here);
addpath(genpath(rrtRoot)); addpath(fileparts(options.sceneScript));
setenv('RRT_HEADLESS','1'); scene=loadUR5JointScene(options.sceneScript);
% Preflight the exact endpoint states before entering any RRT implementation.
% This makes an invalid Home/PreWeld pair observable and prevents a planner
% failure from being misreported later as an endpoint-validation exception.
inputDiagnostics = localInputDiagnostics(scene,qStart,qGoal);
sceneProvenance = localSceneProvenance(scene,options.sceneScript);
fprintf('HMI_PLAN_START algorithm=%s seed=%d\n',algorithm,seed);
fprintf('HMI_PLAN_PREFLIGHT startLimits=%d startFree=%d goalLimits=%d goalFree=%d\n', ...
    inputDiagnostics.start.within_limits,inputDiagnostics.start.collision_free, ...
    inputDiagnostics.goal.within_limits,inputDiagnostics.goal.collision_free);
if ~inputDiagnostics.valid
    report=localBaseReport(request,algorithm,seed,0,struct(),options);
    report.failure_reason=inputDiagnostics.failure_reason;
    report.input_diagnostics=inputDiagnostics;
    report.scene_provenance=sceneProvenance;
    report.planner=struct('algorithm',algorithm,'success',false, ...
        'iterations',0,'samples',0,'collisionConfigurationChecks',0, ...
        'edgeCollisionChecks',0,'terminationReason','preflight_failed');
    localWriteJson(report,options.reportFile);
    error('HMI_PLAN_FAILED preflight: %s; report=%s', ...
        inputDiagnostics.failure_reason,options.reportFile);
end
cfg=struct('scene',scene,'start',qStart,'goal',qGoal, ...
    'maxIterations',options.maxIterations,'maxJointStepRad',deg2rad(options.maxJointStepDeg), ...
    'jointEdgeResolutionRad',deg2rad(options.plannerResolutionDeg), ...
    'goalBias',options.goalBias,'seed',seed,'maxPlanningTimeSec',options.maxPlanningTimeSec, ...
    'anytime',false,'continueAfterFirstSolution',false,'checkpointTimesSec',zeros(1,0), ...
    'candidateValidatorFcn',[]);
plannerTimer=tic;
try
    switch algorithm
        case 'RRT', planner=RRTJoint(cfg);
        case 'RRT-Connect', planner=RRTConnectJoint(cfg);
        case 'Bi-RRT*', planner=BiRRTStarJoint(cfg);
        case 'V1 FastConnectAnytimeRRTStar'
            cfg.anytime=true; cfg.continueAfterFirstSolution=true;
            cfg.checkpointTimesSec=[5 10 15 20 30];
            planner=FastConnectAnytimeRRTStarJoint(cfg);
        otherwise, error('Unsupported HMI algorithm: %s',algorithm);
    end
catch err
    planningTime=toc(plannerTimer);
    report=localBaseReport(request,algorithm,seed,planningTime,struct(),options);
    report.failure_reason=['planner_exception: ' char(string(err.message))];
    report.exception_identifier=char(string(err.identifier));
    report.input_diagnostics=inputDiagnostics;
    report.scene_provenance=sceneProvenance;
    report.planner=struct('algorithm',algorithm,'success',false, ...
        'iterations',0,'samples',0,'collisionConfigurationChecks',0, ...
        'edgeCollisionChecks',0,'terminationReason','exception');
    localWriteJson(report,options.reportFile);
    error('HMI_PLAN_FAILED planner exception: %s; report=%s', ...
        err.message,options.reportFile);
end
planningTime=toc(plannerTimer);
report=localBaseReport(request,algorithm,seed,planningTime,planner,options);
report.input_diagnostics=inputDiagnostics;
report.scene_provenance=sceneProvenance;
report.planner=localPlannerSummary(planner);
if isfield(planner,'success') && planner.success && ...
        (~isfield(planner,'path') || isempty(planner.path))
    report.success=false;
    report.failure_reason='planner reported success but returned an empty path (interface contract violation)';
    localWriteJson(report,options.reportFile);
    error('HMI_PLAN_FAILED %s; report=%s',report.failure_reason,options.reportFile);
end
if ~isfield(planner,'path') || isempty(planner.path) || ...
        ~isfield(planner,'success') || ~planner.success
    report.success=false;
    report.failure_reason=localPlannerFailureReason(planner,inputDiagnostics);
    localWriteJson(report,options.reportFile);
    error('HMI_PLAN_FAILED algorithm=%s: %s; report=%s', ...
        algorithm,report.failure_reason,options.reportFile);
end

qRaw=double(planner.path); [qLifted,turnCounts]=lift_continuous_joint_path(qRaw);
% Planner paths may contain sparse rewiring vertices.  Insert only linear
% joint-space samples for the HMI execution representation; this does not
% replace dense collision validation and never changes the continuous branch.
qPath=localDensify(qLifted,deg2rad(options.executionMaxStepDeg));
dense=localDenseValidate(scene,qPath,qStart,qGoal,deg2rad(options.validationResolutionDeg));
report.dense_validation=dense; report.collision_verified=true;
report.collision_check_pass=dense.pass;
report.collision_sample_count=dense.collision_configuration_checks;
report.raw_point_count=size(qRaw,1); report.execution_point_count=size(qPath,1);
report.path_length_rad=localPathLength(qPath); report.raw_path_length_rad=localPathLength(qRaw);
report.max_adjacent_step_deg=rad2deg(localMaxStep(qPath));
report.continuous_branch_turn_counts=turnCounts;
report.continuous_branch_lift_applied=any(turnCounts(:)~=0);
if ~dense.pass
    report.success=false; report.failure_reason=localDenseFailureReason(dense);
    localWriteJson(report,options.reportFile);
    error('HMI_PLAN_FAILED %s; report=%s',report.failure_reason,options.reportFile);
end

[timeFromStart,timeDetails]=parameterize_joint_path(qPath, ...
    'MaxVelocityDegPerSec',options.maxVelocityDegPerSec, ...
    'MaxAccelerationDegPerSec2',options.maxAccelerationDegPerSec2);
exportReport=export_hmi_trajectory(qPath,options.outputFile,'CoordinateSpace','ros_real', ...
    'TimeFromStart',timeFromStart,'TrajectoryName',sprintf('Online_%s',algorithm), ...
    'Source','MATLAB_HMI_ONLINE_PLANNER', ...
    'Description','Unified HMI MATLAB planner output; offline only, no automatic execution.', ...
    'CollisionVerified',true,'CollisionCheckPass',true, ...
    'CollisionSampleCount',dense.collision_configuration_checks,'MinimumClearanceM',NaN, ...
    'ReportFile',options.reportFile);
% Keep algorithm identity in the MAT metadata as well as in report.json.
saved=load(options.outputFile,'metadata');
metadata=saved.metadata;
metadata.algorithm=algorithm;
metadata.seed=seed;
metadata.request_id=char(string(localGet(request,'request_id',options.requestId)));
metadata.collision_verified=true;
metadata.collision_check_pass=true;
save(options.outputFile,'metadata','-append');
report=localMerge(exportReport,report);
report.algorithm=algorithm; report.seed=seed; report.success=true;
report.first_solution_time=localFirstSolution(planner);
report.first_solution_time_sec=report.first_solution_time;
report.planning_time=planningTime; report.planning_time_sec=planningTime;
report.path_length_rad=localPathLength(qPath); report.raw_point_count=size(qRaw,1);
report.execution_point_count=size(qPath,1); report.duration=timeFromStart(end);
report.duration_sec=timeFromStart(end); report.time_parameterization=timeDetails;
report.planner=localPlannerSummary(planner); report.output_file=options.outputFile;
report.report_file=options.reportFile;
report.note='Generated by unified plan_hmi_request. No ROS2 command was sent.';
localWriteJson(report,options.reportFile);
fprintf('HMI_PLAN_DONE algorithm=%s success=1 planning=%.3f first=%.3f path=%.6f points=%d collisionPASS=1 duration=%.3f\n', ...
    algorithm,planningTime,report.first_solution_time,report.path_length_rad,size(qPath,1),timeFromStart(end));
end

function name=localAlgorithmName(value)
raw=lower(strtrim(char(string(value))));
if any(strcmp(raw,{'rrt','basic rrt'})), name='RRT'; return; end
if any(strcmp(raw,{'rrt-connect','rrt connect'})), name='RRT-Connect'; return; end
if any(strcmp(raw,{'bi-rrt*','bi-rrt','bi rrt*'})), name='Bi-RRT*'; return; end
if contains(raw,'fastconnect') || contains(raw,'v1'), name='V1 FastConnectAnytimeRRTStar'; return; end
error('Unknown algorithm: %s',value);
end

function q=localQ(value,label)
q=double(value(:).');
if numel(q)~=6 || any(~isfinite(q)), error('%s must be finite 1-by-6 radians.',label); end
end

function report=localBaseReport(request,algorithm,seed,planningTime,planner,options)
report=struct('created',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')), ...
    'request_id',char(string(localGet(request,'request_id',options.requestId))), ...
    'algorithm',algorithm,'seed',seed,'success',false, ...
    'first_solution_time',localFirstSolution(planner),'planning_time',planningTime, ...
    'path_length_rad',NaN,'raw_point_count',0,'execution_point_count',0, ...
    'collision_check_pass',false,'duration',NaN,'request',request);
end

function d=localInputDiagnostics(scene,qStart,qGoal)
% Return auditable endpoint checks without changing branch representation.
d=struct();
d.q_start_rad=qStart; d.q_goal_rad=qGoal;
d.q_start_deg=rad2deg(qStart); d.q_goal_deg=rad2deg(qGoal);
d.joint_limits_rad=double(scene.jointLimits);
d.joint_limits_deg=rad2deg(double(scene.jointLimits));
d.branch_policy='continuous radians preserved; no wrapToPi';
d.start=localEndpointState(scene,qStart);
d.goal=localEndpointState(scene,qGoal);
d.valid=logical(d.start.valid && d.goal.valid);
if d.valid
    d.failure_reason='';
else
    reasons={};
    if ~d.start.within_limits, reasons{end+1}='qStart violates joint limits'; end %#ok<AGROW>
    if d.start.within_limits && ~d.start.collision_free, reasons{end+1}='qStart is in collision'; end %#ok<AGROW>
    if ~d.goal.within_limits, reasons{end+1}='qGoal violates joint limits'; end %#ok<AGROW>
    if d.goal.within_limits && ~d.goal.collision_free, reasons{end+1}='qGoal is in collision'; end %#ok<AGROW>
    d.failure_reason=strjoin(reasons,'; ');
end
end

function s=localEndpointState(scene,q)
s=struct('q_rad',q,'q_deg',rad2deg(q),'within_limits',false, ...
    'collision_free',false,'valid',false, ...
    'collision_detail',struct('environmentCollision',false,'selfCollision',false));
s.within_limits=jointWithinLimits(q,scene.jointLimits);
if s.within_limits
    [s.collision_free,detail]=collisionFreeJointState(scene.robot,q,scene.environment,scene.selfCollisionSkipPairs);
    s.collision_detail=struct('environmentCollision',logical(detail.environmentCollision), ...
        'selfCollision',logical(detail.selfCollision));
end
s.valid=logical(s.within_limits && s.collision_free);
end

function p=localSceneProvenance(scene,sceneScript)
% The current sync script intentionally uses the frozen planning calibration.
% Record that fact and compare it with optional real-calibration artifacts so
% a stale qPreWeld target cannot look like a current real-scene plan.
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
    'coordinate_frame_note','scene is MATLAB base; real calibration JSON declares URDF/base and requires explicit frame application');
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
            p.workpiece_origin_delta_mm=p.real_workpiece_origin_mm-p.scene_workpiece_origin_mm;
            p.workpiece_origin_delta_norm_mm=norm(p.workpiece_origin_delta_mm);
            p.workpiece_pose_match=(p.workpiece_origin_delta_norm_mm < 1e-3);
        end
        p.real_workpiece_valid=logical(localField(r,'valid',false));
    catch err, p.real_workpiece_read_error=char(string(err.message)); end
end
end

function v=localField(s,name,default)
if isstruct(s) && isfield(s,name) && ~isempty(s.(name)), v=s.(name); else, v=default; end
end

function reason=localPlannerFailureReason(planner,inputDiagnostics)
if ~inputDiagnostics.valid, reason=inputDiagnostics.failure_reason; return; end
termination='unknown';
if isfield(planner,'terminationReason'), termination=char(string(planner.terminationReason)); end
iter=localNumeric(planner,{'iterations','stage1Iterations','stage2Iterations'},NaN);
samples=localNumeric(planner,{'samples'},NaN);
cc=localNumeric(planner,{'collisionConfigurationChecks','collisionChecks'},NaN);
ec=localNumeric(planner,{'edgeCollisionChecks','edgeChecks'},NaN);
reason=sprintf('planner returned no path; termination=%s iterations=%s samples=%s collisionConfigurationChecks=%s edgeCollisionChecks=%s', ...
    termination,localNumText(iter),localNumText(samples),localNumText(cc),localNumText(ec));
end

function v=localNumeric(s,names,default)
v=default;
for i=1:numel(names)
    if isstruct(s) && isfield(s,names{i}) && ~isempty(s.(names{i}))
        x=double(s.(names{i}));
        if isscalar(x) && isfinite(x), v=x; return; end
    end
end
end

function t=localNumText(v)
if isfinite(v), t=sprintf('%.0f',v); else, t='NaN'; end
end

function reason=localDenseFailureReason(d)
if ~d.node_collision_free && ~isempty(d.collision_nodes)
    reason=sprintf('dense collision validation failed at node %d',d.collision_nodes(1));
elseif ~d.edge_collision_free && ~isempty(d.collision_edges)
    reason=sprintf('dense collision validation failed at edge %d',d.collision_edges(1));
elseif max(abs(d.endpoint_start_error_rad))>1e-8 || max(abs(d.endpoint_goal_periodic_error_rad))>1e-8
    reason='dense validation endpoint mismatch';
else
    reason='dense collision validation failed';
end
end

function value=localGet(s,name,default)
if isstruct(s) && isfield(s,name), value=s.(name); else, value=default; end
end

function t=localFirstSolution(p)
t=NaN; names={'firstSolutionTimeSec','firstSolutionTime','firstSolutionTimeMs'};
for i=1:numel(names)
    name=names{i};
    if isfield(p,name) && isfinite(double(p.(name)))
        t=double(p.(name)); if endsWith(name,'Ms'), t=t/1000; end; return;
    end
end
end

function n=localPathLength(q)
if size(q,1)<2, n=0; else, n=sum(sqrt(sum(diff(q,1,1).^2,2))); end
end

function x=localMaxStep(q)
if size(q,1)<2, x=0; else, x=max(max(abs(diff(q,1,1)),[],2)); end
end

function qOut=localDensify(qIn,maxStep)
if size(qIn,1)<2, qOut=qIn; return; end
qOut=qIn(1,:);
for i=1:size(qIn,1)-1
    dq=qIn(i+1,:)-qIn(i,:);
    n=max(1,ceil(max(abs(dq))/maxStep));
    alpha=(1:n).'/n;
    qOut=[qOut; qIn(i,:)+alpha.*dq]; %#ok<AGROW>
end
end

function dense=localDenseValidate(scene,qPath,qStart,qGoal,resolutionRad)
dense=struct('pass',false,'node_count',size(qPath,1),'edge_count',max(0,size(qPath,1)-1), ...
    'node_collision_free',false,'edge_collision_free',false, ...
    'collision_configuration_checks',0,'edge_collision_checks',0,'collision_nodes',[], ...
    'collision_edges',[],'endpoint_start_error_rad',NaN(1,6), ...
    'endpoint_goal_periodic_error_rad',NaN(1,6));
if isempty(qPath) || size(qPath,2)~=6 || any(~isfinite(qPath(:))), return; end
dense.endpoint_start_error_rad=qPath(1,:)-qStart;
dense.endpoint_goal_periodic_error_rad=localPeriodicDifference(qPath(end,:),qGoal);
if max(abs(dense.endpoint_start_error_rad))>1e-8 || max(abs(dense.endpoint_goal_periodic_error_rad))>1e-8, return; end
for i=1:size(qPath,1)
    [free,~]=collisionFreeJointState(scene.robot,qPath(i,:),scene.environment,scene.selfCollisionSkipPairs);
    dense.collision_configuration_checks=dense.collision_configuration_checks+1;
    if ~free, dense.collision_nodes(end+1)=i; return; end %#ok<AGROW>
end
dense.node_collision_free=true;
for i=1:size(qPath,1)-1
    opts=struct(); opts.wrapJoints=true;
    opts.selfCollisionSkipPairs=scene.selfCollisionSkipPairs;
    [free,~,count,timedOut]=collisionFreeJointEdge(scene.robot,qPath(i,:),qPath(i+1,:),scene.environment,resolutionRad,opts);
    dense.edge_collision_checks=dense.edge_collision_checks+1;
    dense.collision_configuration_checks=dense.collision_configuration_checks+count;
    if timedOut || ~free, dense.collision_edges(end+1)=i; return; end %#ok<AGROW>
end
dense.edge_collision_free=true; dense.pass=true;
end

function d=localPeriodicDifference(a,b)
d=a-b; d=d-2*pi*round(d/(2*pi));
end

function s=localPlannerSummary(p)
s=struct();
names={'success','planningTimeSec','firstSolutionTimeSec','firstSolutionTime','pathLength','firstPathLength', ...
    'jointPathCostL2','nodeCount','samples','collisionConfigurationChecks','collisionChecks','edgeChecks', ...
    'edgeCollisionChecks','rewireCount','iterations','terminationReason'};
for i=1:numel(names)
    if isfield(p,names{i}), s.(names{i})=p.(names{i}); end
end
end

function out=localMerge(base,in)
out=base; if isempty(in), return; end
names=fieldnames(in);
for i=1:numel(names), out.(names{i})=in.(names{i}); end
end

function localWriteJson(report,fileName)
fid=fopen(fileName,'w');
if fid<0, error('Cannot write report: %s',fileName); end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(report),'char');
end
