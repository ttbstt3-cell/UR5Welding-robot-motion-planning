function result=diagnose_hmi_uniform_speed_scaling(trajectoryFile,reportFile,rawFile,outputDir,motionLimitsFile)
% Independent offline 2x diagnostic. NEVER publishes/exports a motion goal.
here=fileparts(mfilename('fullpath')); root=fileparts(here);
addpath(here); addpath(genpath(fullfile(fileparts(root),'RRT')));
assert(~isfolder(outputDir),'Use a new diagnostic directory'); mkdir(outputDir);
timer=tic;
originalHash=hmi_workflow_file_hash(trajectoryFile);
reportHash=hmi_workflow_file_hash(reportFile);
source=load(trajectoryFile); old=jsondecode(fileread(reportFile));
assert(old.continuous_planning_complete && ~old.executionReady && old.controller_validation.complete,...
    'Source is not a completed offline validation');
assert(strcmp(source.metadata.workflow_stage,'PHYSICAL_VALIDATION_PENDING'));
raw=load(rawFile); assert(strcmp(hmi_workflow_file_hash(raw.sceneFile),raw.metadata.scene_snapshot_sha256));
% Preview adds request-only Home/PreWeld keys; compare all actual optimized
% dependency keys, and verify Home separately, never weaken a version check.
for key=fieldnames(source.metadata.calibration_binding).'
    assert(isfield(raw.metadata.calibration_binding,key{1}) &&...
        isequaln(raw.metadata.calibration_binding.(key{1}),source.metadata.calibration_binding.(key{1})),...
        'Raw and optimized dependency differs: %s',key{1});
end
assert(strcmp(raw.metadata.realhome_version,source.metadata.realhome_version));
s=load(raw.sceneFile,'scene'); scene=s.scene;
if nargin>=5
    scene.hmiMotionLimits=hmi_load_robot_motion_limits(motionLimitsFile);
end
motionLimits=hmi_scene_motion_limits(scene);
current=hmi_workflow_binding(scene,source.metadata.calibration_binding,root);
assert(strcmp(current.collision_scene_version,source.metadata.calibration_binding.collision_scene_version),...
    'STALE scene snapshot');
Q=source.qPath; P=source.tcpPosePath; t=source.timeFromStart(:); V=source.velocities;
baseline=accp_cubic_derivative_metrics(Q,t,V);
factor=2; t2=t/factor; V2=V*factor;
candidate=accp_cubic_derivative_metrics(Q,t2,V2);
assert(isequal(Q,source.qPath) && isequal(P,source.tcpPosePath));
fprintf('SPEED_DIAGNOSTIC stage=DENSE_2X_COLLISION nodes=%d\n',size(Q,1));
audit=validate_accp_controller_interpolation(Q,P,t2,V2,scene,struct(...
    'samplesPerSegment',101,'maxContourErrorMm',0.5,'maxTimeSyncErrorMm',0.5,...
    'maxOrientationErrorDeg',0.5,'maxVelocityRadPerSec',motionLimits.velocityLimit,...
    'maxAccelerationRadPerSec2',motionLimits.accelerationLimit));
limit=scene.jointLimits;
exact=candidate.max_velocity<=motionLimits.velocityLimit && candidate.max_acceleration<=motionLimits.accelerationLimit &&...
    all(candidate.position_min>=limit(:,1).') && all(candidate.position_max<=limit(:,2).');
audit.analytic_extrema_pass=exact; audit.complete=audit.complete && exact;
audit.max_joint_velocity_rad_per_sec=candidate.max_velocity;
audit.max_joint_acceleration_rad_per_sec2=candidate.max_acceleration;
lowerBound=sum(max(abs(diff(Q)),[],2))/motionLimits.velocityLimit;
result=struct('scope','OFFLINE_DIAGNOSTIC_ONLY_NOT_A_TRAJECTORY_OR_EXECUTION_PERMIT',...
    'robot_motion','NO','executionReady',false,'speed_factor',factor,...
    'trajectory_file',trajectoryFile,'trajectory_sha256',originalHash,'report_sha256',reportHash,...
    'waypoint_count',size(Q,1),'q_nodes_unchanged',true,'TCP_targets_unchanged',true,...
    'source_binding',source.metadata.calibration_binding,...
    'baseline_duration_sec',t(end),'candidate_duration_sec',t2(end),...
    'baseline_derivatives',baseline,'candidate_derivatives',candidate,...
    'current_limit_velocity_rad_s',motionLimits.velocityLimit,'current_limit_acceleration_rad_s2',motionLimits.accelerationLimit,...
    'fixed_node_velocity_only_duration_lower_bound_sec',lowerBound,...
    'controller_validation',audit,'offline_pass',audit.complete,...
    'uniform_scaling_spatial_invariance','h2*V2=h*V, so ideal Hermite q(u) is identical; collision re-audited here',...
    'physical_servo_validation','NOT_PERFORMED','dynamic_slider_validation','NOT_PERFORMED',...
    'elapsed_sec',toc(timer));
if isfield(scene,'hmiMotionLimits')
    result.motion_limits=scene.hmiMotionLimits;
    assert(strcmp(hmi_workflow_file_hash(motionLimitsFile),result.motion_limits.profile_sha256),'LIMITS:Stale','Limit profile changed');
    result.execution_block_reason='MOTION_LIMITS_PAYLOAD_ACCELERATION_PENDING: tool payload/CoG and application acceleration unverified';
    result.tcp_speed_limit_check=hmi_tcp_speed_upper_bound(scene,candidate.max_velocity_per_joint);
    result.tcp_speed_limit_check.limit_m_s=scene.hmiMotionLimits.normal_tcp_speed_m_s;
    result.tcp_speed_limit_check.pass=result.tcp_speed_limit_check.upper_bound_m_s<=result.tcp_speed_limit_check.limit_m_s;
    result.offline_pass=result.offline_pass && result.tcp_speed_limit_check.pass;
end
assert(strcmp(hmi_workflow_file_hash(trajectoryFile),originalHash));
assert(strcmp(hmi_workflow_file_hash(reportFile),reportHash));
fid=fopen(fullfile(outputDir,'speed_2x_validation.json'),'w'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(result,'PrettyPrint',true),'char');
fprintf('SPEED_DIAGNOSTIC_DONE pass=%d duration=%.6f v=%.9f a=%.9f collision=%d\n',...
    audit.complete,t2(end),candidate.max_velocity,candidate.max_acceleration,audit.collision_pass);
end
