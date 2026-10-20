function report=hmi_continuous_acceptance(candidate,raw,scene,request,report)
% Read-only final acceptance. It cannot repair/retime nodes or grant execution.
report.final_acceptance_pass=false;
try
clock=tic; limits=hmi_scene_motion_limits(scene); Q=candidate.qPath;
report.stage='ACCEPTANCE_INPUT_VALIDATION';
assert(isfield(candidate,'pathTimeReference'),'CONTINUOUS:Reference','Missing independent path/time reference');
assert(isfield(report,'velocity_optimization') && report.velocity_optimization.pass,...
    'CONTINUOUS:Parameterization','Timing stage did not complete');
assert(max(abs(Q(1,:)-raw.metadata.requested_start_q))<1e-6 &&...
    max(abs(Q(end,:)-raw.metadata.real_home_q))<1e-6,...
    'CONTINUOUS:Endpoints','Task endpoints changed');
homeVersionCheck(scene.realHomeSourceFile,raw.metadata.realhome_version);
report.stage='CONTROLLER_VALIDATION'; fprintf('HMI_WELD_STAGE stage=CONTROLLER_VALIDATION\n');
count=max(5000,request.minControllerSamples); m=max(101,ceil((count-1)/(size(Q,1)-1))+1);
a=validate_accp_controller_interpolation(Q,candidate.tcpPosePath,candidate.timeFromStart,...
    candidate.velocities,scene,struct('samplesPerSegment',m,...
    'pathTimeReference',candidate.pathTimeReference,'maxContourErrorMm',.5,...
    'maxTimeSyncErrorMm',.5,'maxOrientationErrorDeg',.5,...
    'maxVelocityRadPerSec',limits.velocityLimit,'maxAccelerationRadPerSec2',limits.accelerationLimit));
dm=accp_cubic_derivative_metrics(Q,candidate.timeFromStart,candidate.velocities);
lim=scene.jointLimits;
a.analytic_extrema_pass=dm.max_velocity<=limits.velocityLimit && dm.max_acceleration<=limits.accelerationLimit &&...
    all(dm.position_min>=lim(:,1).') && all(dm.position_max<=lim(:,2).');
a.complete=a.complete && a.analytic_extrema_pass;
a.max_joint_velocity_rad_per_sec=dm.max_velocity; a.max_joint_acceleration_rad_per_sec2=dm.max_acceleration;
report.controller_validation=a;
report.jerk_statistics=struct('max',dm.max_segment_jerk,'rms',dm.rms_segment_jerk,...
    'node_acceleration_jump_max',dm.max_acceleration_jump,'scope',dm.jerk_scope);
report.stage_timings.controller_validation_sec=toc(clock);
if ~a.complete, error('CONTINUOUS:Acceptance','Full-task controller acceptance failed: %s',a.failureReason); end
if isfield(report,'motion_limits') && ~isempty(report.motion_limits.normal_tcp_speed_m_s)
    report.tcp_speed_limit_check=hmi_tcp_speed_upper_bound(scene,dm.max_velocity_per_joint);
    report.tcp_speed_limit_check.limit_m_s=report.motion_limits.normal_tcp_speed_m_s;
    report.tcp_speed_limit_check.pass=report.tcp_speed_limit_check.upper_bound_m_s<=report.motion_limits.normal_tcp_speed_m_s;
    assert(report.tcp_speed_limit_check.pass,'CONTINUOUS:TCPSpeed','TCP speed bound exceeded');
end
report.stage='VERSION_VALIDATION'; fprintf('HMI_WELD_STAGE stage=VERSIONS\n'); clock=tic;
assert(strcmp(hmi_workflow_file_hash(request.rawFile),request.raw_sha256),...
    'CONTINUOUS:Stale','Raw preview changed');
assert(strcmp(hmi_workflow_file_hash(raw.sceneFile),raw.metadata.scene_snapshot_sha256),...
    'CONTINUOUS:Stale','Scene snapshot changed');
assert(strcmp(raw.metadata.calibration_binding.planning_scene,'REAL_CALIBRATION'),...
    'CONTINUOUS:Stale','Only REAL scene may complete preparation');
root=fileparts(fileparts(which('build_real_planning_scene')));
currentScene=build_real_planning_scene(false);
assert(max(abs(currentScene.realHomeSourceQ-raw.metadata.real_home_q))<1e-10,...
    'CONTINUOUS:HomeStale','RealHome changed during planning');
homeVersionCheck(currentScene.realHomeSourceFile,raw.metadata.realhome_version);
current=hmi_workflow_binding(currentScene,request.current_binding,root);
report.version_validation=validate_hmi_workflow_versions(raw.metadata.calibration_binding,current,raw.segments,request.execution_gate_reasons);
if isfield(report,'motion_limits')
    assert(strcmp(hmi_workflow_file_hash(request.motion_limits_file),report.motion_limits.profile_sha256),...
        'CONTINUOUS:Stale','Motion limits changed during planning');
end
report.stage_timings.version_validation_sec=toc(clock);
assert(report.version_validation.pass,'CONTINUOUS:Version','Version/readiness mismatch: %s; %s',...
    strjoin(report.version_validation.mismatches,', '),strjoin(cellstr(string(request.execution_gate_reasons)),'; '));
report.current_binding=current;
report.executionReady=false;
report.final_acceptance_pass=true;
catch err
    report.acceptance_failure_reason=err.message;
    report.acceptance_failure_identifier=err.identifier;
    report.acceptance_failure_stack=err.stack;
    report.executionReady=false;
end
end
function homeVersionCheck(file,expected)
text=fileread(file);
version=regexp(text,'(?m)^  realhome_version:[ \t]*(REALHOME-[a-fA-F0-9]+)[ \t]*$','tokens','once');
assert(~isempty(version) && strcmp(version{1},expected),...
    'CONTINUOUS:HomeStale','RealHome version changed during planning');
end
