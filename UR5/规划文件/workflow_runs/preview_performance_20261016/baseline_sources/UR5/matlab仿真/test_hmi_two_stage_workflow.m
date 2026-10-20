function report=test_hmi_two_stage_workflow()
% Local adapter regression on the unchanged accepted REAL scene/D path.
here=fileparts(mfilename('fullpath')); root=fileparts(here);
addpath(genpath(fullfile(fileparts(root),'RRT'))); addpath(here);
S=load(fullfile(root,'规划文件','real_anchor_runs','hybrid_velocity_20261014_final','hybrid_velocity_diagnostic.mat'),'r');
B=load(fullfile(root,'规划文件','real_anchor_runs','accp_bridge_comparison_20261013_improved_full_cold_v2.mat'),'result');
r=S.r; scene=B.result.scene;
b=hmi_workflow_binding(scene,r.calibrationBinding,root);
segments={struct('label','weld','source_binding',b)};
v=validate_hmi_workflow_versions(b,b,segments,{}); assert(v.pass);
changed=b; changed.tcp_calibration_version='STALE_TEST';
v=validate_hmi_workflow_versions(b,changed,segments,{}); assert(~v.pass);
missing=rmfield(b,'wire_model_version');
v=validate_hmi_workflow_versions(missing,b,segments,{}); assert(~v.pass);
v=validate_hmi_workflow_versions(b,b,segments,{'OFFLINE_NO_CONTROLLER'}); assert(~v.pass);
% The local transition solver is tested on one actual accepted weld edge.
a=struct('label','approach','qPath',r.qPath(1,:),'tcpPosePath',{r.tcpPosePath(1)});
c=struct('label','weld','qPath',r.qPath(2,:),'tcpPosePath',{r.tcpPosePath(2)});
[bridge,br]=hmi_cartesian_boundary_transition(a,c,scene,struct(...
    'maxBoundaryRefinementDepth',2,'maxBoundaryIKCalls',100,'maxBoundaryTimeSec',30));
assert(br.pass,br.failure_reason);
assert(isequal(bridge.qPath(1,:),r.qPath(1,:)) && isequal(bridge.qPath(end,:),r.qPath(2,:)));
% Algebraic QP feasibility test (stationary fixture, never exported).
Q=repmat(r.qPath(1,:),7,1); t=(0:6)';
[V,vr]=hmi_optimize_boundary_velocities(Q,t,zeros(7,6),3:5,zeros(3,6),[3 5],0);
assert(vr.pass && max(abs(V),[],'all')<1e-6);
% Validate the real 47-node D path with the new >=5000 sampling budget.
opts=struct('samplesPerSegment',110,'maxContourErrorMm',0.5,'maxTimeSyncErrorMm',0.5,...
    'maxOrientationErrorDeg',0.5,'maxVelocityRadPerSec',deg2rad(5),'maxAccelerationRadPerSec2',0.261799);
audit=validate_accp_controller_interpolation(r.qPath,r.tcpPosePath,r.timeFromStart,r.velocities{4},scene,opts);
assert(audit.complete && audit.sample_count>=5000 && audit.collision_pass);
report=struct('robot_motion','NO','executionReady',false,'adapter_tests_pass',true,...
    'version_match_and_stale_tests_pass',true,'local_cartesian_transition',br,...
    'stationary_qp_fixture_pass',vr.pass,'real_D_single_segment_5015_sample_audit',audit,...
    'note','Single-segment runtime test is not a full task acceptance. No execution artifact is exported.');
out=fullfile(root,'规划文件','workflow_runs','gui_regression_20261014','matlab_adapter_tests.json');
fid=fopen(out,'w'); cleanup=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(report,'PrettyPrint',true),'char');
fprintf('HMI_TWO_STAGE_ADAPTER_TEST_PASS wire/version/transition/QP and real D samples=%d\n',audit.sample_count);
end
