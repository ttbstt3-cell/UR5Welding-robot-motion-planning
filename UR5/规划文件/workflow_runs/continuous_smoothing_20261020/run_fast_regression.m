function run_fast_regression(caseName,mode)
if nargin<2, mode='serial'; end
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT'))); addpath(fullfile(here,'copies'));
if strcmp(caseName,'current'), name='prepare_20261019_152229_336637'; else, name='prepare_20261019_121401_560570'; end
request=jsondecode(fileread(fullfile(fileparts(here),name,'request.json')));
raw=load(request.rawFile); ss=load(raw.sceneFile,'scene'); scene=ss.scene;
assert(strcmp(hmi_workflow_file_hash(request.rawFile),request.raw_sha256));
assert(strcmp(hmi_workflow_file_hash(raw.sceneFile),raw.metadata.scene_snapshot_sha256));
if isfield(request,'motion_limits_file'), scene.hmiMotionLimits=hmi_load_robot_motion_limits(request.motion_limits_file,request.motion_limits_sha256); end
out=fullfile(here,[caseName '_spline']); z=load(fullfile(out,'candidate.mat')); c=z.candidate; r=z.r;
assert(r.pass,'SMOOTH:Preflight','Preflight failed'); limits=hmi_scene_motion_limits(scene);
if strcmp(mode,'parallel'), auditor=@parallel_audit_copy; else, auditor=@fast_audit_copy; end
[audit,trace]=auditor(c.qPath,c.tcpPosePath,c.timeFromStart,c.velocities,scene,...
 struct('samplesPerSegment',101,'pathTimeReference',c.pathTimeReference,...
 'maxVelocityRadPerSec',limits.velocityLimit,'maxAccelerationRadPerSec2',limits.accelerationLimit));
r.audit=audit; r.full_validation_pass=audit.complete && r.analytic_limits_pass && r.interface_C2_pass;
r.geometry_unchanged=true; r.original47_q_exact=isequal(c.qPath(c.maps{2},:),raw.segments{2}.qPath);
r.original47_tcp_exact=isequal(c.tcpPosePath(c.maps{2}),reshape(raw.segments{2}.tcpPosePath,1,[]));
z=load(fullfile(fileparts(here),'continuous_refactor_20261019',caseName,'parameterization.mat'));
r.all_execution_q_exact=isequal(c.qPath,z.candidate.qPath);
r.all_execution_tcp_exact=isequal(c.tcpPosePath,z.candidate.tcpPosePath);
r.reference_geometry_exact=isequal(c.pathTimeReference.poses,z.candidate.pathTimeReference.poses) &&...
    isequal(c.pathTimeReference.s,z.candidate.pathTimeReference.s);
r.full_validation_pass=r.full_validation_pass && r.all_execution_q_exact && r.all_execution_tcp_exact && r.reference_geometry_exact;
r.audit_mode=mode;
r.executionReady=false;
if strcmp(mode,'parallel'), mat='parallel_audit.mat'; json='parallel_validation.json'; else, mat='audit.mat'; json='full_validation.json'; end
save(fullfile(out,mat),'r','trace','-v7.3');
fid=fopen(fullfile(out,json),'w'); cleanup=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(r,'PrettyPrint',true),'char');
fprintf('SMOOTH_FULL %s pass=%d nodes=%d samples=%d a=%.6f jerk=%.6f sync=%.6f contour=%.6f angle=%.6f audit=%.3f\n',...
 caseName,r.full_validation_pass,size(c.qPath,1),audit.sample_count,r.derivatives.max_acceleration,r.derivatives.max_segment_jerk,...
 audit.max_tcp_time_sync_error_mm,audit.max_tcp_contour_error_mm,audit.max_tcp_orientation_error_deg,audit.performance.total_sec);
end
