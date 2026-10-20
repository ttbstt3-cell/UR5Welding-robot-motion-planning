function run_diagnosis(caseName,fullAudit)
if nargin<2, fullAudit=true; end
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT'))); addpath(fullfile(here,'copies'));
previous=fullfile(fileparts(here),'continuous_refactor_20261019');
if strcmp(caseName,'current'), name='prepare_20261019_152229_336637'; else, name='prepare_20261019_121401_560570'; end
request=jsondecode(fileread(fullfile(fileparts(here),name,'request.json')));
raw=load(request.rawFile); ss=load(raw.sceneFile,'scene'); scene=ss.scene;
assert(strcmp(hmi_workflow_file_hash(request.rawFile),request.raw_sha256));
assert(strcmp(hmi_workflow_file_hash(raw.sceneFile),raw.metadata.scene_snapshot_sha256));
if isfield(request,'motion_limits_file'), scene.hmiMotionLimits=hmi_load_robot_motion_limits(request.motion_limits_file,request.motion_limits_sha256); end
z=load(fullfile(previous,caseName,'parameterization.mat')); c=z.candidate; dm=accp_cubic_derivative_metrics(c.qPath,c.timeFromStart,c.velocities);
out=fullfile(here,caseName); if ~isfolder(out), mkdir(out); end
[~,order]=sort(max(abs(dm.segment_jerk),[],2),'descend'); peaks=cell(1,10);
for k=1:10
 e=order(k); [~,j]=max(abs(dm.segment_jerk(e,:))); a=c.qPath(e,:); b=c.qPath(e+1,:); h=c.timeFromStart(e+1)-c.timeFromStart(e);
 midpoint=(a+b)/2+h*(c.velocities(e,:)-c.velocities(e+1,:))/8; T=getTransform(scene.robot,midpoint,scene.tcpBodyName);
 peaks{k}=struct('edge',e,'joint',j,'segment',c.segmentLabel{e+1},'time_interval',c.timeFromStart(e:e+1).',...
  'duration_sec',h,'q0_rad',a,'q1_rad',b,'delta_q_rad',b-a,'v0',c.velocities(e,:),'v1',c.velocities(e+1,:),...
  'a_start',dm.acceleration_start(e,:),'a_end',dm.acceleration_end(e,:),'jerk',dm.segment_jerk(e,:),...
  'neighbor_acceleration_jumps',dm.node_acceleration_jump(max(1,e-1):min(size(dm.node_acceleration_jump,1),e+1),:),...
  'midpoint_TCP',T,'is_task_join',any(ismember([e e+1],c.joins)));
end
r=struct('caseName',caseName,'executionReady',false,'peaks',{peaks},'derivatives',dm,'duration',c.timeFromStart(end));
if fullAudit
 limits=hmi_scene_motion_limits(scene);
 [audit,trace]=baseline_audit_profile(c.qPath,c.tcpPosePath,c.timeFromStart,c.velocities,scene,...
  struct('samplesPerSegment',101,'pathTimeReference',c.pathTimeReference,'maxVelocityRadPerSec',limits.velocityLimit,'maxAccelerationRadPerSec2',limits.accelerationLimit));
 r.audit=audit; save(fullfile(out,'baseline_trace.mat'),'trace','-v7.3');
end
save(fullfile(out,'diagnosis.mat'),'r','-v7.3');
fid=fopen(fullfile(out,'diagnosis.json'),'w'); cleanup=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(r,'PrettyPrint',true),'char');
fprintf('DIAGNOSIS %s jerk=%.9f acceleration=%.9f\n',caseName,dm.max_segment_jerk,dm.max_acceleration);
end
