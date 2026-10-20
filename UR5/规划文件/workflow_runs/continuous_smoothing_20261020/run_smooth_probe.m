function run_smooth_probe(caseName,method)
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT'))); addpath(fullfile(here,'copies'));
previous=fullfile(fileparts(here),'continuous_refactor_20261019');
if strcmp(caseName,'current'), name='prepare_20261019_152229_336637'; else, name='prepare_20261019_121401_560570'; end
request=jsondecode(fileread(fullfile(fileparts(here),name,'request.json')));
raw=load(request.rawFile); ss=load(raw.sceneFile,'scene'); scene=ss.scene;
if isfield(request,'motion_limits_file'), scene.hmiMotionLimits=hmi_load_robot_motion_limits(request.motion_limits_file,request.motion_limits_sha256); end
z=load(fullfile(previous,caseName,'geometry.mat')); g=z.g; frozen=g;
[candidate,r]=smooth_parameterize_copy(g,raw,scene,struct('method',method));
assert(isequaln(g,frozen),'SMOOTH:GeometryMutation','Geometry was changed');
out=fullfile(here,[caseName '_' method]); if ~isfolder(out), mkdir(out); end
save(fullfile(out,'candidate.mat'),'candidate','r','-v7.3');
fid=fopen(fullfile(out,'report.json'),'w'); cleanup=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(r,'PrettyPrint',true),'char');
fprintf('SMOOTH_PROBE %s %s pass=%d T=%.3f a=%.9f jerk=%.9f jump=%.9f elapsed=%.3f\n',caseName,method,r.pass,r.duration_sec,r.derivatives.max_acceleration,r.derivatives.max_segment_jerk,r.derivatives.max_acceleration_jump,r.elapsed_sec);
end
