function run_independent(caseName,doAudit)
if nargin<2, doAudit=true; end
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT'))); addpath(fullfile(here,'copy'));
if strcmp(caseName,'current'), source=fullfile(fileparts(here),'prepare_20261019_152229_336637','request.json');
else, source=fullfile(fileparts(here),'prepare_20261019_121401_560570','request.json'); end
request=jsondecode(fileread(source)); raw=load(request.rawFile); ss=load(raw.sceneFile,'scene'); scene=ss.scene;
assert(strcmp(hmi_workflow_file_hash(request.rawFile),request.raw_sha256));
assert(strcmp(hmi_workflow_file_hash(raw.sceneFile),raw.metadata.scene_snapshot_sha256));
if isfield(request,'motion_limits_file'), scene.hmiMotionLimits=hmi_load_robot_motion_limits(request.motion_limits_file,request.motion_limits_sha256); end
out=fullfile(here,caseName); if ~isfolder(out), mkdir(out); end
r=struct('success',false,'executionReady',false,'robot_motion','NO','caseName',caseName,'stages',struct());
clock=tic;
try
    [g,gr]=cp_geometry(raw,scene); r.stages.geometry=gr; save(fullfile(out,'geometry.mat'),'g','gr','-v7.3');
    assert(gr.pass,'CP:Geometry','%s',gr.failure_reason);
    [candidate,tr]=cp_parameterize(g,raw,scene); r.stages.parameterization=tr;
    save(fullfile(out,'parameterization.mat'),'candidate','tr','-v7.3');
    assert(tr.pass,'CP:Parameterization','%s',tr.failure_reason);
    if doAudit
        aclock=tic; lim=hmi_scene_motion_limits(scene);
        audit=cp_validate_copy(candidate.qPath,candidate.tcpPosePath,candidate.timeFromStart,candidate.velocities,scene,...
            struct('samplesPerSegment',101,'maxVelocityRadPerSec',lim.velocityLimit,...
            'maxAccelerationRadPerSec2',lim.accelerationLimit,'pathTimeReference',candidate.pathTimeReference));
        r.stages.acceptance=struct('elapsed_sec',toc(aclock),'audit',audit);
        current=hmi_workflow_binding(scene,request.current_binding,root);
        r.versions=validate_hmi_workflow_versions(raw.metadata.calibration_binding,current,raw.segments,request.execution_gate_reasons);
        r.success=audit.complete && r.versions.pass && tr.interface_C2_pass && tr.analytic_limits_pass;
    end
catch err
    r.failure_reason=err.message; r.failure_identifier=err.identifier; r.stack=err.stack;
end
r.elapsed_sec=toc(clock); save(fullfile(out,'independent_result.mat'),'r','-v7.3');
fid=fopen(fullfile(out,'report.json'),'w'); c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(r,'PrettyPrint',true),'char');
fprintf('INDEPENDENT_RESULT %s pass=%d time=%.3f executionReady=0\n',caseName,r.success,r.elapsed_sec);
end
