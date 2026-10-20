function test_full_audit_equivalence
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT'))); addpath(fullfile(here,'copies'));
req=jsondecode(fileread(fullfile(fileparts(here),'prepare_20261019_152229_336637','request.json')));
raw=load(req.rawFile); ss=load(raw.sceneFile,'scene'); scene=ss.scene;
scene.hmiMotionLimits=hmi_load_robot_motion_limits(req.motion_limits_file,req.motion_limits_sha256);
z=load(fullfile(fileparts(here),'continuous_refactor_20261019','current','parameterization.mat')); c=z.candidate;
base=load(fullfile(here,'current','baseline_trace.mat')); old=base.trace; lim=hmi_scene_motion_limits(scene);
[audit,new]=parallel_audit_copy(c.qPath,c.tcpPosePath,c.timeFromStart,c.velocities,scene,...
 struct('samplesPerSegment',101,'pathTimeReference',c.pathTimeReference,'maxVelocityRadPerSec',lim.velocityLimit,'maxAccelerationRadPerSec2',lim.accelerationLimit));
fields={'t','Q','Qd','Qdd','seg','u','actualP','actualR','targetP','targetR','positionError','orientationError','envFlags','selfFlags','gaps'};
checks=struct(); for k=1:numel(fields), f=fields{k}; checks.(f)=isequaln(old.(f),new.(f)); end
delta=max(abs(old.contourError-new.contourError)); checks.contour=delta<1e-9;
checks.sample_count=isequal(size(old.Q,1),size(new.Q,1)) && size(new.Q,1)==63701;
checks.complete=audit.complete && ~audit.executionReady;
r=struct('pass',all(structfun(@logical,checks)),'checks',checks,'max_contour_difference_mm',delta,...
 'audit',audit,'executionReady',false,'robot_motion','NO');
save(fullfile(here,'full_equivalence.mat'),'r','-v7.3');
fid=fopen(fullfile(here,'full_equivalence.json'),'w'); cleanup=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(r,'PrettyPrint',true),'char');
disp(r); assert(r.pass,'AUDIT:Equivalence','Full coverage equivalence failed');
end
