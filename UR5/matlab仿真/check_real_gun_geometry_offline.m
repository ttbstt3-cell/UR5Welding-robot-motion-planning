function report=check_real_gun_geometry_offline(requestFile,outputFile)
%CHECK_REAL_GUN_GEOMETRY_OFFLINE Numerical review, NOT a planning approval.
% Uses calibrated URDF, unchanged real TCP, source-frame V-HACD components.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(fileparts(root),'RRT')));
req=jsondecode(fileread(requestFile));
m=jsondecode(fileread(fullfile(root,'real_gun_geometry','real_gun_geometry.json')));
robot=importrobot(fullfile(root,'real_robot_description_calibrated.urdf'), ...
    'MeshPath','/opt/ros/humble/share');
robot.DataFormat='row';
cal=load(fullfile(root,'ur5_tool_calibration.mat'));
mount=rigidBody('WeldingGunMount'); j=rigidBodyJoint('GunMount_fixed','fixed');
setFixedTransform(j,cal.T_tool0_mount); mount.Joint=j;
addBody(robot,mount,'tool0');
tcp=rigidBody('WeldTCP'); jt=rigidBodyJoint('WeldTCP_fixed','fixed');
T_flange_mount=getTransform(robot,zeros(1,6),'WeldingGunMount','flange');
tcpFile=fullfile(root,'real_tcp_calibration_active.json');
if ~isfile(tcpFile), tcpFile=fullfile(root,'real_tcp_calibration_applied.json'); end
Tc=load_real_tcp_calibration(tcpFile);
if ~isfield(Tc,'applied_to_planning_model') || ~Tc.applied_to_planning_model
    error('REAL_GUN:TCP','Only the existing explicitly applied TCP version may be used.');
end
if ~Tc.orientationApplied
    Tnom=T_flange_mount*cal.T_mount_tcp;
    Tc.T_flange_tcp(1:3,1:3)=Tnom(1:3,1:3);
end
setFixedTransform(jt,T_flange_mount\Tc.T_flange_tcp); tcp.Joint=jt;
addBody(robot,tcp,'WeldingGunMount');
scene=struct('robot',robot);
scene=apply_real_gun_geometry(scene,double(req.T_flange_source),true);
q=double(req.q_rad); n=size(q,1);
assert(n==10 && size(q,2)==6 && all(isfinite(q(:))),'Expected exactly 10 finite joint rows');
components=m.collision.components;
samples=repmat(struct('q_rad',[],'T_base_flange',[], ...
    'T_base_source',[],'component_centers_base_m',[],'relative_transform_error',0),n,1);
for k=1:n
    Tf=getTransform(scene.robot,q(k,:),'flange','base');
    Ts=Tf*scene.T_flange_source;
    Tm=getTransform(scene.robot,q(k,:),'WeldingGunMount','base');
    relErr=norm(Tf\(Tm*scene.T_mount_source)-scene.T_flange_source,'fro');
    centers=zeros(numel(components),3);
    for c=1:numel(components)
        p=mean(double(components(c).vertices_m),1);
        h=Ts*[p(:);1]; centers(c,:)=h(1:3)';
    end
    samples(k).q_rad=q(k,:); samples(k).T_base_flange=Tf;
    samples(k).T_base_source=Ts;
    samples(k).component_centers_base_m=centers;
    samples(k).relative_transform_error=relErr;
end
% The actual saved placement may be DRAFT; evaluate it as a diagnostic only.
wp=jsondecode(fileread(fullfile(root,'workpiece_pose_real.json')));
assert(strcmp(wp.frame_id,'base'),'Expected URDF base workpiece frame');
Tbasewp=eye(4); Tbasewp(1:3,1:3)=wp.rotation_matrix; Tbasewp(1:3,4)=wp.position;
Trootbase=getTransform(scene.robot,zeros(1,6),'base');
g=jsondecode(fileread(fullfile(root,'scene_geometry.json')));
world=cell(1,numel(g.workpiece.collision_proxies));
for k=1:numel(world)
    p=g.workpiece.collision_proxies(k);
    box=collisionBox(p.size_m(1),p.size_m(2),p.size_m(3));
    quat=reshape(p.pose_workpiece.quaternion,1,4); Tl=quat2tform([quat(4) quat(1:3)]);
    Tl(1:3,4)=p.pose_workpiece.position;
    box.Pose=Trootbase*Tbasewp*Tl; world{k}=box;
end
% Use original attachment skip names that actually exist, no new skips.
skip=deriveStructuralSelfCollisionSkips(scene.robot);
skip=skip(all(ismember(skip,scene.robot.BodyNames),2),:);
home=reshape(double(req.qHome),1,6);
[self,~,~]=checkCollision(scene.robot,home,'IgnoreSelfCollision','off', ...
    'Exhaustive','on','SkippedSelfCollisions',skip);
[env,~,~]=checkCollision(scene.robot,home,world,'IgnoreSelfCollision','on','Exhaustive','on');
wireStatus='UNAVAILABLE';
if isfield(scene,'wireStatus'), wireStatus=char(string(scene.wireStatus)); end
report=struct('status','PREVIEW_ONLY_ALIGNMENT_REQUIRED','samples',samples, ...
    'component_count',numel(components),'wire_status',wireStatus, ...
    'saved_home_q_rad',home,'self_collision',any(self(:)), ...
    'environment_collision',any(env(:)),'self_raw',self,'environment_raw',env, ...
    'body_names',{scene.robot.BodyNames},'workpiece_review_status',wp.review_status, ...
    'T_flange_source',scene.T_flange_source,'TCP_unchanged',true,'TCP_source',tcpFile, ...
    'collision_acceptance','NOT_ACCEPTED_UNTIL_PHYSICAL_ALIGNMENT_CONFIRMATION');
fid=fopen(outputFile,'w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(report),'char');
fprintf('REAL_GUN_REVIEW components=%d self=%d environment=%d previewOnly=1\n',numel(components),any(self(:)),any(env(:)));
end
