function report=test_replacement_gun_geometry(requestFile,outputFile)
%TEST_REPLACEMENT_GUN_GEOMETRY Gun-only offline CAD/V-HACD review.
% No planning authorization, ROS connection, or physical wire assumption.
root=fileparts(fileparts(mfilename('fullpath')));
req=jsondecode(fileread(requestFile));
m=jsondecode(fileread(fullfile(root,'real_gun_geometry','real_gun_geometry.json')));
assert(strcmp(m.source.sha256,req.source_sha256),'Geometry source mismatch');
robot=importrobot(fullfile(root,'real_robot_description_calibrated.urdf'),'MeshPath','/opt/ros/humble/share');
robot.DataFormat='row';
cal=load(fullfile(root,'ur5_tool_calibration.mat'));
mount=rigidBody('WeldingGunMount'); joint=rigidBodyJoint('GunMount_fixed','fixed');
setFixedTransform(joint,cal.T_tool0_mount); mount.Joint=joint; addBody(robot,mount,'tool0');
T=double(req.T_flange_source);
Tfm=getTransform(robot,zeros(1,6),'WeldingGunMount','flange');
Tms=Tfm\T;
mount=getBody(robot,'WeldingGunMount');
components=m.collision.components;
for k=1:numel(components)
    v=double(components(k).vertices_m);
    assert(size(v,2)==3 && all(isfinite(v(:))) && rank(v-v(1,:))==3,'Invalid convex component');
    addCollision(mount,collisionMesh(v),Tms);
end
replaceBody(robot,'WeldingGunMount',mount);
q=double(req.q_rad); assert(isequal(size(q),[10,6]));
positionErrors=zeros(10,numel(components)); transformErrors=zeros(10,1);
for i=1:10
    Tf=getTransform(robot,q(i,:),'flange','base');
    Tm=getTransform(robot,q(i,:),'WeldingGunMount','base');
    Ts=Tm*Tms;
    expected=double(req.hmi_samples(i).T_base_source);
    transformErrors(i)=max(abs(Ts(:)-expected(:)));
    for k=1:numel(components)
        p=Ts*[mean(double(components(k).vertices_m),1)';1];
        positionErrors(i,k)=norm(p(1:3)-double(req.hmi_samples(i).component_centers_base_m(k,:))')*1000;
    end
    assert(norm(Tf\Ts-T,'fro')<1e-10,'Mount must remain fixed');
end
% Check gun-only Saved RealHome against unchanged robot and real workpiece.
addpath(genpath(fullfile(fileparts(root),'RRT')));
skip=deriveStructuralSelfCollisionSkips(robot);
skip=skip(all(ismember(skip,robot.BodyNames),2),:);
home=reshape(double(req.qHome),1,6);
[self,~,~]=checkCollision(robot,home,'IgnoreSelfCollision','off','Exhaustive','on','SkippedSelfCollisions',skip);
wp=jsondecode(fileread(fullfile(root,'workpiece_pose_real.json')));
Twp=eye(4); Twp(1:3,1:3)=double(wp.rotation_matrix); Twp(1:3,4)=double(wp.position(:));
g=jsondecode(fileread(fullfile(root,'scene_geometry.json')));
world=cell(1,numel(g.workpiece.collision_proxies));
Tr=getTransform(robot,zeros(1,6),'base');
for k=1:numel(world)
    p=g.workpiece.collision_proxies(k);
    box=collisionBox(p.size_m(1),p.size_m(2),p.size_m(3));
    quat=reshape(p.pose_workpiece.quaternion,1,4);
    Tl=quat2tform([quat(4),quat(1:3)]); Tl(1:3,4)=p.pose_workpiece.position(:);
    box.Pose=Tr*Twp*Tl; world{k}=box;
end
[env,~,~]=checkCollision(robot,home,world,'IgnoreSelfCollision','on','Exhaustive','on');
report=struct('status','OFFLINE_GUN_ONLY_REVIEW','robot_motion','NO','pose_count',10, ...
    'component_count',numel(components),'max_component_position_error_mm',max(positionErrors(:)), ...
    'max_transform_error',max(transformErrors),'consistency_pass',max(positionErrors(:))<1e-6, ...
    'saved_home_self_collision',any(self(:)),'saved_home_environment_collision',any(env(:)), ...
    'self_raw',self,'environment_raw',env,'body_names',{robot.BodyNames}, ...
    'wire_included',false,'wire_reason','Changed CAD source, old wire reference not transferred', ...
    'installation_confirmed',false,'execution_authorized',false);
fid=fopen(outputFile,'w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(report),'char');
fprintf('NEW_GUN poses=10 components=%d maxCenterMm=%.12g self=%d environment=%d wire=UNVERIFIED\n', ...
    numel(components),max(positionErrors(:)),any(self(:)),any(env(:)));
end
