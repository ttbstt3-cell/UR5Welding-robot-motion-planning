function report=verify_fixed_base_floor_contact(scene,outFile)
% Certify only the model's fixed base support contact, not physical installation.
% The 0.01 mm classification band is NOT a planner collision tolerance.
names=hmi_environment_names(scene); idx=find(strcmp(names,'EnvironmentCollision_Floor'));
assert(isscalar(idx),'FIXED_SUPPORT:Floor','Expected one explicitly named floor');
body=getBody(scene.robot,'base_link_inertia');
chain=struct('body',{},'joint',{},'type',{}); b=body;
while ~isempty(b)
    if strcmp(b.Name,scene.robot.BaseName), break; end
    chain(end+1)=struct('body',b.Name,'joint',b.Joint.Name,'type',b.Joint.Type); %#ok<AGROW>
    b=b.Parent;
end
allFixed=all(strcmp({chain.type},'fixed'));
samples=[zeros(1,6);reshape(scene.qMat0,1,6);sin((1:8)'*(1:6))*0.8];
T0=getTransform(scene.robot,samples(1,:),'base_link_inertia'); drift=0;
for k=1:size(samples,1)
    drift=max(drift,norm(getTransform(scene.robot,samples(k,:),'base_link_inertia')-T0,'fro'));
end
floor=scene.environment{idx}; assert(isa(floor,'collisionBox'));
V=zeros(0,3);
records=scene.robotBodyCollisionLoad.links;
record=records{find(cellfun(@(x)strcmp(x.geometry_link,'base_link_inertia'),records),1)};
for k=1:numel(record.mesh_files)
    assert(strcmp(hmi_workflow_file_hash(record.mesh_files{k}),record.mesh_sha256{k}),...
        'FIXED_SUPPORT:STALE','Base collision STL changed');
    mesh=stlread(record.mesh_files{k}); points=double(mesh.Points);
    W=T0*record.collision_origins{k}*[points ones(size(points,1),1)].';
    F=floor.Pose\W; V=[V;F(1:3,:).']; %#ok<AGROW>
end
assert(~isempty(V));
gap=min(V(:,3))-floor.Z/2;
inside=all(abs(V(:,1))<floor.X/2) && all(abs(V(:,2))<floor.Y/2);
upright=norm(floor.Pose(1:3,3)-[0;0;1])<1e-9;
bandM=0.01/1000;
report=struct('status','NOT_VERIFIED','eligible',false,...
    'pair',{{'base_link_inertia','EnvironmentCollision_Floor'}},...
    'fixed_joint_chain',chain,'all_joints_to_root_fixed',allFixed,...
    'sample_count',size(samples,1),'max_base_transform_drift',drift,...
    'base_inside_floor_footprint',inside,'floor_horizontal',upright,...
    'base_lowest_z_in_floor_frame_m',min(V(:,3)),...
    'floor_top_z_in_floor_frame_m',floor.Z/2,...
    'vertical_gap_mm',gap*1000,'vertical_bounds_overlap_mm',max(0,-gap)*1000,...
    'classification_band_mm',bandM*1000,...
    'T_root_base_body',T0,'floor_pose',floor.Pose,'floor_size_m',[floor.X floor.Y floor.Z],...
    'robot_collision_counts',cellfun(@(n)numel(getBody(scene.robot,n).Collisions),scene.robot.BodyNames),...
    'physical_installation_independently_measured',false,...
    'scope','Model evidence of normal fixed support only; overlap is bounds-based, not penetration depth',...
    'executionReady',false,'robot_motion','NO');
report.eligible=allFixed && drift<1e-12 && inside && upright && abs(gap)<bandM;
if report.eligible, report.status='MODEL_VERIFIED_FIXED_SUPPORT'; end
if nargin>1 && ~isempty(outFile)
    fid=fopen(outFile,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fwrite(fid,jsonencode(report,'PrettyPrint',true),'char');
end
end
