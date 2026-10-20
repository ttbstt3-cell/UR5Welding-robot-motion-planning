function b=hmi_workflow_binding(scene,supplied,root)
% Metadata adapter only. Content versions cover derived wire/scene geometry.
b=supplied;
if isfield(scene,'calibrationBinding')
    for f=fieldnames(scene.calibrationBinding).'
        b.(f{1})=scene.calibrationBinding.(f{1});
    end
end
b.urdf_sha256=fileHash(fullfile(root,'real_robot_description_calibrated.urdf'));
b.scene_geometry_sha256=fileHash(fullfile(root,'scene_geometry.json'));
b.environment_geometry_sha256=''; b.environment_geometry_version='';
b.fixed_environment_contacts_sha256='';
if isfield(b,'planning_scene') && strcmp(string(b.planning_scene),"REAL_CALIBRATION")
    b.environment_geometry_sha256=fileHash(fullfile(root,'environment_geometry_real.json'));
    b.fixed_environment_contacts_sha256=fileHash(fullfile(root,'fixed_environment_contacts.json'));
    if isfield(scene,'realEnvironmentGeometryLoad')
        b.environment_geometry_version=scene.realEnvironmentGeometryLoad.version;
    end
end
% A mesh-free historical robot must not share a scene version with the
% correctly loaded collision robot, even when its URDF bytes are identical.
b.robot_collision_coverage='MISSING'; b.robot_collision_geometry_sha256='';
if isfield(scene,'robotBodyCollisionLoad') && isfield(scene,'robot')
    records=scene.robotBodyCollisionLoad.links; descriptor=cell(1,numel(records));
    complete=true;
    for k=1:numel(records)
        record=records{k}; count=numel(getBody(scene.robot,record.geometry_link).Collisions);
        complete=complete && count>0;
        descriptor{k}=sprintf('%s:%d:%s',record.geometry_link,count,strjoin(record.mesh_sha256,','));
    end
    b.robot_collision_geometry_sha256=hashText(strjoin(descriptor,'|'));
    if complete && numel(records)==7, b.robot_collision_coverage='COMPLETE'; end
end
b.wire_collision_shorten_mm=3;
b.wire_model_version=['WIRE-' digest(b,{'real_gun_geometry_sha256','gun_mount_sha256','tcp_calibration_sha256','wire_collision_shorten_mm'})];
sceneKeys={'urdf_sha256','tcp_calibration_sha256','real_gun_geometry_sha256',...
    'gun_mount_sha256','workpiece_sha256','scene_geometry_sha256','wire_model_version','planning_scene',...
    'robot_collision_geometry_sha256','robot_collision_coverage','environment_geometry_sha256'};
if strcmp(string(b.planning_scene),"REAL_CALIBRATION")
    sceneKeys{end+1}='fixed_environment_contacts_sha256';
end
b.collision_scene_version=['SCENE-' digest(b,sceneKeys)];
end
function h=digest(b,keys)
v=cell(size(keys));
for k=1:numel(keys)
    if ~isfield(b,keys{k}) || isempty(b.(keys{k})), v{k}='MISSING';
    elseif isnumeric(b.(keys{k})), v{k}=sprintf('%g',b.(keys{k}));
    else, v{k}=char(string(b.(keys{k}))); end
end
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(uint8(strjoin(v,'|')));
h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
function h=hashText(value)
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(uint8(value));
h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
function h=fileHash(path)
if ~isfile(path), h=''; return; end
fid=fopen(path,'r'); c=onCleanup(@()fclose(fid)); bytes=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(bytes);
h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
