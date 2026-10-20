function report=refresh_environment_audit_snapshot(sourceRawFile,previousAuditDir,outDir)
% Rebuild metadata after inventory-only export changes. Preserve full prior
% audit, and compare native colliders before reusing its diagnostic findings.
fresh=audit_real_environment_scene(sourceRawFile,outDir,false);
prior=load(fullfile(previousAuditDir,'environment_audit_scene.mat'),'scene');
current=load(fresh.scene_file,'scene'); a=prior.scene; b=current.scene;
same=numel(a.environment)==numel(b.environment) && ...
    isequal(hmi_environment_names(a),hmi_environment_names(b));
comparisons=cell(1,numel(b.environment));
for k=1:numel(b.environment)
    shapeA=a.environment{k}; shapeB=b.environment{k};
    equal=strcmp(class(shapeA),class(shapeB)) && isequal(shapeA.Pose,shapeB.Pose);
    if isa(shapeB,'collisionBox')
        equal=equal && isequal([shapeA.X shapeA.Y shapeA.Z],[shapeB.X shapeB.Y shapeB.Z]);
    elseif isa(shapeB,'collisionCylinder')
        equal=equal && isequal([shapeA.Radius shapeA.Length],[shapeB.Radius shapeB.Length]);
    elseif isa(shapeB,'collisionSphere')
        equal=equal && shapeA.Radius==shapeB.Radius;
    elseif isa(shapeB,'collisionMesh')
        equal=equal && isequal(shapeA.Vertices,shapeB.Vertices);
    else, equal=false; end
    names=hmi_environment_names(b);
    comparisons{k}=struct('name',names{k},'native_geometry_and_pose_identical',equal);
    same=same && equal;
end
assert(same && fresh.model_comparison.original_models_unchanged,...
    'REAL_ENVIRONMENT:ReauditRequired','Geometry changed; prior full audit cannot be reused');
raw=load(sourceRawFile); raw.qPath=raw.qPath(1,:); raw.timeFromStart=0;
raw.segmentLabel={raw.segmentLabel{1}}; raw.tcpPosePath={raw.tcpPosePath{1}};
raw.sceneFile=fresh.scene_file;
raw.metadata.original_calibration_binding=raw.metadata.calibration_binding;
raw.metadata.calibration_binding=fresh.current_binding;
raw.metadata.scene_snapshot_sha256=hmi_workflow_file_hash(fresh.scene_file);
raw.metadata.source_path_status='STALE'; raw.metadata.executionReady=false;
raw.metadata.offline_reaudit=true;
rawFile=fullfile(outDir,'current_binding_single_frame.mat'); save(rawFile,'-struct','raw','-v7.3');
single=audit_hmi_preview_display(rawFile,outDir);
oldReport=jsondecode(fileread(fullfile(previousAuditDir,'real_environment_scene_regression.json')));
report=struct('scope','Inventory-only binding refresh; original full offline audit retained',...
    'current_environment_sha256',fresh.current_binding.environment_geometry_sha256,...
    'previous_environment_sha256',oldReport.current_binding.environment_geometry_sha256,...
    'native_environment_geometry_and_pose_identical',same,'object_comparisons',{comparisons},...
    'original_models_unchanged',fresh.model_comparison.original_models_unchanged,...
    'previous_full_audit_file',fullfile(previousAuditDir,'preview_display_audit.json'),...
    'previous_full_audit_frame_count',numel(oldReport.path_audit.frames),...
    'previous_full_audit_collision_sample_count',oldReport.path_audit.collision_sample_count,...
    'current_scene_file',fresh.scene_file,...
    'current_snapshot_file',fullfile(outDir,'preview_display_snapshot.json'),...
    'current_single_frame_audit',single,'executionReady',false,'robot_motion','NO');
writeJson(fullfile(outDir,'environment_binding_refresh.json'),report);
end
function writeJson(file,data)
fid=fopen(file,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(data,'PrettyPrint',true),'char');
end
