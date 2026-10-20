function report=audit_real_environment_scene(sourceRawFile,outDir,runPathAudit)
% Offline environment-only scene regression. The source trajectory remains
% STALE: copied q nodes are audited, never published as an executable path.
if nargin<3, runPathAudit=true; end
if ~isfolder(outDir), mkdir(outDir); end
here=fileparts(mfilename('fullpath')); root=fileparts(here);
addpath(genpath(fullfile(fileparts(root),'RRT')));
originalRaw=load(sourceRawFile); old=load(originalRaw.sceneFile,'scene'); old=old.scene;
scene=build_real_planning_scene(true); scene=apply_wire_collision_shorten_diagnostic(scene,3);
binding=hmi_workflow_binding(scene,originalRaw.metadata.calibration_binding,root);
oldNames=hmi_environment_names(old); newNames=hmi_environment_names(scene);
report=struct('source_trajectory',sourceRawFile,'source_trajectory_status','STALE',...
    'new_environment_collision_count',numel(scene.environment)-numel(scene.workpieceProxyInfo),...
    'total_environment_collision_count',numel(scene.environment),...
    'environment_names',{newNames},'environment_load',scene.realEnvironmentGeometryLoad,...
    'current_binding',binding,'original_binding',originalRaw.metadata.calibration_binding,...
    'robot_motion','NO','executionReady',false);
report.model_comparison=compareModels(old,scene,oldNames,newNames);
report.current_realhome=validate_hmi_endpoint(scene,scene.qMat0,'CURRENT REALHOME',binding);
report.matlab_zero=validate_hmi_endpoint(scene,zeros(1,6),'MATLAB ZERO',binding);
report.frozen_adapter_zero=validate_hmi_endpoint(scene,deg2rad([180 -90 0 -90 0 0]),'FROZEN ADAPTER ZERO',binding);
report.source_path_start=validate_hmi_endpoint(scene,originalRaw.qPath(1,:),'SOURCE PATH START',binding);
sceneFile=fullfile(outDir,'environment_audit_scene.mat'); save(sceneFile,'scene','-v7.3');
report.scene_file=sceneFile;
if runPathAudit
    raw=originalRaw; raw.sceneFile=sceneFile;
    raw.metadata.original_calibration_binding=originalRaw.metadata.calibration_binding;
    raw.metadata.calibration_binding=binding;
    raw.metadata.scene_snapshot_sha256=hmi_workflow_file_hash(sceneFile);
    raw.metadata.source_path_status='STALE'; raw.metadata.executionReady=false;
    raw.metadata.offline_reaudit=true;
    rawFile=fullfile(outDir,'stale_source_path_audit_input.mat'); save(rawFile,'-struct','raw','-v7.3');
    report.path_audit=audit_hmi_preview_display(rawFile,outDir);
end
writeJson(fullfile(outDir,'real_environment_scene_regression.json'),report);
end

function report=compareModels(old,current,oldNames,newNames)
% Check only the original colliders and kinematic chain. Environment objects
% are intentionally appended; original workpiece indices retain their names.
workpieceCount=numel(old.workpieceProxyInfo); wp=cell(1,workpieceCount);
for k=1:workpieceCount
    a=old.environment{k}; b=current.environment{k};
    wp{k}=struct('name',oldNames{k},'name_unchanged',strcmp(oldNames{k},newNames{k}),...
        'native_class_unchanged',strcmp(class(a),class(b)),...
        'pose_error_fro',norm(a.Pose-b.Pose,'fro'),...
        'dimensions_unchanged',isequal([a.X a.Y a.Z],[b.X b.Y b.Z]));
end
bodyNames=old.robot.BodyNames; sameNames=isequal(bodyNames,current.robot.BodyNames);
counts=cell(1,numel(bodyNames)); fkError=0;
oldQ=reshape(old.qMat0,1,6); currentQ=reshape(current.qMat0,1,6);
samples=[zeros(1,6); oldQ; currentQ; oldQ+linspace(-0.1,0.1,6)];
for k=1:numel(bodyNames)
    counts{k}=struct('name',bodyNames{k},'before',numel(getBody(old.robot,bodyNames{k}).Collisions),...
        'after',numel(getBody(current.robot,bodyNames{k}).Collisions));
    for j=1:size(samples,1)
        fkError=max(fkError,norm(getTransform(old.robot,samples(j,:),bodyNames{k})-...
            getTransform(current.robot,samples(j,:),bodyNames{k}),'fro'));
    end
end
report=struct('body_names_unchanged',sameNames,'body_collision_counts',{counts},...
    'qOffset_unchanged',isequal(old.qOffset,current.qOffset),...
    'joint_limits_unchanged',isequal(old.jointLimits,current.jointLimits),...
    'self_collision_skip_pairs_unchanged',isequal(old.selfCollisionSkipPairs,current.selfCollisionSkipPairs),...
    'gun_proxy_geometry_unchanged',isequal(old.gunProxyInfo,current.gunProxyInfo),...
    'gun_mount_transform_unchanged',isequal(old.T_mount_source,current.T_mount_source),...
    'wire_length_unchanged',old.wireLengthM==current.wireLengthM,...
    'wire_diameter_unchanged',old.wireDiameterM==current.wireDiameterM,...
    'workpiece_original_colliders',{wp},'max_body_FK_matrix_error',fkError,...
    'source_scene_before',old.calibrationBinding,'source_scene_after',current.calibrationBinding);
report.original_models_unchanged=sameNames && fkError<1e-12 && ...
    all(cellfun(@(v)v.before==v.after,counts)) && ...
    report.qOffset_unchanged && report.joint_limits_unchanged && ...
    report.self_collision_skip_pairs_unchanged && report.gun_proxy_geometry_unchanged && ...
    report.gun_mount_transform_unchanged && report.wire_length_unchanged && report.wire_diameter_unchanged && ...
    all(cellfun(@(v)v.name_unchanged && v.native_class_unchanged && v.dimensions_unchanged && v.pose_error_fro<1e-12,wp));
end
function writeJson(file,data)
fid=fopen(file,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(data,'PrettyPrint',true),'char');
end
