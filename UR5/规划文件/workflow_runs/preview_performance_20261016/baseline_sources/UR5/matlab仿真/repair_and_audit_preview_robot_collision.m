function report=repair_and_audit_preview_robot_collision(rawFile,outputDir)
% Repair an isolated preview-scene copy using only calibrated URDF body meshes,
% then re-run the existing trajectory audit. Does not plan or control a robot.
if nargin<1 || isempty(rawFile)
    rawFile='/home/ttbstt/jixiebi/UR5/规划文件/workflow_runs/preview_20261014_121436_257478/full_raw_trajectory.mat';
end
if nargin<2 || isempty(outputDir)
    outputDir='/home/ttbstt/jixiebi/UR5/规划文件/workflow_runs/robot_collision_repair_preview_20261014';
end
here=fileparts(mfilename('fullpath')); root=fileparts(here); addpath(here);
if ~isfolder(outputDir), mkdir(outputDir); end
original=load(rawFile); sourceSceneFile=original.sceneFile;
assert(strcmp(hmi_workflow_file_hash(sourceSceneFile),original.metadata.scene_snapshot_sha256),...
    'PREVIEW:SnapshotChanged','Source scene snapshot does not match original trajectory binding');
sourceScene=load(sourceSceneFile,'scene'); scene=sourceScene.scene;
originalSceneHash=hmi_workflow_file_hash(sourceSceneFile);
originalCounts=bodyCollisionCounts(scene.robot);
originalToolCounts=toolCollisionCounts(scene.robot,scene);
originalEnvironmentPoses=environmentPoses(scene.environment);
originalGeometry=struct('T_flange_source',scene.T_flange_source,'T_mount_source',scene.T_mount_source,...
    'T_mount_wire',scene.T_mount_wire,'T_MatBase_Workpiece',scene.T_MatBase_Workpiece,...
    'wireLengthM',scene.wireLengthM,'wireCollisionLengthM',scene.wireCollisionLengthM,...
    'wireDiameterM',scene.wireDiameterM,'gunProxyInfo',scene.gunProxyInfo,...
    'workpieceProxyInfo',scene.workpieceProxyInfo,'selfCollisionSkipPairs',scene.selfCollisionSkipPairs);
originalRobot=scene.robot; originalBodyNames=[originalRobot.BodyNames {originalRobot.BaseName}];
originalFk=cell(10,numel(originalBodyNames)); pick=unique(round(linspace(1,size(original.qPath,1),10)));
for qi=1:numel(pick)
    q=original.qPath(pick(qi),:);
    for bi=1:numel(originalBodyNames), originalFk{qi,bi}=getTransform(originalRobot,q,originalBodyNames{bi}); end
end
urdfFile=fullfile(root,'real_robot_description_calibrated.urdf');
[scene.robot,scene.robotBodyCollisionLoad]=load_ur5_body_collision_geometry(scene.robot,urdfFile);
afterCounts=bodyCollisionCounts(scene.robot); afterToolCounts=toolCollisionCounts(scene.robot,scene);
assert(all([afterCounts.collision_count]>0),'PREVIEW:Coverage','UR5 collision geometry remains incomplete');
assert(isequal(originalToolCounts,afterToolCounts),'PREVIEW:ToolChanged','Gun/TCP/wire collision geometry changed');
assert(isequal(originalEnvironmentPoses,environmentPoses(scene.environment)),...
    'PREVIEW:EnvironmentChanged','Workpiece collision poses changed');
assert(isequaln(originalGeometry,struct('T_flange_source',scene.T_flange_source,...
    'T_mount_source',scene.T_mount_source,'T_mount_wire',scene.T_mount_wire,...
    'T_MatBase_Workpiece',scene.T_MatBase_Workpiece,'wireLengthM',scene.wireLengthM,...
    'wireCollisionLengthM',scene.wireCollisionLengthM,'wireDiameterM',scene.wireDiameterM,...
    'gunProxyInfo',scene.gunProxyInfo,'workpieceProxyInfo',scene.workpieceProxyInfo,...
    'selfCollisionSkipPairs',scene.selfCollisionSkipPairs)),...
    'PREVIEW:SceneChanged','Tool/workpiece transforms, collision definitions or skip pairs changed');
maxFkDelta=0;
for qi=1:numel(pick)
    q=original.qPath(pick(qi),:);
    for bi=1:numel(originalBodyNames)
        now=getTransform(scene.robot,q,originalBodyNames{bi});
        maxFkDelta=max(maxFkDelta,norm(originalFk{qi,bi}-now,'fro'));
    end
end
assert(maxFkDelta<1e-12,'PREVIEW:FKChanged','Robot forward kinematics changed during collision-only repair');

repairedSceneFile=fullfile(outputDir,'raw_scene_robot_collision_repaired.mat');
copyfile(sourceSceneFile,repairedSceneFile);
save(repairedSceneFile,'scene','-append');
rawOut=fullfile(outputDir,'raw_trajectory_robot_collision_repaired.mat'); copyfile(rawFile,rawOut);
trajectory=load(rawOut); trajectory.sceneFile=repairedSceneFile;
trajectory.metadata.scene_snapshot_sha256=hmi_workflow_file_hash(repairedSceneFile);
trajectory.metadata.preview_only=true; trajectory.metadata.executionReady=false;
trajectory.metadata.collision_audit_revision='URDF_ROBOT_COLLISION_MESHES_LOADED';
trajectory.metadata.source_scene_sha256=originalSceneHash;
trajectory.metadata.source_trajectory_sha256=hmi_workflow_file_hash(rawFile);
trajectory.metadata.note='Collision-only diagnostic scene copy; original preview and scene remain unchanged; never execution-ready.';
sceneFile=trajectory.sceneFile; metadata=trajectory.metadata;
save(rawOut,'sceneFile','metadata','-append');
report=audit_hmi_preview_display(rawOut,outputDir);
report.source_raw_file=rawFile; report.source_scene_file=sourceSceneFile;
report.source_scene_sha256=originalSceneHash; report.source_trajectory_sha256=hmi_workflow_file_hash(rawFile);
report.repaired_scene_file=repairedSceneFile; report.repaired_raw_file=rawOut;
report.urdf_collision_meshes=scene.robotBodyCollisionLoad;
report.before_body_collision_counts=originalCounts; report.after_body_collision_counts=afterCounts;
report.tool_collision_counts_unchanged=true; report.workpiece_collision_poses_unchanged=true;
report.tool_workpiece_transforms_and_skip_pairs_unchanged=true;
report.forward_kinematics_max_frobenius_delta=maxFkDelta;
report.trajectory_qPath_unchanged=isequal(original.qPath,trajectory.qPath);
report.trajectory_time_unchanged=isequal(original.timeFromStart,trajectory.timeFromStart);
report.trajectory_tcpPath_unchanged=isequal(original.tcpPath,trajectory.tcpPath);
report.executionReady=false; report.original_files_modified=false;
writeJson(fullfile(outputDir,'preview_display_audit.json'),report);
fprintf('Collision repair audit: coverage=%d collisionSamples=%d first=%s minGapMm=%g\n',...
    report.collision_coverage_pass,report.node_collision_sample_count+report.edge_collision_sample_count,...
    jsonencode(report.first_collision_sample),report.minimum_positive_distance_mm);
end
function counts=bodyCollisionCounts(robot)
names={'base_link_inertia','shoulder_link','upper_arm_link','forearm_link',...
    'wrist_1_link','wrist_2_link','wrist_3_link'};
counts=struct('name',{},'collision_count',{});
for k=1:numel(names)
    b=getBody(robot,names{k}); counts(k)=struct('name',names{k},'collision_count',numel(b.Collisions)); %#ok<AGROW>
end
end
function counts=toolCollisionCounts(robot,scene)
names={char(scene.mountBodyName),char(scene.tcpBodyName),char(scene.wireBodyName)};
counts=struct('name',{},'collision_count',{});
for k=1:numel(names)
    b=getBody(robot,names{k}); counts(k)=struct('name',names{k},'collision_count',numel(b.Collisions)); %#ok<AGROW>
end
end
function poses=environmentPoses(environment)
poses=cell(size(environment));
for k=1:numel(environment), poses{k}=environment{k}.Pose; end
end
function writeJson(file,data)
fid=fopen(file,'w'); assert(fid>=0,'Cannot write %s',file); c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(data,'PrettyPrint',true),'char');
end
