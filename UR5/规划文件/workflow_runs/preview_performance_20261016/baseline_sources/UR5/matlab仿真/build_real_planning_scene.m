function scene=build_real_planning_scene(previewOnly,realhomeConfigPath)
% Explicit REAL geometry adapter. No changes to algorithms or frozen data.
if nargin<1, previewOnly=false; end
here=fileparts(mfilename('fullpath')); root=fileparts(here);
addpath(genpath(fullfile(fileparts(root),'RRT')));
scene=loadUR5JointScene(fullfile(here,'sync_ur5_joint_states.m'));
frozen=scene;
% MATLAB strips the ROS package name from package:// mesh paths. Search
% inside that package, not its share parent (which silently lost all meshes).
urdfFile=fullfile(root,'real_robot_description_calibrated.urdf');
% MATLAB R2025b's URDF importer accepts one MeshPath value reliably here;
% this package root resolves the package-stripped meshes/ur5/... URI.
robot=importrobot(urdfFile,'MeshPath','/opt/ros/humble/share/ur_description');
robot.DataFormat='row';
[robot,bodyCollisionLoad]=load_ur5_body_collision_geometry(robot,urdfFile);
scene.robotBodyCollisionLoad=bodyCollisionLoad;
mount=rigidBody('WeldingGunMount'); joint=rigidBodyJoint('GunMount_fixed','fixed');
setFixedTransform(joint,frozen.T_tool0_mount); mount.Joint=joint; addBody(robot,mount,'tool0');
Tfm=getTransform(robot,zeros(1,6),'WeldingGunMount','flange');
tcpFile=fullfile(root,'real_tcp_calibration_active.json');
cal=load_real_tcp_calibration(tcpFile);
if ~isfield(cal,'applied_to_planning_model') || ~cal.applied_to_planning_model
    error('REAL_SCENE:TCP','Real TCP has not been explicitly applied');
end
tcpQuality=localTcpQuality(cal);
if ~previewOnly && strcmp(tcpQuality,'INVALID')
    error('REAL_SCENE:TCP','Real TCP quality is INVALID (RMS > 5 mm)');
end
if ~previewOnly && strcmp(tcpQuality,'UNVERIFIED')
    error('REAL_SCENE:TCP','Real TCP quality is UNVERIFIED');
end
if ~cal.orientationApplied
    Tnom=Tfm*frozen.T_mount_tcp;
    cal.T_flange_tcp(1:3,1:3)=Tnom(1:3,1:3);
end
tcp=rigidBody('WeldTCP'); j=rigidBodyJoint('WeldTCP_fixed','fixed');
setFixedTransform(j,Tfm\cal.T_flange_tcp); tcp.Joint=j; addBody(robot,tcp,'WeldingGunMount');
scene.robot=robot; scene.T_mount_tcp=Tfm\cal.T_flange_tcp;
% Read the saved HMI RealHome, never a frozen simulation anchor.  The
% previous implementation used ``\s*`` around newlines; MATLAB's regexp
% treats ``\s`` as including newlines, so a valid YAML list could be
% swallowed and incorrectly reported as missing.  Parse the block line by
% line instead, keeping the file and the Python HMI as one source of truth.
if nargin<2 || isempty(realhomeConfigPath)
    homeFile=fullfile(root,'ur5_hmi','config','hmi_config.yaml');
else
    homeFile=char(realhomeConfigPath);
    if ~startsWith(homeFile,filesep)
        homeFile=fullfile(root,homeFile);
    end
end
if ~isfile(homeFile), error('REAL_SCENE:HomeMissing','Saved HMI RealHome file is missing: %s',homeFile); end
configText=fileread(homeFile);
% Parse the top-level block directly.  MATLAB regexp(...,'split') has
% release-dependent cell/string trimming behaviour, and the HMI's YAML
% serializer legitimately emits either ``- value`` or an indented variant.
% Restrict the expression to horizontal whitespace so a newline can never be
% swallowed as part of the marker or a numeric row.
homeMatch=regexp(configText,'(?m)^real_home:[ \t]*','once');
metadataMatch=regexp(configText,'(?m)^real_home_metadata:[ \t]*','once');
if isempty(homeMatch), error('REAL_SCENE:HomeMissing','Saved HMI RealHome block is missing: %s',homeFile); end
if isempty(metadataMatch) || metadataMatch<=homeMatch
    metadataMatch=numel(configText)+1;
end
homeBlock=configText(homeMatch:metadataMatch-1);
tokens=regexp(homeBlock,'(?m)^[ \t]*-[ \t]*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)[ \t]*$','tokens');
values=zeros(1,numel(tokens));
for tokenIndex=1:numel(tokens)
    values(tokenIndex)=str2double(tokens{tokenIndex}{1});
end
if numel(values)~=6 || any(~isfinite(values)), error('REAL_SCENE:HomeInvalid','Invalid saved RealHome list in %s',homeFile); end
scene.qMat0=reshape(values,1,6);
scene.realHomeSourceFile=homeFile;
scene.realHomeSourceQ=scene.qMat0;
if previewOnly
    mountRecord=jsondecode(fileread(fullfile(root,'real_gun_mount_calibration.json')));
    scene=apply_real_gun_geometry(scene,mountRecord.T_flange_source,true);
else
    scene=apply_real_gun_geometry(scene,[],false);
end
wpFile=fullfile(root,'workpiece_pose_real.json'); wp=jsondecode(fileread(wpFile));
if ~strcmp(wp.frame_id,'base') || ~wp.geometry_valid || ~wp.valid
    error('REAL_SCENE:Workpiece','Invalid workpiece geometry/frame');
end
T=eye(4); T(1:3,1:3)=wp.rotation_matrix; T(1:3,4)=wp.position;
if norm(T(1:3,1:3)'*T(1:3,1:3)-eye(3),'fro')>1e-6 || det(T(1:3,1:3))<0
    error('REAL_SCENE:Workpiece','Invalid workpiece rotation');
end
% Quality is separate from operator/application metadata.  ACTIVE_WARNING
% (5--10 mm) remains usable for loading, preview and planning; only
% INVALID (>10 mm) is rejected by this planner scene gate.  The existing
% geometry-validity and scene-consistency checks remain independent gates.
workpieceQuality=localWorkpieceQuality(wp);
if ~previewOnly && strcmp(workpieceQuality,'INVALID')
    error('REAL_SCENE:Workpiece','Workpiece quality is INVALID (residual > 10 mm)');
end
if ~previewOnly && strcmp(workpieceQuality,'UNVERIFIED')
    error('REAL_SCENE:Workpiece','Workpiece quality is UNVERIFIED (P_CHECK residual is missing)');
end
Trootbase=getTransform(robot,zeros(1,6),'base');
scene.T_MatBase_Workpiece=Trootbase*T;
for k=1:numel(scene.environment)
    scene.environment{k}.Pose=scene.T_MatBase_Workpiece*scene.workpieceProxyInfo(k).T_workpiece_proxy;
end
% Append exported floor/fences/fixtures after the original four workpiece
% proxies. No workpiece index, shape, robot/tool collider or skip pair moves.
environmentFile=fullfile(root,'environment_geometry_real.json');
TrootCopBase=scene.T_CopBase_MatBase\eye(4);
[surroundings,environmentInfo,environmentLoad]=load_real_environment_geometry(...
    environmentFile,TrootCopBase,Trootbase);
scene.environmentNames=[cellstr(string({scene.workpieceProxyInfo.name})),{environmentInfo.name}];
scene.environment=[scene.environment,surroundings];
scene.realEnvironmentProxyInfo=environmentInfo;
scene.realEnvironmentGeometryLoad=environmentLoad;
scene.realEnvironmentGeometryFile=environmentFile;
scene.realEnvironmentLoaded=environmentLoad.pass;
scene.realTcpLoaded=true; scene.realWorkpieceLoaded=true;
scene.realTcpFile=tcpFile; scene.realWorkpieceFile=wpFile;
scene.realTcpAuthority=char(string(cal.tcp_authority));
scene.realTcpQuality=tcpQuality;
scene.realTcpSourceValidation=cal.controllerSourceValidation;
scene.realScenePreviewOnly=previewOnly;
% The REAL gun manifest may define the wire/tip as fixed CAD geometry.  It
% is not a physical contact-tip calibration, but it is execution-usable
% after the finite-transform and collision-package checks in
% apply_real_gun_geometry have passed.
scene.realWireAxisStatus=char(string(scene.wireStatus));
scene.realWireDefinition='MODEL_AXIS_TO_TCP';
scene.realWirePhysicalMeasurementRequired=false;
fprintf('REAL model-defined wire: status=%s, length=%.1f mm, diameter=%.2f mm, collision=%d\n', ...
    scene.realWireAxisStatus,scene.wireLengthM*1000,scene.wireDiameterM*1000,scene.wireCollisionEnabled);
fprintf('REAL wire axis in TCP: [%.6f %.6f %.6f]\n',scene.wireAxisTcp);
fprintf('REAL wire body chain: flange -> %s -> %s\n',char(scene.wireParentBody),char(scene.wireBodyName));
% apply_real_gun_geometry returns a copy with WeldingGunWire added. Use
% that actual collision robot, not the pre-replacement local robot copy.
scene.selfCollisionSkipPairs=deriveStructuralSelfCollisionSkips(scene.robot);
scene.selfCollisionSkipPairs=ignore_fixed_gun_wire_self_collision(scene.robot,scene.selfCollisionSkipPairs);
scene.jointLimits=deriveUR5JointLimits(robot);
% Version strings are kept in addition to file hashes.  Hashes detect any
% byte-level change; explicit versions make a trajectory report readable and
% prevent an anchor generated for one calibration record from being mistaken
% for a current real-scene result.
gunManifestFile=fullfile(root,'real_gun_geometry','real_gun_geometry.json');
gunManifest=jsondecode(fileread(gunManifestFile));
tcpVersion=localVersion(cal,'TCP-' + string(localHash(tcpFile)));
gunVersion=localVersion(gunManifest,'GUN-' + string(localHash(gunManifestFile)));
workpieceVersion=localVersion(wp,'WORKPIECE-' + string(localHash(wpFile)));
urdfFile=fullfile(root,'real_robot_description_calibrated.urdf');
urdfVersion='URDF-' + string(localHash(urdfFile));
scene.calibrationVersions=struct('tcp_calibration_version',char(tcpVersion), ...
 'gun_geometry_version',char(gunVersion),'workpiece_pose_version',char(workpieceVersion), ...
 'urdf_version',char(urdfVersion));
scene.calibrationBinding=struct('tcp_calibration_sha256',localHash(tcpFile), ...
 'real_gun_geometry_sha256',localHash(gunManifestFile), ...
 'gun_mount_sha256',localHash(fullfile(root,'real_gun_mount_calibration.json')), ...
 'workpiece_sha256',localHash(wpFile),'planning_scene','REAL_CALIBRATION', ...
 'tcp_calibration_version',char(tcpVersion),'gun_geometry_version',char(gunVersion), ...
 'workpiece_pose_version',char(workpieceVersion),'urdf_version',char(urdfVersion),...
 'environment_geometry_sha256',environmentLoad.source_sha256,...
 'environment_geometry_version',environmentLoad.version);
[scene.environmentCollisionPolicy,scene.fixedEnvironmentContact]=load_fixed_environment_contacts(scene,root);
scene.calibrationBinding.fixed_environment_contacts_sha256=scene.fixedEnvironmentContact.sha256;
end
function hash=localHash(file)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(file,'r'); c=onCleanup(@()fclose(fid)); bytes=fread(fid,Inf,'*uint8');
md.update(bytes); hash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
function value=localVersion(payload,fallback)
% Prefer the explicit record version, otherwise bind deterministically to
% the complete source-file hash.  No version is invented from geometry.
value=string(fallback);
if isstruct(payload)
    for name={'version','record_version','schema_version'}
        key=name{1};
        if isfield(payload,key) && ~isempty(payload.(key))
            value=string(payload.(key));
            return;
        end
    end
end
end

function level=localWorkpieceQuality(payload)
% Normalize old and new workpiece records to GOOD/WARNING/INVALID.
if isfield(payload,'quality_level') && ~isempty(payload.quality_level)
    candidate=upper(char(string(payload.quality_level)));
    if any(strcmp(candidate,{'GOOD','WARNING','INVALID'}))
        level=candidate; return;
    end
end

if ~isfield(payload,'residual_mm') || isempty(payload.residual_mm)
    level='UNVERIFIED'; return;
end
r=double(payload.residual_mm);
if ~isscalar(r) || ~isfinite(r) || r<0
    level='UNVERIFIED'; return;
end
if r<5
    level='GOOD';
elseif r<=10
    level='WARNING';
else
    level='INVALID';
end
end

function level=localTcpQuality(payload)
% Normalize old and new TCP records to GOOD/WARNING/INVALID.
% A validated controller mirror is authoritative parameter data, not a
% pivot experiment.  Absence of a pivot RMS is not a source validation FAIL.
if isfield(payload,'controllerSourceValidation') && ...
        payload.controllerSourceValidation.pass && ...
        strcmp(string(payload.tcp_authority),"UR_CONTROLLER_ACTIVE_TCP")
    level='CONTROLLER_DEFINED'; return;
end
if isfield(payload,'quality_level') && ~isempty(payload.quality_level)
    candidate=upper(char(string(payload.quality_level)));
    if any(strcmp(candidate,{'GOOD','WARNING','INVALID'}))
        level=candidate; return;
    end
end
r=[];
if isfield(payload,'fit') && isstruct(payload.fit) && isfield(payload.fit,'rms_residual_mm')
    r=double(payload.fit.rms_residual_mm);
end
if isempty(r) || ~isscalar(r) || ~isfinite(r) || r<0
    level='UNVERIFIED'; return;
end
if r<2
    level='GOOD';
elseif r<=5
    level='WARNING';
else
    level='INVALID';
end
end
