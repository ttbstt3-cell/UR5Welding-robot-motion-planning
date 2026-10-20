function report=test_controller_tcp_mirror(outputFile)
% Offline controller-mirror schema/conversion regression; no robot I/O.
% Synthetic RTDE-consistent poses are derived from the unchanged URDF.
% Temporary records cannot overwrite any real/frozen calibration file.
root=fileparts(fileparts(mfilename('fullpath')));
urdfFile=fullfile(root,'real_robot_description_calibrated.urdf');
robot=importrobot(urdfFile,'MeshPath','/opt/ros/humble/share'); robot.DataFormat='row';
temporary=tempname; mkdir(temporary);
cleanup=onCleanup(@()rmdir(temporary,'s')); %#ok<NASGU>
protectedFiles={urdfFile,fullfile(root,'real_tcp_calibration_applied.json'), ...
    fullfile(root,'scene_geometry.json'),fullfile(root,'ur5_tool_calibration.mat'), ...
    fullfile(root,'real_gun_geometry','real_gun_geometry.json')};
originalHashes=cellfun(@localHash,protectedFiles,'UniformOutput',false);
offset=[0.370;0.010;0.007;1.1;-0.35;0.24];
Ttool0Tcp=localUrPose(offset);
T=getTransform(robot,zeros(1,6),'tool0','flange')*Ttool0Tcp;
q=repmat([-1.0 -1.5 0.4 -1.2 0.1 -0.2],10,1)+deg2rad(sin((1:10)'*(1:6))*10);
positionError=zeros(10,1); angleError=zeros(10,1); relativeError=zeros(10,1);
record=struct('schema_version',2,'calibration_type','controller_active_tcp_mirror_6d', ...
    'tcp_authority','UR_CONTROLLER_ACTIVE_TCP','status','ACTIVE','valid',true, ...
    'active_for_planning',true,'applied_to_planning_model',true, ...
    'coordinate_frames',struct('base','URDF base','flange','URDF flange', ...
        'controller_offset','UR controller tool0','tcp','controller active TCP'), ...
    'controller_tcp_offset',offset.','T_flange_tcp',T, ...
    'position_flange_tcp_m',T(1:3,4).','urdf_sha256',localHash(urdfFile), ...
    'quality_level','CONTROLLER_DEFINED','physical_accuracy_status','NOT_MEASURED_BY_HMI', ...
    'version','SYNTHETIC-OFFLINE-CONTROLLER-TCP-FIXTURE');
for k=1:10
    TbaseTool0=getTransform(robot,q(k,:),'tool0','base');
    TbaseTcp=TbaseTool0*Ttool0Tcp;
    record.source_snapshot=struct('frame_id','base','tcp_offset',offset.', ...
        'actual_q',q(k,:),'actual_TCP_pose',localTransformPose(TbaseTcp));
    validation=validate_controller_tcp_calibration(record,urdfFile,robot);
    positionError(k)=validation.position_error_mm;
    angleError(k)=validation.orientation_error_deg;
    TbaseFlange=getTransform(robot,q(k,:),'flange','base');
    relativeError(k)=norm(TbaseFlange\TbaseTcp-T,'fro');
end
assert(max(positionError)<1e-6 && max(angleError)<1e-4 && max(relativeError)<1e-9);
activeFile=fullfile(temporary,'real_tcp_calibration_active.json');
legacyFile=fullfile(temporary,'real_tcp_calibration_applied.json');
copyfile(fullfile(root,'real_tcp_calibration_applied.json'),legacyFile);
% A sibling historical record must never rescue a missing ACTIVE mirror.
localExpectFailure(@()load_real_tcp_calibration(activeFile),'REAL_TCP_CALIBRATION:MissingFile');
legacy=load_real_tcp_calibration(legacyFile);
assert(legacy.orientationApplied==false && ~isfield(legacy,'controllerSourceValidation'));
localWrite(activeFile,record);
loaded=load_real_tcp_calibration(activeFile);
assert(loaded.orientationApplied && loaded.controllerSourceValidation.pass);
assert(strcmp(loaded.quality_level,'CONTROLLER_DEFINED') && ~isfield(loaded,'fit'));
manifest=jsondecode(fileread(fullfile(root,'real_gun_geometry','real_gun_geometry.json')));
wire=model_axis_wire_to_tcp(manifest,temporary);
endpoint=manifest.visual.T_flange_source*[wire.wire_end_source;1];
wireEndpointErrorMm=norm(endpoint(1:3)-T(1:3,4))*1000;
assert(wireEndpointErrorMm<1e-6 && abs(wire.diameter_m-0.0012)<1e-12);
% Persisted diagnostic PASS or relaxed limits cannot override recomputation.
bad=record; bad.source_snapshot.actual_TCP_pose(1)=bad.source_snapshot.actual_TCP_pose(1)+0.004;
bad.frame_alignment=struct('pass',true,'max_position_error_mm',100);
localExpectFailure(@()validate_controller_tcp_calibration(bad,urdfFile,robot), ...
    'REAL_TCP_CALIBRATION:FrameAlignment');
bad=record; bad.urdf_sha256=repmat('0',1,64);
localExpectFailure(@()validate_controller_tcp_calibration(bad,urdfFile,robot), ...
    'REAL_TCP_CALIBRATION:URDFVersion');
bad=record; bad.T_flange_tcp(1:3,4)=bad.T_flange_tcp(1:3,4)+[0.001;0;0];
bad.position_flange_tcp_m=bad.T_flange_tcp(1:3,4).';
localExpectFailure(@()validate_controller_tcp_calibration(bad,urdfFile,robot), ...
    'REAL_TCP_CALIBRATION:FixedTransform');
localWrite(activeFile,bad);
localExpectFailure(@()model_axis_wire_to_tcp(manifest,temporary), ...
    'REAL_TCP_CALIBRATION:FixedTransform');
bad=record; bad.source_snapshot.frame_id='tool0';
localExpectFailure(@()validate_controller_tcp_calibration(bad,urdfFile,robot), ...
    'REAL_TCP_CALIBRATION:SnapshotFrame');
bad=record; bad.source_snapshot.tcp_offset(1)=bad.source_snapshot.tcp_offset(1)+0.001;
localExpectFailure(@()validate_controller_tcp_calibration(bad,urdfFile,robot), ...
    'REAL_TCP_CALIBRATION:Snapshot');
bad=record; bad.controller_tcp_offset(1)=NaN;
localExpectFailure(@()validate_controller_tcp_calibration(bad,urdfFile,robot), ...
    'REAL_TCP_CALIBRATION:Vector');
bad=record; bad.tcp_authority='HMI_LOCAL_FOUR_POINT';
localExpectFailure(@()validate_controller_tcp_calibration(bad,urdfFile,robot), ...
    'REAL_TCP_CALIBRATION:Authority');
% An old four-point file renamed ACTIVE does not become controller data.
localWrite(activeFile,jsondecode(fileread(legacyFile)));
localExpectFailure(@()load_real_tcp_calibration(activeFile),'REAL_TCP_CALIBRATION:Authority');
for k=1:numel(protectedFiles), assert(strcmp(localHash(protectedFiles{k}),originalHashes{k})); end
report=struct('status','PASS','robot_motion','NO','source','SYNTHETIC_OFFLINE_ONLY', ...
    'joint_pose_count',10,'max_source_position_error_mm',max(positionError), ...
    'max_source_orientation_error_deg',max(angleError), ...
    'max_fixed_flange_tcp_transform_error',max(relativeError), ...
    'wire_endpoint_error_mm',wireEndpointErrorMm,'wire_diameter_mm',wire.diameter_m*1000, ...
    'quality_level','CONTROLLER_DEFINED','pivot_accuracy_claim','NONE', ...
    'bad_inputs_blocked',9,'historical_explicit_diagnostics','PASS', ...
    'missing_active_no_fallback','PASS','protected_file_hashes_unchanged',true);
disp(report);
if nargin>0, localWrite(outputFile,report); end
end

function localExpectFailure(callback,identifier)
failed=false;
try
    callback();
catch err
    if strcmp(err.identifier,identifier), failed=true; else, rethrow(err); end
end
assert(failed,'Expected blocked input with %s.',identifier);
end

function localWrite(file,payload)
fid=fopen(file,'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(payload,'PrettyPrint',true));
end

function T=localUrPose(pose)
v=pose(4:6); a=norm(v); S=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
T=eye(4); T(1:3,1:3)=eye(3)+(sin(a)/a)*S+((1-cos(a))/(a*a))*S*S;
T(1:3,4)=pose(1:3);
end

function pose=localTransformPose(T)
axang=rotm2axang(T(1:3,1:3)); pose=[T(1:3,4).' axang(1:3)*axang(4)];
end

function hash=localHash(file)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(file,'r'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
md.update(fread(fid,Inf,'*uint8'));
hash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
