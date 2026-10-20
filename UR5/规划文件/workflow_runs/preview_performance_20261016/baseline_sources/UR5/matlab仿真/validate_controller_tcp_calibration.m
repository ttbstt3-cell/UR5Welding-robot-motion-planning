function validation = validate_controller_tcp_calibration(record,urdfFile,robot)
%VALIDATE_CONTROLLER_TCP_CALIBRATION Pure validation of the HMI RTDE mirror.
% No RTDE/ROS connection, no robot writes, no accuracy residual is fabricated.
% Full TCP offset is in UR controller tool0 coordinates (XYZ m, rotvec rad).
% The fixed URDF flange<-tool0 transform is applied once, never via mount.
if nargin<2 || strlength(string(urdfFile))==0
    % This file lives directly under UR5/matlab仿真; its parent is the
    % project root containing the calibrated URDF.
    root=fileparts(fileparts(mfilename('fullpath')));
    urdfFile=fullfile(root,'real_robot_description_calibrated.urdf');
end
if ~isstruct(record) || ~isfield(record,'tcp_authority') || ...
        ~strcmp(string(record.tcp_authority),"UR_CONTROLLER_ACTIVE_TCP")
    error('REAL_TCP_CALIBRATION:Authority', ...
        'REAL planning requires the controller-authoritative ACTIVE TCP mirror.');
end
for name={'valid','active_for_planning','applied_to_planning_model'}
    field=name{1};
    if ~isfield(record,field) || ~isscalar(record.(field)) || ...
            ~isequal(record.(field),true)
        error('REAL_TCP_CALIBRATION:Inactive','Controller TCP mirror field %s is not true.',field);
    end
end
if ~isfield(record,'status') || ~strcmp(string(record.status),"ACTIVE")
    error('REAL_TCP_CALIBRATION:Inactive','Controller TCP mirror is not ACTIVE.');
end
if ~isfield(record,'coordinate_frames') || ...
        ~isfield(record.coordinate_frames,'flange') || ...
        ~strcmp(string(record.coordinate_frames.flange),"URDF flange") || ...
        ~isfield(record.coordinate_frames,'base') || ...
        ~strcmp(string(record.coordinate_frames.base),"URDF base") || ...
        ~isfield(record.coordinate_frames,'controller_offset') || ...
        ~strcmp(string(record.coordinate_frames.controller_offset),"UR controller tool0")
    error('REAL_TCP_CALIBRATION:Frame','Controller TCP mirror has an incompatible frame declaration.');
end
urdfHash=localHash(urdfFile);
if ~isfield(record,'urdf_sha256') || ~strcmp(string(record.urdf_sha256),string(urdfHash))
    error('REAL_TCP_CALIBRATION:URDFVersion','Controller TCP mirror URDF SHA-256 is stale.');
end
offset=localVector(record,'controller_tcp_offset',6);
T=localTransform(record,'T_flange_tcp');
position=localVector(record,'position_flange_tcp_m',3);
if max(abs(T(1:3,4)-position))>1e-9
    error('REAL_TCP_CALIBRATION:Position','Controller TCP mirror position fields disagree.');
end
if nargin<3 || isempty(robot)
    robot=importrobot(urdfFile,'MeshPath','/opt/ros/humble/share');
    robot.DataFormat='row';
end
expectedNames={'shoulder_pan_joint','shoulder_lift_joint','elbow_joint', ...
    'wrist_1_joint','wrist_2_joint','wrist_3_joint'};
jointNames={};
for k=1:numel(robot.Bodies)
    body=robot.Bodies{k};
    if ~strcmp(body.Joint.Type,'fixed'), jointNames{end+1}=body.Joint.Name; end %#ok<AGROW>
end
if ~isequal(jointNames,expectedNames)
    error('REAL_TCP_CALIBRATION:JointOrder','Calibrated URDF joint order differs from the six standard UR joints.');
end
TflangeTool0=getTransform(robot,zeros(1,6),'tool0','flange');
Texpected=TflangeTool0*localUrPose(offset);
if max(abs(T(:)-Texpected(:)))>1e-9
    error('REAL_TCP_CALIBRATION:FixedTransform', ...
        'Controller TCP mirror is not fixed flange<-tool0 times the full tcp_offset.');
end
if ~isfield(record,'source_snapshot') || ~isstruct(record.source_snapshot)
    error('REAL_TCP_CALIBRATION:Snapshot','Controller TCP mirror has no source RTDE snapshot.');
end
sample=record.source_snapshot;
if isfield(sample,'frame_id') && ~strcmp(string(sample.frame_id),"base")
    error('REAL_TCP_CALIBRATION:SnapshotFrame','RTDE source snapshot is not in UR base.');
end
sampleOffset=localVector(sample,'tcp_offset',6);
if max(abs(sampleOffset-offset))>1e-7
    error('REAL_TCP_CALIBRATION:Snapshot','Stored TCP and source RTDE tcp_offset disagree.');
end
if isfield(sample,'actual_q')
    q=localVector(sample,'actual_q',6);
else
    q=localVector(sample,'q',6);
end
actualTcp=localVector(sample,'actual_TCP_pose',6);
Tfk=getTransform(robot,q.','tool0','base');
Tcontroller=localUrPose(actualTcp)/localUrPose(sampleOffset);
positionErrorMm=norm(Tfk(1:3,4)-Tcontroller(1:3,4))*1000;
R=Tfk(1:3,1:3)'*Tcontroller(1:3,1:3);
orientationErrorDeg=acosd(max(-1,min(1,(trace(R)-1)/2)));
% These are the existing HMI frame-consistency thresholds, not calibration
% quality thresholds and not user-supplied JSON thresholds.
if positionErrorMm>3.0 || orientationErrorDeg>0.5
    error('REAL_TCP_CALIBRATION:FrameAlignment', ...
        'RTDE/URDF source frame check FAIL: %.6f mm / %.6f deg (limits 3 mm / 0.5 deg).', ...
        positionErrorMm,orientationErrorDeg);
end
validation=struct('pass',true,'tcp_authority','UR_CONTROLLER_ACTIVE_TCP', ...
    'quality_level','CONTROLLER_DEFINED','physical_accuracy_status','NOT_MEASURED_BY_HMI', ...
    'position_error_mm',positionErrorMm,'orientation_error_deg',orientationErrorDeg, ...
    'max_position_error_mm',3.0,'max_orientation_error_deg',0.5, ...
    'urdf_sha256',urdfHash,'T_flange_tcp',Texpected);
end

function v=localVector(record,name,n)
if ~isfield(record,name)
    error('REAL_TCP_CALIBRATION:Schema','Missing controller TCP field %s.',name);
end
value=record.(name); v=double(value(:));
if numel(v)~=n || any(~isfinite(v))
    error('REAL_TCP_CALIBRATION:Vector','%s must be a finite %d-vector.',name,n);
end
end

function T=localTransform(record,name)
if ~isfield(record,name)
    error('REAL_TCP_CALIBRATION:Schema','Missing controller TCP field %s.',name);
end
T=double(record.(name));
if ~isequal(size(T),[4 4]) || any(~isfinite(T(:))) || ...
        norm(T(4,:)-[0 0 0 1])>1e-9 || ...
        norm(T(1:3,1:3)'*T(1:3,1:3)-eye(3),'fro')>1e-6 || ...
        abs(det(T(1:3,1:3))-1)>1e-6
    error('REAL_TCP_CALIBRATION:Transform','%s is not a rigid 4x4 transform.',name);
end
end

function T=localUrPose(pose)
v=pose(4:6); a=norm(v);
S=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
if a<1e-8
    A=1-a*a/6; B=0.5-a*a/24;
else
    A=sin(a)/a; B=(1-cos(a))/(a*a);
end
T=eye(4); T(1:3,1:3)=eye(3)+A*S+B*S*S; T(1:3,4)=pose(1:3);
end

function hash=localHash(file)
if ~isfile(file), error('REAL_TCP_CALIBRATION:URDFVersion','Calibrated URDF is missing: %s',file); end
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(file,'r'); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
md.update(fread(fid,Inf,'*uint8'));
hash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
