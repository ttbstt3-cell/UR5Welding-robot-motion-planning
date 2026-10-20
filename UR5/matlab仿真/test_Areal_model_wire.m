function report=test_Areal_model_wire(outputFile)
% Explicit offline wire preview does NOT activate the staged gun package.
root=fileparts(fileparts(mfilename('fullpath')));
folder=fullfile(root,'real_gun_geometry');
m=jsondecode(fileread(fullfile(folder,'real_gun_geometry.json')));
wire=model_axis_wire_to_tcp(m,root,true);
py=jsondecode(fileread(fullfile(folder,'Areal_wire_definition.json')));
assert(strcmp(wire.status,'MODEL_AXIS_TO_TCP_PREVIEW'));
assert(norm(wire.tip_source-double(py.tip_source(:)))<1e-12);
assert(norm(wire.wire_end_source-double(py.wire_end_source(:)))<1e-12);
assert(abs(wire.length_m-py.length_m)<1e-12 && wire.diameter_m==.0012);
robot=importrobot(fullfile(root,'real_robot_description_calibrated.urdf'),'MeshPath','/opt/ros/humble/share');
robot.DataFormat='row';
cal=load(fullfile(root,'ur5_tool_calibration.mat'));
mount=rigidBody('WeldingGunMount'); jm=rigidBodyJoint('GunMount_fixed','fixed');
setFixedTransform(jm,cal.T_tool0_mount); mount.Joint=jm; addBody(robot,mount,'tool0');
Tfm=getTransform(robot,zeros(1,6),'WeldingGunMount','flange');
T=double(m.visual.T_flange_source);
axis=wire.axis_source; ref=[1;0;0]; if abs(dot(ref,axis))>.95, ref=[0;1;0]; end
x=cross(ref,axis); x=x/norm(x); y=cross(axis,x);
Tsw=eye(4); Tsw(1:3,1:3)=[x,y,axis]; Tsw(1:3,4)=(wire.tip_source+wire.wire_end_source)/2;
Tmw=Tfm\T*Tsw;
body=rigidBody('WeldingGunWire'); jw=rigidBodyJoint('WeldingGunWire_fixed','fixed');
setFixedTransform(jw,Tmw); body.Joint=jw;
addCollision(body,collisionCylinder(wire.diameter_m/2,wire.length_m));
addBody(robot,body,'WeldingGunMount');
tcp=load_real_tcp_calibration(fullfile(root,'real_tcp_calibration_active.json'));
req=jsondecode(fileread(fullfile(folder,'Areal_consistency_request.json')));
q=double(req.q_rad); errors=zeros(size(q,1),1); centers=zeros(size(q,1),3);
for k=1:size(q,1)
    Tf=getTransform(robot,q(k,:),'flange','base');
    Tw=getTransform(robot,q(k,:),'WeldingGunWire','base');
    endpoint=Tw*[0;0;wire.length_m/2;1]; expected=Tf*tcp.T_flange_tcp*[0;0;0;1];
    errors(k)=norm(endpoint(1:3)-expected(1:3))*1000;
    centers(k,:)=Tw(1:3,4)';
end
report=struct('status','OFFLINE_PREVIEW_PASS','robot_motion','NO','pose_count',size(q,1), ...
    'diameter_mm',wire.diameter_m*1000,'length_mm',wire.length_m*1000, ...
    'max_endpoint_TCP_error_mm',max(errors),'HMI_MATLAB_definition_pass',true, ...
    'collision_body_created',true,'collision_parent','WeldingGunMount', ...
    'preview_only',true,'gun_manifest_status',m.status,'execution_authorized',false, ...
    'tcp_version',py.tcp_calibration_version,'source_sha256',m.source.sha256, ...
    'centers_base_m',centers);
fid=fopen(outputFile,'w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(report),'char');
fprintf('WIRE_PREVIEW diameter=%.3fmm length=%.6fmm endpointError=%.12gmm collision=CREATED execution=BLOCKED\n', ...
    report.diameter_mm,report.length_mm,report.max_endpoint_TCP_error_mm);
end
