function report=test_model_axis_wire_to_tcp(outputFile)
% Offline REAL-scene geometry consistency only. No planner/ROS/motion.
scene=build_real_planning_scene(false);
root=fileparts(fileparts(mfilename('fullpath')));
g=load_real_gun_geometry(root);
assert(g.wireGeometryValid && g.wireStatus=="MODEL_AXIS_TO_TCP_ACTIVE");
assert(scene.wireCollisionEnabled && scene.realGunComponentCount>0);
body=getBody(scene.robot,'WeldingGunWire');
assert(strcmp(body.Parent.Name,'WeldingGunMount') && numel(body.Collisions)==1);
% rigidBody.Collisions is a description cell in this MATLAB release.
dimensions=sscanf(body.Collisions{1},'Cylinder Radius %f Length %f');
assert(numel(dimensions)==2 && abs(dimensions(1)-0.0006)<1e-12);
assert(abs(dimensions(2)-g.wireLengthM)<1e-12);
pairs=string(scene.selfCollisionSkipPairs);
assert(any((pairs(:,1)=="WeldingGunMount" & pairs(:,2)=="WeldingGunWire") | ...
    (pairs(:,2)=="WeldingGunMount" & pairs(:,1)=="WeldingGunWire")));
qHome=double(scene.qMat0(:).');
q=repmat(qHome,10,1);
q=q+deg2rad(sin((1:10)'*(1:6))*10);
errors=zeros(10,1); fixedErrors=zeros(10,1); startBase=zeros(10,3); endBase=zeros(10,3);
for k=1:10
    Tf=getTransform(scene.robot,q(k,:),'flange');
    Tw=getTransform(scene.robot,q(k,:),'WeldingGunWire');
    Ttcp=getTransform(scene.robot,q(k,:),'WeldTCP');
    endpoint=Tw*[0;0;g.wireLengthM/2;1];
    errors(k)=norm(endpoint(1:3)-Ttcp(1:3,4))*1000;
    expected=Tf*scene.T_flange_source*[g.wireTipSource;1];
    start=Tw*[0;0;-g.wireLengthM/2;1];
    fixedErrors(k)=norm(start(1:3)-expected(1:3))*1000;
    TwBase=getTransform(scene.robot,q(k,:),'WeldingGunWire','base');
    a=TwBase*[0;0;-g.wireLengthM/2;1]; b=TwBase*[0;0;g.wireLengthM/2;1];
    startBase(k,:)=a(1:3).'; endBase(k,:)=b(1:3).';
end
assert(max(errors)<1e-6 && max(fixedErrors)<1e-6);
% Geometry-specific invalid inputs stay blocked (no quality-gate changes).
m=g.manifest; m.wire.gun_tip_model=[0;0;0]; m.visual.T_flange_source=eye(4);
cal=load_real_tcp_calibration(fullfile(root,'real_tcp_calibration_applied.json'));
m.visual.T_flange_source(1:3,4)=cal.position;
failed=false;
try, model_axis_wire_to_tcp(m,root); catch err, failed=strcmp(err.identifier,'REAL_GUN:WireLength'); end
assert(failed);
[selfCollision,~]=checkCollision(scene.robot,qHome,'IgnoreSelfCollision','off', ...
    'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
[environmentCollision,~]=checkCollision(scene.robot,qHome,scene.environment, ...
    'IgnoreSelfCollision','on','Exhaustive','on');
report=struct('status','PASS','robot_motion','NO','wire_axis_status',char(g.wireStatus), ...
    'length_mm',g.wireLengthM*1000,'diameter_mm',g.wireDiameterM*1000, ...
    'tip_source_m',g.wireTipSource.','endpoint_source_m',g.wireEndSource.', ...
    'axis_source',g.wireAxisSource.','gun_component_count',scene.realGunComponentCount, ...
    'max_endpoint_tcp_error_mm',max(errors),'max_start_transform_error_mm',max(fixedErrors), ...
    'pose_count',10,'wire_collision_enabled',scene.wireCollisionEnabled, ...
    'saved_realhome_self_collision',any(selfCollision(:)), ...
    'saved_realhome_environment_collision',any(environmentCollision(:)), ...
    'trajectory_acceptance','NOT_TESTED_OLD_TRAJECTORIES_STALE');
report.q_rad=q; report.wire_start_base_m=startBase; report.wire_end_base_m=endBase;
disp(report);
if nargin>0
    fid=fopen(outputFile,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));
end
end
