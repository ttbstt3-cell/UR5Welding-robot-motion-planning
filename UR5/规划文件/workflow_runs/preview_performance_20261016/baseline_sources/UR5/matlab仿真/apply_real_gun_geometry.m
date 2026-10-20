function scene = apply_real_gun_geometry(scene,T_flange_source,previewOnly)
%APPLY_REAL_GUN_GEOMETRY Explicit independent REAL gun/bracket replacement.
% Preview consumes the same V-HACD components but is never execution-ready.
% NO change to joints, TCP transform, ROS, frozen files or workpiece poses.
if nargin<3, previewOnly=false; end
rootDir=fileparts(fileparts(mfilename('fullpath')));
realGun=load_real_gun_geometry(rootDir);
m=realGun.manifest;
if nargin<2 || isempty(T_flange_source)
    if ~realGun.active
        error('REAL_GUN:Unconfirmed','Gun installation/collision not confirmed: %s',realGun.status);
    end
    T_flange_source=realGun.T_flange_source;
elseif ~previewOnly
    error('REAL_GUN:PreviewOnly','Explicit unconfirmed transforms can only be used for offline preview.');
end
T=double(T_flange_source);
if ~isequal(size(T),[4 4]) || any(~isfinite(T(:))) || ...
        norm(T(4,:)-[0 0 0 1])>1e-9 || ...
        norm(T(1:3,1:3)'*T(1:3,1:3)-eye(3),'fro')>1e-6 || abs(det(T(1:3,1:3))-1)>1e-6
    error('REAL_GUN:Transform','A finite rigid URDF flange<-STL_SOURCE transform is required.');
end
if ~any(strcmp(scene.robot.BodyNames,'flange'))
    error('REAL_GUN:Frame','Use the calibrated real URDF with an explicit flange, not stock MATLAB tool0.');
end
if ~isfield(m,'collision') || ~strcmp(m.collision.method,'CoppeliaSim simConvex.vhacd') || isempty(m.collision.components)
    error('REAL_GUN:Collision','Actual CoppeliaSim V-HACD components required.');
end
robot=copy(scene.robot);
q0=zeros(1,6);
T_flange_mount=getTransform(robot,q0,'WeldingGunMount','flange');
T_mount_source=T_flange_mount\T;
mount=getBody(robot,'WeldingGunMount');
clearCollision(mount);
clearVisual(mount); % New STL includes the L bracket; no old duplicate assembly.
components=m.collision.components;
for k=1:numel(components)
    if iscell(components), c=components{k}; else, c=components(k); end
    v=double(c.vertices_m);
    if size(v,2)~=3 || size(v,1)<4 || any(~isfinite(v(:))) || rank(v-v(1,:))<3
        error('REAL_GUN:Component','Invalid convex component %d',k);
    end
    addCollision(mount,collisionMesh(v),T_mount_source);
end
meshFile=fullfile(rootDir,'real_gun_geometry','welding_gun_visual_m.stl');
if isfile(meshFile), addVisual(mount,'Mesh',meshFile,T_mount_source); end
replaceBody(robot,'WeldingGunMount',mount);
tcp=getBody(robot,'WeldTCP');
clearCollision(tcp); clearVisual(tcp); % TCP joint transform itself is unchanged.
% Use the fixed CAD-defined wire geometry from the same manifest as the gun
% collision package.  No physical measurement or nearest-hull inference is
% required, but all dimensions and transforms must be finite and rigid.
if ~realGun.wireGeometryValid
    error('REAL_GUN:Wire','MODEL_DEFINED_WIRE_INVALID: %s',realGun.wireError);
end
if ~isfield(scene,'T_mount_tcp') || ~isequal(size(scene.T_mount_tcp),[4 4])
    % Offline geometry-review callers may provide only the robot.  Recover
    % the already-installed fixed WeldTCP transform; never invent a new TCP.
    scene.T_mount_tcp=getTransform(scene.robot,zeros(1,6),'WeldTCP','WeldingGunMount');
end
T_mount_tcp=double(scene.T_mount_tcp);
T_tcp_mount=inv(T_mount_tcp);
pTcpCenterMount=T_mount_tcp*[0;0;0;1];
pTcpCenterSource=inv(T_mount_source)*pTcpCenterMount;
tcpBindingErrorM=norm(pTcpCenterSource(1:3)-realGun.wireTcpSource(:));
if tcpBindingErrorM>1e-6
    error('REAL_GUN:WireTcpStale', ...
        'Planning TCP differs from the selected confirmed TCP version by %.3f mm; explicitly synchronize the planning calibration.',tcpBindingErrorM*1000);
end
pTipMount=(T_mount_source*[realGun.wireTipSource(:);1]);
pTipTcp=(T_tcp_mount*pTipMount); pTipTcp=pTipTcp(1:3);
% Endpoint is the physical applied TCP position; marker size is display-only.
pEndMount=(T_mount_source*[realGun.wireEndSource(:);1]);
pEndTcp=(T_tcp_mount*pEndMount); pEndTcp=pEndTcp(1:3);
axisMount=T_mount_source(1:3,1:3)*realGun.wireAxisSource(:);
axisMount=axisMount/norm(axisMount);
axisTcp=T_tcp_mount(1:3,1:3)*axisMount; axisTcp=axisTcp/norm(axisTcp);
% The fixed cylinder must terminate at the applied TCP, not a marker surface.
endpointErrorM=norm((pTipTcp+realGun.wireLengthM*axisTcp)-pEndTcp);
if endpointErrorM>1e-6
    error('REAL_GUN:WireTcpStale', ...
        'Wire endpoint differs from the applied TCP by %.3f mm.',endpointErrorM*1000);
end
xRef=[1;0;0]; if abs(dot(xRef,axisTcp))>0.95, xRef=[0;1;0]; end
xWire=cross(xRef,axisTcp); xWire=xWire/norm(xWire);
yWire=cross(axisTcp,xWire); yWire=yWire/norm(yWire);
T_tcp_wire=eye(4); T_tcp_wire(1:3,1:3)=[xWire yWire axisTcp];
T_tcp_wire(1:3,4)=pTipTcp+0.5*realGun.wireLengthM*axisTcp;
% The wire is a fixed child of the physical gun mount, not of WeldTCP.
% This keeps the chain explicit: flange -> WeldingGunMount -> WeldingGunWire.
T_mount_wire=T_mount_tcp*T_tcp_wire;
wireBody=rigidBody('WeldingGunWire');
wireJoint=rigidBodyJoint('WeldingGunWire_fixed','fixed');
setFixedTransform(wireJoint,T_mount_wire); wireBody.Joint=wireJoint;
addVisual(wireBody,'Cylinder',[realGun.wireDiameterM/2 realGun.wireLengthM],eye(4), ...
    'FaceColor',[0.95 0.75 0.15],'FaceAlpha',1.0);
addCollision(wireBody,collisionCylinder(realGun.wireDiameterM/2,realGun.wireLengthM),eye(4));
addBody(robot,wireBody,'WeldingGunMount');
replaceBody(robot,'WeldTCP',tcp);
scene.robot=robot;
scene.gunCollisionMode="real_vhacd_compound";
scene.realGunGeometryManifest=realGun.manifestFile;
scene.realGunGeometryStatus=string(realGun.status);
scene.realGunGeometryActive=~previewOnly && realGun.active;
scene.realGunPreviewOnly=logical(previewOnly);
scene.realGunRequiresCollisionReview=true;
scene.T_flange_source=T;
scene.T_mount_source=T_mount_source;
scene.wireCollisionEnabled=true;
scene.wireStatus=string(realGun.wireStatus);
scene.wireLengthM=realGun.wireLengthM; scene.wireDiameterM=realGun.wireDiameterM;
scene.T_mount_wire=T_mount_wire; scene.T_tcp_wire=T_tcp_wire;
scene.wireAxisTcp=axisTcp;
scene.wireTipTcp=pTipTcp;
scene.wireEndTcp=pEndTcp;
scene.wireTcpBindingErrorM=tcpBindingErrorM;
scene.wireBodyName="WeldingGunWire";
scene.wireParentBody="WeldingGunMount";
scene.wireDefinition="MODEL_AXIS_TO_TCP";
scene.realGunComponentCount=numel(components);
scene.gunProxyInfo=components;
end
