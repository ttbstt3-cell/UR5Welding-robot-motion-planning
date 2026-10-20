function scene = load_cr7_scene()
% CoppeliaSim zero-frame transforms; joint rotation is around each local Z.
root=fileparts(fileparts(mfilename('fullpath')));
g=jsondecode(fileread(fullfile(root,'scene_geometry_matlab.json')));
robot=rigidBodyTree('DataFormat','row','MaxNumBodies',10);
b=rigidBody('CR7_base');addBody(robot,b,robot.BaseName);
parent='CR7_base';
for i=1:6
 b=rigidBody(sprintf('link%d',i));j=rigidBodyJoint(sprintf('joint%d',i),'revolute');
 j.JointAxis=[0 0 1];setFixedTransform(j,matrix12(g.joints(i).matrix));
 j.PositionLimits=[g.joints(i).interval(1),sum(g.joints(i).interval)];b.Joint=j;
 addBody(robot,b,parent);parent=b.Name;
end
b=rigidBody('WeldTCP');setFixedTransform(b.Joint,matrix12(g.tcp_matrix));addBody(robot,b,parent);
environment={};
Tworkpiece=eye(4);
for i=1:numel(g.shapes)
 s=g.shapes(i);
 if strcmp(s.name,'Workpiece')
  Tworkpiece=matrix12(s.matrix);
  for k=1:numel(g.ur5_workpiece.collision_proxies)
   proxy=g.ur5_workpiece.collision_proxies(k);p=proxy.pose_workpiece;
   c=collisionBox(proxy.size_m(1),proxy.size_m(2),proxy.size_m(3));
   T=eye(4);quat=p.quaternion(:)';T(1:3,1:3)=quat2rotm(quat([4 1 2 3]));T(1:3,4)=p.position(:);
   c.Pose=Tworkpiece*T;environment{end+1}=c;
  end
  continue;
 end
 if ~s.collidable || contains(s.name,'_visual'),continue;end
 parts=s.collision_components;
 for k=1:numel(parts)
  c=collisionMesh(parts(k).vertices);c.Pose=matrix12(s.matrix);
  if s.robot
   if s.owner==0,body='CR7_base';else,body=sprintf('link%d',s.owner);end
   addCollision(getBody(robot,body),c);
  else
   environment{end+1}=c;
  end
 end
end
% Reuse UR5's existing gun-head-to-TCP wire construction without moving TCP.
hullIndex=find(strcmp({g.shapes.name},'GunCollisionHull'),1);
h=g.shapes(hullIndex);mesh=load(h.mesh_file);T6Hull=matrix12(h.matrix);T6Tcp=matrix12(g.tcp_matrix);
points=mesh.vertices*T6Hull(1:3,1:3)'+T6Hull(1:3,4)';
[~,headIndex]=min(vecnorm(points-T6Tcp(1:3,4)',2,2));
headTcp=T6Tcp\[points(headIndex,:)';1];headTcp=headTcp(1:3);
wireAxis=-headTcp/norm(headTcp);ref=[1;0;0];if abs(dot(ref,wireAxis))>.95,ref=[0;1;0];end
x=cross(ref,wireAxis);x=x/norm(x);y=cross(wireAxis,x);
Twire=eye(4);Twire(1:3,1:3)=[x y wireAxis];Twire(1:3,4)=headTcp+.018/2*wireAxis;
wire=collisionCylinder(.0012/2,.018);wire.Pose=Twire;addCollision(getBody(robot,'WeldTCP'),wire);
scene=struct('robot',robot,'environment',{environment},'qCurrent',g.q_current(:)', ...
 'qOffset',zeros(1,6),'tcpBodyName','WeldTCP','T_CopBase_MatBase',eye(4), ...
 'T_MatBase_Workpiece',Tworkpiece,'wireAxisTcp',wireAxis,'T_tcp_wire',Twire,'wireLengthM',.018,'wireDiameterM',.0012,'geometry',g,'root',root,'jointNames',{{'joint1','joint2','joint3','joint4','joint5','joint6'}});
end
function T=matrix12(v)
T=[reshape(v,4,3)';0 0 0 1];
end
