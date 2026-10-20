function publish_cr7_trajectory(q,t)
scene=load_cr7_scene();
assert(size(q,2)==6 && size(q,1)==numel(t),'Require N x 6 q and N timestamps');
assert(all(isfinite(q),'all') && all(isfinite(t)) && all(diff(t)>0) && t(1)>=0,'Invalid trajectory');
for i=1:6
 lim=scene.geometry.joints(i).interval;
 assert(all(q(:,i)>=lim(1) & q(:,i)<=sum(lim)),'Joint limit violation');
end
node=ros2node('/matlab_cr7_trajectory');
pub=ros2publisher(node,'/cr7/joint_trajectory','trajectory_msgs/JointTrajectory');
msg=ros2message(pub);msg.joint_names=scene.jointNames;
for k=1:numel(t)
 p=ros2message('trajectory_msgs/JointTrajectoryPoint');p.positions=q(k,:);
 ns=round(t(k)*1e9);p.time_from_start.sec=int32(floor(ns/1e9));p.time_from_start.nanosec=uint32(mod(ns,1e9));
 msg.points(k)=p;
end
pause(1);send(pub,msg);pause(.2);
end
