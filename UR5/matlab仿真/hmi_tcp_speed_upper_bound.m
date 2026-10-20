function result=hmi_tcp_speed_upper_bound(scene,maxVelocityPerJoint)
% Whole-path conservative Cartesian speed bound, no sampled-peak assumption.
names={'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};
current=scene.tcpBodyName; reach=0; radii=zeros(1,6); found=false(1,6);
for k=1:numel(scene.robot.BodyNames)+1
    if strcmp(current,scene.robot.BaseName), break; end
    body=getBody(scene.robot,current); joint=body.Joint;
    reach=reach+norm(joint.ChildToJointTransform(1:3,4));
    index=find(strcmp(names,joint.Name),1);
    if ~isempty(index), radii(index)=reach; found(index)=true; end
    reach=reach+norm(joint.JointToParentTransform(1:3,4));
    current=body.Parent.Name;
end
assert(all(found) && numel(maxVelocityPerJoint)==6 && all(isfinite(maxVelocityPerJoint)) && all(maxVelocityPerJoint>=0),...
    'LIMITS:TCP','Incomplete joint/TCP chain');
result=struct('method','triangle_inequality_global_joint_lever_bound',...
    'lever_arm_upper_bounds_m',radii,'upper_bound_m_s',sum(radii.*maxVelocityPerJoint(:).'));
end
