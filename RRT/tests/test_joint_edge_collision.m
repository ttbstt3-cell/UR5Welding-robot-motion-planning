function pass = test_joint_edge_collision()
%test_joint_edge_collision Check the shared UR5 edge interface at qMat0.
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
scene=loadUR5JointScene();
opts=struct('wrapJoints',true);
opts.selfCollisionSkipPairs=scene.selfCollisionSkipPairs;
[free,~,count]=collisionFreeJointEdge(scene.robot,scene.qMat0,scene.qMat0, ...
    scene.environment,deg2rad(2),opts);
pass=free && count==2;
assert(pass);
end
