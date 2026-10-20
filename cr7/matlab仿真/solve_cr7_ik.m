function [q,info] = solve_cr7_ik(target,seed)
scene=load_cr7_scene();if nargin<2,seed=scene.qCurrent;end
assert(isequal(size(target),[4 4]),'Target must be 4 x 4 in CR7_base');
ik=inverseKinematics('RigidBodyTree',scene.robot);
[q,info]=ik(scene.tcpBodyName,target,[1 1 1 1 1 1],seed);
end
