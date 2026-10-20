function scene = loadUR5JointScene(syncScript)
%loadUR5JointScene Load the current offline UR5 MATLAB collision scene.
%
% The source script is deliberately the existing UR5 scene builder.  This
% function does not start ROS2/CoppeliaSim and does not alter the scene file.

if nargin < 1 || isempty(syncScript)
    syncScript = '/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m';
end
if ~isfile(syncScript), error('UR5 scene script not found: %s',syncScript); end
scene = struct();
run(syncScript);
if ~exist('scene','var') || ~isstruct(scene)
    error('UR5 scene script did not create a scene structure.');
end
if ~isfield(scene,'selfCollisionSkipPairs') || isempty(scene.selfCollisionSkipPairs)
    scene.selfCollisionSkipPairs=deriveStructuralSelfCollisionSkips(scene.robot);
end
scene.jointLimits=deriveUR5JointLimits(scene.robot);
end
