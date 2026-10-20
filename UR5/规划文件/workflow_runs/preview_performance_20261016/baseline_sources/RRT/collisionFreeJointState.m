function [free,detail] = collisionFreeJointState( ...
    robot,q,worldObjects,selfCollisionSkipPairs,environmentCollisionPolicy)
%collisionFreeJointState Check one UR5 configuration against all collisions.
%
% This is the single state-level collision interface used by
% collisionFreeJointEdge and Phase 12 joint-space planners.  It checks the
% robot/tool against the environment and then checks non-structural robot
% self-collisions.  Structural attachment pairs are supplied explicitly by
% the current UR5 scene loader.
% Optional fifth argument: an explicitly certified fixed base/support policy.
% Omission retains strict native environment checking.

if nargin < 4 || isempty(selfCollisionSkipPairs)
    selfCollisionSkipPairs = {};
end
if nargin<5, environmentCollisionPolicy=[]; end

q = reshape(double(q),1,[]);
if numel(q) ~= 6 || any(~isfinite(q))
    error('UR5 joint state must be a finite 1-by-6 row vector.');
end

detail = struct('environmentCollision',false, ...
    'selfCollision',false,'environmentRaw',false,'selfRaw',false);

[environmentRaw,~,~,detail.fixedEnvironmentContact]=checkEnvironmentCollision(...
    robot,q,worldObjects,environmentCollisionPolicy);

detail.environmentRaw = environmentRaw;
detail.environmentCollision = any(environmentRaw(:));
if detail.environmentCollision
    free = false;
    return;
end

if isempty(selfCollisionSkipPairs)
    try
        selfRaw = checkCollision(robot,q,'IgnoreSelfCollision','off', ...
            'Exhaustive','on');
    catch
        selfRaw = checkCollision(robot,q,'IgnoreSelfCollision','off');
    end
else
    try
        selfRaw = checkCollision(robot,q,'IgnoreSelfCollision','off', ...
            'SkippedSelfCollisions',selfCollisionSkipPairs,'Exhaustive','on');
    catch
        selfRaw = checkCollision(robot,q,'IgnoreSelfCollision','off', ...
            'SkippedSelfCollisions',selfCollisionSkipPairs);
    end
end

detail.selfRaw = selfRaw;
detail.selfCollision = any(selfRaw(:));
free = ~detail.selfCollision;
end
