function [free,collisionIndex,collisionCount,timedOut] = collisionFreeJointEdge( ...
    robot,q1,q2,worldObjects,maxJointStep,options)
%% collisionFreeJointEdge  Check a straight joint-space edge for collision.
%
%   [free,collisionIndex] = collisionFreeJointEdge( ...
%       robot,q1,q2,worldObjects,maxJointStep,options)
%
%   The edge q1--q2 is interpolated linearly in joint space.  Every
%   interpolated configuration is checked with checkCollision, including
%   robot self-collision, collision with worldObjects, and collision
%   geometry attached to the robot (for example, a welding torch).
%
%   robot.DataFormat must be 'row'.  worldObjects must contain collision
%   objects expressed in the robot base frame.  maxJointStep is in radians
%   and defaults to 0.5 degrees.  For UR5 joint-space planning, options may
%   contain selfCollisionSkipPairs and wrapJoints=true.  A planner may also
%   provide stateCheckFcn(q), returning [free,detail,actualCheck], to reuse
%   its per-run state cache.  The third output is the number of actual state
%   collision checks performed.

if nargin < 5 || isempty(maxJointStep)
    maxJointStep = deg2rad(0.5);
end
if nargin < 6 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error('options must be a scalar structure.');
end
if ~isfield(options,'wrapJoints') || isempty(options.wrapJoints)
    options.wrapJoints = false;
end
if ~isfield(options,'selfCollisionSkipPairs')
    % Preserve the historical root-level default.  UR5 planners override
    % this with the explicit attachment-pair table from the scene loader.
    options.selfCollisionSkipPairs = 'parent';
end
if ~isfield(options,'timeoutFcn') || isempty(options.timeoutFcn)
    options.timeoutFcn = [];
end
if ~isfield(options,'environmentCollisionPolicy'), options.environmentCollisionPolicy=[]; end

if ~isscalar(maxJointStep) || ~isfinite(maxJointStep) || maxJointStep <= 0
    error('maxJointStep must be a finite positive scalar in radians.');
end

q1 = reshape(q1,1,[]);
q2 = reshape(q2,1,[]);

if numel(q1) ~= numel(q2)
    error('q1 and q2 must contain the same number of joints.');
end

if any(~isfinite(q1)) || any(~isfinite(q2))
    error('q1 and q2 must contain finite joint values.');
end

free = true;
collisionIndex = NaN;
collisionCount = 0;
timedOut = false;

dq = q2-q1;
if logical(options.wrapJoints)
    dq = atan2(sin(dq),cos(dq));
end
numSegments = max(1,ceil(max(abs(dq))/maxJointStep));

for k = 0:numSegments
    if ~isempty(options.timeoutFcn) && options.timeoutFcn()
        free = false;
        timedOut = true;
        collisionIndex = k+1;
        return;
    end
    alpha = k/numSegments;
    q = q1+alpha*dq;

    if isfield(options,'stateCheckFcn') && ~isempty(options.stateCheckFcn)
        [stateFree,~,actualCheck] = options.stateCheckFcn(q);
    else
        [stateFree,~] = collisionFreeJointState( ...
            robot,q,worldObjects,options.selfCollisionSkipPairs,options.environmentCollisionPolicy);
        actualCheck = true;
    end

    if actualCheck
        collisionCount = collisionCount+1;
    end

    if ~stateFree
        free = false;
        collisionIndex = k+1;
        return;
    end
end

end
