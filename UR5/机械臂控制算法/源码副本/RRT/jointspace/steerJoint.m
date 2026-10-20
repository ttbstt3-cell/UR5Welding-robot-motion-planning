function [qNew,reached,valid,dq] = steerJoint( ...
    qNear,qTarget,maxJointStepRad,jointLimits)
%steerJoint Extend along the shortest joint-space direction.
%
% The step is limited by the Euclidean norm.  The returned state is never
% clamped; if it cannot be represented inside the supplied limits,
% valid=false is returned and the caller must reject the expansion.

qNear = reshape(double(qNear),1,[]);
qTarget = reshape(double(qTarget),1,[]);
if numel(qNear) ~= 6 || numel(qTarget) ~= 6
    error('UR5 states must be 1-by-6.');
end
if ~isscalar(maxJointStepRad) || ~isfinite(maxJointStepRad) || ...
        maxJointStepRad <= 0
    error('maxJointStepRad must be positive and finite.');
end
if ~jointWithinLimits(qNear,jointLimits) || ...
        ~jointWithinLimits(qTarget,jointLimits)
    qNew = qNear; reached = false; valid = false; dq = NaN(1,6); return;
end

dq = jointWrapDelta(qNear,qTarget);
distance = norm(dq,2);
if distance <= 1e-12
    qNew = qTarget;
    reached = true;
else
    scale = min(1,maxJointStepRad/distance);
    qNew = qNear + scale*dq;
    reached = scale >= 1-1e-12;
end

% An equivalent +/-2*pi representation is accepted only when it lies inside
% the declared limits.  No clamping is performed.
for j = 1:6
    while qNew(j) > jointLimits(j,2)+1e-12 && ...
            qNew(j)-2*pi >= jointLimits(j,1)-1e-12
        qNew(j) = qNew(j)-2*pi;
    end
    while qNew(j) < jointLimits(j,1)-1e-12 && ...
            qNew(j)+2*pi <= jointLimits(j,2)+1e-12
        qNew(j) = qNew(j)+2*pi;
    end
end
valid = jointWithinLimits(qNew,jointLimits);
if ~valid, qNew = qNear; end
end
