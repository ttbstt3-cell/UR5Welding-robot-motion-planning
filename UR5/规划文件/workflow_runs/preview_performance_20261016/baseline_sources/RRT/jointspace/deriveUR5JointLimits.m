function limits = deriveUR5JointLimits(robot)
%deriveUR5JointLimits Read the six revolute limits from a rigidBodyTree.

limits = zeros(6,2);
count = 0;
for i = 1:numel(robot.Bodies)
    jt = robot.Bodies{i}.Joint;
    if strcmpi(jt.Type,'revolute') || strcmpi(jt.Type,'continuous')
        count = count+1;
        limits(count,:) = double(jt.PositionLimits);
    end
end
if count ~= 6
    error('Expected six UR5 revolute joints, found %d.',count);
end
if any(~isfinite(limits(:))) || any(limits(:,1) >= limits(:,2))
    error('UR5 joint limits are invalid.');
end
end
