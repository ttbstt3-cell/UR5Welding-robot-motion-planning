function [qContinuous, turnCounts] = lift_continuous_joint_path(qPath)
%LIFT_CONTINUOUS_JOINT_PATH Lift periodic joint representations continuously.
%
% RRT files may contain equivalent angles on opposite sides of a 2*pi
% boundary even when their edge check used the physically short angular
% difference.  This routine keeps the first row unchanged and adds/subtracts
% whole turns on later rows so each consecutive representation is the nearest
% continuous branch.  It never wraps values into [-pi,pi] and never changes
% the original input matrix.

qPath = double(qPath);
if ndims(qPath) ~= 2 || size(qPath,2) ~= 6
    error("qPath must be N-by-6.");
end
qContinuous = qPath;
turnCounts = zeros(size(qPath));
for i = 2:size(qPath,1)
    deltaToPrevious = qPath(i,:) - qContinuous(i-1,:);
    turns = round(deltaToPrevious/(2*pi));
    qContinuous(i,:) = qPath(i,:) - turns*(2*pi);
    turnCounts(i,:) = turns;
end
end
