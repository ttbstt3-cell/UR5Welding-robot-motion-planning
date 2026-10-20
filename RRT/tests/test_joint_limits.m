function pass = test_joint_limits()
%test_joint_limits Verify out-of-range states are rejected, not clamped.
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
limits=repmat([-1 1],6,1);
pass=jointWithinLimits(zeros(1,6),limits) && ...
    ~jointWithinLimits([0 0 0 0 0 1.1],limits);
assert(pass);
end
