function pass = test_joint_wrap()
%test_joint_wrap Verify 179 -> -179 is a 2-degree move.
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
dq = jointWrapDelta([0 0 0 0 0 deg2rad(179)], ...
    [0 0 0 0 0 deg2rad(-179)]);
pass = abs(rad2deg(dq(6))-2) < 1e-10;
assert(pass);
end
