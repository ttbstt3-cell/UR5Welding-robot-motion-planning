function pass = test_joint_distance()
%test_joint_distance Verify shortest-angle weighted metric.
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
pass = abs(jointDistance([0 0 0 0 0 pi-deg2rad(1)], ...
    [0 0 0 0 0 -pi+deg2rad(1)])-deg2rad(2)) < 1e-10;
assert(pass);
end
