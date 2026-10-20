function pass = test_joint_steer()
%test_joint_steer Verify norm-limited shortest-direction steering.
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
limits=repmat([-2*pi 2*pi],6,1);
[qNew,reached,valid]=steerJoint(zeros(1,6),ones(1,6),sqrt(6)/2,limits);
pass=valid && ~reached && abs(norm(qNew)-sqrt(6)/2)<1e-10;
assert(pass);
end
