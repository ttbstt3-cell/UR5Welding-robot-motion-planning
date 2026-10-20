function report = test_phase12a_joint_planners()
%test_phase12a_joint_planners Run a short real-UR5 planner smoke test.
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
scene=loadUR5JointScene();
s=load('/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_approach_exit.mat','result');
c=struct('scene',scene,'start',s.result.qHome,'goal',s.result.preWeld.q, ...
    'maxIterations',50,'maxJointStepRad',deg2rad(25), ...
    'jointEdgeResolutionRad',deg2rad(5),'seed',1);
names={'RRTConnectJoint','BiRRTStarJoint','ClassGoalLiteJoint','ClassifiedRRTStarJoint'};
report=repmat(struct('algorithm','','success',false,'pathValid',false),0,1);
for i=1:numel(names)
    r=feval(names{i},c);
    report(end+1,1)=struct('algorithm',r.algorithm,'success',r.success, ...
        'pathValid',r.success && ~isempty(r.path)); %#ok<AGROW>
end
end
