function path=plan_cr7_rrt(qStart,qGoal)
% Generic collision-aware RRT-Connect; does not publish motion.
scene=load_cr7_scene();
planner=manipulatorRRT(scene.robot,scene.environment);
planner.SkippedSelfCollisions='parent';planner.ValidationDistance=deg2rad(2);
planner.MaxConnectionDistance=deg2rad(15);
path=plan(planner,qStart,qGoal);
assert(~isempty(path),'No collision-free path found');
path=interpolate(planner,path);
end
