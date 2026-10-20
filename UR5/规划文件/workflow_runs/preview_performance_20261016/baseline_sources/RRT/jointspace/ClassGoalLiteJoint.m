function result = ClassGoalLiteJoint(config)
%ClassGoalLiteJoint 6D Classification+GoalTree without extra mechanisms.
if nargin < 1, config = struct(); end
result = jointPlannerCore(config,'ClassGoal-Lite');
end
