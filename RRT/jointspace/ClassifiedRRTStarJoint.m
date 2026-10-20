function result = ClassifiedRRTStarJoint(config)
%ClassifiedRRTStarJoint Full Classified-RRT* six-dimensional migration.
if nargin < 1, config = struct(); end
result = jointPlannerCore(config,'Full Classified-RRT*');
end
