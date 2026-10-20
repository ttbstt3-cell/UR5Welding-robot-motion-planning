function result = BiRRTStarJoint(config)
%BiRRTStarJoint Phase 12A six-dimensional bidirectional RRT* baseline.
if nargin < 1, config = struct(); end
result = jointPlannerCore(config,'Bi-RRT*');
end
