function result = RRTConnectJoint(config)
%RRTConnectJoint Phase 12A six-dimensional RRT-Connect baseline.
if nargin < 1, config = struct(); end
result = jointPlannerCore(config,'RRT-Connect');
end
