function [nodeType,canConnectStart,canConnectGoal] = classifyCRRTNode( ...
    xNew,startPoint,goalPoint,collisionFcn)
% classifyCRRTNode  Classify xNew into the four C-RRT* node types.

xNew = reshape(xNew,1,[]);
startPoint = reshape(startPoint,1,[]);
goalPoint = reshape(goalPoint,1,[]);
if numel(xNew) ~= 2 || numel(startPoint) ~= 2 || numel(goalPoint) ~= 2
    error('All node classification points must be 1-by-2.');
end
if ~isa(collisionFcn,'function_handle')
    error('collisionFcn must be a function handle.');
end

canConnectStart = logical(collisionFcn(startPoint,xNew));
canConnectGoal = logical(collisionFcn(xNew,goalPoint));

if canConnectStart && canConnectGoal
    nodeType = 1;
elseif canConnectStart
    nodeType = 2;
elseif canConnectGoal
    nodeType = 3;
else
    nodeType = 4;
end

end
