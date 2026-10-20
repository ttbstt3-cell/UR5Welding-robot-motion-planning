function result = RRT_basic_impl(config)
% RRT_basic_impl  Original Basic RRT search with unified statistics.
%
% The sampling, nearest-node, fixed-step extension, goal-threshold, and
% collision order mirror RRT_basic_animation.m.  This implementation only
% adds a configuration interface, counters, and a benchmark-safe result.

if nargin < 1
    config = struct();
end

config = normalizeRRTConfig(config);
rng(config.seed,'twister');

result = initRRTResult('Basic RRT',config);
result.optimizationIterationsRequested = 0;
result.finalValidationCollisionChecks = 0;
result.coreCollisionChecks = 0;
result.searchCollisionChecks = 0;

nodes = config.start;
parent = 0;
goalReached = false;
goalIndex = -1;
firstPath = [];
firstPathLength = NaN;
firstSolutionIteration = NaN;
iterationsUsed = 0;
searchPlanningTime = 0;
collisionChecks = 0;

if config.showAnimation
    createPlannerFigure(config,'Basic RRT');
end

if ~checkedCollision(config.start,config.start) || ...
        ~checkedCollision(config.goal,config.goal)
    result.iterations = 0;
    result.nodeCount = size(nodes,1);
    result.nodes = nodes;
    result.parent = parent;
    result.collisionChecks = collisionChecks;
    result.coreCollisionChecks = collisionChecks;
    result.searchCollisionChecks = collisionChecks;
    result.startTree = makeTree(nodes,parent);
    result.goalTree = struct('nodes',config.goal,'parent',0);
    return;
end

for iter = 1:config.maxIterations
    iterationsUsed = iter;
    iterationTimer = tic;

    % Keep the original uniform sample sequence: exactly two rand calls.
    xRand = sampleUniform(config);
    [nearestIndex,~] = nearestNode(nodes,xRand);
    xNear = nodes(nearestIndex,:);
    direction = xRand-xNear;
    distanceToRand = norm(direction);

    if distanceToRand < 1e-12
        searchPlanningTime = searchPlanningTime+toc(iterationTimer);
        continue;
    end

    moveDistance = min(config.stepSize,distanceToRand);
    xNew = xNear+moveDistance*direction/distanceToRand;

    if ~isInsideMap(xNew) || ~checkedCollision(xNear,xNew)
        searchPlanningTime = searchPlanningTime+toc(iterationTimer);
        continue;
    end

    nodes(end+1,:) = xNew;
    parent(end+1,1) = nearestIndex;
    newIndex = size(nodes,1);

    if norm(xNew-config.goal) <= config.goalThreshold && ...
            checkedCollision(xNew,config.goal)
        nodes(end+1,:) = config.goal;
        parent(end+1,1) = newIndex;
        goalIndex = size(nodes,1);
        goalReached = true;
        firstSolutionIteration = iter;
        firstPath = backtrackPath(nodes,parent,goalIndex);
        firstPathLength = pathLength(firstPath);
        firstSolutionTime = searchPlanningTime+toc(iterationTimer);
    end

    iterationElapsed = toc(iterationTimer);
    searchPlanningTime = searchPlanningTime+iterationElapsed;

    if config.showAnimation
        drawPlannerEdge(xNear,xNew,[0.20 0.55 0.20],config.animationPause);
    end

    if goalReached
        break;
    end
end

result.iterations = iterationsUsed;
result.nodeCount = size(nodes,1);
result.nodes = nodes;
result.parent = parent;
result.searchPlanningTime = searchPlanningTime;
result.planningTime = searchPlanningTime;
result.firstSolutionTime = NaN;
result.searchCollisionChecks = collisionChecks;
result.coreCollisionChecks = collisionChecks;

if ~goalReached
    result.collisionChecks = collisionChecks;
    result.startTree = makeTree(nodes,parent);
    result.goalTree = struct('nodes',config.goal,'parent',0);
    if config.showAnimation
        title('Basic RRT failed');
    end
    return;
end

result.success = true;
result.firstSolutionTime = firstSolutionTime;
result.firstSolutionIteration = firstSolutionIteration;
result.firstPath = firstPath;
result.firstPathLength = firstPathLength;
result.path = firstPath;
result.pathLength = firstPathLength;
result.finalValidationCollisionChecks = validatePathCounted(firstPath);
result.collisionChecks = collisionChecks;
result.coreCollisionChecks = collisionChecks;
result.searchCollisionChecks = collisionChecks;
result.collisionChecks = result.collisionChecks+ ...
    result.finalValidationCollisionChecks;
result.planningTime = searchPlanningTime;
result.totalAlgorithmTime = searchPlanningTime;
result.startTree = makeTree(nodes,parent);
result.goalTree = struct('nodes',config.goal,'parent',0);

if config.showAnimation
    plot(firstPath(:,1),firstPath(:,2),'b-','LineWidth',2.8);
    plot(firstPath(:,1),firstPath(:,2),'bo','MarkerFaceColor','b', ...
        'MarkerSize',4);
    title(sprintf('Basic RRT - %.3f ms',result.planningTime*1000));
    drawnow;
end

    function free = checkedCollision(p1,p2)
        collisionChecks = collisionChecks+1;
        free = collisionFree(p1,p2,config.obstacles,config.clearance);
    end

    function valid = validatePathCounted(pathToCheck)
        valid = true;
        for edgeIndex = 1:size(pathToCheck,1)-1
            if ~checkedCollision(pathToCheck(edgeIndex,:), ...
                    pathToCheck(edgeIndex+1,:))
                valid = false;
                return;
            end
        end
    end

    function inside = isInsideMap(point)
        inside = point(1) >= config.mapBounds(1) && ...
            point(1) <= config.mapBounds(2) && ...
            point(2) >= config.mapBounds(3) && ...
            point(2) <= config.mapBounds(4);
    end

end

function tree = makeTree(nodes,parent)
tree.nodes = nodes;
tree.parent = parent;
tree.cost = zeros(size(parent));
for i = 2:size(nodes,1)
    tree.cost(i) = tree.cost(parent(i))+ ...
        norm(nodes(i,:)-nodes(parent(i),:));
end
end
