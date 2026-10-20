function result = Bi_RRT_impl(config)
% Bi_RRT_impl  Original Bi-RRT search with unified statistics.
%
% The alternating tree order, one-step expansion of each tree, connection
% threshold, and random-call order mirror Bi_RRT_animation.m.  Only the
% interface, counters, and benchmark-safe result packaging are added.

if nargin < 1
    config = struct();
end

config = normalizeRRTConfig(config);
rng(config.seed,'twister');

result = initRRTResult('Bi-RRT',config);
result.optimizationIterationsRequested = 0;
result.finalValidationCollisionChecks = 0;
result.coreCollisionChecks = 0;
result.searchCollisionChecks = 0;

Tstart.nodes = config.start;
Tstart.parent = 0;
Tgoal.nodes = config.goal;
Tgoal.parent = 0;
trees = {Tstart,Tgoal};

goalReached = false;
idxStart = 0;
idxGoal = 0;
iterationsUsed = 0;
firstSolutionIteration = NaN;
firstPath = [];
firstPathLength = NaN;
firstSolutionTime = NaN;
searchPlanningTime = 0;
collisionChecks = 0;

if config.showAnimation
    createPlannerFigure(config,'Bi-RRT');
end

if ~checkedCollision(config.start,config.start) || ...
        ~checkedCollision(config.goal,config.goal)
    result.nodeCount = 2;
    result.startTree = trees{1};
    result.goalTree = trees{2};
    result.collisionChecks = collisionChecks;
    result.coreCollisionChecks = collisionChecks;
    result.searchCollisionChecks = collisionChecks;
    return;
end

for iter = 1:config.maxIterations
    iterationsUsed = iter;
    activeTree = 1+mod(iter-1,2);
    otherTree = 3-activeTree;
    iterationTimer = tic;

    % Keep the original Bi-RRT random-call order: one uniform sample.
    xRand = sampleUniform(config);
    active = trees{activeTree};
    [active,newIndexA,addedA] = extendOne(active,xRand);
    trees{activeTree} = active;

    if ~addedA
        searchPlanningTime = searchPlanningTime+toc(iterationTimer);
        continue;
    end

    pointA = active.nodes(newIndexA,:);
    other = trees{otherTree};
    [other,newIndexB,addedB] = extendOne(other,pointA);
    trees{otherTree} = other;
    pointB = other.nodes(newIndexB,:);

    canConnect = norm(pointA-pointB) <= config.stepSize && ...
        checkedCollision(pointA,pointB);

    iterationElapsed = toc(iterationTimer);
    searchPlanningTime = searchPlanningTime+iterationElapsed;

    if config.showAnimation
        drawTreeEdge(active,activeTree,newIndexA);
        if addedB
            drawTreeEdge(other,otherTree,newIndexB);
        end
    end

    if canConnect
        goalReached = true;
        if activeTree == 1
            idxStart = newIndexA;
            idxGoal = newIndexB;
        else
            idxStart = newIndexB;
            idxGoal = newIndexA;
        end
        firstSolutionIteration = iter;
        firstPath = buildConnectionPath(trees{1},idxStart, ...
            trees{2},idxGoal);
        firstPathLength = pathLength(firstPath);
        firstSolutionTime = searchPlanningTime;
        break;
    end
end

result.iterations = iterationsUsed;
result.searchPlanningTime = searchPlanningTime;
result.planningTime = searchPlanningTime;
result.searchCollisionChecks = collisionChecks;
result.coreCollisionChecks = collisionChecks;
result.nodeCount = size(trees{1}.nodes,1)+size(trees{2}.nodes,1);
result.startTreeNodeCount = size(trees{1}.nodes,1);
result.goalTreeNodeCount = size(trees{2}.nodes,1);
result.startTree = trees{1};
result.goalTree = trees{2};

if ~goalReached
    result.collisionChecks = collisionChecks;
    if config.showAnimation
        title('Bi-RRT failed');
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
result.collisionChecks = collisionChecks+result.finalValidationCollisionChecks;
result.totalAlgorithmTime = searchPlanningTime;

if config.showAnimation
    plot(firstPath(:,1),firstPath(:,2),'b-','LineWidth',2.8);
    plot(firstPath(:,1),firstPath(:,2),'bo','MarkerFaceColor','b', ...
        'MarkerSize',4);
    title(sprintf('Bi-RRT - %.3f ms',result.planningTime*1000));
    drawnow;
end

    function [tree,newIndex,added] = extendOne(tree,target)
        added = false;
        target = reshape(target,1,2);
        [nearestIndex,~] = nearestNode(tree.nodes,target);
        xNear = tree.nodes(nearestIndex,:);
        direction = target-xNear;
        distance = norm(direction);
        newIndex = nearestIndex;
        if distance < 1e-12
            return;
        end
        moveDistance = min(config.stepSize,distance);
        xNew = xNear+(moveDistance/distance)*direction;
        if ~checkedCollision(xNear,xNew)
            return;
        end
        tree.nodes(end+1,:) = xNew;
        tree.parent(end+1,1) = nearestIndex;
        newIndex = size(tree.nodes,1);
        added = true;
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

    function drawTreeEdge(tree,treeIndex,newIndex)
        parentIndex = tree.parent(newIndex);
        if parentIndex > 0
            if treeIndex == 1
                color = [0.25 0.70 0.30];
            else
                color = [0.10 0.65 0.85];
            end
            drawPlannerEdge(tree.nodes(parentIndex,:),tree.nodes(newIndex,:), ...
                color,config.animationPause);
        end
    end

end

function path = buildConnectionPath(startTree,startIndex,goalTree,goalIndex)
pathStart = backtrackPath(startTree.nodes,startTree.parent,startIndex);
pathGoal = backtrackPath(goalTree.nodes,goalTree.parent,goalIndex);
pathGoal = flipud(pathGoal);
if norm(pathStart(end,:)-pathGoal(1,:)) < 1e-10
    pathGoal(1,:) = [];
end
path = [pathStart;pathGoal];
end
