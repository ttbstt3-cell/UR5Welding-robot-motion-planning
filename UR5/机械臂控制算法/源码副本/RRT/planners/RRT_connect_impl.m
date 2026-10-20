function result = RRT_connect_impl(config)
% RRT_connect  Standard bidirectional RRT-Connect in 2-D.
%
% The active tree takes one EXTEND step toward a random sample.  If that
% step succeeds, the other tree uses repeated EXTEND operations in a
% CONNECT loop until it reaches the new node or becomes TRAPPED.

if nargin < 1
    config = struct();
end

config = normalizeRRTConfig(config);
rng(config.seed,'twister');

timeBudgetMode = isfield(config,'timeBudgetMs') && ...
    ~isempty(config.timeBudgetMs);
if timeBudgetMode
    timeBudgetsMs = sort(reshape(double(config.timeBudgetMs),1,[]));
    if isempty(timeBudgetsMs) || any(~isfinite(timeBudgetsMs)) || ...
            any(timeBudgetsMs <= 0)
        error('config.timeBudgetMs must contain positive finite values.');
    end
    timeBudgetTimer = tic;
    nextBudgetIndex = 1;
    timeBudgetSnapshots = initTimeBudgetSnapshots(timeBudgetsMs);
else
    timeBudgetsMs = zeros(1,0);
    nextBudgetIndex = 1;
    timeBudgetSnapshots = initTimeBudgetSnapshots(timeBudgetsMs);
end

result = initRRTResult('RRT-Connect',config);
% RRT-Connect is a first-solution planner and has no optimization phase.
result.optimizationIterationsRequested = 0;
result.optimizationIterationsExecuted = 0;
collisionChecks = 0;
planningTime = 0;

if config.showAnimation
    createPlannerFigure(config,'RRT-Connect');
end

trees = cell(1,2);
trees{1} = struct('nodes',config.start,'parent',0);
trees{2} = struct('nodes',config.goal,'parent',0);

if ~checkedCollision(config.start,config.start) || ...
        ~checkedCollision(config.goal,config.goal)
    result.planningTime = planningTime;
    result.collisionChecks = collisionChecks;
    result.nodeCount = 2;
    result.timeBudgetSnapshots = timeBudgetSnapshots;
    return;
end

firstPath = [];
firstSolutionIteration = NaN;
iterationsUsed = 0;

for iter = 1:config.maxIterations
    iterationsUsed = iter;
    iterationTimer = tic;
    activeTree = 1+mod(iter-1,2);
    otherTree = 3-activeTree;
    target = sampleWithGoalBias(config);

    activeBefore = size(trees{activeTree}.nodes,1);
    [active,activeIndex,statusA] = extendTree(trees{activeTree},target);
    trees{activeTree} = active;

    solutionPath = [];
    if ~strcmp(statusA,'Trapped')
        connectionTarget = active.nodes(activeIndex,:);
        otherBefore = size(trees{otherTree}.nodes,1);
        [other,otherIndex,statusB] = connectTree( ...
            trees{otherTree},connectionTarget);
        trees{otherTree} = other;

        if strcmp(statusB,'Reached')
            if activeTree == 1
                startIndex = activeIndex;
                goalIndex = otherIndex;
            else
                startIndex = otherIndex;
                goalIndex = activeIndex;
            end

            solutionPath = buildConnectionPath( ...
                trees{1},startIndex,trees{2},goalIndex);

            if ~validateCounted(solutionPath)
                solutionPath = [];
            end
        end
    else
        otherBefore = size(trees{otherTree}.nodes,1);
    end

    iterationPlanningTime = toc(iterationTimer);
    planningTime = planningTime+iterationPlanningTime;

    if ~isempty(solutionPath) && isempty(firstPath)
        firstPath = solutionPath;
        firstSolutionIteration = iter;
        if timeBudgetMode
            result.firstSolutionTime = toc(timeBudgetTimer);
        else
            result.firstSolutionTime = planningTime;
        end
    end

    if config.showAnimation
        drawNewEdges(trees{activeTree},activeBefore, ...
            [0.25 0.70 0.30],config.animationPause);
        drawNewEdges(trees{otherTree},otherBefore, ...
            [0.10 0.65 0.85],config.animationPause);
    end

    if timeBudgetMode
        actualSnapshotTimeMs = toc(timeBudgetTimer)*1000;
        [currentSnapshots,nextBudgetIndex] = ...
            recordTimeBudgetSnapshots(timeBudgetSnapshots, ...
            nextBudgetIndex,timeBudgetsMs,actualSnapshotTimeMs,firstPath, ...
            result.firstSolutionTime,size(trees{1}.nodes,1)+ ...
            size(trees{2}.nodes,1),collisionChecks,0);
        timeBudgetSnapshots = currentSnapshots;
        if nextBudgetIndex > numel(timeBudgetsMs)
            break;
        end
    elseif ~isempty(solutionPath)
        break;
    end
end

result.iterations = iterationsUsed;
result.planningTime = planningTime;
result.collisionChecks = collisionChecks;
result.nodeCount = size(trees{1}.nodes,1)+size(trees{2}.nodes,1);
result.timeBudgetSnapshots = timeBudgetSnapshots;

if isempty(firstPath)
    result.firstSolutionTime = NaN;
    result.firstSolutionIteration = NaN;
    if config.showAnimation
        title('RRT-Connect failed');
    end
    return;
end

% RRT-Connect stops at the first feasible connection; its final path is
% therefore the first path by definition.
result.path = firstPath;
result.firstPath = firstPath;
result.pathLength = pathLength(firstPath);
result.firstPathLength = result.pathLength;
result.firstSolutionIteration = firstSolutionIteration;
postTimer = tic;
result.success = validateCounted(result.path);
planningTime = planningTime+toc(postTimer);
result.planningTime = planningTime;
result.collisionChecks = collisionChecks;

if config.showAnimation
    plot(result.path(:,1),result.path(:,2),'b-','LineWidth',2.5);
    plot(result.path(:,1),result.path(:,2),'bo','MarkerFaceColor','b','MarkerSize',4);
    title(sprintf('RRT-Connect - %.3f ms',result.planningTime*1000));
    drawnow;
end

    function free = checkedCollision(p1,p2)
        collisionChecks = collisionChecks+1;
        free = collisionFree(p1,p2,config.obstacles,config.clearance);
    end

    function point = sampleWithGoalBias(cfg)
        if rand < cfg.goalBias
            point = cfg.goal;
        else
            point = sampleUniform(cfg);
        end
    end

    function [tree,newIndex,status] = extendTree(tree,target)
        [nearestIndex,~] = nearestNode(tree.nodes,target);
        nearestPoint = tree.nodes(nearestIndex,:);
        [newPoint,reached] = steer(nearestPoint,target,config.stepSize);
        newIndex = nearestIndex;

        if norm(newPoint-nearestPoint) < 1e-12
            status = 'Reached';
            return;
        end

        if ~checkedCollision(nearestPoint,newPoint)
            status = 'Trapped';
            return;
        end

        tree.nodes(end+1,:) = newPoint;
        tree.parent(end+1,1) = nearestIndex;
        newIndex = size(tree.nodes,1);

        if reached
            status = 'Reached';
        else
            status = 'Advanced';
        end
    end

    function [tree,newIndex,status] = connectTree(tree,target)
        while true
            [tree,newIndex,status] = extendTree(tree,target);
            if ~strcmp(status,'Advanced')
                return;
            end
        end
    end

    function valid = validateCounted(candidatePath)
        valid = false;
        if isempty(candidatePath) || size(candidatePath,2) ~= 2
            return;
        end
        if norm(candidatePath(1,:)-config.start) > 1e-8 || ...
                norm(candidatePath(end,:)-config.goal) > 1e-8
            return;
        end
        for edgeIndex = 1:size(candidatePath,1)-1
            if ~checkedCollision(candidatePath(edgeIndex,:), ...
                    candidatePath(edgeIndex+1,:))
                return;
            end
        end
        valid = true;
    end

    function drawNewEdges(tree,oldCount,color,pauseTime)
        for nodeIndex = oldCount+1:size(tree.nodes,1)
            parentIndex = tree.parent(nodeIndex);
            if parentIndex > 0
                drawPlannerEdge(tree.nodes(parentIndex,:), ...
                    tree.nodes(nodeIndex,:),color,pauseTime);
            end
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

path = [pathStart; pathGoal];
end
