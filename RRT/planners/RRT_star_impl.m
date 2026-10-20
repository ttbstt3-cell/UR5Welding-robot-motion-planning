function result = RRT_star_impl(config)
% RRT_star  Standard asymptotically optimal RRT* in 2-D.
%
% The planner keeps going after the first feasible path.  By default it
% spends config.optimizeIterations additional iterations after the first
% solution, capped by config.maxIterations.  Set this field explicitly for
% a fixed optimization budget in benchmark experiments.

if nargin < 1
    config = struct();
end

config = normalizeRRTConfig(config);
rng(config.seed,'twister');

result = initRRTResult('RRT*',config);
collisionChecks = 0;
rewireCount = 0;
planningTime = 0;
result.bestCostHistory = [];
result.iterationHistory = [];

if config.showAnimation
    createPlannerFigure(config,'RRT*');
end

tree.nodes = config.start;
tree.parent = 0;
tree.cost = 0;

if ~checkedCollision(config.start,config.start) || ...
        ~checkedCollision(config.goal,config.goal)
    result.planningTime = planningTime;
    result.collisionChecks = collisionChecks;
    result.nodeCount = 1;
    return;
end

firstPath = [];
firstSolutionIteration = NaN;
bestGoalNode = NaN;
bestGoalCost = Inf;
iterationsUsed = 0;
optimizationStop = config.maxIterations;
bestCostHistory = NaN(config.maxIterations,1);

for iter = 1:config.maxIterations
    iterationsUsed = iter;
    iterationTimer = tic;
    sample = sampleWithGoalBias(config);
    [nearestIndex,~] = nearestNode(tree.nodes,sample);
    nearestPoint = tree.nodes(nearestIndex,:);
    [newPoint,~] = steer(nearestPoint,sample,config.stepSize);
    nodeAdded = false;

    solutionFoundThisIteration = false;
    candidateFirstPath = [];

    if norm(newPoint-nearestPoint) >= 1e-12 && ...
            checkedCollision(nearestPoint,newPoint)
        nAfterInsert = size(tree.nodes,1)+1;
        radius = min( ...
            config.nearGamma*sqrt(log(max(2,nAfterInsert))/nAfterInsert), ...
            config.nearRadiusMax);

        candidates = unique([nearestIndex; nearNodes(tree.nodes,newPoint,radius)]);
        [bestParent,bestCost] = chooseParentRRTStar( ...
            tree,candidates,newPoint,nearestIndex,@checkedCollision);

        tree.nodes(end+1,:) = newPoint;
        tree.parent(end+1,1) = bestParent;
        tree.cost(end+1,1) = bestCost;
        newIndex = size(tree.nodes,1);
        nodeAdded = true;

        [tree,rewiredThisIteration] = rewireRRTStar( ...
            tree,newIndex,candidates,@checkedCollision);
        rewireCount = rewireCount+rewiredThisIteration;

        % A rewire can improve the node currently connected to the goal.
        if ~isnan(bestGoalNode)
            rewiredGoalCost = tree.cost(bestGoalNode)+ ...
                norm(tree.nodes(bestGoalNode,:)-config.goal);
            bestGoalCost = min(bestGoalCost,rewiredGoalCost);
        end

        distanceToGoal = norm(newPoint-config.goal);
        if distanceToGoal <= config.goalThreshold && ...
                checkedCollision(newPoint,config.goal)
            candidateGoalCost = tree.cost(newIndex)+distanceToGoal;
            if candidateGoalCost < bestGoalCost-1e-12
                bestGoalNode = newIndex;
                bestGoalCost = candidateGoalCost;
                candidatePath = buildGoalPath(tree,newIndex);

                if isempty(firstPath)
                    if validateCounted(candidatePath)
                        candidateFirstPath = candidatePath;
                        solutionFoundThisIteration = true;
                        firstSolutionIteration = iter;
                        % The requested budget is additional iterations
                        % after the first feasible solution iteration.
                        optimizationStop = min( ...
                            config.maxIterations, ...
                            firstSolutionIteration+ ...
                            config.optimizeIterations);
                    end
                end
            end
        end
    end

    bestCostHistory(iter) = bestGoalCost;

    iterationPlanningTime = toc(iterationTimer);
    planningTime = planningTime+iterationPlanningTime;

    if solutionFoundThisIteration
        firstPath = candidateFirstPath;
        result.firstSolutionTime = planningTime;
        result.collisionChecksAtFirstSolution = collisionChecks;
    end

    if config.showAnimation && nodeAdded
        drawPlannerEdge(tree.nodes(tree.parent(newIndex),:), ...
            tree.nodes(newIndex,:),[0.35 0.70 0.35],config.animationPause);
        plot(tree.nodes(newIndex,1),tree.nodes(newIndex,2),'.', ...
            'Color',[0.15 0.50 0.15],'MarkerSize',6);
        drawnow;
    end

    if ~isempty(firstPath) && iter >= optimizationStop
        break;
    end
end

result.iterations = iterationsUsed;
result.iterationHistory = (1:iterationsUsed).';
result.bestCostHistory = bestCostHistory(1:iterationsUsed);
result.planningTime = planningTime;
result.collisionChecks = collisionChecks;
result.rewireCount = rewireCount;
result.nodeCount = size(tree.nodes,1);
if isnan(firstSolutionIteration)
    result.optimizationIterationsExecuted = 0;
else
    result.optimizationIterationsExecuted = max( ...
        0,result.iterations-firstSolutionIteration);
end

if isempty(firstPath) || isnan(bestGoalNode)
    result.firstSolutionTime = NaN;
    result.firstSolutionIteration = NaN;
    if config.showAnimation
        title('RRT* failed');
    end
    return;
end

% Build the best path after the optimization phase.  Rewiring may have
% changed the parent chain of bestGoalNode since firstPath was recorded.
postTimer = tic;
result.path = buildGoalPath(tree,bestGoalNode);
result.success = validateCounted(result.path);
planningTime = planningTime+toc(postTimer);
result.planningTime = planningTime;

if ~result.success
    result.path = [];
    result.pathLength = NaN;
else
    result.pathLength = pathLength(result.path);
end

result.firstPath = firstPath;
result.firstPathLength = pathLength(firstPath);
result.firstSolutionIteration = firstSolutionIteration;
result.collisionChecks = collisionChecks;

if config.showAnimation && result.success
    plot(result.path(:,1),result.path(:,2),'b-','LineWidth',2.5);
    plot(result.path(:,1),result.path(:,2),'bo','MarkerFaceColor','b','MarkerSize',4);
    title(sprintf('RRT* - %.3f ms',result.planningTime*1000));
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

    function candidatePath = buildGoalPath(currentTree,goalNode)
        candidatePath = backtrackPath( ...
            currentTree.nodes,currentTree.parent,goalNode);
        if norm(candidatePath(end,:)-config.goal) > 1e-10
            candidatePath = [candidatePath; config.goal];
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

end
