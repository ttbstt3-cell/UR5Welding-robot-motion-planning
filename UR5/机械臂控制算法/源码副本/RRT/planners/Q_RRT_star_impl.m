function result = Q_RRT_star_impl(config)
% Q_RRT_star_impl  Quick-RRT* with configurable ancestor depth.
%
% This Phase 6 implementation keeps uniform sampling and the standard
% Euclidean path cost.  Its only algorithmic additions are ancestor-aware
% ChooseParent and the two-candidate Quick-Rewire operation.

if nargin < 1
    config = struct();
end

config = normalizeRRTConfig(config);
rng(config.seed,'twister');

result = initRRTResult('Q-RRT*',config);
result.qDepth = config.qDepth;
result.ancestorCandidateCount = 0;
result.ancestorChecks = 0;
result.ancestorAcceptedCount = 0;
result.bestCostHistory = [];
result.iterationHistory = [];

collisionChecks = 0;
rewireCount = 0;
planningTime = 0;

if config.showAnimation
    createPlannerFigure(config,'Q-RRT*');
end

tree.nodes = config.start;
tree.parent = 0;
tree.cost = 0;

if ~checkedCollision(config.start,config.start) || ...
        ~checkedCollision(config.goal,config.goal)
    result.planningTime = planningTime;
    result.collisionChecks = collisionChecks;
    result.nodeCount = 1;
    result.tree = tree;
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
    nodeAdded = false;
    solutionFoundThisIteration = false;
    candidateFirstPath = [];

    % Q-RRT* deliberately uses only global uniform sampling.  In
    % particular, config.goalBias is not used in this planner.
    sample = sampleUniform(config);
    [nearestIndex,~] = nearestNode(tree.nodes,sample);
    nearestPoint = tree.nodes(nearestIndex,:);
    [newPoint,~] = steer(nearestPoint,sample,config.stepSize);

    if norm(newPoint-nearestPoint) >= 1e-12 && ...
            checkedCollision(nearestPoint,newPoint)
        nodeAdded = true;
        nAfterInsert = size(tree.nodes,1)+1;
        radius = min( ...
            config.nearGamma*sqrt(log(max(2,nAfterInsert))/nAfterInsert), ...
            config.nearRadiusMax);
        nearIndices = nearNodes(tree.nodes,newPoint,radius);

        [bestParent,bestCost,parentStats] = chooseParentQRRTStar( ...
            tree,nearestIndex,nearIndices,newPoint,config.qDepth, ...
            @checkedCollision);

        result.ancestorCandidateCount = result.ancestorCandidateCount+ ...
            parentStats.ancestorCandidateCount;
        result.ancestorChecks = result.ancestorChecks+ ...
            parentStats.ancestorChecks;
        result.ancestorAcceptedCount = result.ancestorAcceptedCount+ ...
            parentStats.ancestorAcceptedCount;

        tree.nodes(end+1,:) = newPoint;
        tree.parent(end+1,1) = bestParent;
        tree.cost(end+1,1) = bestCost;
        newIndex = size(tree.nodes,1);

        if isempty(nearIndices)
            rewireIndices = nearestIndex;
        else
            rewireIndices = nearIndices;
        end

        [tree,rewiredThisIteration] = rewireQRRTStar( ...
            tree,newIndex,rewireIndices,@checkedCollision);
        rewireCount = rewireCount+rewiredThisIteration;

        % Rewiring may improve the node currently connected to the goal.
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
                        optimizationStop = min( ...
                            config.maxIterations, ...
                            firstSolutionIteration+config.optimizeIterations);
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
            tree.nodes(newIndex,:),[0.55 0.25 0.75],config.animationPause);
        plot(tree.nodes(newIndex,1),tree.nodes(newIndex,2),'.', ...
            'Color',[0.40 0.15 0.55],'MarkerSize',6);
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
result.tree = tree;

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
        title('Q-RRT* failed');
    end
    return;
end

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
    plot(firstPath(:,1),firstPath(:,2),'--', ...
        'Color',[0.35 0.35 0.35],'LineWidth',1.2);
    plot(result.path(:,1),result.path(:,2),'b-','LineWidth',2.8);
    plot(result.path(:,1),result.path(:,2),'bo', ...
        'MarkerFaceColor','b','MarkerSize',4);
    title(sprintf('Q-RRT* - %.3f ms',result.planningTime*1000));
    drawnow;
end

    function free = checkedCollision(p1,p2)
        collisionChecks = collisionChecks+1;
        free = collisionFree(p1,p2,config.obstacles,config.clearance);
    end

    function candidatePath = buildGoalPath(currentTree,goalNode)
        candidatePath = backtrackPath( ...
            currentTree.nodes,currentTree.parent,goalNode);
        if norm(candidatePath(end,:)-config.goal) > 1e-10
            candidatePath = [candidatePath;config.goal];
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
