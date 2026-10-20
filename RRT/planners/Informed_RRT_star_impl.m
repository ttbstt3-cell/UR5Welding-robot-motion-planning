function result = Informed_RRT_star_impl(config)
% Informed_RRT_star_impl  Standard Informed RRT* in 2-D.
%
% Before the first solution this follows the same sampling and RRT* tree
% operations as RRT_star_impl.  Once c_best is finite, sampling switches to
% the directly sampled prolate ellipse in sampleInformedEllipse.m.

if nargin < 1
    config = struct();
end

config = normalizeRRTConfig(config);
rng(config.seed,'twister');

result = initRRTResult('Informed RRT*',config);
result.uniformSampleCount = 0;
result.informedSampleCount = 0;
result.informedSampleRejectCount = 0;
result.cMin = norm(config.goal-config.start);
result.finalCBest = NaN;
result.bestCostHistory = [];
result.iterationHistory = [];

collisionChecks = 0;
rewireCount = 0;
planningTime = 0;
ellipseTolerance = 1e-10;
maxEllipseRetries = 100;

if config.showAnimation
    createPlannerFigure(config,'Informed RRT*');
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
cBest = Inf;
iterationsUsed = 0;
optimizationStop = config.maxIterations;
bestCostHistory = NaN(config.maxIterations,1);

for iter = 1:config.maxIterations
    iterationsUsed = iter;
    iterationTimer = tic;
    nodeAdded = false;
    solutionFoundThisIteration = false;
    candidateFirstPath = [];

    if isempty(firstPath)
        sample = sampleWithGoalBias(config);
        result.uniformSampleCount = result.uniformSampleCount+1;
    else
        [sample,usedInformed,rejectCount] = sampleInformedEllipse( ...
            config.start,config.goal,cBest,config.mapBounds, ...
            maxEllipseRetries,ellipseTolerance);
        result.informedSampleRejectCount = ...
            result.informedSampleRejectCount+rejectCount;
        if usedInformed
            result.informedSampleCount = result.informedSampleCount+1;
        else
            % This only occurs for a numerically degenerate ellipse, where
            % the sampler deliberately falls back to its stable center.
            result.uniformSampleCount = result.uniformSampleCount+1;
        end
    end

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
        candidates = unique([nearestIndex;nearNodes(tree.nodes,newPoint,radius)]);
        [bestParent,bestCost] = chooseParentRRTStar( ...
            tree,candidates,newPoint,nearestIndex,@checkedCollision);

        tree.nodes(end+1,:) = newPoint;
        tree.parent(end+1,1) = bestParent;
        tree.cost(end+1,1) = bestCost;
        newIndex = size(tree.nodes,1);

        [tree,rewiredThisIteration] = rewireRRTStar( ...
            tree,newIndex,candidates,@checkedCollision);
        rewireCount = rewireCount+rewiredThisIteration;

        % A rewire can improve the node currently connected to the goal.
        if ~isnan(bestGoalNode)
            rewiredGoalCost = tree.cost(bestGoalNode)+ ...
                norm(tree.nodes(bestGoalNode,:)-config.goal);
            if rewiredGoalCost < cBest-1e-12
                cBest = rewiredGoalCost;
            end
        end

        distanceToGoal = norm(newPoint-config.goal);
        if distanceToGoal <= config.goalThreshold && ...
                checkedCollision(newPoint,config.goal)
            candidateGoalCost = tree.cost(newIndex)+distanceToGoal;

            if candidateGoalCost < cBest-1e-12
                bestGoalNode = newIndex;
                cBest = candidateGoalCost;
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

    bestCostHistory(iter) = cBest;
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
        if ~isempty(firstPath) && isfinite(cBest)
            drawCurrentEllipse(config.start,config.goal,cBest);
        end
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
        title('Informed RRT* failed');
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
    result.finalCBest = NaN;
else
    result.pathLength = pathLength(result.path);
    result.finalCBest = result.pathLength;
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
    title(sprintf('Informed RRT* - %.3f ms',result.planningTime*1000));
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

    function drawCurrentEllipse(startPoint,goalPoint,currentBest)
        cMinLocal = norm(goalPoint-startPoint);
        if cMinLocal <= ellipseTolerance || ...
                currentBest <= cMinLocal+ellipseTolerance
            return;
        end
        aLocal = currentBest/2;
        bLocal = sqrt(max(currentBest^2-cMinLocal^2,0))/2;
        directionLocal = (goalPoint-startPoint)/cMinLocal;
        rotationLocal = [directionLocal(:), ...
            [-directionLocal(2);directionLocal(1)]];
        angles = linspace(0,2*pi,120).';
        localEllipse = [aLocal*cos(angles),bLocal*sin(angles)].';
        ellipsePoints = rotationLocal*localEllipse + ...
            ((startPoint+goalPoint)/2).';
        plot(ellipsePoints(1,:),ellipsePoints(2,:),'m--','LineWidth',0.8);
    end

end
