function result = Bi_RRT_star_impl(config)
% Bi_RRT_star_impl  Bidirectional RRT* with cost-aware tree connection.
%
% Both trees use the shared RRT* ChooseParent and Rewire operations.  The
% active tree alternates between the start and goal trees.  After each new
% active-tree node, all nearby nodes in the other tree are considered as
% connection candidates and the least-cost feasible pair is retained.

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

result = initRRTResult('Bi-RRT*',config);
collisionChecks = 0;
rewireCount = 0;
planningTime = 0;

if config.showAnimation
    createPlannerFigure(config,'Bi-RRT*');
end

Ts.nodes = config.start;
Ts.parent = 0;
Ts.cost = 0;

Tg.nodes = config.goal;
Tg.parent = 0;
Tg.cost = 0;

trees = {Ts,Tg};

bestConnection = [NaN NaN];
bestTotalCost = Inf;
firstPath = [];
firstSolutionIteration = NaN;
optimizationStop = config.maxIterations;
iterationsUsed = 0;

bestCostHistory = NaN(config.maxIterations,1);

if ~checkedCollision(config.start,config.start) || ...
        ~checkedCollision(config.goal,config.goal)
    result.planningTime = planningTime;
    result.collisionChecks = collisionChecks;
    result.nodeCount = 2;
    result.startTreeNodeCount = 1;
    result.goalTreeNodeCount = 1;
    result.startTree = trees{1};
    result.goalTree = trees{2};
    result.bestConnection = bestConnection;
    result.timeBudgetSnapshots = timeBudgetSnapshots;
    return;
end

for iter = 1:config.maxIterations
    iterationsUsed = iter;
    iterationTimer = tic;

    activeTree = 1+mod(iter-1,2);
    otherTree = 3-activeTree;
    target = sampleWithGoalBias(config);

    activeBefore = size(trees{activeTree}.nodes,1);
    [active,newIndex,addedThisIteration,rewiredThisIteration] = ...
        extendRRTStar(trees{activeTree},target);
    trees{activeTree} = active;
    rewireCount = rewireCount+rewiredThisIteration;

    candidateConnection = [NaN NaN];
    candidateCost = Inf;

    if addedThisIteration
        xNew = active.nodes(newIndex,:);
        other = trees{otherTree};

        nAfterConnectionInsert = size(other.nodes,1)+1;
        connectionRadius = min( ...
            config.nearGamma*sqrt(log(max(2,nAfterConnectionInsert))/ ...
            nAfterConnectionInsert), ...
            config.nearRadiusMax);

        candidateIndices = nearNodes( ...
            other.nodes,xNew,connectionRadius);

        if isempty(candidateIndices)
            [nearestOther,~] = nearestNode(other.nodes,xNew);
            candidateIndices = nearestOther;
        end

        for candidateIndex = reshape(candidateIndices,1,[])
            candidatePoint = other.nodes(candidateIndex,:);

            if ~checkedCollision(xNew,candidatePoint)
                continue;
            end

            total = connectionCost( ...
                trees{1},newIndex,trees{2},candidateIndex,activeTree);

            if total < candidateCost-1e-12
                candidateCost = total;
                if activeTree == 1
                    candidateConnection = [newIndex candidateIndex];
                else
                    candidateConnection = [candidateIndex newIndex];
                end
            end
        end
    end

    % Rewiring can improve the cost of a previously stored connection pair.
    % Re-evaluate that pair before comparing a new connection candidate.
    if all(isfinite(bestConnection))
        currentBestCost = connectionCost( ...
            trees{1},bestConnection(1),trees{2},bestConnection(2),1);
        bestTotalCost = min(bestTotalCost,currentBestCost);
    end

    firstPathThisIteration = [];
    foundFirstThisIteration = false;

    if all(isfinite(candidateConnection)) && ...
            candidateCost < bestTotalCost-1e-12
        bestConnection = candidateConnection;
        bestTotalCost = candidateCost;
        candidatePath = buildConnectionPath( ...
            trees{1},bestConnection(1),trees{2},bestConnection(2));

        if isempty(firstPath)
            if validateCounted(candidatePath)
                firstPathThisIteration = candidatePath;
                foundFirstThisIteration = true;
                firstSolutionIteration = iter;
                if ~timeBudgetMode
                    optimizationStop = min( ...
                        config.maxIterations, ...
                        firstSolutionIteration+config.optimizeIterations);
                end
            end
        end
    end

    bestCostHistory(iter) = bestTotalCost;

    iterationPlanningTime = toc(iterationTimer);
    planningTime = planningTime+iterationPlanningTime;

    if foundFirstThisIteration
        firstPath = firstPathThisIteration;
        if timeBudgetMode
            result.firstSolutionTime = toc(timeBudgetTimer);
        else
            result.firstSolutionTime = planningTime;
        end
        result.collisionChecksAtFirstSolution = collisionChecks;
    end

    if config.showAnimation
        drawNewEdges(trees{activeTree},activeBefore, ...
            activeTreeColor(activeTree),config.animationPause);
    end

    if timeBudgetMode
        snapshotPath = [];
        actualSnapshotTimeMs = toc(timeBudgetTimer)*1000;
        checkpointDue = nextBudgetIndex <= numel(timeBudgetsMs) && ...
            actualSnapshotTimeMs >= timeBudgetsMs(nextBudgetIndex);
        if checkpointDue && all(isfinite(bestConnection))
            candidateSnapshotPath = buildConnectionPath( ...
                trees{1},bestConnection(1),trees{2},bestConnection(2));
            if validateCounted(candidateSnapshotPath)
                snapshotPath = candidateSnapshotPath;
            end
        end
        if isempty(snapshotPath) && ~isempty(firstPath)
            snapshotPath = firstPath;
        end
        [currentSnapshots,nextBudgetIndex] = ...
            recordTimeBudgetSnapshots(timeBudgetSnapshots, ...
            nextBudgetIndex,timeBudgetsMs,actualSnapshotTimeMs,snapshotPath, ...
            result.firstSolutionTime,size(trees{1}.nodes,1)+ ...
            size(trees{2}.nodes,1),collisionChecks,rewireCount);
        timeBudgetSnapshots = currentSnapshots;
        if nextBudgetIndex > numel(timeBudgetsMs)
            break;
        end
    elseif ~isempty(firstPath) && iter >= optimizationStop
        break;
    end
end

result.iterations = iterationsUsed;
result.iterationHistory = (1:iterationsUsed).';
result.bestCostHistory = bestCostHistory(1:iterationsUsed);
result.planningTime = planningTime;
result.collisionChecks = collisionChecks;
result.rewireCount = rewireCount;
result.nodeCount = size(trees{1}.nodes,1)+size(trees{2}.nodes,1);
result.startTreeNodeCount = size(trees{1}.nodes,1);
result.goalTreeNodeCount = size(trees{2}.nodes,1);
result.startTree = trees{1};
result.goalTree = trees{2};
result.bestConnection = bestConnection;
result.bestCost = bestTotalCost;
result.timeBudgetSnapshots = timeBudgetSnapshots;

if isnan(firstSolutionIteration)
    result.optimizationIterationsExecuted = 0;
else
    result.optimizationIterationsExecuted = max( ...
        0,result.iterations-firstSolutionIteration);
end

if isempty(firstPath) || ~all(isfinite(bestConnection))
    result.firstSolutionTime = NaN;
    result.firstSolutionIteration = NaN;
    if config.showAnimation
        title('Bi-RRT* failed');
    end
    return;
end

% Reconstruct after all optimization because rewiring can change either
% branch below a stored connection node.
postTimer = tic;
finalPath = buildConnectionPath( ...
    trees{1},bestConnection(1),trees{2},bestConnection(2));
result.success = validateCounted(finalPath);
planningTime = planningTime+toc(postTimer);
result.planningTime = planningTime;

if ~result.success
    result.path = [];
    result.pathLength = NaN;
else
    result.path = finalPath;
    result.pathLength = pathLength(finalPath);
end

result.firstPath = firstPath;
result.firstPathLength = pathLength(firstPath);
result.firstSolutionIteration = firstSolutionIteration;
result.collisionChecks = collisionChecks;
result.startTree = trees{1};
result.goalTree = trees{2};
result.startTreeNodeCount = size(trees{1}.nodes,1);
result.goalTreeNodeCount = size(trees{2}.nodes,1);
result.nodeCount = result.startTreeNodeCount+result.goalTreeNodeCount;

if config.showAnimation && result.success
    plot(firstPath(:,1),firstPath(:,2),'--', ...
        'Color',[0.35 0.35 0.35],'LineWidth',1.2);
    plot(result.path(:,1),result.path(:,2),'b-','LineWidth',2.8);
    plot(result.path(:,1),result.path(:,2),'bo', ...
        'MarkerFaceColor','b','MarkerSize',4);
    title(sprintf('Bi-RRT* - %.3f ms',result.planningTime*1000));
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

    function [tree,newIndex,added,rewired] = extendRRTStar(tree,target)
        added = false;
        rewired = 0;
        [nearestIndex,~] = nearestNode(tree.nodes,target);
        nearestPoint = tree.nodes(nearestIndex,:);
        [newPoint,~] = steer(nearestPoint,target,config.stepSize);
        newIndex = nearestIndex;

        if norm(newPoint-nearestPoint) < 1e-12 || ...
                ~checkedCollision(nearestPoint,newPoint)
            return;
        end

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
        added = true;

        [tree,rewired] = rewireRRTStar( ...
            tree,newIndex,candidates,@checkedCollision);
    end

    function total = connectionCost(startTree,startIndex,goalTree,goalIndex,active)
        if active == 1
            total = startTree.cost(startIndex)+ ...
                norm(startTree.nodes(startIndex,:)-goalTree.nodes(goalIndex,:))+ ...
                goalTree.cost(goalIndex);
        else
            total = startTree.cost(goalIndex)+ ...
                norm(startTree.nodes(goalIndex,:)-goalTree.nodes(startIndex,:))+ ...
                goalTree.cost(startIndex);
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

    function color = activeTreeColor(treeIndex)
        if treeIndex == 1
            color = [0.25 0.70 0.30];
        else
            color = [0.10 0.65 0.85];
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
