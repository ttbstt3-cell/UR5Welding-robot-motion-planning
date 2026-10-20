function result = APF_Bi_RRT_star_impl(config)
% APF_Bi_RRT_star_impl  Improved APF-Bi-RRT* with post-processing.
%
% The search is an independent bidirectional RRT* implementation.  Dynamic
% target bias chooses the sample used for Nearest; the improved APF force at
% xNear alone determines xNew in paper mode.  Greedy simplification and
% clamped B-spline smoothing are measured as post-processing.

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

if isfield(config,'ablationUseDynamicGoalBias')
    useDynamicGoalBias = logical(config.ablationUseDynamicGoalBias);
else
    useDynamicGoalBias = true;
end
if isfield(config,'ablationUseAPFForce')
    useAPFForce = logical(config.ablationUseAPFForce);
else
    useAPFForce = true;
end

result = initRRTResult('APF-Bi-RRT*',config);
if config.apfPaperMode
    result.optimizationIterationsRequested = 0;
end
[goalBiasThreshold,biasDetails] = computeDynamicGoalBias(config);
if ~useDynamicGoalBias
    goalBiasThreshold = config.goalBias;
end
result.goalBiasThreshold = goalBiasThreshold;
result.obstacleMeasure = biasDetails.obstacleMeasure;
result.workspaceMeasure = biasDetails.workspaceMeasure;
result.apfPaperMode = config.apfPaperMode;
result.apfPaperConnectionMode = config.apfPaperConnectionMode;
if config.apfPaperMode
    result.apfActualConnectionMode = 'nearest';
else
    result.apfActualConnectionMode = config.apfPaperConnectionMode;
end
result.apfGoalBiasExponent = config.apfGoalBiasExponent;
result.apfRepulsiveExponent = config.apfRepulsiveExponent;
result.ablationUseDynamicGoalBias = useDynamicGoalBias;
result.ablationUseAPFForce = useAPFForce;
result.goalBiasedSampleCount = 0;
result.uniformSampleCount = 0;
result.apfZeroForceCount = 0;
result.apfActiveObstacleCount = 0;
result.searchCollisionChecks = 0;
result.postProcessCollisionChecks = 0;
result.searchPlanningTime = 0;
result.postProcessTime = 0;
result.totalAlgorithmTime = 0;
result.rawPath = [];
result.rawPathLength = NaN;
result.simplifiedPath = [];
result.simplifiedPathLength = NaN;
result.smoothedPath = [];
result.smoothedPathLength = NaN;
result.smoothedPathValid = false;
result.rawNodeCount = 0;
result.simplifiedNodeCount = 0;
result.smoothedSampleCount = 0;
result.simplificationTime = 0;
result.smoothingTime = 0;
result.bestCostHistory = zeros(0,1);
result.iterationHistory = zeros(0,1);
result.bestConnection = [NaN NaN];
result.bestCost = Inf;

searchCollisionChecks = 0;
postProcessCollisionChecks = 0;
rewireCount = 0;
searchPlanningTime = 0;

if config.showAnimation
    createPlannerFigure(config,'APF-Bi-RRT*');
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
    result.searchPlanningTime = searchPlanningTime;
    result.totalAlgorithmTime = searchPlanningTime;
    result.planningTime = searchPlanningTime;
    result.searchCollisionChecks = searchCollisionChecks;
    result.collisionChecks = searchCollisionChecks;
    result.nodeCount = 2;
    result.startTreeNodeCount = 1;
    result.goalTreeNodeCount = 1;
    result.startTree = trees{1};
    result.goalTree = trees{2};
    result.timeBudgetSnapshots = timeBudgetSnapshots;
    return;
end

for iter = 1:config.maxIterations
    iterationsUsed = iter;
    iterationTimer = tic;
    activeTree = 1+mod(iter-1,2);
    otherTree = 3-activeTree;
    target = trees{otherTree}.nodes(1,:);

    if rand <= goalBiasThreshold
        xRand = target;
        result.goalBiasedSampleCount = result.goalBiasedSampleCount+1;
    else
        xRand = sampleUniform(config);
        result.uniformSampleCount = result.uniformSampleCount+1;
    end

    activeBefore = size(trees{activeTree}.nodes,1);
    [active,newIndex,addedThisIteration,rewiredThisIteration] = ...
        extendAPFTree(trees{activeTree},xRand,target);
    trees{activeTree} = active;
    rewireCount = rewireCount+rewiredThisIteration;

    candidateConnection = [NaN NaN];
    candidateCost = Inf;
    if addedThisIteration
        xNew = active.nodes(newIndex,:);
        [candidateConnection,candidateCost] = findConnection( ...
            trees{1},trees{2},newIndex,activeTree);
    end

    % Rewiring may improve a previously stored connection pair.
    if all(isfinite(bestConnection))
        bestTotalCost = connectionCost( ...
            trees{1},bestConnection(1),trees{2},bestConnection(2));
    end

    foundFirstThisIteration = false;
    firstPathThisIteration = [];
    if all(isfinite(candidateConnection)) && ...
            candidateCost < bestTotalCost-1e-12
        bestConnection = candidateConnection;
        bestTotalCost = candidateCost;
        candidatePath = buildConnectionPath( ...
            trees{1},bestConnection(1),trees{2},bestConnection(2));

        if isempty(firstPath) && validateSearchPath(candidatePath)
            firstPathThisIteration = candidatePath;
            foundFirstThisIteration = true;
            firstSolutionIteration = iter;
            if config.apfPaperMode
                optimizationStop = iter;
            elseif ~timeBudgetMode
                optimizationStop = min(config.maxIterations, ...
                    firstSolutionIteration+config.optimizeIterations);
            end
        end
    end

    bestCostHistory(iter) = bestTotalCost;
    iterationPlanningTime = toc(iterationTimer);
    searchPlanningTime = searchPlanningTime+iterationPlanningTime;

    if foundFirstThisIteration
        firstPath = firstPathThisIteration;
        if timeBudgetMode
            result.firstSolutionTime = toc(timeBudgetTimer);
        else
            result.firstSolutionTime = searchPlanningTime;
        end
        result.collisionChecksAtFirstSolution = searchCollisionChecks;
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
            if validateSearchPath(candidateSnapshotPath)
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
            size(trees{2}.nodes,1),searchCollisionChecks,rewireCount);
        timeBudgetSnapshots = currentSnapshots;
        if nextBudgetIndex > numel(timeBudgetsMs)
            break;
        end
    elseif ~isempty(firstPath) && ...
            (config.apfPaperMode || iter >= optimizationStop)
        break;
    end
end

result.iterations = iterationsUsed;
result.iterationHistory = (1:iterationsUsed).';
result.bestCostHistory = bestCostHistory(1:iterationsUsed);
result.searchPlanningTime = searchPlanningTime;
result.planningTime = searchPlanningTime;
result.searchCollisionChecks = searchCollisionChecks;
result.collisionChecks = searchCollisionChecks;
result.rewireCount = rewireCount;
result.nodeCount = size(trees{1}.nodes,1)+size(trees{2}.nodes,1);
result.startTreeNodeCount = size(trees{1}.nodes,1);
result.goalTreeNodeCount = size(trees{2}.nodes,1);
result.startTree = trees{1};
result.goalTree = trees{2};
result.bestConnection = bestConnection;
result.bestCost = bestTotalCost;
result.timeBudgetSnapshots = timeBudgetSnapshots;

if isnan(firstSolutionIteration) || ~all(isfinite(bestConnection))
    result.firstSolutionTime = NaN;
    result.firstSolutionIteration = NaN;
    result.optimizationIterationsExecuted = 0;
    result.totalAlgorithmTime = searchPlanningTime;
    if config.showAnimation
        title('APF-Bi-RRT* failed');
    end
    return;
end

if config.apfPaperMode
    result.optimizationIterationsExecuted = 0;
else
    result.optimizationIterationsExecuted = max( ...
        0,result.iterations-firstSolutionIteration);
end

if timeBudgetMode
    rawPath = buildConnectionPath( ...
        trees{1},bestConnection(1),trees{2},bestConnection(2));
    result.success = validateSearchPath(rawPath);
    result.path = rawPath;
    result.firstPath = firstPath;
    result.firstPathLength = pathLength(firstPath);
    result.firstSolutionIteration = firstSolutionIteration;
    if result.success
        result.pathLength = pathLength(rawPath);
        result.rawPath = rawPath;
        result.rawPathLength = result.pathLength;
    else
        result.path = [];
        result.pathLength = NaN;
    end
    result.collisionChecks = searchCollisionChecks;
    result.totalAlgorithmTime = searchPlanningTime;
    result.planningTime = searchPlanningTime;
    return;
end

% Search is complete.  Every operation below is post-processing and uses a
% separate collision counter.
postProcessTimer = tic;
rawPath = buildConnectionPath( ...
    trees{1},bestConnection(1),trees{2},bestConnection(2));
rawPathLength = pathLength(rawPath);

simplifyTimer = tic;
[simplifiedPath,~] = greedySimplifyPath(rawPath,@postCollision);
simplificationTime = toc(simplifyTimer);

smoothingTimer = tic;
[smoothedPath,actualDegree] = generateBSplinePath( ...
    simplifiedPath,config.bsplineDegree,config.bsplineSamples);
[smoothedValid,~] = validateSmoothedPath( ...
    smoothedPath,config,@postCollision);
smoothingTime = toc(smoothingTimer);

result.rawPath = rawPath;
result.rawPathLength = rawPathLength;
result.simplifiedPath = simplifiedPath;
result.simplifiedPathLength = pathLength(simplifiedPath);
result.smoothedPath = smoothedPath;
result.smoothedPathLength = pathLength(smoothedPath);
result.smoothedPathValid = smoothedValid;
result.smoothedSampleCount = size(smoothedPath,1);
result.rawNodeCount = size(rawPath,1);
result.simplifiedNodeCount = size(simplifiedPath,1);
result.bsplineDegreeRequested = config.bsplineDegree;
result.bsplineDegreeUsed = actualDegree;
result.simplificationTime = simplificationTime;
result.smoothingTime = smoothingTime;
result.postProcessTime = toc(postProcessTimer);
result.postProcessCollisionChecks = postProcessCollisionChecks;
result.collisionChecks = searchCollisionChecks+postProcessCollisionChecks;
result.totalAlgorithmTime = searchPlanningTime+result.postProcessTime;
result.planningTime = result.totalAlgorithmTime;

% A collision-free smoothed curve is the final path.  Otherwise retain the
% validated simplified polyline and expose the invalid curve diagnostically.
if smoothedValid
    result.path = smoothedPath;
    result.pathLength = result.smoothedPathLength;
else
    result.path = simplifiedPath;
    result.pathLength = result.simplifiedPathLength;
end
result.success = true;
result.firstPath = firstPath;
result.firstPathLength = pathLength(firstPath);
result.firstSolutionIteration = firstSolutionIteration;

if config.showAnimation
    plot(firstPath(:,1),firstPath(:,2),'--', ...
        'Color',[0.35 0.35 0.35],'LineWidth',1.2);
    plot(rawPath(:,1),rawPath(:,2),'-','Color',[0.10 0.45 0.85], ...
        'LineWidth',1.0);
    plot(result.path(:,1),result.path(:,2),'b-','LineWidth',2.8);
    plot(result.path(:,1),result.path(:,2),'bo', ...
        'MarkerFaceColor','b','MarkerSize',4);
    title(sprintf('APF-Bi-RRT* - %.3f ms',result.totalAlgorithmTime*1000));
    drawnow;
end

    function free = checkedCollision(p1,p2)
        searchCollisionChecks = searchCollisionChecks+1;
        free = collisionFree(p1,p2,config.obstacles,config.clearance);
    end

    function free = postCollision(p1,p2)
        postProcessCollisionChecks = postProcessCollisionChecks+1;
        free = collisionFree(p1,p2,config.obstacles,config.clearance);
    end

    function [tree,newIndex,added,rewired] = extendAPFTree( ...
            tree,randomPoint,apfTarget)
        added = false;
        rewired = 0;
        [nearestIndex,~] = nearestNode(tree.nodes,randomPoint);
        nearestPoint = tree.nodes(nearestIndex,:);
        if useAPFForce
            [force,forceDetails] = computeImprovedAPFDirection( ...
                nearestPoint,apfTarget,config.obstacles,config);
            result.apfActiveObstacleCount = result.apfActiveObstacleCount+ ...
                forceDetails.activeObstacleCount;
            if norm(force) < config.apfForceTolerance
                result.apfZeroForceCount = result.apfZeroForceCount+1;
                newIndex = nearestIndex;
                return;
            end
            xNew = nearestPoint+config.stepSize*force/norm(force);
        else
            % Bias-only ablation: retain the bidirectional RRT* search and
            % sampling bias, but steer toward the sampled point directly.
            [xNew,~] = steer(nearestPoint,randomPoint,config.stepSize);
        end
        newIndex = nearestIndex;
        if ~isInsideMap(xNew) || ~checkedCollision(nearestPoint,xNew)
            return;
        end

        nAfterInsert = size(tree.nodes,1)+1;
        radius = min( ...
            config.nearGamma*sqrt(log(max(2,nAfterInsert))/nAfterInsert), ...
            config.nearRadiusMax);
        candidates = unique([nearestIndex;nearNodes(tree.nodes,xNew,radius)]);
        [bestParent,bestCost] = chooseParentRRTStar( ...
            tree,candidates,xNew,nearestIndex,@checkedCollision);
        tree.nodes(end+1,:) = xNew;
        tree.parent(end+1,1) = bestParent;
        tree.cost(end+1,1) = bestCost;
        newIndex = size(tree.nodes,1);
        added = true;
        [tree,rewired] = rewireRRTStar( ...
            tree,newIndex,candidates,@checkedCollision);
    end

    function [connection,cost] = findConnection( ...
            startTree,goalTree,newIndex,active)
        connection = [NaN NaN];
        cost = Inf;
        xNew = trees{active}.nodes(newIndex,:);
        if config.apfPaperMode || strcmp(config.apfPaperConnectionMode,'nearest')
            [nearestOther,~] = nearestNode(trees{3-active}.nodes,xNew);
            candidateIndices = nearestOther;
        else
            otherTree = trees{3-active};
            nAfterConnectionInsert = size(otherTree.nodes,1)+1;
            connectionRadius = min( ...
                config.nearGamma*sqrt(log(max(2,nAfterConnectionInsert))/ ...
                nAfterConnectionInsert),config.nearRadiusMax);
            candidateIndices = nearNodes(otherTree.nodes,xNew,connectionRadius);
            if isempty(candidateIndices)
                [nearestOther,~] = nearestNode(otherTree.nodes,xNew);
                candidateIndices = nearestOther;
            end
        end

        for candidateIndex = reshape(candidateIndices,1,[])
            if active == 1
                candidatePoint = goalTree.nodes(candidateIndex,:);
                total = connectionCost(startTree,newIndex,goalTree,candidateIndex);
                pair = [newIndex candidateIndex];
            else
                candidatePoint = startTree.nodes(candidateIndex,:);
                total = connectionCost(startTree,candidateIndex,goalTree,newIndex);
                pair = [candidateIndex newIndex];
            end
            if checkedCollision(xNew,candidatePoint) && ...
                    total < cost-1e-12
                connection = pair;
                cost = total;
            end
        end
    end

    function total = connectionCost(startTree,startIndex,goalTree,goalIndex)
        total = startTree.cost(startIndex)+ ...
            norm(startTree.nodes(startIndex,:)-goalTree.nodes(goalIndex,:))+ ...
            goalTree.cost(goalIndex);
    end

    function valid = validateSearchPath(candidatePath)
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

    function inside = isInsideMap(point)
        inside = point(1) >= config.mapBounds(1) && ...
            point(1) <= config.mapBounds(2) && ...
            point(2) >= config.mapBounds(3) && ...
            point(2) <= config.mapBounds(4);
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
path = [pathStart;pathGoal];
end
