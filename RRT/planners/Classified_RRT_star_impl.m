function result = Classified_RRT_star_impl(config)
% Classified_RRT_star_impl  Classified-RRT* with goal-tree branches.
%
% This implementation keeps C-RRT* mechanisms separate from Q-RRT*/
% GD-RRT*: four-way node classification, random offset recovery,
% reverse-backtracking ChooseParent, sequential Rewire, and an explicit goal
% tree are all implemented here.

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

if isfield(config,'ablationUseOffset')
    useOffset = logical(config.ablationUseOffset);
else
    useOffset = true;
end
if isfield(config,'ablationUseClassificationGoalTree')
    useClassificationGoalTree = logical(config.ablationUseClassificationGoalTree);
else
    useClassificationGoalTree = true;
end
if isfield(config,'ablationUseBacktracking')
    useBacktracking = logical(config.ablationUseBacktracking);
else
    useBacktracking = true;
end
if isfield(config,'ablationUseSequentialRewire')
    useSequentialRewire = logical(config.ablationUseSequentialRewire);
else
    useSequentialRewire = true;
end

result = initRRTResult('Classified-RRT*',config);
if config.crrtPaperMode
    result.optimizationIterationsRequested = 0;
end
result.crrtBacktrackDepth = config.crrtBacktrackDepth;
result.crrtPaperMode = config.crrtPaperMode;
result.crrtPaperTotalIterations = config.crrtPaperTotalIterations;
result.nodeTypeCount = zeros(1,4);
result.nodeTypeFraction = zeros(1,4);
result.offsetTriggerCount = 0;
result.offsetAttemptCount = 0;
result.offsetSuccessCount = 0;
result.offsetFailureCount = 0;
result.offsetSuccessRate = 0;
result.offsetSamplePoints = zeros(0,2);
result.backtrackChecks = 0;
result.backtrackAcceptedCount = 0;
result.backtrackAcceptedDepthHistory = zeros(0,1);
result.goalTreeNodeCount = 1;
result.goalTreeConnectionChecks = 0;
result.goalTreeConnectionSuccesses = 0;
result.ablationUseOffset = useOffset;
result.ablationUseClassificationGoalTree = useClassificationGoalTree;
result.ablationUseBacktracking = useBacktracking;
result.ablationUseSequentialRewire = useSequentialRewire;
result.chooseParentCountByType = zeros(1,4);
result.rewireInvocationCountByType = zeros(1,4);

collisionChecks = 0;
rewireCount = 0;
planningTime = 0;

if config.showAnimation
    createPlannerFigure(config,'Classified-RRT*');
end

tree.nodes = config.start;
tree.parent = 0;
tree.cost = 0;

goalTree.nodes = config.goal;
goalTree.sourceTreeIndex = 0;
goalTree.costToGoal = 0;

if ~checkedCollision(config.start,config.start) || ...
        ~checkedCollision(config.goal,config.goal)
    result.planningTime = planningTime;
    result.collisionChecks = collisionChecks;
    result.nodeCount = 1;
    result.tree = tree;
    result.goalTree = goalTree;
    result.timeBudgetSnapshots = timeBudgetSnapshots;
    return;
end

firstPath = [];
firstSolutionIteration = NaN;
bestGoalCost = Inf;
bestConnection = emptyConnection();
iterationsUsed = 0;
optimizationStop = config.maxIterations;

if config.crrtPaperMode
    iterationLimit = min(config.maxIterations,config.crrtPaperTotalIterations);
else
    iterationLimit = config.maxIterations;
end
bestCostHistory = NaN(iterationLimit,1);

for iter = 1:iterationLimit
    iterationsUsed = iter;
    iterationTimer = tic;
    nodeAdded = false;
    nodeTypeThisIteration = NaN;
    offsetUsed = false;
    goalBranchAdded = false;
    candidatePath = [];
    candidateConnection = emptyConnection();
    candidateCost = Inf;

    xRand = sampleUniform(config);
    [nearestIndex,~] = nearestNode(tree.nodes,xRand);
    xNearest = tree.nodes(nearestIndex,:);
    [xNew,~] = steer(xNearest,xRand,config.stepSize);

    baseExpansionFree = false;
    if norm(xNew-xNearest) >= 1e-12
        baseExpansionFree = checkedCollision(xNearest,xNew);
    end

    if ~baseExpansionFree && useOffset
        result.offsetTriggerCount = result.offsetTriggerCount+1;
        [xNew,offsetUsed,offsetSucceeded] = attemptOffsetSample( ...
            xNearest,xNew);
        baseExpansionFree = offsetSucceeded;
    end

    if baseExpansionFree && norm(xNew-xNearest) >= 1e-12
        nodeAdded = true;
        if useClassificationGoalTree
            [nodeType,canConnectStart,canConnectGoal] = classifyCRRTNode( ...
                xNew,config.start,config.goal,@checkedCollision);
            nodeTypeThisIteration = nodeType;
            result.nodeTypeCount(nodeType) = ...
                result.nodeTypeCount(nodeType)+1;
        else
            % This ablation removes both node classification and the
            % explicit goal tree, while retaining the common RRT* parent
            % and rewire structure.
            nodeType = 4;
            canConnectStart = false;
            canConnectGoal = false;
            nodeTypeThisIteration = nodeType;
        end

        nAfterInsert = size(tree.nodes,1)+1;
        radius = min( ...
            config.nearGamma*sqrt(log(max(2,nAfterInsert))/nAfterInsert), ...
            config.nearRadiusMax);
        nearIndices = nearNodes(tree.nodes,xNew,radius);

        if ~useClassificationGoalTree
            [bestParent,bestCost] = chooseParentRRTStar( ...
                tree,unique([nearestIndex;nearIndices]),xNew, ...
                nearestIndex,@checkedCollision);
        elseif nodeType == 1 || nodeType == 2
            % Type 1/2: direct connection to the start is the parent.
            bestParent = 1;
            bestCost = norm(config.start-xNew);
        else
            % Type 3/4: Algorithm 4 with reverse backtracking.
            result.chooseParentCountByType(nodeType) = ...
                result.chooseParentCountByType(nodeType)+1;
            if useBacktracking
                [bestParent,bestCost,parentStats] = chooseParentCRRTStar( ...
                    tree,nearestIndex,nearIndices,xNew, ...
                    config.crrtBacktrackDepth,@checkedCollision);
            else
                [bestParent,bestCost] = chooseParentRRTStar( ...
                    tree,unique([nearestIndex;nearIndices]),xNew, ...
                    nearestIndex,@checkedCollision);
                parentStats = struct('backtrackChecks',0, ...
                    'backtrackAcceptedCount',0, ...
                    'backtrackAcceptedDepth',NaN);
            end
            result.backtrackChecks = result.backtrackChecks+ ...
                parentStats.backtrackChecks;
            result.backtrackAcceptedCount = ...
                result.backtrackAcceptedCount+parentStats.backtrackAcceptedCount;
            if parentStats.backtrackAcceptedCount > 0
                result.backtrackAcceptedDepthHistory(end+1,1) = ...
                    parentStats.backtrackAcceptedDepth;
            end
        end

        tree.nodes(end+1,:) = xNew;
        tree.parent(end+1,1) = bestParent;
        tree.cost(end+1,1) = bestCost;
        newIndex = size(tree.nodes,1);

        if isempty(nearIndices)
            rewireIndices = nearestIndex;
        else
            rewireIndices = nearIndices;
        end
        result.rewireInvocationCountByType(nodeType) = ...
            result.rewireInvocationCountByType(nodeType)+1;
        if useSequentialRewire
            [tree,rewiredThisIteration] = rewireCRRTStar( ...
                tree,newIndex,rewireIndices,@checkedCollision);
        else
            [tree,rewiredThisIteration] = rewireRRTStar( ...
                tree,newIndex,rewireIndices,@checkedCollision);
        end
        rewireCount = rewireCount+rewiredThisIteration;

        % Type 1/3 nodes are guaranteed to have a collision-free direct
        % edge to the goal and become goal-tree branch nodes.
        if useClassificationGoalTree && ...
                (nodeType == 1 || nodeType == 3) && canConnectGoal
            goalTree.nodes(end+1,:) = xNew;
            goalTree.sourceTreeIndex(end+1,1) = newIndex;
            goalTree.costToGoal(end+1,1) = norm(xNew-config.goal);
            goalBranchAdded = true;
        end

        if ~useClassificationGoalTree
            if norm(xNew-config.goal) <= config.goalThreshold && ...
                    checkedCollision(xNew,config.goal)
                candidateConnection.kind = 'direct';
                candidateConnection.treeNodeIndex = newIndex;
                candidateCost = tree.cost(newIndex)+norm(xNew-config.goal);
            end
        elseif nodeType == 1
            candidateConnection.kind = 'direct';
            candidateConnection.treeNodeIndex = newIndex;
            candidateCost = tree.cost(newIndex)+norm(xNew-config.goal);
        elseif nodeType == 3
            candidateConnection.kind = 'direct';
            candidateConnection.treeNodeIndex = newIndex;
            candidateCost = tree.cost(newIndex)+norm(xNew-config.goal);
        else
            [goalNode,bestConnectionCost,connectionStats] = ...
                checkCRRTGoalTreeConnection(tree,goalTree,newIndex, ...
                @checkedCollision);
            result.goalTreeConnectionChecks = ...
                result.goalTreeConnectionChecks+connectionStats.connectionChecks;
            result.goalTreeConnectionSuccesses = ...
                result.goalTreeConnectionSuccesses+connectionStats.connectionSuccesses;
            if ~isnan(goalNode)
                candidateConnection.kind = 'goalTree';
                candidateConnection.treeNodeIndex = newIndex;
                candidateConnection.goalTreeNodeIndex = goalNode;
                candidateCost = bestConnectionCost;
            end
        end

        % Rewiring can change the cost of the existing best tree branch.
        if ~isempty(bestConnection.kind)
            bestGoalCost = currentConnectionCost(bestConnection);
        end

        if ~isempty(candidateConnection.kind) && ...
                candidateCost < bestGoalCost-1e-12
            candidatePath = buildConnectionPath(candidateConnection);
            if validateCounted(candidatePath)
                bestConnection = candidateConnection;
                bestGoalCost = candidateCost;
                if isempty(firstPath)
                    firstPath = candidatePath;
                    firstSolutionIteration = iter;
                    candidatePath = firstPath;
                end
            end
        end
    end

    bestCostHistory(iter) = bestGoalCost;
    iterationPlanningTime = toc(iterationTimer);
    planningTime = planningTime+iterationPlanningTime;

    if ~isempty(firstPath) && isnan(result.firstSolutionTime)
        if timeBudgetMode
            result.firstSolutionTime = toc(timeBudgetTimer);
        else
            result.firstSolutionTime = planningTime;
        end
        result.collisionChecksAtFirstSolution = collisionChecks;
    end

    if config.showAnimation && nodeAdded
        drawCRRTNode(tree,tree.parent(newIndex),newIndex, ...
            nodeTypeThisIteration,offsetUsed,config.animationPause);
        if goalBranchAdded
            plot([xNew(1) config.goal(1)],[xNew(2) config.goal(2)],'-', ...
                'Color',[0.55 0.20 0.75],'LineWidth',0.8);
        end
        drawnow;
    end

    if timeBudgetMode
        snapshotPath = [];
        actualSnapshotTimeMs = toc(timeBudgetTimer)*1000;
        checkpointDue = nextBudgetIndex <= numel(timeBudgetsMs) && ...
            actualSnapshotTimeMs >= timeBudgetsMs(nextBudgetIndex);
        if checkpointDue && ~isempty(bestConnection.kind)
            candidateSnapshotPath = buildConnectionPath(bestConnection);
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
            result.firstSolutionTime,size(tree.nodes,1),collisionChecks, ...
            rewireCount);
        timeBudgetSnapshots = currentSnapshots;
        if nextBudgetIndex > numel(timeBudgetsMs)
            break;
        end
    % Paper mode deliberately runs a total horizon, not first solution plus
    % optimizeIterations.  Extended mode follows the project convention.
    elseif ~config.crrtPaperMode && ~isempty(firstPath)
        if isnan(optimizationStop) || optimizationStop == config.maxIterations
            optimizationStop = min(config.maxIterations, ...
                firstSolutionIteration+config.optimizeIterations);
        end
        if iter >= optimizationStop
            break;
        end
    end
end

result.iterations = iterationsUsed;
result.iterationHistory = (1:iterationsUsed).';
result.bestCostHistory = bestCostHistory(1:iterationsUsed);
result.planningTime = planningTime;
result.collisionChecks = collisionChecks;
result.rewireCount = rewireCount;
result.nodeCount = size(tree.nodes,1);
result.goalTreeNodeCount = size(goalTree.nodes,1);
result.tree = tree;
result.goalTree = goalTree;
result.timeBudgetSnapshots = timeBudgetSnapshots;
result.nodeTypeFraction = result.nodeTypeCount/max(1,sum(result.nodeTypeCount));
if result.offsetAttemptCount > 0
    result.offsetSuccessRate = result.offsetSuccessCount/ ...
        result.offsetAttemptCount;
end

if isnan(firstSolutionIteration)
    result.firstSolutionTime = NaN;
    result.firstSolutionIteration = NaN;
    result.optimizationIterationsExecuted = 0;
    result.paperOptimalPathLength = NaN;
    if config.showAnimation
        title('Classified-RRT* failed');
    end
    return;
end

if config.crrtPaperMode
    result.optimizationIterationsExecuted = 0;
else
    result.optimizationIterationsExecuted = max( ...
        0,result.iterations-firstSolutionIteration);
end

postTimer = tic;
result.path = buildConnectionPath(bestConnection);
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
result.paperInitialPathLength = result.firstPathLength;
result.paperOptimalPathLength = result.pathLength;
result.paperIterationsAfterFirst = max(0,result.iterations-firstSolutionIteration);
result.collisionChecks = collisionChecks;

if config.showAnimation && result.success
    plot(firstPath(:,1),firstPath(:,2),'--', ...
        'Color',[0.35 0.35 0.35],'LineWidth',1.2);
    plot(result.path(:,1),result.path(:,2),'b-','LineWidth',2.8);
    plot(result.path(:,1),result.path(:,2),'bo', ...
        'MarkerFaceColor','b','MarkerSize',4);
    drawnow;
end

    function free = checkedCollision(p1,p2)
        collisionChecks = collisionChecks+1;
        free = collisionFree(p1,p2,config.obstacles,config.clearance);
    end

    function [offsetPoint,used,success] = attemptOffsetSample( ...
            nearestPoint,blockedPoint)
        used = true;
        success = false;
        offsetPoint = blockedPoint;
        radius = norm(blockedPoint-nearestPoint);
        for attempt = 1:config.crrtOffsetAttempts
            result.offsetAttemptCount = result.offsetAttemptCount+1;
            [candidatePoint,~,~] = sampleCRRTOffset( ...
                nearestPoint,blockedPoint);
            if config.crrtStoreOffsetPoints
                result.offsetSamplePoints(end+1,:) = candidatePoint;
            end
            if radius <= 1e-12 || ~isInsideMap(candidatePoint)
                continue;
            end
            if checkedCollision(nearestPoint,candidatePoint)
                offsetPoint = candidatePoint;
                result.offsetSuccessCount = result.offsetSuccessCount+1;
                success = true;
                return;
            end
        end
        result.offsetFailureCount = result.offsetFailureCount+1;
    end

    function inside = isInsideMap(point)
        inside = point(1) >= config.mapBounds(1) && ...
            point(1) <= config.mapBounds(2) && ...
            point(2) >= config.mapBounds(3) && ...
            point(2) <= config.mapBounds(4);
    end

    function connection = emptyConnection()
        connection = struct('kind','','treeNodeIndex',NaN, ...
            'goalTreeNodeIndex',NaN);
    end

    function cost = currentConnectionCost(connection)
        if strcmp(connection.kind,'direct')
            nodeIndex = connection.treeNodeIndex;
            cost = tree.cost(nodeIndex)+ ...
                norm(tree.nodes(nodeIndex,:)-config.goal);
        else
            nodeIndex = connection.treeNodeIndex;
            goalNode = connection.goalTreeNodeIndex;
            cost = tree.cost(nodeIndex)+ ...
                norm(tree.nodes(nodeIndex,:)-goalTree.nodes(goalNode,:))+ ...
                goalTree.costToGoal(goalNode);
        end
    end

    function connectionPath = buildConnectionPath(connection)
        treePath = backtrackPath(tree.nodes,tree.parent, ...
            connection.treeNodeIndex);
        if strcmp(connection.kind,'direct')
            connectionPath = [treePath;config.goal];
        else
            goalNode = connection.goalTreeNodeIndex;
            connectionPath = [treePath;goalTree.nodes(goalNode,:);config.goal];
        end
    end

    function valid = validateCounted(candidate)
        valid = false;
        if isempty(candidate) || size(candidate,2) ~= 2
            return;
        end
        if norm(candidate(1,:)-config.start) > 1e-8 || ...
                norm(candidate(end,:)-config.goal) > 1e-8
            return;
        end
        for edgeIndex = 1:size(candidate,1)-1
            if ~checkedCollision(candidate(edgeIndex,:), ...
                    candidate(edgeIndex+1,:))
                return;
            end
        end
        valid = true;
    end

end

function drawCRRTNode(tree,parentIndex,newIndex,nodeType,offsetUsed,pauseTime)
typeColors = [
    0.85 0.10 0.10
    0.10 0.55 0.85
    0.15 0.65 0.25
    0.45 0.45 0.45];
if offsetUsed
    edgeColor = [0.95 0.55 0.05];
else
    edgeColor = typeColors(nodeType,:);
end
drawPlannerEdge(tree.nodes(parentIndex,:),tree.nodes(newIndex,:), ...
    edgeColor,pauseTime);
plot(tree.nodes(newIndex,1),tree.nodes(newIndex,2),'.', ...
    'Color',typeColors(nodeType,:),'MarkerSize',8);
end
