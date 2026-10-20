function result = GD_RRT_star_impl(config)
% GD_RRT_star_impl  Paper-literal GD-RRT* with an extended project mode.
%
% The planner reuses Q-RRT* ancestor-aware ChooseParent and descendant cost
% propagation.  GD-RRT* adds the ten-trial collision-frequency update,
% target/Gaussian sampling, adaptive step/depth, and paper-order Rewire.

if nargin < 1
    config = struct();
end

config = normalizeRRTConfig(config);
rng(config.seed,'twister');

result = initRRTResult('GD-RRT*',config);
result.gdPaperMode = config.gdPaperMode;
result.gdSamplingInterpretation = config.gdSamplingInterpretation;
result.qDepth = config.qDepth;
result.ancestorCandidateCount = 0;
result.ancestorChecks = 0;
result.ancestorAcceptedCount = 0;
result.pcfHistory = zeros(0,1);
result.pcfIterationHistory = zeros(0,1);
result.depthHistory = zeros(0,1);
result.stepHistory = zeros(0,1);
result.alphaHistory = false(0,1);
result.targetSampleCount = 0;
result.gaussianSampleCount = 0;
result.gaussianFallbackCount = 0;
result.gaussianForwardComponentCount = 0;
result.gaussianBackwardComponentCount = 0;
result.bestCostHistory = zeros(0,1);
result.iterationHistory = zeros(0,1);

collisionChecks = 0;
rewireCount = 0;
planningTime = 0;

if config.showAnimation
    createPlannerFigure(config,'GD-RRT*');
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

frame = computeGaussianFrame(config.start,config.goal);

% Algorithm 1 initial state: alpha=false, Depth=1, p=0, base step.
alpha = false;
currentStep = config.stepSize;
currentDepth = 1;
pcf = 0;
gaussianParams = [];
baseExpansionWindow = false(0,1);
successfulNodeIndices = zeros(0,1);

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

    if alpha && ~isempty(gaussianParams)
        [sample,sampleInfo] = sampleGaussianWithBounds();
        if strcmp(sampleInfo.mode,'gaussian')
            samplingMode = 'gaussian';
        else
            samplingMode = 'gaussian_fallback';
        end
    else
        % Phase 7 uses the paper-literal TargetSample interpretation:
        % when alpha is false, TargetSample returns xGoal directly.  The
        % later prose mentions probabilistic target bias, but that is not
        % silently mixed into this reproduction mode.
        sample = config.goal;
        result.targetSampleCount = result.targetSampleCount+1;
        samplingMode = 'target';
    end

    [nearestIndex,~] = nearestNode(tree.nodes,sample);
    nearestPoint = tree.nodes(nearestIndex,:);
    [newPoint,~] = steer(nearestPoint,sample,currentStep);

    baseExpansionSuccess = false;
    if norm(newPoint-nearestPoint) >= 1e-12
        baseExpansionSuccess = checkedCollision(nearestPoint,newPoint);
    end
    baseExpansionWindow(end+1,1) = baseExpansionSuccess;
    if numel(baseExpansionWindow) > 10
        baseExpansionWindow = baseExpansionWindow(end-9:end);
    end

    if baseExpansionSuccess
        nodeAdded = true;
        nAfterInsert = size(tree.nodes,1)+1;
        radius = min( ...
            config.nearGamma*sqrt(log(max(2,nAfterInsert))/nAfterInsert), ...
            config.nearRadiusMax);
        nearIndices = nearNodes(tree.nodes,newPoint,radius);

        [bestParent,bestCost,parentStats] = chooseParentQRRTStar( ...
            tree,nearestIndex,nearIndices,newPoint,currentDepth, ...
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
        successfulNodeIndices(end+1,1) = newIndex;

        if isempty(nearIndices)
            rewireIndices = nearestIndex;
        else
            rewireIndices = nearIndices;
        end

        [tree,rewiredThisIteration] = rewireGDRRTStar( ...
            tree,newIndex,rewireIndices,@checkedCollision);
        rewireCount = rewireCount+rewiredThisIteration;

        % A rewire can improve the currently selected goal branch.
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

                if isempty(firstPath) && validateCounted(candidatePath)
                    candidateFirstPath = candidatePath;
                    solutionFoundThisIteration = true;
                    firstSolutionIteration = iter;
                    if config.gdPaperMode
                        % Algorithm 1 returns immediately after the first
                        % feasible path in the paper-reproduction mode.
                        optimizationStop = iter;
                    else
                        optimizationStop = min( ...
                            config.maxIterations, ...
                            firstSolutionIteration+config.optimizeIterations);
                    end
                end
            end
        end
    end

    % A ten-trial window updates pcf and ParamUpdate.  pcf is the fraction
    % of failed base expansion edges: pcf=(10-p)/10.
    if mod(iter,10) == 0
        pcf = computeGDRRTPCF(baseExpansionWindow);
        parameterState = updateGDRRTParameters(pcf,config.stepSize);
        alpha = parameterState.alpha;
        currentStep = parameterState.currentStep;
        currentDepth = parameterState.currentDepth;

        result.pcfHistory(end+1,1) = pcf;
        result.pcfIterationHistory(end+1,1) = iter;
        result.depthHistory(end+1,1) = currentDepth;
        result.stepHistory(end+1,1) = currentStep;
        result.alphaHistory(end+1,1) = alpha;

        if alpha
            recentCount = min(10,numel(successfulNodeIndices));
            if recentCount > 0
                recentIndices = successfulNodeIndices(end-recentCount+1:end);
                recentWorldNodes = tree.nodes(recentIndices,:);
                recentLocalNodes = gaussianWorldToLocal( ...
                    frame,recentWorldNodes);
                gaussianParams = computeGDRRTGaussianParameters( ...
                    recentLocalNodes,currentStep,pcf);
            else
                gaussianParams = [];
            end
        else
            gaussianParams = [];
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
        if strcmp(samplingMode,'target')
            edgeColor = [0.90 0.45 0.10];
            nodeColor = [0.80 0.25 0.05];
        elseif strcmp(samplingMode,'gaussian')
            edgeColor = [0.50 0.20 0.75];
            nodeColor = [0.35 0.10 0.60];
        else
            edgeColor = [0.35 0.35 0.35];
            nodeColor = [0.20 0.20 0.20];
        end
        drawPlannerEdge(tree.nodes(tree.parent(newIndex),:), ...
            tree.nodes(newIndex,:),edgeColor,config.animationPause);
        plot(tree.nodes(newIndex,1),tree.nodes(newIndex,2),'.', ...
            'Color',nodeColor,'MarkerSize',7);
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
result.currentDepth = currentDepth;
result.currentStep = currentStep;
result.alpha = alpha;
result.pcf = pcf;

if isnan(firstSolutionIteration)
    result.firstSolutionTime = NaN;
    result.firstSolutionIteration = NaN;
    result.optimizationIterationsExecuted = 0;
    if config.showAnimation
        title('GD-RRT* failed');
    end
    return;
end

if config.gdPaperMode
    result.optimizationIterationsExecuted = 0;
else
    result.optimizationIterationsExecuted = max( ...
        0,result.iterations-firstSolutionIteration);
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
    title(sprintf('GD-RRT* (%s) - %.3f ms', ...
        ternary(config.gdPaperMode,'paper','extended'), ...
        result.planningTime*1000));
    drawnow;
end

    function free = checkedCollision(p1,p2)
        collisionChecks = collisionChecks+1;
        free = collisionFree(p1,p2,config.obstacles,config.clearance);
    end

    function [point,info] = sampleGaussianWithBounds()
        info = struct();
        info.mode = 'gaussian';

        for retry = 1:config.gdGaussianMaxRetries
            [point,sampleInfo] = sampleGDRRTGaussian(frame,gaussianParams);
            result.gaussianSampleCount = result.gaussianSampleCount+1;
            if sampleInfo.yComponent == 1
                result.gaussianForwardComponentCount = ...
                    result.gaussianForwardComponentCount+1;
            else
                result.gaussianBackwardComponentCount = ...
                    result.gaussianBackwardComponentCount+1;
            end

            if isInsideMap(point)
                return;
            end
        end

        point = sampleUniform(config);
        result.gaussianFallbackCount = result.gaussianFallbackCount+1;
        info.mode = 'gaussian_fallback';
    end

    function inside = isInsideMap(point)
        inside = point(1) >= config.mapBounds(1) && ...
            point(1) <= config.mapBounds(2) && ...
            point(2) >= config.mapBounds(3) && ...
            point(2) <= config.mapBounds(4);
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

function output = ternary(condition,whenTrue,whenFalse)
if condition
    output = whenTrue;
else
    output = whenFalse;
end
end
