function result = RRTJoint(config)
%RRTJoint Basic single-tree joint-space RRT for the HMI comparison.
if nargin < 1, config = struct(); end
config = normalizeJointPlannerConfig(config);
rng(config.seed,'twister');
timer = tic;
result = struct('algorithm','RRT','success',false,'path',zeros(0,6), ...
    'firstPath',zeros(0,6),'planningTimeSec',NaN,'planningTime',NaN, ...
    'firstSolutionTimeSec',NaN,'firstSolutionTime',NaN, ...
    'firstSolutionIteration',NaN,'iterations',0,'nodeCount',0, ...
    'collisionConfigurationChecks',0,'collisionChecks',0, ...
    'edgeChecks',0,'edgeCollisionChecks',0,'samples',0, ...
    'terminationReason','not_started','seed',config.seed, ...
    'maxPlanningTimeSec',config.maxPlanningTimeSec);
nodes = config.start;
parents = 0;
stateChecks = 0;
edgeChecks = 0;
timedOut = false;
[startFree,~] = localState(config.start);
[goalFree,~] = localState(config.goal);
if ~startFree || ~goalFree
    result = localFinish(result,[],0,'invalid_endpoint');
    return;
end
for iter = 1:config.maxIterations
    result.samples = iter;
    if toc(timer) >= config.maxPlanningTimeSec
        timedOut = true; break;
    end
    qTarget = sampleJointConfig(config.jointLimits,config.goal,config.goalBias);
    [idx,~] = localNearest(nodes,qTarget);
    [qNew,reached,valid,~] = steerJoint(nodes(idx,:),qTarget, ...
        config.maxJointStepRad,config.jointLimits);
    if ~valid, continue; end
    edgeOptions = struct();
    edgeOptions.wrapJoints = true;
    edgeOptions.selfCollisionSkipPairs = config.selfCollisionSkipPairs;
    edgeChecks = edgeChecks + 1;
    [free,~,count,timedEdge] = collisionFreeJointEdge( ...
        config.robot,nodes(idx,:),qNew,config.environment, ...
        config.jointEdgeResolutionRad,edgeOptions);
    stateChecks = stateChecks + count;
    if timedEdge || ~free, continue; end
    nodes(end+1,:) = qNew; %#ok<AGROW>
    parents(end+1,1) = idx; %#ok<AGROW>
    if reached || norm(jointWrapDelta(qNew,config.goal),2) <= config.goalThresholdRad
        edgeChecks = edgeChecks + 1;
        edgeOptions = struct();
        edgeOptions.wrapJoints = true;
        edgeOptions.selfCollisionSkipPairs = config.selfCollisionSkipPairs;
        [edgeFree,~,count,timedEdge] = collisionFreeJointEdge( ...
            config.robot,qNew,config.goal,config.environment, ...
            config.jointEdgeResolutionRad,edgeOptions);
        stateChecks = stateChecks + count;
        if ~timedEdge && edgeFree
            nodes(end+1,:) = config.goal; %#ok<AGROW>
            parents(end+1,1) = size(nodes,1)-1; %#ok<AGROW>
            path = localBacktrack(nodes,parents,size(nodes,1));
            result = localFinish(result,path,iter,'goal_reached');
            result.firstSolutionIteration = iter;
            result.firstSolutionTimeSec = result.planningTimeSec;
            result.firstSolutionTime = result.planningTimeSec;
            return;
        end
    end
end
reason = 'max_iterations';
if timedOut, reason = 'time_limit'; end
result = localFinish(result,[],result.samples,reason);

    function [free,detail] = localState(q)
        stateChecks = stateChecks + 1;
        [free,detail] = collisionFreeJointState(config.robot,q, ...
            config.environment,config.selfCollisionSkipPairs);
    end
    function result = localFinish(result,path,iterations,reason)
        result.path = path; result.firstPath = path;
        result.success = ~isempty(path); result.iterations = iterations;
        result.nodeCount = size(nodes,1);
        result.collisionConfigurationChecks = stateChecks;
        result.collisionChecks = stateChecks;
        result.edgeChecks = edgeChecks; result.edgeCollisionChecks = edgeChecks;
        result.planningTimeSec = toc(timer); result.planningTime = result.planningTimeSec;
        result.terminationReason = reason;
    end
    function [idx,d] = localNearest(allNodes,target)
        distances = sqrt(sum((allNodes-target).^2,2));
        [d,idx] = min(distances);
    end
    function path = localBacktrack(allNodes,allParents,lastIndex)
        indices = lastIndex;
        while indices(1) ~= 1
            indices = [allParents(indices(1)),indices]; %#ok<AGROW>
        end
        path = allNodes(indices,:);
    end
end
