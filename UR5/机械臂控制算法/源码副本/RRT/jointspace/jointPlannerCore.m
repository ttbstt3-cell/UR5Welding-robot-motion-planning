function result = jointPlannerCore(config,variant)
%jointPlannerCore Shared offline 6-DOF planner for Phase 12A.
%
% The four public wrappers select only the search mechanism.  State metric,
% steering, joint limits, and collision checking are shared here so the
% migration comparison cannot silently use different geometry operations.

config = normalizeJointPlannerConfig(config);
variant = char(string(variant));
switch variant
    case 'RRT-Connect'
        config.rewire = false;
    case 'Bi-RRT*'
        config.rewire = true;
        config.classificationGoalTree = false;
        config.useOffset = false;
        config.useBacktracking = false;
        config.useSequentialRewire = false;
    case 'ClassGoal-Lite'
        config.rewire = true;
        config.classificationGoalTree = true;
        config.useOffset = false;
        config.useBacktracking = false;
        config.useSequentialRewire = false;
    case 'Full Classified-RRT*'
        config.rewire = true;
        config.classificationGoalTree = true;
        config.useOffset = true;
        config.useBacktracking = true;
        config.useSequentialRewire = true;
    otherwise
        error('Unknown Phase 12 planner variant: %s',variant);
end

rng(config.seed,'twister');
timer = tic;
firstSolutionElapsed = NaN;
firstSolutionPath = zeros(0,6);
bestPath = zeros(0,6);
bestCost = Inf;
bestElapsed = NaN;
stateCache = containers.Map('KeyType','char','ValueType','any');
edgeCache = containers.Map('KeyType','char','ValueType','any');
stats = struct('collisionChecks',0,'edgeChecks',0,'rewireCount',0, ...
    'samples',0,'offsetTriggerCount',0,'offsetAttemptCount',0, ...
    'offsetSuccessCount',0,'goalTreeConnectionChecks',0, ...
    'goalTreeConnectionSuccesses',0,'timedOut',false, ...
    'candidateCount',0,'candidateValidationFailures',0, ...
    'firstCandidateElapsed',NaN,'firstCertifiedElapsed',NaN, ...
    'lastCandidateFailureEdge',NaN,'lastCandidateFailureSample',NaN, ...
    'lastCandidateFailureType',"",'lastCandidateValidationTimedOut',false, ...
    'stateCacheHits',0,'edgeCacheHits',0, ...
    'checkpointTimesSec',config.checkpointTimesSec, ...
    'nextCheckpointIndex',1, ...
    'bestPathAtCheckpoint',{cell(1,numel(config.checkpointTimesSec))}, ...
    'bestCostAtCheckpoint',NaN(1,numel(config.checkpointTimesSec)));

result = localEmptyResult(variant,config);
result.edgeCollisionMode = 'discrete interpolated joint-edge checking';
result.jointEdgeResolutionRad = config.jointEdgeResolutionRad;
result.maxPlanningTimeSec = config.maxPlanningTimeSec;

[startFree,~] = stateFree(config.start);
[goalFree,~] = stateFree(config.goal);
if ~startFree || ~goalFree
    result = finishResult(result,[],NaN,0,[],[]);
    return;
end

switch variant
    case 'RRT-Connect'
        [firstPath,firstIter,iterations,trees] = runConnect();
        goalTree = struct('nodes',config.goal,'costToGoal',0);
    case 'Bi-RRT*'
        [firstPath,firstIter,iterations,trees] = runBiRRTStar();
        goalTree = struct('nodes',config.goal,'costToGoal',0);
    otherwise
        [firstPath,firstIter,iterations,trees,goalTree] = runClassified();
end

if isempty(firstPath)
    result = finishResult(result,[],NaN,iterations,trees,goalTree);
    return;
end

result = finishResult(result,firstPath,firstIter,iterations,trees,goalTree);

function result = localEmptyResult(variant,config)
result = struct('algorithm',variant,'success',false,'path',zeros(0,6), ...
    'firstPath',zeros(0,6),'planningTimeSec',NaN,'planningTimeMs',NaN, ...
    'planningTime',NaN,'firstSolutionTime',NaN, ...
    'firstSolutionTimeSec',NaN,'firstSolutionTimeMs',NaN,'iterations',0, ...
    'firstSolutionIteration',NaN,'jointPathCostL2',NaN, ...
    'totalJointMotionPerAxis',NaN(1,6),'maxJointEdgeStepRad',NaN, ...
    'nodeCount',0,'pathStateCount',0,'collisionChecks',0, ...
    'collisionConfigurationChecks',0,'edgeChecks',0,'edgeCollisionChecks',0, ...
    'rewireCount',0,'samples',0,'goalTreeNodeCount',1, ...
    'offsetTriggerCount',0,'offsetAttemptCount',0,'offsetSuccessCount',0, ...
    'goalTreeConnectionChecks',0,'goalTreeConnectionSuccesses',0, ...
    'timedOut',false,'terminationReason',"not_started", ...
    'certifiedSuccess',false,'candidateCount',0, ...
    'candidateValidationFailures',0,'firstCandidateTimeSec',NaN, ...
    'firstCandidateTimeMs',NaN,'firstCertifiedTimeSec',NaN, ...
    'firstCertifiedTimeMs',NaN,'lastCandidateFailureEdge',NaN, ...
    'lastCandidateFailureSample',NaN,'lastCandidateFailureType',"", ...
    'lastCandidateValidationTimedOut',false, ...
    'stateCacheHits',0,'edgeCacheHits',0, ...
    'anytime',config.anytime, ...
    'continueAfterFirstSolution',config.continueAfterFirstSolution, ...
    'checkpointTimesSec',config.checkpointTimesSec, ...
    'bestPathAtCheckpoint',{cell(1,numel(config.checkpointTimesSec))}, ...
    'bestPathLengthAtCheckpoint',NaN(1,numel(config.checkpointTimesSec)), ...
    'maxPlanningTimeSec',config.maxPlanningTimeSec, ...
    'seed',config.seed,'config',config,'jointLimits',config.jointLimits, ...
    'jointWeights',config.jointWeights,'edgeCollisionMode','');
end

function result = finishResult(result,path,firstIter,iterations,trees,goalTree)
elapsed = toc(timer);
captureCheckpoints(true);
if isempty(bestPath), chosenPath=path; else, chosenPath=bestPath; end
result.path = chosenPath;
result.firstPath = firstSolutionPath;
% Every returned path has already passed the common internal validation.
% When candidateValidatorFcn is configured it has also passed strict
% certification.  Do not validate again here: that would double-count
% expensive collision checks and could consume the remaining deadline.
result.success = ~isempty(chosenPath);
result.planningTimeSec = elapsed;
result.planningTimeMs = elapsed*1000;
result.planningTime = elapsed;
result.iterations = iterations;
result.firstSolutionIteration = firstIter;
result.collisionChecks = stats.collisionChecks;
% collisionChecks has historically counted every discrete joint
% configuration sent through collisionFreeJointState.  Preserve it and
% expose the explicit Phase 12 metric names for later benchmarks.
result.collisionConfigurationChecks = stats.collisionChecks;
result.edgeChecks = stats.edgeChecks;
result.edgeCollisionChecks = stats.edgeChecks;
result.rewireCount = stats.rewireCount;
result.samples = stats.samples;
result.offsetTriggerCount = stats.offsetTriggerCount;
result.offsetAttemptCount = stats.offsetAttemptCount;
result.offsetSuccessCount = stats.offsetSuccessCount;
result.goalTreeConnectionChecks = stats.goalTreeConnectionChecks;
result.goalTreeConnectionSuccesses = stats.goalTreeConnectionSuccesses;
result.timedOut = stats.timedOut;
result.maxPlanningTimeSec = config.maxPlanningTimeSec;
result.candidateCount = stats.candidateCount;
result.candidateValidationFailures = stats.candidateValidationFailures;
result.certifiedSuccess = result.success && ...
    (isempty(config.candidateValidatorFcn) || isfinite(stats.firstCertifiedElapsed));
result.firstCandidateTimeSec = stats.firstCandidateElapsed;
result.firstCandidateTimeMs = stats.firstCandidateElapsed*1000;
result.firstCertifiedTimeSec = stats.firstCertifiedElapsed;
result.firstCertifiedTimeMs = stats.firstCertifiedElapsed*1000;
result.lastCandidateFailureEdge = stats.lastCandidateFailureEdge;
result.lastCandidateFailureSample = stats.lastCandidateFailureSample;
result.lastCandidateFailureType = stats.lastCandidateFailureType;
result.lastCandidateValidationTimedOut = stats.lastCandidateValidationTimedOut;
result.stateCacheHits = stats.stateCacheHits;
result.edgeCacheHits = stats.edgeCacheHits;
result.anytime = config.anytime;
result.continueAfterFirstSolution = config.continueAfterFirstSolution;
result.checkpointTimesSec = config.checkpointTimesSec;
result.bestPathAtCheckpoint = stats.bestPathAtCheckpoint;
result.bestPathLengthAtCheckpoint = stats.bestCostAtCheckpoint;
result.firstPathCostL2 = pathCostOrNaN(firstSolutionPath);
result.bestPathCostL2 = pathCostOrNaN(chosenPath);
result.bestPathTimeSec = bestElapsed;
result.bestPathTimeMs = bestElapsed*1000;
if ~isempty(trees)
    if iscell(trees)
        result.nodeCount = size(trees{1}.nodes,1)+size(trees{2}.nodes,1);
    else
        result.nodeCount = size(trees.nodes,1);
    end
end
if ~isempty(goalTree), result.goalTreeNodeCount=size(goalTree.nodes,1); end
if result.success
    if isfinite(stats.firstCertifiedElapsed)
        result.firstSolutionTimeSec = stats.firstCertifiedElapsed;
        result.firstSolutionTimeMs = stats.firstCertifiedElapsed*1000;
    else
        result.firstSolutionTimeSec = elapsed;
        result.firstSolutionTimeMs = elapsed*1000;
    end
    result.firstSolutionTime = result.firstSolutionTimeSec;
    result.jointPathCostL2 = pathCost(chosenPath);
    result.totalJointMotionPerAxis = pathMotion(chosenPath);
    result.maxJointEdgeStepRad = maxPathStep(chosenPath);
    result.pathStateCount = size(chosenPath,1);
end
if stats.timedOut
    if result.success, result.terminationReason = "timeout_with_solution";
    else, result.terminationReason = "timeout"; end
elseif result.success
    result.terminationReason = "success";
else
    result.terminationReason = "maxIterations";
end
end

function [path,firstIter,iterations,trees] = runConnect()
Ts = newTree(config.start); Tg = newTree(config.goal);
trees = {Ts,Tg}; swapped = false; path = []; firstIter = NaN; iterations = 0;
for iter = 1:config.maxIterations
    if abortRequested(), return; end
    iterations = iter; stats.samples = stats.samples+1;
    target = sampleJointConfig(config.jointLimits,config.goal,config.goalBias);
    [trees{1},idx,status] = extendTree(trees{1},target,false);
    if abortRequested(), return; end
    if ~strcmp(status,'trapped')
        [trees{2},otherIdx,statusOther] = connectTree( ...
            trees{2},trees{1}.nodes(idx,:));
        if abortRequested(), return; end
        if strcmp(statusOther,'reached')
            if ~swapped
                path = twoTreePath(trees{1},idx,trees{2},otherIdx);
            else
                path = twoTreePath(trees{2},otherIdx,trees{1},idx);
            end
            [accepted,abortNow]=considerCandidate(path,iter);
            if accepted
                if isnan(firstIter), firstIter=iter; end
                if ~config.continueAfterFirstSolution, return; end
            end
            if abortNow, return; end
        end
    end
    [trees{1},trees{2},swapped] = swapTrees(trees{1},trees{2},swapped);
end
if isempty(bestPath), path=[]; else, path=bestPath; end
end

function [path,firstIter,iterations,trees] = runBiRRTStar()
Ts = newTree(config.start); Tg = newTree(config.goal);
trees = {Ts,Tg}; path = []; firstIter = NaN; iterations = 0;
for iter = 1:config.maxIterations
    if abortRequested(), return; end
    iterations = iter; stats.samples = stats.samples+1;
    active = 1+mod(iter-1,2); other = 3-active;
    target = sampleJointConfig(config.jointLimits,config.goal,config.goalBias);
    [trees{active},idx,added] = extendStar(trees{active},target);
    if abortRequested(), return; end
    if ~added, continue; end
    [otherIdx,connected] = bestConnection(trees{other}, ...
        trees{active}.nodes(idx,:));
    if abortRequested(), return; end
    if connected
        if active==1, path=twoTreePath(trees{1},idx,trees{2},otherIdx);
        else, path=twoTreePath(trees{1},otherIdx,trees{2},idx); end
        [accepted,abortNow]=considerCandidate(path,iter);
        if accepted
            if isnan(firstIter), firstIter=iter; end
            if ~config.continueAfterFirstSolution, return; end
        end
        if abortNow, return; end
    end
end
if isempty(bestPath), path=[]; else, path=bestPath; end
end

function [path,firstIter,iterations,trees,goalTree] = runClassified()
tree = newTree(config.start);
trees = tree;
goalTree = struct('nodes',config.goal,'costToGoal',0);
path=[]; firstIter=NaN; iterations=0;
for iter=1:config.maxIterations
    if abortRequested(), return; end
    iterations=iter; stats.samples=stats.samples+1;
    target=sampleJointConfig(config.jointLimits,config.goal,config.goalBias);
    [tree,newIndex,added]=extendClassified(tree,target);
    if abortRequested(), return; end
    if ~added, continue; end
    xNew=tree.nodes(newIndex,:);
    [canStart,~]=edgeFree(config.start,xNew);
    if abortRequested(), return; end
    [canGoal,~]=edgeFree(xNew,config.goal);
    if abortRequested(), return; end
    if canGoal
        goalTree.nodes(end+1,:)=xNew;
        goalTree.costToGoal(end+1,1)=jointDistance(xNew,config.goal,config.jointWeights);
    end
    candidate=[];
    if (canStart || nodeTypeFor(xNew)==3) && canGoal
        candidate=[traceTree(tree,newIndex);config.goal];
    elseif size(goalTree.nodes,1)>1
        [goalIndex,connected]=bestGoalTreeConnection(tree,newIndex,goalTree);
        if connected
            candidate=[traceTree(tree,newIndex);goalTree.nodes(goalIndex,:);config.goal];
        end
    end
    candidate=removeDuplicateRows(candidate);
    if ~isempty(candidate)
        [accepted,abortNow]=considerCandidate(candidate,iter);
        if accepted
            path=candidate;
            if isnan(firstIter), firstIter=iter; end
            if ~config.continueAfterFirstSolution, trees=tree; return; end
        end
        if abortNow, trees=tree; return; end
    end
end
if isempty(bestPath), path=[]; else, path=bestPath; end
trees=tree;
end

function [accepted,abortNow] = considerCandidate(candidate,~)
% Validate a connected path first with the planner's internal resolution,
% then optionally with the strict certification callback.  A rejected
% candidate is not a success and the search continues until the deadline.
accepted=false; abortNow=false;
if abortRequested(), abortNow=true; return; end
if ~validatePath(candidate)
    abortNow=stats.timedOut;
    return;
end

stats.candidateCount=stats.candidateCount+1;
if isnan(stats.firstCandidateElapsed)
    stats.firstCandidateElapsed=toc(timer);
end

if isempty(config.candidateValidatorFcn)
    if isnan(stats.firstCertifiedElapsed)
        stats.firstCertifiedElapsed=stats.firstCandidateElapsed;
    end
    registerCertifiedCandidate(candidate);
    accepted=true;
    return;
end

[certified,detail]=config.candidateValidatorFcn(candidate,@abortRequested);
if isstruct(detail)
    if isfield(detail,'collisionConfigurationChecks')
        stats.collisionChecks=stats.collisionChecks+detail.collisionConfigurationChecks;
    end
    if isfield(detail,'edgeCollisionChecks')
        stats.edgeChecks=stats.edgeChecks+detail.edgeCollisionChecks;
    end
end
if certified
    if isnan(stats.firstCertifiedElapsed)
        stats.firstCertifiedElapsed=toc(timer);
    end
    registerCertifiedCandidate(candidate);
    accepted=true;
    return;
end

stats.candidateValidationFailures=stats.candidateValidationFailures+1;
if isstruct(detail)
    if isfield(detail,'failureEdgeIndex'), stats.lastCandidateFailureEdge=detail.failureEdgeIndex; end
    if isfield(detail,'failureSampleIndex'), stats.lastCandidateFailureSample=detail.failureSampleIndex; end
    if isfield(detail,'failureCollisionType'), stats.lastCandidateFailureType=detail.failureCollisionType; end
    if isfield(detail,'timedOut') && detail.timedOut
        stats.lastCandidateValidationTimedOut=true;
        stats.timedOut=true;
    end
end
abortNow=stats.timedOut || abortRequested();
end

function tree = newTree(q)
tree=struct('nodes',reshape(double(q),1,6),'parent',0,'cost',0);
end

function [tree,index,status] = extendTree(tree,target,useStar)
if abortRequested(), index=1; status='trapped'; return; end
[nearIndex,~]=nearestIndex(tree.nodes,target);
qNear=tree.nodes(nearIndex,:);
[qNew,reached,valid]=steerJoint(qNear,target,config.maxJointStepRad,config.jointLimits);
index=nearIndex; status='trapped';
if ~valid || jointDistance(qNew,qNear,config.jointWeights)<1e-12, return; end
if ~edgeFree(qNear,qNew), return; end
if abortRequested(), return; end

parent=nearIndex; cost=tree.cost(nearIndex)+jointDistance(qNear,qNew,config.jointWeights);
if useStar
    [parent,cost,parentFound]=chooseParent(tree,qNew,nearIndex);
    if abortRequested(), return; end
    if ~parentFound, return; end
end
tree.nodes(end+1,:)=qNew;
tree.parent(end+1,1)=parent;
tree.cost(end+1,1)=cost;
index=size(tree.nodes,1); status='advanced';
if reached, status='reached'; end
if useStar && config.rewire
    tree=rewireTree(tree,index,false);
end
end

function [tree,index,added] = extendStar(tree,target)
[tree,index,status]=extendTree(tree,target,true);
added=~strcmp(status,'trapped');
end

function [tree,index,status] = connectTree(tree,target)
while true
    if abortRequested(), status='trapped'; return; end
    [tree,index,oneStatus]=extendTree(tree,target,false);
    if abortRequested(), status='trapped'; return; end
    if strcmp(oneStatus,'trapped'), status='trapped'; return; end
    if strcmp(oneStatus,'reached'), status='reached'; return; end
end
end

function [parent,bestCost,found] = chooseParent(tree,qNew,nearest)
n=size(tree.nodes,1);
radius=nearRadius(n+1);
near=nearIndices(tree.nodes,qNew,radius);
candidates=unique([nearest;near(:)]);
parent=nearest;
bestCost=tree.cost(nearest)+jointDistance(tree.nodes(nearest,:),qNew,config.jointWeights);
found=false;
for idx=reshape(candidates,1,[])
    if abortRequested(), return; end
    candidateCost=tree.cost(idx)+jointDistance(tree.nodes(idx,:),qNew,config.jointWeights);
    if candidateCost < bestCost-1e-12 && edgeFree(tree.nodes(idx,:),qNew)
        if abortRequested(), return; end
        parent=idx; bestCost=candidateCost; found=true;
    end
end
if edgeFree(tree.nodes(nearest,:),qNew) && ~abortRequested(), found=true; end
end

function tree=rewireTree(tree,newIndex,sequential)
if ~config.rewire, return; end
radius=nearRadius(size(tree.nodes,1));
near=nearIndices(tree.nodes,tree.nodes(newIndex,:),radius);
for idx=reshape(near,1,[])
    if abortRequested(), return; end
    if idx==newIndex || idx==tree.parent(newIndex), continue; end
    candidateCost=tree.cost(newIndex)+jointDistance( ...
        tree.nodes(newIndex,:),tree.nodes(idx,:),config.jointWeights);
    if candidateCost >= tree.cost(idx)-1e-12 || isDescendant(tree,idx,newIndex)
        continue;
    end
    if sequential
        parents=[tree.parent(newIndex),newIndex,tree.parent(idx)];
        rewired=false;
        for parent=unique(parents,'stable')
            if parent>0 && parent~=idx && ~isDescendant(tree,idx,parent) && ...
                    tree.cost(parent)+jointDistance(tree.nodes(parent,:),tree.nodes(idx,:),config.jointWeights) < tree.cost(idx)-1e-12 && ...
                    edgeFree(tree.nodes(parent,:),tree.nodes(idx,:))
                tree=assignParent(tree,idx,parent);
                stats.rewireCount=stats.rewireCount+1; rewired=true; break;
            end
        end
        if rewired, continue; end
    elseif edgeFree(tree.nodes(newIndex,:),tree.nodes(idx,:)) && ~abortRequested()
        tree=assignParent(tree,idx,newIndex);
        stats.rewireCount=stats.rewireCount+1;
    end
end
end

function [tree,newIndex,added] = extendClassified(tree,target)
if abortRequested(), newIndex=1; added=false; return; end
[nearIndex,~]=nearestIndex(tree.nodes,target);
qNear=tree.nodes(nearIndex,:);
[qNew,reached,valid]=steerJoint(qNear,target,config.maxJointStepRad,config.jointLimits); %#ok<ASGLU>
if ~valid || jointDistance(qNear,qNew,config.jointWeights)<1e-12
    newIndex=nearIndex; added=false; return;
end
if ~edgeFree(qNear,qNew)
    if abortRequested(), newIndex=nearIndex; added=false; return; end
    stats.offsetTriggerCount=stats.offsetTriggerCount+1;
    if config.useOffset
        [qNew,offsetOK]=tryOffset(qNear,qNew);
        if abortRequested(), newIndex=nearIndex; added=false; return; end
        if ~offsetOK, newIndex=nearIndex; added=false; return; end
    else
        newIndex=nearIndex; added=false; return;
    end
end

[parent,cost,parentOK]=chooseClassifiedParent(tree,qNew,nearIndex);
if abortRequested(), newIndex=nearIndex; added=false; return; end
if ~parentOK, newIndex=nearIndex; added=false; return; end
tree.nodes(end+1,:)=qNew; tree.parent(end+1,1)=parent; tree.cost(end+1,1)=cost;
newIndex=size(tree.nodes,1); added=true;
if config.rewire, tree=rewireTree(tree,newIndex,config.useSequentialRewire); end
end

function [parent,cost,found] = chooseClassifiedParent(tree,qNew,nearest)
[parent,cost,found]=chooseParent(tree,qNew,nearest);
if ~config.useBacktracking || ~found, return; end
xmin=parent;
for depth=config.backtrackDepth:-1:0
    if abortRequested(), return; end
    ancestor=xmin;
    for k=1:depth
        if ancestor==0, break; end
        ancestor=tree.parent(ancestor);
    end
    if ancestor>0 && edgeFree(tree.nodes(ancestor,:),qNew)
        if abortRequested(), return; end
        candidate=tree.cost(ancestor)+jointDistance(tree.nodes(ancestor,:),qNew,config.jointWeights);
        if candidate <= cost+1e-12
            parent=ancestor; cost=candidate; stats.goalTreeConnectionSuccesses=stats.goalTreeConnectionSuccesses; return;
        end
    end
end
end

function [qOffset,ok] = tryOffset(qNear,qBlocked)
qOffset=qBlocked; ok=false;
radius=jointDistance(qNear,qBlocked,config.jointWeights)*config.offsetRadiusScale;
for k=1:config.offsetAttempts
    if abortRequested(), return; end
    stats.offsetAttemptCount=stats.offsetAttemptCount+1;
    direction=randn(1,6); direction=direction/max(norm(direction),eps);
    candidate=qNear+radius*(0.5+0.5*rand)*direction;
    if ~jointWithinLimits(candidate,config.jointLimits), continue; end
    if edgeFree(qNear,candidate)
        if abortRequested(), return; end
        qOffset=candidate; ok=true; stats.offsetSuccessCount=stats.offsetSuccessCount+1; return;
    end
end
end

function type = nodeTypeFor(q)
[canStart,~]=edgeFree(config.start,q);
[canGoal,~]=edgeFree(q,config.goal);
if canStart && canGoal, type=1;
elseif canStart, type=2;
elseif canGoal, type=3;
else, type=4;
end
end

function [index,connected] = bestConnection(tree,qNew)
index=NaN; connected=false; radius=nearRadius(size(tree.nodes,1)+1);
candidate=nearIndices(tree.nodes,qNew,radius);
if isempty(candidate), [near,~]=nearestIndex(tree.nodes,qNew); candidate=near; end
best=Inf;
for idx=reshape(candidate,1,[])
    if abortRequested(), return; end
    stats.goalTreeConnectionChecks=stats.goalTreeConnectionChecks+1;
    if edgeFree(qNew,tree.nodes(idx,:))
        stats.goalTreeConnectionSuccesses=stats.goalTreeConnectionSuccesses+1;
        cost=tree.cost(idx)+jointDistance(qNew,tree.nodes(idx,:),config.jointWeights);
        if cost<best, best=cost; index=idx; connected=true; end
    end
end
end

function [index,connected] = bestGoalTreeConnection(tree,newIndex,goalTree)
index=NaN; connected=false; best=Inf; qNew=tree.nodes(newIndex,:);
for idx=2:size(goalTree.nodes,1)
    if abortRequested(), return; end
    stats.goalTreeConnectionChecks=stats.goalTreeConnectionChecks+1;
    if edgeFree(qNew,goalTree.nodes(idx,:))
        stats.goalTreeConnectionSuccesses=stats.goalTreeConnectionSuccesses+1;
        cost=tree.cost(newIndex)+jointDistance(qNew,goalTree.nodes(idx,:),config.jointWeights)+goalTree.costToGoal(idx);
        if cost<best, best=cost; index=idx; connected=true; end
    end
end
end

function [idx,d] = nearestIndex(nodes,q)
d=zeros(size(nodes,1),1);
for i=1:size(nodes,1)
    if abortRequested(), idx=1; d=Inf(size(nodes,1),1); return; end
    d(i)=jointDistance(nodes(i,:),q,config.jointWeights);
end
[~,idx]=min(d);
end

function ids = nearIndices(nodes,q,radius)
ids=zeros(0,1);
for i=1:size(nodes,1)
    if abortRequested(), return; end
    if jointDistance(nodes(i,:),q,config.jointWeights)<=radius
        ids(end+1,1)=i; %#ok<AGROW>
    end
end
end

function radius=nearRadius(n)
radius=min(config.nearGamma*(log(max(2,n))/n)^(1/6),config.nearRadiusMaxRad);
end

function path=twoTreePath(tree1,i1,tree2,i2)
a=traceTree(tree1,i1); b=flipud(traceTree(tree2,i2));
if ~isempty(a) && ~isempty(b) && jointDistance(a(end,:),b(1,:),config.jointWeights)<1e-12
    b(1,:)=[];
end
path=[a;b];
end

function path=traceTree(tree,index)
path=zeros(0,6);
while index>0
    path(end+1,:)=tree.nodes(index,:); %#ok<AGROW>
    index=tree.parent(index);
end
path=flipud(path);
end

function tree=assignParent(tree,index,parent)
if abortRequested(), return; end
tree.parent(index)=parent;
tree.cost(index)=tree.cost(parent)+jointDistance(tree.nodes(parent,:),tree.nodes(index,:),config.jointWeights);
for child=find(tree.parent==index).'
    if abortRequested(), return; end
    if child~=index, tree=assignParent(tree,child,index); end
end
end

function tf=isDescendant(tree,ancestor,node)
tf=false; current=node;
while current>0
    if abortRequested(), return; end
    if current==ancestor, tf=true; return; end
    current=tree.parent(current);
end
end

function [a,b,swappedOut]=swapTrees(a,b,swappedIn)
tmp=a; a=b; b=tmp; swappedOut=~swappedIn;
end

function [free,detail,actualCheck]=stateFree(q)
actualCheck=false;
if abortRequested()
    free=false; detail=struct('environmentCollision',false,'selfCollision',false);
    return;
end
if ~jointWithinLimits(q,config.jointLimits)
    free=false; detail=struct('environmentCollision',false,'selfCollision',false); return;
end
key=stateKey(q);
if isKey(stateCache,key)
    cached=stateCache(key);
    free=cached.free;
    detail=cached.detail;
    stats.stateCacheHits=stats.stateCacheHits+1;
    return;
end
stats.collisionChecks=stats.collisionChecks+1;
actualCheck=true;
[free,detail]=collisionFreeJointState(config.robot,q,config.environment, ...
    config.selfCollisionSkipPairs);
stateCache(key)=struct('free',free,'detail',detail);
end

function [free,detail]=edgeFree(q1,q2)
if abortRequested()
    free=false; detail=struct(); return;
end
if ~jointWithinLimits(q1,config.jointLimits) || ~jointWithinLimits(q2,config.jointLimits)
    free=false; detail=struct(); return;
end
stats.edgeChecks=stats.edgeChecks+1;
key1=stateKey(q1); key2=stateKey(q2);
orderedKeys=sort({key1,key2});
edgeKey=[orderedKeys{1} '|' orderedKeys{2}];
if isKey(edgeCache,edgeKey)
    cached=edgeCache(edgeKey);
    free=cached.free;
    idx=cached.idx;
    count=cached.count;
    timedOut=false;
    stats.edgeCacheHits=stats.edgeCacheHits+1;
    detail=struct('collisionIndex',idx,'sampleCount',count);
    return;
end
opts=struct('wrapJoints',true);
opts.selfCollisionSkipPairs=config.selfCollisionSkipPairs;
opts.stateCheckFcn=@stateFree;
[free,idx,count,timedOut]=collisionFreeJointEdge(config.robot,q1,q2,config.environment, ...
    config.jointEdgeResolutionRad,opts);
if stats.timedOut
    timedOut=true;
end
if timedOut
    stats.timedOut=true;
end
if ~timedOut
    edgeCache(edgeKey)=struct('free',free,'idx',idx,'count',count);
end
detail=struct('collisionIndex',idx,'sampleCount',count);
end

function valid=validatePath(path)
valid=false;
if isempty(path) || size(path,2)~=6 || any(~isfinite(path(:))), return; end
if max(abs(path(1,:)-config.start))>1e-8 || max(abs(path(end,:)-config.goal))>1e-8, return; end
for i=1:size(path,1)
    if ~jointWithinLimits(path(i,:),config.jointLimits), return; end
end
for i=1:size(path,1)-1
    [free,~]=edgeFree(path(i,:),path(i+1,:));
    if ~free, return; end
    if abortRequested(), return; end
end
valid=true;
end

function tf=abortRequested()
captureCheckpoints(false);
if stats.timedOut
    tf=true;
    return;
end
if isfinite(config.maxPlanningTimeSec) && toc(timer)>=config.maxPlanningTimeSec
    stats.timedOut=true;
    tf=true;
else
    tf=false;
end
end

function registerCertifiedCandidate(candidate)
candidate=removeDuplicateRows(candidate);
if isempty(firstSolutionPath), firstSolutionPath=candidate; end
candidateCost=pathCost(candidate);
if candidateCost < bestCost-1e-12
    bestPath=candidate;
    bestCost=candidateCost;
    bestElapsed=toc(timer);
end
captureCheckpoints(false);
end

function captureCheckpoints(forceAll)
if isempty(config.checkpointTimesSec), return; end
elapsed=toc(timer);
while stats.nextCheckpointIndex<=numel(config.checkpointTimesSec) && ...
        (forceAll || elapsed>=config.checkpointTimesSec(stats.nextCheckpointIndex))
    idx=stats.nextCheckpointIndex;
    if isempty(bestPath)
        stats.bestPathAtCheckpoint{idx}=zeros(0,6);
        stats.bestCostAtCheckpoint(idx)=NaN;
    else
        stats.bestPathAtCheckpoint{idx}=bestPath;
        stats.bestCostAtCheckpoint(idx)=bestCost;
    end
    stats.nextCheckpointIndex=idx+1;
end
end

function key=stateKey(q)
key=sprintf('%.15g,',reshape(double(q),1,[]));
end

function p=removeDuplicateRows(p)
if isempty(p), return; end
keep=[true; vecnorm(diff(p,1,1),2,2)>1e-12]; p=p(keep,:);
end

function c=pathCost(p)
c=0;
for i=1:size(p,1)-1, c=c+jointDistance(p(i,:),p(i+1,:),config.jointWeights); end
end

function c=pathCostOrNaN(p)
if isempty(p), c=NaN; else, c=pathCost(p); end
end

function m=pathMotion(p)
m=zeros(1,6);
for i=1:size(p,1)-1, m=m+abs(jointWrapDelta(p(i,:),p(i+1,:))); end
end

function s=maxPathStep(p)
if size(p,1)<2, s=0; return; end
s=0;
for i=1:size(p,1)-1, s=max(s,max(abs(jointWrapDelta(p(i,:),p(i+1,:))))); end
end
end
