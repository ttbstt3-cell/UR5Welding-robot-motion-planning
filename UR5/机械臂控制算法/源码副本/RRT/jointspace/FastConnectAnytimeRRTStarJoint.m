function result = FastConnectAnytimeRRTStarJoint(config)
%FastConnectAnytimeRRTStarJoint CBi-RRT* fast-connect plus anytime RRT* planner.
%
% Stage 1 is the current joint-space RRT-Connect Extend/Connect procedure.
% After the first connection, Stage 2 keeps both trees and uses the same
% joint-space ChooseParent/Rewire semantics as Bi-RRT*.  This file is an
% independent planner: existing RRTConnectJoint and BiRRTStarJoint are not
% modified.

if nargin < 1, config = struct(); end
config = normalizeJointPlannerConfig(config);
rng(config.seed,'twister');

timer = tic;
stage = 1;
stateCache = containers.Map('KeyType','char','ValueType','any');
edgeCache = containers.Map('KeyType','char','ValueType','any');
stats = localStats(config);
result = localEmptyResult(config);
stage1CollisionChecks = 0;
stage2CollisionChecks = 0;
stage1EdgeChecks = 0;
stage2EdgeChecks = 0;
stage1EdgeCollisionChecks = 0;
stage2EdgeCollisionChecks = 0;
firstSolutionElapsed = NaN;
bestPath = zeros(0,6);
bestCost = Inf;
bestElapsed = NaN;

[startFree,startDetail] = stateFree(config.start);
[goalFree,goalDetail] = stateFree(config.goal);
if ~startFree || ~goalFree
    result = finishResult(zeros(0,6),[],NaN,0,[],{});
    result.startStateValid = startFree;
    result.goalStateValid = goalFree;
    result.startStateDiagnostic = startDetail;
    result.goalStateDiagnostic = goalDetail;
    if ~startFree && ~goalFree
        result.terminationReason = 'invalid_start_and_goal_collision';
    elseif ~startFree
        result.terminationReason = 'invalid_start_collision';
    else
        result.terminationReason = 'invalid_goal_collision';
    end
    result.failureReason = result.terminationReason;
    return;
end

% --------------------------- Stage 1 ---------------------------
% Keep this sequence aligned with jointPlannerCore/runConnect: the same
% random sample, alternating tree exchange, one-step Extend and Connect
% loop are used before any RRT* operation is enabled.
trees = {newTree(config.start),newTree(config.goal)};
swapped = false;
firstPath = zeros(0,6);
firstConnection = [NaN NaN];
firstIteration = NaN;
totalIterations = 0;

for iter = 1:config.maxIterations
    if abortRequested(), break; end
    totalIterations = iter;
    stats.stage1Iterations = stats.stage1Iterations+1;
    stats.samples = stats.samples+1;
    target = sampleJointConfig(config.jointLimits,config.goal,config.goalBias);
    [trees{1},idx,status] = extendTree(trees{1},target,false);
    if abortRequested(), break; end
    if ~strcmp(status,'trapped')
        [trees{2},otherIdx,statusOther] = connectTree( ...
            trees{2},trees{1}.nodes(idx,:));
        if abortRequested(), break; end
        if strcmp(statusOther,'reached')
            if ~swapped
                candidatePath = twoTreePath(trees{1},idx,trees{2},otherIdx);
                candidateConnection = [idx otherIdx];
            else
                candidatePath = twoTreePath(trees{2},otherIdx,trees{1},idx);
                candidateConnection = [otherIdx idx];
            end
            % This is the existing planner-resolution path check.  No
            % strict candidate-certification callback is introduced here.
            if validatePath(candidatePath)
                firstPath = candidatePath;
                firstConnection = candidateConnection;
                firstIteration = iter;
                registerImprovement(firstPath,true);
                break;
            end
        end
    end
    [trees{1},trees{2},swapped] = swapTrees(trees{1},trees{2},swapped);
end

if isempty(firstPath)
    result = finishResult([],[],firstIteration,totalIterations,[],trees);
    return;
end

% Normalize the two trees to physical start/goal orientation before Stage 2.
if swapped
    startTree = trees{2};
    goalTree = trees{1};
else
    startTree = trees{1};
    goalTree = trees{2};
end

% Keep the first connection as the initial best connection.  Its path is
% already stored, while later rewiring may improve either branch.
bestConnection = firstConnection;

% --------------------------- Stage 2 ---------------------------
stage = 2;
stage2Iteration = 0;
while totalIterations < config.maxIterations && ~abortRequested()
    stage2Iteration = stage2Iteration+1;
    stats.stage2Iterations = stage2Iteration;
    totalIterations = totalIterations+1;
    stats.samples = stats.samples+1;
    activeTreeIndex = 1+mod(stage2Iteration-1,2);
    target = sampleJointConfig(config.jointLimits,config.goal,config.goalBias);

    if activeTreeIndex == 1
        [startTree,newIndex,added] = extendStar(startTree,target);
        activeTree = startTree;
        otherTree = goalTree;
    else
        [goalTree,newIndex,added] = extendStar(goalTree,target);
        activeTree = goalTree;
        otherTree = startTree;
    end
    if abortRequested(), break; end

    % A rewire can improve the currently stored connection without adding a
    % new cross-tree edge.  Evaluate it before new connection candidates.
    if all(isfinite(bestConnection))
        currentCost = connectionCost(startTree,bestConnection(1), ...
            goalTree,bestConnection(2));
        if currentCost < bestCost-1e-12
            candidatePath = twoTreePath(startTree,bestConnection(1), ...
                goalTree,bestConnection(2));
            registerImprovement(candidatePath,false);
        end
    end

    if added
        qNew = activeTree.nodes(newIndex,:);
        candidateIndices = nearIndices(otherTree.nodes,qNew, ...
            nearRadius(size(otherTree.nodes,1)+1));
        if isempty(candidateIndices)
            [nearestOther,~] = nearestIndex(otherTree.nodes,qNew);
            candidateIndices = nearestOther;
        end
        bestCandidateCost = Inf;
        bestCandidateIndex = NaN;
        for otherIndex = reshape(candidateIndices,1,[])
            if abortRequested(), break; end
            otherPoint = otherTree.nodes(otherIndex,:);
            if ~edgeFree(qNew,otherPoint), continue; end
            if activeTreeIndex == 1
                candidateCost = connectionCost(startTree,newIndex, ...
                    goalTree,otherIndex);
            else
                candidateCost = connectionCost(startTree,otherIndex, ...
                    goalTree,newIndex);
            end
            if candidateCost < bestCandidateCost-1e-12
                bestCandidateCost = candidateCost;
                bestCandidateIndex = otherIndex;
            end
        end
        if isfinite(bestCandidateIndex)
            if activeTreeIndex == 1
                candidateConnection = [newIndex bestCandidateIndex];
            else
                candidateConnection = [bestCandidateIndex newIndex];
            end
            candidatePath = twoTreePath(startTree,candidateConnection(1), ...
                goalTree,candidateConnection(2));
            candidateLength = pathCost(candidatePath);
            if candidateLength < bestCost-1e-12
                bestConnection = candidateConnection;
                registerImprovement(candidatePath,false);
            end
        end
    end

    captureCheckpoints(false);
end

result = finishResult(firstPath,firstConnection,firstIteration, ...
    totalIterations,bestConnection,{startTree,goalTree});

% =====================================================================
% Nested functions.  The geometry and collision semantics below mirror the
% helpers in jointPlannerCore, while the stage counters are CBi-RRT* diagnostics.
% =====================================================================

function s = localStats(cfg)
s = struct('collisionChecks',0,'edgeChecks',0,'stateCacheHits',0, ...
    'edgeCacheHits',0,'rewireAttempts',0,'rewireSuccesses',0, ...
    'samples',0,'stage1Iterations',0,'stage2Iterations',0, ...
    'timedOut',false,'checkpointTimesSec',cfg.checkpointTimesSec, ...
    'nextCheckpointIndex',1,'bestPathAtCheckpoint', ...
    {cell(1,numel(cfg.checkpointTimesSec))}, ...
    'bestCostAtCheckpoint',NaN(1,numel(cfg.checkpointTimesSec)), ...
    'improvementEvents',struct('eventTimeSec',{}, ...
    'bestPathLengthL2Rad',{},'nodeCount',{}, ...
    'edgeCollisionChecks',{},'collisionConfigurationChecks',{}, ...
    'rewireAttempts',{},'rewireSuccesses',{}));
end

function r = localEmptyResult(cfg)
r = struct('algorithm','CBi-RRT*', ...
    'success',false,'path',zeros(0,6),'firstPath',zeros(0,6), ...
    'planningTimeSec',NaN,'planningTimeMs',NaN,'planningTime',NaN, ...
    'firstSolutionTimeSec',NaN,'firstSolutionTimeMs',NaN, ...
    'firstSolutionTime',NaN,'firstSolutionIteration',NaN, ...
    'firstPathLengthL2Rad',NaN,'finalBestPathLengthL2Rad',NaN, ...
    'jointPathCostL2',NaN,'totalJointMotionPerAxis',NaN(1,6), ...
    'totalJointMotionRad',NaN,'stage1Iterations',0,'stage2Iterations',0, ...
    'stage1CollisionChecks',0,'stage2CollisionChecks',0, ...
    'stage1EdgeCollisionChecks',0,'stage2EdgeCollisionChecks',0, ...
    'collisionConfigurationChecks',0,'edgeCollisionChecks',0, ...
    'rewireAttempts',0,'rewireSuccesses',0,'numberOfImprovements',0, ...
    'improvementEvents',stats.improvementEvents, ...
    'bestPathAtCheckpoint',{cell(1,numel(cfg.checkpointTimesSec))}, ...
    'bestPathLengthAtCheckpoint',NaN(1,numel(cfg.checkpointTimesSec)), ...
    'checkpointTimesSec',cfg.checkpointTimesSec,'nodeCount',0, ...
    'treeANodes',0,'treeBNodes',0,'samples',0,'timedOut',false, ...
    'terminationReason','not_started','plannerResolutionDeg', ...
    rad2deg(cfg.jointEdgeResolutionRad),'validationResolutionDeg',0.5, ...
    'maxPlanningTimeSec',cfg.maxPlanningTimeSec,'seed',cfg.seed, ...
    'config',cfg,'jointLimits',cfg.jointLimits);
end

function r = finishResult(first,firstConnection,firstIter,totalIter,~,treePair)
elapsed = toc(timer);
captureCheckpoints(true);
if isempty(bestPath), chosen = first; else, chosen = bestPath; end
r = result;
r.path = chosen;
r.firstPath = first;
r.success = ~isempty(chosen);
r.planningTimeSec = elapsed;
r.planningTimeMs = elapsed*1000;
r.planningTime = elapsed;
r.firstSolutionIteration = firstIter;
r.firstSolutionTimeSec = getFirstSolutionElapsed(first);
r.firstSolutionTimeMs = r.firstSolutionTimeSec*1000;
r.firstSolutionTime = r.firstSolutionTimeSec;
r.firstPathLengthL2Rad = pathCostOrNaN(first);
r.finalBestPathLengthL2Rad = pathCostOrNaN(chosen);
r.jointPathCostL2 = r.finalBestPathLengthL2Rad;
if ~isempty(chosen)
    r.totalJointMotionPerAxis = pathMotion(chosen);
    r.totalJointMotionRad = sum(r.totalJointMotionPerAxis);
end
r.stage1Iterations = stats.stage1Iterations;
r.stage2Iterations = stats.stage2Iterations;
r.stage1CollisionChecks = stageCollisionChecks(1);
r.stage2CollisionChecks = stageCollisionChecks(2);
r.stage1EdgeCollisionChecks = stageEdgeChecks(1);
r.stage2EdgeCollisionChecks = stageEdgeChecks(2);
r.collisionConfigurationChecks = stats.collisionChecks;
r.edgeCollisionChecks = stats.edgeChecks;
r.rewireAttempts = stats.rewireAttempts;
r.rewireSuccesses = stats.rewireSuccesses;
r.numberOfImprovements = numel(stats.improvementEvents);
r.improvementEvents = stats.improvementEvents;
r.bestPathAtCheckpoint = stats.bestPathAtCheckpoint;
r.bestPathLengthAtCheckpoint = stats.bestCostAtCheckpoint;
r.checkpointTimesSec = config.checkpointTimesSec;
r.nodeCount = treeNodeCount(treePair);
if isempty(treePair)
    r.treeANodes = 0;
    r.treeBNodes = 0;
else
    r.treeANodes = size(treePair{1}.nodes,1);
    r.treeBNodes = size(treePair{2}.nodes,1);
end
r.samples = stats.samples;
r.timedOut = stats.timedOut;
if stats.timedOut
    r.terminationReason = 'timeout_with_solution';
elseif r.success && totalIter >= config.maxIterations
    r.terminationReason = 'maxIterations_with_solution';
elseif r.success
    r.terminationReason = 'success';
else
    r.terminationReason = 'timeout';
end
% Keep this argument visible in the result for auditability.
if nargin >= 2, r.firstConnection = firstConnection; end
end

function t = getFirstSolutionElapsed(first)
if isempty(first), t=NaN; else, t=firstSolutionElapsed; end
end

function c = stageCollisionChecks(which)
if which==1, c=stage1CollisionChecks; else, c=stage2CollisionChecks; end
end

function c = stageEdgeChecks(which)
if which==1, c=stage1EdgeChecks; else, c=stage2EdgeChecks; end
end

function n = treeNodeCount(pair)
if isempty(pair), n=0; else, n=size(pair{1}.nodes,1)+size(pair{2}.nodes,1); end
end

function tree = newTree(q)
tree = struct('nodes',reshape(double(q),1,6),'parent',0,'cost',0);
end

function [tree,index,status] = extendTree(tree,target,useStar)
if abortRequested(), index=1; status='trapped'; return; end
[nearIndex,~] = nearestIndex(tree.nodes,target);
qNear=tree.nodes(nearIndex,:);
[qNew,reached,valid] = steerJoint(qNear,target,config.maxJointStepRad,config.jointLimits);
index=nearIndex; status='trapped';
if ~valid || jointDistance(qNew,qNear,config.jointWeights)<1e-12, return; end
if ~edgeFree(qNear,qNew), return; end
if abortRequested(), return; end
parent=nearIndex;
cost=tree.cost(nearIndex)+jointDistance(qNear,qNew,config.jointWeights);
if useStar
    [parent,cost,parentFound]=chooseParent(tree,qNew,nearIndex);
    if abortRequested() || ~parentFound, return; end
end
tree.nodes(end+1,:)=qNew;
tree.parent(end+1,1)=parent;
tree.cost(end+1,1)=cost;
index=size(tree.nodes,1);
status='advanced';
if reached, status='reached'; end
if useStar, tree=rewireTree(tree,index); end
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
candidates=unique([nearest;nearIndices(tree.nodes,qNew,radius)]);
parent=nearest;
bestCost=tree.cost(nearest)+jointDistance(tree.nodes(nearest,:),qNew,config.jointWeights);
found=false;
for candidateNodeIndex=reshape(candidates,1,[])
    if abortRequested(), return; end
    candidateCost=tree.cost(candidateNodeIndex)+jointDistance( ...
        tree.nodes(candidateNodeIndex,:),qNew,config.jointWeights);
    if candidateCost < bestCost-1e-12 && ...
            edgeFree(tree.nodes(candidateNodeIndex,:),qNew)
        parent=candidateNodeIndex; bestCost=candidateCost; found=true;
    end
end
if edgeFree(tree.nodes(nearest,:),qNew) && ~abortRequested(), found=true; end
end

function tree=rewireTree(tree,newIndex)
radius=nearRadius(size(tree.nodes,1));
near=nearIndices(tree.nodes,tree.nodes(newIndex,:),radius);
for rewireNodeIndex=reshape(near,1,[])
    if abortRequested(), return; end
    if rewireNodeIndex==newIndex || rewireNodeIndex==tree.parent(newIndex), continue; end
    stats.rewireAttempts=stats.rewireAttempts+1;
    candidateCost=tree.cost(newIndex)+jointDistance( ...
        tree.nodes(newIndex,:),tree.nodes(rewireNodeIndex,:),config.jointWeights);
    if candidateCost >= tree.cost(rewireNodeIndex)-1e-12 || ...
            isDescendant(tree,rewireNodeIndex,newIndex)
        continue;
    end
    if edgeFree(tree.nodes(newIndex,:),tree.nodes(rewireNodeIndex,:))
        tree=assignParent(tree,rewireNodeIndex,newIndex);
        stats.rewireSuccesses=stats.rewireSuccesses+1;
    end
end
end

function [nearestIndexOut,d] = nearestIndex(nodes,q)
d=Inf(size(nodes,1),1);
for i=1:size(nodes,1)
    if abortRequested(), nearestIndexOut=1; return; end
    d(i)=jointDistance(nodes(i,:),q,config.jointWeights);
end
[~,nearestIndexOut]=min(d);
end

function ids=nearIndices(nodes,q,radius)
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

function total=connectionCost(st,si,gt,gi)
total=st.cost(si)+jointDistance(st.nodes(si,:),gt.nodes(gi,:),config.jointWeights)+gt.cost(gi);
end

function path=twoTreePath(st,si,gt,gi)
a=traceTree(st,si); b=flipud(traceTree(gt,gi));
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
detail=struct('environmentCollision',false,'selfCollision',false);
if abortRequested(), free=false; return; end
if ~jointWithinLimits(q,config.jointLimits), free=false; return; end
key=stateKey(q);
if isKey(stateCache,key)
    cached=stateCache(key); free=cached.free; detail=cached.detail;
    stats.stateCacheHits=stats.stateCacheHits+1; return;
end
stats.collisionChecks=stats.collisionChecks+1;
actualCheck=true;
if stage==1, stage1CollisionChecks=stage1CollisionChecks+1; ...
else, stage2CollisionChecks=stage2CollisionChecks+1; end
[free,detail]=collisionFreeJointState(config.robot,q,config.environment, ...
    config.selfCollisionSkipPairs,config.environmentCollisionPolicy);
stateCache(key)=struct('free',free,'detail',detail);
end

function [free,detail]=edgeFree(q1,q2)
detail=struct('collisionIndex',NaN,'sampleCount',0);
if abortRequested() || ~jointWithinLimits(q1,config.jointLimits) || ...
        ~jointWithinLimits(q2,config.jointLimits)
    free=false; return;
end
stats.edgeChecks=stats.edgeChecks+1;
if stage==1
    stage1EdgeChecks=stage1EdgeChecks+1;
    stage1EdgeCollisionChecks=stage1EdgeCollisionChecks+1;
else
    stage2EdgeChecks=stage2EdgeChecks+1;
    stage2EdgeCollisionChecks=stage2EdgeCollisionChecks+1;
end
key1=stateKey(q1); key2=stateKey(q2); ordered=sort({key1,key2});
edgeKey=[ordered{1} '|' ordered{2}];
if isKey(edgeCache,edgeKey)
    cached=edgeCache(edgeKey); free=cached.free; detail=cached.detail;
    stats.edgeCacheHits=stats.edgeCacheHits+1; return;
end
opts=struct();
opts.wrapJoints=true;
opts.selfCollisionSkipPairs=config.selfCollisionSkipPairs;
opts.stateCheckFcn=@stateFree;
opts.timeoutFcn=@abortRequested;
[free,collisionSampleIndex,count,timedOut]=collisionFreeJointEdge(config.robot,q1,q2, ...
    config.environment,config.jointEdgeResolutionRad,opts);
detail=struct('collisionIndex',collisionSampleIndex,'sampleCount',count);
if timedOut, stats.timedOut=true; end
if ~timedOut, edgeCache(edgeKey)=struct('free',free,'detail',detail); end
end

function valid=validatePath(path)
valid=false;
if isempty(path) || size(path,2)~=6 || any(~isfinite(path(:))), return; end
if max(abs(path(1,:)-config.start))>1e-8 || max(abs(path(end,:)-config.goal))>1e-8, return; end
for i=1:size(path,1)
    if ~jointWithinLimits(path(i,:),config.jointLimits), return; end
end
for i=1:size(path,1)-1
    if ~edgeFree(path(i,:),path(i+1,:)), return; end
    if abortRequested(), return; end
end
valid=true;
end

function tf=abortRequested()
captureCheckpoints(false);
if stats.timedOut, tf=true; return; end
if isfinite(config.maxPlanningTimeSec) && toc(timer)>=config.maxPlanningTimeSec
    stats.timedOut=true; tf=true;
else
    tf=false;
end
end

function registerImprovement(path,isFirst)
path=removeDuplicateRows(path);
candidateCost=pathCost(path);
if ~isFirst && candidateCost >= bestCost-1e-12, return; end
bestPath=path; bestCost=candidateCost; bestElapsed=toc(timer);
if isFirst, firstSolutionElapsed=bestElapsed; end
event=struct('eventTimeSec',bestElapsed,'bestPathLengthL2Rad',candidateCost, ...
    'nodeCount',currentNodeCount(),'edgeCollisionChecks',stats.edgeChecks, ...
    'collisionConfigurationChecks',stats.collisionChecks, ...
    'rewireAttempts',stats.rewireAttempts,'rewireSuccesses',stats.rewireSuccesses);
stats.improvementEvents(end+1)=event;
captureCheckpoints(false);
end

function captureCheckpoints(forceAll)
if isempty(config.checkpointTimesSec), return; end
elapsed=toc(timer);
while stats.nextCheckpointIndex<=numel(config.checkpointTimesSec) && ...
        (forceAll || elapsed>=config.checkpointTimesSec(stats.nextCheckpointIndex))
    checkpointIndex=stats.nextCheckpointIndex;
    if isempty(bestPath)
        stats.bestPathAtCheckpoint{checkpointIndex}=zeros(0,6);
        stats.bestCostAtCheckpoint(checkpointIndex)=NaN;
    else
        stats.bestPathAtCheckpoint{checkpointIndex}=bestPath;
        stats.bestCostAtCheckpoint(checkpointIndex)=bestCost;
    end
    stats.nextCheckpointIndex=checkpointIndex+1;
end
end

function n=currentNodeCount()
% Stage 1 and Stage 2 always keep the current pair in local variables.
if exist('startTree','var') && exist('goalTree','var')
    n=size(startTree.nodes,1)+size(goalTree.nodes,1);
else
    n=size(trees{1}.nodes,1)+size(trees{2}.nodes,1);
end
end

function key=stateKey(q), key=sprintf('%.15g,',reshape(double(q),1,[])); end
function p=removeDuplicateRows(p)
if isempty(p), return; end
p=p([true;vecnorm(diff(p,1,1),2,2)>1e-12],:);
end
function c=pathCost(p)
c=0; for i=1:size(p,1)-1, c=c+jointDistance(p(i,:),p(i+1,:),config.jointWeights); end
end
function c=pathCostOrNaN(p)
if isempty(p), c=NaN; else, c=pathCost(p); end
end
function m=pathMotion(p)
m=zeros(1,6); for i=1:size(p,1)-1, m=m+abs(jointWrapDelta(p(i,:),p(i+1,:))); end
end

end
