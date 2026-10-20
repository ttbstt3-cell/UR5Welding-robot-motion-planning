function result = replan_weld_actual_axis_gao_ar_rrtstar(options)
%REPLAN_WELD_ACTUAL_AXIS_GAO_AR_RRTSTAR
% Fair, independent Gao-style angular-redundancy RRT* baseline.
%
% The implementation keeps the welding task constraint explicit: layer i
% is weld point i, while travel/roll are sampled in the allowed angular
% redundancy space.  IK, joint limits, tool-axis/welding-side constraints,
% node collisions, self-collision, and every interpolated edge are checked
% with the same functions and scene used by the validated planner.
%
% This is an experiment only.  It does not modify the stable planner, ROS2,
% CoppeliaSim, or the real robot.  It deliberately does not load a previous
% qPath, state graph, or checkpoint.

if nargin < 1 || isempty(options), options = struct(); end
defaults = struct( ...
    "numPoints",80,"endMarginM",0.025, ...
    "seamSpec",struct(), ...
    "travelBoundsDeg",[-20 20],"rollBoundsDeg",[0 360], ...
    "maxRollStepDeg",45,"maxTravelStepDeg",15,"maxJointStepDeg",15, ...
    "edgeResolutionDeg",3,"positionToleranceM",1e-4, ...
    "orientationToleranceDeg",0.5,"sideToleranceDeg",5,"sideMarginM",1e-3, ...
    "maxIterations",10000,"timeBudgetSec",30,"goalBias",0.20, ...
    "gaussianSigmaTravelDeg",6,"gaussianSigmaRollDeg",12, ...
    "samplesPerLayer",4, ...
    "maxStatesPerLayer",12,"maxStartStates",8,"maxIKSeeds",8, ...
    "seedPerturbDeg",8,"neighborParents",8, ...
    "checkSelfCollision",true,"reuseCandidateCache",true, ...
    "randomSeed",20260924,"animate",false, ...
    "outputFile","weld_actual_axis_gao_ar_rrtstar.mat");
options = localMergeOptions(defaults,options);

matlabDir = fileparts(mfilename("fullpath"));
fprintf("\n========================================\n");
fprintf("GAO-STYLE ANGULAR-REDUNDANCY RRT* BASELINE\n");
fprintf("========================================\n");
fprintf("Cold start: no previous qPath/state graph is loaded.\n");

% sync_ur5_joint_states is a scene-building script.  Running it here keeps
% this experiment on the exact same robot/tool/wire/environment model.
scene = localLoadScene(fullfile(matlabDir,"sync_ur5_joint_states.m"));
scene.selfCollisionSkipPairs = localStructuralSelfCollisionSkips(scene.robot);
scene.seamSpec = localSeamSpec(options.seamSpec);
scene.weldGunAxisWP = scene.seamSpec.gunAxisWP;
scene.weldGunAxisName = "measured_wire_axis_target";
scene.weldingSide = scene.seamSpec.description;
scene.weldingContactPolicy = "Wire/environment contact is hard; no target-seam exemption";
scene.ikSolver = inverseKinematics("RigidBodyTree",scene.robot);
scene.jointLimits = localRobotJointLimits(scene.robot);
if options.reuseCandidateCache
    scene.ikCandidateCache = containers.Map("KeyType","char","ValueType","any");
end

seam = localInferSeam(scene,options.numPoints,options.endMarginM,scene.seamSpec);
fprintf("Weld layers: %d, measured wire axis = [%.4f %.4f %.4f]\n", ...
    seam.numPoints,scene.weldGunAxisWP);
fprintf("Bounds: travel=[%.1f %.1f] deg, roll=[%.1f %.1f) deg\n", ...
    options.travelBoundsDeg,options.rollBoundsDeg);

rng(options.randomSeed,"twister");
algorithmTimer = tic;
stats = localEmptyStats();

% Layered RRT* state bank.  Each node is tied to exactly one weld point;
% this enforces the continuous welding position constraint from Gao et al.
layers = cell(1,seam.numPoints);
for i = 1:seam.numPoints, layers{i} = repmat(localEmptyNode(),0,1); end

% Initial redundancy sampling is intentionally small.  qMat0 is included
% as the first IK seed, but no result file from an earlier planner is read.
startTravel = unique(-15:15:15);
startRoll = unique(mod([0 45 90 135 180 225 270 315],360));
startSeeds = localSeedSet(scene.qMat0,scene.qMat0,options.maxIKSeeds,options.seedPerturbDeg);
for it = 1:numel(startTravel)
    for ir = 1:numel(startRoll)
        stats.poseSamples = stats.poseSamples + 1;
        [nodeCandidates,c] = localPoseCandidates(scene,seam,1,startTravel(it), ...
            startRoll(ir),startSeeds,options);
        stats = localAddStats(stats,c);
        for k = 1:numel(nodeCandidates)
            n = localNodeFromState(nodeCandidates(k),1,0,0);
            % The Gao comparison concerns the constrained 80-point weld
            % path itself.  Home->first-weld approach is a separate motion
            % task; do not discard a valid first weld state merely because
            % that free-space approach is not part of this experiment.
            n.cost = sum(localWrapRad(n.q-scene.qMat0).^2);
            layers{1}(end+1,1) = n; %#ok<AGROW>
        end
    end
end
layers{1} = localKeepLayer(layers{1},options.maxStartStates,scene.qMat0);
stats.acceptedStartStates = numel(layers{1});
fprintf("Start layer: %d collision-free states\n",numel(layers{1}));

firstFeasibleTime = NaN;
firstFeasibleIteration = NaN;
frontier = 1;
attemptsAtFrontier = 0;

for iter = 1:options.maxIterations
    if toc(algorithmTimer) >= options.timeBudgetSec, break; end
    stats.iterations = iter;
    if frontier >= seam.numPoints
        if isnan(firstFeasibleTime)
            firstFeasibleTime = toc(algorithmTimer);
            firstFeasibleIteration = iter;
        end
        break;
    end
    if isempty(layers{frontier}), break; end
    if attemptsAtFrontier >= options.samplesPerLayer
        if frontier < seam.numPoints && ~isempty(layers{frontier+1})
            frontier = frontier + 1;
            attemptsAtFrontier = 0;
            if frontier >= seam.numPoints
                firstFeasibleTime = toc(algorithmTimer);
                firstFeasibleIteration = iter;
                break;
            end
        else
            break;
        end
    end

    % Goal bias means a local sample at the current layer that preserves
    % the parent orientation; otherwise use regional Gaussian sampling.
    parentIdx = localSelectParent(layers{frontier},options);
    parent = layers{frontier}(parentIdx);
    if rand < options.goalBias
        % Goal-biased regional sample: steer one bounded step toward the
        % neutral redundancy centre, rather than freezing the parent pose.
        % This is analogous to the paper's task-space goal bias while still
        % respecting the weld-process angular step limits.
        travelSample = 0;
        rollSample = 0;
    else
        travelSample = parent.travelDeg + options.gaussianSigmaTravelDeg*randn;
        rollSample = parent.rollDeg + options.gaussianSigmaRollDeg*randn;
    end
    travelSample = min(options.travelBoundsDeg(2),max(options.travelBoundsDeg(1),travelSample));
    rollSample = localWrapRoll(rollSample,options.rollBoundsDeg);
    % Task-space steering in angular redundancy space.
    travelSample = localSteer(parent.travelDeg,travelSample,options.maxTravelStepDeg);
    rollSample = localSteerRoll(parent.rollDeg,rollSample,options.maxRollStepDeg);

    targetLayer = frontier + 1;
    attemptsAtFrontier = attemptsAtFrontier + 1;
    stats.poseSamples = stats.poseSamples + 1;
    qPred = parent.q;
    if isfield(parent,"velocityQ"), qPred = qPred + parent.velocityQ; end
    seeds = localSeedSet(qPred,parent.q,options.maxIKSeeds,options.seedPerturbDeg);
    [nodeCandidates,c] = localPoseCandidates(scene,seam,targetLayer,travelSample, ...
        rollSample,seeds,options);
    stats = localAddStats(stats,c);
    if isempty(nodeCandidates), continue; end

    % RRT* parent selection: choose the lowest-cost predecessor among a
    % bounded set, while every candidate edge receives the full collision
    % interpolation check.  This is a layered task-constrained analogue of
    % choose-parent/rewire; it does not discard layers or weld positions.
    prev = layers{frontier};
    neighborOrder = localNearestParents(prev,parent,qPred,options.neighborParents);
    bestNode = localEmptyNode(); bestCost = inf; bestParent = 0;
    for kc = 1:numel(nodeCandidates)
        cand = nodeCandidates(kc);
        for kp = 1:numel(neighborOrder)
            ip = neighborOrder(kp); pp = prev(ip);
            if ~localPoseStepOK(pp,cand,options), continue; end
            dq = localWrapRad(cand.q-pp.q);
            if max(abs(rad2deg(dq))) > options.maxJointStepDeg, continue; end
            stats.edgeTests = stats.edgeTests + 1;
            if ~localEdgeFree(scene,pp.q,cand.q,options), continue; end
            stats.edgeCollisionFree = stats.edgeCollisionFree + 1;
            edgeCost = sum(dq.^2) + 1e-4*( ...
                localAngleDistanceDeg(pp.rollDeg,cand.rollDeg)^2 + ...
                (pp.travelDeg-cand.travelDeg)^2);
            candidateCost = pp.cost + edgeCost;
            if candidateCost < bestCost
                bestCost = candidateCost; bestNode = cand; bestParent = ip;
            end
        end
    end
    if bestParent == 0, continue; end
    bestNode.parentIndex = bestParent;
    bestNode.cost = bestCost;
    bestNode.velocityQ = localWrapRad(bestNode.q-prev(bestParent).q);
    layers{targetLayer}(end+1,1) = bestNode; %#ok<AGROW>
    stats.acceptedStates = stats.acceptedStates + 1;
    layers{targetLayer} = localKeepLayer(layers{targetLayer}, ...
        options.maxStatesPerLayer,prev(bestParent).q);
    if mod(iter,100)==0 || frontier==seam.numPoints
        fprintf("iter %5d: frontier=%2d/%2d, states=%d, IK=%d, edges=%d\n", ...
            iter,frontier,seam.numPoints,sum(cellfun(@numel,layers)), ...
            stats.ikCalls,stats.edgeTests);
    end
end

if frontier >= seam.numPoints && ~isempty(layers{seam.numPoints})
    if isnan(firstFeasibleTime)
        firstFeasibleTime = toc(algorithmTimer);
        firstFeasibleIteration = stats.iterations;
    end
end

% If stochastic growth stalls, complete only the missing successor layers
% using the same sampled redundancy range.  This remains an independent
% cold-start Gao-style run and is not a fallback to the old planner.
if isempty(layers{seam.numPoints}) && frontier < seam.numPoints && ...
        toc(algorithmTimer) < options.timeBudgetSec
    for i = frontier+1:seam.numPoints
        if isempty(layers{i-1}), break; end
        [layers{i},s2] = localBridgeLayer(scene,seam,i,layers{i-1},options);
        stats = localAddStats(stats,s2);
        if isempty(layers{i}), break; end
    end
    if ~isempty(layers{seam.numPoints})
        firstFeasibleTime = toc(algorithmTimer);
        firstFeasibleIteration = stats.iterations;
    end
end

[qPath,stateIndices,complete,planCost] = localBacktrack(layers,seam.numPoints);
algorithmElapsedSec = toc(algorithmTimer);
nodeFree = false; edgeFree = false; maxJump = NaN;
if complete
    nodeFree = true; edgeFree = true;
    for i=1:size(qPath,1)
        nodeFree = nodeFree && localFullStateFree(scene,qPath(i,:));
    end
    jumps = localWrapRad(diff(qPath,1,1));
    maxJump = max(abs(rad2deg(jumps)),[],'all');
    for i=1:size(qPath,1)-1
        edgeFree = edgeFree && localEdgeFree(scene,qPath(i,:),qPath(i+1,:),options);
    end
end

tcpMetrics = localComputeTcpPathErrors(scene,layers,qPath,stateIndices,options);
result = struct();
sceneSave = scene;
for f = ["ikSolver","jointLimits","ikCandidateCache"]
    if isfield(sceneSave,f), sceneSave = rmfield(sceneSave,f); end
end
result.scene = sceneSave;
result.seam = seam;
result.layers = layers;
result.qPath = qPath;
result.stateIndices = stateIndices;
result.complete = complete;
result.nodeCollisionFree = nodeFree;
result.edgeCollisionFree = edgeFree;
result.maxJointJumpDeg = maxJump;
result.pathCost = planCost;
result.tcpMetrics = tcpMetrics;
result.options = options;
result.stats = stats;
result.firstFeasibleTimeSec = firstFeasibleTime;
result.firstFeasibleIteration = firstFeasibleIteration;
result.algorithmElapsedSec = algorithmElapsedSec;
result.approachPathPlanned = false;
result.exitPathPlanned = false;
result.note = "Gao-style constrained angular-redundancy RRT* baseline; first-feasible timing, not paper-to-paper timing.";

fprintf("\n--- Gao-AR-RRT* result ---\n");
fprintf("Complete 80-point path  : %d\n",complete);
fprintf("Node collision free     : %d\n",nodeFree);
fprintf("Edge collision free     : %d\n",edgeFree);
fprintf("Algorithm time          : %.6f s\n",algorithmElapsedSec);
fprintf("First feasible time     : %.6f s\n",firstFeasibleTime);
fprintf("Iterations              : %d\n",stats.iterations);
fprintf("IK calls / success     : %d / %d\n",stats.ikCalls,stats.ikPoseSuccess);
fprintf("Accepted states         : %d\n",stats.acceptedStates+stats.acceptedStartStates);
fprintf("Edge tests / free       : %d / %d\n",stats.edgeTests,stats.edgeCollisionFree);
fprintf("Max joint jump          : %.3f deg\n",maxJump);
fprintf("Max node TCP error      : %.6f mm\n",tcpMetrics.maxNodePositionErrorMm);
fprintf("Max segment TCP error   : %.6f mm\n",tcpMetrics.maxSegmentPositionErrorMm);
fprintf("Max segment TCP orient  : %.6f deg\n",tcpMetrics.maxSegmentOrientationErrorDeg);

outputFile = options.outputFile;
if ~isfile(outputFile), outputFile = fullfile(matlabDir,outputFile); end
result.outputFile = outputFile;
save(outputFile,"result","-v7.3");
fprintf("Saved result: %s\n",outputFile);
if options.animate && complete, localAnimate(scene,seam,qPath); end
end

function scene = localLoadScene(syncFile)
run(syncFile);
end

function seam = localInferSeam(scene,N,endMarginM,spec)
info = scene.workpieceProxyInfo;
bottom=localFindProxy(info,"WorkpieceCollision_bottom");
wallX0=localFindProxy(info,"WorkpieceCollision_wall_x0");
wallX1=localFindProxy(info,"WorkpieceCollision_wall_x1");
wallY1=localFindProxy(info,"WorkpieceCollision_wall_y1");
bottomMin=bottom.T_workpiece_proxy(1:3,4)-bottom.size_m(:)/2;
bottomMax=bottom.T_workpiece_proxy(1:3,4)+bottom.size_m(:)/2;
zSeam=bottomMax(3);
switch lower(string(spec.name))
    case "bottom_x_wall_y1"
        xMin=max(bottomMin(1),wallX0.T_workpiece_proxy(1,4)+wallX0.size_m(1)/2);
        xMax=min(bottomMax(1),wallX1.T_workpiece_proxy(1,4)-wallX1.size_m(1)/2);
        ySeam=wallY1.T_workpiece_proxy(2,4)-wallY1.size_m(2)/2;
        p0=[xMin+endMarginM;ySeam;zSeam]; p1=[xMax-endMarginM;ySeam;zSeam];
        if p1(1)<=p0(1), error("焊缝端部余量过大。"); end
    case "bottom_x_wall_x0"
        xSeam=wallX0.T_workpiece_proxy(1,4)+wallX0.size_m(1)/2;
        yMin=bottomMin(2);
        yMax=wallY1.T_workpiece_proxy(2,4)-wallY1.size_m(2)/2;
        p0=[xSeam;yMin+endMarginM;zSeam]; p1=[xSeam;yMax-endMarginM;zSeam];
        if p1(2)<=p0(2), error("焊缝端部余量过大。"); end
    case "bottom_x_wall_x1"
        xSeam=wallX1.T_workpiece_proxy(1,4)-wallX1.size_m(1)/2;
        yMin=bottomMin(2);
        yMax=wallY1.T_workpiece_proxy(2,4)-wallY1.size_m(2)/2;
        p0=[xSeam;yMin+endMarginM;zSeam]; p1=[xSeam;yMax-endMarginM;zSeam];
        if p1(2)<=p0(2), error("焊缝端部余量过大。"); end
    otherwise
        error("Unsupported seamSpec.name: %s",spec.name);
end
s=linspace(0,1,N).';
seam=struct(); seam.startWP=p0; seam.endWP=p1;
seam.tangentWP=(p1-p0)/norm(p1-p0);
seam.positionsWP=(1-s).*p0.'+s.*p1.';
P=(scene.T_MatBase_Workpiece*[seam.positionsWP,ones(N,1)].').';
seam.positionsMat=P(:,1:3); seam.numPoints=N; seam.plateThicknessM=bottom.size_m(3);
seam.bottomTopZ=zSeam; seam.freeSpaceCondition=spec.description;
end

function spec=localSeamSpec(in)
spec=struct("name","bottom_x_wall_y1", ...
    "gunAxisWP",[0;1;-1]/sqrt(2),"wallAxis",2, ...
    "wallInnerCoordinate",0.230,"freeDirection",-1, ...
    "description","U-interior: y < wall_y1 inner face, z > bottom top face");
if ~isempty(in)
    f=fieldnames(in);
    for k=1:numel(f), spec.(f{k})=in.(f{k}); end
end
spec.gunAxisWP=double(spec.gunAxisWP(:));
spec.gunAxisWP=spec.gunAxisWP/norm(spec.gunAxisWP);
end

function [states,stats] = localBridgeLayer(scene,seam,i,prev,options)
stats=localEmptyStats(); states=repmat(localEmptyNode(),0,1);
for pass=1:2
    retry=options;
    if pass==2
        % Gao-style local angular redundancy recovery: only a failed layer
        % receives more IK branches; normal layers stay bounded and cheap.
        retry.maxIKSeeds=max(options.maxIKSeeds,16);
        retry.seedPerturbDeg=max(options.seedPerturbDeg,15);
    end
    for ip=1:numel(prev)
        p=prev(ip);
        for dT=[-retry.maxTravelStepDeg 0 retry.maxTravelStepDeg]
            for dR=[-retry.maxRollStepDeg 0 retry.maxRollStepDeg]
                t=min(retry.travelBoundsDeg(2),max(retry.travelBoundsDeg(1),p.travelDeg+dT));
                r=localWrapRoll(p.rollDeg+dR,retry.rollBoundsDeg);
                seeds=localSeedSet(p.q,p.q,retry.maxIKSeeds,retry.seedPerturbDeg);
                [cand,c]=localPoseCandidates(scene,seam,i,t,r,seeds,retry); stats=localAddStats(stats,c);
                for k=1:numel(cand)
                    n=localNodeFromState(cand(k),i,ip,p.cost); n.velocityQ=localWrapRad(n.q-p.q);
                    if ~localPoseStepOK(p,n,retry), continue; end
                    dq=localWrapRad(n.q-p.q); if max(abs(rad2deg(dq)))>retry.maxJointStepDeg, continue; end
                    stats.edgeTests=stats.edgeTests+1;
                    if ~localEdgeFree(scene,p.q,n.q,retry), continue; end
                    stats.edgeCollisionFree=stats.edgeCollisionFree+1;
                    n.cost=p.cost+sum(dq.^2)+1e-4*(dR^2+dT^2);
                    states(end+1,1)=n; %#ok<AGROW>
                end
            end
        end
    end
    if ~isempty(states), break; end
end
if isempty(states)
    % Regional Gaussian sampling can miss a disconnected angular island.
    % On that failed layer only, use a bounded regional grid as the paper's
    % Gaussian-region recovery analogue.  The grid is still subject to the
    % same max pose step, IK, node collision, and interpolated edge checks.
    [regional,regionalStats] = localRegionalRecoveryLayer( ...
        scene,seam,i,prev,options);
    states = regional;
    stats = localAddStats(stats,regionalStats);
end
states=localKeepLayer(states,options.maxStatesPerLayer,scene.qMat0);
end

function [states,stats] = localRegionalRecoveryLayer(scene,seam,i,prev,options)
stats=localEmptyStats(); states=repmat(localEmptyNode(),0,1);
travelGrid=options.travelBoundsDeg(1):options.maxTravelStepDeg:options.travelBoundsDeg(2);
rollGrid=options.rollBoundsDeg(1):options.maxRollStepDeg:options.rollBoundsDeg(2)-eps;
for ip=1:numel(prev)
    p=prev(ip);
    seeds=localSeedSet(p.q,p.q,max(options.maxIKSeeds,16),max(options.seedPerturbDeg,15));
    for it=1:numel(travelGrid)
        for ir=1:numel(rollGrid)
            t=travelGrid(it); r=rollGrid(ir);
            if abs(t-p.travelDeg)>options.maxTravelStepDeg+1e-9 || ...
                    localAngleDistanceDeg(r,p.rollDeg)>options.maxRollStepDeg+1e-9
                continue;
            end
            stats.poseSamples=stats.poseSamples+1;
            [cand,c]=localPoseCandidates(scene,seam,i,t,r,seeds,options); stats=localAddStats(stats,c);
            for k=1:numel(cand)
                n=localNodeFromState(cand(k),i,ip,p.cost); dq=localWrapRad(n.q-p.q);
                if max(abs(rad2deg(dq)))>options.maxJointStepDeg, continue; end
                stats.edgeTests=stats.edgeTests+1;
                if ~localEdgeFree(scene,p.q,n.q,options), continue; end
                stats.edgeCollisionFree=stats.edgeCollisionFree+1;
                n.parentIndex=ip; n.velocityQ=dq;
                n.cost=p.cost+sum(dq.^2)+1e-4*( ...
                    localAngleDistanceDeg(r,p.rollDeg)^2+(t-p.travelDeg)^2);
                states(end+1,1)=n; %#ok<AGROW>
            end
        end
    end
end
end

function [states,stats] = localPoseCandidates(scene,seam,i,travelDeg,rollDeg,seeds,options)
[Ttarget,poseInfo]=localTargetForPose(scene,seam,i,travelDeg,rollDeg);
tic;
 r=generate_seam_axis_ik_candidates(scene,Ttarget,travelDeg,rollDeg,seeds, ...
    localCandidateOptions(options));
elapsed=toc;
stats=localEmptyStats(); stats.ikCalls=r.numIKCalls; stats.ikPoseSuccess=r.numIKPoseOK;
stats.ikElapsedSec=elapsed; stats.acceptedStates=r.numAccepted;
states=repmat(localEmptyNode(),0,1);
for k=1:numel(r.states)
    r.states(k).poseInfo=poseInfo;
    states(end+1,1)=localNodeFromState(r.states(k),i,0,0); %#ok<AGROW>
end
end

function n=localNodeFromState(st,pointIndex,parentIndex,cost)
n=localEmptyNode();
f=fieldnames(st);
for k=1:numel(f), n.(f{k})=st.(f{k}); end
n.pointIndex=pointIndex; n.parentIndex=parentIndex; n.cost=cost;
if ~isfield(n,"velocityQ"), n.velocityQ=zeros(1,6); end
end

function opts=localCandidateOptions(options)
opts=struct("positionToleranceM",options.positionToleranceM, ...
    "orientationToleranceDeg",options.orientationToleranceDeg, ...
    "sideToleranceDeg",options.sideToleranceDeg,"sideMarginM",options.sideMarginM, ...
    "checkSelfCollision",options.checkSelfCollision,"lazyCollision",false, ...
    "reuseCandidateCache",options.reuseCandidateCache);
end

function seeds=localSeedSet(q1,q2,maxSeeds,perturbDeg)
seeds=[q1;q2]; d=deg2rad(perturbDeg);
for j=4:6
    if size(seeds,1)>=maxSeeds, break; end
    q=q1; q(j)=q(j)+d; seeds(end+1,:)=q;
    if size(seeds,1)>=maxSeeds, break; end
    q=q1; q(j)=q(j)-d; seeds(end+1,:)=q;
end
seeds=seeds(1:min(maxSeeds,size(seeds,1)),:);
end

function [Ttarget,poseInfo]=localTargetForPose(scene,seam,i,t,r)
[RtcpWP,poseInfo]=make_actual_gun_axis_pose(scene,seam.tangentWP,scene.weldGunAxisWP,t,r);
Tw=eye(4); Tw(1:3,1:3)=RtcpWP; Tw(1:3,4)=seam.positionsWP(i,:).';
Ttarget=scene.T_MatBase_Workpiece*Tw;
end

function idx=localSelectParent(layer,options)
cost=vertcat(layer.cost); w=exp(-(cost-min(cost))/max(1e-9,std(cost)+1e-9));
if all(~isfinite(w)) || sum(w)<=0, idx=randi(numel(layer)); return; end
w=w/sum(w); idx=find(rand<=cumsum(w),1); if isempty(idx), idx=numel(layer); end
end

function idx=localNearestParents(layer,parent,qPred,K)
d=zeros(1,numel(layer));
for i=1:numel(layer), d(i)=norm(localWrapRad(layer(i).q-qPred)); end
[~,ord]=sort(d); idx=ord(1:min(K,numel(ord)));
end

function y=localSteer(x,y,maxStep), d=y-x; y=x+sign(d)*min(abs(d),maxStep); end
function y=localSteerRoll(x,y,maxStep)
d=rad2deg(atan2(sind(y-x),cosd(y-x))); y=localWrapRoll(x+sign(d)*min(abs(d),maxStep),[0 360]);
end
function y=localWrapRoll(x,bounds)
y=mod(x-bounds(1),360)+bounds(1); if y>=bounds(2), y=bounds(1); end
end

function ok=localPoseStepOK(a,b,o)
ok=localAngleDistanceDeg(a.rollDeg,b.rollDeg)<=o.maxRollStepDeg+1e-9 && abs(a.travelDeg-b.travelDeg)<=o.maxTravelStepDeg+1e-9;
end

function ok=localEdgeFree(scene,q1,q2,o)
dq=localWrapRad(q2-q1); n=max(1,ceil(max(abs(rad2deg(dq)))/o.edgeResolutionDeg)); ok=true;
for k=0:n
    q=q1+(k/n)*dq;
    if ~localFullStateFree(scene,q), ok=false; return; end
end
end

function ok=localFullStateFree(scene,q)
try, [env,~,~]=checkCollision(scene.robot,q,scene.environment,"IgnoreSelfCollision","on","Exhaustive","on");
catch, env=checkCollision(scene.robot,q,scene.environment,"IgnoreSelfCollision","on"); end
if any(env(:)), ok=false; return; end
try, self=checkCollision(scene.robot,q,"IgnoreSelfCollision","off","SkippedSelfCollisions",scene.selfCollisionSkipPairs,"Exhaustive","on");
catch, self=checkCollision(scene.robot,q,"IgnoreSelfCollision","off","SkippedSelfCollisions",scene.selfCollisionSkipPairs); end
ok=~any(self(:));
end

function [qPath,idx,complete,cost]=localBacktrack(layers,N)
qPath=zeros(0,6); idx=zeros(1,0); complete=false; cost=NaN;
if isempty(layers{N}), return; end
[cost,k]=min(vertcat(layers{N}.cost)); idx=zeros(1,N); idx(N)=k;
for i=N:-1:2
    p=layers{i}(idx(i)).parentIndex;
    if p<1 || p>numel(layers{i-1}), qPath=[]; idx=[]; cost=NaN; return; end
    idx(i-1)=p;
end
qPath=zeros(N,6); for i=1:N, qPath(i,:)=layers{i}(idx(i)).q; end
complete=true;
end

function metrics=localComputeTcpPathErrors(scene,layers,qPath,idx,o)
metrics=struct("maxNodePositionErrorMm",NaN,"maxNodeOrientationErrorDeg",NaN,"maxSegmentPositionErrorMm",NaN,"maxSegmentOrientationErrorDeg",NaN);
if isempty(qPath), return; end
N=size(qPath,1); nodeP=zeros(N,1); nodeR=zeros(N,1);
for i=1:N
    st=layers{i}(idx(i)); T=getTransform(scene.robot,qPath(i,:),scene.tcpBodyName);
    nodeP(i)=norm(T(1:3,4)-st.Ttarget(1:3,4)); c=(trace(st.Ttarget(1:3,1:3).'*T(1:3,1:3))-1)/2; nodeR(i)=acos(max(-1,min(1,c)));
end
segP=zeros(N-1,1); segR=zeros(N-1,1);
for i=1:N-1
    a=layers{i}(idx(i)); b=layers{i+1}(idx(i+1)); dq=localWrapRad(qPath(i+1,:)-qPath(i,:)); n=max(2,ceil(max(abs(rad2deg(dq)))/o.edgeResolutionDeg)+1);
    for k=0:n-1
        s=k/(n-1); q=qPath(i,:)+s*dq; T=getTransform(scene.robot,q,scene.tcpBodyName); p=(1-s)*a.Ttarget(1:3,4)+s*b.Ttarget(1:3,4);
        R1=rotm2quat(a.Ttarget(1:3,1:3)); R2=rotm2quat(b.Ttarget(1:3,1:3)); if dot(R1,R2)<0,R2=-R2;end; R=quat2rotm(((1-s)*R1+s*R2)/norm((1-s)*R1+s*R2));
        segP(i)=max(segP(i),norm(T(1:3,4)-p)); c=(trace(R.'*T(1:3,1:3))-1)/2; segR(i)=max(segR(i),acos(max(-1,min(1,c))));
    end
end
metrics.maxNodePositionErrorMm=max(nodeP)*1000; metrics.maxNodeOrientationErrorDeg=max(rad2deg(nodeR)); metrics.maxSegmentPositionErrorMm=max(segP)*1000; metrics.maxSegmentOrientationErrorDeg=max(rad2deg(segR));
end

function states=localKeepLayer(states,maxCount,refQ)
if isempty(states), return; end
score=vertcat(states.cost); [~,ord]=sort(score,"ascend"); states=states(ord(1:min(maxCount,numel(ord))));
% Keep a little angular diversity if costs tie; the layer cap remains hard.
if nargin>2 && numel(states)>1 %#ok<INUSD>
    states=states(1:min(maxCount,numel(states)));
end
end

function s=localEmptyNode()
s=struct("q",zeros(1,6),"travelDeg",0,"rollDeg",0,"Ttarget",eye(4),"Tactual",eye(4), ...
    "positionErrorM",NaN,"orientationErrorDeg",NaN,"gunAxisWP",zeros(1,3),"sideDot",NaN, ...
    "headPositionWP",zeros(1,3),"environmentCollision",false,"selfCollision",false,"pointIndex",0, ...
    "poseInfo",struct(),"parentIndex",0,"cost",Inf,"velocityQ",zeros(1,6));
end

function stats=localEmptyStats()
stats=struct("iterations",0,"poseSamples",0,"ikCalls",0,"ikPoseSuccess",0,"ikElapsedSec",0,"acceptedStates",0,"acceptedStartStates",0,"edgeTests",0,"edgeCollisionFree",0);
end
function out=localAddStats(a,b)
out=a; for f=fieldnames(a).', out.(f{1})=a.(f{1})+b.(f{1}); end
end
function opts=localMergeOptions(d,in)
opts=d; for f=fieldnames(d).', if isfield(in,f{1}) && ~isempty(in.(f{1})), opts.(f{1})=in.(f{1}); end, end
end
function p=localFindProxy(info,name)
idx=find(strcmp(string({info.name}),string(name)),1); if isempty(idx),error("找不到工件代理 %s",name);end; p=info(idx);
end
function d=localAngleDistanceDeg(a,b), d=abs(rad2deg(atan2(sind(b-a),cosd(b-a)))); end
function x=localWrapRad(x), x=atan2(sin(x),cos(x)); end
function skips=localStructuralSelfCollisionSkips(robot)
names=robot.BodyNames; skips=cell(0,2); for i=1:min(10,numel(names)-1), skips(end+1,:)={names{i},names{i+1}}; end
skips(end+1,:)={"WeldingGunMount","wrist_3_link"}; skips(end+1,:)={"WeldingGunMount","ee_link"}; skips(end+1,:)={"WeldingGunMount","WeldTCP"};
end
function limits=localRobotJointLimits(robot)
limits=zeros(6,2); j=0; for i=1:numel(robot.Bodies), jt=robot.Bodies{i}.Joint; if strcmpi(jt.Type,"revolute") || strcmpi(jt.Type,"continuous"), j=j+1; limits(j,:)=double(jt.PositionLimits); end,end
if j~=6,error("UR5关节数应为6，实际为%d",j);end
end
function tf=localAnimate(scene,seam,qPath)
fig=figure("Name","Gao AR RRT* weld baseline","Color","w"); ax=axes(fig); hold(ax,"on");
for k=1:numel(scene.workpieceCollision), show(scene.workpieceCollision{k},"Parent",ax); end
for i=1:size(qPath,1), show(scene.robot,qPath(i,:),"Parent",ax,"Visuals","on","Collisions","on","Frames","off","PreservePlot",false); hold(ax,"on"); for k=1:numel(scene.workpieceCollision),show(scene.workpieceCollision{k},"Parent",ax);end; plot3(ax,seam.positionsMat(:,1),seam.positionsMat(:,2),seam.positionsMat(:,3),"r-","LineWidth",2); axis(ax,"equal"); grid(ax,"on"); view(ax,135,25); drawnow; end
tf=fig;
end
