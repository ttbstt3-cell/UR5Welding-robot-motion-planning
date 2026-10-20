function result = replan_weld_actual_axis_continuation(options)
%REPLAN_WELD_ACTUAL_AXIS_CONTINUATION Cold-start beam continuation experiment.
%
% This independent experiment does not load a previous qPath/state graph.
% It keeps a small beam of collision-free IK branches and propagates them
% with local pose neighborhoods plus a constant-velocity joint prediction.
% The original layered planner is available as an explicit fallback only.
%
% This is an independent baseline: no previous weld path or state graph is
% read.  The MATLAB wire collision cylinder from sync_ur5_joint_states is
% included in every environment/self collision check.  The stable ROS2
% controller and the CoppeliaSim scene are not modified.

if nargin < 1 || isempty(options), options = struct(); end
% The 25 mm trim is required by the imported conservative V-HACD hull to
% keep the gun body clear of the x=0/x=L end walls.  Override only if a
% smaller physical end clearance has been validated separately.
defaults = struct( ...
    "numPoints",80,"endMarginM",0.025, ...
    "coarseTravelDeg",-15:15:15,"coarseRollDeg",0:45:315, ...
    "beamWidth",6,"firstPointMaxStates",6, ...
    "maxPoseSeeds",8,"seedPerturbDeg",8, ...
    "maxPoseNeighbors",9,"maxPoseStepRollDeg",15, ...
    "maxPoseStepTravelDeg",5,"numRandomSeedsFirstPoint",10, ...
    "maxRollStepDeg",45,"maxTravelStepDeg",15,"maxJointStepDeg",15, ...
    "edgeResolutionDeg",3,"maxBacktrackLayers",8, ...
    "positionToleranceM",1e-4,"orientationToleranceDeg",0.5, ...
    "sideToleranceDeg",5,"sideMarginM",1e-3, ...
    "checkSelfCollision",true,"animate",false,"publish",false, ...
    "reuseIKSolver",true,"reuseCandidateCache",true, ...
    "lazyCollision",true, ...
    "progressivePoseNeighbors",false, ...
    "progressiveInitialPoseNeighbors",5, ...
    "progressiveMaxPoseNeighbors",9, ...
    "disableLocalRecovery",false, ...
    "poseNeighborhoodCost","zero", ...
    "angularRedundancyFallback",false, ...
    "angularFallbackRollDeg",[10 20], ...
    "angularFallbackTravelDeg",[0 5 10], ...
    "enableFallback",true, ...
    "randomSeed",20260923,"outputFile", ...
    "weld_actual_axis_replan_continuation.mat");
options = localMergeOptions(defaults,options);

matlabDir = fileparts(mfilename("fullpath"));
rootDir = fileparts(matlabDir); %#ok<NASGU>
fprintf("\n========================================\n");
fprintf("REPLAN WELD WITH LOCAL IK CONTINUATION\n");
fprintf("========================================\n");
fprintf("No previous weld path is used as an input.\n");

[scene] = localLoadScene(fullfile(matlabDir,"sync_ur5_joint_states.m"));
scene.selfCollisionSkipPairs = localStructuralSelfCollisionSkips(scene.robot);

% This experiment uses one IK solver, one joint-limit vector and one cache.
if options.reuseIKSolver || options.reuseCandidateCache
    scene.ikSolver = inverseKinematics("RigidBodyTree",scene.robot);
    scene.jointLimits = localRobotJointLimits(scene.robot);
end
if options.reuseCandidateCache
    scene.ikCandidateCache = containers.Map( ...
        "KeyType","char","ValueType","any");
end
% U-interior side for bottom x wall_y1: y < inner wall and z > bottom top.
scene.weldGunAxisWP = [0;1;-1]/sqrt(2);
scene.weldGunAxisName = "measured_wire_axis_target";
scene.weldingSide = "U-interior: y < wall_y1 inner face, z > bottom top face";
scene.weldingContactPolicy = ...
    "Wire/environment contact is hard; no target-seam collision exemption";

scene.analyticalIK = localCheckAnalyticalIK(scene.robot);
fprintf("Analytical IK valid for loaded universalUR5: %d\n", ...
    scene.analyticalIK.isValidGroupForIK);
if ~scene.analyticalIK.isValidGroupForIK
    fprintf("Analytical IK is not used; numeric IK remains the validated solver.\n");
end

% Algorithm-only timer: scene construction/model loading above is excluded.
% The measured interval covers seam generation, candidate IK, collision
% checks, graph construction/refinement and final path search.
algorithmTimer = tic;

fprintf("\nSelf-collision structural exclusions (attachment only):\n");
for k = 1:size(scene.selfCollisionSkipPairs,1)
    fprintf("  %s <-> %s\n",scene.selfCollisionSkipPairs{k,1},scene.selfCollisionSkipPairs{k,2});
end
fprintf("All other self-collision pairs remain hard constraints.\n");

seam = localInferSeam(scene,options.numPoints,options.endMarginM);
tangentWP = seam.tangentWP(:);
gunAxisWP0 = scene.weldGunAxisWP(:);
fprintf("\nPhysical welding frame:\n");
fprintf("  seam = bottom x wall_y1\n");
fprintf("  start WP = [%.4f %.4f %.4f] m\n",seam.startWP);
fprintf("  end   WP = [%.4f %.4f %.4f] m\n",seam.endWP);
fprintf("  tangent = [%.4f %.4f %.4f]\n",tangentWP);
fprintf("  measured gun axis target = [%.4f %.4f %.4f]\n",gunAxisWP0);
fprintf("  work angle to bottom normal = %.2f deg\n", ...
    rad2deg(acos(abs(dot(gunAxisWP0,[0;0;-1])))));
fprintf("  plate thickness = %.1f mm\n",seam.plateThicknessM*1000);
fprintf("  welding side = %s\n",scene.weldingSide);

rng(options.randomSeed,"twister");
stateBank = cell(1,seam.numPoints);
layerStats = repmat(localEmptyLayerStats(),1,seam.numPoints);

fprintf("\n--- Cold-start bounded continuation ---\n");
stats = localEmptyContinuationStats();
[tLayer] = tic;
[stateBank{1},firstStats] = localGenerateFirstBeam(scene,seam,options);
stats = localAddContinuationStats(stats,firstStats);
stateBank{1} = localKeepBeam(stateBank{1},options.firstPointMaxStates,scene.qMat0);
layerStats(1).coarseStates = numel(stateBank{1});
layerStats(1).elapsedSec = toc(tLayer);
layerStats(1).ikCalls = firstStats.ikCalls;
layerStats(1).ikElapsedSec = firstStats.ikElapsedSec;
layerStats(1).edgeTests = firstStats.edgeTests;
layerStats(1).edgeCollisionChecks = firstStats.edgeCollisionChecks;
fprintf("Point %2d/%2d: beam states = %d\n",1,seam.numPoints,numel(stateBank{1}));

for i = 2:seam.numPoints
    tLayer = tic;
    layerOptions = options;
    if options.progressivePoseNeighbors
        layerOptions.maxPoseNeighbors = options.progressiveInitialPoseNeighbors;
        layerOptions.disableLocalRecovery = true;
    end
    [states,stepStats] = localGenerateContinuationLayer( ...
        scene,seam,i,stateBank{i-1},layerOptions);
    if isempty(states) && options.progressivePoseNeighbors
        % Expand only the failed layer.  Keep all hard pose, IK, joint-limit
        % and full node/edge collision constraints unchanged.
        expandedOptions = options;
        expandedOptions.maxPoseNeighbors = options.progressiveMaxPoseNeighbors;
        expandedOptions.disableLocalRecovery = false;
        [expandedStates,expandedStats] = localGenerateContinuationLayer( ...
            scene,seam,i,stateBank{i-1},expandedOptions);
        stepStats = localAddContinuationStats(stepStats,expandedStats);
        states = expandedStates;
    end
    stats = localAddContinuationStats(stats,stepStats);
    stateBank{i} = localKeepBeam(states,options.beamWidth,scene.qMat0);
    layerStats(i).coarseStates = numel(stateBank{i});
    layerStats(i).elapsedSec = toc(tLayer);
    layerStats(i).ikCalls = stepStats.ikCalls;
    layerStats(i).ikElapsedSec = stepStats.ikElapsedSec;
    layerStats(i).edgeTests = stepStats.edgeTests;
    layerStats(i).edgeCollisionChecks = stepStats.edgeCollisionChecks;
    if mod(i,5)==0 || i==seam.numPoints || isempty(stateBank{i})
        fprintf("Point %2d/%2d: beam states = %d, IK calls = %d\n", ...
            i,seam.numPoints,numel(stateBank{i}),stats.ikCalls);
    end
    if isempty(stateBank{i})
        fprintf("Continuation broke at point %d; bounded fallback will be considered.\n",i);
        break;
    end
end

if all(~cellfun(@isempty,stateBank))
    [graph,plan] = localBuildGraph(scene,stateBank,seam,options);
else
    graph = localEmptyGraph(seam.numPoints);
    plan = localEmptyPlan();
end

fallbackUsed = false;
fallbackResult = struct();
if (~plan.complete || isempty(plan.qPath)) && options.enableFallback
    fprintf("\n[Fallback] Calling original layered planner from a fresh scene.\n");
    fallbackOptions = struct( ...
        "animate",false,"publish",false,"reuseIKSolver",true, ...
        "reuseCandidateCache",true,"lazyCollision",true, ...
        "randomSeed",options.randomSeed, ...
        "outputFile","weld_actual_axis_replan_continuation_fallback.mat");
    fallbackResult = replan_weld_actual_axis(fallbackOptions);
    fallbackUsed = true;
    if fallbackResult.complete
        stateBank = fallbackResult.stateBank;
        graph = fallbackResult.graph;
        plan.qPath = fallbackResult.qPath;
        plan.complete = fallbackResult.complete;
        plan.nodeCollisionFree = fallbackResult.nodeCollisionFree;
        plan.edgeCollisionFree = fallbackResult.edgeCollisionFree;
        plan.maxJointJumpDeg = fallbackResult.maxJointJumpDeg;
    end
end

algorithmElapsedSec = toc(algorithmTimer);

result = struct();
sceneResult = scene;
% Do not serialize performance-only handle objects into the result file.
if isfield(sceneResult,"ikSolver")
    sceneResult = rmfield(sceneResult,"ikSolver");
end
if isfield(sceneResult,"jointLimits")
    sceneResult = rmfield(sceneResult,"jointLimits");
end
if isfield(sceneResult,"ikCandidateCache")
    sceneResult = rmfield(sceneResult,"ikCandidateCache");
end
if isfield(sceneResult,"analyticalIK")
    sceneResult = rmfield(sceneResult,"analyticalIK");
end
result.scene = sceneResult;
result.seam = seam;
result.stateBank = stateBank;
result.layerStats = layerStats;
result.graph = graph;
result.options = options;
result.qPath = plan.qPath;
result.complete = plan.complete;
result.nodeCollisionFree = plan.nodeCollisionFree;
result.edgeCollisionFree = plan.edgeCollisionFree;
result.ikSuccessAll = all(~cellfun(@isempty,stateBank));
result.maxJointJumpDeg = plan.maxJointJumpDeg;
result.actualGunAxisTcp = scene.wireAxisTcp;
result.gunAxisTargetWP = scene.weldGunAxisWP;
result.weldingSideConstraint = scene.weldingSide;
result.approachPathPlanned = false;
result.exitPathPlanned = false;
result.startConnectedStateCount = graph.startConnectedCount;
result.startConnectivityChecked = true;
result.approachExitNote = "自由空间接近/收焊退出未纳入本 80 点焊缝图，需单独规划。";
result.algorithmElapsedSec = algorithmElapsedSec;
result.totalPlanningTimeSec = algorithmElapsedSec;
result.continuationStats = stats;
result.fallbackUsed = fallbackUsed;
result.analyticalIK = scene.analyticalIK;
result.tcpMetrics = localComputeTcpPathErrors(scene,seam,stateBank,plan,options);

if plan.complete
    fprintf("\n[PASS] Complete measured-wire-axis 80-point path found.\n");
else
    fprintf("\n[FAIL] No complete 80-point path under hard constraints.\n");
end
fprintf("Weld points             : %d\n",seam.numPoints);
fprintf("Layers with IK states   : %d / %d\n", ...
    sum(~cellfun(@isempty,stateBank)),seam.numPoints);
fprintf("Reachable final states  : %d\n",graph.finalReachableCount);
fprintf("Node collision free     : %d\n",result.nodeCollisionFree);
fprintf("Edge collision free     : %d\n",result.edgeCollisionFree);
fprintf("Max joint jump          : %.3f deg\n",result.maxJointJumpDeg);
fprintf("IK calls                : %d\n",stats.ikCalls);
fprintf("IK elapsed              : %.6f s\n",stats.ikElapsedSec);
fprintf("IK pose-success rate    : %.2f %%\n", ...
    100*stats.ikPoseSuccess/max(1,stats.ikCalls));
fprintf("Max node TCP error      : %.6f mm\n",result.tcpMetrics.maxNodePositionErrorMm);
fprintf("Max segment TCP error   : %.6f mm\n",result.tcpMetrics.maxSegmentPositionErrorMm);
fprintf("Max segment TCP orient  : %.6f deg\n",result.tcpMetrics.maxSegmentOrientationErrorDeg);
fprintf("Fallback used           : %d\n",fallbackUsed);
fprintf("Wire collision model    : %s\n",ternary(scene.wireCollisionEnabled,"enabled","disabled"));
fprintf("Approach path planned   : %d\n",result.approachPathPlanned);
fprintf("Exit path planned       : %d\n",result.exitPathPlanned);
fprintf("Algorithm time (scene excluded): %.6f s\n",result.algorithmElapsedSec);

outputFile = options.outputFile;
if ~isfile(outputFile), outputFile = fullfile(matlabDir,outputFile); end
result.outputFile = outputFile;
save(outputFile,"result","-v7.3");
fprintf("Saved result: %s\n",outputFile);

if options.animate && plan.complete, localAnimate(scene,seam,plan.qPath); end
if options.publish && plan.complete
    publish_weld_trajectory(plan.qPath,struct("durationPerSegment",0.15));
end
end

function seam = localInferSeam(scene,N,endMarginM)
info = scene.workpieceProxyInfo;
bottom = localFindProxy(info,"WorkpieceCollision_bottom");
wallX0 = localFindProxy(info,"WorkpieceCollision_wall_x0");
wallX1 = localFindProxy(info,"WorkpieceCollision_wall_x1");
wallY1 = localFindProxy(info,"WorkpieceCollision_wall_y1");
bottomMin = bottom.T_workpiece_proxy(1:3,4)-bottom.size_m(:)/2;
bottomMax = bottom.T_workpiece_proxy(1:3,4)+bottom.size_m(:)/2;
xMin = max(bottomMin(1),wallX0.T_workpiece_proxy(1,4)+wallX0.size_m(1)/2);
xMax = min(bottomMax(1),wallX1.T_workpiece_proxy(1,4)-wallX1.size_m(1)/2);
ySeam = wallY1.T_workpiece_proxy(2,4)-wallY1.size_m(2)/2;
zSeam = bottomMax(3);
if xMax <= xMin, error("无法从 Workpiece boxes 推导焊缝交线。"); end
p0 = [xMin+endMarginM;ySeam;zSeam];
p1 = [xMax-endMarginM;ySeam;zSeam];
if p1(1)<=p0(1), error("焊缝端部余量 %.1f mm 过大。",endMarginM*1000); end
s = linspace(0,1,N).';
seam = struct();
seam.startWP = p0; seam.endWP = p1;
seam.tangentWP = (p1-p0)/norm(p1-p0);
seam.positionsWP = (1-s).*p0.'+s.*p1.';
P_B = (scene.T_MatBase_Workpiece*[seam.positionsWP,ones(N,1)].').';
seam.positionsMat = P_B(:,1:3);
seam.startMat = seam.positionsMat(1,:);
seam.endMat = seam.positionsMat(end,:);
seam.numPoints = N; seam.plateThicknessM = bottom.size_m(3);
seam.wallY1InnerY = ySeam; seam.bottomTopZ = zSeam;
seam.freeSpaceCondition = "y < wall_y1 inner face and z > bottom top face";
end

function scene = localLoadScene(syncFile)
% sync_ur5_joint_states is a script beginning with clear/clc/close all.
% Execute it in this helper workspace so it cannot erase replan options.
run(syncFile);
end

function s = localFindProxy(info,name)
idx = find(strcmp(string({info.name}),string(name)),1);
if isempty(idx), error("找不到工件代理 %s。",name); end
s = info(idx);
end

function [states,stats] = localGenerateFirstBeam(scene,seam,options)
% Generate only the first layer from a cold start.  No old qPath is read.
stats = localEmptyContinuationStats();
qSeeds = scene.qMat0;
for k = 1:options.numRandomSeedsFirstPoint
    qSeeds(end+1,:) = randomConfiguration(scene.robot); %#ok<AGROW>
end
states = repmat(localEmptyState(),0,1);
for it = 1:numel(options.coarseTravelDeg)
    for ir = 1:numel(options.coarseRollDeg)
        [Ttarget,poseInfo] = localTargetForPose(scene,seam,1, ...
            options.coarseTravelDeg(it),options.coarseRollDeg(ir)); %#ok<ASGLU>
        [r,ikElapsed] = localTimedIKCall(scene,Ttarget, ...
            options.coarseTravelDeg(it),options.coarseRollDeg(ir),qSeeds,options);
        stats = localAddGeneratorStats(stats,r,ikElapsed);
        for k = 1:numel(r.states)
            r.states(k).pointIndex = 1;
            r.states(k).poseInfo = poseInfo;
            r.states(k).velocityQ = zeros(1,6);
            r.states(k).parentQ = scene.qMat0;
            r.states(k).continuationCost = sum(localWrapRad(r.states(k).q-scene.qMat0).^2);
        end
        states = localAppendUniqueStates(states,r.states,0.5);
    end
end
end

function [states,stats] = localGenerateContinuationLayer(scene,seam,i,prev,options)
% Expand a bounded local pose neighborhood around the previous beam.
stats = localEmptyContinuationStats();
states = repmat(localEmptyState(),0,1);
if isempty(prev), return; end

for ip = 1:numel(prev)
    qPrev = prev(ip).q;
    if isfield(prev,"velocityQ")
        qPred = qPrev + prev(ip).velocityQ;
    else
        qPred = qPrev;
    end
    dRoll = [-options.maxPoseStepRollDeg 0 options.maxPoseStepRollDeg];
    dTravel = [-options.maxPoseStepTravelDeg 0 options.maxPoseStepTravelDeg];
    poseList = zeros(0,2);
    for it = 1:numel(dTravel)
        for ir = 1:numel(dRoll)
            poseList(end+1,:) = [prev(ip).travelDeg+dTravel(it), ...
                mod(prev(ip).rollDeg+dRoll(ir),360)]; %#ok<AGROW>
        end
    end
    poseCenter = [0 0];
    if isfield(options,"poseNeighborhoodCost") && ...
            strcmpi(string(options.poseNeighborhoodCost),"previous")
        poseCenter = [prev(ip).travelDeg prev(ip).rollDeg];
    end
    poseList = localUniquePoseList(poseList,options.maxPoseNeighbors,poseCenter);

    seeds = [qPred;qPrev];
    perturb = deg2rad(options.seedPerturbDeg);
    if options.maxPoseSeeds >= 3, q=qPred; q(4)=q(4)+perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds >= 4, q=qPred; q(4)=q(4)-perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds >= 5, q=qPred; q(5)=q(5)+perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds >= 6, q=qPred; q(5)=q(5)-perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds >= 7, q=qPred; q(6)=q(6)+perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds >= 8, q=qPred; q(6)=q(6)-perturb; seeds(end+1,:)=q; end
    seeds = seeds(1:min(size(seeds,1),options.maxPoseSeeds),:);

    for ipose = 1:size(poseList,1)
        [Ttarget,poseInfo] = localTargetForPose(scene,seam,i, ...
            poseList(ipose,1),poseList(ipose,2)); %#ok<ASGLU>
        [r,ikElapsed] = localTimedIKCall(scene,Ttarget, ...
            poseList(ipose,1),poseList(ipose,2),seeds,options);
        stats = localAddGeneratorStats(stats,r,ikElapsed);
        for ks = 1:numel(r.states)
            st = r.states(ks);
            st.pointIndex = i;
            st.poseInfo = poseInfo;
            [parentIdx,parentDq] = localBestParent(scene,prev,st,options);
            if parentIdx == 0, continue; end
            st.parentQ = prev(parentIdx).q;
            st.velocityQ = localWrapRad(st.q-st.parentQ);
            parentCost = 0;
            if isfield(prev,"continuationCost"), parentCost=prev(parentIdx).continuationCost; end
            st.continuationCost = parentCost + sum(parentDq.^2) + ...
                1e-4*(abs(st.rollDeg-prev(parentIdx).rollDeg)^2 + ...
                (st.travelDeg-prev(parentIdx).travelDeg)^2);
            states(end+1,1) = st; %#ok<AGROW>
            stats.edgeTests = stats.edgeTests + 1;
            stats.edgeCollisionChecks = stats.edgeCollisionChecks + 1;
        end
    end
end

states = localAppendUniqueStates(repmat(localEmptyState(),0,1),states,0.5);
if isempty(states) && options.angularRedundancyFallback && ...
        ~options.disableLocalRecovery
    % Conditional angular-redundancy recovery.  The measured wire axis is
    % already used by localTargetForPose/make_actual_gun_axis_pose; only the
    % redundant travel/roll neighborhood is widened here.
    for kStep = 1:numel(options.angularFallbackRollDeg)
        retry = options;
        retry.maxPoseStepRollDeg = options.angularFallbackRollDeg(kStep);
        retry.maxPoseStepTravelDeg = options.angularFallbackTravelDeg(min( ...
            kStep,numel(options.angularFallbackTravelDeg)));
        retry.maxPoseNeighbors = 9;
        retry.disableLocalRecovery = true;
        [candidate,candidateStats] = localGenerateContinuationLayer( ...
            scene,seam,i,prev,retry);
        stats = localAddContinuationStats(stats,candidateStats);
        if ~isempty(candidate)
            states = candidate;
            break;
        end
    end
end
if isempty(states) && ~options.disableLocalRecovery
    % Local bounded recovery: more seeds and the complete 3x3 pose stencil,
    % still subject to the same hard pose/joint/collision constraints.
    retry = options;
    retry.maxPoseSeeds = max(options.maxPoseSeeds,12);
    retry.maxPoseNeighbors = 25;
    retry.maxPoseStepRollDeg = max(options.maxPoseStepRollDeg,45);
    retry.maxPoseStepTravelDeg = max(options.maxPoseStepTravelDeg,15);
    retry.seedPerturbDeg = max(options.seedPerturbDeg,15);
    [states,retryStats] = localGenerateContinuationLayerNoRetry( ...
        scene,seam,i,prev,retry);
    stats = localAddContinuationStats(stats,retryStats);
end
end

function [states,stats] = localGenerateContinuationLayerNoRetry(scene,seam,i,prev,options)
% Same bounded expansion without recursive recovery.
stats = localEmptyContinuationStats();
states = repmat(localEmptyState(),0,1);
for ip = 1:numel(prev)
    qPrev = prev(ip).q;
    if isfield(prev,"velocityQ"), qPred=qPrev+prev(ip).velocityQ; else, qPred=qPrev; end
    dRoll = [-options.maxPoseStepRollDeg 0 options.maxPoseStepRollDeg];
    dTravel = [-options.maxPoseStepTravelDeg 0 options.maxPoseStepTravelDeg];
    poseList = zeros(0,2);
    for it=1:numel(dTravel), for ir=1:numel(dRoll)
        poseList(end+1,:)=[prev(ip).travelDeg+dTravel(it),mod(prev(ip).rollDeg+dRoll(ir),360)]; %#ok<AGROW>
    end, end
    poseCenter = [0 0];
    if isfield(options,"poseNeighborhoodCost") && ...
            strcmpi(string(options.poseNeighborhoodCost),"previous")
        poseCenter = [prev(ip).travelDeg prev(ip).rollDeg];
    end
    poseList=localUniquePoseList(poseList,options.maxPoseNeighbors,poseCenter);
    seeds=[qPred;qPrev]; perturb=deg2rad(options.seedPerturbDeg);
    if options.maxPoseSeeds>=3, q=qPred; q(4)=q(4)+perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds>=4, q=qPred; q(4)=q(4)-perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds>=5, q=qPred; q(5)=q(5)+perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds>=6, q=qPred; q(5)=q(5)-perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds>=7, q=qPred; q(6)=q(6)+perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds>=8, q=qPred; q(6)=q(6)-perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds>=10, q=qPred; q(5)=q(5)+2*perturb; seeds(end+1,:)=q; q=qPred; q(5)=q(5)-2*perturb; seeds(end+1,:)=q; end
    seeds=seeds(1:min(size(seeds,1),options.maxPoseSeeds),:);
    for ipose=1:size(poseList,1)
        [Ttarget,poseInfo]=localTargetForPose(scene,seam,i,poseList(ipose,1),poseList(ipose,2)); %#ok<ASGLU>
        [r,ikElapsed]=localTimedIKCall(scene,Ttarget,poseList(ipose,1),poseList(ipose,2),seeds,options);
        stats=localAddGeneratorStats(stats,r,ikElapsed);
        for ks=1:numel(r.states)
            st=r.states(ks); st.pointIndex=i; st.poseInfo=poseInfo;
            [parentIdx,parentDq]=localBestParent(scene,prev,st,options); if parentIdx==0, continue; end
            st.parentQ=prev(parentIdx).q; st.velocityQ=localWrapRad(st.q-st.parentQ);
            parentCost=0; if isfield(prev,"continuationCost"), parentCost=prev(parentIdx).continuationCost; end
            st.continuationCost=parentCost+sum(parentDq.^2);
            states(end+1,1)=st; %#ok<AGROW>
            stats.edgeTests=stats.edgeTests+1; stats.edgeCollisionChecks=stats.edgeCollisionChecks+1;
        end
    end
end
states=localAppendUniqueStates(repmat(localEmptyState(),0,1),states,0.5);
end

function [idx,dq] = localBestParent(scene,prev,st,options)
idx=0; dq=zeros(1,6); best=inf;
for k=1:numel(prev)
    if ~localPoseStepOK(prev(k),st,options), continue; end
    d=localWrapRad(st.q-prev(k).q);
    if max(abs(rad2deg(d)))>options.maxJointStepDeg, continue; end
    if sum(d.^2)>=best, continue; end
    if ~localEdgeFree(scene,prev(k).q,st.q,options), continue; end
    best=sum(d.^2); idx=k; dq=d;
end
end

function [Ttarget,poseInfo] = localTargetForPose(scene,seam,i,travelDeg,rollDeg)
[RtcpWP,poseInfo] = make_actual_gun_axis_pose(scene,seam.tangentWP, ...
    scene.weldGunAxisWP,travelDeg,rollDeg);
T_W_TCP=eye(4); T_W_TCP(1:3,1:3)=RtcpWP; T_W_TCP(1:3,4)=seam.positionsWP(i,:).';
Ttarget=scene.T_MatBase_Workpiece*T_W_TCP;
end

function optionsOut = localCandidateOptions(options)
optionsOut=struct("positionToleranceM",options.positionToleranceM, ...
    "orientationToleranceDeg",options.orientationToleranceDeg, ...
    "sideToleranceDeg",options.sideToleranceDeg,"sideMarginM",options.sideMarginM, ...
    "checkSelfCollision",options.checkSelfCollision,"lazyCollision",options.lazyCollision, ...
    "reuseCandidateCache",options.reuseCandidateCache);
end

function poses = localUniquePoseList(poses,maxCount,center)
if isempty(poses), return; end
if nargin < 3 || isempty(center), center=[0 0]; end
poses(:,2)=mod(poses(:,2),360);
keep=true(size(poses,1),1);
for i=1:size(poses,1)
    if ~keep(i), continue; end
    for j=i+1:size(poses,1)
        if keep(j) && abs(poses(i,1)-poses(j,1))<1e-9 && ...
                localAngleDistanceDeg(poses(i,2),poses(j,2))<1e-9
            keep(j)=false;
        end
    end
end
poses=poses(keep,:);
% Prefer the smallest relative travel/roll change.  This implements the
% angular-redundancy continuity cost used by the paper-guided experiment.
score=abs(poses(:,1)-center(1))+arrayfun( ...
    @(x)localAngleDistanceDeg(x,center(2)),poses(:,2));
[~,order]=sort(score,"ascend");
poses=poses(order,:);
if size(poses,1)>maxCount, poses=poses(1:maxCount,:); end
end

function states = localKeepBeam(states,maxCount,refQ)
if isempty(states), return; end
if isfield(states,"continuationCost")
    score=vertcat(states.continuationCost);
else
    score=zeros(numel(states),1);
    for i=1:numel(states), score(i)=sum(localWrapRad(states(i).q-refQ).^2); end
end
[~,idx]=sort(score,"ascend");
states=states(idx(1:min(maxCount,numel(idx))));
end

function stats = localEmptyContinuationStats()
stats=struct("ikCalls",0,"ikPoseSuccess",0,"acceptedStates",0, ...
    "candidatePoseCalls",0,"edgeTests",0,"edgeCollisionChecks",0, ...
    "ikElapsedSec",0);
end

function [r,elapsed] = localTimedIKCall(scene,Ttarget,travelDeg,rollDeg,seeds,options)
t=tic;
r=generate_actual_axis_ik_candidates(scene,Ttarget,travelDeg,rollDeg,seeds, ...
    localCandidateOptions(options));
elapsed=toc(t);
end

function stats = localAddGeneratorStats(stats,r,elapsed)
stats.ikCalls=stats.ikCalls+r.numIKCalls;
stats.ikPoseSuccess=stats.ikPoseSuccess+r.numIKPoseOK;
stats.acceptedStates=stats.acceptedStates+r.numAccepted;
stats.candidatePoseCalls=stats.candidatePoseCalls+1;
if nargin>=3, stats.ikElapsedSec=stats.ikElapsedSec+elapsed; end
end

function out = localAddContinuationStats(a,b)
out=a;
names=fieldnames(a);
for i=1:numel(names), out.(names{i})=a.(names{i})+b.(names{i}); end
end

function graph = localEmptyGraph(N)
graph=struct("reachable",{cell(1,N)},"cost",{cell(1,N)}, ...
    "parent",{cell(1,N)},"edgeTests",0,"collisionFreeEdges",0, ...
    "finalReachableCount",0,"startConnectedCount",0);
end

function info = localCheckAnalyticalIK(robot)
info = struct("available",false,"isValidGroupForIK",false, ...
    "kinematicGroupType","","configIdx",[]);
try
    if exist("analyticalInverseKinematics","class") ~= 8
        return;
    end
    a = analyticalInverseKinematics(robot);
    info.available = true;
    info.isValidGroupForIK = logical(a.IsValidGroupForIK);
    info.kinematicGroupType = char(a.KinematicGroupType);
    info.configIdx = double(a.KinematicGroupConfigIdx);
catch err
    info.error = string(err.message);
end
end

function metrics = localComputeTcpPathErrors(scene,seam,stateBank,plan,options)
metrics = struct("maxNodePositionErrorMm",NaN, ...
    "maxNodeOrientationErrorDeg",NaN,"maxSegmentPositionErrorMm",NaN, ...
    "maxSegmentOrientationErrorDeg",NaN,"checkedSegments",0, ...
    "allSegmentsChecked",false);
if ~plan.complete || isempty(plan.qPath) || ~isfield(plan,"stateIndices")
    return;
end
N=size(plan.qPath,1);
nodePos=zeros(N,1); nodeOri=zeros(N,1);
for i=1:N
    st=stateBank{i}(plan.stateIndices(i));
    T=getTransform(scene.robot,plan.qPath(i,:),scene.tcpBodyName);
    nodePos(i)=norm(T(1:3,4)-st.Ttarget(1:3,4));
    c=(trace(st.Ttarget(1:3,1:3).'*T(1:3,1:3))-1)/2;
    nodeOri(i)=acos(max(-1,min(1,c)));
end
metrics.maxNodePositionErrorMm=max(nodePos)*1000;
metrics.maxNodeOrientationErrorDeg=max(rad2deg(nodeOri));
segPos=zeros(N-1,1); segOri=zeros(N-1,1);
samplesPerSegment=max(2,ceil(max(abs(rad2deg(diff(plan.qPath,1,1))),[],2)/options.edgeResolutionDeg)+1);
for i=1:N-1
    a=stateBank{i}(plan.stateIndices(i)); b=stateBank{i+1}(plan.stateIndices(i+1));
    for k=0:samplesPerSegment(i)-1
        s=k/(samplesPerSegment(i)-1);
        q=plan.qPath(i,:)+s*localWrapRad(plan.qPath(i+1,:)-plan.qPath(i,:));
        T=getTransform(scene.robot,q,scene.tcpBodyName);
        pRef=(1-s)*a.Ttarget(1:3,4)+s*b.Ttarget(1:3,4);
        RRef=localQuatSlerp(a.Ttarget(1:3,1:3),b.Ttarget(1:3,1:3),s);
        segPos(i)=max(segPos(i),norm(T(1:3,4)-pRef));
        c=(trace(RRef.'*T(1:3,1:3))-1)/2;
        segOri(i)=max(segOri(i),acos(max(-1,min(1,c))));
    end
end
metrics.maxSegmentPositionErrorMm=max(segPos)*1000;
metrics.maxSegmentOrientationErrorDeg=max(rad2deg(segOri));
metrics.checkedSegments=N-1;
metrics.allSegmentsChecked=(N-1)==79;
end

function R = localQuatSlerp(R1,R2,s)
q1=rotm2quat(R1); q2=rotm2quat(R2);
if dot(q1,q2)<0, q2=-q2; end
q=(1-s)*q1+s*q2; q=q/norm(q);
R=quat2rotm(q);
end

function states = localGenerateLayer(scene,seam,i,travelCandidates,rollCandidates,options,existing)
seedBank = localSeedBank(scene,existing,options.numRandomSeeds,options.maxContinuationSeeds);
states = repmat(localEmptyState(),0,1);
lastPoseStates = repmat(localEmptyState(),0,1);
pWP = seam.positionsWP(i,:).';
for it = 1:numel(travelCandidates)
    for ir = 1:numel(rollCandidates)
        poseSeeds = seedBank;
        if options.reusePoseCandidates && ~isempty(lastPoseStates)
            nSeed = min(numel(lastPoseStates),options.maxContinuationSeeds);
            idxSeed = unique(round(linspace(1,numel(lastPoseStates),nSeed)));
            poseSeeds = [seedBank(1,:); vertcat(lastPoseStates(idxSeed).q)];
        end
        [RtcpWP,poseInfo] = make_actual_gun_axis_pose(scene,seam.tangentWP, ...
            scene.weldGunAxisWP,travelCandidates(it),rollCandidates(ir));
        T_W_TCP = eye(4); T_W_TCP(1:3,1:3)=RtcpWP; T_W_TCP(1:3,4)=pWP;
        Ttarget = scene.T_MatBase_Workpiece*T_W_TCP;
        r = generate_actual_axis_ik_candidates(scene,Ttarget, ...
            travelCandidates(it),rollCandidates(ir),poseSeeds,struct( ...
            "positionToleranceM",options.positionToleranceM, ...
            "orientationToleranceDeg",options.orientationToleranceDeg, ...
            "sideToleranceDeg",options.sideToleranceDeg, ...
            "sideMarginM",options.sideMarginM, ...
            "checkSelfCollision",options.checkSelfCollision, ...
            "lazyCollision",options.lazyCollision, ...
            "reuseCandidateCache",options.reuseCandidateCache));
        % Retry a failed continuation with the full seed bank, while
        % retaining the last successful pose states for the next pose.
        if options.reusePoseCandidates && isempty(r.states) && ...
                ~isempty(lastPoseStates) && size(poseSeeds,1) < size(seedBank,1)
            r = generate_actual_axis_ik_candidates(scene,Ttarget, ...
                travelCandidates(it),rollCandidates(ir),seedBank,struct( ...
                "positionToleranceM",options.positionToleranceM, ...
                "orientationToleranceDeg",options.orientationToleranceDeg, ...
                "sideToleranceDeg",options.sideToleranceDeg, ...
                "sideMarginM",options.sideMarginM, ...
                "checkSelfCollision",options.checkSelfCollision, ...
                "lazyCollision",options.lazyCollision, ...
                "reuseCandidateCache",options.reuseCandidateCache));
        end
        if options.reusePoseCandidates && ~isempty(r.states)
            lastPoseStates = r.states;
        end
        for k = 1:numel(r.states)
            r.states(k).pointIndex = i; r.states(k).poseInfo = poseInfo;
        end
        if ~isempty(r.states), states = [states;r.states(:)]; %#ok<AGROW>
        end
    end
end
states = localAppendUniqueStates(repmat(localEmptyState(),0,1),states,0.5);
end

function seeds = localSeedBank(scene,prior,numRandom,maxPrior)
seeds = scene.qMat0;
for k = 1:numRandom, seeds(end+1,:)=randomConfiguration(scene.robot); %#ok<AGROW>
end
if ~isempty(prior)
    n = min(numel(prior),maxPrior);
    idx = unique(round(linspace(1,numel(prior),n)));
    for k = idx, seeds(end+1,:) = prior(k).q; %#ok<AGROW>
    end
end
end

function states = localAppendUniqueStates(a,b,tolDeg)
states = a;
for i = 1:numel(b)
    dup = false;
    for j = 1:numel(states)
        if max(abs(rad2deg(localWrapRad(b(i).q-states(j).q)))) < tolDeg
            dup = true; break;
        end
    end
    if ~dup, states(end+1,1)=b(i); %#ok<AGROW>
    end
end
end

function [graph,plan] = localBuildGraph(scene,stateBank,seam,options)
N = numel(stateBank);
graph = struct("reachable",{cell(1,N)},"cost",{cell(1,N)}, ...
    "parent",{cell(1,N)},"edgeTests",0,"collisionFreeEdges",0, ...
    "finalReachableCount",0,"startConnectedCount",0);
nodeFree = cell(1,N);
q0Free = localFullStateFree(scene,scene.qMat0);
if ~q0Free
    fprintf("[WARN] qMat0 is not free under the complete self/environment check; start connectivity is unavailable.\n");
end
if isempty(stateBank{1}), plan = localEmptyPlan(); return; end
% The seam path itself starts at the first validated weld state.  Test the
% real qMat0-to-first-state approach separately, but do not discard an
% otherwise valid weld path merely because free-space approach is not yet
% planned (it is explicitly reported as incomplete below).
graph.reachable{1}=true(1,numel(stateBank{1}));
graph.cost{1}=zeros(1,numel(stateBank{1}));
graph.parent{1}=zeros(1,numel(stateBank{1}));
nodeFree{1}=true(1,numel(stateBank{1}));
for j=1:numel(stateBank{1})
    graph.edgeTests=graph.edgeTests+1;
    if q0Free && nodeFree{1}(j) && localEdgeFree(scene,scene.qMat0,stateBank{1}(j).q,options)
        graph.startConnectedCount=graph.startConnectedCount+1;
        graph.collisionFreeEdges=graph.collisionFreeEdges+1;
    end
end
for i=2:N
    nNow=numel(stateBank{i}); nPrev=numel(stateBank{i-1});
    graph.reachable{i}=false(1,nNow);
    graph.cost{i}=inf(1,nNow); graph.parent{i}=zeros(1,nNow);
    nodeFree{i}=true(1,nNow);
    for j=1:nNow
        for k=1:nPrev
            if ~graph.reachable{i-1}(k), continue; end
            if ~localPoseStepOK(stateBank{i-1}(k),stateBank{i}(j),options), continue; end
            dq=localWrapRad(stateBank{i}(j).q-stateBank{i-1}(k).q);
            if max(abs(rad2deg(dq)))>options.maxJointStepDeg, continue; end
            if ~nodeFree{i}(j), continue; end
            graph.edgeTests=graph.edgeTests+1;
            if options.lazyCollision
                edgeOK=localEdgeFreeInterior(scene,stateBank{i-1}(k).q,stateBank{i}(j).q,options);
            else
                edgeOK=localEdgeFree(scene,stateBank{i-1}(k).q,stateBank{i}(j).q,options);
            end
            if ~edgeOK, continue; end
            graph.collisionFreeEdges=graph.collisionFreeEdges+1;
            edgeCost=sum(dq.^2)+1e-4*( ...
                localAngleDistanceDeg(stateBank{i-1}(k).rollDeg,stateBank{i}(j).rollDeg)^2+ ...
                (stateBank{i}(j).travelDeg-stateBank{i-1}(k).travelDeg)^2);
            candidateCost=graph.cost{i-1}(k)+edgeCost;
            if candidateCost<graph.cost{i}(j)
                graph.cost{i}(j)=candidateCost; graph.parent{i}(j)=k;
                graph.reachable{i}(j)=true;
            end
        end
    end
end
graph.finalReachableCount=sum(graph.reachable{N});
plan=localEmptyPlan();
plan.nodeCollisionFree=graph.finalReachableCount>0 && all(~cellfun(@isempty,stateBank));
plan.edgeCollisionFree=graph.collisionFreeEdges>0;
plan.complete=graph.finalReachableCount>0 && N==seam.numPoints;
if ~plan.complete, return; end
[~,idx]=min(graph.cost{N});
stateIndices=zeros(1,N); stateIndices(N)=idx;
for i=N:-1:2, stateIndices(i-1)=graph.parent{i}(stateIndices(i)); end
qPath=zeros(N,6);
for i=1:N, qPath(i,:)=stateBank{i}(stateIndices(i)).q; end
plan.qPath=qPath; plan.stateIndices=stateIndices;
dq=localWrapRad(diff(qPath,1,1));
plan.maxJointJumpDeg=max(abs(rad2deg(dq)),[],'all');
plan.nodeCollisionFree=all(arrayfun(@(i)localFullStateFree(scene,qPath(i,:)),1:N));
plan.edgeCollisionFree=all(arrayfun(@(i)localEdgeFree(scene,qPath(i,:),qPath(i+1,:),options),1:N-1));
end

function ok=localPoseStepOK(a,b,options)
ok=localAngleDistanceDeg(a.rollDeg,b.rollDeg)<=options.maxRollStepDeg+1e-9 && ...
    abs(a.travelDeg-b.travelDeg)<=options.maxTravelStepDeg+1e-9;
end

function ok=localEdgeFree(scene,q1,q2,options)
dq=localWrapRad(q2-q1);
n=max(1,ceil(max(abs(rad2deg(dq)))/options.edgeResolutionDeg));
ok=true;
for k=0:n
    q=q1+(k/n)*dq;
    if ~localFullStateFree(scene,q), ok=false; return; end
end
end

function ok=localEdgeFreeInterior(scene,q1,q2,options)
%LAZY edge test: endpoints were already checked by the node cache.
dq=localWrapRad(q2-q1);
n=max(1,ceil(max(abs(rad2deg(dq)))/options.edgeResolutionDeg));
ok=true;
for k=1:n-1
    q=q1+(k/n)*dq;
    if ~localFullStateFree(scene,q), ok=false; return; end
end
end

function ok=localFullStateFree(scene,q)
try
    [env,~,~]=checkCollision(scene.robot,q,scene.environment, ...
        "IgnoreSelfCollision","on","Exhaustive","on");
catch
    env=checkCollision(scene.robot,q,scene.environment,"IgnoreSelfCollision","on");
end
if any(env(:)), ok=false; return; end
if isfield(scene,"selfCollisionSkipPairs"), skips=scene.selfCollisionSkipPairs;
else, skips="parent"; end
try
    self=checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",skips,"Exhaustive","on");
catch
    self=checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",skips);
end
ok=~any(self(:));
end

function pairs=localStructuralSelfCollisionSkips(robot)
names=robot.BodyNames; pairs=cell(0,2);
% Mechanical attachment interfaces; not free-space collision exclusions.
for i=1:min(10,numel(names)-1), pairs(end+1,:)={names{i},names{i+1}}; %#ok<AGROW>
end
pairs(end+1,:)={"WeldingGunMount","wrist_3_link"};
pairs(end+1,:)={"WeldingGunMount","ee_link"};
pairs(end+1,:)={"WeldingGunMount","WeldTCP"};
end

function limits=localRobotJointLimits(robot)
limits=zeros(6,2);
j=0;
for i=1:numel(robot.Bodies)
    jt=robot.Bodies{i}.Joint;
    if strcmpi(jt.Type,"revolute") || strcmpi(jt.Type,"continuous")
        j=j+1;
        limits(j,:)=double(jt.PositionLimits);
    end
end
if j~=6, error("UR5 关节数应为 6，实际为 %d。",j); end
end

function gap=localFirstReachabilityGap(reachable)
gap=[];
for i=1:numel(reachable)
    if isempty(reachable{i}) || ~any(reachable{i}), gap=i; return; end
end
end

function p=localEmptyPlan()
p=struct("complete",false,"qPath",zeros(0,6),"stateIndices",zeros(1,0), ...
    "nodeCollisionFree",false,"edgeCollisionFree",false,"maxJointJumpDeg",NaN);
end

function s=localEmptyState()
s=struct("q",zeros(1,6),"travelDeg",0,"rollDeg",0,"Ttarget",eye(4), ...
    "Tactual",eye(4),"positionErrorM",NaN,"orientationErrorDeg",NaN, ...
    "gunAxisWP",zeros(1,3),"sideDot",NaN,"headPositionWP",zeros(1,3), ...
    "environmentCollision",false,"selfCollision",false,"pointIndex",0, ...
    "poseInfo",struct(),"velocityQ",zeros(1,6),"parentQ",zeros(1,6), ...
    "continuationCost",0);
end

function s=localEmptyLayerStats()
s=struct("coarseStates",0,"fineStates",0,"elapsedSec",0, ...
    "ikCalls",0,"ikElapsedSec",0,"edgeTests",0, ...
    "edgeCollisionChecks",0);
end

function out=localMergeOptions(defaults,in)
out=defaults; names=fieldnames(defaults);
for i=1:numel(names)
    if isfield(in,names{i}) && ~isempty(in.(names{i})), out.(names{i})=in.(names{i}); end
end
end

function d=localAngleDistanceDeg(a,b)
d=abs(rad2deg(atan2(sind(b-a),cosd(b-a))));
end

function x=localWrapRad(x), x=atan2(sin(x),cos(x)); end

function out=ternary(condition,a,b)
if condition, out=a; else, out=b; end
end

function localAnimate(scene,seam,qPath)
fig=figure("Name","Measured wire-axis weld replan","Color","w");
ax=axes(fig); hold(ax,"on");
for k=1:numel(scene.workpieceCollision), show(scene.workpieceCollision{k},"Parent",ax); end
for i=1:size(qPath,1)
    show(scene.robot,qPath(i,:),"Parent",ax,"Visuals","on", ...
        "Collisions","on","Frames","off","PreservePlot",false);
    hold(ax,"on");
    for k=1:numel(scene.workpieceCollision), show(scene.workpieceCollision{k},"Parent",ax); end
    plot3(ax,seam.positionsMat(:,1),seam.positionsMat(:,2),seam.positionsMat(:,3), ...
        "Color",[0.85 0 0],"LineWidth",2);
    T=getTransform(scene.robot,qPath(i,:),scene.tcpBodyName); p=T(1:3,4);
    plot3(ax,p(1),p(2),p(3),"go","MarkerFaceColor","g","MarkerSize",6);
    axis(ax,"equal"); grid(ax,"on"); view(ax,135,25);
    title(ax,sprintf("Measured gun-axis weld point %d/%d",i,size(qPath,1)));
    drawnow;
end
end
