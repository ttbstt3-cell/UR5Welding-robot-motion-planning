function result = replan_weld_seam_generalization_feasibility(options)
%REPLAN_WELD_ACTUAL_AXIS_ADAPTIVE_FEASIBILITY
% Feasibility-first adaptive ARC-IK experiment.
%
% Start from a sparse seam (normally 16 points).  If bounded continuation
% fails at layer k, bisect only the preceding geometric interval and retry
% from a cold start.  This file is independent of the stable 80-point
% continuation and does not load an old qPath/state graph.
%
% This first stage deliberately separates feasibility refinement from the
% later TCP execution-waypoint correction.  Every retry still performs the
% complete node/edge checks implemented by the continuation core.

if nargin < 1 || isempty(options), options = struct(); end
defaults = struct( ...
    "initialPoints",16, ...
    "initialSeamPositionsWP",[], ...
    "seamSpec",struct(), ...
    "maxRefinementRounds",4, ...
    "maxPoints",40, ...
    "maxSegmentTcpErrorMm",0.5, ...
    "maxIntermediatePoints",4, ...
    "maxPrecisionRounds",2, ...
    "animate",false,"publish",false, ...
    "enableFallback",false, ...
    "beamWidth",1,"firstPointMaxStates",2,"maxPoseSeeds",3, ...
    "endMarginM",0.025, ...
    "coarseTravelDeg",0, ...
    "coarseRollDeg",0:45:315, ...
    "seedPerturbDeg",8,"maxPoseNeighbors",9, ...
    "maxPoseStepRollDeg",15,"maxPoseStepTravelDeg",5, ...
    "numRandomSeedsFirstPoint",3, ...
    "maxRollStepDeg",45,"maxTravelStepDeg",15, ...
    "maxJointStepDeg",15,"edgeResolutionDeg",3, ...
    "maxBacktrackLayers",8, ...
    "positionToleranceM",1e-4,"orientationToleranceDeg",0.5, ...
    "sideToleranceDeg",5,"sideMarginM",1e-3, ...
    "checkSelfCollision",true, ...
    "reuseIKSolver",true,"reuseCandidateCache",true,"lazyCollision",true, ...
    "progressivePoseNeighbors",true, ...
    "progressiveInitialPoseNeighbors",5, ...
    "progressiveMaxPoseNeighbors",9, ...
    "disableLocalRecovery",false, ...
    "poseNeighborhoodCost","zero", ...
    "angularRedundancyFallback",false, ...
    "angularFallbackRollDeg",[10 20], ...
    "angularFallbackTravelDeg",[0 5 10], ...
    "enablePostRecovery",false, ...
    "recoveryBeamWidths",[2 3], ...
    "recoveryRollFallbackDeg",[10 20], ...
    "recoveryTravelFallbackDeg",[0 5 10], ...
    "recoveryMaxPoseNeighbors",25, ...
    "recoveryPoseNeighborhoodCost","previous", ...
    "recoveryMaxPoseStepRollDeg",20, ...
    "recoveryMaxPoseStepTravelDeg",10, ...
    "randomSeed",20260923, ...
    "outputFile","weld_actual_axis_adaptive_feasibility.mat", ...
    "roundOutputPrefix","weld_actual_axis_adaptive_feasibility_round");
options = localMergeOptions(defaults,options);

here = fileparts(mfilename("fullpath"));
% Optional experiment-only initial sparse seam.  When supplied, the first
% adaptive round starts from these exact anchor positions instead of
% regenerating the default uniform seam.  The normal planner behavior is
% unchanged because the default is empty.
currentPositions = options.initialSeamPositionsWP;
if ~isempty(currentPositions)
    if size(currentPositions,2) ~= 3 || size(currentPositions,1) < 2
        error("initialSeamPositionsWP must be an N-by-3 matrix with N >= 2.");
    end
    options.initialPoints = size(currentPositions,1);
end
rounds = repmat(localEmptyRound(),0,1);
totalTimer = tic;
strongFirstPointTried = false;
finalCore = struct();

fprintf("\n========================================\n");
fprintf("ADAPTIVE ARC-IK: FEASIBILITY REFINEMENT\n");
fprintf("========================================\n");
fprintf("Initial seam points : %d\n",options.initialPoints);
fprintf("Max refinement     : %d rounds / %d points\n", ...
    options.maxRefinementRounds,options.maxPoints);

for iround = 0:options.maxRefinementRounds
    coreOptions = localCoreOptions(options);
    coreOptions.randomSeed = options.randomSeed;
    coreOptions.animate = false;
    coreOptions.publish = false;
    coreOptions.enableFallback = false;
    if isempty(currentPositions)
        coreOptions.numPoints = options.initialPoints;
        coreOptions.seamPositionsWP = [];
    else
        coreOptions.numPoints = size(currentPositions,1);
        coreOptions.seamPositionsWP = currentPositions;
    end
    roundFile = sprintf("%s_%02d.mat",options.roundOutputPrefix,iround);
    if ~startsWith(string(roundFile),filesep)
        roundFile = fullfile(here,roundFile);
    end
    coreOptions.outputFile = roundFile;

    fprintf("\n[Adaptive round %d] planning %d seam points...\n", ...
        iround,coreOptions.numPoints);
    tRound = tic;
    finalCore = replan_weld_seam_generalization_core(coreOptions);
    roundWall = toc(tRound);

    % The core always returns the actual point list used in this round.
    currentPositions = finalCore.seam.positionsWP;
    failIdx = localFirstEmptyLayer(finalCore.stateBank);
    if isempty(failIdx), failIdx = NaN; end
    rr = localEmptyRound();
    rr.round = iround;
    rr.pointsBefore = size(currentPositions,1);
    rr.complete = finalCore.complete;
    rr.furthestPoint = localFurthestPoint(finalCore.stateBank);
    rr.failedLayer = failIdx;
    rr.algorithmTimeSec = finalCore.algorithmElapsedSec;
    rr.wallTimeSec = roundWall;
    rr.ikCalls = finalCore.continuationStats.ikCalls;
    rr.maxJointJumpDeg = finalCore.maxJointJumpDeg;
    rr.maxSegmentTcpErrorMm = finalCore.tcpMetrics.maxSegmentPositionErrorMm;

    if finalCore.complete
        rounds(end+1,1) = rr; %#ok<AGROW>
        fprintf("[PASS] Sparse/adaptively refined seam is feasible at %d points.\n", ...
            size(currentPositions,1));
        break;
    end

    % A failure in the first layer is a seed/pose-branch issue, not a seam
    % spacing issue.  Give it one stronger first-point attempt before any
    % geometric subdivision.
    if isequal(failIdx,1) && ~strongFirstPointTried
        strongFirstPointTried = true;
        options.numRandomSeedsFirstPoint = max(options.numRandomSeedsFirstPoint,10);
        options.coarseTravelDeg = [-15 0 15];
        options.maxPoseSeeds = max(options.maxPoseSeeds,8);
        options.firstPointMaxStates = max(options.firstPointMaxStates,2);
        rr.recovery = "strong_first_point_retry";
        rounds(end+1,1) = rr; %#ok<AGROW>
        fprintf("[Recovery] first layer failed; retrying with stronger first-point seeds.\n");
        continue;
    end

    if isempty(failIdx) || failIdx <= 1 || size(currentPositions,1) >= options.maxPoints
        rr.recovery = "no_bisection_possible";
        rounds(end+1,1) = rr; %#ok<AGROW>
        fprintf("[STOP] No safe interval available for another refinement.\n");
        break;
    end

    % Bisect only the interval immediately preceding the failed layer.
    left = failIdx-1;
    right = failIdx;
    midpoint = 0.5*(currentPositions(left,:)+currentPositions(right,:));
    currentPositions = [currentPositions(1:left,:); midpoint; ...
        currentPositions(right:end,:)];
    rr.recovery = "bisect_failed_interval";
    rr.insertedPoint = midpoint;
    rr.insertedInterval = [left right];
    rr.pointsAfter = size(currentPositions,1);
    rounds(end+1,1) = rr; %#ok<AGROW>
    fprintf("[Refine] failed layer %d: bisect interval %d-%d -> %d points.\n", ...
        failIdx,left,right,size(currentPositions,1));
end

% Enter this stage only after sparse bisection has failed.  Normal samples
% therefore pay no beam/angular-redundancy cost.
postRecoveryUsed = false;
postRecoveryAttempts = 0;
postRecoveryTimeSec = 0;
postRecoveryIkCalls = 0;
if options.enablePostRecovery && ~finalCore.complete && ~isempty(currentPositions)
    fprintf("\n[Post-recovery] sparse refinement failed; trying bounded beam/angular recovery.\n");
    for ibeam = 1:numel(options.recoveryBeamWidths)
        b = options.recoveryBeamWidths(ibeam);
        recoveryOptions = localCoreOptions(options);
        recoveryOptions.numPoints = size(currentPositions,1);
        recoveryOptions.seamPositionsWP = currentPositions;
        recoveryOptions.beamWidth = b;
        recoveryOptions.firstPointMaxStates = max(options.firstPointMaxStates,b);
        recoveryOptions.maxPoseSeeds = max(options.maxPoseSeeds,12);
        recoveryOptions.maxPoseNeighbors = options.recoveryMaxPoseNeighbors;
        recoveryOptions.poseNeighborhoodCost = options.recoveryPoseNeighborhoodCost;
        recoveryOptions.maxPoseStepRollDeg = options.recoveryMaxPoseStepRollDeg;
        recoveryOptions.maxPoseStepTravelDeg = options.recoveryMaxPoseStepTravelDeg;
        recoveryOptions.progressivePoseNeighbors = false;
        recoveryOptions.angularRedundancyFallback = true;
        recoveryOptions.angularFallbackRollDeg = options.recoveryRollFallbackDeg;
        recoveryOptions.angularFallbackTravelDeg = options.recoveryTravelFallbackDeg;
        recoveryOptions.enableFallback = false;
        recoveryFile = sprintf("%s_recovery_beam%d.mat", ...
            options.roundOutputPrefix,b);
        if ~startsWith(string(recoveryFile),filesep)
            recoveryFile = fullfile(here,recoveryFile);
        end
        recoveryOptions.outputFile = recoveryFile;
        postRecoveryAttempts = postRecoveryAttempts + 1;
        tRecovery = tic;
        candidate = replan_weld_seam_generalization_core(recoveryOptions);
        dtRecovery = toc(tRecovery);
        postRecoveryTimeSec = postRecoveryTimeSec + dtRecovery;
        postRecoveryIkCalls = postRecoveryIkCalls + candidate.continuationStats.ikCalls;
        postRecoveryUsed = true;
        rr = localEmptyRound();
        rr.round = options.maxRefinementRounds + ibeam;
        rr.pointsBefore = size(currentPositions,1);
        rr.pointsAfter = size(candidate.seam.positionsWP,1);
        rr.complete = candidate.complete;
        rr.furthestPoint = localFurthestPoint(candidate.stateBank);
        recoveryFail = localFirstEmptyLayer(candidate.stateBank);
        if isempty(recoveryFail), recoveryFail = NaN; end
        rr.failedLayer = recoveryFail;
        rr.algorithmTimeSec = candidate.algorithmElapsedSec;
        rr.wallTimeSec = dtRecovery;
        rr.ikCalls = candidate.continuationStats.ikCalls;
        rr.maxJointJumpDeg = candidate.maxJointJumpDeg;
        rr.maxSegmentTcpErrorMm = candidate.tcpMetrics.maxSegmentPositionErrorMm;
        rr.recovery = sprintf("post_recovery_beam_%d_angular",b);
        rounds(end+1,1) = rr; %#ok<AGROW>
        finalCore = candidate;
        currentPositions = candidate.seam.positionsWP;
        fprintf("[Post-recovery] beam=%d, complete=%d, points=%d, time=%.3fs\n", ...
            b,candidate.complete,size(currentPositions,1),dtRecovery);
        if candidate.complete, break; end
    end
end

result = finalCore;
if postRecoveryUsed
    result.algorithmName = "A-ARC-IK feasibility refinement + bounded recovery";
else
    result.algorithmName = "A-ARC-IK feasibility refinement";
end
result.initialPoints = options.initialPoints;
result.finalPointCount = size(currentPositions,1);
result.finalSeamPositionsWP = currentPositions;
result.refinementRounds = rounds;
result.refinementRoundCount = numel(rounds);
result.feasibilityAdaptiveComplete = finalCore.complete;
result.feasibilityAdaptiveWallTimeSec = toc(totalTimer);
result.feasibilityAdaptiveAlgorithmTimeSec = sum([rounds.algorithmTimeSec]);
result.feasibilityAdaptiveIkCalls = sum([rounds.ikCalls]);
result.postRecoveryUsed = postRecoveryUsed;
result.postRecoveryAttempts = postRecoveryAttempts;
result.postRecoveryTimeSec = postRecoveryTimeSec;
result.postRecoveryIkCalls = postRecoveryIkCalls;

% Once a feasible sparse chain exists, perform the same selective execution
% waypoint correction used by the validated adaptive planner.  This is a
% second stage: feasibility bisection never hides a TCP interpolation error.
if finalCore.complete && ~isempty(finalCore.qPath)
    tPrecision = tic;
    precision = localCorrectExecution(finalCore,options);
    result.qPathExecution = precision.qPathExecution;
    result.tcpTargetExecution = precision.tcpTargetExecution;
    result.initialTcpMetrics = precision.initialTcpMetrics;
    result.finalTcpMetrics = precision.finalTcpMetrics;
    result.adaptiveSegmentReport = precision.segmentReport;
    result.adaptiveCorrectionComplete = precision.complete;
    result.adaptiveIkCalls = precision.ikCalls;
    result.adaptiveIkElapsedSec = precision.ikElapsedSec;
    result.adaptiveCollisionChecks = precision.collisionChecks;
    result.precisionAlgorithmTimeSec = toc(tPrecision);
    result.precisionInsertedPoints = size(precision.qPathExecution,1)-size(finalCore.qPath,1);
    if size(precision.qPathExecution,1)>1
        dqExec = atan2(sin(diff(precision.qPathExecution,1,1)), ...
            cos(diff(precision.qPathExecution,1,1)));
        result.maxJointJumpDeg = max(abs(rad2deg(dqExec)),[],'all');
    end
    result.nodeCollisionFree = precision.finalTcpMetrics.nodeCollisionFree;
    result.edgeCollisionFree = precision.finalTcpMetrics.edgeCollisionFree;
    result.complete = finalCore.complete && precision.complete;
else
    result.qPathExecution = zeros(0,6);
    result.tcpTargetExecution = cell(0,1);
    result.adaptiveCorrectionComplete = false;
    result.adaptiveIkCalls = 0;
    result.adaptiveIkElapsedSec = 0;
    result.adaptiveCollisionChecks = 0;
    result.precisionAlgorithmTimeSec = 0;
    result.precisionInsertedPoints = 0;
    result.complete = false;
end
result.outputFile = localSaveResult(result,here,options.outputFile);

fprintf("\n========================================\n");
fprintf("A-ARC-IK feasibility result\n");
fprintf("Initial points       : %d\n",options.initialPoints);
fprintf("Final points         : %d\n",result.finalPointCount);
fprintf("Refinement rounds    : %d\n",result.refinementRoundCount);
fprintf("Complete             : %d\n",result.feasibilityAdaptiveComplete);
fprintf("Total algorithm time : %.3f s\n",result.feasibilityAdaptiveAlgorithmTimeSec);
fprintf("Total wall time      : %.3f s\n",result.feasibilityAdaptiveWallTimeSec);
if isfield(result,"finalTcpMetrics")
    fprintf("Execution points     : %d\n",size(result.qPathExecution,1));
    fprintf("Precision points     : %d\n",result.precisionInsertedPoints);
    fprintf("Final segment TCP    : %.6f mm\n",max(result.finalTcpMetrics.segmentPositionErrorMm));
    fprintf("Precision PASS       : %d\n",result.adaptiveCorrectionComplete);
end
fprintf("Overall PASS         : %d\n",result.complete);
fprintf("Saved result         : %s\n",result.outputFile);
end

function opts = localCoreOptions(options)
names = { ...
    "numPoints","endMarginM","coarseTravelDeg","coarseRollDeg", ...
    "beamWidth","firstPointMaxStates","maxPoseSeeds","seedPerturbDeg", ...
    "maxPoseNeighbors","maxPoseStepRollDeg","maxPoseStepTravelDeg", ...
    "numRandomSeedsFirstPoint","maxRollStepDeg","maxTravelStepDeg", ...
    "maxJointStepDeg","edgeResolutionDeg","maxBacktrackLayers", ...
    "positionToleranceM","orientationToleranceDeg","sideToleranceDeg", ...
    "sideMarginM","checkSelfCollision","animate","publish", ...
    "reuseIKSolver","reuseCandidateCache","lazyCollision", ...
    "progressivePoseNeighbors","progressiveInitialPoseNeighbors", ...
    "progressiveMaxPoseNeighbors","disableLocalRecovery", ...
    "poseNeighborhoodCost","angularRedundancyFallback", ...
    "angularFallbackRollDeg","angularFallbackTravelDeg","enableFallback", ...
    "randomSeed","outputFile","seamPositionsWP","seamSpec"};
opts = struct();
for i=1:numel(names)
    name = names{i};
    if isfield(options,name), opts.(name) = options.(name); end
end
end

function idx = localFirstEmptyLayer(stateBank)
idx = find(cellfun(@isempty,stateBank),1,"first");
end

function idx = localFurthestPoint(stateBank)
idx = find(~cellfun(@isempty,stateBank),1,"last");
if isempty(idx), idx = 0; end
end

function r = localEmptyRound()
r = struct("round",NaN,"pointsBefore",NaN,"pointsAfter",NaN, ...
    "complete",false,"furthestPoint",NaN,"failedLayer",NaN, ...
    "algorithmTimeSec",NaN,"wallTimeSec",NaN,"ikCalls",NaN, ...
    "maxJointJumpDeg",NaN,"maxSegmentTcpErrorMm",NaN, ...
    "recovery","","insertedPoint",[NaN NaN NaN], ...
    "insertedInterval",[NaN NaN]);
end

function out = localMergeOptions(defaults,user)
out = defaults;
if isempty(user), return; end
names = fieldnames(user);
for i=1:numel(names), out.(names{i}) = user.(names{i}); end
end

function file = localSaveResult(result,here,fileName)
file = fileName;
if ~isfile(file) && ~startsWith(string(file),filesep)
    file = fullfile(here,file);
end
save(file,"result","-v7.3");
end

function out = localCorrectExecution(core,options)
scene = core.scene;
qWeld = core.qPath;
Ttargets = localPathTargets(core);
initial = localMeasureExecution(scene,qWeld,Ttargets,3,true);
ik = inverseKinematics("RigidBodyTree",scene.robot);
limits = localRobotJointLimits(scene.robot);
N = size(qWeld,1);
report = repmat(localEmptySegmentReport(),1,N-1);
qExec = qWeld(1,:);
TExec = Ttargets(1);
ikCalls = 0; ikElapsed = 0; collisionChecks = 0;
for i=1:N-1
    report(i).initialErrorMm = initial.segmentPositionErrorMm(i);
    report(i).initialOrientationErrorDeg = initial.segmentOrientationErrorDeg(i);
    report(i).initialJointJumpDeg = initial.segmentJointJumpDeg(i);
    report(i).initialCollisionFree = initial.segmentCollisionFree(i);
    need = initial.segmentPositionErrorMm(i)>options.maxSegmentTcpErrorMm || ...
        initial.segmentJointJumpDeg(i)>options.maxJointStepDeg || ...
        ~initial.segmentCollisionFree(i);
    report(i).correctionTrigger = localTriggerText( ...
        initial.segmentPositionErrorMm(i),options.maxSegmentTcpErrorMm, ...
        initial.segmentJointJumpDeg(i),options.maxJointStepDeg, ...
        initial.segmentCollisionFree(i));
    correction = localEmptyCorrection();
    if need
        [correction,calls,elapsed,checks] = localRefineSegment( ...
            scene,ik,limits,qWeld(i,:),qWeld(i+1,:),Ttargets{i},Ttargets{i+1}, ...
            initial.segmentPositionErrorMm(i),options);
        ikCalls=ikCalls+calls; ikElapsed=ikElapsed+elapsed; collisionChecks=collisionChecks+checks;
    end
    report(i).ikCalls=correction.ikCalls;
    report(i).ikElapsedSec=correction.ikElapsedSec;
    report(i).correctionAttempted=correction.attempted;
    report(i).correctionAccepted=correction.accepted;
    report(i).insertedPoints=size(correction.qIntermediate,1);
    if correction.accepted
        qExec=[qExec;correction.qIntermediate;qWeld(i+1,:)]; %#ok<AGROW>
        TExec=[TExec;correction.TIntermediate;Ttargets(i+1)]; %#ok<AGROW>
    else
        qExec=[qExec;qWeld(i+1,:)]; %#ok<AGROW>
        TExec=[TExec;Ttargets(i+1)]; %#ok<AGROW>
    end
end
final=localMeasureExecution(scene,qExec,TExec,3,true);
accepted=[report.correctionAccepted] | ~[report.correctionAttempted];
out=struct("qPathExecution",qExec,"tcpTargetExecution",{TExec}, ...
    "initialTcpMetrics",initial,"finalTcpMetrics",final,"segmentReport",report, ...
    "complete",all(accepted)&&final.allSegmentsChecked&& ...
        all(final.segmentCollisionFree)&&final.nodeCollisionFree&&final.edgeCollisionFree, ...
    "ikCalls",ikCalls,"ikElapsedSec",ikElapsed,"collisionChecks",collisionChecks);
end

function [correction,ikCalls,ikElapsed,collisionChecks] = localRefineSegment( ...
    scene,ik,limits,q1,q2,T1,T2,initialErrorMm,options)
correction=localEmptyCorrection(); correction.attempted=true;
ikCalls=0; ikElapsed=0; collisionChecks=0;
n0=max(1,min(options.maxIntermediatePoints, ...
    ceil(initialErrorMm/options.maxSegmentTcpErrorMm)-1));
trialCounts=unique([n0,min(options.maxIntermediatePoints,n0+1), ...
    min(options.maxIntermediatePoints,n0+2)]);
for nExtra=trialCounts
    qMid=zeros(nExtra,6); TMid=cell(nExtra,1); qPrev=q1; ok=true;
    localCalls=0; localTime=0; localChecks=0;
    for k=1:nExtra
        s=k/(nExtra+1); Tgoal=localInterpolatePose(T1,T2,s);
        qSeed=q1+s*localWrapRad(q2-q1); t=tic;
        [qSol,~]=ik(scene.tcpBodyName,Tgoal,[1 1 1 1 1 1],qSeed);
        localTime=localTime+toc(t); localCalls=localCalls+1;
        qSol=qSeed+localWrapRad(double(qSol(:).')-qSeed);
        if numel(qSol)~=6 || any(~isfinite(qSol)) || ...
                any(qSol<limits(:,1).'-1e-8)||any(qSol>limits(:,2).'+1e-8)
            ok=false; break;
        end
        Tactual=getTransform(scene.robot,qSol,scene.tcpBodyName);
        posErr=norm(Tactual(1:3,4)-Tgoal(1:3,4));
        c=(trace(Tgoal(1:3,1:3).'*Tactual(1:3,1:3))-1)/2;
        oriErr=acos(max(-1,min(1,c)));
        if posErr>1e-4||rad2deg(oriErr)>0.5||~localFullStateFree(scene,qSol)
            ok=false; break;
        end
        localChecks=localChecks+1;
        if ~localEdgeFree(scene,qPrev,qSol,3), ok=false; break; end
        qMid(k,:)=qSol; TMid{k}=Tgoal; qPrev=qSol;
    end
    if ok, if ~localEdgeFree(scene,qPrev,q2,3), ok=false; end; localChecks=localChecks+1; end
    ikCalls=ikCalls+localCalls; ikElapsed=ikElapsed+localTime; collisionChecks=collisionChecks+localChecks;
    if ok
        correction.accepted=true; correction.qIntermediate=qMid; correction.TIntermediate=TMid;
        correction.ikCalls=localCalls; correction.ikElapsedSec=localTime; return;
    end
end
correction.ikCalls=ikCalls; correction.ikElapsedSec=ikElapsed;
end

function metrics=localMeasureExecution(scene,qPath,Ttargets,edgeResolutionDeg,checkCollision)
N=size(qPath,1);
metrics=struct("nodePositionErrorMm",zeros(N,1),"nodeOrientationErrorDeg",zeros(N,1), ...
    "segmentPositionErrorMm",zeros(N-1,1),"segmentOrientationErrorDeg",zeros(N-1,1), ...
    "segmentJointJumpDeg",zeros(N-1,1),"segmentCollisionFree",true(N-1,1), ...
    "nodeCollisionFree",true,"edgeCollisionFree",true,"checkedSegments",N-1,"allSegmentsChecked",N>0);
for i=1:N
    T=getTransform(scene.robot,qPath(i,:),scene.tcpBodyName); Ti=Ttargets{i};
    metrics.nodePositionErrorMm(i)=norm(T(1:3,4)-Ti(1:3,4))*1000;
    c=(trace(Ti(1:3,1:3).'*T(1:3,1:3))-1)/2;
    metrics.nodeOrientationErrorDeg(i)=rad2deg(acos(max(-1,min(1,c))));
    if checkCollision&&~localFullStateFree(scene,qPath(i,:)), metrics.nodeCollisionFree=false; end
end
for i=1:N-1
    T1=Ttargets{i}; T2=Ttargets{i+1}; dq=localWrapRad(qPath(i+1,:)-qPath(i,:));
    n=max(2,ceil(max(abs(rad2deg(dq)))/edgeResolutionDeg)+1); metrics.segmentJointJumpDeg(i)=max(abs(rad2deg(dq)));
    for k=0:n-1
        s=k/(n-1); q=qPath(i,:)+s*dq; T=getTransform(scene.robot,q,scene.tcpBodyName);
        p=(1-s)*T1(1:3,4)+s*T2(1:3,4); metrics.segmentPositionErrorMm(i)=max(metrics.segmentPositionErrorMm(i),norm(T(1:3,4)-p)*1000);
        R=localInterpolatePose(T1,T2,s); c=(trace(R(1:3,1:3).'*T(1:3,1:3))-1)/2;
        metrics.segmentOrientationErrorDeg(i)=max(metrics.segmentOrientationErrorDeg(i),rad2deg(acos(max(-1,min(1,c)))));
    end
    if checkCollision
        metrics.segmentCollisionFree(i)=localEdgeFree(scene,qPath(i,:),qPath(i+1,:),edgeResolutionDeg);
        if ~metrics.segmentCollisionFree(i), metrics.edgeCollisionFree=false; end
    end
end
end

function Ttargets=localPathTargets(result)
N=size(result.qPath,1); Ttargets=repmat({eye(4)},N,1);
for i=1:N
    s=result.stateBank{i}; Q=vertcat(s.q); D=rad2deg(atan2(sin(Q-result.qPath(i,:)),cos(Q-result.qPath(i,:)))); [~,j]=min(vecnorm(D,2,2)); Ttargets{i}=s(j).Ttarget;
end
end

function T=localInterpolatePose(T1,T2,s)
q1=rotm2quat(T1(1:3,1:3)); q2=rotm2quat(T2(1:3,1:3)); if dot(q1,q2)<0,q2=-q2;end
q=(1-s)*q1+s*q2; q=q/norm(q); T=eye(4); T(1:3,1:3)=quat2rotm(q); T(1:3,4)=(1-s)*T1(1:3,4)+s*T2(1:3,4);
end

function ok=localFullStateFree(scene,q)
try, [env,~,~]=checkCollision(scene.robot,q,scene.environment,"IgnoreSelfCollision","on","Exhaustive","on");
catch, env=checkCollision(scene.robot,q,scene.environment,"IgnoreSelfCollision","on"); end
if any(env(:)), ok=false; return; end
try, self=checkCollision(scene.robot,q,"IgnoreSelfCollision","off", "SkippedSelfCollisions",scene.selfCollisionSkipPairs,"Exhaustive","on");
catch, self=checkCollision(scene.robot,q,"IgnoreSelfCollision","off", "SkippedSelfCollisions",scene.selfCollisionSkipPairs); end
ok=~any(self(:));
end

function ok=localEdgeFree(scene,q1,q2,resDeg)
dq=localWrapRad(q2-q1); n=max(1,ceil(max(abs(rad2deg(dq)))/resDeg)); ok=true;
for k=0:n, if ~localFullStateFree(scene,q1+(k/n)*dq), ok=false; return; end, end
end

function limits=localRobotJointLimits(robot)
limits=zeros(6,2); j=0;
for i=1:numel(robot.Bodies), jt=robot.Bodies{i}.Joint; if strcmpi(jt.Type,"revolute")||strcmpi(jt.Type,"continuous"), j=j+1; limits(j,:)=double(jt.PositionLimits); end, end
end

function txt=localTriggerText(errorMm,errorLimit,jumpDeg,jumpLimit,collisionFree)
parts=strings(0,1); if errorMm>errorLimit, parts(end+1)="tcp"; end %#ok<AGROW>
if jumpDeg>jumpLimit, parts(end+1)="joint_jump"; end %#ok<AGROW>
if ~collisionFree, parts(end+1)="collision"; end %#ok<AGROW>
if isempty(parts), txt="none"; else, txt=strjoin(parts,"+"); end
end

function c=localEmptyCorrection()
c=struct("attempted",false,"accepted",false,"qIntermediate",zeros(0,6), ...
    "TIntermediate",{cell(0,1)},"ikCalls",0,"ikElapsedSec",0);
end

function s=localEmptySegmentReport()
s=struct("initialErrorMm",0,"initialOrientationErrorDeg",0,"initialJointJumpDeg",0, ...
    "initialCollisionFree",true,"correctionTrigger","none","elapsedSec",0, ...
    "ikCalls",0,"ikElapsedSec",0,"correctionAttempted",false,"correctionAccepted",false,"insertedPoints",0);
end

function x=localWrapRad(x), x=atan2(sin(x),cos(x)); end
