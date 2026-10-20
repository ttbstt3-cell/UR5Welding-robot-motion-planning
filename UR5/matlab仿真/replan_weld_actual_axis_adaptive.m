function result = replan_weld_actual_axis_adaptive(options)
%REPLAN_WELD_ACTUAL_AXIS_ADAPTIVE
% Cold-start continuation planning followed by selective Cartesian TCP
% correction on only the weld segments that exceed a configurable error.
% The 80 weld nodes are never removed; inserted points are execution and
% validation waypoints between the original weld nodes.

if nargin < 1 || isempty(options), options = struct(); end
defaults = struct( ...
    "maxSegmentTcpErrorMm",0.5, ...
    "maxSegmentJointJumpDeg",15, ...
    "maxIntermediatePoints",4, ...
    "maxAdaptiveRounds",2, ...
    "animate",false,"publish",false, ...
    "enableFallback",false, ...
    "beamWidth",1,"firstPointMaxStates",2,"maxPoseSeeds",3, ...
    "numPoints",80,"endMarginM",0.025, ...
    "coarseTravelDeg",[-15 0 15], ...
    "coarseRollDeg",0:45:315, ...
    "seedPerturbDeg",8,"maxPoseNeighbors",9, ...
    "maxPoseStepRollDeg",15,"maxPoseStepTravelDeg",5, ...
    "numRandomSeedsFirstPoint",10, ...
    "maxRollStepDeg",45,"maxTravelStepDeg",15, ...
    "maxJointStepDeg",15,"edgeResolutionDeg",3, ...
    "maxBacktrackLayers",8, ...
    "positionToleranceM",1e-4,"orientationToleranceDeg",0.5, ...
    "sideToleranceDeg",5,"sideMarginM",1e-3, ...
    "checkSelfCollision",true, ...
    "reuseIKSolver",true,"reuseCandidateCache",true,"lazyCollision",true, ...
    "progressivePoseNeighbors",false, ...
    "progressiveInitialPoseNeighbors",5, ...
    "progressiveMaxPoseNeighbors",9, ...
    "disableLocalRecovery",false, ...
    "poseNeighborhoodCost","zero", ...
    "angularRedundancyFallback",false, ...
    "angularFallbackRollDeg",[10 20], ...
    "angularFallbackTravelDeg",[0 5 10], ...
    "randomSeed",20260923, ...
    "outputFile","weld_actual_axis_replan_adaptive.mat", ...
    "baseOutputFile","weld_actual_axis_replan_adaptive_base.mat");
options = localMergeOptions(defaults,options);

here = fileparts(mfilename("fullpath"));
tTotal = tic;

% Fresh continuation planning.  No old qPath or state graph is loaded.
baseOptions = options;
baseOptions.outputFile = options.baseOutputFile;
baseOptions.animate = false;
baseOptions.publish = false;
baseOptions.enableFallback = options.enableFallback;
tBase = tic;
baseResult = replan_weld_actual_axis_continuation(baseOptions);
baseWallTimeSec = toc(tBase);
tAdaptive = tic;

result = baseResult;
result.adaptiveCorrectionEnabled = true;
result.adaptiveOptions = struct( ...
    "maxSegmentTcpErrorMm",options.maxSegmentTcpErrorMm, ...
    "maxSegmentJointJumpDeg",options.maxSegmentJointJumpDeg, ...
    "maxIntermediatePoints",options.maxIntermediatePoints, ...
    "maxAdaptiveRounds",options.maxAdaptiveRounds);

if ~baseResult.complete || isempty(baseResult.qPath)
    result.adaptiveCorrectionComplete = false;
    result.adaptiveAlgorithmElapsedSec = toc(tAdaptive);
    result.baseWallTimeSec = baseWallTimeSec;
    result.adaptiveWallTimeSec = result.adaptiveAlgorithmElapsedSec;
    result.totalPlanningTimeSec = toc(tTotal);
    result.outputFile = localSaveResult(result,here,options.outputFile);
    return;
end

scene = baseResult.scene;
qWeld = baseResult.qPath;
Ttargets = localPathTargets(baseResult);
initial = localMeasureExecution(scene,qWeld,Ttargets,3,true);

% Construct one solver and one limit vector for all adaptive intermediate IK.
ik = inverseKinematics("RigidBodyTree",scene.robot);
limits = localRobotJointLimits(scene.robot);

N = size(qWeld,1);
segmentReport = repmat(localEmptySegmentReport(),1,N-1);
qExec = qWeld(1,:);
TExec = Ttargets(1);
adaptiveIkCalls = 0;
adaptiveIkElapsed = 0;
adaptiveCollisionChecks = 0;

for i = 1:N-1
    segmentReport(i).initialErrorMm = initial.segmentPositionErrorMm(i);
    segmentReport(i).initialOrientationErrorDeg = initial.segmentOrientationErrorDeg(i);
    segmentReport(i).initialJointJumpDeg = initial.segmentJointJumpDeg(i);
    segmentReport(i).initialCollisionFree = initial.segmentCollisionFree(i);
    tSeg = tic;
    correction = localEmptyCorrection();
    needCorrection = initial.segmentPositionErrorMm(i) > options.maxSegmentTcpErrorMm || ...
        initial.segmentJointJumpDeg(i) > options.maxSegmentJointJumpDeg || ...
        ~initial.segmentCollisionFree(i);
    segmentReport(i).correctionTrigger = localTriggerText( ...
        initial.segmentPositionErrorMm(i),options.maxSegmentTcpErrorMm, ...
        initial.segmentJointJumpDeg(i),options.maxSegmentJointJumpDeg, ...
        initial.segmentCollisionFree(i));
    if needCorrection
        [correction,ikCalls,ikElapsed,collisionChecks] = localRefineSegment( ...
            scene,ik,limits,qWeld(i,:),qWeld(i+1,:),Ttargets{i},Ttargets{i+1}, ...
            initial.segmentPositionErrorMm(i),options);
        adaptiveIkCalls = adaptiveIkCalls + ikCalls;
        adaptiveIkElapsed = adaptiveIkElapsed + ikElapsed;
        adaptiveCollisionChecks = adaptiveCollisionChecks + collisionChecks;
    end
    segmentReport(i).elapsedSec = toc(tSeg);
    segmentReport(i).ikCalls = correction.ikCalls;
    segmentReport(i).ikElapsedSec = correction.ikElapsedSec;
    segmentReport(i).correctionAttempted = correction.attempted;
    segmentReport(i).correctionAccepted = correction.accepted;
    segmentReport(i).insertedPoints = size(correction.qIntermediate,1);
    if correction.accepted
        qExec = [qExec;correction.qIntermediate;qWeld(i+1,:)]; %#ok<AGROW>
        TExec = [TExec;correction.TIntermediate;Ttargets(i+1)]; %#ok<AGROW>
    else
        qExec = [qExec;qWeld(i+1,:)]; %#ok<AGROW>
        TExec = [TExec;Ttargets(i+1)]; %#ok<AGROW>
    end
end

final = localMeasureExecution(scene,qExec,TExec,3,true);
result.qPath = qWeld;                 % exactly 80 weld nodes
result.qPathExecution = qExec;        % 80 weld nodes + adaptive points
result.tcpTargetExecution = TExec;
result.initialTcpMetrics = initial;
result.finalTcpMetrics = final;
result.adaptiveSegmentReport = segmentReport;
result.adaptiveCorrectionComplete = all([segmentReport.correctionAccepted] | ...
    ~[segmentReport.correctionAttempted]) && final.allSegmentsChecked && ...
    all(final.segmentCollisionFree);
result.adaptiveIkCalls = adaptiveIkCalls;
result.adaptiveIkElapsedSec = adaptiveIkElapsed;
result.adaptiveCollisionChecks = adaptiveCollisionChecks;
result.adaptiveAlgorithmElapsedSec = toc(tAdaptive);
result.baseWallTimeSec = baseWallTimeSec;
result.adaptiveWallTimeSec = result.adaptiveAlgorithmElapsedSec;
result.algorithmElapsedSec = baseResult.algorithmElapsedSec + ...
    result.adaptiveAlgorithmElapsedSec;
result.totalPlanningTimeSec = toc(tTotal);
result.nodeCollisionFree = baseResult.nodeCollisionFree && final.nodeCollisionFree;
result.edgeCollisionFree = baseResult.edgeCollisionFree && final.edgeCollisionFree;
result.complete = baseResult.complete && result.adaptiveCorrectionComplete && ...
    result.nodeCollisionFree && result.edgeCollisionFree;
result.outputFile = localSaveResult(result,here,options.outputFile);

fprintf("\n========================================\n");
fprintf("ADAPTIVE TCP CORRECTION RESULT\n");
fprintf("========================================\n");
fprintf("Original weld nodes       : %d\n",N);
fprintf("Execution points          : %d\n",size(qExec,1));
fprintf("Segments over %.3f mm     : %d\n",options.maxSegmentTcpErrorMm, ...
    sum(initial.segmentPositionErrorMm > options.maxSegmentTcpErrorMm));
fprintf("Segments over %.3f deg    : %d\n",options.maxSegmentJointJumpDeg, ...
    sum(initial.segmentJointJumpDeg > options.maxSegmentJointJumpDeg));
fprintf("Segments with collision    : %d\n",sum(~initial.segmentCollisionFree));
fprintf("Adaptive points inserted  : %d\n",size(qExec,1)-N);
fprintf("Adaptive IK calls         : %d\n",adaptiveIkCalls);
fprintf("Adaptive IK time          : %.3f s\n",adaptiveIkElapsed);
fprintf("Base wall time            : %.3f s\n",baseWallTimeSec);
fprintf("Adaptive phase time       : %.3f s\n",result.adaptiveAlgorithmElapsedSec);
fprintf("Initial max segment TCP   : %.6f mm\n",max(initial.segmentPositionErrorMm));
fprintf("Final max segment TCP     : %.6f mm\n",max(final.segmentPositionErrorMm));
fprintf("Final max segment orient  : %.6f deg\n",max(final.segmentOrientationErrorDeg));
fprintf("Final node collision free : %d\n",final.nodeCollisionFree);
fprintf("Final edge collision free : %d\n",final.edgeCollisionFree);
fprintf("Total algorithm time      : %.3f s\n",result.algorithmElapsedSec);
fprintf("Total wall time           : %.3f s\n",result.totalPlanningTimeSec);
fprintf("PASS                      : %d\n",result.complete);
end

function [correction,ikCalls,ikElapsed,collisionChecks] = localRefineSegment( ...
    scene,ik,limits,q1,q2,T1,T2,initialErrorMm,options)
correction = localEmptyCorrection();
correction.attempted = true;
ikCalls = 0; ikElapsed = 0; collisionChecks = 0;
n0 = max(1,min(options.maxIntermediatePoints, ...
    ceil(initialErrorMm/options.maxSegmentTcpErrorMm)-1));
trialCounts = unique([n0,min(options.maxIntermediatePoints,n0+1), ...
    min(options.maxIntermediatePoints,n0+2)]);
for nExtra = trialCounts
    qMid = zeros(nExtra,6); TMid = cell(nExtra,1);
    qPrev = q1; ok = true; localCalls = 0; localTime = 0; localChecks = 0;
    for k=1:nExtra
        s=k/(nExtra+1);
        Tgoal=localInterpolatePose(T1,T2,s);
        qSeed=q1+s*localWrapRad(q2-q1);
        t=tic;
        [qSol,~]=ik(scene.tcpBodyName,Tgoal,[1 1 1 1 1 1],qSeed);
        localTime=localTime+toc(t); localCalls=localCalls+1;
        qSol=qSeed+localWrapRad(double(qSol(:).')-qSeed);
        if numel(qSol)~=6 || any(~isfinite(qSol)) || ...
                any(qSol<limits(:,1).'-1e-8) || any(qSol>limits(:,2).'+1e-8)
            ok=false; break;
        end
        Tactual=getTransform(scene.robot,qSol,scene.tcpBodyName);
        posErr=norm(Tactual(1:3,4)-Tgoal(1:3,4));
        c=(trace(Tgoal(1:3,1:3).'*Tactual(1:3,1:3))-1)/2;
        oriErr=acos(max(-1,min(1,c)));
        if posErr>1e-4 || rad2deg(oriErr)>0.5 || ~localFullStateFree(scene,qSol)
            ok=false; break;
        end
        localChecks=localChecks+1;
        if ~localEdgeFree(scene,qPrev,qSol,3), ok=false; break; end
        qMid(k,:)=qSol; TMid{k}=Tgoal; qPrev=qSol;
    end
    if ok
        if ~localEdgeFree(scene,qPrev,q2,3), ok=false; end
        localChecks=localChecks+1;
    end
    ikCalls=ikCalls+localCalls; ikElapsed=ikElapsed+localTime; collisionChecks=collisionChecks+localChecks;
    if ok
        correction.accepted=true;
        correction.qIntermediate=qMid;
        correction.TIntermediate=TMid;
        correction.ikCalls=localCalls;
        correction.ikElapsedSec=localTime;
        return;
    end
end
correction.ikCalls=ikCalls; correction.ikElapsedSec=ikElapsed;
end

function metrics = localMeasureExecution(scene,qPath,Ttargets,edgeResolutionDeg,checkCollision)
N=size(qPath,1);
metrics=struct("nodePositionErrorMm",zeros(N,1), ...
    "nodeOrientationErrorDeg",zeros(N,1),"segmentPositionErrorMm",zeros(N-1,1), ...
    "segmentOrientationErrorDeg",zeros(N-1,1), ...
    "segmentJointJumpDeg",zeros(N-1,1), ...
    "segmentCollisionFree",true(N-1,1), ...
    "nodeCollisionFree",true,"edgeCollisionFree",true, ...
    "checkedSegments",N-1,"allSegmentsChecked",N>0);
for i=1:N
    T=getTransform(scene.robot,qPath(i,:),scene.tcpBodyName);
    Ti=Ttargets{i};
    metrics.nodePositionErrorMm(i)=norm(T(1:3,4)-Ti(1:3,4))*1000;
    c=(trace(Ti(1:3,1:3).'*T(1:3,1:3))-1)/2;
    metrics.nodeOrientationErrorDeg(i)=rad2deg(acos(max(-1,min(1,c))));
    if checkCollision && ~localFullStateFree(scene,qPath(i,:)), metrics.nodeCollisionFree=false; end
end
for i=1:N-1
    T1=Ttargets{i}; T2=Ttargets{i+1};
    dq=localWrapRad(qPath(i+1,:)-qPath(i,:)); n=max(2,ceil(max(abs(rad2deg(dq)))/edgeResolutionDeg)+1);
    metrics.segmentJointJumpDeg(i)=max(abs(rad2deg(dq)));
    for k=0:n-1
        s=k/(n-1); q=qPath(i,:)+s*dq; T=getTransform(scene.robot,q,scene.tcpBodyName);
        p=(1-s)*T1(1:3,4)+s*T2(1:3,4);
        metrics.segmentPositionErrorMm(i)=max(metrics.segmentPositionErrorMm(i),norm(T(1:3,4)-p)*1000);
        R=localInterpolatePose(T1,T2,s); c=(trace(R(1:3,1:3).'*T(1:3,1:3))-1)/2;
        metrics.segmentOrientationErrorDeg(i)=max(metrics.segmentOrientationErrorDeg(i),rad2deg(acos(max(-1,min(1,c)))));
    end
    if checkCollision
        metrics.segmentCollisionFree(i)=localEdgeFree(scene,qPath(i,:),qPath(i+1,:),edgeResolutionDeg);
        if ~metrics.segmentCollisionFree(i), metrics.edgeCollisionFree=false; end
    end
end
end

function txt = localTriggerText(errorMm,errorLimit,jumpDeg,jumpLimit,collisionFree)
parts = strings(0,1);
if errorMm > errorLimit, parts(end+1) = "tcp"; end %#ok<AGROW>
if jumpDeg > jumpLimit, parts(end+1) = "joint_jump"; end %#ok<AGROW>
if ~collisionFree, parts(end+1) = "collision"; end %#ok<AGROW>
if isempty(parts), txt = "none"; else, txt = strjoin(parts,"+"); end
end

function Ttargets = localPathTargets(result)
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
try
    [env,~,~]=checkCollision(scene.robot,q,scene.environment, ...
        "IgnoreSelfCollision","on","Exhaustive","on");
catch
    env=checkCollision(scene.robot,q,scene.environment,"IgnoreSelfCollision","on");
end
if any(env(:)), ok=false; return; end
try
    self=checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",scene.selfCollisionSkipPairs,"Exhaustive","on");
catch
    self=checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",scene.selfCollisionSkipPairs);
end
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

function s=localEmptySegmentReport()
s=struct("initialErrorMm",0,"initialOrientationErrorDeg",0, ...
    "initialJointJumpDeg",0,"initialCollisionFree",true,"correctionTrigger","none", ...
    "elapsedSec",0, ...
    "ikCalls",0,"ikElapsedSec",0,"correctionAttempted",false, ...
    "correctionAccepted",false,"insertedPoints",0);
end

function c=localEmptyCorrection()
c=struct("attempted",false,"accepted",false,"qIntermediate",zeros(0,6), ...
    "TIntermediate",{cell(0,1)},"ikCalls",0,"ikElapsedSec",0);
end

function f=localSaveResult(result,here,name)
f=name; if ~isfile(f) && ~startsWith(string(f),filesep), f=fullfile(here,f); end
[d,d2,d3]=fileparts(f); if ~isempty(d) && ~isfolder(d), mkdir(d); end %#ok<ASGLU>
result.outputFile=f; save(f,"result","-v7.3");
end

function out=localMergeOptions(defaults,in)
out=defaults; names=fieldnames(defaults); for i=1:numel(names), if isfield(in,names{i})&&~isempty(in.(names{i})), out.(names{i})=in.(names{i}); end, end
end

function x=localWrapRad(x), x=atan2(sin(x),cos(x)); end
