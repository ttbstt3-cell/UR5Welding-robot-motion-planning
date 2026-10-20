function result = replan_weld_iklink_baseline(varargin)
%REPLAN_WELD_IKLINK_BASELINE
% MATLAB reproduction of the core IKLink planning strategy.
%
% The scene, IK solver, tool/wire model, workpiece, welding-side rules and
% collision validator are the existing project ones.  Only the planner is
% changed: generate many IK candidates at every weld waypoint, build a
% layered graph, and solve it with dynamic programming.
%
% This is deliberately an independent experiment.  It does not load a
% previous qPath/state graph and it does not modify A-ARC-IK, Gao results,
% ROS2, CoppeliaSim or the formal benchmark files.
%
% Usage:
%   r = replan_weld_iklink_baseline();
%   r = replan_weld_iklink_baseline(options);
%   r = replan_weld_iklink_baseline(scene, seamSpec, options);
%
% options.poseMode is "fixed" (closest to the IKLink fixed-pose baseline)
% or "ar" (the same layered planner over a bounded travel/roll set).

[scene, seamSpec, options] = localParseInputs(varargin{:});
defaults = struct( ...
    "numPoints",80,"endMarginM",0.025, ...
    "poseMode","fixed", ...
    "travelCandidatesDeg",-20:10:20, ...
    "rollCandidatesDeg",0:30:330, ...
    "numIKSeeds",200,"numRandomSeeds",180,"numPropagatedSeeds",20, ...
    "maxCandidatesPerLayer",200,"duplicateToleranceDeg",0.5, ...
    "maxJointStepDeg",15,"edgeResolutionDeg",3, ...
    "positionToleranceM",1e-4,"orientationToleranceDeg",0.5, ...
    "sideToleranceDeg",5,"sideMarginM",1e-3, ...
    "checkSelfCollision",true,"reuseCandidateCache",true, ...
    "randomSeed",20260928,"timeBudgetSec",60, ...
    "outputFile","iklink_matlab_seamA.mat","verbose",true);
options = localMerge(defaults,options);
if isempty(fieldnames(seamSpec)) && isfield(options,"seamSpec")
    seamSpec = options.seamSpec;
end

matlabDir = fileparts(mfilename("fullpath"));
if ~isfield(scene,"robot") || isempty(scene.robot)
    error("场景未正确建立。请检查 sync_ur5_joint_states.m。 ");
end

% The generator and validator use the same physical tool axis and side rule
% as the current project planners.
scene.seamSpec = localSeamSpec(scene,seamSpec);
scene.weldGunAxisWP = scene.seamSpec.gunAxisWP;
scene.weldGunAxisName = "measured_wire_axis_target";
scene.weldingSide = scene.seamSpec.description;
scene.weldingContactPolicy = "Wire/environment contact is hard; no target-seam exemption";
if ~isfield(scene,"selfCollisionSkipPairs")
    scene.selfCollisionSkipPairs = localStructuralSelfCollisionSkips(scene.robot);
end
if ~isfield(scene,"ikSolver")
    scene.ikSolver = inverseKinematics("RigidBodyTree",scene.robot);
end
if ~isfield(scene,"jointLimits")
    scene.jointLimits = localJointLimits(scene.robot);
end
if options.reuseCandidateCache && ~isfield(scene,"ikCandidateCache")
    scene.ikCandidateCache = containers.Map("KeyType","char","ValueType","any");
end

seam = localInferSeam(scene,options.numPoints,options.endMarginM,scene.seamSpec);
fprintf("\n========================================\n");
fprintf("IKLINK MATLAB LAYERED-DP BASELINE\n");
fprintf("========================================\n");
fprintf("Pose mode           : %s\n",string(options.poseMode));
fprintf("Weld layers         : %d\n",seam.numPoints);
fprintf("IK seeds/layer      : %d (%d random + %d propagated)\n", ...
    options.numIKSeeds,options.numRandomSeeds,options.numPropagatedSeeds);
fprintf("Joint step limit    : %.1f deg\n",options.maxJointStepDeg);
fprintf("Edge resolution     : %.1f deg\n",options.edgeResolutionDeg);
fprintf("Seam                : %s\n",scene.seamSpec.name);

rng(options.randomSeed,"twister");
algorithmTimer = tic;
stats = localStats();
layers = cell(1,seam.numPoints);
layerCandidateStats = repmat(localLayerStats(),1,seam.numPoints);
prevStates = repmat(localState(),0,1);
failureReason = "";
firstFeasibleTime = NaN;

for i = 1:seam.numPoints
    if toc(algorithmTimer) >= options.timeBudgetSec
        failureReason = "time_budget";
        break;
    end

    seeds = localSeeds(scene,prevStates,options);
    [states,ls] = localGenerateLayer(scene,seam,i,seeds,options);
    layers{i} = states;
    layerCandidateStats(i) = ls;
    stats.ikCalls = stats.ikCalls + ls.ikCalls;
    stats.ikPoseSuccess = stats.ikPoseSuccess + ls.ikPoseSuccess;
    stats.nodeChecks = stats.nodeChecks + ls.nodeChecks;
    stats.acceptedCandidates = stats.acceptedCandidates + numel(states);

    if isempty(states)
        failureReason = sprintf("empty_layer_%d",i);
        break;
    end
    prevStates = states;
    if options.verbose && (i == 1 || mod(i,5) == 0 || i == seam.numPoints)
        fprintf("Layer %3d/%3d: candidates=%3d, IK=%4d, elapsed=%.2fs\n", ...
            i,seam.numPoints,numel(states),ls.ikCalls,toc(algorithmTimer));
    end
end

generatedLayers = find(~cellfun(@isempty,layers),1,"last");
if isempty(generatedLayers), generatedLayers = 0; end

% Layered graph and DP.  No edge is accepted until the exact interpolated
% robot/tool/wire/workpiece/self-collision check passes.
[stateIndices,dpCost,graphStats,complete] = localLayeredDP( ...
    scene,layers,seam,options,generatedLayers,algorithmTimer);
stats.edgeChecks = graphStats.edgeChecks;
stats.edgeCollisionFree = graphStats.edgeCollisionFree;
stats.graphLayers = generatedLayers;

if complete
    qPath = localBacktrack(layers,stateIndices,seam.numPoints);
    firstFeasibleTime = toc(algorithmTimer);
else
    qPath = zeros(0,6);
    if strlength(failureReason) == 0
        if generatedLayers < seam.numPoints
            failureReason = sprintf("no_layer_%d",generatedLayers+1);
        else
            failureReason = "no_complete_layered_chain";
        end
    end
end

validation = localValidatePath(scene,qPath,layers,stateIndices,seam,options);
algorithmElapsedSec = toc(algorithmTimer);

result = struct();
result.algorithmName = "IKLink-style MATLAB layered candidate DP";
result.success = logical(complete && validation.nodeCollisionFree && validation.edgeCollisionFree && validation.tcpPass);
result.complete = result.success;
result.qPath = qPath;
result.stateIndices = stateIndices;
result.scene = localStripRuntime(scene);
result.seam = seam;
result.layers = layers;
result.layerCandidateStats = layerCandidateStats;
result.options = options;
result.stats = stats;
result.validation = validation;
result.failureReason = ternary(result.success,"",failureReason);
result.tff = firstFeasibleTime;
result.firstFeasibleTimeSec = firstFeasibleTime;
result.algorithmElapsedSec = algorithmElapsedSec;
result.ikCalls = stats.ikCalls;
result.nodeChecks = stats.nodeChecks;
result.edgeChecks = stats.edgeChecks;
result.maxJointJump = validation.maxJointJumpDeg;
result.maxJointJumpDeg = validation.maxJointJumpDeg;
result.maxTcpError = validation.maxSegmentTcpErrorMm;
result.maxTcpErrorMm = validation.maxSegmentTcpErrorMm;
result.nodeCollisionFree = validation.nodeCollisionFree;
result.edgeCollisionFree = validation.edgeCollisionFree;
result.pathCost = dpCost;
result.outputFile = localAbsolutePath(matlabDir,options.outputFile);

fprintf("\nIKLink MATLAB result\n");
fprintf("  success             : %d\n",result.success);
fprintf("  first feasible time : %.3f s\n",firstFeasibleTime);
fprintf("  algorithm time      : %.3f s\n",algorithmElapsedSec);
fprintf("  IK calls            : %d\n",stats.ikCalls);
fprintf("  candidates          : %d\n",stats.acceptedCandidates);
fprintf("  edge tests/free     : %d / %d\n",stats.edgeChecks,stats.edgeCollisionFree);
fprintf("  max joint jump      : %.3f deg\n",validation.maxJointJumpDeg);
fprintf("  max segment TCP err : %.6f mm\n",validation.maxSegmentTcpErrorMm);
fprintf("  node/edge collision : %d / %d\n",validation.nodeCollisionFree,validation.edgeCollisionFree);
if strlength(result.failureReason) > 0, fprintf("  failure reason      : %s\n",result.failureReason); end

save(result.outputFile,"result","-v7.3");
fprintf("Saved: %s\n",result.outputFile);
end

function [scene,spec,options] = localParseInputs(varargin)
scene = struct(); spec = struct(); options = struct();
if nargin >= 1 && isstruct(varargin{1}) && isfield(varargin{1},"robot")
    scene = varargin{1};
    if nargin >= 2 && ~isempty(varargin{2}), spec = varargin{2}; end
    if nargin >= 3 && ~isempty(varargin{3}), options = varargin{3}; end
elseif nargin >= 1 && ~isempty(varargin{1})
    options = varargin{1};
    if nargin >= 2 && ~isempty(varargin{2}), spec = varargin{2}; end
end
if isempty(fieldnames(spec)) && isstruct(options) && isfield(options,"seamSpec")
    spec = options.seamSpec;
end
if isempty(fieldnames(scene))
    here = fileparts(mfilename("fullpath"));
    scene = localLoadScene(fullfile(here,"sync_ur5_joint_states.m"));
end
end

function scene = localLoadScene(syncFile)
run(syncFile);
end

function spec = localSeamSpec(scene,in)
info = scene.workpieceProxyInfo;
bottom = localFindProxy(info,"WorkpieceCollision_bottom");
x0 = localFindProxy(info,"WorkpieceCollision_wall_x0");
x1 = localFindProxy(info,"WorkpieceCollision_wall_x1");
y1 = localFindProxy(info,"WorkpieceCollision_wall_y1");
bmin = bottom.T_workpiece_proxy(1:3,4)-bottom.size_m(:)/2;
bmax = bottom.T_workpiece_proxy(1:3,4)+bottom.size_m(:)/2;
defaults = struct("name","bottom_x_wall_y1", ...
    "wallAxis",2,"wallInnerCoordinate",y1.T_workpiece_proxy(2,4)-y1.size_m(2)/2, ...
    "freeDirection",-1,"gunAxisWP",[0;1;-1]/sqrt(2), ...
    "description","U-interior: y < wall_y1 inner face, z > bottom top face");
if ~isempty(in)
    f = fieldnames(in); for k=1:numel(f), defaults.(f{k}) = in.(f{k}); end
end
switch lower(string(defaults.name))
    case "bottom_x_wall_x0"
        if ~isfield(in,"wallInnerCoordinate"), defaults.wallInnerCoordinate=x0.T_workpiece_proxy(1,4)+x0.size_m(1)/2; end
        if ~isfield(in,"freeDirection"), defaults.freeDirection=1; end
        if ~isfield(in,"gunAxisWP"), defaults.gunAxisWP=[-1;0;-1]/sqrt(2); end
        if ~isfield(in,"wallAxis"), defaults.wallAxis=1; end
        if ~isfield(in,"description"), defaults.description="U-interior: x > wall_x0 inner face, z > bottom top face"; end
    case "bottom_x_wall_x1"
        if ~isfield(in,"wallInnerCoordinate"), defaults.wallInnerCoordinate=x1.T_workpiece_proxy(1,4)-x1.size_m(1)/2; end
        if ~isfield(in,"freeDirection"), defaults.freeDirection=-1; end
        if ~isfield(in,"gunAxisWP"), defaults.gunAxisWP=[1;0;-1]/sqrt(2); end
        if ~isfield(in,"wallAxis"), defaults.wallAxis=1; end
        if ~isfield(in,"description"), defaults.description="U-interior: x < wall_x1 inner face, z > bottom top face"; end
    otherwise
        % Seam A defaults are already set.
end
defaults.gunAxisWP = double(defaults.gunAxisWP(:));
defaults.gunAxisWP = defaults.gunAxisWP/norm(defaults.gunAxisWP);
spec = defaults;
end

function seam = localInferSeam(scene,N,margin,spec)
info = scene.workpieceProxyInfo;
b = localFindProxy(info,"WorkpieceCollision_bottom");
x0 = localFindProxy(info,"WorkpieceCollision_wall_x0");
x1 = localFindProxy(info,"WorkpieceCollision_wall_x1");
y1 = localFindProxy(info,"WorkpieceCollision_wall_y1");
bmin=b.T_workpiece_proxy(1:3,4)-b.size_m(:)/2;
bmax=b.T_workpiece_proxy(1:3,4)+b.size_m(:)/2;
z=bmax(3);
switch lower(string(spec.name))
    case "bottom_x_wall_y1"
        p0=[max(bmin(1),x0.T_workpiece_proxy(1,4)+x0.size_m(1)/2)+margin; ...
            y1.T_workpiece_proxy(2,4)-y1.size_m(2)/2; z];
        p1=[min(bmax(1),x1.T_workpiece_proxy(1,4)-x1.size_m(1)/2)-margin; p0(2); z];
    case "bottom_x_wall_x0"
        p0=[x0.T_workpiece_proxy(1,4)+x0.size_m(1)/2; bmin(2)+margin; z];
        p1=[p0(1); y1.T_workpiece_proxy(2,4)-y1.size_m(2)/2-margin; z];
    case "bottom_x_wall_x1"
        p0=[x1.T_workpiece_proxy(1,4)-x1.size_m(1)/2; bmin(2)+margin; z];
        p1=[p0(1); y1.T_workpiece_proxy(2,4)-y1.size_m(2)/2-margin; z];
    otherwise, error("Unsupported seamSpec.name: %s",spec.name);
end
if norm(p1-p0) <= 0, error("端部余量过大，焊缝长度非正。 "); end
s=linspace(0,1,N).';
seam=struct(); seam.startWP=p0; seam.endWP=p1; seam.tangentWP=(p1-p0)/norm(p1-p0);
seam.positionsWP=(1-s).*p0.'+s.*p1.';
P=(scene.T_MatBase_Workpiece*[seam.positionsWP,ones(N,1)].').';
seam.positionsMat=P(:,1:3); seam.numPoints=N; seam.plateThicknessM=b.size_m(3);
seam.endMarginM=margin; seam.name=spec.name;
end

function [states,ls] = localGenerateLayer(scene,seam,i,seeds,o)
ls=localLayerStats(); states=repmat(localState(),0,1);
poseList=localPoseList(o);
for ip=1:size(poseList,1)
    if isfield(scene,"ikCandidateCache") && ~o.reuseCandidateCache %#ok<*ISFLD>
        % no-op; retained for readability of the cache policy
    end
    [Rtcp,poseInfo]=make_actual_gun_axis_pose(scene,seam.tangentWP, ...
        scene.weldGunAxisWP,poseList(ip,1),poseList(ip,2));
    Tw=eye(4); Tw(1:3,1:3)=Rtcp; Tw(1:3,4)=seam.positionsWP(i,:).';
    Ttarget=scene.T_MatBase_Workpiece*Tw;
    copt=struct("positionToleranceM",o.positionToleranceM, ...
        "orientationToleranceDeg",o.orientationToleranceDeg, ...
        "sideToleranceDeg",o.sideToleranceDeg,"sideMarginM",o.sideMarginM, ...
        "duplicateToleranceDeg",o.duplicateToleranceDeg, ...
        "checkSelfCollision",o.checkSelfCollision,"lazyCollision",false, ...
        "reuseCandidateCache",o.reuseCandidateCache);
    r=generate_seam_axis_ik_candidates(scene,Ttarget,poseList(ip,1),poseList(ip,2),seeds,copt);
    ls.ikCalls=ls.ikCalls+r.numIKCalls;
    ls.ikPoseSuccess=ls.ikPoseSuccess+r.numIKPoseOK;
    ls.nodeChecks=ls.nodeChecks+r.numEnvironmentRejected+r.numSelfRejected+r.numAccepted;
    for k=1:numel(r.states)
        st=r.states(k); st.pointIndex=i; st.poseInfo=poseInfo; st.Ttarget=Ttarget;
        states(end+1,1)=st; %#ok<AGROW>
    end
end
states=localUniqueStates(states,o.duplicateToleranceDeg);
if numel(states)>o.maxCandidatesPerLayer
    states=localKeepDiverse(states,o.maxCandidatesPerLayer,scene.qMat0);
end
ls.accepted=numel(states);
end

function seeds = localSeeds(scene,prev,o)
limits=scene.jointLimits;
nRand=max(0,round(o.numRandomSeeds));
nProp=max(0,round(o.numPropagatedSeeds));
seeds=zeros(nRand+nProp,6);
for k=1:nRand
    seeds(k,:)=limits(:,1).'+rand(1,6).*(limits(:,2)-limits(:,1)).';
end
if ~isempty(prev)
    d=zeros(numel(prev),1);
    for k=1:numel(prev), d(k)=norm(atan2(sin(prev(k).q-scene.qMat0),cos(prev(k).q-scene.qMat0))); end
    [~,ord]=sort(d,"ascend"); take=ord(1:min(nProp,numel(ord)));
    seeds(nRand+(1:numel(take)),:)=vertcat(prev(take).q);
else
    seeds(nRand+1:end,:)=repmat(scene.qMat0,nProp,1);
end
seeds=seeds(any(isfinite(seeds),2),:);
end

function poses=localPoseList(o)
if strcmpi(string(o.poseMode),"fixed")
    poses=[0 0];
elseif strcmpi(string(o.poseMode),"ar")
    [t,r]=ndgrid(o.travelCandidatesDeg,o.rollCandidatesDeg);
    poses=[t(:),r(:)];
else
    error("poseMode 必须是 fixed 或 ar。 ");
end
end

function [idx,dpCost,gs,complete]=localLayeredDP(scene,layers,seam,o,lastLayer,timer)
N=seam.numPoints; idx=zeros(1,N); dp=cell(1,N); parent=cell(1,N);
gs=struct("edgeChecks",0,"edgeCollisionFree",0,"layersReachable",zeros(1,N)); complete=false; dpCost=NaN;
if lastLayer<1 || isempty(layers{1}), return; end
% IKLink's first layer has no predecessor.  Its DP cost is initialized to
% zero; subsequent layers accumulate joint-motion and posture-change cost.
dp{1}=zeros(numel(layers{1}),1); parent{1}=zeros(numel(layers{1}),1); gs.layersReachable(1)=numel(layers{1});
for i=2:min(N,lastLayer)
    if toc(timer)>=o.timeBudgetSec, return; end
    ni=numel(layers{i}); np=numel(layers{i-1});
    cur=inf(ni,1); par=zeros(ni,1);
    for b=1:ni
        for a=1:np
            if ~isfinite(dp{i-1}(a)), continue; end
            dq=atan2(sin(layers{i}(b).q-layers{i-1}(a).q),cos(layers{i}(b).q-layers{i-1}(a).q));
            if max(abs(rad2deg(dq)))>o.maxJointStepDeg, continue; end
            gs.edgeChecks=gs.edgeChecks+1;
            if ~localEdgeFree(scene,layers{i-1}(a).q,layers{i}(b).q,o), continue; end
            gs.edgeCollisionFree=gs.edgeCollisionFree+1;
            c=dp{i-1}(a)+sum(dq.^2)+1e-4*localPoseChangeCost(layers{i-1}(a),layers{i}(b));
            if c<cur(b), cur(b)=c; par(b)=a; end
        end
    end
    dp{i}=cur; parent{i}=par; gs.layersReachable(i)=sum(isfinite(cur));
    if gs.layersReachable(i)==0, return; end
end
if lastLayer<N || isempty(dp{N}), return; end
[dpCost,k]=min(dp{N}); if ~isfinite(dpCost), return; end
idx=zeros(1,N); idx(N)=k;
for i=N:-1:2
    idx(i-1)=parent{i}(idx(i));
    if idx(i-1)<1, return; end
end
complete=true;
end

function c=localPoseChangeCost(a,b)
c=localAngleDistance(a.rollDeg,b.rollDeg)^2+(a.travelDeg-b.travelDeg)^2;
end

function ok=localEdgeFree(scene,q1,q2,o)
dq=atan2(sin(q2-q1),cos(q2-q1)); n=max(1,ceil(max(abs(rad2deg(dq)))/o.edgeResolutionDeg)); ok=true;
for k=0:n
    q=q1+(k/n)*dq;
    if ~localFullStateFree(scene,q), ok=false; return; end
end
end

function ok=localFullStateFree(scene,q)
try, [env,~,~]=checkCollision(scene.robot,q,scene.environment,"IgnoreSelfCollision","on","Exhaustive","on");
catch, env=checkCollision(scene.robot,q,scene.environment,"IgnoreSelfCollision","on"); end
if any(env(:)), ok=false; return; end
try, self=checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",scene.selfCollisionSkipPairs,"Exhaustive","on");
catch, self=checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",scene.selfCollisionSkipPairs); end
ok=~any(self(:));
end

function q=localBacktrack(layers,idx,N)
q=zeros(N,6); for i=1:N, q(i,:)=layers{i}(idx(i)).q; end
end

function v=localValidatePath(scene,qPath,layers,idx,seam,o)
v=struct("nodeCollisionFree",false,"edgeCollisionFree",false,"tcpPass",false, ...
    "maxJointJumpDeg",NaN,"maxNodeTcpErrorMm",NaN,"maxSegmentTcpErrorMm",NaN, ...
    "maxNodeTcpOrientationErrorDeg",NaN,"maxSegmentTcpOrientationErrorDeg",NaN);
if isempty(qPath), return; end
N=size(qPath,1); nodeP=zeros(N,1); nodeR=zeros(N,1); nodeOK=true;
for i=1:N
    nodeOK=nodeOK&&localFullStateFree(scene,qPath(i,:));
    T=getTransform(scene.robot,qPath(i,:),scene.tcpBodyName); tar=layers{i}(idx(i)).Ttarget;
    nodeP(i)=norm(T(1:3,4)-tar(1:3,4)); c=(trace(tar(1:3,1:3).'*T(1:3,1:3))-1)/2; nodeR(i)=acos(max(-1,min(1,c)));
end
edgeOK=true; segP=zeros(N-1,1); segR=zeros(N-1,1);
for i=1:N-1
    edgeOK=edgeOK&&localEdgeFree(scene,qPath(i,:),qPath(i+1,:),o);
    a=layers{i}(idx(i)).Ttarget; b=layers{i+1}(idx(i+1)).Ttarget; dq=atan2(sin(qPath(i+1,:)-qPath(i,:)),cos(qPath(i+1,:)-qPath(i,:)));
    n=max(2,ceil(max(abs(rad2deg(dq)))/o.edgeResolutionDeg)+1); qa=a; qb=b;
    for k=0:n-1
        s=k/(n-1); q=qPath(i,:)+s*dq; T=getTransform(scene.robot,q,scene.tcpBodyName); p=(1-s)*qa(1:3,4)+s*qb(1:3,4); segP(i)=max(segP(i),norm(T(1:3,4)-p));
        R1=rotm2quat(qa(1:3,1:3)); R2=rotm2quat(qb(1:3,1:3)); if dot(R1,R2)<0,R2=-R2;end; qq=((1-s)*R1+s*R2); qq=qq/norm(qq); R=quat2rotm(qq); cc=(trace(R.'*T(1:3,1:3))-1)/2; segR(i)=max(segR(i),acos(max(-1,min(1,cc))));
    end
end
v.nodeCollisionFree=nodeOK; v.edgeCollisionFree=edgeOK; v.maxNodeTcpErrorMm=max(nodeP)*1000; v.maxSegmentTcpErrorMm=max(segP)*1000; v.maxNodeTcpOrientationErrorDeg=max(rad2deg(nodeR)); v.maxSegmentTcpOrientationErrorDeg=max(rad2deg(segR));
v.maxJointJumpDeg=max(abs(rad2deg(atan2(sin(diff(qPath,1,1)),cos(diff(qPath,1,1))))),[],'all');
v.tcpPass=v.maxNodeTcpErrorMm<=o.positionToleranceM*1000 && v.maxSegmentTcpErrorMm<=0.5 && v.maxNodeTcpOrientationErrorDeg<=o.orientationToleranceDeg;
end

function states=localUniqueStates(states,tol)
if isempty(states), return; end; keep=true(numel(states),1);
for i=1:numel(states)
    if ~keep(i), continue; end
    for j=i+1:numel(states)
        if keep(j), dq=atan2(sin(states(i).q-states(j).q),cos(states(i).q-states(j).q)); if max(abs(rad2deg(dq)))<tol, keep(j)=false; end, end
    end
end
states=states(keep);
end

function states=localKeepDiverse(states,n,ref)
if numel(states)<=n, return; end
score=zeros(numel(states),1); for i=1:numel(states), score(i)=norm(atan2(sin(states(i).q-ref),cos(states(i).q-ref))); end; [~,ord]=sort(score,"ascend"); states=states(ord(1:n));
end

function s=localState()
s=struct("q",zeros(1,6),"travelDeg",0,"rollDeg",0,"Ttarget",eye(4),"Tactual",eye(4), ...
    "positionErrorM",NaN,"orientationErrorDeg",NaN,"gunAxisWP",zeros(1,3),"sideDot",NaN, ...
    "headPositionWP",zeros(1,3),"environmentCollision",false,"selfCollision",false, ...
    "pointIndex",0,"poseInfo",struct());
end
function s=localLayerStats(), s=struct("ikCalls",0,"ikPoseSuccess",0,"nodeChecks",0,"accepted",0); end
function s=localStats(), s=struct("ikCalls",0,"ikPoseSuccess",0,"nodeChecks",0,"acceptedCandidates",0,"edgeChecks",0,"edgeCollisionFree",0,"graphLayers",0); end
function limits=localJointLimits(robot)
limits=zeros(6,2); j=0; for i=1:numel(robot.Bodies), jt=robot.Bodies{i}.Joint; if strcmpi(jt.Type,"revolute")||strcmpi(jt.Type,"continuous"), j=j+1; limits(j,:)=double(jt.PositionLimits); end, end; if j~=6,error("UR5关节数应为6，实际为%d。",j);end
end
function skips=localStructuralSelfCollisionSkips(robot)
names=robot.BodyNames; skips=cell(0,2); for i=1:min(10,numel(names)-1), skips(end+1,:)={names{i},names{i+1}}; end; skips(end+1,:)={"WeldingGunMount","wrist_3_link"}; skips(end+1,:)={"WeldingGunMount","ee_link"}; skips(end+1,:)={"WeldingGunMount","WeldTCP"};
end
function p=localFindProxy(info,name), i=find(strcmp(string({info.name}),string(name)),1); if isempty(i),error("找不到工件代理 %s",name);end; p=info(i); end
function d=localAngleDistance(a,b), d=abs(rad2deg(atan2(sind(b-a),cosd(b-a)))); end
function out=localMerge(d,in), out=d; if isempty(in),return;end; f=fieldnames(d); for i=1:numel(f), if isfield(in,f{i})&&~isempty(in.(f{i})),out.(f{i})=in.(f{i});end,end; end
function out=localStripRuntime(s), out=s; for f=["ikSolver","jointLimits","ikCandidateCache"], if isfield(out,f),out=rmfield(out,f);end,end; end
function p=localAbsolutePath(root,p), if isstring(p),p=char(p);end; if isempty(p)||p(1)~=filesep,p=fullfile(root,p);end; end
function y=ternary(tf,a,b), if tf,y=a;else,y=b;end,end
