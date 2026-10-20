function report = diagnose_adaptive_arc_ik_failures(options)
%DIAGNOSE_ADAPTIVE_ARC_IK_FAILURES Local feasibility/continuity diagnosis.
%
% Offline diagnostic only: it does not modify the A-ARC-IK planner.  It
% scans the first empty layer of the three current failed recovery seeds in
% a five-layer neighbourhood, using a fine local travel/roll grid.
%
% Example:
%   addpath('/home/ttbstt/jixiebi/UR5/matlab仿真');
%   report = diagnose_adaptive_arc_ik_failures();

if nargin < 1 || isempty(options), options = struct(); end
options = localOptions(options);
outDir = fileparts(options.outputMat);
if ~isfolder(outDir), mkdir(outDir); end

fprintf('\n==============================================\n');
fprintf('A-ARC-IK LOCAL FAILURE DIAGNOSTIC\n');
fprintf('Fine grid: travel/roll offsets %s deg\n',mat2str(options.offsetGrid));
fprintf('Only failed seeds are scanned; main planner is unchanged.\n');
fprintf('==============================================\n');

allCases = repmat(localEmptyCase(),0,1);
for k = 1:numel(options.failureFiles)
    file = char(options.failureFiles(k));
    if ~isfile(file), warning('Failure MAT not found: %s',file); continue; end
    fprintf('\n[%d/%d] Loading %s\n',k,numel(options.failureFiles),file);
    rng(options.randomSeedBase+k,'twister');
    ticCase = tic;
    S = load(file);
    if ~isfield(S,'result'), warning('No result variable in %s',file); continue; end
    c = localScanCase(S.result,file,options);
    c.elapsedSec = toc(ticCase);
    allCases(end+1,1) = c; %#ok<AGROW>
    fprintf('  seed=%d failLayer=%d window=%s\n',c.seed,c.failLayer,mat2str(c.window));
    fprintf('  local chain=%d, reachable [%s], states [%s], edges=%d free=%d\n', ...
        c.hasLocalChain,mat2str(c.reachableCounts),mat2str(c.stateCounts),c.edgeTests,c.edgeFree);
    fprintf('  diagnosis: %s\n',c.diagnosis);
end

report = struct();
report.generatedAt = string(datetime('now'));
report.options = options;
report.cases = allCases;
report.outputMat = options.outputMat;
report.outputCsv = options.outputCsv;
save(options.outputMat,'report','-v7.3');
localWriteSummaryCsv(allCases,options.outputCsv);
fprintf('\nSaved diagnostic MAT: %s\n',options.outputMat);
fprintf('Saved diagnostic CSV: %s\n',options.outputCsv);
for k = 1:numel(allCases), fprintf('seed %d: %s\n',allCases(k).seed,allCases(k).diagnosis); end
end

function options = localOptions(options)
root = '/home/ttbstt/jixiebi/UR5/matlab仿真';
defaults = struct( ...
    'failureFiles',[string(fullfile(root,'adaptive_arc_ik_recovery_runs','adaptive_recovery_seed20269018.mat')); ...
                     string(fullfile(root,'adaptive_arc_ik_recovery_runs','adaptive_recovery_seed20269019.mat')); ...
                     string(fullfile(root,'adaptive_arc_ik_recovery_runs','adaptive_recovery_seed20269022.mat'))], ...
    'offsetGrid',-30:5:30,'maxSeedsPerLayer',6,'randomSeedsPerLayer',1, ...
    'windowHalfWidth',2, ...
    'maxStatesPerLayer',160,'maxRollStepDeg',45,'maxTravelStepDeg',15, ...
    'maxJointStepDeg',15,'edgeResolutionDeg',3,'positionToleranceM',1e-4, ...
    'orientationToleranceDeg',0.5,'sideToleranceDeg',5,'sideMarginM',1e-3, ...
    'checkSelfCollision',true, ...
    'outputMat',fullfile(root,'adaptive_arc_ik_failure_diagnostics.mat'), ...
    'outputCsv',fullfile(root,'adaptive_arc_ik_failure_diagnostics.csv'), ...
    'randomSeedBase',88000);
names = fieldnames(defaults);
for i=1:numel(names)
    if ~isfield(options,names{i}) || isempty(options.(names{i})), options.(names{i})=defaults.(names{i}); end
end
if ischar(options.failureFiles)
    options.failureFiles = string(options.failureFiles);
else
    options.failureFiles = string(options.failureFiles(:));
end
options.offsetGrid = double(options.offsetGrid(:).');
end

function c = localScanCase(r,file,options)
c = localEmptyCase();
c.file = string(file);
c.seed = localScalar(r,{'options','randomSeed'},NaN);
c.complete = logical(r.complete);
c.originalFinalPointCount = localScalar(r,{'finalPointCount'},numel(r.stateBank));
c.originalFurthestLayer = find(~cellfun(@isempty,r.stateBank),1,'last');
if isempty(c.originalFurthestLayer), c.originalFurthestLayer=0; end
c.failLayer = find(cellfun(@isempty,r.stateBank),1,'first');
if isempty(c.failLayer), c.failLayer=min(numel(r.stateBank),c.originalFurthestLayer+1); end
c.window = max(1,c.failLayer-options.windowHalfWidth):min(numel(r.stateBank),c.failLayer+options.windowHalfWidth);

scene = r.scene;
if ~isfield(scene,'ikSolver'), scene.ikSolver=inverseKinematics('RigidBodyTree',scene.robot); end
if ~isfield(scene,'jointLimits'), scene.jointLimits=localJointLimits(scene.robot); end
positionsWP = localPositions(r);
if size(positionsWP,1)<max(c.window), error('Seed %d: seam positions do not cover diagnostic window.',c.seed); end
tangentWP = localTangent(r,positionsWP);
gunAxisWP = scene.weldGunAxisWP(:); gunAxisWP=gunAxisWP/norm(gunAxisWP);

Nw = numel(c.window);
layers=cell(Nw,1);
layerDiag=repmat(localEmptyLayer(),Nw,1);
for il=1:Nw
    i=c.window(il);
    centers=localPoseCenters(r.stateBank,i,c.window);
    qSeeds=localSeedBank(r.stateBank,i,c.window,scene,options);
    if isempty(qSeeds), qSeeds=scene.qMat0; end
    if size(qSeeds,1)>options.maxSeedsPerLayer, qSeeds=qSeeds(1:options.maxSeedsPerLayer,:); end
    if options.randomSeedsPerLayer>0
        qRand=localRandomConfigurations(scene.robot,options.randomSeedsPerLayer);
        qSeeds=localUniqueQ([qSeeds;qRand],0.25);
        qSeeds=qSeeds(1:min(end,options.maxSeedsPerLayer),:);
    end

    stateList=repmat(localEmptyStateRecord(),0,1);
    poseSummary=repmat(localEmptyPose(),0,1);
    callCount=0; accepted=0; sideReject=0; envReject=0; selfReject=0; poseOK=0; jointReject=0;
    for dt=options.offsetGrid
        for dr=options.offsetGrid
            travel=centers(1)+dt; roll=centers(2)+dr;
            [RtcpWP,~]=make_actual_gun_axis_pose(scene,tangentWP,gunAxisWP,travel,roll);
            T_W_TCP=eye(4); T_W_TCP(1:3,1:3)=RtcpWP; T_W_TCP(1:3,4)=positionsWP(i,:).';
            Ttarget=scene.T_MatBase_Workpiece*T_W_TCP;
            optIK=struct('positionToleranceM',options.positionToleranceM, ...
                'orientationToleranceDeg',options.orientationToleranceDeg, ...
                'sideToleranceDeg',options.sideToleranceDeg,'sideMarginM',options.sideMarginM, ...
                'checkSelfCollision',options.checkSelfCollision,'lazyCollision',false, ...
                'reuseCandidateCache',false,'verbose',false);
            rr=generate_actual_axis_ik_candidates(scene,Ttarget,travel,roll,qSeeds,optIK);
            callCount=callCount+rr.numIKCalls; poseOK=poseOK+rr.numIKPoseOK;
            sideReject=sideReject+rr.numSideRejected; envReject=envReject+rr.numEnvironmentRejected;
            selfReject=selfReject+rr.numSelfRejected; jointReject=jointReject+rr.numJointRejected;
            for jj=1:rr.numAccepted
                st=rr.states(jj);
                stateList(end+1,1)=localStateRecord(st,i,travel,roll,dt,dr); %#ok<AGROW>
                accepted=accepted+1;
            end
            ps=localEmptyPose(); ps.travelDeg=travel; ps.rollDeg=roll;
            ps.offsetTravelDeg=dt; ps.offsetRollDeg=dr; ps.numIKCalls=rr.numIKCalls;
            ps.numIKPoseOK=rr.numIKPoseOK; ps.numSideRejected=rr.numSideRejected;
            ps.numJointRejected=rr.numJointRejected; ps.numEnvironmentRejected=rr.numEnvironmentRejected;
            ps.numSelfRejected=rr.numSelfRejected; ps.numAccepted=rr.numAccepted;
            poseSummary(end+1,1)=ps; %#ok<AGROW>
        end
    end
    if numel(stateList)>options.maxStatesPerLayer
        score=[[stateList.positionErrorM].',[stateList.orientationErrorDeg].'];
        [~,ord]=sortrows(score); stateList=stateList(ord(1:options.maxStatesPerLayer));
    end
    layers{il}=stateList;
    layerDiag(il).pointIndex=i; layerDiag(il).poseCenters=centers;
    layerDiag(il).numPoses=numel(poseSummary); layerDiag(il).stateCount=numel(stateList);
    layerDiag(il).ikCalls=callCount; layerDiag(il).ikPoseOK=poseOK;
    layerDiag(il).sideRejected=sideReject; layerDiag(il).jointRejected=jointReject;
    layerDiag(il).environmentRejected=envReject; layerDiag(il).selfRejected=selfReject;
    layerDiag(il).acceptedCandidateCount=accepted; layerDiag(il).poseSummary=poseSummary;
    fprintf('  point %d: poses=%d states=%d IK=%d accepted=%d\n',i,numel(poseSummary),numel(stateList),callCount,accepted);
end

reachable=cell(Nw,1);
parentIndex=cell(Nw,1);
edgeDiag=repmat(localEmptyEdge(),0,1);
leftBoundary=localBoundaryStates(r.stateBank,c.window(1)-1);
if isempty(leftBoundary)
    reachable{1}=true(1,numel(layers{1}));
    parentIndex{1}=zeros(1,numel(layers{1}));
else
    reachable{1}=false(1,numel(layers{1}));
    parentIndex{1}=zeros(1,numel(layers{1}));
    for b=1:numel(leftBoundary)
        for j=1:numel(layers{1})
            [ok,info]=localEdgeCheck(scene,leftBoundary(b),layers{1}(j),options);
            edgeDiag(end+1,1)=info; %#ok<AGROW>
            if ok, reachable{1}(j)=true; end
        end
    end
end
for il=2:Nw
    reachable{il}=false(1,numel(layers{il}));
    parentIndex{il}=zeros(1,numel(layers{il}));
    for a=1:numel(layers{il-1})
        if ~reachable{il-1}(a), continue; end
        for b=1:numel(layers{il})
            [ok,info]=localEdgeCheck(scene,layers{il-1}(a),layers{il}(b),options);
            edgeDiag(end+1,1)=info; %#ok<AGROW>
            if ok && ~reachable{il}(b)
                reachable{il}(b)=true; parentIndex{il}(b)=a;
            end
        end
    end
end

c.layer=layerDiag; c.layers=layers; c.reachableMasks=reachable;
c.parentIndex=parentIndex; c.reachableCounts=cellfun(@sum,reachable).';
c.stateCounts=cellfun(@numel,layers).'; c.edgeTests=numel(edgeDiag);
if isempty(edgeDiag), c.edgeFree=0; c.edgeRejectedPose=0; c.edgeRejectedJoint=0; c.edgeRejectedCollision=0; c.edgeMaxJointDeltaDeg=0;
else
    c.edgeFree=sum([edgeDiag.free]); c.edgeRejectedPose=sum([edgeDiag.rejectedPose]);
    c.edgeRejectedJoint=sum([edgeDiag.rejectedJoint]); c.edgeRejectedCollision=sum([edgeDiag.rejectedCollision]);
    c.edgeMaxJointDeltaDeg=max([edgeDiag.maxJointDeltaDeg]);
end
c.edgeDiag=edgeDiag; c.hasLocalChain=~isempty(reachable) && any(reachable{end});
c.leftBoundaryCount=numel(leftBoundary); c.rightReachableCount=sum(reachable{end});
c.totalStates=sum(c.stateCounts); c.totalAcceptedCandidates=sum([layerDiag.acceptedCandidateCount]);
c.totalIKCalls=sum([layerDiag.ikCalls]); c.totalPoseOK=sum([layerDiag.ikPoseOK]);
c.totalSideRejected=sum([layerDiag.sideRejected]); c.totalJointRejected=sum([layerDiag.jointRejected]);
c.totalEnvironmentRejected=sum([layerDiag.environmentRejected]); c.totalSelfRejected=sum([layerDiag.selfRejected]);
if c.hasLocalChain, c.diagnosis="LOCAL_CHAIN_EXISTS_SEARCH_MISSED";
else, c.diagnosis="NO_LOCAL_CHAIN_UNDER_CURRENT_GRID_AND_CONSTRAINTS"; end
c.localChain=localRecoverChain(layers,reachable,parentIndex);
c.transitionSummary=localTransitionSummary(edgeDiag,c.window);
end

function chain=localRecoverChain(layers,reachable,parentIndex)
chain=repmat(localEmptyStateRecord(),0,1);
if isempty(layers) || isempty(reachable{end}) || ~any(reachable{end}), return; end
idx=zeros(1,numel(layers)); idx(end)=find(reachable{end},1,'first');
for i=numel(layers):-1:2
    idx(i-1)=parentIndex{i}(idx(i));
end
if idx(1)==0, chain=chain([]); return; end
chain=repmat(localEmptyStateRecord(),numel(layers),1);
for i=1:numel(layers), chain(i)=layers{i}(idx(i)); end
end

function out=localTransitionSummary(edges,window)
out=repmat(struct('fromPoint',0,'toPoint',0,'tests',0,'free',0, ...
    'minMaxJointDeltaDeg',Inf,'minDTravelDeg',Inf,'minDRollDeg',Inf),0,1);
for i=1:numel(window)-1
    a=window(i); b=window(i+1); e=edges([edges.fromPoint]==a & [edges.toPoint]==b);
    x=struct('fromPoint',a,'toPoint',b,'tests',numel(e),'free',0, ...
        'minMaxJointDeltaDeg',Inf,'minDTravelDeg',Inf,'minDRollDeg',Inf);
    if ~isempty(e)
        x.free=sum([e.free]); x.minMaxJointDeltaDeg=min([e.maxJointDeltaDeg]);
        x.minDTravelDeg=min([e.dTravelDeg]); x.minDRollDeg=min([e.dRollDeg]);
    end
    out(end+1,1)=x; %#ok<AGROW>
end
end

function [ok,info]=localEdgeCheck(scene,a,b,options)
info=localEmptyEdge(); info.fromPoint=a.pointIndex; info.toPoint=b.pointIndex;
info.fromTravelDeg=a.travelDeg; info.toTravelDeg=b.travelDeg;
info.fromRollDeg=a.rollDeg; info.toRollDeg=b.rollDeg;
info.dTravelDeg=abs(localAngleDiffDeg(a.travelDeg,b.travelDeg));
info.dRollDeg=abs(localAngleDiffDeg(a.rollDeg,b.rollDeg));
dq=atan2(sin(b.q-a.q),cos(b.q-a.q)); info.maxJointDeltaDeg=max(abs(rad2deg(dq)));
if info.dTravelDeg>options.maxTravelStepDeg || info.dRollDeg>options.maxRollStepDeg
    info.rejectedPose=true; ok=false; return;
end
if info.maxJointDeltaDeg>options.maxJointStepDeg
    info.rejectedJoint=true; ok=false; return;
end
info.tests=max(1,ceil(info.maxJointDeltaDeg/options.edgeResolutionDeg))+1;
info.free=localEdgeFree(scene,a.q,b.q,options.edgeResolutionDeg);
info.rejectedCollision=~info.free; ok=info.free;
end

function free=localEdgeFree(scene,q1,q2,resDeg)
dq=atan2(sin(q2-q1),cos(q2-q1));
n=max(1,ceil(max(abs(rad2deg(dq)))/resDeg)); free=true;
for k=0:n
    q=q1+(k/n)*dq;
    if ~localFullStateFree(scene,q), free=false; return; end
end
end

function free=localFullStateFree(scene,q)
free=false;
try, [cc,~,~]=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on');
catch, cc=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on'); end
if any(cc(:)), return; end
try, cs=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
catch, cs=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs); end
free=~any(cs(:));
end

function q=localSeedBank(bank,i,window,scene,options)
q=zeros(0,6); idx=unique([i-2:i+2,window(1)-1,window(end)+1]);
idx=idx(idx>=1 & idx<=numel(bank));
for k=idx
    if isempty(bank{k}), continue; end
    s=bank{k}; for j=1:numel(s), q(end+1,:)=s(j).q; end %#ok<AGROW>
end
q(end+1,:)=scene.qMat0; q=localUniqueQ(q,0.25);
if size(q,1)>options.maxSeedsPerLayer, q=q(1:options.maxSeedsPerLayer,:); end
end

function q=localRandomConfigurations(robot,n)
q=zeros(0,6);
for i=1:n
    try
        x=randomConfiguration(robot);
        if isstruct(x)
            names=robot.JointNames;
            x=cell2mat(arrayfun(@(nm)x.(char(nm)),names,'UniformOutput',false));
        end
        q(end+1,:)=double(x(:).'); %#ok<AGROW>
    catch
    end
end
end

function q=localUniqueQ(q,tolDeg)
if isempty(q), return; end
keep=true(size(q,1),1);
for i=1:size(q,1)
    if ~keep(i), continue; end
    for j=i+1:size(q,1)
        if keep(j)
            d=atan2(sin(q(i,:)-q(j,:)),cos(q(i,:)-q(j,:)));
            if max(abs(rad2deg(d)))<tolDeg, keep(j)=false; end
        end
    end
end
q=q(keep,:);
end

function centers=localPoseCenters(bank,i,window)
vals=zeros(0,2); idx=unique([i-2:i+2,window(1)-1,window(end)+1]);
idx=idx(idx>=1 & idx<=numel(bank));
for k=idx
    if isempty(bank{k}), continue; end
    s=bank{k}; vals=[vals; [[s.travelDeg].',[s.rollDeg].']]; %#ok<AGROW>
end
if isempty(vals), centers=[0 0]; else, centers=median(vals,1); end
end

function s=localBoundaryStates(bank,i)
if i<1 || i>numel(bank) || isempty(bank{i}), s=localEmptyStateRecord(); s=s([]);
else
    raw=bank{i}; s=repmat(localEmptyStateRecord(),numel(raw),1);
    for k=1:numel(raw), s(k)=localStateRecord(raw(k),i,raw(k).travelDeg,raw(k).rollDeg,0,0); end
end
end

function rec=localStateRecord(st,i,travel,roll,dt,dr)
rec=localEmptyStateRecord(); rec.q=st.q; rec.travelDeg=travel; rec.rollDeg=roll; rec.pointIndex=i;
rec.positionErrorM=st.positionErrorM; rec.orientationErrorDeg=st.orientationErrorDeg;
rec.sideDot=st.sideDot; rec.offsetTravelDeg=dt; rec.offsetRollDeg=dr;
rec.Ttarget=st.Ttarget; rec.Tactual=st.Tactual;
end

function p=localPositions(r)
if isfield(r,'finalSeamPositionsWP') && ~isempty(r.finalSeamPositionsWP), p=r.finalSeamPositionsWP;
elseif isfield(r,'seam') && isfield(r.seam,'positionsWP'), p=r.seam.positionsWP;
else, error('No seam positions in result.'); end
end

function t=localTangent(r,p)
if isfield(r,'seam') && isfield(r.seam,'tangentWP'), t=r.seam.tangentWP(:);
else, t=(p(end,:)-p(1,:)).'; end
t=t/norm(t);
end

function x=localScalar(s,path,default)
x=default;
try
    y=s; for i=1:numel(path), y=y.(path{i}); end
    if ~isempty(y), x=double(y); end
catch
end
end

function lim=localJointLimits(robot)
lim=zeros(6,2);
j=0;
for i=1:numel(robot.Bodies)
    jt=robot.Bodies{i}.Joint;
    if strcmpi(jt.Type,'revolute') || strcmpi(jt.Type,'continuous')
        j=j+1; lim(j,:)=double(jt.PositionLimits);
    end
end
if j~=6, error('UR5 joint count is %d, expected 6.',j); end
end

function d=localAngleDiffDeg(a,b)
d=rad2deg(atan2(sin(deg2rad(a-b)),cos(deg2rad(a-b))));
end

function c=localEmptyCase()
c=struct('file',"",'seed',NaN,'complete',false,'originalFinalPointCount',0,'originalFurthestLayer',0,'failLayer',0,'window',zeros(1,0), ...
    'layer',repmat(localEmptyLayer(),0,1),'layers',{{}},'reachableMasks',{{}},'parentIndex',{{}},'localChain',repmat(localEmptyStateRecord(),0,1),'transitionSummary',repmat(struct(),0,1),'stateCounts',zeros(1,0),'reachableCounts',zeros(1,0), ...
    'edgeTests',0,'edgeFree',0,'edgeRejectedPose',0,'edgeRejectedJoint',0,'edgeRejectedCollision',0,'edgeMaxJointDeltaDeg',0, ...
    'edgeDiag',repmat(localEmptyEdge(),0,1),'hasLocalChain',false,'leftBoundaryCount',0,'rightReachableCount',0, ...
    'totalStates',0,'totalAcceptedCandidates',0,'totalIKCalls',0,'totalPoseOK',0,'totalSideRejected',0,'totalJointRejected',0, ...
    'totalEnvironmentRejected',0,'totalSelfRejected',0,'elapsedSec',NaN,'diagnosis',"");
end

function x=localEmptyLayer()
x=struct('pointIndex',0,'poseCenters',[0 0],'numPoses',0,'stateCount',0,'ikCalls',0,'ikPoseOK',0,'sideRejected',0,'jointRejected',0,'environmentRejected',0,'selfRejected',0,'acceptedCandidateCount',0,'poseSummary',repmat(localEmptyPose(),0,1));
end

function x=localEmptyPose()
x=struct('travelDeg',0,'rollDeg',0,'offsetTravelDeg',0,'offsetRollDeg',0,'numIKCalls',0,'numIKPoseOK',0,'numSideRejected',0,'numJointRejected',0,'numEnvironmentRejected',0,'numSelfRejected',0,'numAccepted',0);
end

function x=localEmptyEdge()
x=struct('fromPoint',0,'toPoint',0,'fromTravelDeg',0,'toTravelDeg',0,'fromRollDeg',0,'toRollDeg',0,'dTravelDeg',0,'dRollDeg',0,'maxJointDeltaDeg',0,'tests',0,'free',false,'rejectedPose',false,'rejectedJoint',false,'rejectedCollision',false);
end

function x=localEmptyStateRecord()
x=struct('q',zeros(1,6),'travelDeg',0,'rollDeg',0,'pointIndex',0,'offsetTravelDeg',0,'offsetRollDeg',0,'positionErrorM',NaN,'orientationErrorDeg',NaN,'sideDot',NaN,'Ttarget',eye(4),'Tactual',eye(4));
end

function localWriteSummaryCsv(cases,file)
fid=fopen(file,'w');
fprintf(fid,'seed,complete,originalFinalPointCount,originalFurthestLayer,failLayer,windowStart,windowEnd,stateCountSum,reachableFinal,edgeTests,edgeFree,edgePoseReject,edgeJointReject,edgeCollisionReject,totalIKCalls,totalAcceptedCandidates,elapsedSec,diagnosis\n');
for i=1:numel(cases)
    c=cases(i);
    fprintf(fid,'%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.6f,%s\n', ...
        c.seed,c.complete,c.originalFinalPointCount,c.originalFurthestLayer,c.failLayer,c.window(1),c.window(end),c.totalStates,c.rightReachableCount,c.edgeTests,c.edgeFree,c.edgeRejectedPose,c.edgeRejectedJoint,c.edgeRejectedCollision,c.totalIKCalls,c.totalAcceptedCandidates,c.elapsedSec,char(c.diagnosis));
end
fclose(fid);
end
