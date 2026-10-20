function report = analyze_local_dense_connectivity(options)
%ANALYZE_LOCAL_DENSE_CONNECTIVITY
% Dense, local-only connectivity diagnosis for the three remaining
% A-ARC-IK failure seeds.  This is diagnostic code only: it does not alter
% the main planner, its 15-degree hard joint-step limit, or any recovery
% parameters.
%
% The selected failure window is locally densified along the real seam.
% Every dense layer keeps all accepted IK states from the sampled
% travel/roll grid (subject only to an optional safety cap), then a local
% layered graph is evaluated in both directions.  The report separates:
%   - no valid node at a layer;
%   - nodes exist but no pose-compatible edge;
%   - pose-compatible edges rejected by the 15-degree joint limit;
%   - joint-compatible edges rejected by interpolated collision checks.

if nargin < 1 || isempty(options), options = struct(); end
options = localOptions(options);
if ~isfolder(options.outputDir), mkdir(options.outputDir); end

warning('off','robotics:manip:RigidBodyTree:ConfigurationOutOfBounds');
warning('off','MATLAB:rigidBodyTree:JointPositionLimits');

fprintf('\n====================================================\n');
fprintf('LOCAL DENSE CONNECTIVITY ANALYSIS\n');
fprintf('Dense interval subdivisions : %d\n',options.intervalSubdivisions);
fprintf('Travel/roll local grid     : %s deg\n',mat2str(options.offsetGrid));
fprintf('Hard joint-step limit      : %.1f deg\n',options.maxJointStepDeg);
fprintf('Only failed local windows; main planner unchanged.\n');
fprintf('====================================================\n');

cases = repmat(localEmptyCase(),0,1);
for ic = 1:numel(options.failureFiles)
    file = char(options.failureFiles(ic));
    if ~isfile(file)
        warning('Failure MAT not found: %s',file);
        continue;
    end
    fprintf('\n[%d/%d] %s\n',ic,numel(options.failureFiles),file);
    rng(options.randomSeedBase+ic,'twister');
    tCase = tic;
    S = load(file);
    if ~isfield(S,'result')
        warning('No result variable in %s',file);
        continue;
    end
    c = localScanCase(S.result,file,options);
    c.elapsedSec = toc(tCase);
    cases(end+1,1) = c; %#ok<AGROW>
    fprintf('  seed=%d, original fail=%d, dense layers=%d\n', ...
        c.seed,c.failLayer,c.denseLayerCount);
    fprintf('  node counts      : [%s]\n',mat2str(c.stateCounts.'));
    fprintf('  forward reachable: [%s]\n',mat2str(c.forwardReachableCounts.'));
    fprintf('  backward reachable: [%s]\n',mat2str(c.backwardReachableCounts.'));
    fprintf('  complete local chain=%d, diagnosis=%s\n', ...
        c.hasDenseLocalChain,c.diagnosis);
end

report = struct();
report.generatedAt = string(datetime('now'));
report.options = options;
report.cases = cases;
report.outputMat = options.outputMat;
report.outputCsv = options.outputCsv;
save(options.outputMat,'report','-v7.3');
localWriteCsv(cases,options.outputCsv);

fprintf('\nSaved MAT: %s\nSaved CSV: %s\n',options.outputMat,options.outputCsv);
for ic=1:numel(cases)
    fprintf('seed %d: %s\n',cases(ic).seed,cases(ic).diagnosis);
end
end

function options = localOptions(options)
root = '/home/ttbstt/jixiebi/UR5/matlab仿真';
defaults = struct( ...
    'failureFiles',[ ...
        string(fullfile(root,'adaptive_arc_ik_multigrid_corrected_runs','multigrid_seed20269018_grid0_uniform.mat')); ...
        string(fullfile(root,'adaptive_arc_ik_multigrid_corrected_runs','multigrid_seed20269019_grid0_uniform.mat')); ...
        string(fullfile(root,'adaptive_arc_ik_multigrid_corrected_runs','multigrid_seed20269022_grid0_uniform.mat'))], ...
    'windowHalfWidth',2, ...
    'intervalSubdivisions',4, ...
    'offsetGrid',-30:5:30, ...
    'maxSeedsPerPose',16, ...
    'randomSeedsPerPose',6, ...
    'maxStatesPerLayer',1000, ...
    'maxRollStepDeg',15, ...
    'maxTravelStepDeg',5, ...
    'maxJointStepDeg',15, ...
    'edgeResolutionDeg',3, ...
    'positionToleranceM',1e-4, ...
    'orientationToleranceDeg',0.5, ...
    'sideToleranceDeg',5, ...
    'sideMarginM',1e-3, ...
    'checkSelfCollision',true, ...
    'outputDir',fullfile(root,'local_dense_connectivity_runs'), ...
    'outputMat',fullfile(root,'local_dense_connectivity.mat'), ...
    'outputCsv',fullfile(root,'local_dense_connectivity.csv'), ...
    'randomSeedBase',99100);
names = fieldnames(defaults);
for i=1:numel(names)
    if ~isfield(options,names{i}) || isempty(options.(names{i}))
        options.(names{i}) = defaults.(names{i});
    end
end
options.failureFiles = string(options.failureFiles(:));
options.offsetGrid = double(options.offsetGrid(:).');
end

function c = localScanCase(r,file,options)
c = localEmptyCase();
c.file = string(file);
c.seed = localScalar(r,{'options','randomSeed'},NaN);
c.originalFinalPointCount = localScalar(r,{'finalPointCount'},numel(r.stateBank));
c.originalFurthestLayer = find(~cellfun(@isempty,r.stateBank),1,'last');
if isempty(c.originalFurthestLayer), c.originalFurthestLayer=0; end
c.failLayer = find(cellfun(@isempty,r.stateBank),1,'first');
if isempty(c.failLayer), c.failLayer=min(numel(r.stateBank),c.originalFurthestLayer+1); end

basePositions = localPositions(r);
lo = max(1,c.failLayer-options.windowHalfWidth);
hi = min(size(basePositions,1),c.failLayer+options.windowHalfWidth);
c.originalWindow = [lo hi];
[positions,baseIndex] = localDensify(basePositions,lo,hi,options.intervalSubdivisions);
c.densePositionsWP = positions;
c.denseBaseIndex = baseIndex;
c.denseLayerCount = size(positions,1);

scene = r.scene;
if ~isfield(scene,'ikSolver'), scene.ikSolver=inverseKinematics('RigidBodyTree',scene.robot); end
if ~isfield(scene,'jointLimits'), scene.jointLimits=localJointLimits(scene.robot); end
scene.ikCandidateCache = containers.Map('KeyType','char','ValueType','any');
if ~isfield(scene,'selfCollisionSkipPairs')
    scene.selfCollisionSkipPairs = localStructuralSelfCollisionSkips(scene.robot);
end

tangent = localTangent(r,basePositions);
gunAxis = scene.weldGunAxisWP(:); gunAxis=gunAxis/norm(gunAxis);
globalSeedQ = localSeedPool(r.stateBank,scene,options);

layers = cell(c.denseLayerCount,1);
layerDiag = repmat(localEmptyLayer(),c.denseLayerCount,1);
for il=1:c.denseLayerCount
    pWP = positions(il,:);
    center = localPoseCenter(r.stateBank,baseIndex(il));
    [states,diag] = localGenerateDenseLayer(scene,pWP,center,globalSeedQ,tangent,gunAxis,options,il);
    layers{il}=states;
    layerDiag(il)=diag;
    fprintf('  dense layer %2d/%2d (base %.2f): states=%d IK=%d accepted=%d\n', ...
        il,c.denseLayerCount,baseIndex(il),numel(states),diag.ikCalls,diag.accepted);
end

[forwardMask,backwardMask,transitions,forwardParent] = ...
    localConnectivity(scene,layers,layerDiag,baseIndex,r,options);

c.layers=layers;
c.layerDiag=layerDiag;
c.transitions=transitions;
c.forwardMasks=forwardMask;
c.backwardMasks=backwardMask;
c.forwardParent=forwardParent;
c.stateCounts=cellfun(@numel,layers);
c.forwardReachableCounts=cellfun(@sum,forwardMask);
c.backwardReachableCounts=cellfun(@sum,backwardMask);
c.totalStates=sum(c.stateCounts);
c.totalIKCalls=sum([layerDiag.ikCalls]);
c.totalAcceptedCandidates=sum([layerDiag.accepted]);
c.totalSideRejected=sum([layerDiag.sideRejected]);
c.totalJointRejected=sum([layerDiag.jointRejected]);
c.totalEnvironmentRejected=sum([layerDiag.environmentRejected]);
c.totalSelfRejected=sum([layerDiag.selfRejected]);
c.hasDenseLocalChain = any(forwardMask{end});
c.minDenseJointDeltaDeg = [transitions.minMaxJointDeltaAll].';
c.minPoseCompatibleJointDeltaDeg = [transitions.minMaxJointDeltaPoseCompatible].';
c.minCollisionFreeJointDeltaDeg = [transitions.minMaxJointDeltaCollisionFree].';
c.transitionFreeCounts = [transitions.collisionFreeCount].';
c.transitionJointRejectCounts = [transitions.jointRejectCount].';
c.transitionCollisionRejectCounts = [transitions.collisionRejectCount].';
c.transitionPoseRejectCounts = [transitions.poseRejectCount].';

if c.hasDenseLocalChain
    c.diagnosis = "DENSE_LOCAL_CHAIN_EXISTS";
elseif any(c.stateCounts==0)
    c.diagnosis = "NO_NODE_AT_ONE_OR_MORE_DENSE_LAYERS";
elseif any(c.minPoseCompatibleJointDeltaDeg > options.maxJointStepDeg)
    c.diagnosis = "POSE_COMPATIBLE_BRANCH_GAP_EXCEEDS_JOINT_LIMIT";
elseif any(c.transitionCollisionRejectCounts>0)
    c.diagnosis = "NODES_EXIST_BUT_INTERPOLATED_COLLISION_BLOCKS_EDGES";
else
    c.diagnosis = "NO_DENSE_CHAIN_UNDER_CURRENT_LOCAL_GRID";
end
end

function [states,diag] = localGenerateDenseLayer(scene,pWP,center,qSeeds,tangent,gunAxis,options,layerIndex)
states=repmat(localEmptyState(),0,1);
diag=localEmptyLayer(); diag.layerIndex=layerIndex; diag.poseCenter=center;
for dt=options.offsetGrid
    for dr=options.offsetGrid
        travel=center(1)+dt; roll=center(2)+dr;
        [Rtcp,~]=make_actual_gun_axis_pose(scene,tangent,gunAxis,travel,roll);
        T_W_TCP=eye(4); T_W_TCP(1:3,1:3)=Rtcp; T_W_TCP(1:3,4)=pWP(:);
        Ttarget=scene.T_MatBase_Workpiece*T_W_TCP;
        ikOpt=struct('positionToleranceM',options.positionToleranceM, ...
            'orientationToleranceDeg',options.orientationToleranceDeg, ...
            'sideToleranceDeg',options.sideToleranceDeg,'sideMarginM',options.sideMarginM, ...
            'checkSelfCollision',options.checkSelfCollision,'lazyCollision',false, ...
            'reuseCandidateCache',true,'verbose',false);
        rr=generate_actual_axis_ik_candidates(scene,Ttarget,travel,roll,qSeeds,ikOpt);
        diag.poses=diag.poses+1; diag.ikCalls=diag.ikCalls+rr.numIKCalls;
        diag.ikPoseOK=diag.ikPoseOK+rr.numIKPoseOK;
        diag.sideRejected=diag.sideRejected+rr.numSideRejected;
        diag.jointRejected=diag.jointRejected+rr.numJointRejected;
        diag.environmentRejected=diag.environmentRejected+rr.numEnvironmentRejected;
        diag.selfRejected=diag.selfRejected+rr.numSelfRejected;
        diag.accepted=diag.accepted+rr.numAccepted;
        for k=1:rr.numAccepted
            st=rr.states(k); rec=localStateFromCandidate(st,layerIndex,travel,roll);
            states(end+1,1)=rec; %#ok<AGROW>
        end
    end
end
% Candidate-cache and seed duplication can return the same branch repeatedly.
states=localUniqueStates(states,0.5);
if numel(states)>options.maxStatesPerLayer
    score=[[states.positionErrorM].',[states.orientationErrorDeg].'];
    [~,ord]=sortrows(score); states=states(ord(1:options.maxStatesPerLayer));
    diag.truncated=true;
end
diag.stateCount=numel(states);
end

function [forward,backward,transitions,parent] = localConnectivity(scene,layers,diag,baseIndex,r,options)
N=numel(layers);
forward=cell(N,1); backward=cell(N,1); parent=cell(N,1);
for i=1:N, forward{i}=false(1,numel(layers{i})); backward{i}=false(1,numel(layers{i})); parent{i}=zeros(1,numel(layers{i})); end

% The left boundary is the last nonempty original state immediately before
% the local window. If unavailable, every dense first-layer state is a
% valid diagnostic start.
leftBase=baseIndex(1)-1; left=localBoundaryStates(r.stateBank,leftBase);
if isempty(left)
    forward{1}(:)=true;
else
    for j=1:numel(layers{1})
        for k=1:numel(left)
            if localEdgeFree(scene,left(k),layers{1}(j),options.edgeResolutionDeg)
                forward{1}(j)=true; break;
            end
        end
    end
end

% Every state in the final dense layer is a local backward target. This
% answers whether a complete chain exists across the dense window even when
% the original global planner had no state after its failure layer.
backward{N}(:)=true;
transitions=repmat(localEmptyTransition(),N-1,1);
for i=1:N-1
    for a=1:numel(layers{i})
        for b=1:numel(layers{i+1})
            [ok,kind,dq,poseOK,collisionOK]=localEdgeClass(scene,layers{i}(a),layers{i+1}(b),options);
            tr=transitions(i);
            tr.tests=tr.tests+1;
            tr.minMaxJointDeltaAll=min(tr.minMaxJointDeltaAll,dq);
            if poseOK
                tr.poseCompatibleCount=tr.poseCompatibleCount+1;
                tr.minMaxJointDeltaPoseCompatible=min(tr.minMaxJointDeltaPoseCompatible,dq);
                if dq<=options.maxJointStepDeg
                    tr.jointCompatibleCount=tr.jointCompatibleCount+1;
                    tr.minMaxJointDeltaCollisionFree=min(tr.minMaxJointDeltaCollisionFree,dq);
                else
                    tr.jointRejectCount=tr.jointRejectCount+1;
                end
            else
                tr.poseRejectCount=tr.poseRejectCount+1;
            end
            if poseOK && dq<=options.maxJointStepDeg
                if collisionOK
                    tr.collisionFreeCount=tr.collisionFreeCount+1;
                    if forward{i}(a) && ~forward{i+1}(b)
                        forward{i+1}(b)=true; parent{i+1}(b)=a;
                    end
                    if backward{i+1}(b) && ~backward{i}(a)
                        backward{i}(a)=true;
                    end
                else
                    tr.collisionRejectCount=tr.collisionRejectCount+1;
                end
            end
            %#ok<NASGU>
            transitions(i)=tr;
        end
    end
    transitions(i).fromBase=baseIndex(i); transitions(i).toBase=baseIndex(i+1);
end
% One pass is sufficient because edges are only between adjacent layers and
% masks are propagated in order.  Backward masks need reverse propagation.
for i=N-1:-1:1
    for a=1:numel(layers{i})
        if backward{i}(a), continue; end
        for b=1:numel(layers{i+1})
            if ~backward{i+1}(b), continue; end
            [ok,~,~,~,~]=localEdgeClass(scene,layers{i}(a),layers{i+1}(b),options);
            if ok, backward{i}(a)=true; break; end
        end
    end
end
end

function [ok,kind,dq,poseOK,collisionOK]=localEdgeClass(scene,a,b,options)
dq=max(abs(rad2deg(atan2(sin(b.q-a.q),cos(b.q-a.q)))));
dTravel=abs(localAngleDiffDeg(a.travelDeg,b.travelDeg));
dRoll=abs(localAngleDiffDeg(a.rollDeg,b.rollDeg));
poseOK=dTravel<=options.maxTravelStepDeg && dRoll<=options.maxRollStepDeg;
collisionOK=false; kind="pose";
if ~poseOK, ok=false; return; end
if dq>options.maxJointStepDeg, ok=false; kind="joint"; return; end
collisionOK=localEdgeFree(scene,a,b,options.edgeResolutionDeg);
ok=collisionOK; kind="collision";
end

function free=localEdgeFree(scene,a,b,resDeg)
q1=a.q; q2=b.q; dq=atan2(sin(q2-q1),cos(q2-q1));
n=max(1,ceil(max(abs(rad2deg(dq)))/resDeg)); free=true;
for k=0:n
    q=q1+(k/n)*dq;
    if ~localFullStateFree(scene,q), free=false; return; end
end
end

function free=localFullStateFree(scene,q)
try
    [cc,~,~]=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on');
catch
    cc=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on');
end
if any(cc(:)), free=false; return; end
try
    cs=checkCollision(scene.robot,q,'IgnoreSelfCollision','off', ...
        'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
catch
    cs=checkCollision(scene.robot,q,'IgnoreSelfCollision','off', ...
        'SkippedSelfCollisions',scene.selfCollisionSkipPairs);
end
free=~any(cs(:));
end

function q=localSeedPool(bank,scene,options)
q=scene.qMat0;
for i=1:numel(bank)
    if isempty(bank{i}), continue; end
    for k=1:numel(bank{i}), q(end+1,:)=bank{i}(k).q; end %#ok<AGROW>
end
for k=1:options.randomSeedsPerPose
    try
        x=randomConfiguration(scene.robot);
        if isstruct(x)
            names=scene.robot.JointNames;
            x=cell2mat(arrayfun(@(nm)x.(char(nm)),names,'UniformOutput',false));
        end
        q(end+1,:)=double(x(:).'); %#ok<AGROW>
    catch
    end
end
q=localUniqueQ(q,0.25);
if size(q,1)>options.maxSeedsPerPose, q=q(1:options.maxSeedsPerPose,:); end
end

function [p,baseIndex]=localDensify(basePositions,lo,hi,nSub)
p=zeros(0,3); baseIndex=zeros(0,1);
for i=lo:hi-1
    for s=0:nSub-1
        u=s/nSub; p(end+1,:)= (1-u)*basePositions(i,:)+u*basePositions(i+1,:); %#ok<AGROW>
        baseIndex(end+1,1)=i+u; %#ok<AGROW>
    end
end
p(end+1,:)=basePositions(hi,:); baseIndex(end+1,1)=hi;
end

function center=localPoseCenter(bank,x)
i0=floor(x); i1=ceil(x); vals=zeros(0,2);
for i=unique([i0 i1 max(1,i0-1) min(numel(bank),i1+1)])
    if i<1 || i>numel(bank) || isempty(bank{i}), continue; end
    for k=1:numel(bank{i}), vals(end+1,:)=[bank{i}(k).travelDeg bank{i}(k).rollDeg]; end %#ok<AGROW>
end
if isempty(vals), center=[0 90]; else, center=median(vals,1); end
end

function rec=localStateFromCandidate(st,i,travel,roll)
rec=localEmptyState();
rec.q=st.q; rec.travelDeg=travel; rec.rollDeg=roll; rec.pointIndex=i;
rec.positionErrorM=st.positionErrorM; rec.orientationErrorDeg=st.orientationErrorDeg;
rec.sideDot=st.sideDot; rec.Ttarget=st.Ttarget; rec.Tactual=st.Tactual;
end

function s=localBoundaryStates(bank,i)
if i<1 || i>numel(bank) || isempty(bank{i}), s=localEmptyState(); s=s([]); return; end
s=repmat(localEmptyState(),numel(bank{i}),1);
for k=1:numel(bank{i})
    s(k)=localStateFromCandidate(bank{i}(k),i,bank{i}(k).travelDeg,bank{i}(k).rollDeg);
end
end

function states=localUniqueStates(states,tolDeg)
if isempty(states), return; end
keep=true(numel(states),1);
for i=1:numel(states)
    if ~keep(i), continue; end
    for j=i+1:numel(states)
        if keep(j)
            d=atan2(sin(states(i).q-states(j).q),cos(states(i).q-states(j).q));
            if max(abs(rad2deg(d)))<tolDeg, keep(j)=false; end
        end
    end
end
states=states(keep);
end

function q=localUniqueQ(q,tolDeg)
if isempty(q), return; end
keep=true(size(q,1),1);
for i=1:size(q,1)
    if ~keep(i), continue; end
    for j=i+1:size(q,1)
        d=atan2(sin(q(i,:)-q(j,:)),cos(q(i,:)-q(j,:)));
        if max(abs(rad2deg(d)))<tolDeg, keep(j)=false; end
    end
end
q=q(keep,:);
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
lim=zeros(6,2); j=0;
for i=1:numel(robot.Bodies)
    jt=robot.Bodies{i}.Joint;
    if strcmpi(jt.Type,'revolute') || strcmpi(jt.Type,'continuous')
        j=j+1; lim(j,:)=double(jt.PositionLimits);
    end
end
if j~=6, error('UR5 joint count is %d, expected 6.',j); end
end

function pairs=localStructuralSelfCollisionSkips(robot)
names=robot.BodyNames; pairs=cell(0,2);
for i=1:min(10,numel(names)-1)
    pairs(end+1,:)={names{i},names{i+1}}; %#ok<AGROW>
end
pairs(end+1,:)={"WeldingGunMount","wrist_3_link"};
pairs(end+1,:)={"WeldingGunMount","ee_link"};
pairs(end+1,:)={"WeldingGunMount","WeldTCP"};
end

function d=localAngleDiffDeg(a,b)
d=rad2deg(atan2(sin(deg2rad(a-b)),cos(deg2rad(a-b))));
end

function x=localEmptyCase()
x=struct('file',"",'seed',NaN,'originalFinalPointCount',0,'originalFurthestLayer',0, ...
    'failLayer',0,'originalWindow',[0 0],'denseLayerCount',0,'densePositionsWP',zeros(0,3), ...
    'denseBaseIndex',zeros(0,1),'layers',{{}},'layerDiag',repmat(localEmptyLayer(),0,1), ...
    'transitions',repmat(localEmptyTransition(),0,1),'forwardMasks',{{}},'backwardMasks',{{}}, ...
    'forwardParent',{{}},'stateCounts',zeros(0,1),'forwardReachableCounts',zeros(0,1), ...
    'backwardReachableCounts',zeros(0,1),'totalStates',0,'totalIKCalls',0,'totalAcceptedCandidates',0, ...
    'totalSideRejected',0,'totalJointRejected',0,'totalEnvironmentRejected',0,'totalSelfRejected',0, ...
    'minDenseJointDeltaDeg',zeros(0,1),'minPoseCompatibleJointDeltaDeg',zeros(0,1), ...
    'minCollisionFreeJointDeltaDeg',zeros(0,1),'transitionFreeCounts',zeros(0,1), ...
    'transitionJointRejectCounts',zeros(0,1),'transitionCollisionRejectCounts',zeros(0,1), ...
    'transitionPoseRejectCounts',zeros(0,1),'hasDenseLocalChain',false,'elapsedSec',NaN,'diagnosis',"");
end

function x=localEmptyLayer()
x=struct('layerIndex',0,'poseCenter',[0 0],'poses',0,'ikCalls',0,'ikPoseOK',0, ...
    'sideRejected',0,'jointRejected',0,'environmentRejected',0,'selfRejected',0, ...
    'accepted',0,'stateCount',0,'truncated',false);
end

function x=localEmptyState()
x=struct('q',zeros(1,6),'travelDeg',0,'rollDeg',0,'pointIndex',0, ...
    'positionErrorM',NaN,'orientationErrorDeg',NaN,'sideDot',NaN, ...
    'Ttarget',eye(4),'Tactual',eye(4));
end

function x=localEmptyTransition()
x=struct('fromBase',0,'toBase',0,'tests',0,'poseCompatibleCount',0, ...
    'jointCompatibleCount',0,'collisionFreeCount',0,'poseRejectCount',0, ...
    'jointRejectCount',0,'collisionRejectCount',0,'minMaxJointDeltaAll',Inf, ...
    'minMaxJointDeltaPoseCompatible',Inf,'minMaxJointDeltaCollisionFree',Inf);
end

function localWriteCsv(cases,file)
fid=fopen(file,'w');
fprintf(fid,['seed,failLayer,windowStart,windowEnd,denseLayers,totalStates,totalIKCalls,' ...
    'forwardFinal,backwardFirst,localChain,minDenseJointDeltaDeg,' ...
    'minPoseCompatibleJointDeltaDeg,transitionFreeEdges,transitionJointRejects,' ...
    'transitionCollisionRejects,diagnosis,elapsedSec\n']);
for i=1:numel(cases)
    c=cases(i);
    minAll=localFiniteMin(c.minDenseJointDeltaDeg);
    minPose=localFiniteMin(c.minPoseCompatibleJointDeltaDeg);
    fprintf(fid,'%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.6f,%.6f,%d,%d,%d,%s,%.6f\n', ...
        c.seed,c.failLayer,c.originalWindow(1),c.originalWindow(2),c.denseLayerCount, ...
        c.totalStates,c.totalIKCalls,c.forwardReachableCounts(end),c.backwardReachableCounts(1), ...
        c.hasDenseLocalChain,minAll,minPose,sum(c.transitionFreeCounts), ...
        sum(c.transitionJointRejectCounts),sum(c.transitionCollisionRejectCounts), ...
        char(c.diagnosis),c.elapsedSec);
end
fclose(fid);
end

function x=localFiniteMin(v)
v=v(isfinite(v)); if isempty(v), x=NaN; else, x=min(v); end
end
