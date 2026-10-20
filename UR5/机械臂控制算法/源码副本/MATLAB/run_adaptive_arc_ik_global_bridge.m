function report = run_adaptive_arc_ik_global_bridge(options)
%RUN_ADAPTIVE_ARC_IK_GLOBAL_BRIDGE
% Integrate the validated local bridge into one global continuation run.
%
% This experiment is intentionally separate from the normal benchmark.  It
% seeds the already verified prefix and the local bridge for seed 20269019,
% then lets the unchanged continuation core solve the remaining seam points.
% The resulting complete path is checked again by the core graph and by an
% independent dense node/edge validation.

if nargin<1 || isempty(options), options=struct(); end
options=localOptions(options);
seed=options.seed;
failureFile=options.failureFile;
diagnosticFile=options.diagnosticFile;
fprintf('\n==============================================\n');
fprintf('A-ARC-IK GLOBAL LOCAL-BRIDGE VALIDATION\n');
fprintf('seed = %d\n',seed);
fprintf('==============================================\n');

r=load(failureFile).result;
Diag=load(diagnosticFile).report;
Bridge=load(options.bridgeMat).report;
c=localFindCase(Diag.cases,seed);
bc=localFindCase(Bridge.cases,seed);
if isempty(c) || isempty(bc), error('No bridge diagnostic case for seed %d.',seed); end
if ~bc.bridgeFound || ~bc.completeBridge
    error('Seed %d does not have a complete local bridge candidate.',seed);
end
if ~strcmp(string(bc.diagnosis),"LOCAL_BIDIRECTIONAL_BRIDGE_FOUND")
    error('Diagnostic case is not classified as a local bridge.');
end

chain=c.localChain;
if isempty(chain), error('Diagnostic case has no recovered chain.'); end
N=size(r.finalSeamPositionsWP,1);
startLayer=chain(1).pointIndex;
endLayer=chain(end).pointIndex;
if startLayer<=1 || endLayer>=N
    error('Bridge chain does not have a global prefix/suffix: %d..%d of %d.',startLayer,endLayer,N);
end

% Rebuild full state records from the saved planner state template.  The
% diagnostic stores the numerical state fields needed for the bridge; this
% adds the continuation metadata required by the core.
template=r.stateBank{1}(1);
seedBank=cell(1,N);
for i=1:startLayer-1
    if isempty(r.stateBank{i}), error('Saved prefix layer %d is empty.',i); end
    seedBank{i}=r.stateBank{i};
end
for k=1:numel(chain)
    i=chain(k).pointIndex;
    if k==1
        parent=localBestBoundaryParent(r.stateBank{i-1},chain(k),r.scene);
    else
        parent=chain(k-1);
    end
    seedBank{i}=localStateFromRecord(template,chain(k),parent);
end

% Verify prefix-to-bridge entry before invoking the continuation core.
if ~localEntryFree(r.scene,r.stateBank{startLayer-1},seedBank{startLayer},options)
    error('Prefix -> bridge entry edge failed independent validation.');
end
for i=startLayer:endLayer-1
    if ~localEdgeFree(r.scene,seedBank{i}(1).q,seedBank{i+1}(1).q,options.edgeResolutionDeg)
        error('Bridge internal edge %d -> %d failed independent validation.',i,i+1);
    end
end

coreOptions=struct( ...
    'numPoints',N,'seamPositionsWP',r.finalSeamPositionsWP, ...
    'endMarginM',r.options.endMarginM, ...
    'beamWidth',3,'firstPointMaxStates',3,'maxPoseSeeds',12, ...
    'maxPoseNeighbors',25,'poseNeighborhoodCost','previous', ...
    'maxPoseStepRollDeg',20,'maxPoseStepTravelDeg',10, ...
    'progressivePoseNeighbors',false,'angularRedundancyFallback',true, ...
    'angularFallbackRollDeg',[10 20],'angularFallbackTravelDeg',[0 5 10], ...
    'maxRollStepDeg',r.options.maxRollStepDeg, ...
    'maxTravelStepDeg',r.options.maxTravelStepDeg, ...
    'maxJointStepDeg',r.options.maxJointStepDeg, ...
    'edgeResolutionDeg',options.edgeResolutionDeg, ...
    'positionToleranceM',r.options.positionToleranceM, ...
    'orientationToleranceDeg',r.options.orientationToleranceDeg, ...
    'sideToleranceDeg',r.options.sideToleranceDeg,'sideMarginM',r.options.sideMarginM, ...
    'checkSelfCollision',true,'reuseIKSolver',true,'reuseCandidateCache',true, ...
    'lazyCollision',true,'enableFallback',false,'randomSeed',seed, ...
    'initialStateBank',{seedBank},'initialStartLayer',endLayer, ...
    'outputFile',options.coreOutputFile);
t=tic;
core=replan_weld_actual_axis_adaptive_core(coreOptions);
elapsed=toc(t);

% Independent complete-path validation uses the same hard collision rules.
full=localValidateCoreResult(core,options);
report=struct();
report.seed=seed;
report.bridgeFrom=bc.bridge.fromPoint;
report.bridgeTo=bc.bridge.toPoint;
report.bridgeMaxJointDeltaDeg=bc.bridge.maxJointDeltaDeg;
report.prefixLayerCount=startLayer-1;
report.bridgeLayerRange=[startLayer endLayer];
report.core=core;
report.independentValidation=full;
report.elapsedSec=elapsed;
report.complete=core.complete && full.nodeCollisionFree && full.edgeCollisionFree && ...
    full.tcpNodeFree && full.tcpSegmentFree;
report.outputFile=options.outputFile;
save(options.outputFile,'report','-v7.3');
fprintf('\nGLOBAL BRIDGE RESULT: PASS=%d\n',report.complete);
fprintf('Core complete=%d, node=%d, edge=%d, TCP node=%d, TCP segment=%d\n', ...
    core.complete,full.nodeCollisionFree,full.edgeCollisionFree,full.tcpNodeFree,full.tcpSegmentFree);
fprintf('Max joint jump=%.3f deg, max TCP segment error=%.6f mm\n', ...
    full.maxJointJumpDeg,full.maxTcpSegmentErrorMm);
fprintf('Saved: %s\n',options.outputFile);
end

function options=localOptions(options)
root='/home/ttbstt/jixiebi/UR5/matlab仿真';
defaults=struct( ...
    'seed',20269019, ...
    'failureFile',fullfile(root,'adaptive_arc_ik_recovery_runs','adaptive_recovery_seed20269019.mat'), ...
    'diagnosticFile',fullfile(root,'adaptive_arc_ik_failure_diagnostics.mat'), ...
    'bridgeMat',fullfile(root,'adaptive_arc_ik_local_bridge_experiment.mat'), ...
    'edgeResolutionDeg',3, ...
    'outputFile',fullfile(root,'adaptive_arc_ik_global_bridge_seed20269019.mat'), ...
    'coreOutputFile',fullfile(root,'adaptive_arc_ik_global_bridge_core_seed20269019.mat'));
n=fieldnames(defaults);
for i=1:numel(n)
    if ~isfield(options,n{i}) || isempty(options.(n{i})), options.(n{i})=defaults.(n{i}); end
end
end

function c=localFindCase(cases,seed)
c=[];
for i=1:numel(cases)
    if iscell(cases), x=cases{i}; else, x=cases(i); end
    if x.seed==seed, c=x; return; end
end
end

function st=localStateFromRecord(template,rec,parent)
st=template;
st.q=rec.q; st.travelDeg=rec.travelDeg; st.rollDeg=rec.rollDeg;
st.Ttarget=rec.Ttarget; st.Tactual=rec.Tactual;
st.positionErrorM=rec.positionErrorM; st.orientationErrorDeg=rec.orientationErrorDeg;
st.sideDot=rec.sideDot; st.pointIndex=rec.pointIndex;
st.environmentCollision=false; st.selfCollision=false;
st.parentQ=parent.q; st.velocityQ=localWrapRad(st.q-parent.q);
if isfield(parent,'continuationCost'), pc=parent.continuationCost; else, pc=0; end
st.continuationCost=pc+sum(st.velocityQ.^2);
if isfield(rec,'poseInfo'), st.poseInfo=rec.poseInfo; end
end

function p=localBestBoundaryParent(states,rec,scene)
best=Inf; p=states(1);
for i=1:numel(states)
    dq=localWrapRad(rec.q-states(i).q);
    if max(abs(rad2deg(dq)))>15, continue; end
    if ~localEdgeFree(scene,states(i).q,rec.q,3), continue; end
    if max(abs(rad2deg(dq)))<best, best=max(abs(rad2deg(dq))); p=states(i); end
end
if isinf(best), error('No valid prefix-to-bridge parent found.'); end
end

function ok=localEntryFree(scene,states,rec,o)
ok=false;
for i=1:numel(states)
    dq=localWrapRad(rec(1).q-states(i).q);
    if max(abs(rad2deg(dq)))<=15 && localEdgeFree(scene,states(i).q,rec(1).q,o.edgeResolutionDeg)
        ok=true; return;
    end
end
end

function out=localValidateCoreResult(core,o)
out=struct('nodeCollisionFree',true,'edgeCollisionFree',true,'tcpNodeFree',true, ...
    'tcpSegmentFree',true,'maxJointJumpDeg',0,'maxTcpSegmentErrorMm',NaN, ...
    'nodeCollisionChecks',0,'edgeCollisionChecks',0);
q=core.qPath;
if isempty(q), out.nodeCollisionFree=false; out.edgeCollisionFree=false; return; end
for i=1:size(q,1)
    out.nodeCollisionChecks=out.nodeCollisionChecks+1;
    if ~localFullStateFree(core.scene,q(i,:)), out.nodeCollisionFree=false; end
    if isfield(core,'stateBank') && ~isempty(core.stateBank{i})
        d=core.stateBank{i}(1).Ttarget;
        T=getTransform(core.scene.robot,q(i,:),core.scene.tcpBodyName);
        pe=norm(T(1:3,4)-d(1:3,4));
        re=(trace(d(1:3,1:3)'*T(1:3,1:3))-1)/2;
        if pe>1e-4 || rad2deg(acos(max(-1,min(1,re))))>0.5, out.tcpNodeFree=false; end
    end
end
segErr=zeros(max(0,size(q,1)-1),1);
for i=1:size(q,1)-1
    dq=localWrapRad(q(i+1,:)-q(i,:));
    out.maxJointJumpDeg=max(out.maxJointJumpDeg,max(abs(rad2deg(dq))));
    out.edgeCollisionChecks=out.edgeCollisionChecks+1;
    if max(abs(rad2deg(dq)))>15 || ~localEdgeFree(core.scene,q(i,:),q(i+1,:),o.edgeResolutionDeg)
        out.edgeCollisionFree=false;
    end
    T1=getTransform(core.scene.robot,q(i,:),core.scene.tcpBodyName);
    T2=getTransform(core.scene.robot,q(i+1,:),core.scene.tcpBodyName);
    n=max(1,ceil(max(abs(rad2deg(dq)))/o.edgeResolutionDeg));
    for k=0:n
        T=getTransform(core.scene.robot,q(i,:)+(k/n)*dq,core.scene.tcpBodyName);
        s=k/n; p=(1-s)*T1(1:3,4)+s*T2(1:3,4);
        segErr(i)=max(segErr(i),norm(T(1:3,4)-p));
    end
end
out.maxTcpSegmentErrorMm=1000*max([0;segErr]);
end

function free=localFullStateFree(scene,q)
try, [cc,~,~]=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on');
catch, cc=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on'); end
if any(cc(:)), free=false; return; end
try, cs=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
catch, cs=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs); end
free=~any(cs(:));
end

function free=localEdgeFree(scene,q1,q2,resDeg)
dq=localWrapRad(q2-q1); n=max(1,ceil(max(abs(rad2deg(dq)))/resDeg)); free=true;
for k=0:n
    if ~localFullStateFree(scene,q1+(k/n)*dq), free=false; return; end
end
end

function x=localWrapRad(x), x=atan2(sin(x),cos(x)); end
