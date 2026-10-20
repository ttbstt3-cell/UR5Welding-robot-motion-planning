function report = run_local_dense_graph_recovery(options)
%RUN_LOCAL_DENSE_GRAPH_RECOVERY Recover global paths using dense local chains.
% This is an independent experiment; the normal A-ARC-IK planner is unchanged.

if nargin<1 || isempty(options), options=struct(); end
options=localOptions(options);
if ~isfile(options.denseReportFile), error('Dense report not found: %s',options.denseReportFile); end

D=load(options.denseReportFile); denseReport=D.report;
recoverSeeds=[20269018 20269019];
rows=repmat(localEmptyRow(),0,1); results=cell(0,1);

fprintf('\n====================================================\n');
fprintf('LOCAL DENSE GRAPH GLOBAL RECOVERY\n');
fprintf('====================================================\n');

for seed=recoverSeeds
    idx=find([denseReport.cases.seed]==seed,1);
    if isempty(idx), warning('No dense case for seed %d.',seed); continue; end
    c=denseReport.cases(idx);
    if ~c.hasDenseLocalChain, warning('Seed %d has no dense chain.',seed); continue; end
    S=load(char(c.file)); original=S.result;
    denseBank=localForwardDenseBank(c);
    basePositions=localPositions(original);
    lo=c.originalWindow(1); hi=c.originalWindow(2);
    combinedPositions=[basePositions(1:lo-1,:); c.densePositionsWP; basePositions(hi+1:end,:)];
    prefix=original.stateBank(1:lo-1);
    prefix=prefix(:);
    suffixCount=size(combinedPositions,1)-(lo-1)-numel(denseBank);
    initialStateBank=[prefix; cell(numel(denseBank)+suffixCount,1)];
    startLayer=numel(prefix)+numel(denseBank);
    for k=1:numel(denseBank)
        layerStates=denseBank{k};
        augStates=repmat(localEmptyState(),numel(layerStates),1);
        for j=1:numel(layerStates), augStates(j)=localAugmentState(layerStates(j)); end
        initialStateBank{numel(prefix)+k}=augStates;
    end

    coreOptions=localCoreOptions(original.options);
    coreOptions.seamPositionsWP=combinedPositions;
    coreOptions.numPoints=size(combinedPositions,1);
    coreOptions.initialStateBank=initialStateBank;
    coreOptions.initialStartLayer=startLayer;
    coreOptions.enableFallback=false; coreOptions.animate=false; coreOptions.publish=false;
    coreOptions.randomSeed=seed;
    outFile=fullfile(options.outputDir,sprintf('local_dense_recovery_seed%d.mat',seed));
    coreOptions.outputFile=outFile;

    fprintf('\nSeed %d: prefix=%d dense=%d suffix=%d total=%d\n', ...
        seed,numel(prefix),numel(denseBank),suffixCount,size(combinedPositions,1));
    t=tic;
    try
        result=replan_weld_actual_axis_adaptive_core(coreOptions);
        wall=toc(t); row=localResultRow(result,seed,wall,outFile,startLayer);
        results{end+1,1}=result; %#ok<AGROW>
        fprintf('  complete=%d node=%d edge=%d wall=%.3fs firstFail=%d\n', ...
            row.complete,row.nodeCollisionFree,row.edgeCollisionFree,row.wallTimeSec,row.firstFailureLayer);
    catch ME
        wall=toc(t); row=localEmptyRow(); row.seed=seed; row.wallTimeSec=wall;
        row.errorMessage=string(ME.message); row.outputFile=string(outFile);
        results{end+1,1}=struct('error',ME); %#ok<AGROW>
        warning('Seed %d recovery failed: %s',seed,ME.message);
    end
    rows(end+1,1)=row; %#ok<AGROW>
end

collision22=localDiagnoseSeed22(denseReport);
report=struct('generatedAt',string(datetime('now')),'rows',rows,'results',{results}, ...
    'seed20269022Collision',collision22,'denseReportFile',options.denseReportFile, ...
    'outputMat',options.outputMat,'outputCsv',options.outputCsv);
save(options.outputMat,'report','-v7.3'); localWriteCsv(rows,options.outputCsv);
fprintf('\nSaved MAT: %s\nSaved CSV: %s\n',options.outputMat,options.outputCsv);
fprintf('Seed 20269022 first blocked transition: %s\n',collision22.transitionLabel);
for k=1:numel(collision22.pairs)
    fprintf('  %s : %d samples\n',collision22.pairs(k).label,collision22.pairs(k).count);
end
end

function options=localOptions(options)
root='/home/ttbstt/jixiebi/UR5/matlab仿真';
defaults=struct('denseReportFile',fullfile(root,'local_dense_connectivity.mat'), ...
    'outputDir',fullfile(root,'local_dense_recovery_runs'), ...
    'outputMat',fullfile(root,'local_dense_graph_recovery.mat'), ...
    'outputCsv',fullfile(root,'local_dense_graph_recovery.csv'));
names=fieldnames(defaults);
for i=1:numel(names)
    if ~isfield(options,names{i}) || isempty(options.(names{i})), options.(names{i})=defaults.(names{i}); end
end
if ~isfolder(options.outputDir), mkdir(options.outputDir); end
end

function chain=localRecoverDenseChain(c)
N=numel(c.layers); chain=repmat(localEmptyState(),0,1);
if ~c.hasDenseLocalChain, return; end
idx=zeros(1,N); idx(N)=find(c.forwardMasks{N},1,'first');
for i=N:-1:2, idx(i-1)=c.forwardParent{i}(idx(i)); end
if any(idx==0), chain=chain([]); return; end
chain=repmat(localEmptyState(),N,1);
for i=1:N, chain(i)=localConvertState(c.layers{i}(idx(i))); end
end

function bank=localForwardDenseBank(c)
bank=cell(numel(c.layers),1);
for i=1:numel(c.layers)
    idx=find(c.forwardMasks{i});
    bank{i}=c.layers{i}(idx);
end
end

function st=localConvertState(x)
st=localEmptyState(); st.q=x.q; st.travelDeg=x.travelDeg; st.rollDeg=x.rollDeg;
st.Ttarget=x.Ttarget; st.Tactual=x.Tactual; st.positionErrorM=x.positionErrorM;
st.orientationErrorDeg=x.orientationErrorDeg; if isfield(x,'sideDot'), st.sideDot=x.sideDot; end
st.pointIndex=x.pointIndex; st.velocityQ=zeros(1,6); st.parentQ=zeros(1,6); st.continuationCost=0;
end

function st=localAugmentState(x)
st=localEmptyState(); names=fieldnames(st);
for i=1:numel(names), if isfield(x,names{i}), st.(names{i})=x.(names{i}); end, end
end

function opts=localCoreOptions(src)
names={'numPoints','endMarginM','coarseTravelDeg','coarseRollDeg','beamWidth', ...
    'firstPointMaxStates','maxPoseSeeds','seedPerturbDeg','maxPoseNeighbors', ...
    'maxPoseStepRollDeg','maxPoseStepTravelDeg','numRandomSeedsFirstPoint', ...
    'maxRollStepDeg','maxTravelStepDeg','maxJointStepDeg','edgeResolutionDeg', ...
    'maxBacktrackLayers','positionToleranceM','orientationToleranceDeg', ...
    'sideToleranceDeg','sideMarginM','checkSelfCollision','animate','publish', ...
    'reuseIKSolver','reuseCandidateCache','lazyCollision','progressivePoseNeighbors', ...
    'progressiveInitialPoseNeighbors','progressiveMaxPoseNeighbors','disableLocalRecovery', ...
    'poseNeighborhoodCost','angularRedundancyFallback','angularFallbackRollDeg', ...
    'angularFallbackTravelDeg','enableFallback','randomSeed'};
opts=struct();
for i=1:numel(names), if isfield(src,names{i}), opts.(names{i})=src.(names{i}); end, end
end

function p=localPositions(r)
if isfield(r,'finalSeamPositionsWP') && ~isempty(r.finalSeamPositionsWP), p=r.finalSeamPositionsWP;
elseif isfield(r,'seam') && isfield(r.seam,'positionsWP'), p=r.seam.positionsWP;
else, error('No seam positions.'); end
end

function st=localEmptyState()
st=struct('q',zeros(1,6),'travelDeg',0,'rollDeg',0,'pointIndex',0,'Ttarget',eye(4), ...
    'Tactual',eye(4),'positionErrorM',NaN,'orientationErrorDeg',NaN,'gunAxisWP',zeros(1,3), ...
    'sideDot',NaN,'headPositionWP',zeros(1,3),'environmentCollision',false,'selfCollision',false, ...
    'poseInfo',struct(),'velocityQ',zeros(1,6),'parentQ',zeros(1,6),'continuationCost',0);
end

function row=localResultRow(r,seed,wall,outFile,startLayer)
row=localEmptyRow(); row.seed=seed; row.complete=logical(r.complete);
row.nodeCollisionFree=logical(r.nodeCollisionFree); row.edgeCollisionFree=logical(r.edgeCollisionFree);
row.wallTimeSec=wall; row.algorithmTimeSec=r.algorithmElapsedSec; row.outputFile=string(outFile);
row.totalPoints=numel(r.stateBank); row.startLayer=startLayer;
empty=find(cellfun(@isempty,r.stateBank),1,'first'); if isempty(empty), empty=NaN; end
row.firstFailureLayer=empty; row.finalPointCount=size(r.qPath,1); row.maxJointJumpDeg=r.maxJointJumpDeg;
end

function out=localDiagnoseSeed22(denseReport)
out=struct('seed',20269022,'transitionLabel',"",'pairs',repmat(struct('label',"",'count',0),0,1), ...
    'poseRejectCount',0,'jointRejectCount',0,'collisionRejectCount',0,'compatibleCount',0,'freeCount',0);
idx=find([denseReport.cases.seed]==20269022,1); if isempty(idx), out.transitionLabel="missing"; return; end
c=denseReport.cases(idx); S=load(char(c.file)); original=S.result; scene=original.scene;
if ~isfield(scene,'selfCollisionSkipPairs'), scene.selfCollisionSkipPairs=localStructuralSelfCollisionSkips(scene.robot); end
fail=find(c.forwardReachableCounts==0,1,'first');
if isempty(fail) || fail<=1, out.transitionLabel="no_forward_break"; return; end
i=fail-1; out.transitionLabel=sprintf('dense layer %d -> %d (base %.2f -> %.2f)',i,i+1,c.denseBaseIndex(i),c.denseBaseIndex(i+1));
src=find(c.forwardMasks{i}); tgt=1:numel(c.layers{i+1}); labels=string.empty(0,1); counts=[];
for aa=src
    for bb=tgt
        a=c.layers{i}(aa); b=c.layers{i+1}(bb);
        dq=max(abs(rad2deg(atan2(sin(b.q-a.q),cos(b.q-a.q)))));
        dT=abs(localAngleDiffDeg(a.travelDeg,b.travelDeg)); dR=abs(localAngleDiffDeg(a.rollDeg,b.rollDeg));
        if dT>5 || dR>15
            out.poseRejectCount=out.poseRejectCount+1; continue;
        end
        if dq>15
            out.jointRejectCount=out.jointRejectCount+1; continue;
        end
        out.compatibleCount=out.compatibleCount+1;
        pairs=localCollisionPairs(scene,a.q,b.q,3);
        if isempty(pairs), out.freeCount=out.freeCount+1; else, out.collisionRejectCount=out.collisionRejectCount+1; end
        for k=1:numel(pairs)
            j=find(labels==pairs(k),1); if isempty(j), labels(end+1,1)=pairs(k); counts(end+1,1)=1; else, counts(j)=counts(j)+1; end %#ok<AGROW>
        end
    end
end
for k=1:numel(labels), out.pairs(end+1,1)=struct('label',labels(k),'count',counts(k)); end %#ok<AGROW>
end

function labels=localCollisionPairs(scene,q1,q2,resDeg)
dq=atan2(sin(q2-q1),cos(q2-q1)); n=max(1,ceil(max(abs(rad2deg(dq)))/resDeg)); labels=string.empty(0,1);
envNames=string.empty(0,1);
if isfield(scene,'workpieceProxyInfo') && ~isempty(scene.workpieceProxyInfo), envNames=string({scene.workpieceProxyInfo.name}); end
for k=0:n
    q=q1+(k/n)*dq;
    for j=1:numel(scene.environment)
        try
            cc=checkCollision(scene.robot,q,scene.environment{j},'IgnoreSelfCollision','on','Exhaustive','on');
        catch
            cc=checkCollision(scene.robot,q,scene.environment{j},'IgnoreSelfCollision','on');
        end
        if cc
            if j<=numel(envNames), labels(end+1,1)="UR5/tool <-> "+envNames(j); %#ok<AGROW>
            else, labels(end+1,1)="UR5/tool <-> environment"; %#ok<AGROW>
            end
        end
    end
    try, cs=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on'); catch, cs=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs); end
    if cs, labels(end+1,1)="robot self-collision"; end %#ok<AGROW>
end
labels=unique(labels);
end

function pairs=localStructuralSelfCollisionSkips(robot)
names=robot.BodyNames; pairs=cell(0,2);
for i=1:min(10,numel(names)-1), pairs(end+1,:)={names{i},names{i+1}}; end %#ok<AGROW>
pairs(end+1,:)={"WeldingGunMount","wrist_3_link"}; pairs(end+1,:)={"WeldingGunMount","ee_link"}; pairs(end+1,:)={"WeldingGunMount","WeldTCP"};
end

function d=localAngleDiffDeg(a,b)
d=rad2deg(atan2(sin(deg2rad(a-b)),cos(deg2rad(a-b))));
end

function x=localEmptyRow()
x=struct('seed',NaN,'complete',false,'nodeCollisionFree',false,'edgeCollisionFree',false,'wallTimeSec',NaN,'algorithmTimeSec',NaN,'totalPoints',0,'startLayer',0,'firstFailureLayer',NaN,'finalPointCount',0,'maxJointJumpDeg',NaN,'outputFile',"",'errorMessage',"");
end

function localWriteCsv(rows,file)
fid=fopen(file,'w'); fprintf(fid,'seed,complete,nodeCollisionFree,edgeCollisionFree,wallTimeSec,algorithmTimeSec,totalPoints,startLayer,firstFailureLayer,finalPointCount,maxJointJumpDeg,outputFile,errorMessage\n');
for i=1:numel(rows), x=rows(i); fprintf(fid,'%d,%d,%d,%d,%.6f,%.6f,%d,%d,%.0f,%d,%.6f,"%s","%s"\n',x.seed,x.complete,x.nodeCollisionFree,x.edgeCollisionFree,x.wallTimeSec,x.algorithmTimeSec,x.totalPoints,x.startLayer,x.firstFailureLayer,x.finalPointCount,x.maxJointJumpDeg,char(x.outputFile),char(x.errorMessage)); end
fclose(fid);
end
