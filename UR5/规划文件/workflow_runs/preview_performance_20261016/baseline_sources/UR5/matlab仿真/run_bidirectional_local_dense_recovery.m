function report = run_bidirectional_local_dense_recovery(options)
%RUN_BIDIRECTIONAL_LOCAL_DENSE_RECOVERY
% Forward-reachable local dense states are combined with a backward
% reachable set generated from the remaining seam suffix.  This is a
% diagnostic/recovery experiment only; the normal planner is untouched.

if nargin<1 || isempty(options), options=struct(); end
options=localOptions(options);
D=load(options.denseReportFile); denseReport=D.report;
rows=repmat(localEmptyRow(),0,1); results=cell(0,1);

fprintf('\n====================================================\n');
fprintf('BIDIRECTIONAL LOCAL DENSE RECOVERY\n');
fprintf('Forward set + suffix backward set + local meet\n');
fprintf('====================================================\n');

for ic=1:numel(denseReport.cases)
    c=denseReport.cases(ic); seed=c.seed;
    S=load(char(c.file)); original=S.result;
    fprintf('\nSeed %d, local base window [%d,%d]\n',seed,c.originalWindow(1),c.originalWindow(2));
    t=tic;
    try
        out=localSolveCase(original,c,options);
        out.elapsedSec=toc(t); results{end+1,1}=out; %#ok<AGROW>
        row=localRow(out); rows(end+1,1)=row; %#ok<AGROW>
        fprintf('  meet=%d complete=%d forwardFinal=%d backwardFirst=%d maxJump=%.3f deg TCPseg=%.4f mm\n', ...
            out.meetLayer,out.complete,out.forwardFinal,out.backwardFirst,out.maxJointJumpDeg,out.maxTcpSegmentErrorMm);
    catch ME
        out=struct('seed',seed,'errorMessage',string(ME.message),'complete',false,'elapsedSec',toc(t));
        results{end+1,1}=out; %#ok<AGROW>
        row=localEmptyRow(); row.seed=seed; row.errorMessage=string(ME.message); row.wallTimeSec=out.elapsedSec;
        rows(end+1,1)=row; %#ok<AGROW>
        warning('Seed %d BLDR failed: %s',seed,ME.message);
    end
end

report=struct('generatedAt',string(datetime('now')),'cases',{results}, ...
    'denseReportFile',options.denseReportFile,'outputMat',options.outputMat,'outputCsv',options.outputCsv);
save(options.outputMat,'report','-v7.3'); localWriteCsv(rows,options.outputCsv);
fprintf('\nSaved MAT: %s\nSaved CSV: %s\n',options.outputMat,options.outputCsv);
end

function out=localSolveCase(original,c,options)
scene=original.scene;
if ~isfield(scene,'ikSolver'), scene.ikSolver=inverseKinematics('RigidBodyTree',scene.robot); end
if ~isfield(scene,'jointLimits'), scene.jointLimits=localJointLimits(scene.robot); end
if ~isfield(scene,'selfCollisionSkipPairs'), scene.selfCollisionSkipPairs=localStructuralSelfCollisionSkips(scene.robot); end
scene.bldrMaxTravelStepDeg=options.maxTravelStepDeg;
scene.bldrMaxRollStepDeg=options.maxRollStepDeg;
scene.bldrMaxJointStepDeg=options.maxJointStepDeg;
scene.ikCandidateCache=containers.Map('KeyType','char','ValueType','any');
basePositions=localPositions(original); lo=c.originalWindow(1); hi=c.originalWindow(2); N=size(basePositions,1);
tangent=localTangent(original,basePositions); gunAxis=scene.weldGunAxisWP(:); gunAxis=gunAxis/norm(gunAxis);
seedPool=localSeedPool(original.stateBank,c.layers,scene,options);

% Build a backward reachable set on the original seam suffix.  The suffix
% endpoint is generated independently, so this is not a continuation of
% the failed forward branch.
[suffix,suffixParent]=localBuildBackwardSuffix(scene,basePositions,hi,seedPool,tangent,gunAxis,options);

M=numel(c.layers); back=cell(M,1); backParent=cell(M,1);
for i=1:M, back{i}=false(1,numel(c.layers{i})); backParent{i}=zeros(1,numel(c.layers{i})); end
if isempty(suffix)
    back{M}(:)=true; backToSuffix=zeros(1,numel(c.layers{M}));
else
    backToSuffix=zeros(1,numel(c.layers{M}));
    for a=1:numel(c.layers{M})
        for b=1:numel(suffix{1})
            if localEdgeFree(scene,c.layers{M}(a),suffix{1}(b),options.edgeResolutionDeg)
                back{M}(a)=true; backToSuffix(a)=b; break;
            end
        end
    end
end
for i=M-1:-1:1
    for a=1:numel(c.layers{i})
        for b=1:numel(c.layers{i+1})
            if ~back{i+1}(b), continue; end
            if localEdgeFree(scene,c.layers{i}(a),c.layers{i+1}(b),options.edgeResolutionDeg)
                back{i}(a)=true; backParent{i}(a)=b; break;
            end
        end
    end
end

% Recompute forward reachability with the strict BLDR edge predicate.  The
% diagnostic dense report's forward mask intentionally ignored the boundary
% joint-step test, so it must not be reused as a final-path certificate.
[forward,forwardParent]=localForwardDenseReachability(scene,original,c,lo,options);
meetLayer=0; meetState=0;
for i=1:M
    both=find(forward{i} & back{i},1,'first');
    if ~isempty(both), meetLayer=i; meetState=both; break; end
end

out=struct(); out.seed=c.seed; out.originalWindow=c.originalWindow; out.forwardCounts=cellfun(@sum,forward);
out.backwardCounts=cellfun(@sum,back); out.forwardFinal=out.forwardCounts(end); out.backwardFirst=out.backwardCounts(1);
out.meetLayer=meetLayer; out.suffixCounts=cellfun(@numel,suffix); out.complete=false;
out.qPath=zeros(0,6); out.pathStates=repmat(localEmptyState(),0,1); out.maxJointJumpDeg=NaN; out.maxTcpSegmentErrorMm=NaN;
out.nodeCollisionFree=false; out.edgeCollisionFree=false; out.elapsedSec=NaN; out.errorMessage="";
if meetLayer==0, out.diagnosis="NO_FORWARD_BACKWARD_MEET"; return; end

% Recover dense prefix to the meet state using the stored forward parents.
idx=zeros(1,M); idx(meetLayer)=meetState;
for i=meetLayer:-1:2, idx(i-1)=c.forwardParent{i}(idx(i)); end
pathStates=repmat(localEmptyState(),0,1);
% Original prefix ends at the point before the dense window.
for i=1:lo-1
    if isempty(original.stateBank{i}), out.diagnosis="EMPTY_GLOBAL_PREFIX"; return; end
    pathStates(end+1,1)=localAugmentState(original.stateBank{i}(1)); %#ok<AGROW>
end
for i=1:meetLayer
    pathStates(end+1,1)=localAugmentState(c.layers{i}(idx(i))); %#ok<AGROW>
end

cur=meetState;
for i=meetLayer:M-1
    nxt=backParent{i}(cur); if nxt==0, out.diagnosis="BROKEN_DENSE_BACKTRACE"; return; end
    pathStates(end+1,1)=localAugmentState(c.layers{i+1}(nxt)); %#ok<AGROW>
    cur=nxt;
end
if ~isempty(suffix)
    sidx=backToSuffix(cur); if sidx==0, out.diagnosis="NO_DENSE_TO_SUFFIX_EDGE"; return; end
    pathStates(end+1,1)=localAugmentState(suffix{1}(sidx)); %#ok<AGROW>
    for i=1:numel(suffix)-1
        sidx=suffixParent{i}(sidx); if sidx==0, out.diagnosis="BROKEN_SUFFIX_BACKTRACE"; return; end
        pathStates(end+1,1)=localAugmentState(suffix{i+1}(sidx)); %#ok<AGROW>
    end
end

% Ensure the final path contains all original seam endpoint positions.  The
% suffix builder uses every original point after the local window.
v=localValidatePath(scene,pathStates,options);
out.complete=v.complete; out.nodeCollisionFree=v.nodeCollisionFree; out.edgeCollisionFree=v.edgeCollisionFree;
out.pathStates=pathStates; out.qPath=vertcat(pathStates.q); out.maxJointJumpDeg=v.maxJointJumpDeg;
out.maxTcpSegmentErrorMm=v.maxTcpSegmentErrorMm; out.diagnosis=v.diagnosis;
out.forwardParent=forwardParent; out.backwardParent=backParent;
end

function [forward,parent]=localForwardDenseReachability(scene,original,c,lo,options)
M=numel(c.layers);
forward=cell(M,1); parent=cell(M,1);
for i=1:M
    forward{i}=false(1,numel(c.layers{i}));
    parent{i}=zeros(1,numel(c.layers{i}));
end

% Boundary states are the last states of the globally reachable prefix.
left=repmat(localEmptyState(),0,1);
if lo>1 && lo-1<=numel(original.stateBank) && ~isempty(original.stateBank{lo-1})
    for k=1:numel(original.stateBank{lo-1})
        left(end+1,1)=localAugmentState(original.stateBank{lo-1}(k)); %#ok<AGROW>
    end
end
if isempty(left)
    forward{1}(:)=true;
else
    for j=1:numel(c.layers{1})
        for k=1:numel(left)
            if localEdgeFree(scene,left(k),c.layers{1}(j),options.edgeResolutionDeg)
                forward{1}(j)=true;
                parent{1}(j)=k;
                break;
            end
        end
    end
end

for i=1:M-1
    for a=1:numel(c.layers{i})
        if ~forward{i}(a), continue; end
        for b=1:numel(c.layers{i+1})
            if forward{i+1}(b), continue; end
            if localEdgeFree(scene,c.layers{i}(a),c.layers{i+1}(b),options.edgeResolutionDeg)
                forward{i+1}(b)=true;
                parent{i+1}(b)=a;
            end
        end
    end
end
end

function [suffix,parent]=localBuildBackwardSuffix(scene,basePositions,hi,seedPool,tangent,gunAxis,options)
N=size(basePositions,1); suffix=cell(0,1); parent=cell(0,1);
if hi>=N, return; end
% Endpoint first.  Pose center is the validated physical orientation center.
nextCenter=[0 90];
for i=N:-1:hi+1
    if i<N && ~isempty(suffix), nextCenter=localPoseCenter(suffix{1}); end
    states=localGenerateLayer(scene,basePositions(i,:),nextCenter,seedPool,tangent,gunAxis,i,options);
    if isempty(states), suffix={}; parent={}; return; end
    suffix=[{states}; suffix]; %#ok<AGROW>
    parent=[{zeros(1,numel(states))}; parent]; %#ok<AGROW>
end
% Backward filtering: process from the endpoint toward hi+1.  suffix{end}
% is the endpoint and all its states are backward targets.
for i=numel(suffix)-1:-1:1
    next=suffix{i+1}; keep=false(1,numel(suffix{i})); p=zeros(1,numel(suffix{i}));
    for a=1:numel(suffix{i})
        for b=1:numel(next)
            if localEdgeFree(scene,suffix{i}(a),next(b),options.edgeResolutionDeg)
                keep(a)=true; p(a)=b; break;
            end
        end
    end
    suffix{i}=suffix{i}(keep); p=p(keep);
    if isempty(suffix{i}), suffix={}; parent={}; return; end
    parent{i}=p;
end
end

function states=localGenerateLayer(scene,pWP,center,qSeeds,tangent,gunAxis,pointIndex,options)
states=repmat(localEmptyState(),0,1);
for dt=options.offsetGrid
    for dr=options.offsetGrid
        travel=center(1)+dt; roll=center(2)+dr;
        [Rtcp,~]=make_actual_gun_axis_pose(scene,tangent,gunAxis,travel,roll);
        T=eye(4); T(1:3,1:3)=Rtcp; T(1:3,4)=pWP(:); Ttarget=scene.T_MatBase_Workpiece*T;
        ikOpt=struct('positionToleranceM',options.positionToleranceM,'orientationToleranceDeg',options.orientationToleranceDeg, ...
            'sideToleranceDeg',options.sideToleranceDeg,'sideMarginM',options.sideMarginM,'checkSelfCollision',true, ...
            'lazyCollision',false,'reuseCandidateCache',true,'verbose',false);
        rr=generate_actual_axis_ik_candidates(scene,Ttarget,travel,roll,qSeeds,ikOpt);
        for k=1:rr.numAccepted
            st=rr.states(k); st.pointIndex=pointIndex; states(end+1,1)=st; %#ok<AGROW>
        end
    end
end
states=localUniqueStates(states,0.5);
end

function q=localSeedPool(bank,layers,scene,options)
q=scene.qMat0;
for i=1:numel(bank)
    if isempty(bank{i}), continue; end
    for k=1:numel(bank{i}), q(end+1,:)=bank{i}(k).q; end %#ok<AGROW>
end
for i=1:numel(layers)
    for k=1:numel(layers{i}), q(end+1,:)=layers{i}(k).q; end %#ok<AGROW>
end
for k=1:options.randomSeedsPerPose
    try
        x=randomConfiguration(scene.robot); if isstruct(x), names=scene.robot.JointNames; x=cell2mat(arrayfun(@(nm)x.(char(nm)),names,'UniformOutput',false)); end
        q(end+1,:)=double(x(:).'); %#ok<AGROW>
    catch
    end
end
q=localUniqueQ(q,0.25); if size(q,1)>options.maxSeedsPerPose, q=q(1:options.maxSeedsPerPose,:); end
end

function center=localPoseCenter(states)
if isempty(states), center=[0 90]; else, center=median([[states.travelDeg].',[states.rollDeg].'],1); end
end

function ok=localEdgeFree(scene,a,b,resDeg)
% Apply the same hard continuity limits as the planner before expensive
% interpolated collision checks.
if isfield(a,'travelDeg') && isfield(b,'travelDeg') && ...
        abs(double(b.travelDeg)-double(a.travelDeg)) > scene.bldrMaxTravelStepDeg + 1e-9
    ok=false; return;
end
if isfield(a,'rollDeg') && isfield(b,'rollDeg') && ...
        localCircularAngleDistanceDeg(double(b.rollDeg),double(a.rollDeg)) > scene.bldrMaxRollStepDeg + 1e-9
    ok=false; return;
end
dq=atan2(sin(b.q-a.q),cos(b.q-a.q)); n=max(1,ceil(max(abs(rad2deg(dq)))/resDeg)); ok=true;
if max(abs(rad2deg(dq))) > scene.bldrMaxJointStepDeg + 1e-9
    ok=false; return;
end
for k=0:n
    q=a.q+(k/n)*dq;
    try, [cc,~,~]=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on'); catch, cc=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on'); end
    if any(cc(:)), ok=false; return; end
    try, cs=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on'); catch, cs=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs); end
    if any(cs(:)), ok=false; return; end
end
end

function d=localCircularAngleDistanceDeg(a,b)
d=abs(rad2deg(atan2(sind(a-b),cosd(a-b))));
end

function v=localValidatePath(scene,path,options)
v=struct('complete',false,'nodeCollisionFree',true,'edgeCollisionFree',true,'maxJointJumpDeg',0,'maxTcpSegmentErrorMm',0,'diagnosis',"");
if isempty(path), v.nodeCollisionFree=false; v.edgeCollisionFree=false; v.diagnosis="EMPTY_PATH"; return; end
for i=1:numel(path)
    if ~localStateFree(scene,path(i).q), v.nodeCollisionFree=false; v.diagnosis="NODE_COLLISION"; return; end
end
for i=1:numel(path)-1
    dq=atan2(sin(path(i+1).q-path(i).q),cos(path(i+1).q-path(i).q)); v.maxJointJumpDeg=max(v.maxJointJumpDeg,max(abs(rad2deg(dq))));
    if ~localEdgeFree(scene,path(i),path(i+1),options.edgeResolutionDeg), v.edgeCollisionFree=false; v.diagnosis="EDGE_COLLISION_OR_DISCONTINUITY"; return; end
    n=max(1,ceil(max(abs(rad2deg(dq)))/options.edgeResolutionDeg));
    for k=0:n
        u=k/n; q=path(i).q+(u)*dq; T=getTransform(scene.robot,q,scene.tcpBodyName);
        p=(1-u)*path(i).Ttarget(1:3,4)+u*path(i+1).Ttarget(1:3,4); v.maxTcpSegmentErrorMm=max(v.maxTcpSegmentErrorMm,1000*norm(T(1:3,4)-p));
    end
end
if v.maxTcpSegmentErrorMm > options.maxTcpSegmentErrorMm
    v.edgeCollisionFree=false;
    v.diagnosis="TCP_SEGMENT_ERROR";
end
v.complete=v.nodeCollisionFree && v.edgeCollisionFree;
if v.complete, v.diagnosis="PASS"; end
end

function ok=localStateFree(scene,q)
try, [cc,~,~]=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on'); catch, cc=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on'); end
if any(cc(:)), ok=false; return; end
try, cs=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on'); catch, cs=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs); end
ok=~any(cs(:));
end

function p=localPositions(r)
if isfield(r,'finalSeamPositionsWP') && ~isempty(r.finalSeamPositionsWP), p=r.finalSeamPositionsWP; elseif isfield(r,'seam') && isfield(r.seam,'positionsWP'), p=r.seam.positionsWP; else, error('No seam positions.'); end
end
function t=localTangent(r,p)
if isfield(r,'seam') && isfield(r.seam,'tangentWP'), t=r.seam.tangentWP(:); else, t=(p(end,:)-p(1,:)).'; end; t=t/norm(t);
end
function st=localAugmentState(x)
st=localEmptyState(); names=fieldnames(st);
for i=1:numel(names)
    if isfield(x,names{i}), st.(names{i})=x.(names{i}); end
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

function lim=localJointLimits(robot)
lim=zeros(6,2); j=0;
for i=1:numel(robot.Bodies)
    jt=robot.Bodies{i}.Joint;
    if strcmpi(jt.Type,'revolute')||strcmpi(jt.Type,'continuous')
        j=j+1; lim(j,:)=double(jt.PositionLimits);
    end
end
if j~=6, error('Expected six joints.'); end
end

function pairs=localStructuralSelfCollisionSkips(robot)
names=robot.BodyNames; pairs=cell(0,2);
for i=1:min(10,numel(names)-1)
    pairs(end+1,:)={names{i},names{i+1}}; %#ok<AGROW>
end
pairs(end+1,:)={"WeldingGunMount","wrist_3_link"}; %#ok<AGROW>
pairs(end+1,:)={"WeldingGunMount","ee_link"}; %#ok<AGROW>
pairs(end+1,:)={"WeldingGunMount","WeldTCP"}; %#ok<AGROW>
end
function options=localOptions(options)
root='/home/ttbstt/jixiebi/UR5/matlab仿真';
defaults=struct( ...
    'denseReportFile',fullfile(root,'local_dense_connectivity.mat'), ...
    'maxSeedsPerPose',16, ...
    'randomSeedsPerPose',6, ...
    'offsetGrid',-30:5:30, ...
    'maxRollStepDeg',15, ...
    'maxTravelStepDeg',5, ...
    'maxJointStepDeg',15, ...
    'edgeResolutionDeg',3, ...
    'positionToleranceM',1e-4, ...
    'orientationToleranceDeg',0.5, ...
    'sideToleranceDeg',5, ...
    'sideMarginM',1e-3, ...
    'maxTcpSegmentErrorMm',0.5, ...
    'outputMat',fullfile(root,'bidirectional_local_dense_recovery.mat'), ...
    'outputCsv',fullfile(root,'bidirectional_local_dense_recovery.csv'));
n=fieldnames(defaults);
for i=1:numel(n)
    if ~isfield(options,n{i}) || isempty(options.(n{i}))
        options.(n{i})=defaults.(n{i});
    end
end
end

function row=localRow(o)
row=localEmptyRow();
row.seed=o.seed;
row.complete=o.complete;
row.nodeCollisionFree=o.nodeCollisionFree;
row.edgeCollisionFree=o.edgeCollisionFree;
row.wallTimeSec=o.elapsedSec;
row.meetLayer=o.meetLayer;
row.forwardFinal=o.forwardFinal;
row.backwardFirst=o.backwardFirst;
row.maxJointJumpDeg=o.maxJointJumpDeg;
row.maxTcpSegmentErrorMm=o.maxTcpSegmentErrorMm;
row.diagnosis=o.diagnosis;
end

function x=localEmptyRow()
x=struct('seed',NaN,'complete',false,'nodeCollisionFree',false, ...
    'edgeCollisionFree',false,'wallTimeSec',NaN,'meetLayer',0, ...
    'forwardFinal',0,'backwardFirst',0,'maxJointJumpDeg',NaN, ...
    'maxTcpSegmentErrorMm',NaN,'diagnosis',"",'errorMessage',"");
end

function x=localEmptyState()
x=struct('q',zeros(1,6),'travelDeg',0,'rollDeg',0,'pointIndex',0, ...
    'Ttarget',eye(4),'Tactual',eye(4),'positionErrorM',NaN, ...
    'orientationErrorDeg',NaN,'gunAxisWP',zeros(1,3),'sideDot',NaN, ...
    'headPositionWP',zeros(1,3),'environmentCollision',false, ...
    'selfCollision',false,'poseInfo',struct(),'velocityQ',zeros(1,6), ...
    'parentQ',zeros(1,6),'continuationCost',0);
end

function localWriteCsv(rows,file)
fid=fopen(file,'w');
if fid<0, error('Cannot open CSV: %s',file); end
fprintf(fid,['seed,complete,nodeCollisionFree,edgeCollisionFree,wallTimeSec,' ...
    'meetLayer,forwardFinal,backwardFirst,maxJointJumpDeg,' ...
    'maxTcpSegmentErrorMm,diagnosis,errorMessage\n']);
for i=1:numel(rows)
    x=rows(i);
    fprintf(fid,'%d,%d,%d,%d,%.6f,%d,%d,%d,%.6f,%.6f,%s,%s\n', ...
        x.seed,x.complete,x.nodeCollisionFree,x.edgeCollisionFree, ...
        x.wallTimeSec,x.meetLayer,x.forwardFinal,x.backwardFirst, ...
        x.maxJointJumpDeg,x.maxTcpSegmentErrorMm,char(x.diagnosis), ...
        char(x.errorMessage));
end
fclose(fid);
end
