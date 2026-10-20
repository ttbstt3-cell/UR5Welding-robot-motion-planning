function out = refine_accp_cartesian_path(core, options)
%REFINE_ACCP_CARTESIAN_PATH Internal ACCP precision stage, no motion.
% Original ACCP endpoints and continuous joint branches are preserved. Every
% inserted pose uses the existing ACCP IK/axis/limit/collision filters. Final
% error is measured along raw joint interpolation, not only at IK nodes.
arguments
    core struct
    options struct = struct()
end
defaults=struct('maxSegmentTcpErrorMm',0.5,'maxIntermediatePoints',4, ...
    'maxJointStepDeg',15,'positionToleranceM',1e-4, ...
    'orientationToleranceDeg',0.5,'sideToleranceDeg',5,'sideMarginM',1e-3, ...
    'axis_angle_tolerance_deg',[],'errorSamplesPerEdge',21,...
    'performanceExactQueryCache',false,'performanceBinding',struct());
f=fieldnames(defaults);
for i=1:numel(f), if ~isfield(options,f{i}),options.(f{i})=defaults.(f{i});end,end
configurationComplete=core.complete;original=double(core.qPath);
if isfield(core,'configurationComplete'),configurationComplete=core.configurationComplete;end
if isfield(core,'qPathInitial'),original=double(core.qPathInitial);end
assert(configurationComplete && size(original,2)==6 && size(original,1)>=2, ...
    'Cartesian refinement requires a complete ACCP chain.');
validateattributes(options.errorSamplesPerEdge,{'numeric'},{'scalar','integer','>=',21});
scene=core.scene; N=size(original,1);
if options.performanceExactQueryCache
    % Read immutable model limits once, instead of copying every rigid body
    % for every dense sample. Values and hard inequalities stay identical.
    options.performanceJointLimits=deriveUR5JointLimits(scene.robot);
    scene.exactPreviewQueries=HmiExactSceneQueries(scene,options.performanceBinding,true);
end
targets=cell(N,1);
for i=1:N
    states=core.stateBank{i}; Q=vertcat(states.q);
    % Match the selected raw continuous branch (no per-point angle wrapping).
    [d,j]=min(vecnorm(Q-original(i,:),2,2));
    assert(d<1e-8,'Selected ACCP target/branch not found at point %d.',i);
    targets{i}=states(j).Ttarget;
end
scene.ikSolver=inverseKinematics('RigidBodyTree',scene.robot);
scene.ikSolver.SolverParameters.AllowRandomRestart=false;
ikOptions=struct('positionToleranceM',options.positionToleranceM, ...
    'orientationToleranceDeg',options.orientationToleranceDeg, ...
    'sideToleranceDeg',options.sideToleranceDeg,'sideMarginM',options.sideMarginM, ...
    'axis_angle_tolerance_deg',options.axis_angle_tolerance_deg, ...
    'checkSelfCollision',true,'reuseCandidateCache',false, ...
    'disableAxisConstraint',false,'disable_axis_constraint',false);
initial=measure(scene,original,targets,options);
collisionChecks=initial.collisionChecks;
qExec=original(1,:); TExec=targets(1); originalIndices=1;
empty=struct('initialErrorMm',0,'initialOrientationErrorDeg',0, ...
    'initialJointJumpDeg',0,'initialCollisionFree',false, ...
    'correctionTrigger',"cartesian",'elapsedSec',0,'ikCalls',0, ...
    'ikElapsedSec',0,'correctionAttempted',true,'correctionAccepted',false, ...
    'insertedPoints',0,'finalErrorMm',inf,'failureReason',"not attempted");
report=repmat(empty,N-1,1); calls=0; ikTime=0;
for i=1:N-1
    tSegment=tic;
    report(i).initialErrorMm=initial.segmentPositionErrorMm(i);
    report(i).initialOrientationErrorDeg=initial.segmentOrientationErrorDeg(i);
    report(i).initialJointJumpDeg=initial.segmentJointJumpDeg(i);
    report(i).initialCollisionFree=initial.segmentCollisionFree(i);
    % Densify every seam segment; if the first density fails, increase it.
    % maxIntermediatePoints is the existing precision-stage budget, unchanged.
    for nExtra=1:options.maxIntermediatePoints
        Q=original(i,:); Ts=targets(i); ok=true;
        reason="";
        for k=1:nExtra
            s=k/(nExtra+1); goal=interpolate_tcp_pose_slerp(targets{i},targets{i+1},s);
            seed=original(i,:)+s*(original(i+1,:)-original(i,:));
            tIK=tic;
            candidates=generate_actual_axis_ik_candidates(scene,goal,0,0, ...
                [seed;Q(end,:)],ikOptions);
            dt=toc(tIK); calls=calls+candidates.numIKCalls; ikTime=ikTime+dt;
            collisionChecks=collisionChecks+candidates.numIKPoseOK-candidates.numSideRejected;
            report(i).ikCalls=report(i).ikCalls+candidates.numIKCalls;
            report(i).ikElapsedSec=report(i).ikElapsedSec+dt;
            if isempty(candidates.qCandidates)
                ok=false; reason="IK or original ACCP hard filter rejected inserted pose"; break;
            end
            C=candidates.qCandidates;
            % Raw joint distance: do not hide a 2*pi branch jump.
            step=max(abs(rad2deg(C-Q(end,:))),[],2);
            C=C(step<=options.maxJointStepDeg,:);
            if isempty(C),ok=false;reason="continuous branch/joint step";break;end
            [~,j]=min(vecnorm(C-Q(end,:),2,2));
            Q=[Q;C(j,:)]; Ts=[Ts;{goal}]; %#ok<AGROW>
        end
        if ok
            Q=[Q;original(i+1,:)]; Ts=[Ts;targets(i+1)]; %#ok<AGROW>
            m=measure(scene,Q,Ts,options);
            collisionChecks=collisionChecks+m.collisionChecks;
            ok=m.nodeCollisionFree && m.edgeCollisionFree && ...
                max(m.segmentJointJumpDeg)<=options.maxJointStepDeg && ...
                max(m.nodePositionErrorMm)<options.maxSegmentTcpErrorMm && ...
                max(m.segmentPositionErrorMm)<options.maxSegmentTcpErrorMm && ...
                all(m.nodeHardConstraintsPass);
            if ~ok,reason="dense edge collision, branch, hard constraint or TCP error";end
        end
        if ok
            qExec=[qExec;Q(2:end,:)]; TExec=[TExec;Ts(2:end)]; %#ok<AGROW>
            report(i).correctionAccepted=true;
            report(i).insertedPoints=nExtra;
            report(i).finalErrorMm=max(m.segmentPositionErrorMm);
            report(i).failureReason="";
            break;
        end
        report(i).failureReason=reason;
    end
    if ~report(i).correctionAccepted
        % Diagnostic output only on failure; complete=false prohibits use.
        qExec=[qExec;original(i+1,:)]; TExec=[TExec;targets(i+1)]; %#ok<AGROW>
    end
    originalIndices(end+1,1)=size(qExec,1); %#ok<AGROW>
    report(i).elapsedSec=toc(tSegment);
end
final=measure(scene,qExec,TExec,options);
collisionChecks=collisionChecks+final.collisionChecks;
assert(isequal(qExec(originalIndices,:),original),'Original ACCP nodes changed.');
complete=all([report.correctionAccepted]) && final.nodeCollisionFree && ...
    final.edgeCollisionFree && all(final.nodeHardConstraintsPass) && ...
    max(final.segmentJointJumpDeg)<=options.maxJointStepDeg && ...
    max(final.segmentPositionErrorMm)<options.maxSegmentTcpErrorMm && ...
    max(final.nodePositionErrorMm)<options.maxSegmentTcpErrorMm;
out=struct('qPathOriginal',original,'originalNodeIndices',originalIndices, ...
    'tcpTargetOriginal',{targets},'qPathExecution',qExec,'tcpTargetExecution',{TExec}, ...
    'interpolation',"Cartesian position-linear + quaternion SLERP + continuous-branch IK", ...
    'initialTcpMetrics',initial,'finalTcpMetrics',final,'segmentReport',report, ...
    'complete',complete,'ikCalls',calls,'ikElapsedSec',ikTime, ...
    'collisionChecks',collisionChecks,'options',options);
if isfield(scene,'exactPreviewQueries')
    out.performanceQueries=scene.exactPreviewQueries.summary();
end
end

function m=measure(scene,Q,targets,options)
N=size(Q,1); nSamples=options.errorSamplesPerEdge;
m=struct('nodePositionErrorMm',zeros(N,1),'nodeOrientationErrorDeg',zeros(N,1), ...
    'nodeHardConstraintsPass',true(N,1),'segmentPositionErrorMm',zeros(N-1,1), ...
    'segmentOrientationErrorDeg',zeros(N-1,1),'segmentJointJumpDeg',zeros(N-1,1), ...
    'segmentCollisionFree',true(N-1,1),'nodeCollisionFree',true, ...
    'edgeCollisionFree',true,'checkedSegments',N-1,'allSegmentsChecked',true, ...
    'errorSamplesPerEdge',nSamples,'collisionChecks',0);
for i=1:N
    T=forward(scene,Q(i,:));
    [m.nodePositionErrorMm(i),m.nodeOrientationErrorDeg(i)]=poseError(T,targets{i});
    m.nodeHardConstraintsPass(i)=hardState(scene,Q(i,:),T,options) && ...
        m.nodePositionErrorMm(i)<=options.positionToleranceM*1000 && ...
        m.nodeOrientationErrorDeg(i)<=options.orientationToleranceDeg;
    free=collisionFree(scene,Q(i,:)); m.collisionChecks=m.collisionChecks+1;
    m.nodeCollisionFree=m.nodeCollisionFree && free;
end
for i=1:N-1
    dq=Q(i+1,:)-Q(i,:);
    m.segmentJointJumpDeg(i)=max(abs(rad2deg(dq)));
    % Always sample midpoints; a 3-degree grid can otherwise sample endpoints
    % only and miss Cartesian chord error. Finer joint grid for large jumps.
    count=max(nSamples,2*ceil(m.segmentJointJumpDeg(i)/0.25)+1);
    for k=0:count-1
        s=k/(count-1); q=Q(i,:)+s*dq;
        T=forward(scene,q);
        goal=interpolate_tcp_pose_slerp(targets{i},targets{i+1},s);
        [p,r]=poseError(T,goal);
        m.segmentPositionErrorMm(i)=max(m.segmentPositionErrorMm(i),p);
        m.segmentOrientationErrorDeg(i)=max(m.segmentOrientationErrorDeg(i),r);
        free=collisionFree(scene,q); m.collisionChecks=m.collisionChecks+1;
        m.segmentCollisionFree(i)=m.segmentCollisionFree(i) && free && hardState(scene,q,T,options);
    end
end
m.edgeCollisionFree=all(m.segmentCollisionFree);
end

function [p,r]=poseError(T,goal)
p=norm(T(1:3,4)-goal(1:3,4))*1000;
c=(trace(goal(1:3,1:3).'*T(1:3,1:3))-1)/2;
r=rad2deg(acos(max(-1,min(1,c))));
end

function ok=hardState(scene,q,T,options)
if isfield(options,'performanceJointLimits')
    limits=options.performanceJointLimits;
else
    limits=zeros(6,2); j=0;
    for i=1:numel(scene.robot.Bodies)
        jt=scene.robot.Bodies{i}.Joint;
        if strcmp(jt.Type,'revolute') || strcmp(jt.Type,'continuous')
            j=j+1;limits(j,:)=jt.PositionLimits;
        end
    end
end
ok=all(isfinite(q)) && all(q>=limits(:,1).'-1e-8) && all(q<=limits(:,2).'+1e-8);
RWB=scene.T_MatBase_Workpiece(1:3,1:3).';
gB=T(1:3,1:3)*scene.wireAxisTcp(:); gWP=RWB*gB; gWP=gWP/norm(gWP);
axis=[0;1;-1];if isfield(scene,'weldGunAxisWP'),axis=scene.weldGunAxisWP(:);end
axis=axis/norm(axis); tolerance=options.sideToleranceDeg;
if ~isempty(options.axis_angle_tolerance_deg),tolerance=options.axis_angle_tolerance_deg;end
d=0.020;if isfield(scene,'hullHeadDistanceM') && isfinite(scene.hullHeadDistanceM),d=scene.hullHeadDistanceM;end
head=RWB*(T(1:3,4)-d*gB-scene.T_MatBase_Workpiece(1:3,4));
% Same physical axis and welding-side inequalities as ACCP, not relaxed.
ok=ok && dot(gWP,axis)>=cosd(tolerance) && ...
    head(2)<0.230-options.sideMarginM && head(3)>0.005+options.sideMarginM;
end

function ok=collisionFree(scene,q)
if isfield(scene,'exactPreviewQueries')
    ok=scene.exactPreviewQueries.isFree(q); return;
end
env=check_scene_environment_collision(scene,q);
self=checkCollision(scene.robot,q,'IgnoreSelfCollision','off', ...
    'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
ok=~any(env(:)) && ~any(self(:));
end

function T=forward(scene,q)
if isfield(scene,'exactPreviewQueries')
    T=scene.exactPreviewQueries.forward(q,scene.tcpBodyName);
else
    T=getTransform(scene.robot,q,scene.tcpBodyName);
end
end
