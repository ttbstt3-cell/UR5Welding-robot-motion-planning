function timing = adaptive_time_scale_accp_output(qPath, tcpPosePath, scene, options)
%ADAPTIVE_TIME_SCALE_ACCP_OUTPUT ACCP Stage 3: adaptive node timing.
% The qPath and Cartesian targets are immutable inputs.  Only timeFromStart is
% optimized.  This module is internal to ACCP and never sends a ROS command.
if nargin < 4 || isempty(options), options=struct(); end
tTiming=tic;
motionLimits=hmi_scene_motion_limits(scene);
if ~isfield(scene,'hmiMotionLimits'), motionLimits.accelerationLimit=deg2rad(15); end
cfg=struct('maxVelocityRadPerSec',motionLimits.velocityLimit,...
    'maxAccelerationRadPerSec2',motionLimits.accelerationLimit,'maxDurationSec',50,...
    'minimumSegmentDurationSec',0.25,'lambda',0.001,'samplesPerSegment',101,...
    'collisionPass',false,'collisionSampleCount',0,'maxIterations',3000,...
    'controllerSamplesPerSegment',101,'controllerMaxOrientationErrorDeg',0.5);
fn=fieldnames(cfg);
for i=1:numel(fn), if isfield(options,fn{i}), cfg.(fn{i})=options.(fn{i}); end, end
Q=double(qPath);
if size(Q,2)~=6||size(Q,1)<2||any(~isfinite(Q(:))), error('ACCP_TIME:QPath','qPath must be finite N-by-6.'); end
[P,R]=localPoseSource(tcpPosePath,size(Q,1));
[baseT,~]=parameterize_joint_path(Q);
D=diff(Q,1,1); n=size(D,1); vLim=cfg.maxVelocityRadPerSec; aLim=cfg.maxAccelerationRadPerSec2;
dtFloor=max(max(abs(D),[],2)/vLim,cfg.minimumSegmentDurationSec);

dqFeature=vecnorm(D,2,2); dqFeature=dqFeature/max(max(dqFeature),eps);
tcpStep=vecnorm(diff(P,1,1),2,2); tcpFeature=tcpStep/max(max(tcpStep),eps);
poseStep=zeros(n,1); for i=1:n, poseStep(i)=localRotationAngle(R(:,:,i),R(:,:,i+1)); end
poseFeature=poseStep/max(max(poseStep),eps);
curvature=zeros(n,1); dP=diff(P,1,1); lens=vecnorm(dP,2,2); dirs=dP./max(lens,eps);
if n>1
    turns=zeros(n-1,1); for i=1:n-1, turns(i)=acos(max(-1,min(1,dot(dirs(i,:),dirs(i+1,:))))); end
    curvature(1)=turns(1); curvature(end)=turns(end);
    if n>2, curvature(2:end-1)=0.5*(turns(1:end-1)+turns(2:end)); end
end
curvature=curvature/max(max(curvature),eps);
feature=1+0.20*dqFeature+0.20*tcpFeature+0.15*poseFeature+0.25*curvature;
seedDt=max(dtFloor,diff(baseT).*feature); x0=log(max(seedDt-dtFloor,1e-6));
obj=@(x)localObjective(x,dtFloor,D,vLim,aLim,cfg,x0);
opts=optimset('Display','off','MaxIter',cfg.maxIterations,...
    'MaxFunEvals',max(10000,10*cfg.maxIterations),'TolX',1e-6,'TolFun',1e-6);
[xOpt,fval,exitflag,output]=fminsearch(obj,x0,opts);
dt=dtFloor+exp(xOpt); timeFromStart=[0;cumsum(dt)];
[segV,segA]=localTimingDerivatives(D,dt);
velocities=[zeros(1,6);segV]; accelerations=[zeros(1,6);segA;zeros(1,6)];
if size(accelerations,1)~=size(Q,1), accelerations=zeros(size(Q)); end

% The positions-only controller model is sampled densely for TCP metrics.
[td,qd,seg,u]=localSampleLinear(Q,timeFromStart,cfg.samplesPerSegment);
[Pe,Re]=localSamplePose(P,R,seg,u); [Pa,Ra]=localFk(scene.robot,qd,localField(scene,'tcpBodyName','tool0'));
syncErr=1000*vecnorm(Pa-Pe,2,2); contourErr=localPolylineError(P,Pa);
oriErr=localOrientationErrors(Ra,Re);
[denseV,denseA]=localSampleDerivatives(qd,td);
limits=localJointLimits(scene.robot); limitPass=all(all(qd>=limits(:,1).'-1e-10))&&all(all(qd<=limits(:,2).'+1e-10));
segmentVelocity=max(abs(segV(:))); segmentAcceleration=max(abs(segA(:)));
% The controller-facing representation exported by ACCP is positions-only
% with explicit node timestamps.  At a node, a finite difference over the
% tiny sample interval straddling two linear segments reports an artificial
% acceleration spike (the velocity slope is discontinuous by construction).
% Keep that diagnostic, but use the same piecewise-linear segment metrics as
% the returned velocities/accelerations for the execution gate.
finiteDifferenceVelocity=max(abs(denseV(:)));
finiteDifferenceAcceleration=max(abs(denseA(:)));
% The time-parameterized message carries velocities/accelerations to the
% ROS2 controller.  Validate the exact controller-facing interpolation before
% declaring this ACCP stage complete; segment-average slopes alone are not a
% sufficient acceleration or TCP-tracking check.
controllerValidation=validate_accp_controller_interpolation(Q,tcpPosePath,...
    timeFromStart,velocities,scene,struct(...
    'samplesPerSegment',cfg.controllerSamplesPerSegment,...
    'maxContourErrorMm',0.5,'maxTimeSyncErrorMm',0.5,...
    'maxOrientationErrorDeg',cfg.controllerMaxOrientationErrorDeg,...
    'maxVelocityRadPerSec',vLim,...
    'maxAccelerationRadPerSec2',aLim));
complete= max(contourErr)<0.5 && max(syncErr)<0.5 && limitPass && logical(cfg.collisionPass) && ...
    segmentVelocity<=vLim+eps && segmentAcceleration<=aLim+eps && timeFromStart(end)<=cfg.maxDurationSec;
complete=complete && logical(controllerValidation.complete);
reason='';
if ~complete
    reasons={}; if max(contourErr)>=0.5, reasons{end+1}=sprintf('TCP contour %.6f mm >= 0.5 mm',max(contourErr)); end
    if max(syncErr)>=0.5, reasons{end+1}=sprintf('time sync %.6f mm >= 0.5 mm',max(syncErr)); end
    if ~limitPass, reasons{end+1}='joint limits failed'; end
    if ~cfg.collisionPass, reasons{end+1}='dense collision validation was not PASS'; end
    if segmentVelocity>vLim, reasons{end+1}=sprintf('segment velocity %.6f rad/s exceeds limit',segmentVelocity); end
    if segmentAcceleration>aLim, reasons{end+1}=sprintf('segment acceleration %.6f rad/s^2 exceeds limit',segmentAcceleration); end
    if timeFromStart(end)>cfg.maxDurationSec, reasons{end+1}=sprintf('duration %.6f s exceeds limit',timeFromStart(end)); end
    if ~controllerValidation.complete, reasons{end+1}=controllerValidation.failureReason; end
    reason=['ACCP_TIME_PARAMETERIZATION_FAILED: ' strjoin(reasons,'; ')];
end
timing=struct('schema_version','ACCP_ADAPTIVE_TIME_SCALING_v1',...
    'stage','Adaptive Time Scaling','algorithm','ACCP','robot_motion','NO',...
    'q_node_count',size(Q,1),'qPath',Q,'tcpPath',P,'timeFromStart',timeFromStart,...
    'velocities',velocities,'accelerations',accelerations,...
    'segmentDurationsSec',dt,'segmentDeltaQMaxRad',max(abs(D),[],2),...
    'segmentTcpStepM',tcpStep,'segmentPoseChangeRad',poseStep,'segmentTcpCurvature',curvature,...
    'maxVelocityRadPerSec',segmentVelocity,...
    'maxAccelerationRadPerSec2',segmentAcceleration,...
    'accelerationMetric','piecewise_linear_segment_slope',...
    'accelerationMetricNote','Finite-difference node-corner spikes are diagnostic only.',...
    'finiteDifferenceMaxVelocityRadPerSec',finiteDifferenceVelocity,...
    'finiteDifferenceMaxAccelerationRadPerSec2',finiteDifferenceAcceleration,...
    'controllerInterpolationValidation',controllerValidation,...
    'controllerInterpolationPass',logical(controllerValidation.complete),...
    'controllerInterpolationMode',controllerValidation.interpolation_mode,...
    'maxSegmentVelocityRadPerSec',segmentVelocity,'maxSegmentAccelerationRadPerSec2',segmentAcceleration,...
    'tcpContourMaxMm',max(contourErr),'tcpContourRmsMm',sqrt(mean(contourErr.^2)),...
    'timeSyncMaxMm',max(syncErr),'timeSyncRmsMm',sqrt(mean(syncErr.^2)),...
    'maxOrientationErrorDeg',max(oriErr),'collisionPass',logical(cfg.collisionPass),...
    'collisionSampleCount',cfg.collisionSampleCount,'collisionValidationSource',...
    'ACCP Cartesian refinement dense node/edge validation (inherited)',...
    'jointLimitsPass',limitPass,...
    'durationSec',timeFromStart(end),'maxDurationSec',cfg.maxDurationSec,...
    'velocityLimitRadPerSec',vLim,'accelerationLimitRadPerSec2',aLim,...
    'objectiveValue',fval,'optimizerExitFlag',exitflag,'optimizerIterations',output.iterations,...
    'algorithmTimeSec',toc(tTiming),...
    'complete',logical(complete),'executionReady',false,'failureReason',reason,...
    'sampleCount',numel(td));
end

function f=localObjective(x,dtFloor,D,vLim,aLim,cfg,x0)
dt=dtFloor+exp(x); [v,a]=localTimingDerivatives(D,dt); mv=max(abs(v(:))); ma=max(abs(a(:))); T=sum(dt);
vv=max(0,mv/vLim-1); av=max(0,ma/aLim-1); dv=max(0,T-cfg.maxDurationSec)/max(cfg.maxDurationSec,eps);
f=ma/aLim+0.02*T+10*vv^2+10*av^2+100*dv^2+cfg.lambda*sum(x.^2)+1e-5*sum((x-x0).^2);
end

function [v,a]=localTimingDerivatives(D,dt)
v=D./dt(:,ones(1,size(D,2))); if size(v,1)<2, a=zeros(size(v)); return; end
den=(dt(1:end-1)+dt(2:end))/2; a=diff(v,1,1)./den(:,ones(1,size(D,2)));
end

function [t,Q,seg,uOut]=localSampleLinear(Q0,t0,m)
n=size(Q0,1); total=1+(n-1)*(m-1); t=zeros(total,1); Q=zeros(total,6); seg=zeros(total,1); uOut=zeros(total,1); k=1;
for e=1:n-1, h=t0(e+1)-t0(e); for j=1:m, if e>1&&j==1, continue; end, u=(j-1)/(m-1); t(k)=t0(e)+u*h; Q(k,:)=(1-u)*Q0(e,:)+u*Q0(e+1,:); seg(k)=e; uOut(k)=u; k=k+1; end, end
end

function [P,R]=localSamplePose(P0,R0,seg,u)
P=zeros(numel(seg),3); R=zeros(3,3,numel(seg));
for i=1:numel(seg), e=seg(i); z=u(i); P(i,:)=(1-z)*P0(e,:)+z*P0(e+1,:); R(:,:,i)=localSlerp(R0(:,:,e),R0(:,:,e+1),z); end
end

function R=localSlerp(A,B,u)
qa=rotm2quat(A); qb=rotm2quat(B); if dot(qa,qb)<0, qb=-qb; end, d=max(-1,min(1,dot(qa,qb)));
if d>0.9995, q=(1-u)*qa+u*qb; q=q/norm(q); else, th=acos(d); q=(sin((1-u)*th)*qa+sin(u*th)*qb)/sin(th); end, R=quat2rotm(q);
end

function [P,R]=localFk(robot,Q,body)
P=zeros(size(Q,1),3); R=zeros(3,3,size(Q,1)); for i=1:size(Q,1), T=getTransform(robot,Q(i,:),body); P(i,:)=T(1:3,4).'; R(:,:,i)=T(1:3,1:3); end
end

function e=localPolylineError(P,A)
e=zeros(size(A,1),1); for i=1:size(A,1), best=Inf; for k=1:size(P,1)-1, v=P(k+1,:)-P(k,:); z=max(0,min(1,dot(A(i,:)-P(k,:),v)/max(dot(v,v),eps))); best=min(best,norm(A(i,:)-(P(k,:)+z*v))); end, e(i)=1000*best; end
end

function e=localOrientationErrors(A,B)
e=zeros(size(A,3),1); for i=1:size(A,3), c=(trace(B(:,:,i)'*A(:,:,i))-1)/2; e(i)=rad2deg(acos(max(-1,min(1,c)))); end
end

function [v,a]=localSampleDerivatives(Q,t)
dt=diff(t); v=diff(Q)./dt(:,ones(1,6)); if size(v,1)<2, a=zeros(size(v)); return; end, den=(dt(1:end-1)+dt(2:end))/2; a=diff(v,1,1)./den(:,ones(1,6));
end

function lim=localJointLimits(robot)
lim=zeros(6,2); j=0; for i=1:numel(robot.Bodies), typ=robot.Bodies{i}.Joint.Type; if strcmp(typ,'revolute')||strcmp(typ,'continuous'), j=j+1; lim(j,:)=robot.Bodies{i}.Joint.PositionLimits; end, end
end

function [P,R]=localPoseSource(x,N)
P=zeros(N,3); R=zeros(3,3,N); if iscell(x), for i=1:N, T=double(x{i}); P(i,:)=T(1:3,4).'; R(:,:,i)=T(1:3,1:3); end; return; end
if ndims(x)==3&&size(x,1)==4, for i=1:N, P(i,:)=x(1:3,4,i).'; R(:,:,i)=x(1:3,1:3,i); end; return; end
error('ACCP_TIME:TCPPath','Expected N TCP 4x4 poses.');
end

function a=localRotationAngle(A,B), c=(trace(A'*B)-1)/2; a=acos(max(-1,min(1,c))); end
function v=localField(s,n,d), if isfield(s,n), v=s.(n); else, v=d; end, end
