function report = validate_accp_controller_interpolation(qPath,tcpPosePath,timeFromStart,velocities,scene,options)
%VALIDATE_ACCP_CONTROLLER_INTERPOLATION Offline controller-equivalent audit.
% Reconstructs the interpolation used by the ROS2 joint trajectory
% controller: positions+velocities are evaluated as piecewise cubic Hermite;
% when no velocity field is present the controller-facing message is treated
% as positions-only linear interpolation.  Every sampled configuration is
% checked with FK, joint limits, environment collision and non-exempt
% self-collision.  This is a diagnostic/gating helper only: it never creates
% or sends a ROS action goal.

if nargin < 6 || isempty(options), options=struct(); end
cfg=struct('samplesPerSegment',101,'maxContourErrorMm',0.5,...
    'maxTimeSyncErrorMm',0.5,'maxOrientationErrorDeg',0.5,...
    'maxVelocityRadPerSec',inf,'maxAccelerationRadPerSec2',inf,'pathTimeReference',[]);
fn=fieldnames(cfg);
for k=1:numel(fn)
    if isfield(options,fn{k}) && ~isempty(options.(fn{k})), cfg.(fn{k})=options.(fn{k}); end
end
cfg.samplesPerSegment=max(101,round(double(cfg.samplesPerSegment)));

Q=double(qPath); N=size(Q,1);
if size(Q,2)~=6 || N<2 || any(~isfinite(Q(:)))
    error('ACCP_CONTROLLER:QPath','qPath must be finite N-by-6 with N>=2.');
end
t=double(timeFromStart(:));
if numel(t)~=N || any(~isfinite(t)) || any(diff(t)<=0)
    error('ACCP_CONTROLLER:Time','timeFromStart must be finite and strictly increasing.');
end
V=[];
if ~isempty(velocities)
    V=double(velocities);
    if ~isequal(size(V),size(Q)) || any(~isfinite(V(:)))
        error('ACCP_CONTROLLER:Velocity','velocities must be finite N-by-6.');
    end
end
[Pexp,Rexp]=localExpectedPose(tcpPosePath,N);

% Build one dense sample per controller segment.  The first sample of every
% segment after the first is shared with the preceding segment.
m=cfg.samplesPerSegment;
M=1+(N-1)*(m-1);
ts=zeros(M,1); Qs=zeros(M,6); Qd=zeros(M,6); Qdd=zeros(M,6);
seg=zeros(M,1); uOut=zeros(M,1);
kk=1;
for e=1:N-1
    h=t(e+1)-t(e);
    for j=1:m
        if e>1 && j==1, continue; end
        u=(j-1)/(m-1); ts(kk)=t(e)+u*h; seg(kk)=e; uOut(kk)=u;
        if isempty(V)
            Qs(kk,:)=(1-u)*Q(e,:)+u*Q(e+1,:);
            Qd(kk,:)=(Q(e+1,:)-Q(e,:))/h;
            Qdd(kk,:)=0;
        else
            [Qs(kk,:),Qd(kk,:),Qdd(kk,:)]=localHermite(Q(e,:),Q(e+1,:),V(e,:),V(e+1,:),h,u);
        end
        kk=kk+1;
    end
end
if isempty(cfg.pathTimeReference)
    [Ptarget,Rtarget]=localSampleExpected(Pexp,Rexp,seg,uOut);
else
    [Ptarget,Rtarget]=hmi_continuous_reference(cfg.pathTimeReference,ts);
end
[Pactual,Ractual]=localFk(scene.robot,Qs,localField(scene,'tcpBodyName','tool0'));
positionErrMm=1000*vecnorm(Pactual-Ptarget,2,2);
orientationErrDeg=localOrientationErrors(Ractual,Rtarget);
contourErrMm=localPolylineError(Pexp,Pactual);

limits=localJointLimits(scene.robot);
if isfield(scene,'jointLimits')
    limits=[max(limits(:,1),scene.jointLimits(:,1)),min(limits(:,2),scene.jointLimits(:,2))];
end
limitPass=all(all(Qs>=limits(:,1).'-1e-10)) && all(all(Qs<=limits(:,2).'+1e-10));
collisionPass=true; collisionChecks=0; minGap=NaN; firstCollision=[];
for i=1:size(Qs,1)
    [free,gap,envHit,selfHit]=localStateFreeWithGap(scene,Qs(i,:));
    collisionChecks=collisionChecks+1;
    if isfinite(gap), minGap=localMinFinite(minGap,gap); end
    if ~free
        collisionPass=false;
        if isempty(firstCollision)
            firstCollision=struct('sample_index',i,'time_sec',ts(i),...
                'segment',seg(i),'segment_u',uOut(i),'q_rad',Qs(i,:),...
                'environment_collision',logical(envHit),'self_collision',logical(selfHit));
        end
    end
end

maxVel=max(abs(Qd(:))); [~,iv]=max(abs(Qd(:))); %#ok<ASGLU>
maxAcc=max(abs(Qdd(:))); [~,ia]=max(abs(Qdd(:))); %#ok<ASGLU>
[ivTime,ivJoint]=ind2sub(size(Qd),iv); [iaTime,iaJoint]=ind2sub(size(Qdd),ia);
reasons={};
if max(contourErrMm)>=cfg.maxContourErrorMm
    reasons{end+1}=sprintf('controller TCP contour %.6f mm >= %.6f mm',max(contourErrMm),cfg.maxContourErrorMm); %#ok<AGROW>
end
if max(positionErrMm)>=cfg.maxTimeSyncErrorMm
    reasons{end+1}=sprintf('controller TCP time-sync %.6f mm >= %.6f mm',max(positionErrMm),cfg.maxTimeSyncErrorMm); %#ok<AGROW>
end
if max(orientationErrDeg)>cfg.maxOrientationErrorDeg
    reasons{end+1}=sprintf('controller TCP orientation %.6f deg > %.6f deg',max(orientationErrDeg),cfg.maxOrientationErrorDeg); %#ok<AGROW>
end
if ~limitPass, reasons{end+1}='controller interpolation exceeded joint limits'; end %#ok<AGROW>
if ~collisionPass, reasons{end+1}='controller interpolation dense collision check failed'; end %#ok<AGROW>
if maxVel>cfg.maxVelocityRadPerSec
    reasons{end+1}=sprintf('controller peak joint velocity %.9g rad/s exceeds %.9g',maxVel,cfg.maxVelocityRadPerSec); %#ok<AGROW>
end
if maxAcc>cfg.maxAccelerationRadPerSec2
    reasons{end+1}=sprintf('controller peak joint acceleration %.9g rad/s^2 exceeds %.9g',maxAcc,cfg.maxAccelerationRadPerSec2); %#ok<AGROW>
end

if isempty(V)
    mode='positions_only_linear';
    source='controller message has positions and timestamps only';
else
    mode='positions_plus_velocities_cubic_hermite';
    source='ROS2 joint_trajectory_controller trajectory.hpp behavior verified by offline export audit';
end
report=struct();
report.schema_version='ACCP_CONTROLLER_INTERPOLATION_VALIDATION_v1';
if ~isempty(cfg.pathTimeReference), report.time_reference_mode=cfg.pathTimeReference.mode; end
report.stage='Adaptive Time Scaling / controller-equivalent validation';
report.robot_motion='NO';
report.interpolation_mode=mode;
report.interpolation_rule_source=source;
report.waypoint_count=N;
report.sample_count=size(Qs,1);
report.samples_per_segment=m;
report.duration_sec=t(end);
report.max_tcp_contour_error_mm=max(contourErrMm);
report.rms_tcp_contour_error_mm=sqrt(mean(contourErrMm.^2));
report.max_tcp_time_sync_error_mm=max(positionErrMm);
report.rms_tcp_time_sync_error_mm=sqrt(mean(positionErrMm.^2));
report.max_tcp_time_sync_error_time_sec=ts(find(positionErrMm==max(positionErrMm),1,'first'));
report.max_tcp_orientation_error_deg=max(orientationErrDeg);
[~,ip]=max(positionErrMm); [~,io]=max(orientationErrDeg); [~,ic]=max(contourErrMm);
report.max_position_error_case=errorCase(ip);
report.max_orientation_error_case=errorCase(io);
report.max_contour_error_case=errorCase(ic);
report.max_joint_velocity_rad_per_sec=maxVel;
report.max_joint_velocity_time_sec=ts(ivTime);
report.max_joint_velocity_joint=ivJoint;
report.max_joint_acceleration_rad_per_sec2=maxAcc;
report.max_joint_acceleration_time_sec=ts(iaTime);
report.max_joint_acceleration_joint=iaJoint;
report.joint_limits_pass=logical(limitPass);
report.collision_pass=logical(collisionPass);
report.collision_sample_count=collisionChecks;
report.min_positive_clearance_mm=minGap;
report.first_collision=firstCollision;
report.acceptance=struct('max_tcp_contour_error_mm',cfg.maxContourErrorMm,...
    'max_tcp_time_sync_error_mm',cfg.maxTimeSyncErrorMm,...
    'max_tcp_orientation_error_deg',cfg.maxOrientationErrorDeg,...
    'max_joint_velocity_rad_per_sec',cfg.maxVelocityRadPerSec,...
    'max_joint_acceleration_rad_per_sec2',cfg.maxAccelerationRadPerSec2);
report.complete=isempty(reasons);
report.executionReady=false;
if isempty(reasons), report.failureReason='';
else, report.failureReason=['ACCP_CONTROLLER_INTERPOLATION_FAILED: ' strjoin(reasons,'; ')]; end
report.note='Dense offline audit only. No FollowJointTrajectory goal was created or sent.';
    function detail=errorCase(i)
        actual=eye(4); expected=eye(4);
        actual(1:3,1:3)=Ractual(:,:,i); actual(1:3,4)=Pactual(i,:).';
        expected(1:3,1:3)=Rtarget(:,:,i); expected(1:3,4)=Ptarget(i,:).';
        detail=struct('time_sec',ts(i),'path_segment_index',seg(i),'segment_u',uOut(i),...
            'q_rad',Qs(i,:),'q_deg',rad2deg(Qs(i,:)),...
            'joint_velocity_rad_s',Qd(i,:),'joint_acceleration_rad_s2',Qdd(i,:),...
            'actual_TCP_pose',actual,'expected_TCP_pose',expected,'frame_id',scene.robot.BaseName,...
            'position_error_mm',positionErrMm(i),'orientation_error_deg',orientationErrDeg(i),...
            'contour_error_mm',contourErrMm(i));
    end
end

function [q,qd,qdd]=localHermite(q0,q1,v0,v1,h,u)
h00=2*u^3-3*u^2+1; h10=u^3-2*u^2+u;
h01=-2*u^3+3*u^2; h11=u^3-u^2;
q=h00*q0+h10*h*v0+h01*q1+h11*h*v1;
d00=6*u^2-6*u; d10=3*u^2-4*u+1;
d01=-6*u^2+6*u; d11=3*u^2-2*u;
qd=(d00*q0+d10*h*v0+d01*q1+d11*h*v1)/h;
dd00=12*u-6; dd10=6*u-4; dd01=-12*u+6; dd11=6*u-2;
qdd=(dd00*q0+dd10*h*v0+dd01*q1+dd11*h*v1)/(h*h);
end

function [P,R]=localExpectedPose(x,N)
P=zeros(N,3); R=zeros(3,3,N);
if iscell(x)
    if numel(x)~=N, error('ACCP_CONTROLLER:ExpectedPose','Expected pose count mismatch.'); end
    for i=1:N
        T=double(x{i});
        if ~isequal(size(T),[4 4]) || any(~isfinite(T(:))), error('ACCP_CONTROLLER:ExpectedPose','Invalid 4x4 TCP pose.'); end
        P(i,:)=T(1:3,4).'; R(:,:,i)=T(1:3,1:3);
    end
elseif ndims(x)==3 && size(x,1)==4 && size(x,2)==4 && size(x,3)==N
    for i=1:N, P(i,:)=x(1:3,4,i).'; R(:,:,i)=x(1:3,1:3,i); end
else
    error('ACCP_CONTROLLER:ExpectedPose','tcpPosePath must contain N 4x4 poses.');
end
end

function [P,R]=localSampleExpected(P0,R0,seg,u)
P=zeros(numel(seg),3); R=zeros(3,3,numel(seg));
for i=1:numel(seg)
    e=seg(i); a=u(i);
    P(i,:)=(1-a)*P0(e,:)+a*P0(e+1,:);
    R(:,:,i)=localSlerp(R0(:,:,e),R0(:,:,e+1),a);
end
end

function R=localSlerp(A,B,u)
qa=rotm2quat(A); qb=rotm2quat(B); if dot(qa,qb)<0, qb=-qb; end
d=max(-1,min(1,dot(qa,qb)));
if d>0.9995, q=(1-u)*qa+u*qb; q=q/norm(q);
else, th=acos(d); q=(sin((1-u)*th)*qa+sin(u*th)*qb)/sin(th); end
R=quat2rotm(q/norm(q));
end

function [P,R]=localFk(robot,Q,body)
P=zeros(size(Q,1),3); R=zeros(3,3,size(Q,1));
for i=1:size(Q,1)
    T=getTransform(robot,Q(i,:),body); P(i,:)=T(1:3,4).'; R(:,:,i)=T(1:3,1:3);
end
end

function e=localOrientationErrors(A,B)
e=zeros(size(A,3),1);
for i=1:size(A,3)
    c=(trace(B(:,:,i)'*A(:,:,i))-1)/2;
    e(i)=rad2deg(acos(max(-1,min(1,c))));
end
end

function e=localPolylineError(P,A)
e=zeros(size(A,1),1);
for i=1:size(A,1)
    best=Inf;
    for k=1:size(P,1)-1
        v=P(k+1,:)-P(k,:); z=max(0,min(1,dot(A(i,:)-P(k,:),v)/max(dot(v,v),eps)));
        best=min(best,norm(A(i,:)-(P(k,:)+z*v)));
    end
    e(i)=1000*best;
end
end

function lim=localJointLimits(robot)
lim=zeros(6,2); j=0;
for i=1:numel(robot.Bodies)
    typ=robot.Bodies{i}.Joint.Type;
    if strcmp(typ,'revolute')||strcmp(typ,'continuous')
        j=j+1; lim(j,:)=robot.Bodies{i}.Joint.PositionLimits;
    end
end
end

function [ok,gap,envHit,selfHit]=localStateFreeWithGap(scene,q)
envHit=false; selfHit=false; gap=NaN;
if isfield(scene,'environment') && ~isempty(scene.environment)
    [env,distEnv]=check_scene_environment_collision(scene,q);
    envHit=any(env(:));
else
    distEnv=[];
end
try
    if isfield(scene,'selfCollisionSkipPairs')
        [self,distSelf]=checkCollision(scene.robot,q,'IgnoreSelfCollision','off',...
            'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
    else
        [self,distSelf]=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','Exhaustive','on');
    end
catch
    [self,distSelf]=checkCollision(scene.robot,q,'IgnoreSelfCollision','off');
end
selfHit=any(self(:));
d=[double(distEnv(:));double(distSelf(:))]; d=d(isfinite(d)&d>0);
if ~isempty(d), gap=min(d)*1000; end
ok=~envHit && ~selfHit;
end

function g=localMinFinite(a,b)
if ~isfinite(b), g=a; elseif ~isfinite(a), g=b; else, g=min(a,b); end
end

function v=localField(s,n,d)
if isfield(s,n), v=s.(n); else, v=d; end
end
