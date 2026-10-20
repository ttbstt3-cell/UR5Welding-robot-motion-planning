function result=optimize_accp_controller_timing(qPath,tcpPosePath,timeFromStart,scene,options)
% Independent offline optimization: q nodes and Cartesian poses are immutable.
% Only segment durations are optimized. Velocities are always reconstructed
% by the existing exporter rule [0;diff(q)./diff(t)], never optimized freely.
% Cubic Hermite requires positions+velocities, with NO acceleration field.
if nargin<5, options=struct(); end
cfg=struct('velocityLimit',deg2rad(5),'accelerationLimit',0.261799,...
    'positionLimitMm',0.5,'orientationLimitDeg',0.5,...
    'maxIterations',400,'maxOptimizationSec',120,'prioritySegments',[],...
    'outputDirectory','');
for f=fieldnames(cfg).'
    if isfield(options,f{1}), cfg.(f{1})=options.(f{1}); end
end
Q=double(qPath); t=double(timeFromStart(:)); N=size(Q,1); D=diff(Q);
assert(size(Q,2)==6 && N>=2 && all(isfinite(Q(:))), 'Invalid qPath');
assert(numel(t)==N && all(isfinite(t)) && all(diff(t)>0),'Invalid time');
assert(iscell(tcpPosePath) && numel(tcpPosePath)==N,'Expected one 4x4 TCP pose per node');
P=zeros(N,3); R=zeros(3,3,N);
for k=1:N
    T=double(tcpPosePath{k}); assert(isequal(size(T),[4 4]) && all(isfinite(T(:))),'Invalid TCP pose');
    P(k,:)=T(1:3,4).'; R(:,:,k)=T(1:3,1:3);
end
dt0=diff(t); n=N-1; body=scene.tcpBodyName;
auditCfg=struct('samplesPerSegment',101,'maxContourErrorMm',cfg.positionLimitMm,...
    'maxTimeSyncErrorMm',cfg.positionLimitMm,'maxOrientationErrorDeg',cfg.orientationLimitDeg,...
    'maxVelocityRadPerSec',cfg.velocityLimit,'maxAccelerationRadPerSec2',cfg.accelerationLimit);
fprintf('CONTROLLER_TIMING stage=BASELINE_AUDIT\n');
before=validate_accp_controller_interpolation(Q,tcpPosePath,t,...
    [zeros(1,6);D./dt0],scene,auditCfg);
[before.max_joint_velocity_rad_per_sec,before.max_joint_acceleration_rad_per_sec2]=localPeakDerivatives(D,dt0);
before.analytic_derivative_peak_check=true;

% Position/orientation sensitivities of the fixed joint chords are used only
% to accelerate the duration search. Final acceptance uses exact FK and full
% collision checks, including both sides of every node transition.
u=linspace(0,1,21); h10=u.^3-2*u.^2+u;
Jp=zeros(3,6,n,numel(u)); Jr=Jp;
ep=zeros(3,n,numel(u)); er=ep;
poseChange=zeros(n,1);
for e=1:n
    poseChange(e)=norm(localRotVector(R(:,:,e+1)*R(:,:,e)'));
    for j=1:numel(u)
        qq=Q(e,:)+u(j)*D(e,:);
        A=getTransform(scene.robot,qq,body);
        G=interpolate_tcp_pose_slerp(tcpPosePath{e},tcpPosePath{e+1},u(j));
        J=geometricJacobian(scene.robot,qq,body);
        Jp(:,:,e,j)=1000*J(4:6,:); Jr(:,:,e,j)=rad2deg(J(1:3,:));
        ep(:,e,j)=1000*(A(1:3,4)-G(1:3,4));
        er(:,e,j)=rad2deg(localRotVector(A(1:3,1:3)*G(1:3,1:3)'));
    end
end
% Prioritize bridge/high-pose-change segments through local defect weights.
weights=1+3*poseChange/max(max(poseChange),eps);
priority=unique(cfg.prioritySegments(:)); priority=priority(priority>=1 & priority<=n);
weights(priority)=weights(priority)+3;
ub=linspace(0,1,101); firstErr=zeros(numel(ub),1);
for j=1:numel(ub)
    s=ub(j); q=Q(1,:)+(s-(s^3-2*s^2+s))*D(1,:);
    A=getTransform(scene.robot,q,body);
    G=interpolate_tcp_pose_slerp(tcpPosePath{1},tcpPosePath{2},s);
    firstErr(j)=1000*norm(A(1:3,4)-G(1:3,4));
end
firstLowerBound=max(firstErr);
timer=tic; evaluated=0;
fprintf('CONTROLLER_TIMING stage=DURATION_OPTIMIZATION first_segment_invariant=%.9fmm\n',firstLowerBound);
% Bounded log-duration search. No joint rebranching, FK changes or geometry
% changes are performed by this objective.
floorDt=max(0.02,max(abs(D),[],2)/cfg.velocityLimit/3);
lower=log(floorDt); upper=log(max(30,10*dt0));
opt=optimoptions('fmincon','Display','off','Algorithm','sqp',...
    'MaxIterations',cfg.maxIterations,'MaxFunctionEvaluations',30000,...
    'OptimalityTolerance',1e-7,'StepTolerance',1e-8,'OutputFcn',@stopBudget);
[x,fval,exitflag,info]=fmincon(@objective,log(dt0),[],[],[],[],lower,upper,[],opt);
dt=exp(x); [vPeak,aPeak]=localPeakDerivatives(D,dt);
% Uniform final dilation enforces exact cubic derivative peaks without
% changing ratios or the interpolated q(u)/TCP path.
scale=max([1,vPeak/cfg.velocityLimit,sqrt(aPeak/cfg.accelerationLimit)])*(1+1e-6);
dt=dt*scale; optimizedT=[t(1);t(1)+cumsum(dt)];
V=[zeros(1,6);D./dt];
fprintf('CONTROLLER_TIMING stage=EXACT_FK_COLLISION_AUDIT duration=%.6fs\n',sum(dt));
after=validate_accp_controller_interpolation(Q,tcpPosePath,optimizedT,V,scene,auditCfg);
[exactV,exactA]=localPeakDerivatives(D,dt);
after.max_joint_velocity_rad_per_sec=exactV;
after.max_joint_acceleration_rad_per_sec2=exactA;
after.analytic_derivative_peak_check=true;
result=struct('schema_version','ACCP_CONTROLLER_AWARE_TIMING_v1',...
    'robot_motion','NO','offlineOnly',true,'executionReady',false,...
    'numericalAcceptancePass',logical(after.complete),...
    'qPath',Q,'tcpPosePath',{tcpPosePath},'timeFromStart',optimizedT,...
    'velocities',V,'controllerMessageFields',{{'positions','velocities','time_from_start'}},...
    'accelerationFieldExported',false,'velocityRule','[zeros(1,6);diff(qPath)./diff(timeFromStart)]',...
    'segmentDurationsBefore',dt0,'segmentDurationsAfter',dt,...
    'localTimeScale',dt./dt0,'before',before,'after',after,...
    'inputQNodesUnchanged',isequal(Q,qPath),'inputTcpPosesUnchanged',true,...
    'firstSegmentTimeInvariantLowerBoundMm',firstLowerBound,...
    'timingOnlyFeasibleUnderCurrentVelocityRule',firstLowerBound<cfg.positionLimitMm,...
    'optimizationIterations',info.iterations,'objectiveEvaluations',evaluated,...
    'optimizerExitFlag',exitflag,'objectiveValue',fval,...
    'optimizationTimeSec',toc(timer),'prioritySegments',priority,'limits',cfg,...
    'physicalSafetyMarginValidated',false,'continuousCollisionProof',false,...
    'wireCollisionShortenMm',3,'wireShortenPhysicalValidationRequired',true);
if isfield(scene,'calibrationBinding'), result.calibrationBinding=scene.calibrationBinding; end
assert(isequal(result.qPath,qPath) && isequal(result.tcpPosePath,tcpPosePath));
result.failureReason=after.failureReason;
if firstLowerBound>=cfg.positionLimitMm
    result.failureReason=sprintf(['TIME_ONLY_INFEASIBLE_WITH_EXISTING_VELOCITY_RULE: '...
        'first segment error >= %.9f mm for every positive duration; %s'],firstLowerBound,after.failureReason);
end
if ~isempty(cfg.outputDirectory)
    if ~isfolder(cfg.outputDirectory), mkdir(cfg.outputDirectory); end
    save(fullfile(cfg.outputDirectory,'controller_timing_diagnostic.mat'),'result','-v7.3');
    payload=rmfield(result,{'qPath','tcpPosePath'});
    fid=fopen(fullfile(cfg.outputDirectory,'report.json'),'w'); assert(fid>=0);
    cleanup=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(payload,'PrettyPrint',true),'char'); clear cleanup;
end

    function value=objective(z)
        evaluated=evaluated+1; durations=exp(z);
        preceding=[zeros(1,6);D(1:end-1,:)./durations(1:end-1)];
        mismatch=preceding.*durations-D;
        penalty=0;
        for ee=1:n
            E=reshape(ep(:,ee,:),3,[]); O=reshape(er(:,ee,:),3,[]);
            for jj=1:numel(u)
                delta=h10(jj)*mismatch(ee,:).';
                E(:,jj)=E(:,jj)+Jp(:,:,ee,jj)*delta;
                O(:,jj)=O(:,jj)+Jr(:,:,ee,jj)*delta;
            end
            pe=vecnorm(E); oe=vecnorm(O);
            % A stricter search target compensates for the linear FK
            % approximation. Exact acceptance thresholds remain unchanged.
            penalty=penalty+weights(ee)*(sum(max(0,pe/0.40-1).^2)+sum(max(0,oe/0.35-1).^2));
        end
        [mv,ma]=localPeakDerivatives(D,durations);
        value=penalty+100*max(0,mv/cfg.velocityLimit-1)^2+...
            100*max(0,ma/cfg.accelerationLimit-1)^2+0.001*sum(durations);
    end
    function stop=stopBudget(~,~,~)
        stop=toc(timer)>cfg.maxOptimizationSec;
    end
end

function v=localRotVector(R)
a=rotm2axang(R); v=a(1:3).'*a(4);
end

function [mv,ma]=localPeakDerivatives(D,dt)
% Cubic q=q0+b*u+c*u^2+d*u^3. Acceleration is affine in u;
% check both endpoints. Velocity extrema also occur at -c/(3*d).
V=[zeros(1,6);D./dt]; mv=0; ma=0;
for e=1:size(D,1)
    h=dt(e); b=h*V(e,:); c=3*D(e,:)-2*b-h*V(e+1,:);
    d=-2*D(e,:)+b+h*V(e+1,:);
    ma=max(ma,max(abs([2*c,2*c+6*d]))/h^2);
    mv=max(mv,max(abs([V(e,:),V(e+1,:)])));
    for j=1:6
        if abs(d(j))>eps
            u=-c(j)/(3*d(j));
            if u>0 && u<1, mv=max(mv,abs(b(j)+2*c(j)*u+3*d(j)*u^2)/h); end
        end
    end
end
end

