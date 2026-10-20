function report = compare_accp_time_parameterizations(result, outputFile, options)
%COMPARE_ACCP_TIME_PARAMETERIZATIONS Offline timing candidates for ACCP output.
% No ACCP search, TCP, geometry, collision thresholds, timestamps, or ROS
% execution are changed. Each candidate is sampled densely (>=3001 samples)
% and compared with the expected Cartesian weld path.
%
% Schemes:
%   baseline_cubic          current export: forward-difference velocities
%   positions_only_linear   positions only, controller "none"/linear model
%   shape_preserving_cubic  limited monotone joint velocities
%   retimed_cubic           cubic with geometry-aware node time allocation
%   adaptive_time_scaled_positions_only  positions-only with adaptive node timing
%
% The returned report is diagnostic. It does not mark a trajectory executable.

if nargin < 1 || ~isstruct(result), error('result struct is required.'); end
if nargin < 2 || isempty(outputFile), outputFile=fullfile(pwd,'accp_time_parameterization_comparison.json'); end
if nargin < 3 || isempty(options), options=struct(); end
cfg=defaults(options);
Q=fieldOr(result,'qPathExecution',fieldOr(result,'qPath',[]));
if isempty(Q)||size(Q,2)~=6||size(Q,1)<2, error('result.qPathExecution/qPath must be N-by-6.'); end
Q=double(Q);
P0=fieldOr(result,'tcpTargetExecution',fieldOr(result,'tcpPosePath',[]));
[P,R,hasR]=poseSource(P0);
if size(P,1)~=size(Q,1), error('Expected Cartesian path and qPath row counts differ.'); end
scene=fieldOr(result,'scene',[]);
if isempty(scene)||~isfield(scene,'robot'), error('result.scene.robot is required for FK.'); end
baseT=fieldOr(result,'timeFromStart',[]);
if isempty(baseT), [baseT,~]=parameterize_joint_path(Q); end
baseT=double(baseT(:));
if numel(baseT)~=size(Q,1)||any(diff(baseT)<=0), error('Invalid base timeFromStart.'); end
N=size(Q,1); m=max(cfg.samplesPerSegment,ceil(3000/(N-1))+1);

Vbase=[zeros(1,6);diff(Q,1,1)./diff(baseT)];
Vshape=shapeVelocities(Q,baseT);
[retimedT,retimeSearch]=chooseRetimedTimes(Q,P,baseT,m,scene);
[scaledT,timeScaleSearch]=chooseAdaptiveTimeScaling(Q,P,R,hasR,baseT,m,cfg);

spec=cell(5,1);
spec{1}=struct('name','baseline_cubic','time',baseT,'velocity',Vbase,'description','current exporter forward-difference velocities');
spec{2}=struct('name','positions_only_linear','time',baseT,'velocity',[],'description','positions only; linear joint interpolation');
spec{3}=struct('name','shape_preserving_cubic','time',baseT,'velocity',Vshape,'description','limited shape-preserving joint velocities');
spec{4}=struct('name','retimed_cubic','time',retimedT,'velocity',[zeros(1,6);diff(Q,1,1)./diff(retimedT)],'description','geometry-aware node time allocation + current forward-difference cubic');
spec{5}=struct('name','adaptive_time_scaled_positions_only','time',scaledT,'velocity',[],'description','positions-only with adaptive node time allocation');

schemeCells=cell(numel(spec),1);
for i=1:numel(spec)
    evalCfg=cfg;
    % Uniform time scaling produces exactly the same Q samples at each
    % segment fraction as positions_only_linear. Reuse its dense collision
    % result rather than checking the identical configurations twice.
    if strcmp(spec{i}.name,'adaptive_time_scaled_positions_only')
        evalCfg.collisionCheck=false;
    end
    schemeCells{i}=evaluateScheme(scene,Q,P,R,hasR,spec{i},m,evalCfg);
    if strcmp(spec{i}.name,'adaptive_time_scaled_positions_only')
        linearIdx=find(cellfun(@(x)strcmp(x.name,'positions_only_linear'),spec),1);
        if ~isempty(linearIdx) && ~isempty(schemeCells{linearIdx})
            schemeCells{i}.validation=schemeCells{linearIdx}.validation;
        end
    end
end
schemes=vertcat(schemeCells{:});

syncMax = arrayfun(@(s)s.time_sync.max_position_error_mm,schemes);
[~,bestIdx]=min(syncMax);
report=struct();
report.schema_version='ACCP_TIME_PARAMETERIZATION_COMPARISON_v3';
report.created=char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
report.robot_motion='NO';
report.point_count=N; report.samples_per_segment=m;
report.base_duration_sec=baseT(end);
report.controller_interface=struct('action','/scaled_joint_trajectory_controller/follow_joint_trajectory',...
    'command_interfaces','[position]','state_interfaces','[position velocity]',...
    'interpolation_method','splines','positions_only','linear',...
    'positions_plus_velocities','cubic spline','positions_plus_velocities_plus_accelerations','quintic spline',...
    'evidence','UR5/logs/ur5_driver_reconnect.log and /opt/ros/humble/include/joint_trajectory_controller/trajectory.hpp');
report.retiming_search=retimeSearch;
report.time_scaling_optimization=timeScaleSearch;
report.schemes=schemes;
report.best_offline_candidate=schemes(bestIdx).name;
report.best_time_scaled_candidate='adaptive_time_scaled_positions_only';
report.acceptance=struct('geometric_path_max_mm',cfg.geometricAcceptanceMm,...
    'geometric_path_pass',schemes(bestIdx).contour.max_error_mm<=cfg.geometricAcceptanceMm,...
    'time_sync_is_separate_metric',true,...
    'time_sync_gate_configured',cfg.timeSyncAcceptanceMm<Inf,...
    'time_sync_acceptance_mm',cfg.timeSyncAcceptanceMm,...
    'execution_ready',false,...
    'reason','Diagnostic comparison only; final export gate is not changed in this pass.');
report.note='No candidate was exported or sent to ROS2. Collision and limit checks are dense diagnostics.';

[outDir,outName,~]=fileparts(char(string(outputFile)));
if isempty(outDir),outDir=pwd; end
if ~isfolder(outDir), mkdir(outDir); end
jsonPath=fullfile(outDir,[outName '.json']);
csvPath=fullfile(outDir,[outName '_samples.csv']);
matPath=fullfile(outDir,[outName '_samples.mat']);
allSamples=cell(numel(schemes),1);
for i=1:numel(schemes), allSamples{i}=schemes(i).sampleData; end
report.schemes=rmfield(schemes,'sampleData');
report.artifacts=struct('json',jsonPath,'csv',csvPath,'mat',matPath);
writeSamplesCsv(csvPath,allSamples);
save(matPath,'allSamples','report','-v7');
fid=fopen(jsonPath,'w'); if fid<0,error('Cannot write %s',jsonPath);end
c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(report,'PrettyPrint',true),'char');
fprintf('ACCP timing comparison: %s\n',jsonPath);
for i=1:numel(schemes)
    s=schemes(i);
    fprintf('%s: contour %.6f mm, sync %.6f mm RMS %.6f mm, orient %.6f deg, duration %.6f s, collision=%d limits=%d\n',...
        s.name,s.contour.max_error_mm,s.time_sync.max_position_error_mm,s.time_sync.rms_position_error_mm,...
        s.orientation.max_error_deg,s.duration_sec,s.validation.collision_pass,s.validation.joint_limits_pass);
end
end

function cfg=defaults(x)
cfg=struct('samplesPerSegment',101,'scene',[],'collisionCheck',true,...
    'geometricAcceptanceMm',0.5,'timeSyncAcceptanceMm',Inf,...
    'timeScalingMaxVelocityRadPerSec',deg2rad(5),...
    'timeScalingMaxAccelerationRadPerSec2',deg2rad(15),...
    'adaptiveTimeScalingLambda',0.001,...
    'adaptiveTimeScalingMaxDurationSec',50,...
    'adaptiveTimeScalingMaxIterations',3000,...
    'adaptiveTimeScalingMinimumSegmentDurationSec',0.25);
f=fieldnames(cfg);
for i=1:numel(f), if isfield(x,f{i}), cfg.(f{i})=x.(f{i}); end, end
end

function s=evaluateScheme(scene,Q,P,R,hasR,sp,m,cfg)
[t,Qs,seg,u]=sampleTimed(Q,sp.time,sp.velocity,m);
[Pe,Re]=sampleExpected(P,R,hasR,seg,u);
[Pa,Ra]=fkPath(scene.robot,Qs,fieldOr(scene,'tcpBodyName','tool0'));
syncErr=1000*sqrt(sum((Pa-Pe).^2,2));
oriErr=orientationErrors(Ra,Re,hasR);
contourErr=polylineErrors(P,Pa);
[vel,acc]=sampleDerivatives(Qs,t);
limits=jointLimits(scene.robot);
limitPass=all(all(Qs>=limits(:,1).'-1e-10))&&all(all(Qs<=limits(:,2).'+1e-10));
collisionPass=true; collisionCount=0; collisionFirst='';
if cfg.collisionCheck
    for k=1:size(Qs,1)
        [ok,detail]=collisionFree(scene,Qs(k,:));
        collisionCount=collisionCount+1;
        if ~ok&&isempty(collisionFirst), collisionFirst=detail; end
        collisionPass=collisionPass&&ok;
    end
end
s=struct();
s.name=sp.name; s.description=sp.description; s.duration_sec=t(end);
s.time_parameterization=struct('time_from_start_sec',sp.time(:).',...
    'velocities_rad_per_sec',sp.velocity,'velocity_source',velocitySource(sp.velocity));
s.contour=metricSummary(contourErr,'TCP to Cartesian weld polyline');
s.time_sync=metricSummary(syncErr,'TCP to expected Cartesian pose at same time');
s.time_sync.max_position_error_time_sec=t(find(syncErr==max(syncErr),1));
s.orientation=struct('available',hasR,'max_error_deg',maxFinite(oriErr),...
    'rms_error_deg',rmsFinite(oriErr),'max_error_time_sec',timeOfMaxFinite(oriErr,t));
s.joint_motion=struct('max_velocity_rad_per_sec',max(abs(vel(:))),...
    'max_acceleration_rad_per_sec2',max(abs(acc(:))),...
    'per_joint_max_velocity_rad_per_sec',max(abs(vel),[],1),...
    'per_joint_max_acceleration_rad_per_sec2',max(abs(acc),[],1));
if isempty(sp.velocity)
    [segVel,segAcc]=timingDerivatives(diff(Q,1,1),diff(sp.time));
    s.joint_motion.max_segment_velocity_rad_per_sec=max(abs(segVel(:)));
    s.joint_motion.max_segment_acceleration_rad_per_sec2=max(abs(segAcc(:)));
else
    s.joint_motion.max_segment_velocity_rad_per_sec=NaN;
    s.joint_motion.max_segment_acceleration_rad_per_sec2=NaN;
end
s.validation=struct('collision_pass',collisionPass,'collision_sample_count',collisionCount,...
    'first_collision_detail',collisionFirst,'joint_limits_pass',limitPass,...
    'sample_count',size(Qs,1));
s.sampleData=struct('name',sp.name,'time_sec',t,'q_rad',Qs,...
    'actual_tcp_xyz_m',Pa,'expected_tcp_xyz_m',Pe,'contour_error_mm',contourErr,...
    'time_sync_error_mm',syncErr,'orientation_error_deg',oriErr,...
    'segment',seg,'segment_u',u,'velocity_rad_per_sec',vel,'acceleration_rad_per_sec2',acc);
end

function [retimed,search]=chooseRetimedTimes(Q,P,baseT,m,scene)
dt0=diff(baseT); arc=vecnorm(diff(P,1,1),2,2);
if sum(arc)<=eps, geom=dt0; else, geom=baseT(end)*arc/sum(arc); end
% Keep a positive minimum and preserve total duration.
geom=max(geom,min(dt0)*0.25); geom=geom*baseT(end)/sum(geom);
alphaSet=0:0.1:1; rows=repmat(struct('alpha',0,'duration_sec',0,'sync_max_mm',NaN,'sync_rms_mm',NaN),numel(alphaSet),1);
for i=1:numel(alphaSet)
    a=alphaSet(i); dt=(1-a)*dt0+a*geom; tt=[0;cumsum(dt)];
    sp=struct('name','retime_search','time',tt,'velocity',[zeros(1,6);diff(Q,1,1)./diff(tt)],'description','search');
    metric=evaluateScheme(scene,Q,P,[],false,sp,m,struct('collisionCheck',false));
    rows(i).alpha=a; rows(i).duration_sec=tt(end);
    rows(i).sync_max_mm=metric.time_sync.max_position_error_mm;
    rows(i).sync_rms_mm=metric.time_sync.rms_position_error_mm;
end
[~,idx]=min([rows.sync_max_mm]); a=rows(idx).alpha;
dt=(1-a)*dt0+a*geom; retimed=[0;cumsum(dt)];
search=struct('base_segment_duration_sec',dt0(:).','geometry_segment_duration_sec',geom(:).',...
    'alphas',[rows.alpha],'sync_max_mm',[rows.sync_max_mm],...
    'sync_rms_mm',[rows.sync_rms_mm],'selected_alpha',a,...
    'selection','minimum pre-screened time-sync max error; collision/limits rechecked in final scheme');
end

function [scaled,search]=chooseAdaptiveTimeScaling(Q,P,R,hasR,baseT,m,cfg)
% Allocate each segment duration independently.  The q nodes never change.
% The objective penalizes velocity/acceleration violations and total time;
% fminsearch is used so no Optimization Toolbox is required.
D=diff(Q,1,1); n=size(D,1); baseDt=diff(baseT);
vLim=cfg.timeScalingMaxVelocityRadPerSec; aLim=cfg.timeScalingMaxAccelerationRadPerSec2;
dtFloor=max(max(abs(D),[],2)/vLim,cfg.adaptiveTimeScalingMinimumSegmentDurationSec);

% Segment features used to seed the search and recorded for auditability.
dqFeature=vecnorm(D,2,2); dqFeature=dqFeature/max(max(dqFeature),eps);
tcpStep=vecnorm(diff(P,1,1),2,2); tcpFeature=tcpStep/max(max(tcpStep),eps);
poseStep=zeros(n,1);
if hasR
    for i=1:n, poseStep(i)=rotationAngle(R(:,:,i),R(:,:,i+1)); end
end
poseFeature=poseStep/max(max(poseStep),eps);
curvature=zeros(n,1);
if n>1
    dP=diff(P,1,1); lens=vecnorm(dP,2,2); dirs=dP./max(lens,eps);
    turns=zeros(n-1,1);
    for i=1:n-1, turns(i)=acos(max(-1,min(1,dot(dirs(i,:),dirs(i+1,:))))); end
    curvature(1)=turns(1); curvature(end)=turns(end);
    if n>2, curvature(2:end-1)=0.5*(turns(1:end-1)+turns(2:end)); end
end
curvature=curvature/max(max(curvature),eps);
feature=1+0.20*dqFeature+0.20*tcpFeature+0.15*poseFeature+0.25*curvature;
seedDt=max(dtFloor,baseDt.*feature);
x0=log(max(seedDt-dtFloor,1e-6));
obj=@(x)adaptiveTimingObjective(x,dtFloor,D,vLim,aLim,cfg,x0);
opts=optimset('Display','off','MaxIter',cfg.adaptiveTimeScalingMaxIterations,...
    'MaxFunEvals',max(10000,10*cfg.adaptiveTimeScalingMaxIterations),...
    'TolX',1e-6,'TolFun',1e-6);
[xOpt,fval,exitflag,output]=fminsearch(obj,x0,opts);
dt=dtFloor+exp(xOpt); scaled=[0;cumsum(dt)];
[v,a]=timingDerivatives(D,dt);
rows=struct('segment',num2cell((1:n).'),'duration_sec',num2cell(dt),...
    'delta_q_max_rad',num2cell(max(abs(D),[],2)),'tcp_step_m',num2cell(tcpStep),...
    'pose_change_rad',num2cell(poseStep),...
    'tcp_curvature_feature',num2cell(curvature),'seed_duration_sec',num2cell(seedDt));
search=struct('method','adaptive independent segment duration via fminsearch',...
    'max_velocity_limit_rad_per_sec',vLim,'max_acceleration_limit_rad_per_sec2',aLim,...
    'max_duration_limit_sec',cfg.adaptiveTimeScalingMaxDurationSec,...
    'selected_duration_sec',scaled(end),'selected_max_velocity_rad_per_sec',max(abs(v(:))),...
    'selected_max_acceleration_rad_per_sec2',max(abs(a(:))),...
    'selected_dense_transition_acceleration_rad_per_sec2',max(abs(a(:)))*max(m-1,1),...
    'selected_meets_limits',max(abs(v(:)))<=vLim+eps && max(abs(a(:)))<=aLim+eps && scaled(end)<=cfg.adaptiveTimeScalingMaxDurationSec,...
    'objective_value',fval,'exitflag',exitflag,'iterations',output.iterations,...
    'segments',rows,'selected_time_from_start_sec',scaled(:).',...
    'selection','minimize acceleration and total duration subject to velocity, acceleration and duration checks');
end

function f=adaptiveTimingObjective(x,dtFloor,D,vLim,aLim,cfg,x0)
dt=dtFloor+exp(x); [v,a]=timingDerivatives(D,dt);
mv=max(abs(v(:))); ma=max(abs(a(:))); T=sum(dt);
vViolation=max(0,mv/vLim-1); aViolation=max(0,ma/aLim-1);
durationViolation=max(0,T-cfg.adaptiveTimeScalingMaxDurationSec)/max(cfg.adaptiveTimeScalingMaxDurationSec,eps);
f=ma/aLim+0.02*T+10*vViolation^2+10*aViolation^2+100*durationViolation^2+cfg.adaptiveTimeScalingLambda*sum(x.^2)+1e-5*sum((x-x0).^2);
end

function [v,a]=timingDerivatives(D,dt)
v=D./dt(:,ones(1,size(D,2)));
if size(v,1)<2, a=zeros(size(v)); return; end
den=(dt(1:end-1)+dt(2:end))/2;
a=diff(v,1,1)./den(:,ones(1,size(D,2)));
end

function a=rotationAngle(Ra,Rb)
c=(trace(Ra'*Rb)-1)/2; a=acos(max(-1,min(1,c)));
end

function V=shapeVelocities(Q,t)
n=size(Q,1); V=zeros(n,6); h=diff(t); D=diff(Q)./h;
if n==2, V(1,:)=D(1,:); V(2,:)=D(1,:); return; end
for j=1:6
    for i=2:n-1
        if D(i-1,j)*D(i,j)>0
            w1=2*h(i)+h(i-1); w2=h(i)+2*h(i-1);
            V(i,j)=(w1+w2)/(w1/D(i-1,j)+w2/D(i,j));
        else
            V(i,j)=0;
        end
    end
    V(1,j)=endpointSlope(h(1),h(2),D(1,j),D(2,j));
    V(n,j)=endpointSlope(h(end),h(end-1),D(end,j),D(end-1,j));
end
end

function v=endpointSlope(h1,h2,d1,d2)
v=((2*h1+h2)*d1-h1*d2)/(h1+h2);
if sign(v)~=sign(d1), v=0; elseif sign(d1)~=sign(d2)&&abs(v)>3*abs(d1), v=3*d1; end
end

function [t,Qs,seg,uOut]=sampleTimed(Q,t0,V,m)
n=size(Q,1); total=1+(n-1)*(m-1);
t=zeros(total,1); Qs=zeros(total,6); seg=zeros(total,1); uOut=zeros(total,1); k=1;
for e=1:n-1
    h=t0(e+1)-t0(e);
    for j=1:m
        if e>1&&j==1, continue; end
        u=(j-1)/(m-1); t(k)=t0(e)+u*h; seg(k)=e; uOut(k)=u;
        if isempty(V), Qs(k,:)=(1-u)*Q(e,:)+u*Q(e+1,:);
        else
            h00=2*u^3-3*u^2+1; h10=u^3-2*u^2+u; h01=-2*u^3+3*u^2; h11=u^3-u^2;
            Qs(k,:)=h00*Q(e,:)+h10*h*V(e,:)+h01*Q(e+1,:)+h11*h*V(e+1,:);
        end
        k=k+1;
    end
end
end

function [P,R]=sampleExpected(P0,R0,hasR,seg,u)
n=numel(seg); P=zeros(n,3);
if hasR
    R=zeros(3,3,n);
else
    R=[];
end
for i=1:n
    e=seg(i); z=u(i); P(i,:)=(1-z)*P0(e,:)+z*P0(e+1,:);
    if hasR, R(:,:,i)=slerp(R0(:,:,e),R0(:,:,e+1),z); end
end
end

function R=slerp(Ra,Rb,u)
qa=rotm2quat(Ra); qb=rotm2quat(Rb); if dot(qa,qb)<0, qb=-qb; end
d=max(-1,min(1,dot(qa,qb)));
if d>0.9995, q=(1-u)*qa+u*qb; q=q/norm(q);
else, th=acos(d); q=(sin((1-u)*th)*qa+sin(u*th)*qb)/sin(th); end
R=quat2rotm(q);
end

function [P,R]=fkPath(robot,Q,body)
n=size(Q,1); P=zeros(n,3); R=zeros(3,3,n);
for i=1:n, T=getTransform(robot,Q(i,:),body); P(i,:)=T(1:3,4).'; R(:,:,i)=T(1:3,1:3); end
end

function e=orientationErrors(Ra,Rb,hasR)
if ~hasR, e=NaN(size(Ra,3),1); return; end
e=zeros(size(Ra,3),1);
for i=1:size(Ra,3)
    c=(trace(Rb(:,:,i)'*Ra(:,:,i))-1)/2; e(i)=rad2deg(acos(max(-1,min(1,c))));
end
end

function d=polylineErrors(P,A)
n=size(P,1); d=zeros(size(A,1),1);
for i=1:size(A,1)
    best=Inf;
    for k=1:n-1
        v=P(k+1,:)-P(k,:); den=dot(v,v);
        if den<eps, z=0; else, z=max(0,min(1,dot(A(i,:)-P(k,:),v)/den)); end
        best=min(best,norm(A(i,:)-(P(k,:)+z*v)));
    end
    d(i)=1000*best;
end
end

function [v,a]=sampleDerivatives(Q,t)
dt=diff(t); v=diff(Q)./dt;
if isempty(v), v=zeros(1,6); end
if size(v,1)<2, a=zeros(size(v)); return; end
dtv=(dt(1:end-1)+dt(2:end))/2; a=diff(v)./dtv;
end

function lim=jointLimits(robot)
lim=zeros(6,2); j=0;
for i=1:numel(robot.Bodies)
    typ=robot.Bodies{i}.Joint.Type;
    if strcmp(typ,'revolute')||strcmp(typ,'continuous'), j=j+1; lim(j,:)=robot.Bodies{i}.Joint.PositionLimits; end
end
end

function [ok,detail]=collisionFree(scene,q)
env=check_scene_environment_collision(scene,q);
if isfield(scene,'selfCollisionSkipPairs')
    self=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
else
    self=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','Exhaustive','on');
end
ok=~any(env(:))&&~any(self(:)); detail='';
if ~ok, detail=sprintf('environment=%d self=%d',any(env(:)),any(self(:))); end
end

function s=metricSummary(x,label)
[m,idx]=max(x); s=struct('label',label,'max_error_mm',m,'rms_error_mm',sqrt(mean(x.^2)),...
    'max_sample',idx,'sample_count',numel(x),...
    'max_position_error_mm',m,'rms_position_error_mm',sqrt(mean(x.^2)));
end

function txt=velocitySource(V)
if isempty(V), txt='none (positions-only)'; elseif all(abs(V(1,:))<eps), txt='forward difference with first row zero'; else, txt='supplied'; end
end

function m=maxFinite(x)
if any(isfinite(x)), m=max(x,[],'omitnan'); else, m=NaN; end
end
function m=rmsFinite(x)
x=x(isfinite(x)); if isempty(x), m=NaN; else, m=sqrt(mean(x.^2)); end
end
function tt=timeOfMaxFinite(x,t)
if any(isfinite(x)), [~,i]=max(x,[],'omitnan'); tt=t(i); else, tt=NaN; end
end
function v=fieldOr(s,n,d)
if isstruct(s)&&isfield(s,n), v=s.(n); else, v=d; end
end
function [P,R,hasR]=poseSource(x)
if iscell(x)
    n=numel(x); P=zeros(n,3); R=zeros(3,3,n);
    for i=1:n, T=double(x{i}); P(i,:)=T(1:3,4).'; R(:,:,i)=T(1:3,1:3); end
    hasR=true; return
end
x=double(x);
if ismatrix(x)&&size(x,2)==3, P=x; R=[]; hasR=false; return; end
if ndims(x)==3&&size(x,1)==4&&size(x,2)==4
    n=size(x,3); P=zeros(n,3); R=zeros(3,3,n);
    for i=1:n, P(i,:)=x(1:3,4,i).'; R(:,:,i)=x(1:3,1:3,i); end
    hasR=true; return
end
error('Expected tcpTargetExecution/tcpPosePath must be cell 4x4, 4x4xN, or N-by-3.');
end

function writeSamplesCsv(path,samples)
f=fopen(path,'w'); if f<0,error('Cannot write %s',path);end
c=onCleanup(@()fclose(f));
fprintf(f,'scheme,sample,time_sec,segment,segment_u,q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad,actual_x_m,actual_y_m,actual_z_m,expected_x_m,expected_y_m,expected_z_m,contour_error_mm,time_sync_error_mm,orientation_error_deg\n');
for a=1:numel(samples)
    s=samples{a}; n=numel(s.time_sec);
    for i=1:n
        fprintf(f,['%s,%d,%.12g,%d,%.12g,','%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,',...
            '%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n'],...
            s.name,i,s.time_sec(i),s.segment(i),s.segment_u(i),s.q_rad(i,:),s.actual_tcp_xyz_m(i,:),...
            s.expected_tcp_xyz_m(i,:),s.contour_error_mm(i),s.time_sync_error_mm(i),s.orientation_error_deg(i));
    end
end
end
