function report = verify_time_parameterized_trajectory(trajectoryFile, expectedSource, outputFile, options)
%VERIFY_TIME_PARAMETERIZED_TRAJECTORY Offline diagnostic for timing-induced TCP error.
% A: direct FK of refined qPath waypoints. B: controller-equivalent timed FK.
% This function is read-only: it does not change ACCP/TCP/collision models or
% send a ROS/action goal. expectedSource must contain one Cartesian waypoint
% per qPath row as tcpPosePath/tcpTargetExecution (4x4 poses) or tcpPath
% (N-by-3 metres). Positions-only sources report orientation as unavailable.

if nargin < 1 || isempty(trajectoryFile), error('trajectoryFile is required.'); end
if nargin < 2, expectedSource = []; end
if nargin < 3 || isempty(outputFile)
    [d,n,~]=fileparts(char(string(trajectoryFile)));
    outputFile=fullfile(d,[n '_time_tcp_diagnostic.json']);
end
if nargin < 4 || isempty(options), options=struct(); end
cfg=localOptions(options);
T=localLoad(trajectoryFile);
[Q,t,V]=localTrajectory(T);
[P0,R0,hasR,expectedName]=localExpected(expectedSource,T,size(Q,1));
if isempty(cfg.scene)
    if isfield(T,'scene'), cfg.scene=T.scene; end
    if isempty(cfg.scene) && isfield(T,'result') && isstruct(T.result) && isfield(T.result,'scene')
        cfg.scene=T.result.scene;
    end
end
if isempty(cfg.scene), error('Pass options.scene or include scene/result.scene in trajectory MAT.'); end
scene=cfg.scene; robot=fieldOr(scene,'robot',[]);
if isempty(robot), error('Scene has no robot field.'); end
body=fieldOr(scene,'tcpBodyName','tool0');

% A: direct refined/exported waypoint FK.
[A,RA]=fkPath(robot,Q,body);
eA=1000*sqrt(sum((A-P0).^2,2));
oA=oriErr(RA,R0,hasR);
direct=summary(eA,oA,t,'DIRECT_REFINED_QPATH');

% B: reconstruct controller sampling only for diagnosis.
[ts,Qs,seg,u]=sampleTimed(Q,t,V,cfg.samplesPerSegment);
[Ps,Rs]=sampleExpected(P0,R0,hasR,seg,u);
[B,RB]=fkPath(robot,Qs,body);
eB=1000*sqrt(sum((B-Ps).^2,2));
oB=oriErr(RB,Rs,hasR);
timed=summary(eB,oB,ts,'TIME_PARAMETERIZED_TRAJECTORY');
if isempty(V), timed.interpolation='linear joint interpolation (no velocities exported)';
else, timed.interpolation='controller-equivalent cubic Hermite (positions+velocities)'; end
timed.samples_per_segment=cfg.samplesPerSegment;

[outDir,outName,~]=fileparts(char(string(outputFile)));
if isempty(outDir), outDir=pwd; end
if ~isfolder(outDir), mkdir(outDir); end
jsonPath=fullfile(outDir,[outName '.json']);
csvPath=fullfile(outDir,[outName '_samples.csv']);
matPath=fullfile(outDir,[outName '_samples.mat']);
writeCsv(csvPath,t,Q,A,P0,eA,oA,ts,Qs,B,Ps,eB,oB,seg,u);
directSample=struct('time_sec',t,'q_rad',Q,'actual_tcp_xyz_m',A,'expected_tcp_xyz_m',P0,...
    'position_error_mm',eA,'orientation_error_deg',oA);
timedSample=struct('time_sec',ts,'q_rad',Qs,'actual_tcp_xyz_m',B,'expected_tcp_xyz_m',Ps,...
    'position_error_mm',eB,'orientation_error_deg',oB,'segment',seg,'segment_u',u);
save(matPath,'directSample','timedSample','-v7');

report=struct();
report.schema_version='TIME_PARAMETERIZED_TCP_DIAGNOSTIC_v1';
report.created=char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
report.robot_motion='NO';
report.trajectory_file=char(string(trajectoryFile));
report.expected_source=expectedName;
report.coordinate_units=struct('joint','rad','tcp','m','error','mm','orientation','deg');
report.waypoint_count=size(Q,1); report.duration_sec=t(end);
report.velocity_field_present=~isempty(V); report.orientation_available=hasR;
report.direct_refined_qPath=direct; report.time_parameterized_trajectory=timed;
report.difference=struct('max_position_error_increase_mm',timed.max_position_error_mm-direct.max_position_error_mm,...
    'rms_position_error_increase_mm',timed.rms_position_error_mm-direct.rms_position_error_mm,...
    'max_orientation_error_increase_deg',finiteDiff(timed.max_orientation_error_deg,direct.max_orientation_error_deg),...
    'interpretation',interpret(direct,timed));
report.artifacts=struct('summary_json',jsonPath,'samples_csv',csvPath,'samples_mat',matPath);
report.note='Diagnostic only: timestamps/qPath unchanged; no FollowJointTrajectory goal sent.';
fid=fopen(jsonPath,'w'); if fid<0, error('Cannot write %s',jsonPath); end
c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(report,'PrettyPrint',true),'char');
fprintf('Time trajectory diagnostic: %s\n',jsonPath);
fprintf('A direct refined qPath: max %.6f mm, RMS %.6f mm\n',direct.max_position_error_mm,direct.rms_position_error_mm);
fprintf('B time-parameterized: max %.6f mm at %.6f s, RMS %.6f mm\n',...
    timed.max_position_error_mm,timed.max_position_error_time_sec,timed.rms_position_error_mm);
if hasR, fprintf('B maximum orientation error: %.6f deg at %.6f s\n',timed.max_orientation_error_deg,timed.max_orientation_error_time_sec);
else, fprintf('Orientation error: unavailable (positions-only expected source)\n'); end
end

function cfg=localOptions(x)
cfg=struct('samplesPerSegment',101,'scene',[]);
if ~isstruct(x), error('options must be a struct.'); end
if isfield(x,'samplesPerSegment') && ~isempty(x.samplesPerSegment), cfg.samplesPerSegment=round(double(x.samplesPerSegment)); end
if cfg.samplesPerSegment<2 || ~isfinite(cfg.samplesPerSegment), error('samplesPerSegment must be an integer >=2.'); end
if isfield(x,'scene'), cfg.scene=x.scene; end
end

function s=localLoad(x)
if isstruct(x), s=x; return; end
p=char(string(x)); if ~isfile(p), error('MAT file does not exist: %s',p); end
s=load(p);
if isfield(s,'result') && isstruct(s.result) && isfield(s.result,'qPath')
    s=s.result;
end
end

function [Q,t,V]=localTrajectory(S)
Q=fieldOr(S,'qPath',fieldOr(S,'positions',[]));
if isempty(Q) || ~isnumeric(Q) || size(Q,2)~=6, error('Trajectory qPath/positions must be N-by-6.'); end
Q=double(Q); if any(~isfinite(Q(:))), error('qPath contains NaN/Inf.'); end
t=fieldOr(S,'timeFromStart',fieldOr(S,'time_from_start',[]));
if isempty(t) || numel(t)~=size(Q,1), error('timeFromStart must have one value per qPath row.'); end
t=double(t(:)); if any(~isfinite(t)) || t(1)<0 || any(diff(t)<=0), error('timeFromStart is not finite/strictly increasing.'); end
V=fieldOr(S,'velocities',[]);
if ~isempty(V)
    V=double(V); if any(size(V)~=size(Q)) || any(~isfinite(V(:))), error('velocities must be finite N-by-6.'); end
end
end

function [P,R,hasR,name]=localExpected(source,T,n)
if isempty(source), source=T; end
if ~(isstruct(source)||iscell(source)||isnumeric(source)), source=localLoad(source); end
if isstruct(source) && isfield(source,'result') && isstruct(source.result)
    source=source.result;
end
if isstruct(source)
    x=fieldOr(source,'tcpPosePath',[]);
    if isempty(x), x=fieldOr(source,'tcpTargetExecution',[]); end
    if isempty(x), x=fieldOr(source,'expectedTcpPosePath',[]); end
    if ~isempty(x), name='tcpPosePath/tcpTargetExecution';
    else
        x=fieldOr(source,'tcpPath',fieldOr(source,'expectedTcpPath',[]));
        if isempty(x), error('Expected tcpPosePath/tcpTargetExecution/tcpPath not found.'); end
        name='tcpPath (position-only)';
    end
else
    x=source; name='explicit expected pose source';
end
[P,R,hasR]=poseSource(x);
if size(P,1)~=n, error('Expected Cartesian points=%d, qPath points=%d.',size(P,1),n); end
end

function [P,R,hasR]=poseSource(x)
R=[]; 
if iscell(x)
    n=numel(x); P=zeros(n,3); R=zeros(3,3,n);
    for i=1:n
        Ti=double(x{i}); if ~isequal(size(Ti),[4 4])||any(~isfinite(Ti(:))), error('Invalid expected 4x4 pose.'); end
        P(i,:)=Ti(1:3,4).'; R(:,:,i)=Ti(1:3,1:3);
    end
    hasR=true; return
end
x=double(x);
if ismatrix(x)&&size(x,2)==3, P=x; hasR=false; return; end
if ndims(x)==3&&size(x,1)==4&&size(x,2)==4
    n=size(x,3); P=zeros(n,3); R=zeros(3,3,n);
    for i=1:n, P(i,:)=x(1:3,4,i).'; R(:,:,i)=x(1:3,1:3,i); end
    hasR=true; return
end
error('Expected Cartesian source must be cell 4x4 poses, 4x4xN, or N-by-3 metres.');
end

function [P,R]=fkPath(robot,Q,body)
n=size(Q,1); P=zeros(n,3); R=zeros(3,3,n);
for i=1:n
    Ti=getTransform(robot,Q(i,:),body); P(i,:)=Ti(1:3,4).'; R(:,:,i)=Ti(1:3,1:3);
end
end

function [ts,Qs,seg,uOut]=sampleTimed(Q,t,V,m)
n=size(Q,1); total=1+(n-1)*(m-1);
ts=zeros(total,1); Qs=zeros(total,6); seg=zeros(total,1); uOut=zeros(total,1); k=1;
for e=1:n-1
    h=t(e+1)-t(e);
    for j=1:m
        if e>1&&j==1, continue; end
        u=(j-1)/(m-1); ts(k)=t(e)+u*h; seg(k)=e; uOut(k)=u;
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

function e=oriErr(Ra,Rb,hasR)
if ~hasR, e=NaN(size(Ra,3),1); return; end
e=zeros(size(Ra,3),1);
for i=1:size(Ra,3)
    c=(trace(Rb(:,:,i)'*Ra(:,:,i))-1)/2;
    e(i)=rad2deg(acos(max(-1,min(1,c))));
end
end

function s=summary(ep,eo,t,label)
[m,idx]=max(ep);
s=struct('label',label,'sample_count',numel(ep),'max_position_error_mm',m,...
    'rms_position_error_mm',sqrt(mean(ep.^2)),'max_position_error_sample',idx,...
    'max_position_error_time_sec',t(idx),'orientation_available',any(isfinite(eo)));
if any(isfinite(eo))
    [m,idx]=max(eo,[],'omitnan'); s.max_orientation_error_deg=m;
    s.max_orientation_error_sample=idx; s.max_orientation_error_time_sec=t(idx);
    s.rms_orientation_error_deg=sqrt(mean(eo(isfinite(eo)).^2));
else
    s.max_orientation_error_deg=NaN; s.max_orientation_error_sample=NaN;
    s.max_orientation_error_time_sec=NaN; s.rms_orientation_error_deg=NaN;
end
end

function txt=interpret(a,b)
if b.max_position_error_mm>a.max_position_error_mm+1e-9
    txt='Additional error appears between waypoint FK and time-parameterized interpolation; direct refined qPath FK is the baseline.';
else
    txt='No additional position error at this sampling resolution.';
end
end

function writeCsv(path,tA,QA,AA,EA,errA,oriA,tB,QB,AB,EB,errB,oriB,seg,u)
f=fopen(path,'w'); if f<0, error('Cannot write %s',path); end
c=onCleanup(@()fclose(f));
fprintf(f,['mode,sample,time_sec,segment,segment_u,q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad,',...
    'actual_x_m,actual_y_m,actual_z_m,expected_x_m,expected_y_m,expected_z_m,position_error_mm,orientation_error_deg\n']);
for i=1:numel(tA)
    fprintf(f,['DIRECT_REFINED_QPATH,%d,%.12g,%d,%.12g,','%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,',...
        '%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n'],i,tA(i),i,0,QA(i,:),AA(i,:),EA(i,:),errA(i),oriA(i));
end
for i=1:numel(tB)
    fprintf(f,['TIME_PARAMETERIZED_TRAJECTORY,%d,%.12g,%d,%.12g,','%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,',...
        '%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n'],i,tB(i),seg(i),u(i),QB(i,:),AB(i,:),EB(i,:),errB(i),oriB(i));
end
end

function v=fieldOr(s,name,default)
if isstruct(s)&&isfield(s,name), v=s.(name); else, v=default; end
end
function d=finiteDiff(a,b)
if isfinite(a)&&isfinite(b), d=a-b; else, d=NaN; end
end
