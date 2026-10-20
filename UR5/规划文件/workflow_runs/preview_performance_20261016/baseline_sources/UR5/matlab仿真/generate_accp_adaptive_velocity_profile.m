function result=generate_accp_adaptive_velocity_profile(Q,poses,t,scene,options)
% Offline node-velocity experiments. Nodes, timestamps and scene are immutable.
% Quintic construction is used ONLY to derive velocities; every candidate is
% audited as positions+velocities CUBIC Hermite, never as a quintic controller.
if nargin<5, options=struct(); end
cfg=struct('velocityLimit',deg2rad(5),'accelerationLimit',0.261799,'outputDirectory','');
for f=fieldnames(cfg).', if isfield(options,f{1}), cfg.(f{1})=options.(f{1}); end, end
t=t(:); N=size(Q,1); h=diff(t); D=diff(Q); slopes=D./h;
assert(size(Q,2)==6 && N==numel(t) && all(isfinite(Q(:))) && all(isfinite(t)) && all(h>0));
assert(iscell(poses) && numel(poses)==N);
V0=[zeros(1,6);slopes];
VA=zeros(N,6); VA(1,:)=slopes(1,:); VA(end,:)=slopes(end,:);
for i=2:N-1
    VA(i,:)=(h(i)*slopes(i-1,:)+h(i-1)*slopes(i,:))/(h(i-1)+h(i));
end
% Minimum-integrated-jerk C2 quintic spline. Optimize auxiliary node
% derivatives analytically; the acceleration values are not exported.
VB=localQuinticVelocities(Q,h);
% Optimize only velocities using a fixed-chord FK Jacobian surrogate.
% Cubic acceleration is affine: both endpoint constraints are exact.
[VC,solver]=localControllerVelocities(Q,poses,h,scene,VA,cfg);
names={'BASELINE_PREVIOUS_SEGMENT','A_CENTRAL_DIFFERENCE',...
    'B_MINIMUM_JERK_QUINTIC_DERIVED','C_NONZERO_ENTRY_CONTROLLER_AWARE'};
profiles={V0,VA,VB,VC}; audits=cell(1,4);
auditCfg=struct('samplesPerSegment',101,'maxVelocityRadPerSec',cfg.velocityLimit,...
    'maxAccelerationRadPerSec2',cfg.accelerationLimit,...
    'maxContourErrorMm',0.5,'maxTimeSyncErrorMm',0.5,'maxOrientationErrorDeg',0.5);
for k=1:4
    fprintf('VELOCITY_PROFILE audit=%s\n',names{k});
    a=validate_accp_controller_interpolation(Q,poses,t,profiles{k},scene,auditCfg);
    [vpeak,apeak]=localPeaks(Q,h,profiles{k});
    a.max_joint_velocity_rad_per_sec=vpeak; a.max_joint_acceleration_rad_per_sec2=apeak;
    a.analytic_derivative_peak_check=true;
    a.complete=a.complete && vpeak<=cfg.velocityLimit && apeak<=cfg.accelerationLimit;
    if vpeak>cfg.velocityLimit || apeak>cfg.accelerationLimit
        a.failureReason=[a.failureReason ' EXACT_DERIVATIVE_PEAK_LIMIT_FAILED'];
    end
    a.strategy=names{k}; audits{k}=a;
    fprintf('duration=%.6f sync=%.6f contour=%.6f orientation=%.6f vmax=%.9f amax=%.9f collision=%d pass=%d\n',...
        a.duration_sec,a.max_tcp_time_sync_error_mm,a.max_tcp_contour_error_mm,...
        a.max_tcp_orientation_error_deg,vpeak,apeak,a.collision_pass,a.complete);
end
result=struct('schema_version','ADAPTIVE_VELOCITY_PROFILE_v1','offlineOnly',true,...
    'executionReady',false,'robot_motion','NO','qPath',Q,'tcpPosePath',{poses},...
    'timeFromStart',t,'strategies',{names},'velocities',{profiles},'audits',{audits},...
    'solver',solver,'limits',cfg,'accelerationFieldExported',false,...
    'controllerMessageFields',{{'positions','velocities','time_from_start'}},...
    'nonzeroEntryRequiresSeparateApproachValidation',true,...
    'nonzeroExitRequiresSeparateExitValidation',true,...
    'physicalSafetyMarginValidated',false,'wireCollisionShortenMm',3,...
    'continuousCollisionProof',false);
if isfield(scene,'calibrationBinding'), result.calibrationBinding=scene.calibrationBinding; end
pass=find(cellfun(@(a)a.complete,audits(2:end)))+1;
result.numericalPassingStrategies=names(pass);
result.entryVelocitiesRadPerSec=cellfun(@(v)v(1,:),profiles,'UniformOutput',false);
result.exitVelocitiesRadPerSec=cellfun(@(v)v(end,:),profiles,'UniformOutput',false);
if ~isempty(cfg.outputDirectory)
    if ~isfolder(cfg.outputDirectory), mkdir(cfg.outputDirectory); end
    save(fullfile(cfg.outputDirectory,'velocity_profiles_diagnostic.mat'),'result','-v7.3');
    report=rmfield(result,{'qPath','tcpPosePath','velocities'});
    fid=fopen(fullfile(cfg.outputDirectory,'report.json'),'w'); assert(fid>=0);
    c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(report,'PrettyPrint',true),'char'); clear c;
    for k=1:4
        curve=localCurve(Q,t,profiles{k});
        curveTable=array2table(curve,'VariableNames',{'time_sec','J1_rad_s','J2_rad_s',...
            'J3_rad_s','J4_rad_s','J5_rad_s','J6_rad_s'});
        writetable(curveTable,fullfile(cfg.outputDirectory,[names{k} '_velocity_curve.csv']));
    end
    summary=table(string(names(:)),cellfun(@(a)a.duration_sec,audits)',...
        cellfun(@(a)a.max_tcp_time_sync_error_mm,audits)',...
        cellfun(@(a)a.max_tcp_contour_error_mm,audits)',...
        cellfun(@(a)a.max_tcp_orientation_error_deg,audits)',...
        cellfun(@(a)a.max_joint_velocity_rad_per_sec,audits)',...
        cellfun(@(a)a.max_joint_acceleration_rad_per_sec2,audits)',...
        cellfun(@(a)a.collision_pass,audits)',cellfun(@(a)a.complete,audits)',...
        'VariableNames',{'strategy','duration_s','sync_mm','contour_mm','orientation_deg',...
        'velocity_rad_s','acceleration_rad_s2','collision_pass','numerical_pass'});
    writetable(summary,fullfile(cfg.outputDirectory,'comparison.csv'));
    fig=figure('Visible','off','Position',[100 100 1200 900]);
    for j=1:6
        subplot(3,2,j); hold on;
        for k=1:4, curve=localCurve(Q,t,profiles{k}); plot(curve(:,1),curve(:,1+j)); end
        title(sprintf('J%d velocity',j)); xlabel('Time (s)'); ylabel('rad/s'); grid on;
    end
    legend({'Baseline','Central difference','Quintic-derived','Nonzero entry optimized'},'Location','best');
    exportgraphics(fig,fullfile(cfg.outputDirectory,'velocity_curves.png')); close(fig);
end
end

function V=localQuinticVelocities(Q,h)
N=size(Q,1); B=[]; E=[];
for e=1:N-1
    % Quintic coefficient solve in normalized u, q/v/a boundaries.
    M=[1 1 1;3 4 5;6 12 20];
    F=zeros(6,4); g=zeros(6,6); g(1,:)=Q(e,:);
    F(2,1)=h(e); F(3,2)=h(e)^2/2;
    F(4:6,:)=M\[-h(e) -h(e)^2/2 0 0;...
        -h(e) -h(e)^2 h(e) 0;0 -h(e)^2 0 h(e)^2];
    g(4:6,:)=M\[Q(e+1,:)-Q(e,:);zeros(2,6)];
    for u=linspace(0,1,21)
        z=[0 0 0 6 24*u 60*u^2]/h(e)^3*sqrt(h(e)/21);
        row=zeros(1,2*N); row([e,N+e,e+1,N+e+1])=z*F;
        B(end+1,:)=row; E(end+1,:)=z*g; %#ok<AGROW>
    end
end
% Rest-to-rest quintic baseline; C strategy separately permits nonzero entry.
fixed=[1,N,N+1,2*N]; free=setdiff(1:2*N,fixed);
X=zeros(2*N,6); X(free,:)=-B(:,free)\E; V=X(1:N,:);
end

function [V,info]=localControllerVelocities(Q,poses,h,scene,Vseed,cfg)
N=size(Q,1); K=6*N;
L=zeros(6*21*(N-1),K); y=zeros(size(L,1),1); il=0;
constraintCount=12*103*(N-1);
A=spalloc(constraintCount,K,2*constraintCount); b=zeros(constraintCount,1); ia=0;
for e=1:N-1
    d=Q(e+1,:)-Q(e,:); i0=e+(0:5)*N; i1=e+1+(0:5)*N;
    for u=linspace(0,1,21)
        H=zeros(6,K); H(:,i0)=eye(6)*h(e)*(u^3-2*u^2+u);
        H(:,i1)=eye(6)*h(e)*(u^3-u^2);
        q0=Q(e,:)+(3*u^2-2*u^3)*d;
        T=getTransform(scene.robot,q0,scene.tcpBodyName);
        G=interpolate_tcp_pose_slerp(poses{e},poses{e+1},u);
        J=geometricJacobian(scene.robot,q0,scene.tcpBodyName);
        rot=rotm2axang(T(1:3,1:3)*G(1:3,1:3)');
        rows=il+(1:6); il=il+6;
        L(rows,:)=[1000*J(4:6,:)*H/0.5;rad2deg(J(1:3,:))*H/0.5];
        y(rows)=[-1000*(T(1:3,4)-G(1:3,4))/0.5;-rad2deg(rot(1:3)'*rot(4))/0.5];
    end
    for u=[0 1]
        H=zeros(6,K); H(:,i0)=eye(6)*(6*u-4)/h(e);
        H(:,i1)=eye(6)*(6*u-2)/h(e);
        offset=(6-12*u)*d'/h(e)^2;
        rows=ia+(1:12); ia=ia+12; A(rows,:)=[H;-H];
        b(rows)=[cfg.accelerationLimit*0.99999-offset;cfg.accelerationLimit*0.99999+offset];
    end
    for u=linspace(0,1,101)
        H=zeros(6,K); H(:,i0)=eye(6)*(3*u^2-4*u+1); H(:,i1)=eye(6)*(3*u^2-2*u);
        offset=(6*u-6*u^2)*d'/h(e);
        rows=ia+(1:12); ia=ia+12; A(rows,:)=[H;-H];
        b(rows)=[cfg.velocityLimit*0.99999-offset;cfg.velocityLimit*0.99999+offset];
    end
end
opts=optimoptions('quadprog','Display','off','MaxIterations',500);
[x,f,flag,out]=quadprog(2*(L'*L+1e-8*eye(K)),-2*L'*y,A,b,[],[],...
    -cfg.velocityLimit*ones(K,1),cfg.velocityLimit*ones(K,1),Vseed(:),opts);
assert(flag>0 && ~isempty(x),'Velocity QP failed: %s',out.message);
% Add constraints at actual cubic velocity extrema, not only the sampling
% grid. The final audit independently checks these extrema analytically.
cuts=0;
for round=1:10
    V=reshape(x,N,6); [peak,~]=localPeaks(Q,h,V);
    if peak<=cfg.velocityLimit*0.99999, break; end
    added=0;
    for e=1:N-1
        d=Q(e+1,:)-Q(e,:); bb=h(e)*V(e,:);
        cc=3*d-2*bb-h(e)*V(e+1,:); dd=-2*d+bb+h(e)*V(e+1,:);
        for j=1:6
            if abs(dd(j))<=eps, continue; end
            u=-cc(j)/(3*dd(j));
            if u<=0 || u>=1, continue; end
            vv=(bb(j)+2*cc(j)*u+3*dd(j)*u^2)/h(e);
            if abs(vv)<=cfg.velocityLimit*0.99999, continue; end
            H=sparse(1,K); H(e+(j-1)*N)=3*u^2-4*u+1;
            H(e+1+(j-1)*N)=3*u^2-2*u;
            offset=(6*u-6*u^2)*d(j)/h(e);
            A=[A;H;-H]; b=[b;cfg.velocityLimit*0.99998-offset;cfg.velocityLimit*0.99998+offset]; %#ok<AGROW>
            added=added+1;
        end
    end
    if added==0, break; end
    cuts=cuts+added;
    [x,f,flag,out]=quadprog(2*(L'*L+1e-8*eye(K)),-2*L'*y,A,b,[],[],...
        -cfg.velocityLimit*ones(K,1),cfg.velocityLimit*ones(K,1),x,opts);
    assert(flag>0 && ~isempty(x),'Velocity extremum QP failed: %s',out.message);
end
V=reshape(x,N,6); info=struct('exitflag',flag,'iterations',out.iterations,'objective',f,...
    'analyticVelocityExtremumConstraintsAdded',cuts,...
    'method','FK_JACOBIAN_QUADRATIC_NODE_VELOCITY_OPTIMIZATION');
end

function [mv,ma]=localPeaks(Q,h,V)
mv=0; ma=0;
for e=1:numel(h)
    dQ=Q(e+1,:)-Q(e,:); bb=h(e)*V(e,:);
    cc=3*dQ-2*bb-h(e)*V(e+1,:); dd=-2*dQ+bb+h(e)*V(e+1,:);
    ma=max(ma,max(abs([2*cc,2*cc+6*dd]))/h(e)^2);
    mv=max(mv,max(abs([V(e,:),V(e+1,:)])));
    for j=1:6
        if abs(dd(j))>eps
            u=-cc(j)/(3*dd(j));
            if u>0 && u<1, mv=max(mv,abs(bb(j)+2*cc(j)*u+3*dd(j)*u^2)/h(e)); end
        end
    end
end
end

function curve=localCurve(Q,t,V)
curve=[];
for e=1:numel(t)-1
    h=t(e+1)-t(e); d=Q(e+1,:)-Q(e,:);
    for u=linspace(0,1,101)
        vel=(6*u-6*u^2)*d/h+(3*u^2-4*u+1)*V(e,:)+(3*u^2-2*u)*V(e+1,:);
        curve(end+1,:)=[t(e)+u*h vel]; %#ok<AGROW>
    end
end
end
