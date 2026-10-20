function result=optimize_accp_hybrid_velocity(base,scene,options)
% Independent constrained smoothing experiment. No formal trajectory export.
% Q/t/poses are immutable; A/B references are objective terms, not averages.
if nargin<3, options=struct(); end
motionLimits=hmi_scene_motion_limits(scene);
cfg=struct('velocityLimit',motionLimits.velocityLimit,'accelerationLimit',motionLimits.accelerationLimit,...
    'accelerationSoftFraction',0.9,'maxSequentialIterations',3,'outputDirectory','',...
    'compareReferences',true);
for f=fieldnames(cfg).', if isfield(options,f{1}), cfg.(f{1})=options.(f{1}); end, end
assert(islogical(cfg.compareReferences) && isscalar(cfg.compareReferences),...
    'HYBRID:Mode','compareReferences must be a scalar logical');
assert(cfg.compareReferences || isempty(cfg.outputDirectory),...
    'HYBRID:ComparisonExport','Comparison plots require compareReferences=true');
Q=base.qPath; poses=base.tcpPosePath; t=base.timeFromStart(:); h=diff(t);
assert(size(Q,1)==47 && size(Q,2)==6 && all(isfinite(Q(:))) && all(h>0));
refs=base.velocities(2:4); names={'A_CENTER','B_QUINTIC_REFERENCE','C_ACCURACY','D_HYBRID'};
metrics=cell(1,4); audits=cell(1,4); profiles=refs;
acfg=struct('samplesPerSegment',101,'maxContourErrorMm',0.5,...
    'maxTimeSyncErrorMm',0.5,'maxOrientationErrorDeg',0.5,...
    'maxVelocityRadPerSec',cfg.velocityLimit,'maxAccelerationRadPerSec2',cfg.accelerationLimit);
for k=1:3
    metrics{k}=accp_cubic_derivative_metrics(Q,t,refs{k},[12 15]);
    if cfg.compareReferences || k==3
        audits{k}=localAudit(Q,poses,t,refs{k},scene,acfg,metrics{k});
    else
        audits{k}=struct('strategy',names{k},'status','NOT_RUN_COMPARISON_ONLY',...
            'complete',false,'executionReady',false,...
            'failureReason','Reference used in objective; comparison audit not requested');
    end
end
assert(audits{3}.complete,'C is not a valid initial snapshot');
attempts={}; VD=refs{3}; best=Inf; found=false;
% Prefer 90% acceleration cap. If infeasible or exact FK fails, use the
% original hard cap; this does NOT lower the hard acceptance requirements.
for fraction=[cfg.accelerationSoftFraction 1]
    seed=refs{3};
    for iteration=1:cfg.maxSequentialIterations
        fprintf('HYBRID cap_fraction=%.2f iteration=%d\n',fraction,iteration);
        [candidate,solver]=localSolve(Q,poses,t,refs,seed,scene,cfg,fraction);
        if isempty(candidate)
            attempts{end+1}=struct('fraction',fraction,'iteration',iteration,'solver',solver,...
                'accepted',false,'reason',solver.status); %#ok<AGROW>
            if iteration<cfg.maxSequentialIterations
                seed=refs{min(iteration,2)}; % A/B backup linearization seeds.
                continue;
            end
            break;
        end
        cm=accp_cubic_derivative_metrics(Q,t,candidate,[12 15]);
        ca=localAudit(Q,poses,t,candidate,scene,acfg,cm);
        score=cm.rms_segment_jerk^2+4*cm.rms_acceleration_jump^2;
        attempts{end+1}=struct('fraction',fraction,'iteration',iteration,'solver',solver,...
            'accepted',logical(ca.complete),'audit',ca,'metrics',cm,...
            'reason',localAuditReason(ca)); %#ok<AGROW>
        if ca.complete && score<best
            VD=candidate; best=score; audits{4}=ca; metrics{4}=cm; found=true;
        end
        seed=candidate;
        if ca.complete, break; end
    end
    if found, break; end
end
if ~found, audits{4}=audits{3}; metrics{4}=metrics{3}; end
profiles{4}=VD;
result=struct('schema_version','HYBRID_ADAPTIVE_VELOCITY_OFFLINE_v1',...
    'qPath',Q,'tcpPosePath',{poses},'timeFromStart',t,'velocities',{profiles},...
    'strategies',{names},'audits',{audits},'derivativeMetrics',{metrics},...
    'attempts',{attempts},'hybridCandidateFound',found,'executionReady',false,...
    'offlineOnly',true,'robot_motion','NO','limits',cfg,'calibrationBinding',base.calibrationBinding,...
    'acceleration90PercentTargetMet',found && metrics{4}.max_acceleration<=0.9*cfg.accelerationLimit,...
    'controllerInterpolationScope','ROS2_JTC_POSITIONS_VELOCITIES_CUBIC_NOT_UR_INTERNAL_SERVO',...
    'runtimeControllerParametersVerified',false,'hardwareInnerServoVerified',false,...
    'scaledTimeMappingPhysicallyVerified',false,...
    'physicalSafetyMarginValidated',false,'wireCollisionShortenMm',3,...
    'continuousCollisionProof',false,'localTimeExperimentPerformed',false,...
    'formalReplacement',false,'boundaryChecks',localBoundaries(scene,Q,t,profiles,base.calibrationBinding));
result.reference_comparison_enabled=cfg.compareReferences;
% D is optional improvement, NOT a validity gate for the already audited C.
% A stopped QP iterate is never accepted as a trajectory. Preserve C exactly
% and expose its non-optimal provenance for mandatory downstream validation.
if found, source='D_HYBRID'; status='VALIDATED_HYBRID_CANDIDATE';
else, source='C_ACCURACY_FALLBACK'; status='NON_OPTIMAL_VALIDATED_SEED'; end
result.candidate=struct('available',true,'source',source,'status',status,...
    'is_fallback',~found,'scope','WELD_SEGMENT_ONLY',...
    'requires_global_continuity',true,'requires_full_task_validation',true,...
    'executionReady',false);
result.hybridPreferredOffline=found && metrics{4}.rms_segment_jerk<metrics{3}.rms_segment_jerk &&...
    metrics{4}.max_acceleration_jump<metrics{3}.max_acceleration_jump &&...
    metrics{4}.max_acceleration<metrics{3}.max_acceleration;
assert(isequal(Q,base.qPath) && isequal(t,base.timeFromStart) && isequal(poses,base.tcpPosePath));
if ~isempty(cfg.outputDirectory), localSave(result,cfg.outputDirectory); end
end

function audit=localAudit(Q,poses,t,V,scene,cfg,m)
audit=validate_accp_controller_interpolation(Q,poses,t,V,scene,cfg);
lim=localLimits(scene.robot);
exact=all(m.position_min>=lim(:,1)') && all(m.position_max<=lim(:,2)') &&...
    m.max_velocity<=cfg.maxVelocityRadPerSec && m.max_acceleration<=cfg.maxAccelerationRadPerSec2;
audit.max_joint_velocity_rad_per_sec=m.max_velocity;
audit.max_joint_acceleration_rad_per_sec2=m.max_acceleration;
audit.analytic_derivative_and_position_extrema_pass=exact;
audit.complete=audit.complete && exact;
if ~exact, audit.failureReason=[audit.failureReason ' ANALYTIC_EXTREMUM_LIMIT_FAIL']; end
end

function [V,info]=localSolve(Q,poses,t,refs,seed,scene,cfg,fraction)
N=size(Q,1); K=6*N; h=diff(t); D=diff(Q); T=sum(h);
A=sparse(0,K); b=[]; L=[]; y=[]; aStart=cell(N-1,1); aEnd=aStart;
gStart=aStart; gEnd=aStart; limit=localLimits(scene.robot);
for e=1:N-1
    i=e+(0:5)*N; j=e+1+(0:5)*N; d=D(e,:)';
    S=sparse(6,K); S(:,i)=-4*eye(6)/h(e); S(:,j)=-2*eye(6)/h(e);
    E=sparse(6,K); E(:,i)=2*eye(6)/h(e); E(:,j)=4*eye(6)/h(e);
    gs=6*d/h(e)^2; ge=-gs; aStart{e}=S; aEnd{e}=E; gStart{e}=gs; gEnd{e}=ge;
    cap=cfg.accelerationLimit*fraction*0.99999;
    A=[A;S;-S;E;-E]; b=[b;cap-gs;cap+gs;cap-ge;cap+ge]; %#ok<AGROW>
    % Exact integral of constant segment jerk.
    J=sparse(6,K); J(:,i)=6*eye(6)/h(e)^2; J(:,j)=6*eye(6)/h(e)^2;
    gj=-12*d/h(e)^3; weight=sqrt(h(e)/(6*T));
    L=[L;weight*J]; y=[y;-weight*gj]; %#ok<AGROW>
    % Exact integrated acceleration energy using two-point Gauss quadrature.
    for u=[0.5-sqrt(3)/6,0.5+sqrt(3)/6]
        H=(1-u)*S+u*E; g=(1-u)*gs+u*ge;
        weight=sqrt(0.5*h(e)/(6*T)); L=[L;weight*H]; y=[y;-weight*g]; %#ok<AGROW>
    end
    for u=linspace(0,1,21)
        Hv=sparse(6,K); Hv(:,i)=eye(6)*(3*u^2-4*u+1); Hv(:,j)=eye(6)*(3*u^2-2*u);
        gv=(6*u-6*u^2)*d/h(e);
        A=[A;Hv;-Hv]; b=[b;cfg.velocityLimit*0.99998-gv;cfg.velocityLimit*0.99998+gv]; %#ok<AGROW>
        Hq=sparse(6,K); Hq(:,i)=eye(6)*h(e)*(u^3-2*u^2+u);
        Hq(:,j)=eye(6)*h(e)*(u^3-u^2);
        gq=Q(e,:)'+(3*u^2-2*u^3)*d; qq=gq+Hq*seed(:);
        A=[A;Hq;-Hq]; b=[b;limit(:,2)-gq;gq-limit(:,1)]; %#ok<AGROW>
        actual=getTransform(scene.robot,qq',scene.tcpBodyName);
        target=interpolate_tcp_pose_slerp(poses{e},poses{e+1},u);
        Jfk=geometricJacobian(scene.robot,qq',scene.tcpBodyName);
        aa=rotm2axang(actual(1:3,1:3)*target(1:3,1:3)');
        F=[1000*Jfk(4:6,:);rad2deg(Jfk(1:3,:))]*Hq;
        g=[1000*(actual(1:3,4)-target(1:3,4));rad2deg(aa(1:3)'*aa(4))]-F*seed(:);
        % Inscribed component box is stricter than norm <=0.5; never a
        % relaxed TCP gate. Nonlinear exact FK/collision audit remains final.
        bound=0.49/sqrt(3); A=[A;F;-F]; b=[b;bound-g;bound+g]; %#ok<AGROW>
    end
end
% Penalize acceleration jumps and unnecessary changes in velocity curvature.
for e=1:N-2
    H=aStart{e+1}-aEnd{e}; g=gStart{e+1}-gEnd{e};
    weight=sqrt(4/(6*(N-2))); L=[L;weight*H]; y=[y;-weight*g]; %#ok<AGROW>
    H=sparse(6,K); i=e+(0:5)*N; j=e+1+(0:5)*N; k=e+2+(0:5)*N;
    H(:,i)=eye(6)/h(e); H(:,j)=-eye(6)*(1/h(e)+1/h(e+1)); H(:,k)=eye(6)/h(e+1);
    weight=sqrt(0.1/(6*(N-2))); L=[L;weight*H]; y=[y;zeros(6,1)]; %#ok<AGROW>
end
% References are independent least-squares costs, not an averaged profile.
L=[L;sqrt(0.1/K)*eye(K);sqrt(0.02/K)*eye(K)];
y=[y;sqrt(0.1/K)*refs{1}(:);sqrt(0.02/K)*refs{2}(:)];
% Active-set factors in the 282-variable space, rather than forming a huge
% interior-point system for the dense cross-joint TCP constraint rows.
opts=optimoptions('quadprog','Display','off','Algorithm','active-set','MaxIterations',1000);
solverTimer=tic;
try
[x,f,flag,out]=quadprog(2*(L'*L+1e-9*eye(K)),-2*L'*y,A,b,[],[],...
    -cfg.velocityLimit*ones(K,1),cfg.velocityLimit*ones(K,1),seed(:),opts);
catch err
    V=[]; info=struct('status','SOLVER_ERROR','message',err.message,...
        'error_identifier',err.identifier,'algorithm','active-set',...
        'elapsed_sec',toc(solverTimer),'acceleration_cap_fraction',fraction);
    return;
end
info=struct('exitflag',flag,'objective',f,'iterations',out.iterations,'initialProfile','C_THEN_SEQUENTIAL',...
    'acceleration_cap_fraction',fraction,'message',out.message,...
    'algorithm','active-set','elapsed_sec',toc(solverTimer),...
    'status',hmi_quadprog_exit_status(flag),'constraint_generation',{{}});
if flag<=0 || isempty(x)
    if flag>0, info.status='EMPTY_SOLUTION'; end
    V=[]; return;
end
% Exact velocity extrema are enforced by iterative constraint generation.
for round=1:8
    V=reshape(x,N,6); mm=accp_cubic_derivative_metrics(Q,t,V);
    if mm.max_velocity<=cfg.velocityLimit, break; end
    added=0;
    for e=1:N-1
        bb=h(e)*V(e,:); cc=3*D(e,:)-2*bb-h(e)*V(e+1,:);
        dd=-2*D(e,:)+bb+h(e)*V(e+1,:);
        for j=1:6
            if abs(dd(j))<=eps, continue; end
            u=-cc(j)/(3*dd(j));
            if u<=0 || u>=1, continue; end
            vv=(bb(j)+2*cc(j)*u+3*dd(j)*u^2)/h(e);
            if abs(vv)<=cfg.velocityLimit*0.99999, continue; end
            H=sparse(1,K); H(e+(j-1)*N)=3*u^2-4*u+1; H(e+1+(j-1)*N)=3*u^2-2*u;
            g=(6*u-6*u^2)*D(e,j)/h(e);
            A=[A;H;-H]; b=[b;cfg.velocityLimit*0.99998-g;cfg.velocityLimit*0.99998+g]; %#ok<AGROW>
            added=added+1;
        end
    end
    if added==0, break; end
    solverTimer=tic;
    try
    [x,f,flag,out]=quadprog(2*(L'*L+1e-9*eye(K)),-2*L'*y,A,b,[],[],...
        -cfg.velocityLimit*ones(K,1),cfg.velocityLimit*ones(K,1),x,opts);
    catch err
        V=[]; info.status='SOLVER_ERROR'; info.message=err.message;
        info.error_identifier=err.identifier; return;
    end
    run=struct('round',round,'exitflag',flag,'objective',f,'iterations',out.iterations,...
        'message',out.message,'status',hmi_quadprog_exit_status(flag),'elapsed_sec',toc(solverTimer));
    info.constraint_generation{end+1}=run;
    if flag<=0 || isempty(x)
        V=[]; info.exitflag=flag; info.status=run.status; info.message=out.message;
        if flag>0, info.status='EMPTY_SOLUTION'; end
        return;
    end
end
V=reshape(x,N,6);
end

function reason=localAuditReason(audit)
if audit.complete, reason='EXACT_WELD_AUDIT_PASS'; else, reason='EXACT_WELD_AUDIT_FAILED'; end
end

function limit=localLimits(robot)
limit=[];
for k=1:numel(robot.Bodies)
    j=robot.Bodies{k}.Joint;
    if ~strcmp(j.Type,'fixed'), limit(end+1,:)=j.PositionLimits; end %#ok<AGROW>
end
end

function b=localBoundaries(~,Q,t,profiles,binding)
% No compatible verified approach/exit was supplied; never assume joins.
b=struct('status','BLOCKED_NO_CURRENT_BOUND_APPROACH_EXIT',...
    'position_continuity','NOT_VERIFIED','velocity_continuity','NOT_VERIFIED',...
    'acceleration_continuity','NOT_VERIFIED','calibrationBinding',binding,...
    'required_start_q_rad',Q(1,:),'required_end_q_rad',Q(end,:),...
    'note','Frozen/unbound old approach and exit files cannot certify current REAL seam');
b.evidence=struct('oldApproachReport','UR5/规划文件/online_realhome_to_preweld_v1_fastconnect_anytime_rrt_star_hmi_20261010_105205_168803_report.json',...
    'oldApproachBinding','MISSING','latestRealApproachReport',...
    'UR5/规划文件/online_realhome_to_preweld_v1_fastconnect_anytime_rrt_star_hmi_20261012_102457_109585_report.json',...
    'latestRealApproachStatus','PLAN_FAILED_REAL_SCENE_OR_STALE_GOAL',...
    'currentBoundExitProvided',false);
for k=1:numel(profiles)
    m=accp_cubic_derivative_metrics(Q,t,profiles{k});
    b.profiles(k)=struct('entry_velocity_rad_s',profiles{k}(1,:),...
        'exit_velocity_rad_s',profiles{k}(end,:),...
        'entry_acceleration_rad_s2',m.acceleration_start(1,:),...
        'exit_acceleration_rad_s2',m.acceleration_end(end,:));
end
end

function localSave(r,out)
if ~isfolder(out), mkdir(out); end
save(fullfile(out,'hybrid_velocity_diagnostic.mat'),'r','-v7.3');
p=rmfield(r,{'qPath','tcpPosePath','velocities'});
fid=fopen(fullfile(out,'report.json'),'w'); assert(fid>=0); c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(p,'PrettyPrint',true),'char'); clear c;
summary=cell(4,13);
for k=1:4
    a=r.audits{k}; m=r.derivativeMetrics{k};
    summary(k,:)={r.strategies{k},a.max_tcp_contour_error_mm,a.max_tcp_time_sync_error_mm,...
        a.max_tcp_orientation_error_deg,m.max_velocity,m.max_acceleration,m.rms_acceleration,...
        m.max_segment_jerk,m.rms_segment_jerk,m.max_acceleration_jump,a.duration_sec,...
        a.collision_pass,a.joint_limits_pass};
    jumps=array2table([r.timeFromStart(2:end-1),m.node_acceleration_jump],...
        'VariableNames',{'time_s','J1','J2','J3','J4','J5','J6'});
    writetable(jumps,fullfile(out,[r.strategies{k} '_acceleration_jumps.csv']));
end
tab=cell2table(summary,'VariableNames',{'strategy','contour_mm','sync_mm','orientation_deg',...
    'velocity_rad_s','acceleration_rad_s2','rms_acceleration_rad_s2','max_jerk_rad_s3','rms_jerk_rad_s3',...
    'max_acceleration_jump_rad_s2','duration_s','collision_pass','joint_limits_pass'});
tab.boundary_status=repmat(string(r.boundaryChecks.status),4,1);
tab.numerical_pass=cellfun(@(a)a.complete,r.audits)';
writetable(tab,fullfile(out,'comparison.csv'));
localPlots(r,out);
end

function localPlots(r,out)
colors=lines(4); curves=cell(1,4);
for k=1:4
    curves{k}=localCurve(r.qPath,r.timeFromStart,r.velocities{k});
    vars={'time_s','segment','u'};
    for prefix={'v','a','segment_jerk'}
        for j=1:6, vars{end+1}=sprintf('%s_J%d',prefix{1},j); end %#ok<AGROW>
    end
    writetable(array2table(curves{k},'VariableNames',vars),fullfile(out,[r.strategies{k} '_curves.csv']));
end
for kind=1:3
    titles={'Velocity (rad/s)','Acceleration (rad/s^2)','Segment jerk (rad/s^3)'};
    files={'velocity_comparison.png','acceleration_comparison.png','segment_jerk_comparison.png'};
    fig=figure('Visible','off','Position',[100 100 1200 900]);
    for j=1:6
        subplot(3,2,j); hold on;
        for k=1:4
            curve=curves{k}; plot(curve(:,1),curve(:,3+(kind-1)*6+j),'Color',colors(k,:));
        end
        title(sprintf('J%d %s',j,titles{kind})); xlabel('Time (s)'); grid on;
    end
    legend(r.strategies,'Interpreter','none','Location','best');
    exportgraphics(fig,fullfile(out,files{kind})); close(fig);
end
fig=figure('Visible','off','Position',[100 100 1200 850]);
for col=1:2
    joint=[3 6]; j=joint(col);
    for kind=1:3
        subplot(3,2,(kind-1)*2+col); hold on;
        for k=1:4, c=curves{k}; plot(c(:,1),c(:,3+(kind-1)*6+j),'Color',colors(k,:)); end
        xlim([12 15]); title(sprintf('J%d %s',j,titles{kind})); grid on; xlabel('Time (s)');
    end
end
legend(r.strategies,'Interpreter','none','Location','best');
exportgraphics(fig,fullfile(out,'J3_J6_focus_12_15s.png')); close(fig);
fig=figure('Visible','off','Position',[100 100 1200 900]);
for j=1:6
    subplot(3,2,j); hold on;
    for k=1:4, plot(r.timeFromStart(2:end-1),r.derivativeMetrics{k}.node_acceleration_jump(:,j),'Color',colors(k,:)); end
    title(sprintf('J%d node acceleration jump (rad/s^2)',j)); grid on; xlabel('Time (s)');
end
legend(r.strategies,'Interpreter','none','Location','best');
exportgraphics(fig,fullfile(out,'node_acceleration_jump_comparison.png')); close(fig);
end

function curve=localCurve(Q,t,V)
h=diff(t); curve=zeros(numel(h)*101,21); index=0;
for e=1:numel(h)
    d=Q(e+1,:)-Q(e,:); bb=h(e)*V(e,:); cc=3*d-2*bb-h(e)*V(e+1,:);
    dd=-2*d+bb+h(e)*V(e+1,:);
    for u=linspace(0,1,101)
        index=index+1;
        curve(index,:)=[t(e)+h(e)*u,e,u,(bb+2*cc*u+3*dd*u^2)/h(e),...
            (2*cc+6*dd*u)/h(e)^2,6*dd/h(e)^3];
    end
end
end
