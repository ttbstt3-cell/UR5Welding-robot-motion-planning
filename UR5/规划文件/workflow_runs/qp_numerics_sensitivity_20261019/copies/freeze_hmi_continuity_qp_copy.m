function [V,report]=freeze_hmi_continuity_qp_copy(Q,P,t,seed,scene,options)
% Sequential FK-linearized QP for controller-facing P+V cubic boundary data.
% Q and Cartesian targets are immutable. Exact audit is external/mandatory.
if nargin<6, options=struct(); end
motionLimits=hmi_scene_motion_limits(scene);
cfg=struct('maxIterations',5,'searchError',0.46,'samplesPerSegment',13,...
    'restEndpoints',false,'joins',[],'velocityLimit',motionLimits.velocityLimit,'accelerationLimit',motionLimits.accelerationLimit,...
    'feasibilityDiagnostics',false,'captureFile','');
for f=fieldnames(options).', if isfield(cfg,f{1}), cfg.(f{1})=options.(f{1}); end, end
assert(size(Q,2)==6 && isequal(size(Q),size(seed)) && numel(P)==size(Q,1));
assert(numel(t)==size(Q,1) && all(diff(t(:))>0) && all(isfinite([Q(:);seed(:);t(:)])));
t=t(:); n=size(Q,1); K=6*n; h=diff(t); D=diff(Q); x=seed(:); V=seed;
report=struct('pass',false,'executionReady',false,'iterations',{{}},'failure_reason','');
lim=scene.jointLimits; timer=tic;
extraSamples=cell(n-1,1);
for iteration=1:cfg.maxIterations
    Ac={}; bc={}; Lc={}; yc={}; Eq=sparse(0,K); eq=[]; fkBlocks=[];
    groups=[]; blockEdges=[]; blockUs=[];
    aStart=cell(n-1,1); aEnd=aStart; gStart=aStart; gEnd=aStart;
    for e=1:n-1
        i=e+(0:5)*n; j=e+1+(0:5)*n; d=D(e,:).';
        S=sparse(6,K); E=S; S(:,i)=-4*eye(6)/h(e); S(:,j)=-2*eye(6)/h(e);
        E(:,i)=2*eye(6)/h(e); E(:,j)=4*eye(6)/h(e);
        gs=6*d/h(e)^2; ge=-gs;
        aStart{e}=S; aEnd{e}=E; gStart{e}=gs; gEnd{e}=ge;
        Ac{end+1}=[S;-S;E;-E]; %#ok<AGROW>
        bc{end+1}=[cfg.accelerationLimit*(1-1e-5)-gs;cfg.accelerationLimit*(1-1e-5)+gs;...
            cfg.accelerationLimit*(1-1e-5)-ge;cfg.accelerationLimit*(1-1e-5)+ge];
        tag(1,e,NaN);
        J=sparse(6,K); J(:,i)=6*eye(6)/h(e)^2; J(:,j)=6*eye(6)/h(e)^2;
        w=sqrt(1e-4*h(e)/sum(h)); Lc{end+1}=w*J; yc{end+1}=w*12*d/h(e)^3; %#ok<AGROW>
        % Include current analytic velocity extrema along with a fixed grid.
        bb=h(e)*V(e,:); cc=3*D(e,:)-2*bb-h(e)*V(e+1,:);
        dd=-2*D(e,:)+bb+h(e)*V(e+1,:); rootsU=-cc./(3*dd);
        us=unique([linspace(0,1,cfg.samplesPerSegment),extraSamples{e},rootsU(isfinite(rootsU)&rootsU>0&rootsU<1)]);
        for u=us
            Hv=sparse(6,K); Hv(:,i)=(3*u^2-4*u+1)*eye(6); Hv(:,j)=(3*u^2-2*u)*eye(6);
            gv=(6*u-6*u^2)*d/h(e); cap=cfg.velocityLimit*(1-1e-6);
            Ac{end+1}=[Hv;-Hv]; bc{end+1}=[cap-gv;cap+gv]; %#ok<AGROW>
            tag(2,e,u);
            if u==0 || u==1, continue; end
            Hq=sparse(6,K); Hq(:,i)=h(e)*(u^3-2*u^2+u)*eye(6); Hq(:,j)=h(e)*(u^3-u^2)*eye(6);
            gq=Q(e,:).'+(3*u^2-2*u^3)*d; q=gq+Hq*x;
            Ac{end+1}=[Hq;-Hq]; bc{end+1}=[lim(:,2)-gq;gq-lim(:,1)]; %#ok<AGROW>
            tag(3,e,u);
            actual=getTransform(scene.robot,q.',scene.tcpBodyName);
            target=interpolate_tcp_pose_slerp(P{e},P{e+1},u);
            aa=rotm2axang(actual(1:3,1:3)*target(1:3,1:3)');
            err=[1000*(actual(1:3,4)-target(1:3,4));rad2deg(aa(1:3).'*aa(4))];
            G=geometricJacobian(scene.robot,q.',scene.tcpBodyName);
            F=sparse([1000*G(4:6,:);rad2deg(G(1:3,:))])*Hq; g=err-F*x;
            % Outer polyhedral norm approximation; add current radial cuts.
            % Final exact vector-norm FK/collision acceptance is NEVER waived.
            for part=1:2
                rows=(part-1)*3+(1:3); v=err(rows);
                directions=[eye(3);[1 1 0;1 -1 0;1 0 1;1 0 -1;0 1 1;0 1 -1]/sqrt(2);...
                    [1 1 1;1 1 -1;1 -1 1;-1 1 1]/sqrt(3)];
                if norm(v)>1e-9, directions=[directions;v.'/norm(v)]; end %#ok<AGROW>
                A=sparse(directions)*F(rows,:); offset=directions*g(rows);
                Ac{end+1}=[A;-A]; bc{end+1}=[cfg.searchError-offset;cfg.searchError+offset]; %#ok<AGROW>
                tag(3+part,e,u);
                fkBlocks(end+1)=numel(Ac); %#ok<AGROW>
            end
            weight=sqrt(1/(max(1,n-1)*(cfg.samplesPerSegment-2)));
            Lc{end+1}=weight*F/0.5; yc{end+1}=-weight*g/0.5; %#ok<AGROW>
        end
    end
    for e=1:n-2
        weight=sqrt(1e-3/max(1,n-2));
        Lc{end+1}=weight*(aStart{e+1}-aEnd{e}); yc{end+1}=-weight*(gStart{e+1}-gEnd{e}); %#ok<AGROW>
    end
    if cfg.restEndpoints
        Eq=sparse(12,K); Eq(1:6,1+(0:5)*n)=eye(6); Eq(7:12,n+(0:5)*n)=eye(6); eq=zeros(12,1);
    end
    for p=cfg.joins(:).'
        if p>1 && p<n
            Eq=[Eq;aStart{p}-aEnd{p-1}]; eq=[eq;-(gStart{p}-gEnd{p-1})]; %#ok<AGROW>
        end
    end
    L=[vertcat(Lc{:});sqrt(1e-3/K)*speye(K)]; y=[vertcat(yc{:});sqrt(1e-3/K)*seed(:)];
    A=vertcat(Ac{:}); b=vertcat(bc{:}); cap=cfg.velocityLimit*(1-1e-6);
    assert(~isempty(cfg.captureFile),'Diagnostic capture file required');
    lengths=cellfun(@(a)size(a,1),Ac); ends=cumsum(lengths); starts=[1,ends(1:end-1)+1];
    rowGroup=zeros(numel(b),1,'uint8'); rowEdge=zeros(numel(b),1,'uint32'); rowU=zeros(numel(b),1);
    for k=1:numel(Ac)
        rows=starts(k):ends(k); rowGroup(rows)=groups(k); rowEdge(rows)=blockEdges(k); rowU(rows)=blockUs(k);
    end
    problem=struct('H',2*(L'*L+1e-9*speye(K)),'f',-2*L'*y,...
        'A',A,'b',b,'Eq',Eq,'eq',eq,'lb',-cap*ones(K,1),'ub',cap*ones(K,1),...
        'x0',x,'Q',Q,'P',{P},'t',t,'seed',seed,'rowGroup',rowGroup,...
        'rowEdge',rowEdge,'rowU',rowU,'cfg',cfg,'executionReady',false,...
        'scope','DIAGNOSTIC_FROZEN_EXACT_FIRST_LINEARIZATION');
    save(cfg.captureFile,'problem','-v7.3');
    report.failure_reason='DIAGNOSTIC_CAPTURE_ONLY_NO_SOLVE'; report.elapsed_sec=toc(timer);
    return;
    opts=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex','MaxIterations',300,...
        'ConstraintTolerance',1e-8,'OptimalityTolerance',1e-8);
    [z,~,flag,info]=quadprog(2*(L'*L+1e-9*speye(K)),-2*L'*y,A,b,Eq,eq,...
        -cap*ones(K,1),cap*ones(K,1),[],opts);
    trial=struct('iteration',iteration,'exitflag',flag,'message',info.message);
    if flag<=0 || isempty(z)
        if cfg.feasibilityDiagnostics
            lengths=cellfun(@(a)size(a,1),Ac); ends=cumsum(lengths); starts=[1,ends(1:end-1)+1];
            mask=true(size(b)); for block=fkBlocks, mask(starts(block):ends(block))=false; end
            trial.seed_kinematic_violation=max([0;A(mask,:)*x-b(mask)]);
            trial.seed_fk_linear_violation=max([0;A(~mask,:)*x-b(~mask)]);
            lo=optimoptions('linprog','Display','none');
            [~,~,lf]=linprog(zeros(K,1),A(mask,:),b(mask),Eq,eq,-cap*ones(K,1),cap*ones(K,1),lo);
            trial.kinematic_feasibility_exitflag=lf;
            trial.diagnostic_only='Constraint subset for failure attribution; NEVER an acceptance candidate';
            fprintf('QP_FAILURE kin_feasible_flag=%d kin_violation=%.6g fk_violation=%.6g\n',lf,trial.seed_kinematic_violation,trial.seed_fk_linear_violation);
        end
        report.iterations{end+1}=trial; report.failure_reason='FK_CONSTRAINED_VELOCITY_QP_INFEASIBLE'; break;
    end
    x=z; V=reshape(x,n,6); [quick,dense]=hmi_cubic_tracking_diagnostics(Q,P,t,V,scene,51);
    % Exchange method: constrain observed between-grid maxima next iteration.
    bad=find(dense.positionError>cfg.searchError | dense.orientationError>cfg.searchError);
    for k=bad(:).', e=dense.edge(k); extraSamples{e}=unique([extraSamples{e},dense.u(k)]); end
    trial.position_mm=quick.position_max_mm; trial.orientation_deg=quick.orientation_max_deg;
    trial.max_velocity=quick.derivatives.max_velocity; trial.max_acceleration=quick.derivatives.max_acceleration;
    report.iterations{end+1}=trial;
    fprintf('CONTINUITY_VELOCITY iter=%d nodes=%d sync=%.6f ori=%.6f v=%.9f a=%.9f\n',...
        iteration,n,trial.position_mm,trial.orientation_deg,trial.max_velocity,trial.max_acceleration);
    if quick.position_max_mm<0.49 && quick.orientation_max_deg<0.49 &&...
            quick.derivatives.max_velocity<=cfg.velocityLimit && quick.derivatives.max_acceleration<=cfg.accelerationLimit
        report.pass=true; report.failure_reason=''; break;
    end
end
report.elapsed_sec=toc(timer); report.q_nodes_unchanged=true; report.tcp_targets_unchanged=true;
report.collision_status='NOT_CHECKED: requires independent dense audit';
    function tag(group,edge,u)
        groups(end+1)=group; blockEdges(end+1)=edge; blockUs(end+1)=u;
    end
end
