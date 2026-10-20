function info=hybrid_qp_readonly_probe(base,scene)
% Diagnostic copy of EXACT production matrix assembly; no acceptance/export.
% Only feasibility of these matrices is tested. Production files unchanged.
Q=base.qPath; poses=base.tcpPosePath; t=base.timeFromStart(:);
refs=base.velocities(2:4); seed=refs{3}; cfg=base.limits; fraction=0.9;
clock=tic;
N=size(Q,1); K=6*N; h=diff(t); D=diff(Q); T=sum(h);
A=sparse(0,K); b=[]; L=[]; y=[]; fkRows=[]; aStart=cell(N-1,1); aEnd=aStart;
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
        bound=0.49/sqrt(3); firstRow=size(A,1)+1; A=[A;F;-F]; b=[b;bound-g;bound+g]; fkRows=[fkRows,firstRow:size(A,1)]; %#ok<AGROW>
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

mask=true(size(b)); mask(fkRows)=false;
v=A*seed(:)-b; lo=-cfg.velocityLimit*ones(K,1); hi=-lo;
options=optimoptions('linprog','Display','none','MaxIterations',10000);
[feasible,~,allFlag,allInfo]=linprog(zeros(K,1),A,b,[],[],lo,hi,options);
[~,~,kinFlag,kinInfo]=linprog(zeros(K,1),A(mask,:),b(mask),[],[],lo,hi,options);
H=2*(L'*L+1e-9*eye(K));
info=struct('scope','DIAGNOSTIC_MATRIX_FEASIBILITY_ONLY_NOT_A_TRAJECTORY',...
 'production_source','optimize_accp_hybrid_velocity/localSolve exact assembly',...
 'QP_variables',K,'inequality_rows',numel(b),'TCP_rows',numel(fkRows),...
 'all_constraints_LP_exitflag',allFlag,'all_constraints_LP_message',allInfo.message,...
 'kinematics_only_LP_exitflag',kinFlag,'kinematics_only_LP_message',kinInfo.message,...
 'C_seed_max_violation',max([0;v]),'C_seed_kinematic_violation',max([0;v(mask)]),...
 'C_seed_TCP_box_violation',max([0;v(~mask)]),...
 'TCP_component_box_bound',0.49/sqrt(3),'hessian_reciprocal_condition',rcond(full(H)),...
 'elapsed_sec',toc(clock),'executionReady',false,'robot_motion','NO');
% Counterfactual DIAGNOSTIC ONLY: same matrix, objective, hard limits,
% active-set algorithm and 1000-iteration budget; change initial point only.
if allFlag>0
    qpOptions=optimoptions('quadprog','Display','off','Algorithm','active-set','MaxIterations',1000);
    trialClock=tic;
    [candidate,objective,flag,solver]=quadprog(H,-2*L'*y,A,b,[],[],lo,hi,feasible,qpOptions);
    trial=struct('scope','FEASIBLE_INITIALIZATION_COMPARISON_ONLY',...
        'exitflag',flag,'message',solver.message,'iterations',solver.iterations,...
        'objective',objective,'solver_elapsed_sec',toc(trialClock),'executionReady',false);
    if flag>0 && ~isempty(candidate)
        V=reshape(candidate,N,6); metrics=accp_cubic_derivative_metrics(Q,t,V);
        audit=validate_accp_controller_interpolation(Q,poses,t,V,scene,struct(...
            'samplesPerSegment',101,'maxContourErrorMm',0.5,'maxTimeSyncErrorMm',0.5,...
            'maxOrientationErrorDeg',0.5,'maxVelocityRadPerSec',cfg.velocityLimit,...
            'maxAccelerationRadPerSec2',cfg.accelerationLimit));
        trial.audit=audit;
        trial.analytic_extrema_pass=metrics.max_velocity<=cfg.velocityLimit &&...
            metrics.max_acceleration<=cfg.accelerationLimit &&...
            all(metrics.position_min>=limit(:,1).') && all(metrics.position_max<=limit(:,2).');
        trial.weld_only_pass=audit.complete && trial.analytic_extrema_pass;
        trial.full_task_validated=false;
    end
    info.feasible_initialization_trial=trial;
end
end
function limit=localLimits(robot)
limit=[];
for k=1:numel(robot.Bodies)
    j=robot.Bodies{k}.Joint;
    if ~strcmp(j.Type,'fixed'), limit(end+1,:)=j.PositionLimits; end %#ok<AGROW>
end
end


