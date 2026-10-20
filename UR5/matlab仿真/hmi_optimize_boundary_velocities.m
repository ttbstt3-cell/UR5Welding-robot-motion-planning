function [V,report]=hmi_optimize_boundary_velocities(Q,t,reference,weldIndices,weldVelocity,joins,boundaryNodes)
% Local task-boundary velocity adapter. Q/TCP/time and interior D speeds fixed.
if nargin<7, boundaryNodes=2; end
N=size(Q,1); K=N*6; h=diff(t); D=diff(Q); A=sparse(0,K); b=[];
L=sqrt(0.1/K)*speye(K); y=sqrt(0.1/K)*reference(:);
S=cell(N-1,1); E=S; gs=S; ge=S;
for e=1:N-1
    i=e+(0:5)*N; j=e+1+(0:5)*N;
    S{e}=sparse(6,K); E{e}=sparse(6,K);
    S{e}(:,i)=-4*eye(6)/h(e); S{e}(:,j)=-2*eye(6)/h(e);
    E{e}(:,i)=2*eye(6)/h(e); E{e}(:,j)=4*eye(6)/h(e);
    gs{e}=6*D(e,:)'/h(e)^2; ge{e}=-gs{e}; cap=0.261799*(1-1e-5);
    A=[A;S{e};-S{e};E{e};-E{e}]; b=[b;cap-gs{e};cap+gs{e};cap-ge{e};cap+ge{e}]; %#ok<AGROW>
    J=sparse(6,K); J(:,i)=6*eye(6)/h(e)^2; J(:,j)=6*eye(6)/h(e)^2;
    weight=sqrt(h(e)/(6*sum(h))); L=[L;weight*J]; y=[y;weight*12*D(e,:)'/h(e)^3]; %#ok<AGROW>
end
for e=1:N-2
    L=[L;2*(S{e+1}-E{e})]; y=[y;-2*(gs{e+1}-ge{e})]; %#ok<AGROW>
end
% Fixed weld profile and rest at the two RealHome endpoints.
interior=(boundaryNodes+1):(numel(weldIndices)-boundaryNodes);
fixed=[reshape(weldIndices(interior),[],1);1;N]; values=[weldVelocity(interior,:);zeros(2,6)];
Eq=sparse(0,K); eq=[];
for k=1:numel(fixed)
    row=sparse(6,K); row(:,fixed(k)+(0:5)*N)=eye(6);
    Eq=[Eq;row]; eq=[eq;values(k,:)']; %#ok<AGROW>
end
% C2 only at task interfaces. Interior ACCP Hermite jumps are reported.
for p=joins(:).'
    if p>1 && p<N
        Eq=[Eq;S{p}-E{p-1}]; eq=[eq;-(gs{p}-ge{p-1})]; %#ok<AGROW>
    end
end
options=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex','MaxIterations',400);
capV=deg2rad(5)*(1-1e-6); x=[]; flag=-1;
for iteration=1:8
    [x,~,flag]=quadprog(2*(L'*L+1e-9*speye(K)),-2*L'*y,A,b,Eq,eq,...
        -capV*ones(K,1),capV*ones(K,1),[],options);
    if flag<=0 || isempty(x), break; end
    V=reshape(x,N,6); m=accp_cubic_derivative_metrics(Q,t,V);
    if m.max_velocity<=capV, break; end
    bb=h.*V(1:end-1,:); cc=3*D-2*bb-h.*V(2:end,:); dd=-2*D+bb+h.*V(2:end,:);
    for e=1:N-1
        for j=1:6
            if abs(dd(e,j))<eps, continue; end
            u=-cc(e,j)/(3*dd(e,j)); if u<=0 || u>=1, continue; end
            row=sparse(1,K); row(e+(j-1)*N)=3*u^2-4*u+1; row(e+1+(j-1)*N)=3*u^2-2*u;
            c=(6*u-6*u^2)*D(e,j)/h(e);
            A=[A;row;-row]; b=[b;capV-c;capV+c]; %#ok<AGROW>
        end
    end
end
report=struct('pass',false,'exitflag',flag,'weld_interior_velocities_fixed',true,...
    'optimized_weld_boundary_node_count_per_side',boundaryNodes,...
    'failure_reason','Boundary velocity/acceleration continuity infeasible under fixed weld nodes/timestamps');
if flag<=0 || isempty(x), V=[]; return; end
V=reshape(x,N,6); m=accp_cubic_derivative_metrics(Q,t,V);
report.derivatives=m;
report.pass=m.max_velocity<=capV && m.max_acceleration<=0.261799 &&...
    max(abs(V(weldIndices(interior),:)-weldVelocity(interior,:)),[],'all')<1e-7;
if report.pass, report.failure_reason=''; end
end
