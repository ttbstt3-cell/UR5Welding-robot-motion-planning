function [c,r]=cp_parameterize(g,raw,scene)
% Sole timing/velocity owner. No QP cascade, geometry mutation or retry branch.
clock=tic; duration=0;
for k=1:3, a=raw.segments{k}; duration=duration+a.timeFromStart(end)-a.timeFromStart(1); end
pp=g.pp; dp=derivative(pp); tau=inverseProgress(g.knots);
originalTau=inverseProgress(g.originalS);
r=struct('pass',false,'trials',{{}},'executionReady',false,'failure_reason','');
for iteration=1:6
    s=progress(tau); Q=ppval(pp,s).';
    t=tau*duration; dS=30*tau.^2.*(1-tau).^2/duration;
    V=ppval(dp,s).'.*dS;
    [~,originalIndices]=ismember(originalTau,tau); assert(all(originalIndices>0));
    Q(originalIndices,:)=g.originalQ;
    maps=cellfun(@(m)originalIndices(m).',g.originalMaps,'UniformOutput',false);
    joins=originalIndices(g.originalJoins).';
    [~,~,P]=cp_pose_at_s(g.reference,s);
    P(originalIndices)=g.originalP;
    % Exact cubic compatibility at stationary ends and task joins. This
    % small analytic correction belongs to this module, not another planner.
    V([1 end],:)=0; h=diff(t); D=diff(Q);
    V(2,:)=3*D(1,:)/h(1); V(end-1,:)=3*D(end,:)/h(end);
    for j=joins
        a=h(j-1); b=h(j);
        V(j,:)=(6*D(j,:)/b^2+6*D(j-1,:)/a^2-2*V(j+1,:)/b-2*V(j-1,:)/a)/(4*(1/a+1/b));
    end
    ref=g.reference; ref.duration=duration;
    check=quick(Q,t,V,ref,scene,9);
    r.trials{end+1}=struct('iteration',iteration,'execution_nodes',numel(t),...
        'max_position_mm',check.position,'max_orientation_deg',check.orientation);
    fprintf('CP_TIME iter=%d nodes=%d pos=%.6f ori=%.6f\n',iteration,numel(t),check.position,check.orientation);
    if check.position<.42 && check.orientation<.42
        r.pass=true; break;
    end
    if iteration==6 || numel(t)>2400
        r.failure_reason='CONTROLLER_DISCRETIZATION_BUDGET'; break;
    end
    bad=unique(check.edges(check.pos>.40 | check.ang>.40));
    tau=unique([tau;(tau(bad)+tau(bad+1))/2]);
end
limits=hmi_scene_motion_limits(scene); dm=accp_cubic_derivative_metrics(Q,t,V);
scale=max([1,dm.max_velocity/(limits.velocityLimit*.98),sqrt(dm.max_acceleration/(limits.accelerationLimit*.98))]);
if isfield(scene,'hmiMotionLimits')
    bound=hmi_tcp_speed_upper_bound(scene,dm.max_velocity_per_joint);
    scale=max(scale,bound.upper_bound_m_s/(scene.hmiMotionLimits.normal_tcp_speed_m_s*.98));
end
t=t*scale; V=V/scale; ref.duration=duration*scale;
dm=accp_cubic_derivative_metrics(Q,t,V); r.derivatives=dm;
lim=scene.jointLimits;
r.analytic_limits_pass=dm.max_velocity<=limits.velocityLimit && dm.max_acceleration<=limits.accelerationLimit &&...
    all(dm.position_min>=lim(:,1).') && all(dm.position_max<=lim(:,2).');
r.interface_C2_pass=all(abs(dm.node_acceleration_jump(joins-1,:))<1e-6,'all');
r.pass=r.pass && r.analytic_limits_pass && r.interface_C2_pass;
if ~r.pass && isempty(r.failure_reason), r.failure_reason='ANALYTIC_LIMIT_OR_INTERFACE_C2_FAILED'; end
r.elapsed_sec=toc(clock); r.duration_sec=t(end); r.uniform_limit_scale=scale;
r.method='C2_GEOMETRIC_SPLINE_QUINTIC_PROGRESS_ANALYTIC_CUBIC_BOUNDARIES';
r.note='Nonconstant feed; no claim of unchanged weld speed, global C2, physical servo or execution approval';
edges=discretize(s,[-Inf;g.originalS(2:end-1);Inf]); labels=g.originalLabels(min(edges+1,numel(g.originalLabels)));
c=struct('qPath',Q,'tcpPosePath',{P},'timeFromStart',t,'velocities',V,'maps',{maps},'joins',joins,...
    'segmentLabel',{labels},'pathTimeReference',ref,'executionReady',false,'diagnostic_only',true);
assert(isequal(Q(maps{2},:),raw.segments{2}.qPath));
assert(isequal(reshape(P(maps{2}),1,[]),reshape(raw.segments{2}.tcpPosePath,1,[])));
end
function v=progress(t)
v=10*t.^3-15*t.^4+6*t.^5; v(1)=max(0,v(1)); v(end)=min(1,v(end));
end
function tau=inverseProgress(s)
lo=zeros(size(s)); hi=ones(size(s));
for k=1:60
    m=(lo+hi)/2; below=(10*m.^3-15*m.^4+6*m.^5)<s;
    lo(below)=m(below); hi(~below)=m(~below);
end
tau=(lo+hi)/2; tau(s==0)=0; tau(s==1)=1;
end
function d=derivative(p)
d=p; d.order=p.order-1; d.coefs=p.coefs(:,1:end-1).*(p.order-1:-1:1);
end
function r=quick(Q,t,V,ref,scene,m)
n=size(Q,1); ts=[]; states=[]; ee=[];
for e=1:n-1
    u=linspace(0,1,m).'; h=t(e+1)-t(e); D=Q(e+1,:)-Q(e,:);
    states=[states;Q(e,:)+(3*u.^2-2*u.^3)*D+h*(u.^3-2*u.^2+u)*V(e,:)+h*(u.^3-u.^2)*V(e+1,:)]; %#ok<AGROW>
    ts=[ts;t(e)+u*h]; ee=[ee;repmat(e,m,1)]; %#ok<AGROW>
end
[P,R]=cp_reference(ref,ts); pos=zeros(size(ts)); ang=pos;
for k=1:numel(ts)
    A=getTransform(scene.robot,states(k,:),scene.tcpBodyName);
    pos(k)=1000*norm(A(1:3,4).'-P(k,:));
    ang(k)=rad2deg(acos(max(-1,min(1,(trace(A(1:3,1:3)*R(:,:,k)')-1)/2))));
end
r=struct('position',max(pos),'orientation',max(ang),'edges',ee,'pos',pos,'ang',ang);
end
