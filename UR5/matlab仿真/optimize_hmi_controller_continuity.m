function [candidate,report]=optimize_hmi_controller_continuity(Q,P,t,V,scene,labels,maps,joins)
% Bounded controller-facing optimization, independent of RRT/ACCP/IK.
% Candidate is DIAGNOSTIC ONLY. External dense collision + version validation
% remains mandatory; this function never grants execution readiness.
P=reshape(P,1,[]); t=t(:); V([1 end],:)=0;
motionLimits=hmi_scene_motion_limits(scene);
originalQ=Q; originalP=P; originalMaps=maps; initialCount=size(Q,1);
report=struct('pass',false,'executionReady',false,'refinement_trials',{{}},...
    'timing_trials',{{}},'failure_reason','','collision_status','NOT_CHECKED');
weld=maps{find(cellfun(@(m)strcmp(labels{m(end)},'weld'),maps),1)};
if isempty(weld), error('CONTINUITY:MissingWeld','No welding segment map'); end
for refinement=0:3
    [V,op]=optimize_hmi_cubic_node_velocities(Q,P,t,V,scene,struct('joins',joins,...
        'restEndpoints',true,'samplesPerSegment',7));
    report.refinement_trials{end+1}=op;
    if op.pass, break; end
    if refinement==3, break; end
    [~,s]=hmi_cubic_tracking_diagnostics(Q,P,t,V,scene,31,labels);
    edges=unique(s.edge(s.positionError>.40 | s.orientationError>.40));
    edges=setdiff(edges,weld(1):weld(end)-1);
    edges=unique([edges(:);1;size(Q,1)-1]);
    [Q,P,t,V,labels,maps,joins]=subdivide(Q,P,t,V,labels,maps,joins,edges,weld);
    weld=maps{find(cellfun(@(m)strcmp(labels{m(end)},'weld'),maps),1)};
end
% Reserve modest timing headroom, then slow only analytically difficult
% neighborhoods. This is NOT a global acceleration-based multiplier.
t=t*1.12; V=V/1.12;
for iteration=1:4
    h=diff(t); dm=accp_cubic_derivative_metrics(Q,t,V);
    scale=max(1,sqrt(max(abs([dm.acceleration_start,dm.acceleration_end]),[],2)/(motionLimits.accelerationLimit*.85)));
    for k=2:numel(scale), scale(k)=max(scale(k),scale(k-1)/1.3); end
    for k=numel(scale)-1:-1:1, scale(k)=max(scale(k),scale(k+1)/1.3); end
    t=[0;cumsum(h.*scale)]; V=V./[scale(1);max(scale(1:end-1),scale(2:end));scale(end)];
    [V,op]=optimize_hmi_cubic_node_velocities(Q,P,t,V,scene,struct('restEndpoints',true,...
        'searchError',.46,'maxIterations',8));
    report.timing_trials{end+1}=op;
    if op.pass, break; end
end
if op.pass
    [V,c2]=optimize_hmi_cubic_node_velocities(Q,P,t,V,scene,struct('joins',joins,...
        'searchError',.46,'restEndpoints',true,'maxIterations',8));
    report.interface_c2_optimization=c2; report.pass=c2.pass;
    if ~c2.pass, report.failure_reason='INTERFACE_C2_NOT_FEASIBLE'; end
else
    report.failure_reason='CONTROLLER_FK_VELOCITY_TIMING_NOT_FEASIBLE';
end
for k=1:numel(maps)
    assert(isequal(Q(maps{k},:),originalQ(originalMaps{k},:)),'Original q node changed');
    a=P(maps{k}); b=originalP(originalMaps{k}); assert(isequal(a(:),b(:)),'Original Cartesian target changed');
end
report.original_q_and_tcp_targets_preserved=true;
report.inserted_execution_nodes=size(Q,1)-initialCount;
report.final_tracking=hmi_cubic_tracking_diagnostics(Q,P,t,V,scene,101,labels);
report.derivatives=report.final_tracking.derivatives;
report.requires_independent_physical_safety_validation=true;
candidate=struct('qPath',Q,'tcpPosePath',{P},'timeFromStart',t,'velocities',V,...
    'segmentLabel',{labels},'maps',{maps},'joins',joins,'executionReady',false,'diagnostic_only',true);
end
function [Q2,P2,t2,V2,l2,maps2,joins2]=subdivide(Q,P,t,V,labels,maps,joins,edges,weld)
Q2=Q(1,:); P2=P(1); t2=t(1); l2=labels(1); oldMap=ones(size(Q,1),1);
for e=1:size(Q,1)-1
    if ismember(e,edges), us=[.25 .5 .75 1]; else, us=1; end
    for u=us
        Q2(end+1,:)=(1-u)*Q(e,:)+u*Q(e+1,:); %#ok<AGROW>
        P2{end+1}=interpolate_tcp_pose_slerp(P{e},P{e+1},u); %#ok<AGROW>
        t2(end+1,1)=(1-u)*t(e)+u*t(e+1); l2{end+1}=labels{e+1}; %#ok<AGROW>
    end
    oldMap(e+1)=size(Q2,1);
end
maps2=cellfun(@(m)oldMap(m).',maps,'UniformOutput',false); joins2=oldMap(joins).';
h=diff(t2); slope=diff(Q2)./h; V2=[slope(1,:);zeros(size(Q2,1)-2,6);slope(end,:)];
for k=2:size(Q2,1)-1, V2(k,:)=(h(k)*slope(k-1,:)+h(k-1)*slope(k,:))/(h(k-1)+h(k)); end
V2(oldMap(weld),:)=V(weld,:); V2([1 end],:)=0;
end
