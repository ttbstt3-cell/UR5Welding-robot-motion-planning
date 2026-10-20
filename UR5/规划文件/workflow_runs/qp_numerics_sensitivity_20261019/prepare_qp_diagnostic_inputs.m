function prepare_qp_diagnostic_inputs()
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT'))); addpath(fullfile(here,'copies'));
prior=fullfile(fileparts(here),'hybrid_fallback_recovery_20261019','full_prepare');
request=jsondecode(fileread(fullfile(prior,'request.json'))); raw=load(request.rawFile);
assert(strcmp(hmi_workflow_file_hash(request.rawFile),request.raw_sha256));
assert(strcmp(hmi_workflow_file_hash(raw.sceneFile),raw.metadata.scene_snapshot_sha256));
input=load(fullfile(prior,'velocity_profile_input_diagnostic.mat'),'base'); base=input.base;
ss=load(raw.sceneFile,'scene'); scene=ss.scene;
scene.hmiMotionLimits=hmi_load_robot_motion_limits(request.motion_limits_file,request.motion_limits_sha256);
save(fullfile(here,'scene_snapshot.mat'),'scene','request','-v7.3');
parts=raw.segments; parts{2}.timeFromStart=base.timeFromStart;
[Q,P,t,V,labels,maps,joins]=merge(parts); V(maps{2},:)=base.velocities{4}; V([1 end],:)=0;
original=struct('Q',Q,'P',{P},'t',t,'V',V,'labels',{labels},'maps',{maps},'joins',joins);
save(fullfile(here,'initial_input.mat'),'-struct','original','-v7.3');
freeze_hmi_continuity_qp_copy(Q,P,t,V,scene,struct('restEndpoints',true,'joins',joins,...
    'samplesPerSegment',7,'captureFile',fullfile(here,'initial_qp.mat')));
fprintf('INITIAL_QP_FROZEN nodes=%d\n',size(Q,1));
% Reconstruct the actual failed recovery exactly: every QP returned its seed.
old=jsondecode(fileread(fullfile(prior,'report.json')));
trials=old.velocity_optimization.refinement_trials;
if isstruct(trials), trials=num2cell(trials); end
assert(all(cellfun(@(x)~x.pass,trials)));
weld=maps{2}; steps={};
for refinement=1:3
    [r,s]=hmi_cubic_tracking_diagnostics(Q,P,t,V,scene,31,labels);
    edges=unique(s.edge(s.positionError>.40 | s.orientationError>.40));
    edges=setdiff(edges,weld(1):weld(end)-1); edges=unique([edges(:);1;size(Q,1)-1]);
    [Q,P,t,V,labels,maps,joins]=subdivide(Q,P,t,V,labels,maps,joins,edges,weld);
    weld=maps{2}; steps{end+1}=struct('before',r,'inserted_edges',edges,'node_count',size(Q,1)); %#ok<AGROW>
end
t=t*1.12; V=V/1.12; limits=hmi_scene_motion_limits(scene);
for iteration=1:4
    h=diff(t); dm=accp_cubic_derivative_metrics(Q,t,V);
    scale=max(1,sqrt(max(abs([dm.acceleration_start,dm.acceleration_end]),[],2)/(limits.accelerationLimit*.85)));
    for k=2:numel(scale), scale(k)=max(scale(k),scale(k-1)/1.3); end
    for k=numel(scale)-1:-1:1, scale(k)=max(scale(k),scale(k+1)/1.3); end
    t=[0;cumsum(h.*scale)]; V=V./[scale(1);max(scale(1:end-1),scale(2:end));scale(end)];
end
assert(abs(t(end)-old.velocity_optimization.final_tracking.duration_sec)<1e-9);
for k=1:3, assert(isequal(Q(maps{k},:),original.Q(original.maps{k},:))); end
save(fullfile(here,'last_input.mat'),'Q','P','t','V','labels','maps','joins','steps','-v7.3');
nodeErrors=zeros(size(Q,1),2);
for k=1:size(Q,1)
    T=getTransform(scene.robot,Q(k,:),scene.tcpBodyName); G=P{k};
    nodeErrors(k,:)=[1000*norm(T(1:3,4)-G(1:3,4)),...
        rad2deg(acos(max(-1,min(1,(trace(T(1:3,1:3)*G(1:3,1:3)')-1)/2))))];
end
[err,idx]=max(nodeErrors(:,1));
nodeAudit=struct('node_count',size(Q,1),'max_fixed_node_TCP_error_mm',err,'max_node_index',idx,...
    'max_node_label',labels{idx},'max_node_time',t(idx),'max_node_q_deg',rad2deg(Q(idx,:)),...
    'max_node_actual_pose',getTransform(scene.robot,Q(idx,:),scene.tcpBodyName),...
    'max_node_target_pose',P{idx},'nodes_above_0p5mm',find(nodeErrors(:,1)>.5),...
    'nodes_above_0p46mm',find(nodeErrors(:,1)>.46),'node_errors',nodeErrors,...
    'all_original_nodes_preserved',true,'executionReady',false,'robot_motion','NO');
writeJson(fullfile(here,'fixed_node_audit.json'),nodeAudit);
freeze_hmi_continuity_qp_copy(Q,P,t,V,scene,struct('restEndpoints',true,...
    'searchError',.46,'maxIterations',8,'captureFile',fullfile(here,'last_qp.mat')));
fprintf('LAST_QP_FROZEN nodes=%d fixed_node_max_mm=%.9f\n',size(Q,1),err);
end
function [Q,P,t,V,labels,maps,joins]=merge(parts)
Q=[]; P={}; t=[]; V=[]; labels={}; maps=cell(size(parts)); joins=[];
for k=1:numel(parts)
    a=parts{k}; n=size(a.qPath,1); start=size(Q,1);
    if k==1, idx=1:n; offset=-a.timeFromStart(1); maps{k}=1:n;
    else, idx=2:n; offset=t(end)-a.timeFromStart(1); maps{k}=[start,start+(1:n-1)]; joins(end+1)=start; end %#ok<AGROW>
    Q=[Q;a.qPath(idx,:)]; P=[P,reshape(a.tcpPosePath(idx),1,[])]; t=[t;a.timeFromStart(idx)+offset]; %#ok<AGROW>
    vv=central(a.qPath,a.timeFromStart); V=[V;vv(idx,:)]; labels=[labels,repmat({a.label},1,numel(idx))]; %#ok<AGROW>
end
end
function V=central(Q,t)
h=diff(t); slope=diff(Q)./h; V=[slope(1,:);zeros(size(Q,1)-2,6);slope(end,:)];
for k=2:size(Q,1)-1, V(k,:)=(h(k)*slope(k-1,:)+h(k-1)*slope(k,:))/(h(k-1)+h(k)); end
end
function [Q2,P2,t2,V2,l2,maps2,joins2]=subdivide(Q,P,t,V,labels,maps,joins,edges,weld)
Q2=Q(1,:); P2=P(1); t2=t(1); l2=labels(1); oldMap=ones(size(Q,1),1);
for e=1:size(Q,1)-1
    if ismember(e,edges), us=[.25 .5 .75 1]; else, us=1; end
    for u=us
        Q2(end+1,:)=(1-u)*Q(e,:)+u*Q(e+1,:); P2{end+1}=interpolate_tcp_pose_slerp(P{e},P{e+1},u); %#ok<AGROW>
        t2(end+1,1)=(1-u)*t(e)+u*t(e+1); l2{end+1}=labels{e+1}; %#ok<AGROW>
    end
    oldMap(e+1)=size(Q2,1);
end
maps2=cellfun(@(m)oldMap(m).',maps,'UniformOutput',false); joins2=oldMap(joins).';
V2=central(Q2,t2); V2(oldMap(weld),:)=V(weld,:); V2([1 end],:)=0;
end
function writeJson(path,value)
fid=fopen(path,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(value,'PrettyPrint',true),'char');
end
