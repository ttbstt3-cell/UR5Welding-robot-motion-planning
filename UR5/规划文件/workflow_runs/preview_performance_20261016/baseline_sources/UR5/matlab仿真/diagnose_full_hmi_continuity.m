function result=diagnose_full_hmi_continuity()
% Offline controller-facing timing experiment. No executable export.
here=fileparts(mfilename('fullpath')); root=fileparts(here); addpath(here);
addpath(genpath(fullfile(fileparts(root),'RRT')));
out=fullfile(root,'规划文件','workflow_runs','continuity_optimization_20261015');
rawFile=fullfile(root,'规划文件','workflow_runs','interface_branch_repair_20261015','replanned','full_raw_trajectory.mat');
raw=load(rawFile); ss=load(raw.sceneFile,'scene'); scene=ss.scene;
w=load(fullfile(out,'weld_candidate.mat'));
[Q,P,t,V,labels,maps,joins]=merge(raw.segments,w);
original=struct('Q',Q,'P',{P},'t',t,'maps',{maps},'joins',joins);
result=struct('executionReady',false,'robot_motion','NO','trials',{{}},'source_file',rawFile);
[baseline,~]=hmi_cubic_tracking_diagnostics(Q,P,t,V,scene,21,labels);
result.baseline=baseline; writeJson(fullfile(out,'full_progress.json'),result);
fprintf('FULL_BASELINE nodes=%d sync=%.6f ori=%.6f duration=%.6f\n',size(Q,1),baseline.position_max_mm,baseline.orientation_max_deg,t(end));
for trialNo=1:4
    % Increasing actual duration relaxes derivative constraints, not accuracy.
    if trialNo>1
        [Q,P,t,V,labels,maps,joins,inserted]=refineWorst(Q,P,t,V,labels,maps,joins,scene);
    else, inserted=0; end
    t=t*1.12; V=V/1.12;
    options=struct('maxIterations',4,'samplesPerSegment',7,'restEndpoints',true,'joins',joins);
    [V,op]=optimize_hmi_cubic_node_velocities(Q,P,t,V,scene,options);
    [quick,samples]=hmi_cubic_tracking_diagnostics(Q,P,t,V,scene,51,labels);
    tr=struct('trial',trialNo,'node_count',size(Q,1),'inserted_this_trial',inserted,'optimization',op,'quick',quick);
    result.trials{end+1}=tr;
    save(fullfile(out,sprintf('full_trial_%d.mat',trialNo)),'Q','P','t','V','labels','maps','joins','op','quick','samples','original','-v7.3');
    writeJson(fullfile(out,'full_progress.json'),result);
    fprintf('FULL_TRIAL %d nodes=%d pass=%d sync=%.6f ori=%.6f time=%.6f\n',trialNo,size(Q,1),op.pass,quick.position_max_mm,quick.orientation_max_deg,t(end));
    if op.pass
        assert(isequal(Q(maps{2},:),raw.segments{2}.qPath),'Original ACCP nodes changed');
        pp=P(maps{2}); assert(isequal(pp(:),raw.segments{2}.tcpPosePath(:)),'Original ACCP targets changed');
        save(fullfile(out,'full_candidate.mat'),'Q','P','t','V','labels','maps','joins','scene','original','rawFile','-v7.3');
        result.candidate_file=fullfile(out,'full_candidate.mat'); break;
    end
end
writeJson(fullfile(out,'full_progress.json'),result);
end
function [Q,P,t,V,labels,maps,joins]=merge(parts,w)
Q=[]; P={}; t=[]; V=[]; labels={}; maps=cell(1,3); joins=[];
for k=1:3
    a=parts{k}; tt=a.timeFromStart(:); vv=central(a.qPath,tt);
    if k==2, tt=w.trialT; vv=w.V; end
    n=size(a.qPath,1); start=size(Q,1);
    if k==1, idx=1:n; offset=-tt(1); maps{k}=1:n;
    else
        assert(max(abs(Q(end,:)-a.qPath(1,:)))<1e-10);
        idx=2:n; offset=t(end)-tt(1); maps{k}=[start,start+(1:n-1)]; joins(end+1)=start;
        if k==2, V(end,:)=vv(1,:); end
    end
    Q=[Q;a.qPath(idx,:)]; P=[P,reshape(a.tcpPosePath(idx),1,[])]; t=[t;tt(idx)+offset]; V=[V;vv(idx,:)]; %#ok<AGROW>
    labels=[labels,repmat({a.label},1,numel(idx))]; %#ok<AGROW>
end
V(1,:)=0; V(end,:)=0;
end
function V=central(Q,t)
h=diff(t); slope=diff(Q)./h; V=[slope(1,:);zeros(size(Q,1)-2,6);slope(end,:)];
for k=2:size(Q,1)-1, V(k,:)=(h(k)*slope(k-1,:)+h(k-1)*slope(k,:))/(h(k-1)+h(k)); end
end
function [Q2,P2,t2,V2,l2,maps2,joins2,inserted]=refineWorst(Q,P,t,V,labels,maps,joins,scene)
% Execution-node subdivision of the SAME numeric edge/reference; no IK or
% target changes. Every original node, especially all 47 weld targets, stays.
[~,s]=hmi_cubic_tracking_diagnostics(Q,P,t,V,scene,31,labels);
edges=unique(s.edge(s.positionError>0.40 | s.orientationError>0.40));
% Do not modify the 47-node welding segment in this experiment.
edges=setdiff(edges,maps{2}(1):maps{2}(end)-1);
edges=unique([edges(:);1;size(Q,1)-1]);
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
for k=1:numel(maps), assert(isequal(Q2(maps2{k},:),Q(maps{k},:))); end
V2=central(Q2,t2); V2(1,:)=0; V2(end,:)=0;
% Seed original weld velocities, not the free-space central-difference rule.
V2(oldMap(maps{2}),:)=V(maps{2},:); inserted=size(Q2,1)-size(Q,1);
end
function writeJson(path,value)
fid=fopen(path,'w'); assert(fid>0); c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(value,PrettyPrint=true),'char');
end
