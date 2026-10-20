function result=diagnose_hmi_local_time_allocation()
here=fileparts(mfilename('fullpath')); root=fileparts(here); addpath(here);
addpath(genpath(fullfile(fileparts(root),'RRT')));
out=fullfile(root,'规划文件','workflow_runs','continuity_optimization_20261015');
d=load(fullfile(out,'full_trial_4.mat')); raw=load(fullfile(root,'规划文件','workflow_runs','interface_branch_repair_20261015','replanned','full_raw_trajectory.mat'));
ss=load(raw.sceneFile,'scene'); scene=ss.scene; Q=d.Q; P=d.P; maps=d.maps; joins=d.joins; labels=d.labels; original=d.original;
result=struct('executionReady',false,'trials',{{}},'original_weld_q_and_targets_preserved',true);
for minimumDuration=[.25 .5 1]
    h=diff(d.t)/1.12^4; isWeld=false(size(h)); isWeld(maps{2}(1):maps{2}(end)-1)=true;
    h(~isWeld)=max(1.12*h(~isWeld),minimumDuration); t=[0;cumsum(h)];
    slope=diff(Q)./h; V=[slope(1,:);zeros(size(Q,1)-2,6);slope(end,:)];
    for k=2:size(Q,1)-1, V(k,:)=(h(k)*slope(k-1,:)+h(k-1)*slope(k,:))/(h(k-1)+h(k)); end
    V(maps{2},:)=d.V(maps{2},:)*1.12^4; V([1 end],:)=0;
    [V,op]=optimize_hmi_cubic_node_velocities(Q,P,t,V,scene,struct('joins',joins,...
        'searchError',0.49,'restEndpoints',true,'samplesPerSegment',9,'feasibilityDiagnostics',true));
    if ~op.pass
        [V,withoutC2]=optimize_hmi_cubic_node_velocities(Q,P,t,V,scene,struct('joins',[],...
            'searchError',0.49,'restEndpoints',true,'samplesPerSegment',9,'feasibilityDiagnostics',true));
        op.without_c2_diagnostic=withoutC2;
        if withoutC2.pass
            save(fullfile(out,'full_candidate_c1.mat'),'Q','P','t','V','maps','joins','labels','scene','original','-v7.3');
        end
    end
    result.trials{end+1}=struct('minimum_free_segment_sec',minimumDuration,'duration_sec',t(end),'optimization',op);
    fprintf('LOCAL_TIME minimum=%.2f duration=%.6f pass=%d\n',minimumDuration,t(end),op.pass);
    if op.pass
        save(fullfile(out,'full_candidate.mat'),'Q','P','t','V','maps','joins','labels','scene','original','-v7.3'); break;
    end
end
fid=fopen(fullfile(out,'local_time_diagnosis.json'),'w'); c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(result,PrettyPrint=true),'char');
end
