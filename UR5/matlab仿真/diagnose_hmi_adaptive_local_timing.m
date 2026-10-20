function result=diagnose_hmi_adaptive_local_timing()
here=fileparts(mfilename('fullpath')); root=fileparts(here); addpath(here);
addpath(genpath(fullfile(fileparts(root),'RRT')));
out=fullfile(root,'规划文件','workflow_runs','continuity_optimization_20261015');
d=load(fullfile(out,'full_trial_4.mat')); raw=load(fullfile(root,'规划文件','workflow_runs','interface_branch_repair_20261015','replanned','full_raw_trajectory.mat'));
ss=load(raw.sceneFile,'scene'); scene=ss.scene; Q=d.Q; P=d.P; maps=d.maps; joins=d.joins; labels=d.labels; original=d.original;
t=d.t/1.12^3; V=d.V*1.12^3;
result=struct('executionReady',false,'trials',{{}},'q_and_targets_unchanged',true);
for iteration=1:4
    h=diff(t); a=accp_cubic_derivative_metrics(Q,t,V);
    scale=max(1,sqrt(max(abs([a.acceleration_start,a.acceleration_end]),[],2)/(.261799*.85)));
    % Limit neighboring scale ratios, so locally required slowing tapers out.
    for k=2:numel(scale), scale(k)=max(scale(k),scale(k-1)/1.3); end
    for k=numel(scale)-1:-1:1, scale(k)=max(scale(k),scale(k+1)/1.3); end
    h=h.*scale; t=[0;cumsum(h)];
    nodeScale=[scale(1);max(scale(1:end-1),scale(2:end));scale(end)]; V=V./nodeScale;
    [V,op]=optimize_hmi_cubic_node_velocities(Q,P,t,V,scene,struct('joins',[],...
        'searchError',.46,'restEndpoints',true,'samplesPerSegment',13,'maxIterations',8));
    [quick,samples]=hmi_cubic_tracking_diagnostics(Q,P,t,V,scene,51,labels);
    result.trials{end+1}=struct('iteration',iteration,'optimization',op,'quick',quick,'duration_sec',t(end));
    fprintf('ADAPTIVE_LOCAL %d pass=%d duration=%.3f sync=%.6f ori=%.6f a=%.9f\n',iteration,op.pass,t(end),quick.position_max_mm,quick.orientation_max_deg,quick.derivatives.max_acceleration);
    save(fullfile(out,sprintf('adaptive_local_%d.mat',iteration)),'Q','P','t','V','maps','joins','labels','scene','original','op','quick','samples','-v7.3');
    fid=fopen(fullfile(out,'adaptive_local.json'),'w'); fwrite(fid,jsonencode(result,PrettyPrint=true),'char'); fclose(fid);
    if op.pass
        save(fullfile(out,'full_candidate_c1.mat'),'Q','P','t','V','maps','joins','labels','scene','original','-v7.3'); break;
    end
end
end
