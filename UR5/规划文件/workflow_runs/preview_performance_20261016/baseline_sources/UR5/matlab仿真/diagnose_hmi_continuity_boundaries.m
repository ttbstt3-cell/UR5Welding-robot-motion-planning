function result=diagnose_hmi_continuity_boundaries()
here=fileparts(mfilename('fullpath')); root=fileparts(here); addpath(here);
addpath(genpath(fullfile(fileparts(root),'RRT')));
out=fullfile(root,'规划文件','workflow_runs','continuity_optimization_20261015');
d=load(fullfile(out,'full_trial_4.mat')); raw=load(fullfile(root,'规划文件','workflow_runs','interface_branch_repair_20261015','replanned','full_raw_trajectory.mat'));
ss=load(raw.sceneFile,'scene'); scene=ss.scene; Q=d.Q; P=d.P; t=d.t; V=d.V; maps=d.maps; joins=d.joins; labels=d.labels; original=d.original;
result=struct('executionReady',false,'trials',{{}},'q_nodes_changed',false);
for c2=[false true]
    if c2, jj=joins; else, jj=[]; end
    [candidate,op]=optimize_hmi_cubic_node_velocities(Q,P,t,V,scene,struct('joins',jj,...
        'searchError',0.49,'restEndpoints',true,'samplesPerSegment',9));
    result.trials{end+1}=struct('c2_joins',c2,'optimization',op);
    fprintf('BOUNDARY_TRIAL C2=%d pass=%d\n',c2,op.pass);
    if op.pass
        V=candidate;
        save(fullfile(out,sprintf('full_candidate_c2_%d.mat',c2)),'Q','P','t','V','maps','joins','labels','scene','original','-v7.3');
    end
end
fid=fopen(fullfile(out,'boundary_diagnosis.json'),'w'); c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(result,PrettyPrint=true),'char');
end
