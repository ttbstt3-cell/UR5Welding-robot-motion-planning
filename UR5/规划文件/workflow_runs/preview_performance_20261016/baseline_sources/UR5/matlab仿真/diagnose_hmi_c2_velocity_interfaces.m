function result=diagnose_hmi_c2_velocity_interfaces()
here=fileparts(mfilename('fullpath')); root=fileparts(here); addpath(here);
addpath(genpath(fullfile(fileparts(root),'RRT')));
out=fullfile(root,'规划文件','workflow_runs','continuity_optimization_20261015');
d=load(fullfile(out,'full_candidate_c1.mat'));
Q=d.Q; P=d.P; t=d.t; scene=d.scene; maps=d.maps; joins=d.joins; labels=d.labels; original=d.original;
[V,result]=optimize_hmi_cubic_node_velocities(Q,P,t,d.V,scene,struct('joins',joins,...
    'searchError',.46,'restEndpoints',true,'samplesPerSegment',13,'maxIterations',8));
if result.pass
    save(fullfile(out,'full_candidate_c2.mat'),'Q','P','t','V','scene','maps','joins','labels','original','-v7.3');
end
fid=fopen(fullfile(out,'c2_velocity_result.json'),'w'); c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(result,PrettyPrint=true),'char');
fprintf('C2_INTERFACE pass=%d\n',result.pass);
end
