function result=diagnose_optimize_hmi_continuity()
% Offline only: never creates an execution-ready trajectory or sends ROS data.
here=fileparts(mfilename('fullpath')); root=fileparts(here); addpath(here);
addpath(genpath(fullfile(fileparts(root),'RRT')));
out=fullfile(root,'规划文件','workflow_runs','continuity_optimization_20261015');
rawFile=fullfile(root,'规划文件','workflow_runs','interface_branch_repair_20261015','replanned','full_raw_trajectory.mat');
raw=load(rawFile); ss=load(raw.sceneFile,'scene'); scene=ss.scene;
d=load(fullfile(root,'规划文件','workflow_runs','interface_branch_repair_20261015','timing','velocity_profile_input_diagnostic.mat'),'base');
base=d.base; Q=base.qPath; P=base.tcpPosePath; t=base.timeFromStart(:); seed=base.velocities{4};
result=struct('executionReady',false,'robot_motion','NO','source_file',rawFile,'trials',{{}});
[r,s]=hmi_cubic_tracking_diagnostics(Q,P,t,seed,scene,101);
r.weld_start_in_full_task_sec=raw.segments{1}.timeFromStart(end);
result.baseline=r; save(fullfile(out,'baseline_diagnostic.mat'),'r','s','-v7.3');
writeJson(fullfile(out,'baseline.json'),r);
fprintf('BASELINE sync=%.9f ori=%.9f time=%.6f\n',r.position_max_mm,r.orientation_max_deg,t(end));
for scale=[1 1.15 1.4]
    trialT=t*scale; trialSeed=seed/scale;
    [V,op]=optimize_hmi_cubic_node_velocities(Q,P,trialT,trialSeed,scene,struct());
    trial=struct('scale',scale,'optimization',op); result.trials{end+1}=trial;
    save(fullfile(out,sprintf('weld_trial_%.2f.mat',scale)),'Q','P','trialT','V','op','-v7.3');
    writeJson(fullfile(out,'optimization_progress.json'),result);
    if op.pass
        result.weld_candidate=struct('scale',scale,'duration_sec',trialT(end),'optimization',op);
        save(fullfile(out,'weld_candidate.mat'),'Q','P','trialT','V','scene','-v7.3');
        break;
    end
end
writeJson(fullfile(out,'optimization_progress.json'),result);
end
function writeJson(path,value)
fid=fopen(path,'w'); assert(fid>0); clean=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(value,PrettyPrint=true),'char');
end
