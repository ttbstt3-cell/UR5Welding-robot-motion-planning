function test_hmi_velocity_qp_feasible_seed()
here=fileparts(mfilename('fullpath')); root=fileparts(here); addpath(here);
addpath(genpath(fullfile(fileparts(root),'RRT')));
out=fullfile(root,'规划文件','workflow_runs','continuity_optimization_20261015');
d=load(fullfile(out,'full_trial_4.mat')); raw=load(fullfile(root,'规划文件','workflow_runs','interface_branch_repair_20261015','replanned','full_raw_trajectory.mat'));
s=load(raw.sceneFile,'scene'); scene=s.scene;
[V,op]=optimize_hmi_cubic_node_velocities(d.Q,d.P,d.t*7.3,d.V/7.3,scene,...
    struct('searchError',.49,'restEndpoints',true,'samplesPerSegment',7,'feasibilityDiagnostics',true));
save(fullfile(out,'qp_feasible_seed_test.mat'),'op','V');
disp(op);
end
