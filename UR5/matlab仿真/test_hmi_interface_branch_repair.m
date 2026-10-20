function report=test_hmi_interface_branch_repair(sourceDir,outputDir)
% Offline positive/negative tests. No real model or policy edits.
here=fileparts(mfilename('fullpath')); addpath(here); addpath(genpath(fullfile(fileparts(fileparts(here)),'RRT')));
if ~isfolder(outputDir), mkdir(outputDir); end
raw=load(fullfile(sourceDir,'full_raw_trajectory.mat')); ss=load(raw.sceneFile,'scene'); scene=ss.scene;
home=raw.qPath(1,:); tiny=[home;home+[1e-4,zeros(1,5)]];
[Q,good]=hmi_checked_free_path_branch(scene,tiny,tiny(1,:),tiny(2,:));
assert(good.pass && isequal(Q,tiny));
different=tiny; different(:,6)=different(:,6)-2*pi;
[Q,badStart]=hmi_checked_free_path_branch(scene,different,home,tiny(2,:));
assert(~badStart.pass && isempty(Q),'Periodic equivalence cannot override numeric start');
unsafe=[raw.segments{1}.qPath(end,:);raw.segments{2}.qPath(1,:)];
[Q,badLimit]=hmi_checked_free_path_branch(scene,unsafe,unsafe(1,:),unsafe(2,:));
assert(~badLimit.pass && isempty(Q),'Equivalent endpoint must not admit over-limit path');
% The unsafe full-turn edge is tested literally, not collapsed to zero.
badEdge=hmi_audit_joint_interface_path(scene,unsafe,101);
assert(~badEdge.pass && badEdge.limit_sample_count>0 && badEdge.collision_sample_count>0);
assert(badEdge.maximum_numeric_joint_edge_deg>359.99);
% Old previews must fail before IK/velocity optimization, not be silently
% wrapped to a different branch by execution preparation.
r=prepare_hmi_continuous_request(struct('rawFile',fullfile(sourceDir,'full_raw_trajectory.mat'),...
    'outputDir',fullfile(outputDir,'old_preview_rejected')));
assert(~r.executionReady && strcmp(r.failure_identifier,'PREPARE:InterfaceLimits'));
report=struct('pass',true,'robot_motion','NO','executionReady',false,...
    'valid_numeric_path_pass',good.pass,'equivalent_wrong_start_rejected',~badStart.pass,...
    'over_limit_equivalent_branch_rejected',~badLimit.pass,'actual_360_edge_rejected',~badEdge.pass,...
    'actual_360_edge_samples',badEdge.sample_count,'old_preview_preparation_rejected',true);
fid=fopen(fullfile(outputDir,'regression.json'),'w'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(report,'PrettyPrint',true),'char'); %#ok<NASGU>
end
