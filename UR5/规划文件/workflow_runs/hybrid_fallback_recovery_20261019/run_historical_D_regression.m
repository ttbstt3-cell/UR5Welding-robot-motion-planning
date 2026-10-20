function run_historical_D_regression()
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT')));
b=load(fullfile(root,'规划文件','real_anchor_runs','adaptive_velocity_profile_20261013_final',...
    'velocity_profiles_diagnostic.mat'),'result');
s=load(fullfile(root,'规划文件','real_anchor_runs','accp_bridge_comparison_20261013_improved_full_cold_v2.mat'),'result');
scene=s.result.scene;
% Historical snapshot predates scene.jointLimits. Populate the SAME limits
% from its immutable rigidBodyTree in this test only; never from new settings.
limits=[];
for k=1:numel(scene.robot.Bodies)
    joint=scene.robot.Bodies{k}.Joint;
    if ~strcmp(joint.Type,'fixed'), limits(end+1,:)=joint.PositionLimits; end %#ok<AGROW>
end
scene.jointLimits=limits;
r=optimize_accp_hybrid_velocity(b.result,scene,struct('compareReferences',false));
assert(r.hybridCandidateFound && r.hybridPreferredOffline && ~r.executionReady);
[~,selection]=hmi_select_hybrid_velocity_candidate(r,b.result,scene);
assert(~selection.nonoptimal_fallback && strcmp(selection.source,'D_HYBRID'));
save(fullfile(here,'historical_D_positive_regression.mat'),'r','selection','-v7.3');
summary=struct('pass',true,'scope','HISTORICAL_SCENE_NOT_CURRENT_REALHOME',...
    'candidate',selection,'weld_audit',r.audits{4},'attempts',{r.attempts},'executionReady',false);
fid=fopen(fullfile(here,'historical_D_positive_regression.json'),'w'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(summary,'PrettyPrint',true),'char');
fprintf('HISTORICAL_D_POSITIVE_REGRESSION_PASS\n');
end
