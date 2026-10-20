function report=diagnose_current_point1_axis_ab(outputDir)
% Cold Point1 candidate generation with unchanged saved search parameters.
% A/B exhaustive filtering reuses one IK pool, so only axis gate differs.
here=fileparts(mfilename('fullpath')); root=fileparts(here);
baseline=fullfile(root,'规划文件','real_anchor_runs','Areal_confirmed_20261013','real_seam_round_01.mat');
S=load(baseline,'result'); opts=S.result.options;
opts.sceneScript=fullfile(here,'sync_ur5_real_scene.m');
assert(~opts.disableAxisConstraint && ~opts.disable_axis_constraint);
assert(opts.checkSelfCollision && ~opts.publish && ~opts.animate);
if ~isfolder(outputDir), mkdir(outputDir); end
opts.outputFile=fullfile(outputDir,'current_core.mat');
% Diagnostic mode suppresses refinement/timing/executable output only.
opts.configurationOnlyDiagnostic=true;
result=replan_weld_actual_axis_adaptive_core(opts); %#ok<NASGU>
report=diagnose_accp_point1_collision_pairs(opts.outputFile,outputDir,'all_ik');
fprintf('POINT1_AXIS_AB %s\n',jsonencode(report.ab_comparison));
end
