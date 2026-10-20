function result=test_hmi_daily_velocity_result(beforeDir,afterDir,outputFile)
% Compare actual full HMI results: only comparison audits may disappear.
beforeDir=char(beforeDir); afterDir=char(afterDir);
old=load(fullfile(beforeDir,'velocity_profile_input_diagnostic.mat'),'base');
new=load(fullfile(afterDir,'velocity_profile_input_diagnostic.mat'),'base');
assert(~isfield(old.base,'comparison_enabled') || old.base.comparison_enabled);
assert(~new.base.comparison_enabled && isequal(new.base.audited_strategy_indices,4));
for k=1:3
    assert(strcmp(new.base.audits{k}.status,'NOT_RUN_COMPARISON_ONLY'));
    assert(~new.base.audits{k}.complete && ~isfield(new.base.audits{k},'collision_pass'));
end
assert(isequal(old.base.qPath,new.base.qPath));
assert(isequal(old.base.tcpPosePath,new.base.tcpPosePath));
assert(isequal(old.base.timeFromStart,new.base.timeFromStart));
assert(isequal(old.base.velocities,new.base.velocities),'Velocity generation changed');
assert(isequal(old.base.audits{4},new.base.audits{4}),'C seed audit changed');
oldReport=jsondecode(fileread(fullfile(beforeDir,'report.json')));
newReport=jsondecode(fileread(fullfile(afterDir,'report.json')));
assert(newReport.success && newReport.continuous_planning_complete && ~newReport.executionReady);
assert(~newReport.velocity_comparison.enabled && all(structfun(@(v)v,newReport.offline_checks)));
assert(isequal(oldReport.controller_validation,newReport.controller_validation),...
    'Final dense controller validation changed');
a=load(oldReport.artifacts.trajectory_mat); b=load(newReport.artifacts.trajectory_mat);
for field={'qPath','tcpPosePath','timeFromStart','velocities','segmentLabel'}
    assert(isequal(a.(field{1}),b.(field{1})),['Final output changed: ' field{1}]);
end
assert(b.metadata.preview_only && b.metadata.diagnostic_only && ~b.metadata.executionReady);
assert(isempty(dir(fullfile(afterDir,'trajectory_execute_ready.*'))));
result=struct('pass',true,'before_sec',oldReport.planning_time,'after_sec',newReport.planning_time,...
    'saved_sec',oldReport.planning_time-newReport.planning_time,...
    'same_q_tcp_time_velocities',true,'same_C_seed_audit',true,...
    'same_final_dense_controller_validation',true,...
    'comparison_audits_removed',3,...
    'comparison_collision_samples_removed',sum(cellfun(@(a)a.sample_count,old.base.audits(1:3))),...
    'executionReady',false,'robot_motion','NO');
fid=fopen(outputFile,'w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(result,'PrettyPrint',true),'char'); disp(result);
end
