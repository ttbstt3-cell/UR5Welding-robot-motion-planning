function test_validate_accp_timed_output()
%TEST_VALIDATE_ACCP_TIMED_OUTPUT Offline gate regression; no robot motion.
root=fileparts(mfilename('fullpath'));
reportFile=fullfile(fileparts(root),'planning_runs','accp_embedded_precision_20261012','time_parameterization_comparison.json');
g=validate_accp_timed_output(reportFile,struct('scheme','positions_only_linear','timeSyncAcceptanceMm',0.5));
assert(g.execution_ready,'Expected positions-only offline candidate to pass the explicit 0.5 mm gate.');
g2=validate_accp_timed_output(reportFile,struct('scheme','baseline_cubic','timeSyncAcceptanceMm',0.5));
assert(~g2.execution_ready && contains(g2.failure_reason,'time-sync'),'Baseline must be blocked by time-sync error.');
g3=validate_accp_timed_output(reportFile,struct('scheme','positions_only_linear'));
assert(~g3.execution_ready && contains(g3.failure_reason,'not configured'),'Unconfigured timing gate must remain non-executable.');
g4=validate_accp_timed_output(reportFile,struct('scheme','adaptive_time_scaled_positions_only','timeSyncAcceptanceMm',0.5));
assert(g4.execution_ready,'Adaptive time-scaled positions-only candidate should pass the explicit 0.5 mm gate.');
fprintf('ACCP timed output gate regression: PASS (NO ROBOT MOTION)\n');
end
