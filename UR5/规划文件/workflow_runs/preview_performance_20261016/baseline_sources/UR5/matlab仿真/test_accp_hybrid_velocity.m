function result=test_accp_hybrid_velocity()
root=fileparts(fileparts(mfilename('fullpath')));
basePath=fullfile(root,'规划文件','real_anchor_runs','adaptive_velocity_profile_20261013_final','velocity_profiles_diagnostic.mat');
scenePath=fullfile(root,'规划文件','real_anchor_runs','accp_bridge_comparison_20261013_improved_full_cold_v2.mat');
out=fullfile(root,'规划文件','real_anchor_runs','hybrid_velocity_20261014_final');
if ~isfolder(out), mkdir(out); end
diary(fullfile(out,'matlab.log')); cleanup=onCleanup(@()diary('off'));
% Analytic regression distinguishes constant segment jerk from node jumps.
T=[0;1;2]; QQ=repmat([0;0.01;0.02],1,6);
mm=accp_cubic_derivative_metrics(QQ,T,zeros(3,6));
assert(abs(mm.max_velocity-0.015)<1e-12 && abs(mm.max_acceleration-0.06)<1e-12);
assert(abs(mm.max_segment_jerk-0.12)<1e-12 && abs(mm.max_acceleration_jump-0.12)<1e-12);
mm=accp_cubic_derivative_metrics(QQ,T,0.01*ones(3,6));
assert(mm.max_segment_jerk<1e-12 && mm.max_acceleration_jump<1e-12);
S=load(basePath,'result'); B=load(scenePath,'result');
assert(isequal(S.result.calibrationBinding,B.result.scene.calibrationBinding));
assert(B.result.options.wire_collision_shorten_mm==3);
result=optimize_accp_hybrid_velocity(S.result,B.result.scene,struct('outputDirectory',out));
assert(isequal(result.qPath,S.result.qPath));
assert(isequal(result.timeFromStart,S.result.timeFromStart));
assert(isequal(result.tcpPosePath,S.result.tcpPosePath));
assert(~result.executionReady && result.offlineOnly);
assert(result.hybridCandidateFound && result.hybridPreferredOffline);
assert(result.acceleration90PercentTargetMet);
assert(strcmp(result.boundaryChecks.status,'BLOCKED_NO_CURRENT_BOUND_APPROACH_EXIT'));
assert(result.audits{4}.complete && result.audits{4}.collision_pass && result.audits{4}.joint_limits_pass);
assert(result.audits{4}.max_tcp_time_sync_error_mm<=0.5);
assert(result.audits{4}.max_tcp_orientation_error_deg<=0.5);
assert(all(cellfun(@(a)a.sample_count>=3001,result.audits)));
assert(result.derivativeMetrics{4}.max_segment_jerk<result.derivativeMetrics{3}.max_segment_jerk);
assert(result.derivativeMetrics{4}.max_acceleration_jump<result.derivativeMetrics{3}.max_acceleration_jump);
for k=1:4
    a=result.audits{k}; m=result.derivativeMetrics{k};
    fprintf('%s sync=%.6f contour=%.6f orientation=%.6f v=%.9f a=%.9f jerk=%.6f rmsJerk=%.6f jump=%.6f pass=%d\n',...
        result.strategies{k},a.max_tcp_time_sync_error_mm,a.max_tcp_contour_error_mm,...
        a.max_tcp_orientation_error_deg,m.max_velocity,m.max_acceleration,...
        m.max_segment_jerk,m.rms_segment_jerk,m.max_acceleration_jump,a.complete);
end
fprintf('HYBRID_REGRESSION_PASS found=%d preferredOffline=%d soft90=%d executionReady=%d\n',...
    result.hybridCandidateFound,result.hybridPreferredOffline,result.acceleration90PercentTargetMet,result.executionReady);
end
