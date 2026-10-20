function result=test_accp_adaptive_velocity_profile()
% Reuse accepted offline timing snapshot; no planning or robot connection.
root=fileparts(fileparts(mfilename('fullpath')));
src=fullfile(root,'规划文件','real_anchor_runs','controller_aware_timing_20261013_v2','controller_timing_diagnostic.mat');
sceneSrc=fullfile(root,'规划文件','real_anchor_runs','accp_bridge_comparison_20261013_improved_full_cold_v2.mat');
out=fullfile(root,'规划文件','real_anchor_runs','adaptive_velocity_profile_20261013_final');
if ~isfolder(out), mkdir(out); end
diary(fullfile(out,'matlab.log')); cleanup=onCleanup(@()diary('off')); %#ok<NASGU>
S=load(src,'result'); B=load(sceneSrc,'result'); r=S.result;
assert(isequal(r.qPath,B.result.qPath) && isequal(r.tcpPosePath,B.result.tcpPosePath));
assert(isequal(r.calibrationBinding,B.result.scene.calibrationBinding));
assert(B.result.options.wire_collision_shorten_mm==3);
result=generate_accp_adaptive_velocity_profile(r.qPath,r.tcpPosePath,r.timeFromStart,B.result.scene,...
    struct('outputDirectory',out));
assert(isequal(result.qPath,r.qPath) && isequal(result.tcpPosePath,r.tcpPosePath));
assert(isequal(result.timeFromStart,r.timeFromStart));
assert(~result.executionReady && result.offlineOnly);
assert(all(cellfun(@(a)a.sample_count>=3001,result.audits)));
assert(result.audits{4}.complete && result.audits{4}.collision_pass);
assert(result.audits{4}.max_joint_velocity_rad_per_sec<=result.limits.velocityLimit);
assert(result.audits{4}.max_joint_acceleration_rad_per_sec2<=result.limits.accelerationLimit);
fprintf('VELOCITY_PROFILE_REGRESSION_PASS numericalPassingStrategies=%s\n',strjoin(result.numericalPassingStrategies,','));
end
