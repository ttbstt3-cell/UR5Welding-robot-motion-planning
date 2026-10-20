function result=test_accp_controller_timing_optimization()
% Reproducible offline test on the complete ACCP + Cartesian Bridge seam.
root=fileparts(fileparts(mfilename('fullpath')));
input=fullfile(root,'规划文件','real_anchor_runs','accp_bridge_comparison_20261013_improved_full_cold_v2.mat');
out=fullfile(root,'规划文件','real_anchor_runs','controller_aware_timing_20261013_v2');
if ~isfolder(out), mkdir(out); end
diary(fullfile(out,'matlab.log')); c=onCleanup(@()diary('off')); %#ok<NASGU>
S=load(input,'result'); r=S.result;
assert(r.configurationComplete && r.nodeCollisionFree && r.edgeCollisionFree);
assert(r.options.wire_collision_shorten_mm==3);
% Map the actual bridge neighborhood to the refined immutable nodes.
recovered=r.cartesianBridge.recovered_point_indices(1);
lo=recovered-1; hi=recovered+r.cartesianBridge.selected_path_added_nodes;
idx=zeros(1,hi-lo+1);
for k=lo:hi
    [d,idx(k-lo+1)]=min(vecnorm(r.qPath-r.qPathInitial(k,:),2,2));
    assert(d<1e-8,'Bridge node not found in refined qPath');
end
priority=max(1,min(idx)-1):min(size(r.qPath,1)-1,max(idx)+1);
result=optimize_accp_controller_timing(r.qPath,r.tcpPosePath,r.timeFromStart,r.scene,...
    struct('outputDirectory',out,'accelerationLimit',0.261799,'prioritySegments',priority));
assert(isequal(result.qPath,r.qPath));
assert(isequal(result.tcpPosePath,r.tcpPosePath));
assert(~result.executionReady && result.offlineOnly);
assert(result.after.sample_count>=3001);
assert(result.after.max_joint_velocity_rad_per_sec<=result.limits.velocityLimit);
assert(result.after.max_joint_acceleration_rad_per_sec2<=0.261799);
assert(result.after.collision_pass && result.after.joint_limits_pass);
assert(result.after.max_tcp_contour_error_mm<0.5);
assert(result.after.max_tcp_orientation_error_deg<0.5);
assert(result.firstSegmentTimeInvariantLowerBoundMm>0.5);
assert(~result.numericalAcceptancePass);
fprintf('BEFORE duration=%.9f contour=%.9f sync=%.9f orientation=%.9f velocity=%.9f acceleration=%.9f\n',...
    result.before.duration_sec,result.before.max_tcp_contour_error_mm,...
    result.before.max_tcp_time_sync_error_mm,result.before.max_tcp_orientation_error_deg,...
    result.before.max_joint_velocity_rad_per_sec,result.before.max_joint_acceleration_rad_per_sec2);
fprintf('AFTER duration=%.9f contour=%.9f sync=%.9f orientation=%.9f velocity=%.9f acceleration=%.9f collision=%d numericalPass=%d executionReady=%d\n',...
    result.after.duration_sec,result.after.max_tcp_contour_error_mm,...
    result.after.max_tcp_time_sync_error_mm,result.after.max_tcp_orientation_error_deg,...
    result.after.max_joint_velocity_rad_per_sec,result.after.max_joint_acceleration_rad_per_sec2,...
    result.after.collision_pass,result.numericalAcceptancePass,result.executionReady);
fprintf('Failure reason: %s\n',result.failureReason);
end
