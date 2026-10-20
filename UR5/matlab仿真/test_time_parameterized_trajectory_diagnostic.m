function report=test_time_parameterized_trajectory_diagnostic()
%TEST_TIME_PARAMETERIZED_TRAJECTORY_DIAGNOSTIC Offline regression only.
% Replays the saved embedded ACCP seam path; no planning or robot motion.
root=fileparts(mfilename('fullpath'));
runDir=fullfile(root,'..','planning_runs','accp_embedded_precision_20261012');
S=load(fullfile(runDir,'embedded_accp.mat'),'result');
R=S.result;
Q=R.qPathExecution;
[t,~]=parameterize_joint_path(Q);
V=[zeros(1,6); diff(Q,1,1)./diff(t)];
trajectoryFile=fullfile(runDir,'time_parameterized_diagnostic_input.mat');
qPath=Q; timeFromStart=t; velocities=V; tcpPosePath=R.tcpTargetExecution; scene=R.scene;
metadata=struct('source','offline diagnostic','robot_motion','NO');
save(trajectoryFile,'qPath','timeFromStart','velocities','tcpPosePath','scene','metadata','-v7');
reportFile=fullfile(runDir,'time_parameterized_tcp_diagnostic.json');
report=verify_time_parameterized_trajectory(trajectoryFile,R,reportFile,struct('scene',R.scene,'samplesPerSegment',101));
fprintf('A direct max/RMS: %.9f / %.9f mm\n',report.direct_refined_qPath.max_position_error_mm,report.direct_refined_qPath.rms_position_error_mm);
fprintf('B timed max/RMS: %.9f / %.9f mm\n',report.time_parameterized_trajectory.max_position_error_mm,report.time_parameterized_trajectory.rms_position_error_mm);
fprintf('B worst time: %.9f s\n',report.time_parameterized_trajectory.max_position_error_time_sec);
assert(report.direct_refined_qPath.max_position_error_mm < 0.5,'Direct refined baseline unexpectedly exceeds 0.5 mm.');
assert(report.time_parameterized_trajectory.max_position_error_mm > report.direct_refined_qPath.max_position_error_mm,'Timed diagnostic did not exceed direct baseline.');
end
