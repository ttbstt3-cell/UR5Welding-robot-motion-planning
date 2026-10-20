function report = export_v1_realhome_to_preweld_hmi(varargin)
%EXPORT_V1_REALHOME_TO_PREWELD_HMI Export the latest V1 Task-A result.
%
% Reads the saved V1 FastConnectAnytimeRRTStar result only.  It does not
% re-run the planner, change the weld path, or send a ROS2 command.

here = fileparts(mfilename("fullpath"));
trajectoryDir = fullfile(here,"..","轨迹文件");
v1Default = fullfile(here,"..","..","RRT","results", ...
    "v1_fast_connect_anytime_taskA_seed20260917.mat");
p = inputParser;
addParameter(p,"ResultFile",v1Default,@(x)ischar(x)||isstring(x));
addParameter(p,"OutputFile",fullfile(trajectoryDir,"realhome_to_preweld_v1.mat"),@(x)ischar(x)||isstring(x));
addParameter(p,"ReportFile",fullfile(trajectoryDir,"realhome_to_preweld_v1_report.json"),@(x)ischar(x)||isstring(x));
addParameter(p,"MaxVelocityDegPerSec",5,@(x)isscalar(x)&&x>0);
addParameter(p,"MaxAccelerationDegPerSec2",15,@(x)isscalar(x)&&x>0);
parse(p,varargin{:});
cfg = p.Results;

if ~isfile(cfg.ResultFile)
    error("V1 result not found: %s",cfg.ResultFile);
end
s = load(cfg.ResultFile,"report");
if ~isfield(s,"report") || ~isfield(s.report,"paths") || ~isfield(s.report,"rows")
    error("V1 result does not contain report.paths/report.rows.");
end
r = s.report;
labels = string(r.algorithmLabels(:));
idx = find(contains(labels,"V1 FastConnectAnytimeRRTStar"),1,"first");
if isempty(idx)
    error("V1 FastConnectAnytimeRRTStar entry not found in result.");
end
qPathRaw = double(r.paths{idx});
row = r.rows(idx);
if ~logical(row.validSuccess) || ~logical(row.finalDenseValidationPass) || isempty(qPathRaw)
    error("Saved V1 path is not dense-validation PASS; refusing export.");
end

[qPath,turnCounts] = lift_continuous_joint_path(qPathRaw);
[timeFromStart,timeDetails] = parameterize_joint_path(qPath, ...
    "MaxVelocityDegPerSec",cfg.MaxVelocityDegPerSec, ...
    "MaxAccelerationDegPerSec2",cfg.MaxAccelerationDegPerSec2);

collisionChecks = double(row.denseValidationCollisionConfigurationChecks) + ...
    double(row.denseValidationEdgeChecks);
report = export_hmi_trajectory(qPath,cfg.OutputFile, ...
    "CoordinateSpace","ros_real", ...
    "TimeFromStart",timeFromStart, ...
    "TrajectoryName","RealHome_to_PreWeld_V1", ...
    "Source","MATLAB_V1_FASTCONNECT_ANYTIME_RRTSTAR", ...
    "Description","Saved V1 FastConnectAnytimeRRTStar Task-A path qHome to qPreWeld. Dense validation PASS. No weld path replanning.", ...
    "CollisionVerified",true, ...
    "CollisionCheckPass",true, ...
    "CollisionSampleCount",collisionChecks, ...
    "MinimumClearanceM",NaN, ...
    "ReportFile",cfg.ReportFile);

report.result_file = char(string(cfg.ResultFile));
report.algorithm = char(labels(idx));
report.task = r.task;
report.algorithm_row = row;
report.time_parameterization = timeDetails;
report.raw_path_max_adjacent_step_deg = rad2deg(max(max(abs(diff(qPathRaw,1,1)),[],2)));
report.exported_path_max_adjacent_step_deg = rad2deg(max(max(abs(diff(qPath,1,1)),[],2)));
report.continuous_branch_turn_counts = turnCounts;
report.continuous_branch_lift_applied = any(turnCounts(:) ~= 0);

fid = fopen(cfg.ReportFile,'w');
if fid < 0, error("Cannot rewrite report: %s",cfg.ReportFile); end
cleanup = onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(report),'char');

fprintf("\n[PASS] Saved V1 path exported without replanning.\n");
fprintf("Algorithm: %s\n",labels(idx));
fprintf("Points: %d, duration: %.3f s, max step: %.3f deg\n", ...
    report.point_count,report.duration_sec,report.exported_path_max_adjacent_step_deg);
end
