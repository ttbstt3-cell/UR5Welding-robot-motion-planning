function report = export_realhome_to_preweld_hmi(varargin)
%EXPORT_REALHOME_TO_PREWELD_HMI Export the existing validated RRT approach.
%
% This wrapper reads result.approach.qPath from the existing
% weld_rrt_connect_approach_exit.mat.  It never replans, modifies, unwraps,
% or sends the trajectory.  The result is in the ROS/real MATLAB joint space
% because the RRT was built from scene.qMat0 and the MATLAB robot.

here = fileparts(mfilename("fullpath"));
trajectoryDir = fullfile(here,"..","轨迹文件");
p = inputParser;
addParameter(p,"ResultFile",fullfile(here,"weld_rrt_connect_approach_exit.mat"),@(x)ischar(x)||isstring(x));
addParameter(p,"OutputFile",fullfile(trajectoryDir,"realhome_to_preweld.mat"),@(x)ischar(x)||isstring(x));
addParameter(p,"ReportFile",fullfile(trajectoryDir,"realhome_to_preweld_report.json"),@(x)ischar(x)||isstring(x));
addParameter(p,"MaxVelocityDegPerSec",5,@(x)isscalar(x)&&x>0);
addParameter(p,"MaxAccelerationDegPerSec2",15,@(x)isscalar(x)&&x>0);
parse(p,varargin{:});
cfg = p.Results;

if ~isfile(cfg.ResultFile)
    error("RRT result not found: %s",cfg.ResultFile);
end
s = load(cfg.ResultFile,"result");
if ~isfield(s,"result") || ~isfield(s.result,"approach")
    error("RRT result has no result.approach field.");
end
r = s.result;
if ~isfield(r.approach,"qPath") || isempty(r.approach.qPath)
    error("result.approach.qPath is empty.");
end
if ~logical(r.approach.success) || ~logical(r.approach.pass)
    error("Approach path is not independently PASS; refusing export.");
end
if ~isfield(r.approach,"validation") || ~logical(r.approach.validation.pass)
    error("Approach collision validation is not PASS; refusing export.");
end

qPathRaw = double(r.approach.qPath);
[qPath,turnCounts] = lift_continuous_joint_path(qPathRaw);
[timeFromStart,timeDetails] = parameterize_joint_path(qPath, ...
    "MaxVelocityDegPerSec",cfg.MaxVelocityDegPerSec, ...
    "MaxAccelerationDegPerSec2",cfg.MaxAccelerationDegPerSec2);

v = r.approach.validation;
collisionChecks = NaN;
if isfield(v,"collisionChecks"), collisionChecks = double(v.collisionChecks); end
clearance = NaN;

report = export_hmi_trajectory(qPath,cfg.OutputFile, ...
    "CoordinateSpace","ros_real", ...
    "TimeFromStart",timeFromStart, ...
    "TrajectoryName","RealHome_to_PreWeld", ...
    "Source","MATLAB_RRT_CONNECT", ...
    "Description","Existing validated RRT-Connect approach: qHome to qPreWeld; weld path not replanned. Periodic representations were lifted to one continuous joint branch; no wrapToPi was used.", ...
    "CollisionVerified",true, ...
    "CollisionCheckPass",logical(v.pass), ...
    "CollisionSampleCount",collisionChecks, ...
    "MinimumClearanceM",clearance, ...
    "ReportFile",cfg.ReportFile);

report.result_file = char(string(cfg.ResultFile));
report.approach_stats = r.approach.stats;
report.time_parameterization = timeDetails;
report.q_home_rad = double(r.qHome);
report.q_preweld_rad = double(r.preWeld.q);
report.validation = v;
report.raw_path_max_adjacent_step_deg = rad2deg(max(max(abs(diff(qPathRaw,1,1)),[],2)));
report.exported_path_max_adjacent_step_deg = rad2deg(max(max(abs(diff(qPath,1,1)),[],2)));
report.continuous_branch_turn_counts = turnCounts;
report.continuous_branch_lift_applied = any(turnCounts(:) ~= 0);

% Rewrite the report with the additional source/validation context.
fid = fopen(cfg.ReportFile,'w');
if fid < 0, error("Cannot rewrite report: %s",cfg.ReportFile); end
cleanup = onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(report),'char');

fprintf("\n[PASS] Existing RRT approach exported without replanning.\n");
fprintf("Points: %d, duration: %.3f s, max step: %.3f deg\n", ...
    report.point_count,report.duration_sec,report.max_adjacent_step_deg);
end
