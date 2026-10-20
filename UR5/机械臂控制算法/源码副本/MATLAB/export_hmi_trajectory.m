function report = export_hmi_trajectory(qPath, outputFile, varargin)
%EXPORT_HMI_TRAJECTORY Export an explicit MATLAB path for the UR5 HMI.
%
% CoordinateSpace is mandatory and must be either "ros_real" (the ROS/URDF
% continuous joint branch) or "coppeliasim".  CoppeliaSim values are
% converted with qReal = qCop + qOffsetRad; no wrapToPi is performed.
% TimeFromStart is optional only when AllowMissingTime=true.  Such a file is
% intentionally preview-only in the HMI.

qPath = double(qPath);
if ndims(qPath) ~= 2 || size(qPath,2) ~= 6 || size(qPath,1) < 1
    error("qPath must be an N-by-6 matrix.");
end
if any(~isfinite(qPath(:)))
    error("qPath contains NaN or Inf.");
end

p = inputParser;
p.FunctionName = mfilename;
addParameter(p,"CoordinateSpace","",@(x)ischar(x)||isstring(x));
addParameter(p,"QOffsetDeg",[180 -90 0 -90 0 0],@(x)isnumeric(x)&&numel(x)==6&&all(isfinite(x)));
addParameter(p,"TimeFromStart",[],@(x)isnumeric(x));
addParameter(p,"AllowMissingTime",false,@(x)islogical(x)||ismember(x,[0 1]));
addParameter(p,"TrajectoryName","unnamed_trajectory",@(x)ischar(x)||isstring(x));
addParameter(p,"Source","MATLAB",@(x)ischar(x)||isstring(x));
addParameter(p,"Description","",@(x)ischar(x)||isstring(x));
addParameter(p,"CollisionVerified",false,@(x)islogical(x)||ismember(x,[0 1]));
addParameter(p,"CollisionCheckPass",false,@(x)islogical(x)||ismember(x,[0 1]));
addParameter(p,"CollisionSampleCount",NaN,@(x)isscalar(x));
addParameter(p,"MinimumClearanceM",NaN,@(x)isscalar(x));
addParameter(p,"ReportFile","",@(x)ischar(x)||isstring(x));
addParameter(p,"Velocities",[],@(x)isnumeric(x));
addParameter(p,"Accelerations",[],@(x)isnumeric(x));
parse(p,varargin{:});
cfg = p.Results;

space = lower(string(cfg.CoordinateSpace));
if ~ismember(space,["ros_real","coppeliasim"])
    error("CoordinateSpace must be explicitly 'ros_real' or 'coppeliasim'.");
end

qOffsetRad = deg2rad(double(cfg.QOffsetDeg(:).'));
if space == "coppeliasim"
    qPathExport = qPath + qOffsetRad;
else
    qPathExport = qPath;
end

timeFromStart = double(cfg.TimeFromStart(:));
hasTime = ~isempty(timeFromStart);
if hasTime
    if numel(timeFromStart) ~= size(qPathExport,1) || any(~isfinite(timeFromStart)) || ...
            timeFromStart(1) < 0 || any(diff(timeFromStart) <= 0)
        error("TimeFromStart must be finite, nonnegative, and strictly increasing.");
    end
elseif ~logical(cfg.AllowMissingTime)
    error("TimeFromStart is required. Use AllowMissingTime=true only for preview-only export.");
end

velocities = double(cfg.Velocities);
if ~isempty(velocities) && (any(size(velocities) ~= size(qPathExport)) || any(~isfinite(velocities(:))))
    error("Velocities must be finite N-by-6 when supplied.");
end
accelerations = double(cfg.Accelerations);
if ~isempty(accelerations) && (any(size(accelerations) ~= size(qPathExport)) || any(~isfinite(accelerations(:))))
    error("Accelerations must be finite N-by-6 when supplied.");
end

outputFile = char(string(outputFile));
[outputDir,~,~] = fileparts(outputFile);
if ~isempty(outputDir) && ~isfolder(outputDir)
    mkdir(outputDir);
end

joint_names = { ...
    'shoulder_pan_joint', ...
    'shoulder_lift_joint', ...
    'elbow_joint', ...
    'wrist_1_joint', ...
    'wrist_2_joint', ...
    'wrist_3_joint'};
qPath = qPathExport; %#ok<NASGU>  % canonical HMI variable name
metadata = struct();
metadata.source = char(string(cfg.Source));
metadata.trajectory_name = char(string(cfg.TrajectoryName));
metadata.description = char(string(cfg.Description));
metadata.coordinate_space_input = char(space);
metadata.coordinate_space = 'ros_real';
metadata.q_offset_deg = double(cfg.QOffsetDeg(:).');
metadata.continuous_joint_branch_preserved = true;
metadata.collision_verified = logical(cfg.CollisionVerified);
metadata.collision_check_pass = logical(cfg.CollisionCheckPass);
metadata.collision_sample_count = double(cfg.CollisionSampleCount);
metadata.minimum_clearance_m = double(cfg.MinimumClearanceM);
metadata.time_parameterized = hasTime;
metadata.preview_only = ~hasTime;

if hasTime
    if isempty(velocities) && size(qPath,1) > 1
        velocities = [zeros(1,6); diff(qPath,1,1) ./ diff(timeFromStart)]; %#ok<NASGU>
    end
    if isempty(accelerations)
        accelerations = []; %#ok<NASGU>
    end
end

if hasTime && ~isempty(velocities) && ~isempty(accelerations)
    save(outputFile,'qPath','joint_names','timeFromStart','velocities','accelerations','metadata','-v7');
elseif hasTime && ~isempty(velocities)
    save(outputFile,'qPath','joint_names','timeFromStart','velocities','metadata','-v7');
elseif hasTime
    save(outputFile,'qPath','joint_names','timeFromStart','metadata','-v7');
else
    save(outputFile,'qPath','joint_names','metadata','-v7');
end

if hasTime
    durationSec = timeFromStart(end);
else
    durationSec = NaN;
end
if size(qPath,1) > 1
    adjacentStepRad = max(abs(diff(qPath,1,1)),[],2);
    maxAdjacentStepRad = max(adjacentStepRad);
else
    maxAdjacentStepRad = 0;
end

report = struct();
report.created = char(datetime("now","Format","yyyy-MM-dd HH:mm:ss"));
report.output_file = outputFile;
report.trajectory_name = metadata.trajectory_name;
report.source = metadata.source;
report.coordinate_space_input = metadata.coordinate_space_input;
report.coordinate_space_output = metadata.coordinate_space;
report.point_count = size(qPath,1);
report.duration_sec = durationSec;
report.collision_verified = metadata.collision_verified;
report.collision_check_pass = metadata.collision_check_pass;
report.collision_sample_count = metadata.collision_sample_count;
report.minimum_clearance_m = metadata.minimum_clearance_m;
report.max_adjacent_step_rad = maxAdjacentStepRad;
report.max_adjacent_step_deg = rad2deg(maxAdjacentStepRad);
report.time_parameterized = hasTime;
report.q_start_rad = qPath(1,:);
report.q_end_rad = qPath(end,:);
report.note = 'Export only; HMI validation and final execution gate remain mandatory.';

reportFile = char(string(cfg.ReportFile));
if isempty(reportFile)
    [d,n,~] = fileparts(outputFile);
    reportFile = fullfile(d,[n '_report.json']);
end
[reportDir,~,~] = fileparts(reportFile);
if ~isempty(reportDir) && ~isfolder(reportDir)
    mkdir(reportDir);
end
fid = fopen(reportFile,'w');
if fid < 0
    error("Cannot write report: %s",reportFile);
end
cleanup = onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(report),'char');
report.report_file = reportFile;
fprintf("Exported HMI trajectory: %s\n",outputFile);
fprintf("Report: %s\n",reportFile);
end
