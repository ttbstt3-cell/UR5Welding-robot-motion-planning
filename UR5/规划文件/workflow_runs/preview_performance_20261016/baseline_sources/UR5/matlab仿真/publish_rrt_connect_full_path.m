function out = publish_rrt_connect_full_path(resultFile,options)
%PUBLISH_RRT_CONNECT_FULL_PATH Send the complete validated path to CoppeliaSim.
%
% The RRT result is stored in MATLAB/base coordinates.  The external ROS2
% controller drives CoppeliaSim joints, whose zero is the Coppelia zero.  This
% helper therefore unwraps the MATLAB path continuously and applies
% qCop = qMat - scene.qOffset before publishing JointTrajectory.
%
% The default is a dry run (Send=false).  Set Send=true only after confirming
% that CoppeliaSim is open, the simulation is running, and the existing
% ur5_trajectory_controller is running with send_demo:=false.
% The default playback timing is restored to the original fixed 0.15 s per
% trajectory segment.  Set timingMode="tcp_speed" to use the optional
% TCP-speed time parameterization instead.

if nargin < 1 || isempty(resultFile)
    here = fileparts(mfilename("fullpath"));
    resultFile = fullfile(here,"weld_rrt_connect_approach_exit.mat");
else
    resultFile = string(resultFile);
    here = fileparts(mfilename("fullpath"));
end
if nargin < 2 || isempty(options), options = struct(); end

defaults = struct( ...
    "send",false, ...
    "timingMode","fixed", ...
    "fixedSegmentDuration",0.15, ...
    "freeSegmentDuration",0.15, ...
    "transitionSpeedMps",0.043, ...
    "weldSpeedMps",0.043, ...
    "minSegmentDuration",0.02, ...
    "waitForDiscovery",2.0, ...
    "commandFile",fullfile(here,"weld_rrt_connect_full_path_command.mat"), ...
    "reportFile",fullfile(here,"weld_rrt_connect_full_path_command_report.txt"));
options = localMergeOptions(defaults,options);

timingMode = lower(string(options.timingMode));
if ~ismember(timingMode,["fixed","tcp_speed"])
    error("timingMode 必须为 'fixed' 或 'tcp_speed'。 ");
end
if ~isscalar(options.fixedSegmentDuration) || options.fixedSegmentDuration <= 0
    error("fixedSegmentDuration 必须为正标量。 ");
end

if ~isfile(resultFile)
    error("找不到 RRT-Connect 结果：%s",resultFile);
end
s = load(resultFile,"result");
result = s.result;
if ~isfield(result,"fullPass") || ~result.fullPass
    error("RRT-Connect 结果未通过，拒绝发送。请先重新验证结果。");
end

% Assemble the five physical segments without changing the existing weld path.
parts = { ...
    result.approach.qPath, ...
    [result.preWeld.q;result.qWeld(1,:)], ...
    result.qWeld, ...
    [result.qWeld(end,:);result.postWeld.q], ...
    result.exit.qPath};
partNames = ["approach","pre_weld","welding","post_weld","exit"];
qMatRaw = zeros(0,6);
labels = strings(0,1);
for k = 1:numel(parts)
    qPart = double(parts{k});
    if isempty(qPart), continue; end
    % Adjacent segments share their endpoint; retain one copy only.
    if ~isempty(qMatRaw) && norm(qPart(1,:) - qMatRaw(end,:)) < 1e-10
        qPart = qPart(2:end,:);
    end
    if isempty(qPart), continue; end
    qMatRaw = [qMatRaw;qPart]; %#ok<AGROW>
    labels = [labels;repmat(partNames(k),size(qPart,1),1)]; %#ok<AGROW>
end
if size(qMatRaw,2) ~= 6 || size(qMatRaw,1) < 2
    error("拼接后的完整路径无效。");
end

% Resolve 2*pi branch changes before converting to Coppelia joint coordinates.
qMat = localUnwrapPath(qMatRaw);
qOffset = double(result.scene.qOffset(:).');
qCop = qMat - qOffset;

qHomeResidual = atan2(sin(qCop(end,:)),cos(qCop(end,:)));
if norm(qCop(1,:)) > 1e-8 || norm(qHomeResidual) > 1e-8
    error("完整路径首尾未回到 CoppeliaSim Home；请检查 qOffset 映射。");
end

% Use FK of the saved validated scene to time welding/transition segments.
tcpPos = zeros(size(qMat,1),3);
for i = 1:size(qMat,1)
    tcpPos(i,:) = tform2trvec(getTransform( ...
        result.scene.robot,qMat(i,:),result.scene.tcpBodyName));
end

N = size(qMat,1);
if timingMode == "fixed"
    % Original slow playback: every adjacent trajectory point is 0.15 s.
    dt = repmat(double(options.fixedSegmentDuration),N-1,1);
else
    dt = repmat(double(options.freeSegmentDuration),N-1,1);
    for i = 1:N-1
        ds = norm(tcpPos(i+1,:) - tcpPos(i,:));
        if labels(i) == "welding" || labels(i+1) == "welding"
            dt(i) = max(options.minSegmentDuration,ds/options.weldSpeedMps);
        elseif labels(i) == "pre_weld" || labels(i+1) == "pre_weld" || ...
                labels(i) == "post_weld" || labels(i+1) == "post_weld"
            dt(i) = max(options.minSegmentDuration,ds/options.transitionSpeedMps);
        end
    end
end
timeFromStart = [0;cumsum(dt)];

command = struct();
command.created = char(datetime("now","Format","yyyy-MM-dd HH:mm:ss"));
command.sourceResultFile = char(resultFile);
command.qMatRaw = qMatRaw;
command.qMatContinuous = qMat;
command.qCop = qCop;
command.qOffset = qOffset;
command.labels = labels;
command.tcpPositionsMat = tcpPos;
command.segmentDurationSec = dt;
command.timeFromStartSec = timeFromStart;
command.totalDurationSec = timeFromStart(end);
command.timingMode = char(timingMode);
command.fixedSegmentDurationSec = options.fixedSegmentDuration;
command.weldSpeedMps = options.weldSpeedMps;
command.transitionSpeedMps = options.transitionSpeedMps;
command.freeSegmentDurationSec = options.freeSegmentDuration;
command.jointNames = { ...
    'shoulder_pan_joint';'shoulder_lift_joint';'elbow_joint'; ...
    'wrist_1_joint';'wrist_2_joint';'wrist_3_joint'};
command.sent = false;

if options.send
    node = ros2node("/matlab_rrt_connect_executor");
    pub = ros2publisher(node,"/joint_trajectory", ...
        "trajectory_msgs/JointTrajectory");
    fprintf("Waiting %.1f s for /joint_trajectory discovery...\n", ...
        options.waitForDiscovery);
    pause(options.waitForDiscovery);
    msg = ros2message(pub);
    msg.joint_names = command.jointNames;
    msg.header.frame_id = 'base';
    for i = 1:N
        point = ros2message("trajectory_msgs/JointTrajectoryPoint");
        point.positions = qCop(i,:).';
        t = timeFromStart(i);
        secValue = int32(floor(t));
        nanosecValue = round((t-double(secValue))*1e9);
        if nanosecValue >= 1e9
            secValue = secValue + int32(1);
            nanosecValue = nanosecValue - 1e9;
        end
        point.time_from_start.sec = secValue;
        point.time_from_start.nanosec = uint32(nanosecValue);
        msg.points(i) = point; %#ok<AGROW>
    end
    send(pub,msg);
    command.sent = true;
    fprintf("Published complete path: %d points, duration %.3f s.\n", ...
        N,timeFromStart(end));
end

save(options.commandFile,"command","-v7.3");

fid = fopen(options.reportFile,"w");
if fid < 0, error("无法写入命令报告：%s",options.reportFile); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,"RRT-Connect complete-path command report\n");
fprintf(fid,"Created: %s\n",command.created);
fprintf(fid,"Source result: %s\n",command.sourceResultFile);
fprintf(fid,"Sent: %d\n",command.sent);
fprintf(fid,"Points: %d\n",N);
fprintf(fid,"Total duration (s): %.6f\n",command.totalDurationSec);
fprintf(fid,"Timing mode: %s\n",command.timingMode);
fprintf(fid,"Fixed segment duration (s): %.6f\n",command.fixedSegmentDurationSec);
fprintf(fid,"Weld speed target (m/s): %.6f\n",command.weldSpeedMps);
fprintf(fid,"qMat Home (deg): %s\n",mat2str(rad2deg(qMat(1,:)),8));
fprintf(fid,"qCop Home (deg): %s\n",mat2str(rad2deg(qCop(1,:)),8));
fprintf(fid,"qCop final (deg): %s\n",mat2str(rad2deg(qCop(end,:)),8));
fprintf(fid,"qCop final modulo 2pi (deg): %s\n", ...
    mat2str(rad2deg(qHomeResidual),8));
fprintf(fid,"Max qMat step (deg): %.6f\n", ...
    rad2deg(max(max(abs(diff(qMat,1,1))))));
fprintf(fid,"Max qCop step (deg): %.6f\n", ...
    rad2deg(max(max(abs(diff(qCop,1,1))))));
fprintf(fid,"Existing 80-point weld path was not replanned: 1\n");

out = command;
out.commandFile = options.commandFile;
out.reportFile = options.reportFile;
fprintf("Saved command MAT: %s\n",options.commandFile);
fprintf("Saved command report: %s\n",options.reportFile);
end

function q = localUnwrapPath(qRaw)
q = qRaw;
for i = 2:size(q,1)
    delta = q(i,:) - q(i-1,:);
    q(i,:) = q(i,:) - 2*pi*round(delta/(2*pi));
end
end

function options = localMergeOptions(defaults,overrides)
options = defaults;
if isempty(overrides), return; end
fields = fieldnames(overrides);
for i = 1:numel(fields)
    options.(fields{i}) = overrides.(fields{i});
end
end
