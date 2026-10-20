function publish_weld_trajectory(qPath, options)
%PUBLISH_WELD_TRAJECTORY Publish a validated qPath on /joint_trajectory.
% The existing rclpy controller consumes this standard topic; no controller
% source or CoppeliaSim scene changes are made here.

if nargin < 2 || isempty(options), options = struct(); end
if ~isfield(options,"durationPerSegment"), options.durationPerSegment = 0.15; end
if size(qPath,2) ~= 6 || size(qPath,1) < 2
    error("qPath 必须是至少 2×6 的 rad 关节轨迹。");
end

node = ros2node("/matlab_weld_replanner");
pub = ros2publisher(node,"/joint_trajectory","trajectory_msgs/JointTrajectory");
pause(2.0);  % allow DDS discovery with the existing controller
msg = ros2message(pub);
msg.joint_names = { ...
    'shoulder_pan_joint';'shoulder_lift_joint';'elbow_joint'; ...
    'wrist_1_joint';'wrist_2_joint';'wrist_3_joint'};

N = size(qPath,1);
% MATLAB ROS 2 message arrays are easiest to construct by appending the
% trajectory point message type returned by ros2message('...').
for i = 1:N
    point = ros2message("trajectory_msgs/JointTrajectoryPoint");
    point.positions = qPath(i,:).';
    t = (i-1)*options.durationPerSegment;
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
fprintf("Published %d trajectory points on /joint_trajectory (%.2f s).\n", ...
    N,(N-1)*options.durationPerSegment);
end
