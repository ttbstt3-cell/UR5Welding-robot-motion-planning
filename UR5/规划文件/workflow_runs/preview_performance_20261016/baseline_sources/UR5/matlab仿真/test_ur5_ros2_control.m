clear
clc

%% ROS 2 节点
node = ros2node("/matlab_ur5_controller");

%% 发布标准 JointTrajectory
cmdPub = ros2publisher( ...
    node, ...
    "/joint_trajectory", ...
    "trajectory_msgs/JointTrajectory");

%% 持续接收关节状态：只保留最新一帧
jointSub = ros2subscriber( ...
    node, ...
    "/joint_states", ...
    "sensor_msgs/JointState", ...
    @jointStateCallback, ...
    "History", "keeplast", ...
    "Depth", 1);

fprintf("Waiting for ROS 2 discovery...\n");
pause(2);

%% UR5 标准关节名称
jointNames = {
    'shoulder_pan_joint'
    'shoulder_lift_joint'
    'elbow_joint'
    'wrist_1_joint'
    'wrist_2_joint'
    'wrist_3_joint'
};

%% 目标姿态（度 -> 弧度）
qTargetDeg = [
     10
    -30
     45
    -30
     20
     10
];
qTarget = deg2rad(qTargetDeg);

%% 生成 10 个关节轨迹点，0 秒到达起点，2 秒到达目标
numPoints = 10;
qStart = zeros(6, 1);

cmd = ros2message(cmdPub);
cmd.joint_names = jointNames;
cmd.header.frame_id = 'base';

points = repmat(cmd.points, numPoints, 1);
for k = 1:numPoints
    alpha = (k - 1) / (numPoints - 1);
    q = qStart + alpha * (qTarget - qStart);
    t = 2.0 * alpha;
    sec = floor(t);
    nanosec = round((t - sec) * 1e9);

    points(k).positions = q(:)';
    points(k).velocities = zeros(1, 6);
    points(k).accelerations = zeros(1, 6);
    points(k).effort = zeros(1, 6);
    points(k).time_from_start.sec = int32(sec);
    points(k).time_from_start.nanosec = uint32(nanosec);
end
cmd.points = points;

fprintf("\n10-point trajectory sent (deg):\n");
disp(qTargetDeg');
send(cmdPub, cmd);

fprintf("Monitoring UR5; press Ctrl+C to stop.\n\n");

%% 保持 MATLAB 进程运行，让 subscriber callback 持续执行
while true
    pause(0.1);
end

%% =========================================================
% Joint-state callback
%% =========================================================
function jointStateCallback(msg)
    q = double(msg.position(:));
    if numel(q) ~= 6
        return
    end

    qDeg = rad2deg(q);
    fprintf( ...
        "J1=%7.2f  J2=%7.2f  J3=%7.2f  J4=%7.2f  J5=%7.2f  J6=%7.2f deg\n", ...
        qDeg(1), qDeg(2), qDeg(3), ...
        qDeg(4), qDeg(5), qDeg(6));
end
