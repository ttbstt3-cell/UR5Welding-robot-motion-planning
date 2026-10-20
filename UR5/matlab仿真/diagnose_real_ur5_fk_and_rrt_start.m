clear;
clc;

%% ============================================================
% diagnose_real_ur5_fk_and_rrt_start
%
% 只读诊断，不发送任何运动指令：
%   1) 读取真机 /joint_states；
%   2) 使用 MATLAB universalUR5 做正运动学；
%   3) 与 /tcp_pose_broadcaster/pose 比较 TCP；
%   4) 与现有 RRT 路径起点比较初始关节跳变。
%
% 重要坐标约定：
%   真机 ROS2 驱动的 joint_states 使用标准 URDF 关节坐标，
%   直接作为 universalUR5 的 q。不要对真机 q 加 CoppeliaSim 的
%   qOffset；qOffset 只用于旧的 CoppeliaSim -> MATLAB 映射。
%% ============================================================

matlabDir = fileparts(mfilename("fullpath"));
rootDir = fileparts(matlabDir);

calibrationFile = fullfile(rootDir,"ur5_tool_calibration.mat");
rrtFile = fullfile(matlabDir,"weld_rrt_connect_approach_exit.mat");
commandFile = fullfile(matlabDir,"weld_rrt_connect_full_path_command.mat");
calibratedUrdf = fullfile(rootDir,"real_robot_description_calibrated.urdf");
realCalibrationYaml = "/home/ttbstt/ur5_calibration.yaml";

if isfile(calibratedUrdf)
    robot = importrobot(calibratedUrdf);
    robot.DataFormat = "row";
    robotModelSource = calibratedUrdf;
else
    warning("未找到驱动生成的 calibrated URDF，退回 universalUR5；请先运行 read_robot_description_readonly.py。");
    robot = loadrobot("universalUR5","DataFormat","row");
    robotModelSource = "MATLAB universalUR5 fallback";
end

jointNames = [
    "shoulder_pan_joint"
    "shoulder_lift_joint"
    "elbow_joint"
    "wrist_1_joint"
    "wrist_2_joint"
    "wrist_3_joint"
];

fprintf("\n========================================\n");
fprintf("REAL UR5 FK / RRT START DIAGNOSTIC\n");
fprintf("========================================\n");
fprintf("Kinematics YAML (driver launch): %s\n",realCalibrationYaml);
fprintf("MATLAB robot model: %s\n",robotModelSource);

%% 1. 读取真机关节状态
node = ros2node("/matlab_real_ur5_fk_diagnostic");
jointSub = ros2subscriber( ...
    node, "/joint_states", "sensor_msgs/JointState");
tcpSub = ros2subscriber( ...
    node, "/tcp_pose_broadcaster/pose", "geometry_msgs/PoseStamped");

fprintf("Waiting for real /joint_states ...\n");
jointMsg = receive(jointSub,10);

rosNames = string(jointMsg.name);
indexMap = zeros(1,6);
for i = 1:6
    idx = find(rosNames == jointNames(i),1);
    if isempty(idx)
        error("/joint_states 中找不到关节：%s",jointNames(i));
    end
    indexMap(i) = idx;
end

qReal = double(jointMsg.position(:).');
qReal = qReal(indexMap);

fprintf("\nReal UR5 q (canonical ROS/URDF order) [rad]:\n");
disp(qReal);
fprintf("Real UR5 q [deg]:\n");
disp(rad2deg(qReal));
fprintf("Joint name map: ");
disp(indexMap);

%% 2. MATLAB universalUR5 tool0 FK
TBaseTool0 = getTransform(robot,qReal,"tool0");

fprintf("MATLAB FK: base -> tool0\n");
fprintf("position [m] = ");
disp(tform2trvec(TBaseTool0));
fprintf("quaternion [w x y z] = ");
disp(tform2quat(TBaseTool0));

%% 3. 读取驱动报告的 TCP 位姿
fprintf("Waiting for /tcp_pose_broadcaster/pose ...\n");
tcpMsg = receive(tcpSub,10);
TBaseTcpDriver = poseStampedMsgToTform(tcpMsg);

fprintf("Driver reported TCP frame = %s\n",string(tcpMsg.header.frame_id));
fprintf("Driver TCP position [m] = ");
disp(tform2trvec(TBaseTcpDriver));
fprintf("Driver TCP quaternion [w x y z] = ");
disp(tform2quat(TBaseTcpDriver));

% ROS UR 驱动中 base 与 base_link 的固定关系为 Rz(pi)。
% universalUR5 的根坐标对应 base_link；先把驱动 TCP 从 base
% 转到 base_link，再与 MATLAB FK 比较。
TBaseBaseLink = eye(4);
TBaseBaseLink(1:2,1:2) = [-1 0; 0 -1];
TBaseLinkTcpDriver = TBaseBaseLink \ TBaseTcpDriver;

fprintf("Driver TCP expressed in MATLAB/base_link frame:\n");
fprintf("position [m] = ");
disp(tform2trvec(TBaseLinkTcpDriver));
fprintf("quaternion [w x y z] = ");
disp(tform2quat(TBaseLinkTcpDriver));

%% 4. 使用当前 MATLAB 工具标定预测 WeldTCP
if isfile(calibrationFile)
    calib = load(calibrationFile);
    required = ["T_tool0_mount","T_mount_tcp"];
    for i = 1:numel(required)
        if ~isfield(calib,required(i))
            error("标定文件缺少字段：%s",required(i));
        end
    end

    TTool0WeldTcp = double(calib.T_tool0_mount) * ...
        double(calib.T_mount_tcp);
    TBaseTcpMatlab = TBaseTool0 * TTool0WeldTcp;

    fprintf("\nMATLAB calibrated WeldTCP prediction\n");
    fprintf("position [m] = ");
    disp(tform2trvec(TBaseTcpMatlab));
    fprintf("quaternion [w x y z] = ");
    disp(tform2quat(TBaseTcpMatlab));

    posErrM = norm(tform2trvec(TBaseTcpMatlab) - ...
        tform2trvec(TBaseLinkTcpDriver));
    oriErrDeg = rotationErrorDeg(TBaseTcpMatlab,TBaseLinkTcpDriver);

    fprintf("MATLAB calibrated TCP vs driver TCP:\n");
    fprintf("  position error     = %.3f mm\n",posErrM*1000);
    fprintf("  orientation error  = %.3f deg\n",oriErrDeg);

    % 由同一时刻 tool0 和 driver TCP 反推出驱动当前工具变换。
    TTool0TcpDriver = TBaseTool0 \ TBaseLinkTcpDriver;
    fprintf("\nImplied driver tool0 -> TCP transform:\n");
    fprintf("position [m] = ");
    disp(tform2trvec(TTool0TcpDriver));
    fprintf("quaternion [w x y z] = ");
    disp(tform2quat(TTool0TcpDriver));
    fprintf("Configured MATLAB tool0 -> WeldTCP transform:\n");
    fprintf("position [m] = ");
    disp(tform2trvec(TTool0WeldTcp));
    fprintf("quaternion [w x y z] = ");
    disp(tform2quat(TTool0WeldTcp));
    fprintf("tool0->TCP transform difference: %.3f mm, %.3f deg\n", ...
        norm(tform2trvec(TTool0TcpDriver)-tform2trvec(TTool0WeldTcp))*1000, ...
        rotationErrorDeg(TTool0TcpDriver,TTool0WeldTcp));

    % 用户提供的示教器四点 TCP 参数：位置 mm，旋转向量 rad。
    % UR 的 RX/RY/RZ 是旋转向量，不是 XYZ 欧拉角。
    pendantTcp = [ ...
         13.53, 17.13, 379.83, ...
         -0.9562, 2.4566, 1.2418];
    TTool0TcpPendant = poseVectorToTform(pendantTcp);
    fprintf("\nPendant TCP parameter (provided photo):\n");
    fprintf("position [mm] = ");
    disp(pendantTcp(1:3));
    fprintf("rotation vector [rad] = ");
    disp(pendantTcp(4:6));
    fprintf("rotation-vector norm [deg] = %.3f\n", ...
        rad2deg(norm(pendantTcp(4:6))));
    fprintf("Pendant tool0 -> TCP transform:\n");
    fprintf("position [mm] = ");
    disp(1000*tform2trvec(TTool0TcpPendant));
    fprintf("quaternion [w x y z] = ");
    disp(tform2quat(TTool0TcpPendant));
    fprintf("Driver-implied TCP vs pendant TCP:\n");
    fprintf("  position difference = %.3f mm\n", ...
        norm(tform2trvec(TTool0TcpDriver)-tform2trvec(TTool0TcpPendant))*1000);
    fprintf("  orientation difference = %.3f deg\n", ...
        rotationErrorDeg(TTool0TcpDriver,TTool0TcpPendant));
    fprintf("MATLAB WeldTCP vs pendant TCP:\n");
    fprintf("  position difference = %.3f mm\n", ...
        norm(tform2trvec(TTool0WeldTcp)-tform2trvec(TTool0TcpPendant))*1000);
    fprintf("  orientation difference = %.3f deg\n", ...
        rotationErrorDeg(TTool0WeldTcp,TTool0TcpPendant));
else
    warning("未找到标定文件，跳过 WeldTCP 比较：%s",calibrationFile);
    posErrM = NaN;
    oriErrDeg = NaN;
end

%% 5. 读取现有 RRT 路径起点并比较
qPathStart = [];
qPathStartLabel = "";

if isfile(rrtFile)
    s = load(rrtFile,"result");
    if isfield(s.result,"qHome")
        qPathStart = double(s.result.qHome(:).');
        qPathStartLabel = "result.qHome";
    elseif isfield(s.result,"approach") && ...
            isfield(s.result.approach,"qPath")
        qPathStart = double(s.result.approach.qPath(1,:));
        qPathStartLabel = "result.approach.qPath(1,:)";
    end
end

if isempty(qPathStart) && isfile(commandFile)
    s = load(commandFile,"command");
    if isfield(s.command,"qMatRaw")
        qPathStart = double(s.command.qMatRaw(1,:));
        qPathStartLabel = "command.qMatRaw(1,:)";
    end
end

if ~isempty(qPathStart)
    dq = shortestJointDelta(qReal,qPathStart);
    fprintf("\nExisting RRT start (%s) [deg]:\n",qPathStartLabel);
    disp(rad2deg(qPathStart));
    fprintf("Real q - RRT start shortest delta [deg]:\n");
    disp(rad2deg(dq));
    fprintf("max initial joint jump = %.3f deg\n",max(abs(rad2deg(dq))));
    fprintf("initial joint-space distance = %.3f deg (2-norm)\n",norm(rad2deg(dq)));
else
    warning("没有找到现有 RRT 路径起点。");
end

fprintf("\nInterpretation:\n");
fprintf("  - 真机 q 直接使用 ROS/URDF 关节坐标；不要加 Coppelia qOffset。\n");
fprintf("  - MATLAB universalUR5 根坐标按 base_link 处理；驱动 TCP 已转换到 base_link。\n");
fprintf("  - 若 TCP 误差较大，先确认示教器 TCP 与驱动 TCP 配置是否就是同一工具。\n");
fprintf("  - 当前脚本只诊断，不发送 trajectory，也不改变控制器状态。\n");

%% ============================================================
% Local functions
%% ============================================================

function T = poseStampedMsgToTform(msg)
    p = [double(msg.pose.position.x); ...
         double(msg.pose.position.y); ...
         double(msg.pose.position.z)];
    % ROS geometry_msgs quaternion order is x,y,z,w;
    % MATLAB quat2rotm expects w,x,y,z.
    qWxyz = [double(msg.pose.orientation.w), ...
             double(msg.pose.orientation.x), ...
             double(msg.pose.orientation.y), ...
             double(msg.pose.orientation.z)];
    T = eye(4);
    T(1:3,1:3) = quat2rotm(qWxyz);
    T(1:3,4) = p;
end

function dq = shortestJointDelta(qA,qB)
    dq = atan2(sin(qA-qB),cos(qA-qB));
end

function errDeg = rotationErrorDeg(Ta,Tb)
    R = Ta(1:3,1:3).' * Tb(1:3,1:3);
    c = (trace(R)-1)/2;
    c = min(1,max(-1,c));
    errDeg = rad2deg(acos(c));
end

function T = poseVectorToTform(v)
    % UR pose vector: [x y z rx ry rz], position in metres here.
    p = double(v(1:3))*1e-3;
    rv = double(v(4:6)).';
    a = norm(rv);
    if a < 1e-12
        R = eye(3);
    else
        axis = rv/a;
        K = [0 -axis(3) axis(2); ...
             axis(3) 0 -axis(1); ...
             -axis(2) axis(1) 0];
        R = eye(3)*cos(a) + ...
            (1-cos(a))*(axis*axis.') + sin(a)*K;
    end
    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4) = p;
end
