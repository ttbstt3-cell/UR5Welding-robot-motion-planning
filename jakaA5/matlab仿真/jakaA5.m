clc;
clear;
close all;

%% =====================================================
%  JAKA A5 机器人基础模型建立
%  MATLAB Robotics System Toolbox
%
%  功能：
%  URDF导入
%  TCP建立
%  FK/IK测试
%  环境障碍建模
%  碰撞检测
%
% ======================================================


%% 1. 导入URDF

urdfPath = ...
"C:\Users\ASUS\Desktop\JIXIEBI\JAKA_A5_URDF_download\jaka_ros2-main\src\jaka_description\urdf\jaka_a5.urdf";


robot = importrobot(urdfPath);


% 数据格式
robot.DataFormat = "row";


% 设置重力
robot.Gravity = [0 0 -9.81];


disp("========== Robot Loaded ==========")

showdetails(robot)



%% =====================================================
% 2. 添加焊枪TCP
% 
% J6
% |
% TCP
% |
% Welding Torch
%
% =====================================================


tcp = rigidBody("TCP");


tcpJoint = rigidBodyJoint("tcp_fixed","fixed");


% TCP相对于J6的位置
% 根据你的焊枪长度修改

tcpOffset = trvec2tform([0 0 0.15]);


setFixedTransform(tcpJoint,tcpOffset);


tcp.Joint = tcpJoint;


addBody(robot,tcp,"J6");



disp("TCP added")



%% =====================================================
% 3. 显示机器人
% =====================================================


figure;

show(robot,...
    "Frames","on",...
    "PreservePlot",false);


title("JAKA A5 Robot Model");

view(135,25)

grid on



%% =====================================================
% 4. 正运动学测试 FK
%
% 给定关节角
% 求TCP位置
%
% =====================================================


qHome = zeros(1,6);


T_TCP = getTransform(...
    robot,...
    qHome,...
    "TCP");


disp("========== TCP Pose ==========")

disp(T_TCP)



tcpPosition = tform2trvec(T_TCP);


disp("TCP position:")
disp(tcpPosition)



%% =====================================================
% 5. 逆运动学 IK
%
% 目标:
% TCP移动到焊接起点
%
% =====================================================


ik = inverseKinematics(...
    "RigidBodyTree",robot);


weights = ...
    [0.25 0.25 0.25 1 1 1];


% 模拟 WeldStart

targetXYZ = [0.4 0.2 0.5];


targetPose = ...
    trvec2tform(targetXYZ);



initialGuess = qHome;



[qSol,solInfo] = ik(...
    "TCP",...
    targetPose,...
    weights,...
    initialGuess);



disp("========== IK Result ==========")

disp(qSol)

disp(solInfo.Status)



%% =====================================================
% 6. 显示IK结果
% =====================================================


figure;

show(robot,...
    qSol,...
    "Frames","on");


title("IK Solution - WeldStart");

view(135,25)

grid on




%% =====================================================
% 7. 建立船舶狭小空间障碍物
%
% 模拟：
% 架子
% 船舱结构
% 管路
%
% =====================================================


% 障碍物1：支架

rack = collisionBox(...
    0.6,...
    0.3,...
    1.0);


rack.Pose = ...
    trvec2tform([0.5 0 0.5]);



% 障碍物2：管道

pipe = collisionCylinder(...
    0.08,...
    1.2);


pipe.Pose = ...
    trvec2tform([0.2 0.4 0.6]);



disp("Obstacle created")



%% =====================================================
% 8. 显示环境
% =====================================================


figure;

show(robot,qHome);

hold on


show(rack)

show(pipe)


title("Robot + Ship Cabin Obstacles")


view(135,25)

grid on



%% =====================================================
% 9. 碰撞检测测试
%
% 这是后续RRT*核心
%
% =====================================================


collision1 = checkCollision(...
    robot,...
    qHome,...
    {rack,pipe});


disp("========== Collision Test ==========")


if collision1

    disp("机器人发生碰撞")

else

    disp("当前姿态安全")

end




%% =====================================================
% 10. 创建起点和终点
%
% 给RRT*使用
%
% =====================================================


qStart = zeros(1,6);


% 一个模拟目标姿态

qGoal = qSol;


disp("========== Planning Point ==========")


disp("Start:")
disp(qStart)


disp("Goal:")
disp(qGoal)



%% =====================================================
% 11. 后续RRT接口
%
% 下一步：
%
% qStart
%    |
%    |
% RL-RRT*
%    |
%    |
% qGoal
%
% =====================================================


disp("======================================")
disp("Robot model ready")
disp("Next step: RRT*/RL-RRT* planning")
disp("======================================")