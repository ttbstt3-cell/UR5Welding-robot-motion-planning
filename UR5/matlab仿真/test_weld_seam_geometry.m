clear;
clc;
close all;

%% ============================================================
% 1. 建立当前规划场景
%% ============================================================

run('/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m');


%% ============================================================
% 2. 第一条焊缝：
% bottom × wall_y1
%
% bottom 顶面：
% z = 0.005
%
% wall_y1 内表面：
% y = 0.230
%
% side walls 内表面：
% x = 0.005 ~ 0.230
%
% 两端各留 10 mm
%% ============================================================

endMargin = 0.010;

xMinInner = 0.005;
xMaxInner = 0.230;

ySeam = 0.230;
zSeam = 0.005;

pStartWP = [
    xMinInner + endMargin, ...
    ySeam, ...
    zSeam
];

pEndWP = [
    xMaxInner - endMargin, ...
    ySeam, ...
    zSeam
];


fprintf("\n========================================\n");
fprintf("WELD SEAM DEFINITION\n");
fprintf("========================================\n");

fprintf("pStartWP = [%.4f %.4f %.4f] m\n", ...
    pStartWP);

fprintf("pEndWP   = [%.4f %.4f %.4f] m\n", ...
    pEndWP);

fprintf("Weld length = %.1f mm\n", ...
    norm(pEndWP - pStartWP) * 1000);


%% ============================================================
% 3. TCP焊接姿态
%
% +X_TCP：
%   沿焊缝运动方向
%
% +Z_TCP：
%   从自由空间指向 bottom-wall_y1 内角
%
% 即：
%
%        wall
%         |
%         | ↘ +Z_TCP
%         |  \
%         |   *
% --------+---------- bottom
%
%% ============================================================

xTCP = [1; 0; 0];

zTCP = [0; 1; -1];
zTCP = zTCP / norm(zTCP);

yTCP = cross(zTCP, xTCP);
yTCP = yTCP / norm(yTCP);

RtcpWP = [
    xTCP, ...
    yTCP, ...
    zTCP
];


fprintf("\nRtcpWP =\n");
disp(RtcpWP);

fprintf("det(RtcpWP) = %.6f\n", ...
    det(RtcpWP));


%% ============================================================
% 4. 生成焊缝 TCP 位姿
%% ============================================================

numPoints = 80;
tcpStandoff = 0;       % WeldTCP 已定义为实际焊接作用点

seam = build_weld_seam( ...
    scene, ...
    pStartWP, ...
    pEndWP, ...
    RtcpWP, ...
    numPoints, ...
    tcpStandoff);


%% ============================================================
% 5. 复用 sync_ur5_joint_states.m 的主显示窗口
%% ============================================================

% sync_ur5_joint_states.m 已经创建了唯一的规划场景 Figure。
% 这里不重新 show UR5/Workpiece，只获取原有 Figure/axes 叠加焊缝。
fig = gcf;
ax = gca;
hold(ax, "on");

%% ============================================================
% 6. 红色焊缝
%% ============================================================

% 实际焊缝（红色）与 TCP 退让轨迹（蓝色虚线）分开显示。
plot3( ...
    ax, ...
    seam.positionsMat(:,1), ...
    seam.positionsMat(:,2), ...
    seam.positionsMat(:,3), ...
    "--", ...
    "Color", [0.15 0.35 0.90], ...
    "LineWidth", 1.5);

plot3( ...
    ax, ...
    seam.positionsSeamMat(:,1), ...
    seam.positionsSeamMat(:,2), ...
    seam.positionsSeamMat(:,3), ...
    "Color", [0.85 0 0], ...
    "LineWidth", 4);


%% 起点
plot3( ...
    ax, ...
    seam.positionsSeamMat(1,1), ...
    seam.positionsSeamMat(1,2), ...
    seam.positionsSeamMat(1,3), ...
    "o", ...
    "MarkerSize", 9, ...
    "MarkerFaceColor", [0 0.7 0], ...
    "MarkerEdgeColor", "k");


%% 终点
plot3( ...
    ax, ...
    seam.positionsSeamMat(end,1), ...
    seam.positionsSeamMat(end,2), ...
    seam.positionsSeamMat(end,3), ...
    "o", ...
    "MarkerSize", 9, ...
    "MarkerFaceColor", [0 0.3 1], ...
    "MarkerEdgeColor", "k");


%% ============================================================
% 7. 在焊缝中点画 TCP 坐标轴
%% ============================================================

idx = round(numPoints/2);

Tmid = seam.tcpPoses(:,:,idx);

p = Tmid(1:3,4);
R = Tmid(1:3,1:3);

L = 0.060;


%% TCP X
quiver3( ...
    ax, ...
    p(1), p(2), p(3), ...
    L*R(1,1), ...
    L*R(2,1), ...
    L*R(3,1), ...
    0, ...
    "Color", [1 0 0], ...
    "LineWidth", 2.5);


%% TCP Y
quiver3( ...
    ax, ...
    p(1), p(2), p(3), ...
    L*R(1,2), ...
    L*R(2,2), ...
    L*R(3,2), ...
    0, ...
    "Color", [0 0.65 0], ...
    "LineWidth", 2.5);


%% TCP Z / welding gun axis
quiver3( ...
    ax, ...
    p(1), p(2), p(3), ...
    L*R(1,3), ...
    L*R(2,3), ...
    L*R(3,3), ...
    0, ...
    "Color", [0 0.25 1], ...
    "LineWidth", 3);


% 保留主场景原有的光照、材质、视角和坐标轴设置。
drawnow;


%% ============================================================
% 9. 输出焊缝在 MATLAB base 中的起点/终点
%% ============================================================

fprintf("\n========================================\n");
fprintf("WELD SEAM IN MATLAB BASE\n");
fprintf("========================================\n");

fprintf("Start [m] = ");
disp(seam.positionsSeamMat(1,:));

fprintf("End   [m] = ");
disp(seam.positionsSeamMat(end,:));

fprintf("TCP start [m] = ");
disp(seam.positionsMat(1,:));

fprintf("TCP end   [m] = ");
disp(seam.positionsMat(end,:));

fprintf("TCP standoff = %.1f mm\n", seam.standoff_m*1000);

fprintf("\nGeometry generation complete.\n");
