function seam = build_weld_seam(scene, pStartWP, pEndWP, RtcpWP, numPoints, standoff_m)

%BUILD_WELD_SEAM
% 在 Workpiece 局部坐标系中生成直线焊缝和带 standoff 的 TCP 轨迹。
%
% 几何焊缝点与 TCP 目标点是两个不同的概念：
%   P_seam = P_TCP + standoff_m * Z_TCP
% 因此：
%   P_TCP  = P_seam - standoff_m * Z_TCP
%
% 输入：
% scene       当前规划场景
% pStartWP    实际焊缝起点，Workpiece 坐标系 [x y z]，单位 m
% pEndWP      实际焊缝终点，Workpiece 坐标系 [x y z]，单位 m
% RtcpWP      TCP 相对于 Workpiece 的固定旋转矩阵 3x3；第3列是 +Z_TCP
% numPoints   焊缝离散点数量
% standoff_m  TCP 沿 -Z_TCP 相对实际焊缝的退让距离，默认 0 m。
%             需要 18 mm 时由调用方显式传入 0.018。
%
% 输出：
% seam.positionsSeamWP   实际焊缝点（Workpiece 坐标系）
% seam.positionsTCPWP    TCP 目标点（Workpiece 坐标系）
% seam.positionsSeamMat  实际焊缝点（MATLAB base 坐标系）
% seam.positionsMat      TCP 目标点（MATLAB base 坐标系）
% seam.tcpPoses          TCP 目标位姿（MATLAB base 坐标系）
% seam.startPose         第一个 TCP 目标位姿
% seam.endPose           最后一个 TCP 目标位姿

    arguments
        scene
        pStartWP (1,3) double
        pEndWP (1,3) double
        RtcpWP (3,3) double
        numPoints (1,1) double = 80
        standoff_m (1,1) double = 0
    end

    if numPoints < 2 || numPoints ~= fix(numPoints)
        error("numPoints 必须是大于等于 2 的整数。");
    end

    if standoff_m < 0
        error("standoff_m 不能为负数。");
    end

    %% --------------------------------------------------------
    % 1. Workpiece -> MATLAB base
    %% --------------------------------------------------------

    T_B_W = scene.T_MatBase_Workpiece;

    %% --------------------------------------------------------
    % 2. 直线插值
    %% --------------------------------------------------------

    s = linspace(0,1,numPoints)';

    positionsSeamWP = ...
        (1-s).*pStartWP + ...
        s.*pEndWP;

    % 固定 TCP 姿态的 +Z 轴指向焊缝。
    zTCP_WP = RtcpWP(:,3);
    zNorm = norm(zTCP_WP);

    if zNorm < eps
        error("RtcpWP 的第3列不能是零向量。");
    end

    % 允许调用方传入有微小数值误差的旋转矩阵。
    zTCP_WP = zTCP_WP / zNorm;

    % 实际焊缝点位于 TCP 前方 standoff_m，因此 TCP 沿 -Z_TCP 退让。
    positionsTCPWP = ...
        positionsSeamWP - ...
        standoff_m * zTCP_WP.';

    tcpPoses = zeros(4,4,numPoints);
    positionsSeamMat = zeros(numPoints,3);
    positionsMat = zeros(numPoints,3);

    %% --------------------------------------------------------
    % 3. 生成每个 TCP 位姿
    %% --------------------------------------------------------

    for i = 1:numPoints

        T_W_TCP = eye(4);

        T_W_TCP(1:3,1:3) = RtcpWP;
        T_W_TCP(1:3,4) = positionsTCPWP(i,:)';

        % 转到 MATLAB base
        T_B_TCP = T_B_W * T_W_TCP;

        tcpPoses(:,:,i) = T_B_TCP;
        pSeamB = T_B_W * [positionsSeamWP(i,:) 1].';
        positionsSeamMat(i,:) = pSeamB(1:3).';
        positionsMat(i,:) = T_B_TCP(1:3,4)';

    end

    %% --------------------------------------------------------
    % 4. 输出结构体
    %% --------------------------------------------------------

    seam = struct();

    seam.positionsSeamWP = positionsSeamWP;
    seam.positionsTCPWP = positionsTCPWP;

    % positionsMat 保持为 TCP 在 MATLAB base 中的轨迹。
    seam.positionsSeamMat = positionsSeamMat;
    seam.positionsTCPMat = positionsMat;
    seam.positionsMat = positionsMat;

    % 兼容旧调用方：历史 positionsWP 表示的是 TCP目标点。
    seam.positionsWP = positionsTCPWP;

    seam.tcpPoses = tcpPoses;

    seam.startPose = tcpPoses(:,:,1);
    seam.endPose = tcpPoses(:,:,end);

    seam.pStartWP = pStartWP;
    seam.pEndWP = pEndWP;

    seam.RtcpWP = RtcpWP;
    seam.zTCP_WP = zTCP_WP;
    seam.standoff_m = standoff_m;
    seam.standoff = standoff_m;
    seam.pStartTCPWP = positionsTCPWP(1,:);
    seam.pEndTCPWP = positionsTCPWP(end,:);

    seam.numPoints = numPoints;

end
