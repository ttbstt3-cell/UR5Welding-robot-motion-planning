function out = animate_weld_actual_axis_replan(resultFile,opts)

%ANIMATE_WELD_ACTUAL_AXIS_REPLAN Animate a saved measured-wire-axis plan.
%
% 只播放已保存的规划结果：不重新求 IK、不重新规划、不发布 ROS2、
% 不修改结果文件、不修改控制器。
%
% 数据来源：
%   weld_actual_axis_replan.mat -> result.qPath / result.scene / result.seam
%
% 关于 2*pi 回绕（重要）：
%   保存的 result.qPath 中大量相邻点处在不同的 2*pi 分支上（raw 相邻差
%   可达 ±360 deg 以上）。直接按 raw 值插值会让关节在焊缝中途整转一圈。
%   本脚本播放前先做 unwrap，并打印发现回绕的位置。unwrap 不改变任何
%   姿态（mod 2*pi 完全等价），只把关节角拉回连续分支。
%
% 用法：
%   animate_weld_actual_axis_replan                       % 默认 3/4 全场景
%   animate_weld_actual_axis_replan("",View="allTop")     % 俯视
%   animate_weld_actual_axis_replan("",View="allSide")    % +X 侧视
%   animate_weld_actual_axis_replan(f,SaveVideo=true,VideoFile="x.avi")
%
% 参数（Name-Value）：
%   StepDeg          每帧允许的最大关节变化，默认 1.0 deg
%   Speed            回放速度倍率，默认 1.0
%   TcpSpeedMps      TCP 沿实际回放轨迹的固定速度，默认 0.043 m/s
%   ShowTrail        是否画 TCP 拖尾，默认 true
%   ShowCollisions   是否显示碰撞模型，默认 true
%   SaveVideo        是否录制，默认 false
%   VideoFile        录像文件，默认 weld_actual_axis_replan.avi
%   View             相机视角："all"（主三维）、"allTop"（俯视）、
%                   "allSide"（+X 侧视）、"allSideNeg"（-X 侧视）、
%                   "allAlt"（反向三维）、"allWeldSide"（高位侧视）、
%                   "allWeld"（焊缝内侧三维）、"allWeldReverse"（焊缝反向三维），默认 "all"
%   MaxFrames        只渲染前 N 帧（试角度用），默认 Inf

    arguments
        resultFile (1,1) string = ""
        opts.StepDeg (1,1) double {mustBePositive} = 1.0
        opts.Speed (1,1) double {mustBePositive} = 1.0
        opts.TcpSpeedMps (1,1) double {mustBePositive} = 0.043
        opts.ShowTrail (1,1) logical = true
        opts.ShowCollisions (1,1) logical = true
        opts.SaveVideo (1,1) logical = false
        opts.VideoFile (1,1) string = "weld_actual_axis_replan.avi"
        opts.View (1,1) string {mustBeMember(opts.View,["all","allTop", ...
            "allSide","allSideNeg","allAlt","allWeldSide", ...
            "allWeld","allWeldReverse"])} = "all"
        opts.MaxFrames (1,1) double {mustBePositive} = Inf
    end

    here = fileparts(mfilename("fullpath"));
    if isempty(here)
        here = pwd;
    end

    if strlength(resultFile) == 0
        resultFile = fullfile(here,"weld_actual_axis_replan.mat");
    end

    if ~isfile(resultFile)
        error("找不到规划结果文件：%s",resultFile);
    end

    fprintf("\n========================================\n");
    fprintf("MEASURED WIRE-AXIS REPLAN ANIMATION\n");
    fprintf("========================================\n");

    %% 1. 读取结果

    s = load(resultFile,"result");
    result = s.result;

    if ~result.complete || isempty(result.qPath)
        error("结果文件中没有完整 qPath，无法播放动画。");
    end

    % 直接复用 sync_ur5_joint_states.m 建立的原始主 Figure。
    % 这样保留原场景的 light、材质和配色；单独 new figure 再 show
    % 会丢失 light，导致颜色变成未受光的亮绿色/亮橙色。
    [scene,fig,ax] = localBuildScene(here);
    seam = result.seam;

    if ~isfield(seam,"positionsMat")
        P = (scene.T_MatBase_Workpiece * ...
            [seam.positionsWP,ones(size(seam.positionsWP,1),1)].').';
        seam.positionsMat = P(:,1:3);
    end

    seamPts = seam.positionsMat;
    qPathRaw = double(result.qPath);
    numPoints = size(qPathRaw,1);

    fprintf("结果文件            : %s\n",resultFile);
    fprintf("焊点数              : %d\n",numPoints);

    %% 2. 2*pi 回绕处理

    [qPath,wrapEdges] = localUnwrapPath(qPathRaw);

    if isempty(wrapEdges)
        fprintf("2*pi 回绕           : 无\n");
    else
        fprintf(2,"2*pi 回绕           : **发现 %d 处**，已按连续分支播放\n", ...
            numel(wrapEdges));
        fprintf(2,"  raw 最大相邻跳变 = %.3f deg，unwrap 后 = %.3f deg\n", ...
            localMaxJump(qPathRaw),localMaxJump(qPath));
        fprintf(2,"  回绕所在相邻对（edge=第 edge 点到 edge+1 点）：%s\n", ...
            strjoin(string(wrapEdges(:).'), ", "));
        fprintf(2,"  注意：result.qPath 本身仍带这些分支，发布前必须先处理。\n");
    end

    %% 3. 生成回放帧（关节空间线性插值）

    frameRate = 30;
    [qFrames,framePointIdx,tcpPathLength,tcpDuration] = ...
        localInterpolatePath(scene,qPath,opts.StepDeg, ...
        opts.TcpSpeedMps,frameRate);
    numFrames = size(qFrames,1);

    fprintf("TCP 固定速度        : %.3f m/s (%.1f mm/s)\n", ...
        opts.TcpSpeedMps,opts.TcpSpeedMps*1000);
    fprintf("TCP 轨迹弧长        : %.3f m\n",tcpPathLength);
    fprintf("时间参数化时长      : %.2f s\n",tcpDuration);
    fprintf("回放帧数            : %d（%.2f s @%d fps）\n", ...
        numFrames,(numFrames-1)/frameRate,frameRate);

    %% 4. 数值复核

    tcpFrames = localTcpPositions(scene,qFrames);
    seamDeviation = max(localSeamDeviation(tcpFrames,seamPts));

    tcpFrameSpeed = vecnorm(diff(tcpFrames,1,1),2,2)*frameRate;
    if isempty(tcpFrameSpeed)
        tcpFrameSpeed = 0;
    end
    fprintf("TCP 帧速度范围      : %.4f .. %.4f m/s（目标 %.4f m/s）\n", ...
        min(tcpFrameSpeed),max(tcpFrameSpeed),opts.TcpSpeedMps);

    fprintf("回放帧 TCP 偏离焊缝 : max %.4f mm\n",seamDeviation*1000);

    frameCollisions = NaN;
    if opts.ShowCollisions
        frameCollisions = 0;
        % 只抽查插值帧，逐帧全查在大场景上很慢。
        probe = unique(round(linspace(1,numFrames,min(20,numFrames))));
        for i = probe
            if localHasEnvironmentCollision(scene,qFrames(i,:))
                frameCollisions = frameCollisions + 1;
                fprintf("  [插值帧碰撞] 第 %d 帧（焊点 %d）\n", ...
                    i,framePointIdx(i));
            end
        end
        fprintf("抽查插值帧碰撞数    : %d / %d\n",frameCollisions,numel(probe));
    end

    %% 5. 画布与静态叠加

    if opts.SaveVideo
        fig.Resize = "off";
    end

    hold(ax,"on");

    % sync 脚本已经在该 Figure 中画好工件碰撞体；这里直接复用，
    % 不再创建第二套工件对象。

    hSeam = plot3(ax,seamPts(:,1),seamPts(:,2),seamPts(:,3), ...
        "r-","LineWidth",4);

    plot3(ax,seamPts(1,1),seamPts(1,2),seamPts(1,3), ...
        "o","MarkerSize",11,"MarkerFaceColor",[0 0.7 0],"MarkerEdgeColor","k");
    plot3(ax,seamPts(end,1),seamPts(end,2),seamPts(end,3), ...
        "o","MarkerSize",11,"MarkerFaceColor",[0 0.3 1],"MarkerEdgeColor","k");

    hTrail = plot3(ax,nan,nan,nan,"-","LineWidth",2, ...
        "Color",[0.95 0.75 0.05]);
    if ~opts.ShowTrail
        hTrail.Visible = "off";
    end

    hTcp = plot3(ax,nan,nan,nan,"o","MarkerSize",9, ...
        "MarkerFaceColor",[1 0.2 0.2],"MarkerEdgeColor","k");

    % 当前实际枪轴（焊丝轴），每帧更新
    hAxis = quiver3(ax,0,0,0,0,0,0,0, ...
        "Color",[0.95 0.55 0.05],"LineWidth",3,"MaxHeadSize",0.6);

    % 焊丝本体显式叠加。碰撞圆柱仍由 scene.robot 的 collision 模型
    % 提供；这里的线段只是为了在全场景视频中清楚显示 1.2 mm 细焊丝。
    hWire = plot3(ax,nan,nan,nan,"-", ...
        "Color",[0.98 0.45 0.05],"LineWidth",4);
    hWireHead = plot3(ax,nan,nan,nan,"o", ...
        "Color",[0.55 0.15 0.02],"MarkerFaceColor",[0.98 0.45 0.05], ...
        "MarkerSize",5);

    % 焊缝是本次动画的重点，即使它位于工件内角，也要在视频中完整可见。
    % 把叠加线放到最上层不会改变机器人/工件的原始颜色和几何，只改变标注的可见性。
    uistack(hSeam,"top");

    %% 6. 全场景自动取景

    bounds = localSceneBounds(ax,scene,qPath,seamPts,tcpFrames,15);
    localApplyBounds(ax,bounds,0.08,opts.View);

    fprintf("相机视角            : %s（全场景自动取景，含 8%% 余量）\n", ...
        opts.View);
    fprintf("  取景包围盒 X [%.3f %.3f] Y [%.3f %.3f] Z [%.3f %.3f] m\n", ...
        bounds(1),bounds(2),bounds(3),bounds(4),bounds(5),bounds(6));

    %% 7. 录制准备

    videoFile = "";
    vw = [];
    if opts.SaveVideo
        videoFile = fullfile(here,opts.VideoFile);
        vw = localOpenVideoWriter(videoFile,frameRate);
        if ~isempty(vw)
            fprintf("\n录制到：%s\n",videoFile);
        end
    end

    %% 8. 回放

    fprintf("\n开始回放 ...（Ctrl+C 可中断）\n");

    framePause = 1/(30*opts.Speed);
    numRender = min(numFrames,opts.MaxFrames);

    if numRender < numFrames
        fprintf("试渲染模式          : 只渲染前 %d / %d 帧\n",numRender,numFrames);
    end

    videoFrameSize = [];
    frameSizeWarned = false;
    axisLength = 0.07;

    for i = 1:numRender

        show(scene.robot,qFrames(i,:), ...
            "Parent",ax, ...
            "Visuals","on", ...
            "Collisions",localOnOff(opts.ShowCollisions), ...
            "Frames","off", ...
            "PreservePlot",false);

        T = getTransform(scene.robot,qFrames(i,:),scene.tcpBodyName);
        p = tform2trvec(T);
        g = tform2rotm(T) * scene.wireAxisTcp(:);

        % T_tcp_wire 是焊丝碰撞圆柱相对 WeldTCP 的固定位姿。
        % 圆柱局部 z 轴沿焊丝，从枪头端 (-L/2) 指向 TCP 端 (+L/2)。
        if isfield(scene,"T_tcp_wire") && isfield(scene,"wireLengthM")
            Twire = T * scene.T_tcp_wire;
            pWireHead = Twire * [0;0;-scene.wireLengthM/2;1];
            pWireTcp = Twire * [0;0; scene.wireLengthM/2;1];
            hWire.XData = [pWireHead(1),pWireTcp(1)];
            hWire.YData = [pWireHead(2),pWireTcp(2)];
            hWire.ZData = [pWireHead(3),pWireTcp(3)];
            hWireHead.XData = pWireHead(1);
            hWireHead.YData = pWireHead(2);
            hWireHead.ZData = pWireHead(3);
        end

        hTcp.XData = p(1); hTcp.YData = p(2); hTcp.ZData = p(3);

        hAxis.XData = p(1); hAxis.YData = p(2); hAxis.ZData = p(3);
        hAxis.UData = axisLength*g(1);
        hAxis.VData = axisLength*g(2);
        hAxis.WData = axisLength*g(3);

        if opts.ShowTrail
            hTrail.XData = tcpFrames(1:i,1);
            hTrail.YData = tcpFrames(1:i,2);
            hTrail.ZData = tcpFrames(1:i,3);
        end

        % show(...,PreservePlot=false) 会重新绘制机器人 patch，确保每一帧
        % 的焊缝、TCP、焊丝和轴标注仍位于最上层，避免被实体遮挡。
        uistack([hSeam,hTrail,hTcp,hAxis,hWire,hWireHead],"top");

        title(ax,sprintf( ...
            "Measured gun-axis weld point %d/%d", ...
            framePointIdx(i),numPoints));

        drawnow;

        if ~isempty(vw)

            frame = getframe(fig);
            frameSize = size(frame.cdata,1:2);

            if isempty(videoFrameSize)
                videoFrameSize = frameSize;
            elseif any(frameSize ~= videoFrameSize)
                frame.cdata = localResampleFrame(frame.cdata,videoFrameSize);
                if ~frameSizeWarned
                    warning("捕获帧尺寸由 %s 变为 %s，已重采样。", ...
                        mat2str(videoFrameSize),mat2str(frameSize));
                    frameSizeWarned = true;
                end
            end

            writeVideo(vw,frame);

        end

        pause(framePause);

    end

    fprintf("回放完成。\n");

    if ~isempty(vw)
        close(vw);
        d = dir(videoFile);
        if ~isempty(d)
            fprintf("录像文件：%s（%.1f MB）\n",videoFile,d.bytes/1e6);
        end
    end

    out = struct();
    out.qPathRaw = qPathRaw;
    out.qPath = qPath;
    out.wrapEdges = wrapEdges;
    out.qFrames = qFrames;
    out.framePointIdx = framePointIdx;
    out.tcpFrames = tcpFrames;
    out.seamPts = seamPts;
    out.seamDeviation = seamDeviation;
    out.frameCollisions = frameCollisions;
    out.tcpPathLength = tcpPathLength;
    out.tcpSpeedMps = opts.TcpSpeedMps;
    out.tcpDuration = tcpDuration;
    out.tcpFrameSpeed = tcpFrameSpeed;
    out.videoFile = videoFile;
    out.figure = fig;
    out.axes = ax;

end


%% =============================================================
% Local functions
%% =============================================================

function [scene,fig,ax] = localBuildScene(here)

    % sync_ur5_joint_states.m 是脚本，会在自己的函数工作区执行 clear/clc/
    % close all；返回的 Figure 就是主场景原始显示窗口。
    run(fullfile(here,"sync_ur5_joint_states.m"));
    fig = gcf;
    ax = gca;

end

function [qPath,edges] = localUnwrapPath(qPathRaw)

    % 把每个关节拉回连续分支。unwrap 只改 2*pi 的整数倍，
    % 不改变任何末端姿态。
    qPath = unwrap(qPathRaw,[],1);

    dRaw = diff(qPathRaw,1,1);
    dWrap = diff(qPath,1,1);

    edges = find(any(abs(rad2deg(dRaw-dWrap)) > 1e-6,2));

end


function m = localMaxJump(qPath)

    if size(qPath,1) < 2
        m = 0;
        return;
    end

    m = max(rad2deg(max(abs(diff(qPath,1,1)),[],2)));

end


function [qFrames,pointIdx,totalLength,duration] = ...
        localInterpolatePath(scene,qPath,stepDeg,tcpSpeedMps,frameRate)

    % 先用较细的关节空间折线近似每个 qPath 段的 TCP 轨迹，再按
    % TCP 弧长均匀采样。这样姿态调整会和焊缝前进同时发生，不会因为
    % 某一段关节变化较大而在焊点处停顿。
    numPoints = size(qPath,1);
    denseStepDeg = max(stepDeg/8,0.05);

    qDense = qPath(1,:);
    pointDense = 1;

    for i = 1:numPoints-1

        dq = qPath(i+1,:) - qPath(i,:);
        numSub = max(1,ceil(rad2deg(max(abs(dq)))/denseStepDeg));

        for k = 1:numSub
            t = k/numSub;
            qDense(end+1,:) = qPath(i,:) + t*dq; %#ok<AGROW>
            pointDense(end+1,1) = i + 1; %#ok<AGROW>
        end

    end

    tcpDense = zeros(size(qDense,1),3);
    for i = 1:size(qDense,1)
        T = getTransform(scene.robot,qDense(i,:),scene.tcpBodyName);
        tcpDense(i,:) = tform2trvec(T);
    end

    ds = vecnorm(diff(tcpDense,1,1),2,2);
    arc = [0;cumsum(ds)];

    % interp1 要求自变量严格递增；完全重合的 TCP 采样点只保留最后
    % 一个姿态，避免生成零速度重复帧。
    [arcUnique,keep] = unique(arc,"stable");
    qUnique = qDense(keep,:);
    pointUnique = pointDense(keep);

    totalLength = arcUnique(end);
    if totalLength <= eps
        qFrames = qPath(1,:);
        pointIdx = 1;
        duration = 0;
        return;
    end

    requestedDuration = totalLength/tcpSpeedMps;
    % 使用整数个视频帧，并在整个区间内等时长采样，避免最后一帧
    % 变成短帧而造成肉眼可见的停顿。实际速度与目标速度的差异
    % 仅来自帧率量化（通常小于 0.5%%）。
    numIntervals = max(1,round(requestedDuration*frameRate));
    duration = numIntervals/frameRate;
    targetArc = linspace(0,totalLength,numIntervals+1).';

    qFrames = zeros(numel(targetArc),size(qPath,2));
    for j = 1:size(qPath,2)
        qFrames(:,j) = interp1(arcUnique,qUnique(:,j),targetArc, ...
            "linear","extrap");
    end

    % TCP 对 q 的映射是非线性的。再用生成出的帧重新计算弧长并迭代
    % 重采样几次，可消除关节线性插值在局部造成的速度凹陷，同时保留
    % 起点/终点和完整姿态变化。每一轮都依据当前弧长重新量化视频
    % 帧数，避免初始密集采样弧长与最终 TCP 折线弧长不同而造成
    % 整段速度偏低。
    for pass = 1:10
        tcpFrames = zeros(size(qFrames,1),3);
        for i = 1:size(qFrames,1)
            T = getTransform(scene.robot,qFrames(i,:),scene.tcpBodyName);
            tcpFrames(i,:) = tform2trvec(T);
        end

        frameArc = [0;cumsum(vecnorm(diff(tcpFrames,1,1),2,2))];
        [frameArcUnique,frameKeep] = unique(frameArc,"stable");
        qFrameUnique = qFrames(frameKeep,:);

        frameLength = frameArcUnique(end);
        numIntervals = max(1,round(frameLength/tcpSpeedMps*frameRate));
        duration = numIntervals/frameRate;
        targetFrameArc = linspace(0,frameLength,numIntervals+1).';

        qNext = zeros(numel(targetFrameArc),size(qFrames,2));
        for j = 1:size(qFrames,2)
            qNext(:,j) = interp1(frameArcUnique,qFrameUnique(:,j), ...
                targetFrameArc,"linear","extrap");
        end
        qNext(1,:) = qPath(1,:);
        qNext(end,:) = qPath(end,:);
        qFrames = qNext;
    end

    % 用最终帧重新计算实际弧长；它与初始弧长只会有很小的数值差。
    tcpFrames = zeros(size(qFrames,1),3);
    for i = 1:size(qFrames,1)
        T = getTransform(scene.robot,qFrames(i,:),scene.tcpBodyName);
        tcpFrames(i,:) = tform2trvec(T);
    end
    totalLength = sum(vecnorm(diff(tcpFrames,1,1),2,2));

    % 最终帧数可能因弧长重算而变化；用最终帧数在原始 TCP 弧长上
    % 建立焊点索引，确保标题和拖尾索引仍与回放帧一一对应。
    targetPointArc = linspace(0,arcUnique(end),size(qFrames,1)).';
    pointIdx = interp1(arcUnique,double(pointUnique),targetPointArc, ...
        "nearest","extrap");
    pointIdx = round(pointIdx(:));

end


function p = localTcpPositions(scene,qFrames)

    p = zeros(size(qFrames,1),3);

    for i = 1:size(qFrames,1)
        T = getTransform(scene.robot,qFrames(i,:),scene.tcpBodyName);
        p(i,:) = tform2trvec(T);
    end

end


function d = localSeamDeviation(points,seamPts)

    a = seamPts(1,:);
    b = seamPts(end,:);
    ab = b - a;
    denom = dot(ab,ab);

    d = zeros(size(points,1),1);

    for i = 1:size(points,1)
        ap = points(i,:) - a;
        t = min(max(dot(ap,ab)/denom,0),1);
        d(i) = norm(ap - t*ab);
    end

end


function tf = localHasEnvironmentCollision(scene,q)

    try
        result = checkCollision( ...
            scene.robot,q,scene.environment, ...
            "IgnoreSelfCollision","on", ...
            "Exhaustive","on");
        tf = logical(result);
    catch
        tf = false;
    end

end


function bounds = localSceneBounds(ax,scene,qPath,seamPts,tcpFrames,numSamples)

    % 采样若干帧，取所有 patch 顶点的并集，再加上焊缝与 TCP 轨迹。
    % 必须自己算：show() 只按机器人本体设轴范围，工件会被裁在画面外。

    bounds = [inf -inf inf -inf inf -inf];
    idx = unique(round(linspace(1,size(qPath,1),numSamples)));

    for i = idx
        show(scene.robot,qPath(i,:), ...
            "Parent",ax, ...
            "Visuals","on", ...
            "Collisions","on", ...
            "Frames","off", ...
            "PreservePlot",false);
        bounds = localUnionPatchBounds(ax,bounds);
    end

    bounds = localUnionPoints(bounds,seamPts);
    bounds = localUnionPoints(bounds,tcpFrames);

end


function bounds = localUnionPatchBounds(ax,bounds)

    for h = findobj(ax,"Type","patch").'
        bounds = localUnionPoints(bounds,h.Vertices);
    end

end


function bounds = localUnionPoints(bounds,P)

    if isempty(P)
        return;
    end

    bounds(1) = min(bounds(1),min(P(:,1)));
    bounds(2) = max(bounds(2),max(P(:,1)));
    bounds(3) = min(bounds(3),min(P(:,2)));
    bounds(4) = max(bounds(4),max(P(:,2)));
    bounds(5) = min(bounds(5),min(P(:,3)));
    bounds(6) = max(bounds(6),max(P(:,3)));

end


function localApplyBounds(ax,bounds,marginFrac,viewName)

    span = [bounds(2)-bounds(1), bounds(4)-bounds(3), bounds(6)-bounds(5)];
    pad = marginFrac*span;

    xlim(ax,[bounds(1)-pad(1), bounds(2)+pad(1)]);
    ylim(ax,[bounds(3)-pad(2), bounds(4)+pad(2)]);
    zlim(ax,[bounds(5)-pad(3), bounds(6)+pad(3)]);

    daspect(ax,[1 1 1]);
    grid(ax,"on");

    xlabel(ax,"MATLAB base X (m)");
    ylabel(ax,"MATLAB base Y (m)");
    zlabel(ax,"MATLAB base Z (m)");

    switch viewName
        case "all"
            % 与 sync_ur5_joint_states.m 建立主场景时的原始三维视角一致，
            % 使机器人/工件的受光面和颜色观感保持不变。
            view(ax,135,25);
        case "allTop"
            view(ax,0,90);
        case "allSide"
            % 从 +X 侧观察，焊缝（MATLAB base +Y）呈纵向，
            % 同时可见焊枪沿焊缝运动。
            view(ax,90,18);
        case "allSideNeg"
            % 从 -X 侧观察，作为侧视反向检查。
            view(ax,-90,18);
        case "allAlt"
            % 反向三维视角，避免主视角遮挡焊枪或焊缝。
            view(ax,-45,25);
        case "allWeldSide"
            % 较高的侧视角：可以同时看到工件内角焊缝和末端焊枪。
            view(ax,90,55);
        case "allWeld"
            % 从焊缝开口侧的高位三维视角观察焊枪沿缝运动。
            view(ax,120,55);
        case "allWeldReverse"
            % 反向高位三维视角，用于检查焊枪另一侧和整条焊缝。
            view(ax,140,55);
    end

end


function cdata = localResampleFrame(cdata,targetSize)

    % 最近邻重采样，避免依赖 Image Processing Toolbox 的 imresize。
    rows = round(linspace(1,size(cdata,1),targetSize(1)));
    cols = round(linspace(1,size(cdata,2),targetSize(2)));

    cdata = cdata(rows,cols,:);

end


function s = localOnOff(tf)

    if tf
        s = "on";
    else
        s = "off";
    end

end


function vw = localOpenVideoWriter(videoFile,frameRate)

    % 该 Linux MATLAB 没有注册 MPEG-4/H.264 profile，
    % 因此默认使用可用的 Motion JPEG AVI；若能写入 MPEG-4 则优先。
    vw = [];

    if strlength(videoFile) == 0
        return;
    end

    [~,~,ext] = fileparts(videoFile);

    candidates = ["MPEG-4","Motion JPEG AVI"];

    if ~strcmpi(ext,".mp4") && ~strcmpi(ext,".m4v")
        candidates = ["Motion JPEG AVI","MPEG-4"];
    end

    for k = 1:numel(candidates)

        try

            vw = VideoWriter(char(videoFile),char(candidates(k)));
            vw.FrameRate = frameRate;
            open(vw);

            if k > 1
                fprintf("提示：%s 不可用，改用 %s。\n", ...
                    candidates(1),candidates(k));
            end

            return;

        catch

            if ~isempty(vw)
                try
                    close(vw);
                catch
                end
                vw = [];
            end

        end

    end

    warning("无法创建视频文件，本次不录制：%s",videoFile);

end
