%SHOW_ROBOT_WORKPIECE_FRAMES
% Display the existing planning scene with UR5_base and Workpiece frames.
%
% The figure uses the same scene, colors, robot model and collision proxies
% as sync_ur5_joint_states.m. It only adds explanatory coordinate frames and
% the relative displacement from UR5_base to Workpiece.

clear;
clc;
close all;

matlabDir = fileparts(mfilename("fullpath"));
syncFile = fullfile(matlabDir,"sync_ur5_joint_states.m");
if ~isfile(syncFile)
    error("找不到场景脚本：%s",syncFile);
end

% Build the already validated scene and reuse its only main figure.
run(syncFile);

if ~exist("scene","var") || ~exist("fig","var") || ~exist("ax","var")
    error("sync_ur5_joint_states.m 未返回 scene/fig/ax。 ");
end

hold(ax,"on");

% The displayed robot is expressed in the MATLAB planning base frame.
% CoppeliaSim and MATLAB use a frozen, calibrated base transform; their
% numerical Workpiece coordinates are therefore different.
TBase = eye(4);
TWorkpiece = scene.T_MatBase_Workpiece;
pBase = TBase(1:3,4);
pWorkpiece = TWorkpiece(1:3,4);
delta = pWorkpiece - pBase;
TWorkpieceCop = scene.T_CopBase_MatBase * TWorkpiece;
deltaCop = TWorkpieceCop(1:3,4);

% MATLAB planning base frame: X red, Y green, Z blue.
drawFrameLocal(ax,TBase,0.12,"MATLAB_base");
% Workpiece frame expressed in MATLAB planning coordinates.
drawFrameLocal(ax,TWorkpiece,0.10,"Workpiece");

% Mark origins and connect them with the relative-position vector.
plot3(ax,pBase(1),pBase(2),pBase(3),"ko", ...
    "MarkerSize",8,"MarkerFaceColor","k");
plot3(ax,pWorkpiece(1),pWorkpiece(2),pWorkpiece(3),"mo", ...
    "MarkerSize",8,"MarkerFaceColor","m");
plot3(ax,[pBase(1) pWorkpiece(1)], ...
    [pBase(2) pWorkpiece(2)], ...
    [pBase(3) pWorkpiece(3)], ...
    "k--","LineWidth",1.8);

mid = (pBase+pWorkpiece)/2;
text(ax,mid(1),mid(2),mid(3), ...
    sprintf('  d=[%.1f, %.1f, %.1f] mm', ...
        delta(1)*1000,delta(2)*1000,delta(3)*1000), ...
    "FontSize",10,"Color","k","FontWeight","bold", ...
    "BackgroundColor","w","Margin",3);

% Add a compact coordinate explanation inside the figure.
infoText = sprintf([ ...
    'MATLAB base -> Workpiece\n' ...
    'p = [%.1f, %.1f, %.1f] mm\n' ...
    'R = Rz(+90 deg) in MATLAB\n' ...
    'Coppelia: p=[%.1f, %.1f, %.1f] mm, R=I\n' ...
    'X=red, Y=green, Z=blue'], ...
    delta(1)*1000,delta(2)*1000,delta(3)*1000, ...
    deltaCop(1)*1000,deltaCop(2)*1000,deltaCop(3)*1000);

annotation(fig,"textbox",[0.02 0.76 0.30 0.18], ...
    "String",infoText, ...
    "FitBoxToText","off","BackgroundColor","w", ...
    "EdgeColor",[0.2 0.2 0.2],"FontSize",10);

axis(ax,"equal");
axis(ax,"vis3d");
grid(ax,"on");
camproj(ax,"perspective");
view(ax,135,25);
title(ax,"MATLAB 规划坐标中的 UR5 base 与 Workpiece 坐标轴");
legend(ax,[ ...
    "UR5 base origin","Workpiece origin","base→workpiece"], ...
    "Location","best");

drawnow;

outputFile = fullfile(matlabDir,"robot_workpiece_frames.png");
try
    exportgraphics(fig,outputFile,"Resolution",180);
catch
    saveas(fig,outputFile);
end

fprintf("已生成坐标关系图：%s\n",outputFile);
fprintf("Workpiece origin relative MATLAB_base [mm] = [%.3f %.3f %.3f]\n", ...
    delta(1)*1000,delta(2)*1000,delta(3)*1000);
fprintf("Workpiece origin relative Coppelia_base [mm] = [%.3f %.3f %.3f]\n", ...
    deltaCop(1)*1000,deltaCop(2)*1000,deltaCop(3)*1000);

function drawFrameLocal(ax,T,scale,labelText)
    p = T(1:3,4);
    R = T(1:3,1:3);
    colors = [1 0 0; 0 0.65 0; 0 0.25 1];
    axisLabels = ["X","Y","Z"];
    for k = 1:3
        e = p + scale*R(:,k);
        plot3(ax,[p(1) e(1)],[p(2) e(2)],[p(3) e(3)], ...
            "-","Color",colors(k,:),"LineWidth",2.6);
        text(ax,e(1),e(2),e(3), ...
            " " + labelText + "-" + axisLabels(k), ...
            "Color",colors(k,:),"FontWeight","bold","FontSize",9);
    end
    text(ax,p(1),p(2),p(3),"  " + labelText, ...
        "Color","k","FontWeight","bold","FontSize",10);
end
