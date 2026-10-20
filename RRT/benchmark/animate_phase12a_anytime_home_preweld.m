function out = animate_phase12a_anytime_home_preweld(resultFile, options)
%ANIMATE_PHASE12A_ANYTIME_HOME_PREWELD Create offline MATLAB videos.
% RRT-Connect uses its first certified path; Bi-RRT* uses its best path at
% the end of the 30 s budget. No ROS2/CoppeliaSim command is sent.

if nargin < 1 || isempty(resultFile)
    root = fileparts(fileparts(mfilename('fullpath')));
    resultFile = fullfile(root,'results','phase12a_anytime_home_preweld_seed1.mat');
end
if nargin < 2 || isempty(options), options = struct(); end
root = fileparts(fileparts(mfilename('fullpath')));
defaults = struct('outputDirectory',fullfile(root,'results'), ...
    'stepDeg',1.0,'frameRate',30,'showCollisions',true, ...
    'showTcpAxis',false,'showTrail',true, ...
    'videoPrefix','phase12a_anytime_home_preweld','algorithmIndices',[1 2]);
options = localMergeOptions(defaults,options);
if ~isfile(resultFile), error('Result file not found: %s',resultFile); end
if ~isfolder(options.outputDirectory), mkdir(options.outputDirectory); end

addpath(genpath(root));
setenv('RRT_HEADLESS','1');
s = load(resultFile,'report'); report = s.report;
if ~isfield(report,'paths') || numel(report.paths) < 2
    error('The result file does not contain the two expected paths.');
end
scene = loadUR5JointScene();
fig = figure('Visible','off','Color','w', ...
    'Name','Phase 12A anytime Home to PreWeld');
fig.Units = 'pixels'; fig.Position = [100 100 1394 840]; fig.Resize = 'off';
% Let show() create the axes and its single light, exactly like the main
% planning scene.  Creating an empty axes first (axes(fig)) leaves the figure
% unlit, and an unlit figure renders the raw visual FaceColors: bright green
% robot and neon orange workpiece, which no longer match the planning scene.
show(scene.robot,scene.qMat0,'Visuals','on','Collisions','on', ...
    'Frames','off','PreservePlot',false);
ax = gca(fig); hold(ax,'on');
for k = 1:numel(scene.workpieceCollision)
    show(scene.workpieceCollision{k},'Parent',ax);
end
localRequireSceneLight(ax);

out = struct('resultFile',string(resultFile),'videos',strings(0,1), ...
    'algorithms',strings(0,1),'frameCounts',zeros(0,1));
for k = 1:numel(options.algorithmIndices)
    ai = options.algorithmIndices(k);
    if ai < 1 || ai > numel(report.paths) || isempty(report.paths{ai})
        warning('Skipping algorithm index %d because no path is available.',ai);
        continue;
    end
    qPath = double(report.paths{ai});
    label = string(report.algorithmLabels(ai));
    safeLabel = regexprep(char(label),'[^A-Za-z0-9]+','');
    videoFile = fullfile(options.outputDirectory, ...
        sprintf('%s_%s.avi',options.videoPrefix,safeLabel));
    nFrames = localWriteVideo(fig,ax,scene,qPath,report.task,report.rows(ai), ...
        label,videoFile,options);
    out.videos(end+1,1) = string(videoFile); %#ok<AGROW>
    out.algorithms(end+1,1) = label; %#ok<AGROW>
    out.frameCounts(end+1,1) = nFrames; %#ok<AGROW>
    fprintf('VIDEO_DONE algorithm=%s frames=%d file=%s\n',char(label),nFrames,videoFile);
end
if isgraphics(fig), close(fig); end
end

function nFrames = localWriteVideo(fig,ax,scene,qPath,task,row,label,videoFile,options)
qFrames = localInterpolateJoint(qPath,options.stepDeg);
tcpFrames = localTcpPositions(scene,qFrames);
% Clear only the drawn objects.  cla(ax,'reset') would also delete the scene
% light, which is what used to make the colours drift away from the planning
% scene.  Keep the light and the axes material intact instead.
localClearDrawn(ax);
hold(ax,'on');
localRequireSceneLight(ax);
for k = 1:numel(scene.workpieceCollision)
    show(scene.workpieceCollision{k},'Parent',ax);
end

pStart = tform2trvec(getTransform(scene.robot,task.start,scene.tcpBodyName));
pGoal = tform2trvec(getTransform(scene.robot,task.goal,scene.tcpBodyName));
plot3(ax,pStart(1),pStart(2),pStart(3),'o','MarkerSize',11, ...
    'MarkerFaceColor',[0.1 0.75 0.25],'MarkerEdgeColor','k');
plot3(ax,pGoal(1),pGoal(2),pGoal(3),'o','MarkerSize',11, ...
    'MarkerFaceColor',[0.1 0.35 1.0],'MarkerEdgeColor','k');
text(ax,pStart(1),pStart(2),pStart(3),'  Home','FontSize',10);
text(ax,pGoal(1),pGoal(2),pGoal(3),'  PreWeld','FontSize',10);
hTrail = plot3(ax,nan,nan,nan,'-','LineWidth',2,'Color',[0.95 0.75 0.05]);
hTcp = plot3(ax,nan,nan,nan,'o','MarkerSize',9, ...
    'MarkerFaceColor',[1 0.2 0.2],'MarkerEdgeColor','k');
hAxis = quiver3(ax,0,0,0,0,0,0,0,'Color',[0.95 0.55 0.05], ...
    'LineWidth',3,'MaxHeadSize',0.6);
hAxis.Visible = localOnOff(options.showTcpAxis);
hWire = plot3(ax,nan,nan,nan,'-','Color',[0.98 0.45 0.05],'LineWidth',4);
hWireHead = plot3(ax,nan,nan,nan,'o','Color',[0.55 0.15 0.02], ...
    'MarkerFaceColor',[0.98 0.45 0.05],'MarkerSize',5);
if ~options.showTrail, hTrail.Visible = 'off'; end
b = localBounds(ax,scene,qFrames,tcpFrames); localApplyView(ax,b);
title(ax,sprintf('%s | Home -> PreWeld',char(label)));

vw = VideoWriter(videoFile,'Motion JPEG AVI'); vw.FrameRate = options.frameRate;
open(vw); videoFrameSize = [];
cleanupVideo = onCleanup(@() localCloseVideo(vw)); %#ok<NASGU>
nFrames = size(qFrames,1);
for i = 1:nFrames
    show(scene.robot,qFrames(i,:),'Parent',ax,'Visuals','on', ...
        'Collisions',localOnOff(options.showCollisions),'Frames','off', ...
        'PreservePlot',false);
    T = getTransform(scene.robot,qFrames(i,:),scene.tcpBodyName); p = T(1:3,4);
    hTcp.XData = p(1); hTcp.YData = p(2); hTcp.ZData = p(3);
    if options.showTcpAxis
        g = T(1:3,1:3)*scene.wireAxisTcp(:);
        hAxis.XData = p(1); hAxis.YData = p(2); hAxis.ZData = p(3);
        hAxis.UData = 0.07*g(1); hAxis.VData = 0.07*g(2); hAxis.WData = 0.07*g(3);
    end
    if isfield(scene,'T_tcp_wire') && isfield(scene,'wireLengthM')
        Tw = T*scene.T_tcp_wire;
        a = Tw*[0;0;-scene.wireLengthM/2;1]; bWire = Tw*[0;0;scene.wireLengthM/2;1];
        hWire.XData = [a(1),bWire(1)]; hWire.YData = [a(2),bWire(2)];
        hWire.ZData = [a(3),bWire(3)];
        hWireHead.XData = a(1); hWireHead.YData = a(2); hWireHead.ZData = a(3);
    end
    if options.showTrail
        hTrail.XData = tcpFrames(1:i,1); hTrail.YData = tcpFrames(1:i,2);
        hTrail.ZData = tcpFrames(1:i,3);
    end
    title(ax,sprintf('%s | Home -> PreWeld | frame %d/%d | T_{first}=%.2f s | L_{best}=%.3f rad', ...
        char(label),i,nFrames,row.firstSolutionTime_ms/1000,row.finalBestPathLengthL2_rad));
    drawnow;
    frame = getframe(fig);
    if isempty(videoFrameSize)
        videoFrameSize = size(frame.cdata,[2 1]);
    elseif ~isequal(size(frame.cdata,[2 1]),videoFrameSize)
        frame.cdata = localFitFrame(frame.cdata,videoFrameSize);
    end
    writeVideo(vw,frame);
end
close(vw);
end

function qFrames = localInterpolateJoint(qPath,stepDeg)
qFrames = qPath(1,:);
for i = 1:size(qPath,1)-1
    dq = localWrapRad(qPath(i+1,:)-qPath(i,:));
    n = max(1,ceil(max(abs(rad2deg(dq)))/stepDeg));
    for k = 1:n, qFrames(end+1,:) = qPath(i,:) + (k/n)*dq; %#ok<AGROW>
    end
end
end

function p = localTcpPositions(scene,qFrames)
p = zeros(size(qFrames,1),3);
for i = 1:size(qFrames,1)
    p(i,:) = tform2trvec(getTransform(scene.robot,qFrames(i,:),scene.tcpBodyName));
end
end

function b = localBounds(ax,scene,qFrames,tcpFrames)
b = [inf -inf inf -inf inf -inf];
for i = unique(round(linspace(1,size(qFrames,1),min(24,size(qFrames,1)))))
    show(scene.robot,qFrames(i,:),'Parent',ax,'Visuals','on','Collisions','on', ...
        'Frames','off','PreservePlot',false);
    for h = findobj(ax,'Type','patch').'
        if isprop(h,'Vertices'), b = localPoints(b,h.Vertices); end
    end
end
b = localPoints(b,tcpFrames);
if any(~isfinite(b)), b = [-1 1 -1 1 0 1]; end
end

function b = localPoints(b,P)
if isempty(P), return; end
b(1) = min(b(1),min(P(:,1))); b(2) = max(b(2),max(P(:,1)));
b(3) = min(b(3),min(P(:,2))); b(4) = max(b(4),max(P(:,2)));
b(5) = min(b(5),min(P(:,3))); b(6) = max(b(6),max(P(:,3)));
end

function localApplyView(ax,b)
span = [b(2)-b(1),b(4)-b(3),b(6)-b(5)]; span(span < 1e-3) = 1e-3;
pad = 0.08*span;
xlim(ax,[b(1)-pad(1),b(2)+pad(1)]); ylim(ax,[b(3)-pad(2),b(4)+pad(2)]);
zlim(ax,[b(5)-pad(3),b(6)+pad(3)]); daspect(ax,[1 1 1]); grid(ax,'on'); box(ax,'on');
view(ax,135,25); xlabel(ax,'MATLAB base X (m)'); ylabel(ax,'MATLAB base Y (m)'); zlabel(ax,'MATLAB base Z (m)');
end

function C = localFitFrame(C,targetWH)
targetW = targetWH(1); targetH = targetWH(2); [h,w,~] = size(C);
if w > targetW, C = C(:,1:targetW,:); w = targetW; end
if h > targetH, C = C(1:targetH,:,:); h = targetH; end
if w < targetW || h < targetH
    P = zeros(targetH,targetW,size(C,3),'uint8'); P(1:h,1:w,:) = C; C = P;
end
end

function localClearDrawn(ax)
for t = ["line","patch","text","surface","quiver"]
    h = findall(ax,'Type',char(t));
    if ~isempty(h), delete(h); end
end
end

function localRequireSceneLight(ax)
% The planning scene is lit by the single light that show() creates.  Without
% it the raw visual FaceColors are displayed (bright green / neon orange).
if numel(findall(ax,'Type','light')) < 1
    error(['动画场景缺少主场景光源，颜色会退化为无光照的亮绿色/亮橙色；' ...
        '拒绝按错误配色录制视频。']);
end
end

function localCloseVideo(vw)
try, if ~isempty(vw), close(vw); end, catch, end
end

function s = localOnOff(tf)
if tf, s = 'on'; else, s = 'off'; end
end

function x = localWrapRad(x), x = atan2(sin(x),cos(x)); end

function out = localMergeOptions(defaults,in)
out = defaults; names = fieldnames(defaults);
for i = 1:numel(names)
    name = names{i};
    if isfield(in,name) && ~isempty(in.(name)), out.(name) = in.(name); end
end
end
