function manifest = record_three_weld_videos(options)
%RECORD_THREE_WELD_VIDEOS Record the validated A/B/C weld paths.
%
% This is a read-only playback utility.  It does not re-plan, change the
% robot model, change ROS2/CoppeliaSim, or send any command.  The selected
% result MAT files are the already validated A-ARC-IK paths from the fair
% task benchmark.
%
% The videos use the original planning-scene display (visuals and collision
% geometry on), show the welding wire and TCP trail, and deliberately do not
% draw a TCP coordinate arrow (so it cannot be mistaken for the wire).

if nargin < 1 || isempty(options), options = struct(); end
here = fileparts(mfilename("fullpath"));
defaults = struct( ...
    "resultDir",fullfile(here,"task_level_arc_vs_gao_runs"), ...
    "outputDir",fullfile(here,"weld_seam_videos"), ...
    "frameRate",30, ...
    "tcpSpeedMps",0.043, ...
    "speedLabel","", ...
    "speed",1.0, ...
    "stepDeg",1.0, ...
    "showCollisions",true, ...
    "showTrail",true, ...
    "writeTopView",true, ...
    "taskNames",strings(0,1), ...
    "figurePosition",[100 100 1394 840]);
options = localMergeOptions(defaults,options);

if ~isfolder(options.outputDir), mkdir(options.outputDir); end

% The task-level run folder was renamed from fair_task_pre5_runs to
% task_level_arc_vs_gao_runs; accept either so old call sites keep working.
if ~isfolder(options.resultDir)
    legacyDir = fullfile(here,"fair_task_pre5_runs");
    if isfolder(legacyDir)
        options.resultDir = legacyDir;
    else
        error("找不到任务级结果目录：%s（也不存在旧目录 %s）。", ...
            options.resultDir,legacyDir);
    end
end

% Select the first complete PASS for each seam.  Some benchmark seeds use
% the reverse direction or a recovery round, so a hard-coded
% *_arc_001_forward.mat is not guaranteed to be a PASS.
tasks = repmat(struct("name","","resultFile",""),1,0);
for name = ["Seam_A","Seam_B","Seam_C"]
    f = localFirstPassingResult(options.resultDir,name);
    if isempty(f)
        error("%s 没有找到完整 PASS 的 A-ARC-IK 结果。",name);
    end
    tasks(end+1) = struct("name",name,"resultFile",f); %#ok<AGROW>
end
if ~isempty(options.taskNames)
    keep = ismember(string({tasks.name}),string(options.taskNames));
    tasks = tasks(keep);
end
if isempty(tasks), error("taskNames 没有匹配到 Seam_A/B/C。"); end

manifest = struct([]);
for k = 1:numel(tasks)
    if ~isfile(tasks(k).resultFile)
        error("找不到 %s 的已验证结果：%s",tasks(k).name,tasks(k).resultFile);
    end

    s = load(tasks(k).resultFile,"result");
    result = s.result;
    localAssertPass(result,tasks(k).name);

    % Use the final adaptive execution path when available.  Fall back to
    % the sparse path only for older result files.
    if isfield(result,"qPathExecution") && ~isempty(result.qPathExecution)
        qPath = double(result.qPathExecution);
    else
        qPath = double(result.qPath);
    end
    if size(qPath,2) ~= 6 || size(qPath,1) < 2
        error("%s 的 qPath 无效。",tasks(k).name);
    end

    scene = result.scene;
    seamPts = localDenseSeam(result.seam,160);
    [qFrames,~] = localInterpolateTcp(scene,qPath,options.stepDeg, ...
        options.tcpSpeedMps,options.frameRate);

    views = "all";
    if options.writeTopView
        views = ["all","allTop"];
    end
    taskManifest = struct();
    taskManifest.name = tasks(k).name;
    taskManifest.resultFile = tasks(k).resultFile;
    taskManifest.qPathPoints = size(qPath,1);
    taskManifest.frameCount = size(qFrames,1);
    taskManifest.tcpSpeedMps = options.tcpSpeedMps;
    speedCmMin = 6000*options.tcpSpeedMps;
    % File naming defaults to cm/min.  Callers can override it so a historical
    % set keeps its published names: the 43 mm/s videos are named "43mmps"
    % even though 43 mm/s equals 258 cm/min.
    if strlength(string(options.speedLabel)) > 0
        speedLabel = string(options.speedLabel);
    else
        speedLabel = sprintf("%.0fcmmin",speedCmMin);
    end
    speedText = sprintf("%.1f cm/min",speedCmMin);
    taskManifest.videoFiles = strings(numel(views),1);

    for iv = 1:numel(views)
        viewName = views(iv);
        suffix = "perspective";
        if viewName == "allTop", suffix = "top"; end
        videoFile = fullfile(options.outputDir, ...
            sprintf("%s_AARC_IK_%s_%s.avi", ...
            tasks(k).name,speedLabel,suffix));
        taskManifest.videoFiles(iv) = string(videoFile);
        localWriteVideo(scene,seamPts,qFrames,tasks(k).name,videoFile, ...
            viewName,options,speedText);
    end
    manifest = [manifest;taskManifest]; %#ok<AGROW>
end

manifestFile = fullfile(options.outputDir,"three_weld_video_manifest.mat");
save(manifestFile,"manifest","-v7");
fprintf("\n三条焊缝视频录制完成。\n输出目录：%s\n清单：%s\n", ...
    options.outputDir,manifestFile);
for k = 1:numel(manifest)
    fprintf("  %s: %s\n",manifest(k).name, ...
        strjoin(cellstr(manifest(k).videoFiles),", "));
end
end

function localAssertPass(result,name)
required = ["complete","nodeCollisionFree","edgeCollisionFree"];
for i = 1:numel(required)
    if ~isfield(result,required(i)) || ~logical(result.(required(i)))
        error("%s 结果不是完整 PASS，拒绝录制：缺少/未通过 %s。", ...
            name,required(i));
    end
end
if isfield(result,"finalTcpMetrics")
    m = result.finalTcpMetrics;
    if isfield(m,"nodeCollisionFree") && ~logical(m.nodeCollisionFree), error("%s TCP节点碰撞未通过。",name); end
    if isfield(m,"edgeCollisionFree") && ~logical(m.edgeCollisionFree), error("%s TCP段边碰撞未通过。",name); end
end
end

function f=localFirstPassingResult(dirName,stem)
f="";
% Sort by filename so selection is deterministic and reproducible.
a=dir(fullfile(dirName,stem+"_arc_*.mat"));
[~,ord]=sort({a.name}); a=a(ord);
for k=1:numel(a)
    p=fullfile(a(k).folder,a(k).name);
    try
        S=load(p,"result");
        if isfield(S,"result") && isfield(S.result,"complete") && logical(S.result.complete)
            f=p; return;
        end
    catch
    end
end
end

function seamPts = localDenseSeam(seam,n)
if isfield(seam,"startMat") && isfield(seam,"endMat")
    p0 = double(seam.startMat(:).'); p1 = double(seam.endMat(:).');
elseif isfield(seam,"positionsMat")
    P = double(seam.positionsMat); p0=P(1,:); p1=P(end,:);
else
    error("焊缝结果中没有 positionsMat/startMat/endMat。")
end
s = linspace(0,1,n).';
seamPts = (1-s).*p0+s.*p1;
end

function localWriteVideo(scene,seamPts,qFrames,name,videoFile,viewName,options,speedText)
fig = figure("Name",name+" weld path", ...
    "Color","w","Visible","on");
fig.Units = "pixels";
fig.Position = options.figurePosition;
fig.Resize = "off";
ax = axes(fig);
hold(ax,"on");

% Preserve the original scene appearance: robot visuals + collision model,
% and the original MATLAB collision-box workpiece appearance.
show(scene.robot,qFrames(1,:),"Parent",ax,"Visuals","on", ...
    "Collisions",localOnOff(options.showCollisions),"Frames","off", ...
    "PreservePlot",false);
for j = 1:numel(scene.workpieceCollision)
    show(scene.workpieceCollision{j},"Parent",ax);
end

hSeam = plot3(ax,seamPts(:,1),seamPts(:,2),seamPts(:,3),"-", ...
    "Color",[0.85 0 0],"LineWidth",3.5);
plot3(ax,seamPts(1,1),seamPts(1,2),seamPts(1,3),"o", ...
    "MarkerSize",9,"MarkerFaceColor",[0 0.70 0],"MarkerEdgeColor","k");
plot3(ax,seamPts(end,1),seamPts(end,2),seamPts(end,3),"o", ...
    "MarkerSize",9,"MarkerFaceColor",[0 0.30 1],"MarkerEdgeColor","k");
hTrail = plot3(ax,nan,nan,nan,"-","LineWidth",2.5, ...
    "Color",[0.95 0.72 0.05]);
hTcp = plot3(ax,nan,nan,nan,"o","MarkerSize",8, ...
    "MarkerFaceColor",[1 0.15 0.15],"MarkerEdgeColor","k");
% The wire is drawn explicitly because it is a collision cylinder, not a
% robot visual.  No TCP coordinate arrow is created.
hWire = plot3(ax,nan,nan,nan,"-","Color",[0.98 0.45 0.05],"LineWidth",4);
hWireTip = plot3(ax,nan,nan,nan,"o","MarkerSize",5, ...
    "MarkerFaceColor",[0.98 0.45 0.05],"MarkerEdgeColor",[0.4 0.1 0]);
if ~options.showTrail, hTrail.Visible="off"; end
uistack([hSeam,hTrail,hTcp,hWire,hWireTip],"top");

tcpFrames = localTcpPositions(scene,qFrames);
b = localBounds(ax,scene,qFrames,seamPts,options.showCollisions);
localApplyView(ax,b,viewName);
camproj(ax,"perspective"); axis(ax,"vis3d"); grid(ax,"on");
% Main planning-scene lighting: the single local light that the planning
% scene itself has.  camlight("headlight") lit every visible face head-on and
% made the workpiece far brighter than the planning scene (orange mean ~188
% instead of ~114), so it must not be used here.
light(ax,"Position",[1.5 0 1.5],"Style","local","Color",[1 1 1]);

vw = VideoWriter(char(videoFile),"Motion JPEG AVI");
vw.FrameRate = options.frameRate;
open(vw);
fprintf("录制 %s: %s (%d frames)\n",name,videoFile,size(qFrames,1));

for i = 1:size(qFrames,1)
    show(scene.robot,qFrames(i,:),"Parent",ax,"Visuals","on", ...
        "Collisions",localOnOff(options.showCollisions),"Frames","off", ...
        "PreservePlot",false);
    T = getTransform(scene.robot,qFrames(i,:),scene.tcpBodyName);
    p = T(1:3,4);
    hTcp.XData=p(1); hTcp.YData=p(2); hTcp.ZData=p(3);
    if options.showTrail
        hTrail.XData=tcpFrames(1:i,1); hTrail.YData=tcpFrames(1:i,2); hTrail.ZData=tcpFrames(1:i,3);
    end
    if isfield(scene,"T_tcp_wire") && isfield(scene,"wireLengthM")
        Tw=T*scene.T_tcp_wire;
        a=Tw*[0;0;-scene.wireLengthM/2;1];
        btip=Tw*[0;0;scene.wireLengthM/2;1];
        hWire.XData=[a(1),btip(1)]; hWire.YData=[a(2),btip(2)]; hWire.ZData=[a(3),btip(3)];
        hWireTip.XData=a(1); hWireTip.YData=a(2); hWireTip.ZData=a(3);
    end
    uistack([hSeam,hTrail,hTcp,hWire,hWireTip],"top");
    title(ax,sprintf("%s | A-ARC-IK | %s | frame %d/%d",name,speedText,i,size(qFrames,1)));
    drawnow;
    fr = getframe(fig);
    if i == 1
        targetWH = size(fr.cdata,[2 1]);
    elseif ~isequal(size(fr.cdata,[2 1]),targetWH)
        fr.cdata = localFitFrame(fr.cdata,targetWH);
    end
    writeVideo(vw,fr);
    pause(1/(options.frameRate*options.speed));
end
close(vw);
if isvalid(fig), close(fig); end
end

function [qFrames,pointIdx]=localInterpolateTcp(scene,qPath,stepDeg,tcpSpeed,frameRate)
% Dense FK sampling followed by TCP arc-length reparameterization.
denseStepDeg=max(stepDeg/8,0.05);
qDense=qPath(1,:); pointDense=1;
for i=1:size(qPath,1)-1
    dq=qPath(i+1,:)-qPath(i,:);
    n=max(1,ceil(rad2deg(max(abs(dq)))/denseStepDeg));
    for k=1:n
        qDense(end+1,:)=qPath(i,:)+(k/n)*dq; %#ok<AGROW>
        pointDense(end+1,1)=i+1; %#ok<AGROW>
    end
end
pDense=localTcpPositions(scene,qDense);
arc=[0;cumsum(vecnorm(diff(pDense,1,1),2,2))];
[arcU,keep]=unique(arc,"stable"); qU=qDense(keep,:); pointU=pointDense(keep);
if arcU(end)<=eps, qFrames=qPath(1,:); pointIdx=1; return; end
nInt=max(1,round(arcU(end)/tcpSpeed*frameRate));
target=linspace(0,arcU(end),nInt+1).';
qFrames=localInterpQ(arcU,qU,target);
for pass=1:8
    p=localTcpPositions(scene,qFrames);
    a=[0;cumsum(vecnorm(diff(p,1,1),2,2))];
    [aU,kp]=unique(a,"stable"); qP=qFrames(kp,:);
    nInt=max(1,round(aU(end)/tcpSpeed*frameRate));
    target=linspace(0,aU(end),nInt+1).';
    qFrames=localInterpQ(aU,qP,target);
    qFrames(1,:)=qPath(1,:); qFrames(end,:)=qPath(end,:);
end
pointArc=linspace(0,arcU(end),size(qFrames,1)).';
pointIdx=round(interp1(arcU,double(pointU),pointArc,"nearest","extrap"));
end

function q=localInterpQ(a,Q,x)
q=zeros(numel(x),size(Q,2));
for j=1:size(Q,2), q(:,j)=interp1(a,Q(:,j),x,"linear","extrap"); end
end

function p=localTcpPositions(scene,qFrames)
p=zeros(size(qFrames,1),3);
for i=1:size(qFrames,1)
    p(i,:)=tform2trvec(getTransform(scene.robot,qFrames(i,:),scene.tcpBodyName));
end
end

function b=localBounds(ax,scene,qFrames,seamPts,showCollisions)
b=[inf -inf inf -inf inf -inf];
for i=unique(round(linspace(1,size(qFrames,1),12)))
    show(scene.robot,qFrames(i,:),"Parent",ax,"Visuals","on", ...
        "Collisions",localOnOff(showCollisions),"Frames","off", ...
        "PreservePlot",false);
    for h=findobj(ax,"Type","patch").', b=localPoints(b,h.Vertices); end
end
for j=1:numel(scene.workpieceCollision)
    show(scene.workpieceCollision{j},"Parent",ax);
    for h=findobj(ax,"Type","patch").', b=localPoints(b,h.Vertices); end
end
b=localPoints(b,seamPts); b=localPoints(b,localTcpPositions(scene,qFrames));
end

function b=localPoints(b,P)
if isempty(P), return; end
b(1)=min(b(1),min(P(:,1))); b(2)=max(b(2),max(P(:,1)));
b(3)=min(b(3),min(P(:,2))); b(4)=max(b(4),max(P(:,2)));
b(5)=min(b(5),min(P(:,3))); b(6)=max(b(6),max(P(:,3)));
end

function localApplyView(ax,b,viewName)
span=[b(2)-b(1),b(4)-b(3),b(6)-b(5)];
pad=max(0.08*span,0.02);
xlim(ax,[b(1)-pad(1),b(2)+pad(1)]); ylim(ax,[b(3)-pad(2),b(4)+pad(2)]);
zlim(ax,[b(5)-pad(3),b(6)+pad(3)]); daspect(ax,[1 1 1]);
xlabel(ax,"MATLAB base X (m)"); ylabel(ax,"MATLAB base Y (m)"); zlabel(ax,"MATLAB base Z (m)");
switch string(viewName)
    case "all", view(ax,135,25);
    case "allTop", view(ax,0,90);
    otherwise, view(ax,135,25);
end
end

function C=localFitFrame(C,targetWH)
targetW=targetWH(1); targetH=targetWH(2); [h,w,~]=size(C);
if w>targetW, C=C(:,1:targetW,:); w=targetW; end
if h>targetH, C=C(1:targetH,:,:); h=targetH; end
if w<targetW || h<targetH
    P=zeros(targetH,targetW,size(C,3),'uint8'); P(1:h,1:w,:)=C; C=P;
end
end

function s=localMergeOptions(defaults,in)
s=defaults; f=fieldnames(defaults);
for i=1:numel(f)
    if isfield(in,f{i}) && ~isempty(in.(f{i})), s.(f{i})=in.(f{i}); end
end
end

function s=localOnOff(tf)
if tf, s="on"; else, s="off"; end
end
