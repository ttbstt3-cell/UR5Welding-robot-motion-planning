function out = animate_phase12a_anytime_sequence(reportFile,options)
%ANIMATE_PHASE12A_ANYTIME_SEQUENCE Full-chain offline replay of the anytime results.
%
%   Replays the complete task chain
%
%       safe point -> weld start -> welding -> weld end -> safe point
%
%   for every algorithm stored in the Phase 12A anytime report:
%
%       Home -> PreWeld         replanned free-space approach (this method)
%       PreWeld -> Point 1      frozen local safety link
%       Welding Point 1 -> 80   frozen 80-point weld path, TCP arc-length timing
%       Point 80 -> PostWeld    frozen local safety link
%       PostWeld -> Home        replanned free-space exit (this method)
%
%   The approach and the exit are both taken from the same anytime run, so the
%   replay covers the return to the safe point and not only the approach.
%
%   Scene appearance: the figure, its axes and its single light are created by
%   show() exactly the way the main planning scene does it.  No visual
%   FaceColor is overridden and no headlight is added, so the replay keeps the
%   original planning-scene colours.  Creating an empty axes first (or adding a
%   camlight headlight) is what previously produced the bright, unlit
%   green/orange frames; this script never does that.
%
%   Nothing is re-planned, no TCP/scene/collision parameter is changed and no
%   ROS2 or CoppeliaSim command is sent.

arguments
    reportFile (1,1) string = ""
    options struct = struct()
end

root=fileparts(fileparts(mfilename("fullpath")));
defaults=struct( ...
    "sourceFile",fullfile(root,"results","phase12a_seam_legacy_rrtconnect_repro.mat"), ...
    "outputDirectory",fullfile(root,"results"), ...
    "videoPrefix","phase12a_anytime", ...
    "views",["perspective","top"], ...
    "algorithmIndices",[1 2], ...
    "stepDeg",1.0, ...
    "tcpSpeedMps",0.043, ...
    "frameRate",30, ...
    "showCollisions",true, ...
    "showTcpAxis",false, ...
    "showTrail",true, ...
    "figurePosition",[100 100 1394 840]);
options=localMergeOptions(defaults,options);
if strlength(reportFile)==0
    reportFile=fullfile(root,"results","phase12a_anytime_seamA_two_successful.mat");
end
if ~isfile(reportFile), error("找不到 anytime 结果：%s",reportFile); end
if ~isfile(options.sourceFile), error("找不到冻结焊缝结果：%s",options.sourceFile); end
if ~isfolder(options.outputDirectory), mkdir(options.outputDirectory); end

setenv("RRT_HEADLESS","1");
scene=loadUR5JointScene();
r=load(reportFile,"report"); report=r.report;
s=load(options.sourceFile,"result"); source=s.result;

qHome=double(source.qHome(:).'); qWeld=double(source.qWeld);
qPre=double(source.preWeld.q(:).'); qPost=double(source.postWeld.q(:).');
if isempty(qWeld) || size(qWeld,2)~=6, error("冻结结果中的 qWeld 无效。"); end

out=struct("reportFile",string(reportFile),"sourceFile",string(options.sourceFile), ...
    "videos",strings(0,1),"algorithms",strings(0,1),"views",strings(0,1), ...
    "frameCounts",zeros(0,1),"segmentFrameCounts",zeros(0,5), ...
    "frameRate",options.frameRate,"tcpSpeedMps",options.tcpSpeedMps);

for ai=options.algorithmIndices
    if ai<1 || ai>numel(report.algorithmLabels), continue; end
    if size(report.paths,2)<ai || isempty(report.paths{1,ai}) || isempty(report.paths{2,ai})
        warning("算法 %d 缺少接近或收焊路径，跳过。",ai); continue;
    end
    approach=double(report.paths{1,ai});
    exitPath=double(report.paths{2,ai});
    label=string(report.algorithmLabels(ai));
    localAssertEndpoints(approach,qHome,qPre,"approach",label);
    localAssertEndpoints(exitPath,qPost,qHome,"exit",label);

    % Five named segments; repeated endpoints are removed during assembly.
    segmentPaths={approach,[qPre;qWeld(1,:)],qWeld,[qWeld(end,:);qPost],exitPath};
    segmentNames=["Home -> PreWeld","PreWeld -> Point 1", ...
        "Welding Point 1 -> 80","Point 80 -> PostWeld","PostWeld -> Home"];

    qFrames=zeros(0,6); frameSegment=strings(0,1); segmentCounts=zeros(1,5);
    for k=1:numel(segmentPaths)
        qSeg=double(segmentPaths{k});
        if isempty(qSeg), continue; end
        if k==3
            [qLocal,~]=localInterpolateTcp(scene,qSeg,options.stepDeg, ...
                options.tcpSpeedMps,options.frameRate);
        else
            [qLocal,~]=localInterpolateJoint(qSeg,options.stepDeg);
        end
        if k>1 && ~isempty(qLocal), qLocal=qLocal(2:end,:); end
        segmentCounts(k)=size(qLocal,1);
        qFrames=[qFrames;qLocal]; %#ok<AGROW>
        frameSegment=[frameSegment;repmat(segmentNames(k),size(qLocal,1),1)]; %#ok<AGROW>
    end
    if isempty(qFrames), error("算法 %s 没有生成动画帧。",label); end
    localReportSegments(label,segmentNames,segmentCounts,size(qFrames,1),options);

    safeLabel=regexprep(char(label),"[^A-Za-z0-9]+","");
    for viewName=options.views
        videoFile=fullfile(options.outputDirectory, ...
            sprintf("%s_%s_%s.avi",options.videoPrefix,safeLabel,viewName));
        info=localWriteVideo(scene,qFrames,frameSegment,label, ...
            string(viewName),videoFile,options,source);
        out.videos(end+1,1)=string(videoFile); %#ok<AGROW>
        out.algorithms(end+1,1)=label; %#ok<AGROW>
        out.views(end+1,1)=string(viewName); %#ok<AGROW>
        out.frameCounts(end+1,1)=info.frameCount; %#ok<AGROW>
        out.segmentFrameCounts(end+1,:)=segmentCounts; %#ok<AGROW>
        fprintf("VIDEO_DONE algorithm=%s view=%s frames=%d light=%d green=%s file=%s\n", ...
            char(label),char(viewName),info.frameCount,info.lightCount, ...
            mat2str(info.greenMean),char(videoFile));
    end
end
out.table=table(out.algorithms(:),out.views(:),out.frameCounts(:), ...
    "VariableNames",["algorithm","view","frameCount"]);
fprintf("\nFULL_CHAIN_VIDEO_DONE videos=%d\n",numel(out.videos));
end

function localAssertEndpoints(path,expectedStart,expectedGoal,whichPath,label)
if size(path,2)~=6 || size(path,1)<2
    error("%s 的 %s 路径无效。",label,whichPath);
end
if max(abs(path(1,:)-expectedStart))>1e-6 || max(abs(path(end,:)-expectedGoal))>1e-6
    error("%s 的 %s 路径端点与冻结焊缝不匹配。",label,whichPath);
end
end

function localReportSegments(label,names,counts,totalFrames,options)
fprintf("\n%s 完整序列（安全点 -> 起焊点 -> 焊缝 -> 收焊点 -> 安全点）\n",label);
for k=1:numel(names)
    fprintf("  %-22s %5d 帧\n",names(k),counts(k));
end
fprintf("  合计 %d 帧 ≈ %.2f s @ %d fps\n",totalFrames, ...
    totalFrames/options.frameRate,options.frameRate);
if counts(5)==0, error("%s 缺少收焊点回到安全点的退出段。",label); end
if counts(1)==0, error("%s 缺少安全点到起焊点的接近段。",label); end
end

function info=localWriteVideo(scene,qFrames,frameSegment,label,viewName,videoFile,options,source)
% Build the scene exactly like the main planning scene: let show() create the
% axes and its light, then add the workpiece and the overlays.
fig=figure("Name",label+" full sequence","Color","w","Visible","off");
fig.Units="pixels"; fig.Position=options.figurePosition; fig.Resize="off";
show(scene.robot,qFrames(1,:),"Visuals","on", ...
    "Collisions",localOnOff(options.showCollisions),"Frames","off", ...
    "PreservePlot",false);
ax=gca; hold(ax,"on");
for k=1:numel(scene.workpieceCollision)
    show(scene.workpieceCollision{k},"Parent",ax);
end
lightCount=numel(findall(ax,"Type","light"));
if lightCount<1
    error("场景光源缺失，颜色会退化为无光照的亮绿色/亮橙色，拒绝录制。");
end

seamPts=double(source.weldResult.seam.positionsMat);
hSeam=plot3(ax,seamPts(:,1),seamPts(:,2),seamPts(:,3),"r-","LineWidth",4);
plot3(ax,seamPts(1,1),seamPts(1,2),seamPts(1,3),"o","MarkerSize",11, ...
    "MarkerFaceColor",[0 0.7 0],"MarkerEdgeColor","k");
plot3(ax,seamPts(end,1),seamPts(end,2),seamPts(end,3),"o","MarkerSize",11, ...
    "MarkerFaceColor",[0 0.3 1],"MarkerEdgeColor","k");
hTrail=plot3(ax,nan,nan,nan,"-","LineWidth",2,"Color",[0.95 0.75 0.05]);
hTcp=plot3(ax,nan,nan,nan,"o","MarkerSize",9, ...
    "MarkerFaceColor",[1 0.2 0.2],"MarkerEdgeColor","k");
hAxis=quiver3(ax,0,0,0,0,0,0,0,"Color",[0.95 0.55 0.05], ...
    "LineWidth",3,"MaxHeadSize",0.6);
hAxis.Visible=localOnOff(options.showTcpAxis);
hWire=plot3(ax,nan,nan,nan,"-","Color",[0.98 0.45 0.05],"LineWidth",4);
hWireHead=plot3(ax,nan,nan,nan,"o","Color",[0.55 0.15 0.02], ...
    "MarkerFaceColor",[0.98 0.45 0.05],"MarkerSize",5);
if ~options.showTrail, hTrail.Visible="off"; end
uistack([hSeam,hTrail,hTcp,hAxis,hWire,hWireHead],"top");

tcpFrames=localTcpPositions(scene,qFrames);
bounds=localBounds(ax,scene,qFrames,seamPts);
localApplyView(ax,bounds,viewName);

% Fix the video frame size to the requested figure size before recording.
% A figure whose window has not been mapped yet can report a smaller drawable
% area on its first getframe (observed: 1394x818 instead of the required
% 1394x840), and using that first frame as the reference would silently shrink
% every frame of the video.
targetWH=options.figurePosition(3:4);
settledWH=[0 0];
for attempt=1:10
    drawnow; pause(0.1); drawnow;
    settledWH=size(getframe(fig).cdata,[2 1]);
    if all(settledWH>=targetWH), break; end
    fig.Position=[options.figurePosition(1:2), ...
        options.figurePosition(3:4)+(targetWH-settledWH)];
end
if ~isequal(settledWH,targetWH)
    warning("full-sequence video: 出图区为 %dx%d，已裁剪/补齐到要求的 %dx%d。", ...
        settledWH(1),settledWH(2),targetWH(1),targetWH(2));
end

vw=VideoWriter(char(videoFile),"Motion JPEG AVI");
vw.FrameRate=options.frameRate;
open(vw);
cleanupVideo=onCleanup(@() localCloseVideo(vw)); %#ok<NASGU>
greenMean=NaN(1,3);
for i=1:size(qFrames,1)
    show(scene.robot,qFrames(i,:),"Parent",ax,"Visuals","on", ...
        "Collisions",localOnOff(options.showCollisions),"Frames","off", ...
        "PreservePlot",false);
    T=getTransform(scene.robot,qFrames(i,:),scene.tcpBodyName);
    p=T(1:3,4);
    hTcp.XData=p(1); hTcp.YData=p(2); hTcp.ZData=p(3);
    if options.showTcpAxis
        g=T(1:3,1:3)*scene.wireAxisTcp(:);
        hAxis.XData=p(1); hAxis.YData=p(2); hAxis.ZData=p(3);
        hAxis.UData=0.07*g(1); hAxis.VData=0.07*g(2); hAxis.WData=0.07*g(3);
    end
    if isfield(scene,"T_tcp_wire") && isfield(scene,"wireLengthM")
        Tw=T*scene.T_tcp_wire;
        a=Tw*[0;0;-scene.wireLengthM/2;1];
        bTip=Tw*[0;0;scene.wireLengthM/2;1];
        hWire.XData=[a(1),bTip(1)]; hWire.YData=[a(2),bTip(2)]; hWire.ZData=[a(3),bTip(3)];
        hWireHead.XData=a(1); hWireHead.YData=a(2); hWireHead.ZData=a(3);
    end
    if options.showTrail
        hTrail.XData=tcpFrames(1:i,1); hTrail.YData=tcpFrames(1:i,2);
        hTrail.ZData=tcpFrames(1:i,3);
    end
    uistack([hSeam,hTrail,hTcp,hAxis,hWire,hWireHead],"top");
    title(ax,sprintf("%s | %s | frame %d/%d",label,frameSegment(i),i,size(qFrames,1)));
    drawnow;
    frame=getframe(fig);
    % Every frame is fitted to the required size, so a desktop compositor that
    % changes the drawable rectangle by a pixel cannot produce a video whose
    % frames have inconsistent dimensions.
    frame.cdata=localFitFrame(frame.cdata,targetWH);
    if i==1, greenMean=localGreenMean(frame.cdata); end
    writeVideo(vw,frame);
end
close(vw);
if isgraphics(fig), close(fig); end
info=struct("frameCount",size(qFrames,1),"lightCount",lightCount, ...
    "greenMean",greenMean,"videoFile",videoFile);
end

function m=localGreenMean(C)
rgb=double(C); r=rgb(:,:,1); g=rgb(:,:,2); b=rgb(:,:,3);
mask=(g>r+18)&(g>b+18);
if ~any(mask(:)), m=[NaN NaN NaN]; return; end
m=round([mean(r(mask)) mean(g(mask)) mean(b(mask))]);
end

function [qFrames,pointIdx]=localInterpolateJoint(qPath,stepDeg)
qFrames=qPath(1,:); pointIdx=1;
for i=1:size(qPath,1)-1
    dq=localWrapRad(qPath(i+1,:)-qPath(i,:));
    n=max(1,ceil(max(abs(rad2deg(dq)))/stepDeg));
    for k=1:n
        qFrames(end+1,:)=qPath(i,:)+(k/n)*dq; %#ok<AGROW>
        pointIdx(end+1,1)=i+1; %#ok<AGROW>
    end
end
end

function [qFrames,pointIdx]=localInterpolateTcp(scene,qPath,stepDeg,tcpSpeed,frameRate)
% Dense FK sampling followed by arc-length reparameterization so the TCP moves
% at the requested constant speed; joint values stay on the frozen path.
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
for pass=1:10
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

function b=localBounds(ax,scene,qFrames,seamPts)
b=[inf -inf inf -inf inf -inf];
for i=unique(round(linspace(1,size(qFrames,1),20)))
    show(scene.robot,qFrames(i,:),"Parent",ax,"Visuals","on","Collisions","on", ...
        "Frames","off","PreservePlot",false);
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
span=[b(2)-b(1),b(4)-b(3),b(6)-b(5)]; span(span<1e-3)=1e-3;
pad=0.08*span;
xlim(ax,[b(1)-pad(1),b(2)+pad(1)]); ylim(ax,[b(3)-pad(2),b(4)+pad(2)]);
zlim(ax,[b(5)-pad(3),b(6)+pad(3)]); daspect(ax,[1 1 1]); grid(ax,"on");
xlabel(ax,"MATLAB base X (m)"); ylabel(ax,"MATLAB base Y (m)");
zlabel(ax,"MATLAB base Z (m)");
switch string(viewName)
    case "perspective", view(ax,135,25);
    case "top", view(ax,0,90);
    case "weldSide", view(ax,90,55);
    case "weld", view(ax,120,55);
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

function localCloseVideo(vw)
try, if ~isempty(vw), close(vw); end, catch, end
end

function s=localOnOff(tf)
if tf, s="on"; else, s="off"; end
end

function x=localWrapRad(x)
x=atan2(sin(x),cos(x));
end

function out=localMergeOptions(defaults,in)
out=defaults; names=fieldnames(defaults);
for i=1:numel(names)
    if isfield(in,names{i}) && ~isempty(in.(names{i})), out.(names{i})=in.(names{i}); end
end
end
