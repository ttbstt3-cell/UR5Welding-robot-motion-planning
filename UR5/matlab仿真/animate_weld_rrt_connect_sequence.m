function out = animate_weld_rrt_connect_sequence(resultFile,options)
%ANIMATE_WELD_RRT_CONNECT_SEQUENCE Animate Home/approach/weld/exit sequence.
%
% Reads weld_rrt_connect_approach_exit.mat and displays:
% Home -> PreWeld -> Point 1 -> Point 80 -> PostWeld -> Home.
% The existing 80-point weld path is read-only.  Welding playback uses
% TCP arc-length time parameterization; free-space RRT segments use joint
% interpolation.  No ROS2 command is sent.

arguments
    resultFile (1,1) string = ""
    options struct = struct()
end

here = fileparts(mfilename("fullpath"));
defaults = struct( ...
    "saveVideo",true, ...
    "videoFile",fullfile(here,"weld_rrt_connect_approach_exit.avi"), ...
    "view","all", ...
    "speed",1.0, ...
    "tcpSpeedMps",0.043, ...
    "stepDeg",1.0, ...
    "showCollisions",true, ...
    "showTcpAxis",false, ...
    "showTrail",true, ...
    "frameRate",30);
options=localMergeOptions(defaults,options);
if strlength(resultFile)==0
    resultFile=fullfile(here,"weld_rrt_connect_approach_exit.mat");
end
if ~isfile(resultFile), error("找不到 RRT-Connect 结果：%s",resultFile); end

scene=localLoadScene(fullfile(here,"sync_ur5_joint_states.m"));
s=load(resultFile,"result"); result=s.result;
if ~result.fullPass
    error("RRT-Connect 结果未通过，不能制作完整序列动画。");
end

qHome=result.qHome; qPre=result.preWeld.q; qWeld=result.qWeld; qPost=result.postWeld.q;
if isempty(qWeld) || size(qWeld,2)~=6, error("结果中的 qWeld 无效。"); end

% Build five named segments; repeated endpoints are removed during assembly.
segmentPaths={ ...
    result.approach.qPath, ...
    [qPre;qWeld(1,:)], ...
    qWeld, ...
    [qWeld(end,:);qPost], ...
    result.exit.qPath};
segmentNames=["Home -> PreWeld","PreWeld -> Point 1", ...
    "Welding Point 1 -> 80","Point 80 -> PostWeld","PostWeld -> Home"];

qFrames=zeros(0,6); frameSegment=strings(0,1); frameWeldPoint=zeros(0,1);
for k=1:numel(segmentPaths)
    qSeg=double(segmentPaths{k});
    if isempty(qSeg), continue; end
    if k==3
        [qLocal,pLocal]=localInterpolateTcp(scene,qSeg,options.stepDeg, ...
            options.tcpSpeedMps,options.frameRate);
    else
        [qLocal,pLocal]=localInterpolateJoint(qSeg,options.stepDeg);
    end
    if k>1 && ~isempty(qLocal), qLocal=qLocal(2:end,:); pLocal=pLocal(2:end); end
    qFrames=[qFrames;qLocal]; %#ok<AGROW>
    frameSegment=[frameSegment;repmat(segmentNames(k),numel(pLocal),1)]; %#ok<AGROW>
    frameWeldPoint=[frameWeldPoint;pLocal(:)]; %#ok<AGROW>
end
if isempty(qFrames), error("没有生成动画帧。"); end

% Static weld line, start/end markers, and dynamic overlays.
fig=gcf; ax=gca; hold(ax,"on");
% The scene must stay lit.  gcf/gca can hand back a figure without the
% planning-scene light, and an unlit figure renders the raw visual FaceColors
% (bright green robot, neon orange workpiece).  Guarantee the scene light
% before recording instead of relying on whatever figure happened to be open.
if numel(findall(ax,"Type","light"))<1
    light(ax,"Position",[1.5 0 1.5],"Style","local","Color",[1 1 1]);
    fprintf("已补加主场景光源（Position [1.5 0 1.5], Style local）。\n");
end
% Keep the captured video frame size invariant across camera/view changes.
% CoppeliaSim scene setup may leave a figure with a platform-dependent
% outer size; VideoWriter requires every frame to have identical dimensions.
fig.Units="pixels";
fig.Position=[100 100 1394 840];
fig.Resize="off";
% The scene builder draws a static TCP coordinate frame for interactive
% inspection.  The documented animation mode hides TCP direction arrows;
% keep the workpiece/base frame, but remove only the static WeldTCP frame.
if ~options.showTcpAxis
    localHideStaticTcpFrame(ax,scene);
end
seamPts=result.weldResult.seam.positionsMat;
hSeam=plot3(ax,seamPts(:,1),seamPts(:,2),seamPts(:,3), ...
    "r-","LineWidth",4);
plot3(ax,seamPts(1,1),seamPts(1,2),seamPts(1,3), ...
    "o","MarkerSize",11,"MarkerFaceColor",[0 0.7 0],"MarkerEdgeColor","k");
plot3(ax,seamPts(end,1),seamPts(end,2),seamPts(end,3), ...
    "o","MarkerSize",11,"MarkerFaceColor",[0 0.3 1],"MarkerEdgeColor","k");
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

% Include all sequence frames in the camera bounds.
bounds=localBounds(ax,scene,qFrames,seamPts);
localApplyView(ax,bounds,options.view);

vw=[]; videoFile="";
if options.saveVideo
    videoFile=options.videoFile;
    vw=VideoWriter(char(videoFile),"Motion JPEG AVI");
    vw.FrameRate=options.frameRate;
    open(vw);
end

framePause=1/(options.frameRate*options.speed);
tcpFrames=localTcpPositions(scene,qFrames);
tcpSpeed=zeros(max(0,size(tcpFrames,1)-1),1);
for i=1:numel(tcpSpeed), tcpSpeed(i)=norm(tcpFrames(i+1,:)-tcpFrames(i,:))*options.frameRate; end

for i=1:size(qFrames,1)
    show(scene.robot,qFrames(i,:),"Parent",ax,"Visuals","on", ...
        "Collisions",localOnOff(options.showCollisions),"Frames","off", ...
        "PreservePlot",false);
    T=getTransform(scene.robot,qFrames(i,:),scene.tcpBodyName);
    p=T(1:3,4); g=T(1:3,1:3)*scene.wireAxisTcp(:);
    hTcp.XData=p(1); hTcp.YData=p(2); hTcp.ZData=p(3);
    if options.showTcpAxis
        hAxis.XData=p(1); hAxis.YData=p(2); hAxis.ZData=p(3);
        hAxis.UData=0.07*g(1); hAxis.VData=0.07*g(2); hAxis.WData=0.07*g(3);
    end
    if isfield(scene,"T_tcp_wire") && isfield(scene,"wireLengthM")
        Tw=T*scene.T_tcp_wire;
        a=Tw*[0;0;-scene.wireLengthM/2;1];
        b=Tw*[0;0; scene.wireLengthM/2;1];
        hWire.XData=[a(1),b(1)]; hWire.YData=[a(2),b(2)]; hWire.ZData=[a(3),b(3)];
        hWireHead.XData=a(1); hWireHead.YData=a(2); hWireHead.ZData=a(3);
    end
    if options.showTrail
        hTrail.XData=tcpFrames(1:i,1); hTrail.YData=tcpFrames(1:i,2); hTrail.ZData=tcpFrames(1:i,3);
    end
    uistack([hSeam,hTrail,hTcp,hAxis,hWire,hWireHead],"top");
    title(ax,sprintf("%s | frame %d/%d",frameSegment(i),i,size(qFrames,1)));
    drawnow;
    if ~isempty(vw)
        frame=getframe(fig);
        % Crop/pad to the writer's established size if the desktop compositor
        % changes the drawable rectangle by one or more pixels.
        if i==1
            videoFrameSize=size(frame.cdata,[2 1]);
        elseif ~isequal(size(frame.cdata,[2 1]),videoFrameSize)
            frame.cdata=localFitFrame(frame.cdata,videoFrameSize);
        end
        writeVideo(vw,frame);
    end
    pause(framePause);
end
if ~isempty(vw), close(vw); end

out=struct();
out.resultFile=resultFile; out.qFrames=qFrames; out.frameSegment=frameSegment;
out.frameWeldPoint=frameWeldPoint; out.tcpFrames=tcpFrames;
out.tcpSpeedMps=options.tcpSpeedMps; out.tcpSpeedFrame=tcpSpeed;
out.videoFile=videoFile; out.figure=fig; out.axes=ax;
fprintf("Sequence animation complete: %d frames, video=%s\n",size(qFrames,1),videoFile);
if ~isempty(tcpSpeed)
    weldMask=frameSegment=="Welding Point 1 -> 80";
    if any(weldMask)
        idx=find(weldMask); idx=idx(idx>1);
        if ~isempty(idx), fprintf("Weld TCP frame speed range: %.4f .. %.4f m/s\n", ...
            min(tcpSpeed(idx-1)),max(tcpSpeed(idx-1))); end
    end
end

function C=localFitFrame(C,targetWH)
% Fit RGB frame to target [width height] without changing its aspect ratio.
% Padding is used instead of resizing so visual scale remains unchanged.
targetW=targetWH(1); targetH=targetWH(2);
[h,w,~]=size(C);
if w>targetW, C=C(:,1:targetW,:); w=targetW; end
if h>targetH, C=C(1:targetH,:,:); h=targetH; end
if w<targetW || h<targetH
    P=zeros(targetH,targetW,size(C,3),'uint8');
    P(1:h,1:w,:)=C;
    C=P;
end
end
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
% Dense FK sampling followed by arc-length reparameterization.  The number
% of frames is updated during the refinement passes so the requested TCP
% speed is not biased by the initial joint-space approximation.
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
span=[b(2)-b(1),b(4)-b(3),b(6)-b(5)]; pad=0.08*span;
xlim(ax,[b(1)-pad(1),b(2)+pad(1)]); ylim(ax,[b(3)-pad(2),b(4)+pad(2)]);
zlim(ax,[b(5)-pad(3),b(6)+pad(3)]); daspect(ax,[1 1 1]); grid(ax,"on");
xlabel(ax,"MATLAB base X (m)"); ylabel(ax,"MATLAB base Y (m)"); zlabel(ax,"MATLAB base Z (m)");
switch string(viewName)
    case "all", view(ax,135,25);
    case "allTop", view(ax,0,90);
    case "allWeldSide", view(ax,90,55);
    case "allWeld", view(ax,120,55);
    case "allWeldReverse", view(ax,140,55);
end
end

function localHideStaticTcpFrame(ax,scene)
% Hide the three static TCP axes and its label created by sync_ur5_joint_states.
if isfield(scene,"T_MatBase_TCP0")
    p=scene.T_MatBase_TCP0(1:3,4).';
    for h=findobj(ax,"Type","line").'
        if ~isgraphics(h), continue; end
        x=double(h.XData); y=double(h.YData); z=double(h.ZData);
        if numel(x)==2 && numel(y)==2 && numel(z)==2
            if norm([x(1) y(1) z(1)]-p)<1e-7 && norm([x(2) y(2) z(2)]-p)>1e-5
                h.Visible="off";
            end
        end
    end
end
for h=findobj(ax,"Type","text").'
    if isgraphics(h) && contains(string(h.String),"WeldTCP")
        h.Visible="off";
    end
end
end

function scene=localLoadScene(syncFile)
run(syncFile);
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

function x=localWrapRad(x)
x=atan2(sin(x),cos(x));
end
