function manifest = record_three_weld_gao_videos(options)
%RECORD_THREE_WELD_GAO_VIDEOS
% Record validated Gao-AR-RRT* trajectories using the original MATLAB
% planning-scene appearance.  This is playback only: no planning, ROS2,
% CoppeliaSim, or robot command is executed.

if nargin < 1 || isempty(options), options=struct(); end
here=fileparts(mfilename("fullpath"));
defaults=struct( ...
    "resultDir",fullfile(here,"task_level_arc_vs_gao_runs"), ...
    "outputDir",fullfile(here,"weld_seam_videos_gao"), ...
    "frameRate",30,"tcpSpeedMps",100/60000, ...
    "speedLabel","100cmmin","speedText","100.0 cm/min", ...
    "speedFactor",1.0,"stepDeg",1.0,"showCollisions",true,"showTrail",true, ...
    "writeTopView",true,"taskNames",strings(0,1),"visible","off", ...
    "figurePosition",[100 100 1394 840]);
options=localMerge(defaults,options);
if ~isfolder(options.outputDir), mkdir(options.outputDir); end

% Select deterministic successful runs already produced by the formal
% task-level Gao benchmark.  We never use a failed or partial qPath.
tasks=repmat(struct("name","","resultFile",""),1,0);
for name=["Seam_A","Seam_B","Seam_C"]
    f=localFirstPassingResult(options.resultDir,name);
    if isempty(f), error("%s 没有找到完整 PASS 的 Gao 结果。",name); end
    tasks(end+1)=struct("name",name,"resultFile",f); %#ok<AGROW>
end
if ~isempty(options.taskNames)
    keep=ismember(string({tasks.name}),string(options.taskNames));
    tasks=tasks(keep);
    if isempty(tasks), error("taskNames 没有匹配到 Seam_A/B/C。 "); end
end

manifest=repmat(struct("name","","resultFile","","qPathPoints",0, ...
    "frameCount",0,"tcpSpeedMps",0,"videoFiles",strings(0,1)),1,0);
for k=1:numel(tasks)
    S=load(tasks(k).resultFile,"result");
    if ~isfield(S,"result"), error("结果文件缺少 result：%s",tasks(k).resultFile); end
    result=S.result;
    localAssertPass(result,tasks(k).name);
    qPath=double(result.qPath);
    if size(qPath,2)~=6 || size(qPath,1)<2, error("%s qPath 无效。",tasks(k).name); end
    scene=result.scene; seam=result.seam;
    seamPts=localDenseSeam(seam,160);
    [qFrames,~]=localInterpolateTcp(scene,qPath,options.stepDeg, ...
        options.tcpSpeedMps,options.frameRate);
    views="perspective";
    if options.writeTopView, views=["perspective","top"]; end
    files=strings(numel(views),1);
    for iv=1:numel(views)
        suffix=views(iv);
        outFile=fullfile(options.outputDir,sprintf( ...
            "%s_Gao_AR_RRTstar_%s_%s.avi",tasks(k).name, ...
            options.speedLabel,suffix));
        files(iv)=string(outFile);
        localWriteVideo(scene,seamPts,qFrames,tasks(k).name,outFile, ...
            suffix,options);
    end
    row=struct("name",tasks(k).name,"resultFile",tasks(k).resultFile, ...
        "qPathPoints",size(qPath,1),"frameCount",size(qFrames,1), ...
        "tcpSpeedMps",options.tcpSpeedMps,"videoFiles",files);
    manifest(end+1)=row; %#ok<AGROW>
end

manifestFile=fullfile(options.outputDir,"gao_three_weld_video_manifest.mat");
save(manifestFile,"manifest","-v7");
fprintf("\nGao-AR-RRT* 三条焊缝视频录制完成。\n输出目录：%s\n",options.outputDir);
for k=1:numel(manifest), fprintf("  %s: %s\n",manifest(k).name,strjoin(cellstr(manifest(k).videoFiles),", ")); end
end

function f=localFirstPassingResult(dirName,stem)
f=""; a=dir(fullfile(dirName,stem+"_gao_*_result.mat"));
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

function localAssertPass(r,name)
for f=["complete","nodeCollisionFree","edgeCollisionFree"]
    if ~isfield(r,f) || ~logical(r.(f)), error("%s Gao 结果未通过 %s。",name,f); end
end
end

function P=localDenseSeam(seam,n)
if isfield(seam,"startMat") && isfield(seam,"endMat")
    p0=double(seam.startMat(:).'); p1=double(seam.endMat(:).');
elseif isfield(seam,"positionsMat")
    X=double(seam.positionsMat); p0=X(1,:); p1=X(end,:);
else
    error("焊缝结果中缺少 positionsMat。 ");
end
s=linspace(0,1,n).'; P=(1-s).*p0+s.*p1;
end

function localWriteVideo(scene,seamPts,qFrames,name,fileName,viewName,o)
fig=figure("Name",name+" Gao-AR-RRT* weld path", ...
    "Color","w","Visible",o.visible);
fig.Units="pixels"; fig.Position=o.figurePosition; fig.Resize="off";
ax=axes(fig); hold(ax,"on");
show(scene.robot,qFrames(1,:),"Parent",ax,"Visuals","on", ...
    "Collisions",localOnOff(o.showCollisions),"Frames","off", ...
    "PreservePlot",false);
for j=1:numel(scene.workpieceCollision), show(scene.workpieceCollision{j},"Parent",ax); end

hSeam=plot3(ax,seamPts(:,1),seamPts(:,2),seamPts(:,3),"-", ...
    "Color",[0.85 0 0],"LineWidth",3.5);
plot3(ax,seamPts(1,1),seamPts(1,2),seamPts(1,3),"o", ...
    "MarkerSize",9,"MarkerFaceColor",[0 0.70 0],"MarkerEdgeColor","k");
plot3(ax,seamPts(end,1),seamPts(end,2),seamPts(end,3),"o", ...
    "MarkerSize",9,"MarkerFaceColor",[0 0.30 1],"MarkerEdgeColor","k");
hTrail=plot3(ax,nan,nan,nan,"-","LineWidth",2.5,"Color",[0.95 0.72 0.05]);
hTcp=plot3(ax,nan,nan,nan,"o","MarkerSize",8,"MarkerFaceColor",[1 0.15 0.15],"MarkerEdgeColor","k");
hWire=plot3(ax,nan,nan,nan,"-","Color",[0.98 0.45 0.05],"LineWidth",4);
hWireTip=plot3(ax,nan,nan,nan,"o","MarkerSize",5,"MarkerFaceColor",[0.98 0.45 0.05],"MarkerEdgeColor",[0.4 0.1 0]);
if ~o.showTrail, hTrail.Visible="off"; end
uistack([hSeam,hTrail,hTcp,hWire,hWireTip],"top");

tcp=localTcpPositions(scene,qFrames); b=localBounds(ax,scene,qFrames,seamPts,o.showCollisions);
if viewName=="top", view(ax,0,90); else, view(ax,135,25); end
axis(ax,"equal"); axis(ax,"vis3d"); grid(ax,"on"); camproj(ax,"perspective");
if all(isfinite(b)), xlim(ax,b(1:2)); ylim(ax,b(3:4)); zlim(ax,b(5:6)); end
light(ax,"Position",[1.5 0 1.5],"Style","local","Color",[1 1 1]);

vw=VideoWriter(char(fileName),"Motion JPEG AVI"); vw.FrameRate=o.frameRate; open(vw);
fprintf("录制 Gao %s: %s (%d frames)\n",name,fileName,size(qFrames,1));
for i=1:size(qFrames,1)
    show(scene.robot,qFrames(i,:),"Parent",ax,"Visuals","on", ...
        "Collisions",localOnOff(o.showCollisions),"Frames","off","PreservePlot",false);
    T=getTransform(scene.robot,qFrames(i,:),scene.tcpBodyName); p=T(1:3,4);
    hTcp.XData=p(1); hTcp.YData=p(2); hTcp.ZData=p(3);
    if o.showTrail, hTrail.XData=tcp(1:i,1); hTrail.YData=tcp(1:i,2); hTrail.ZData=tcp(1:i,3); end
    if isfield(scene,"T_tcp_wire") && isfield(scene,"wireLengthM")
        Tw=T*scene.T_tcp_wire; a=Tw*[0;0;-scene.wireLengthM/2;1]; z=Tw*[0;0;scene.wireLengthM/2;1];
        hWire.XData=[a(1),z(1)]; hWire.YData=[a(2),z(2)]; hWire.ZData=[a(3),z(3)];
        hWireTip.XData=a(1); hWireTip.YData=a(2); hWireTip.ZData=a(3);
    end
    uistack([hSeam,hTrail,hTcp,hWire,hWireTip],"top");
    title(ax,sprintf("%s | Gao-AR-RRT* | %s | frame %d/%d",name,o.speedText,i,size(qFrames,1)));
    drawnow;
    fr=getframe(fig); writeVideo(vw,fr);
    pause(1/(o.frameRate*max(1,o.speedFactor)));
end
close(vw); if isvalid(fig), close(fig); end
end

function [qFrames,pointIdx]=localInterpolateTcp(scene,qPath,stepDeg,speed,frameRate)
denseStep=max(stepDeg/8,0.05); qDense=qPath(1,:);
for i=1:size(qPath,1)-1
    dq=qPath(i+1,:)-qPath(i,:); n=max(1,ceil(rad2deg(max(abs(dq)))/denseStep));
    for k=1:n, qDense(end+1,:)=qPath(i,:)+(k/n)*dq; %#ok<AGROW>
    end
end
p=localTcpPositions(scene,qDense); arc=[0;cumsum(vecnorm(diff(p,1,1),2,2))];
[a,keep]=unique(arc,"stable"); Q=qDense(keep,:); if a(end)<=eps, qFrames=qPath(1,:); pointIdx=1; return; end
n=max(1,round(a(end)/speed*frameRate)); x=linspace(0,a(end),n+1).'; qFrames=localInterpQ(a,Q,x);
for pass=1:6
    p=localTcpPositions(scene,qFrames); aa=[0;cumsum(vecnorm(diff(p,1,1),2,2))]; [au,ku]=unique(aa,"stable");
    n=max(1,round(au(end)/speed*frameRate)); qFrames=localInterpQ(au,qFrames(ku,:),linspace(0,au(end),n+1).');
end
qFrames(1,:)=qPath(1,:); qFrames(end,:)=qPath(end,:); pointIdx=[];
end

function q=localInterpQ(a,Q,x), q=zeros(numel(x),size(Q,2)); for j=1:size(Q,2), q(:,j)=interp1(a,Q(:,j),x,"linear","extrap"); end, end
function p=localTcpPositions(scene,q)
p=zeros(size(q,1),3); for i=1:size(q,1), p(i,:)=tform2trvec(getTransform(scene.robot,q(i,:),scene.tcpBodyName)); end
end
function b=localBounds(ax,scene,q,seam,coll)
% Include every robot body frame over the whole trajectory.  The previous
% version bounded the camera only by TCP/seam points, which cropped the
% robot base and part of the workpiece out of the Gao videos.
allP=[localTcpPositions(scene,q);seam];
for iq=1:size(q,1)
    for ib=1:numel(scene.robot.Bodies)
        try
            T=getTransform(scene.robot,q(iq,:),scene.robot.Bodies{ib}.Name);
            allP(end+1,:)=T(1:3,4).'; %#ok<AGROW>
        catch
        end
    end
end

% Add the exact world-space AABBs of the MATLAB workpiece proxies.
if isfield(scene,"workpieceProxyInfo")
    info=scene.workpieceProxyInfo;
    for j=1:numel(info)
        if isfield(info(j),"T_matbase_proxy")
            T=info(j).T_matbase_proxy;
        elseif isfield(info(j),"T_workpiece_proxy") && isfield(scene,"T_MatBase_Workpiece")
            T=scene.T_MatBase_Workpiece*info(j).T_workpiece_proxy;
        else
            continue;
        end
        s=double(info(j).size_m(:).');
        c=[-1 -1 -1; -1 -1 1; -1 1 -1; -1 1 1; ...
            1 -1 -1; 1 -1 1; 1 1 -1; 1 1 1].*(s/2);
        C=(T*[c,ones(8,1)].').';
        allP=[allP;C(:,1:3)]; %#ok<AGROW>
    end
end

% The hull/wire can extend beyond their body origins; a generous but fixed
% margin keeps the complete tool visible without changing geometry.
lo=min(allP,[],1)-[.16 .16 .16]; hi=max(allP,[],1)+[.16 .16 .16];
b=[lo(1),hi(1),lo(2),hi(2),lo(3),hi(3)];
end
function y=localOnOff(tf), if tf,y="on";else,y="off";end,end
function o=localMerge(d,in), o=d; if isempty(in),return;end; f=fieldnames(d); for i=1:numel(f), if isfield(in,f{i})&&~isempty(in.(f{i})),o.(f{i})=in.(f{i});end,end; end
