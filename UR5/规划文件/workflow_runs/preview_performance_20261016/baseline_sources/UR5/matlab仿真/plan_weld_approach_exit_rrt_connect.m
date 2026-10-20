function result = plan_weld_approach_exit_rrt_connect(options)
%PLAN_WELD_APPROACH_EXIT_RRT_CONNECT Plan free-space approach and exit.
%
% Reads the validated 80-point weld result and plans only:
%   qHome -> qPreWeld,   qPostWeld -> qHome
% with bidirectional RRT-Connect in six-dimensional joint space.
% The scene, TCP calibration, ROS2 controller, and saved weld path are not
% modified.  Collision checks include the V-HACD gun, MATLAB wire cylinder,
% four Workpiece proxies, and non-structural robot self-collisions.

arguments
    options struct = struct()
end

here = fileparts(mfilename("fullpath"));
defaults = struct( ...
    "resultFile",fullfile(here,"weld_actual_axis_replan.mat"), ...
    "outputFile",fullfile(here,"weld_rrt_connect_approach_exit.mat"), ...
    "reportFile",fullfile(here,"weld_rrt_connect_approach_exit_report.txt"), ...
    "preDistancesM",[0.030 0.050 0.075 0.100 0.125], ...
    "postDistancesM",[0.030 0.050 0.075 0.100 0.125], ...
    "numIKRestarts",96, ...
    "positionToleranceM",1e-4, ...
    "orientationToleranceDeg",0.5, ...
    "stepDeg",12, ...
    "goalBias",0.20, ...
    "maxIterations",8000, ...
    "edgeResolutionDeg",2, ...
    "validationResolutionDeg",0.5, ...
    "randomSeed",20260917, ...
    "saveResult",true);
options = localMergeOptions(defaults,options);

if ~isfile(options.resultFile)
    error("找不到已验收焊缝结果：%s",options.resultFile);
end

scene = localLoadScene(fullfile(here,"sync_ur5_joint_states.m"));
s = load(options.resultFile,"result");
weldResult = s.result;
if ~weldResult.complete || isempty(weldResult.qPath) || size(weldResult.qPath,2)~=6
    error("现有 weld_actual_axis_replan.mat 不是完整的 80×6 焊缝结果。");
end

qWeld = unwrap(double(weldResult.qPath),[],1);
qHome = double(scene.qMat0(:).');
scene.selfCollisionSkipPairs = localStructuralSelfCollisionSkips(scene.robot);

fprintf("\n========================================\n");
fprintf("RRT-CONNECT APPROACH / EXIT PLANNING\n");
fprintf("========================================\n");
fprintf("Weld result       : %s\n",options.resultFile);
fprintf("Weld path         : %d x %d rad\n",size(qWeld,1),size(qWeld,2));
fprintf("qHome [deg]       : [%s]\n",sprintf(" %.3f",rad2deg(qHome)));
fprintf("Point 1 [deg]     : [%s]\n",sprintf(" %.3f",rad2deg(qWeld(1,:))));
fprintf("Point 80 [deg]    : [%s]\n",sprintf(" %.3f",rad2deg(qWeld(end,:))));

TPoint1 = getTransform(scene.robot,qWeld(1,:),scene.tcpBodyName);
TPoint80 = getTransform(scene.robot,qWeld(end,:),scene.tcpBodyName);
gStartB = tform2rotm(TPoint1)*scene.wireAxisTcp(:); gStartB=gStartB/norm(gStartB);
gEndB = tform2rotm(TPoint80)*scene.wireAxisTcp(:); gEndB=gEndB/norm(gEndB);

fprintf("\n--- PreWeld candidate search ---\n");
pre = localFindTransitionPose(scene,TPoint1,gStartB,qWeld(1,:), ...
    options.preDistancesM,options,"PreWeld");
if ~pre.success
    fprintf(2,"[FAIL] No collision-free continuous PreWeld pose found.\n");
    result = localFailureResult(weldResult,scene,qHome,qWeld,pre, ...
        localEmptyTransition(),options,"PreWeld candidate search failed");
    localSaveAndReport(result,options);
    return;
end

fprintf("\n--- PostWeld candidate search ---\n");
post = localFindTransitionPose(scene,TPoint80,gEndB,qWeld(end,:), ...
    options.postDistancesM,options,"PostWeld");
if ~post.success
    fprintf(2,"[FAIL] No collision-free continuous PostWeld pose found.\n");
    result = localFailureResult(weldResult,scene,qHome,qWeld,pre,post, ...
        options,"PostWeld candidate search failed");
    localSaveAndReport(result,options);
    return;
end

fprintf("\nSelected PreWeld  : %.1f mm, q=[%s]\n",pre.distanceM*1000, ...
    sprintf(" %.3f",rad2deg(pre.q)));
fprintf("Selected PostWeld : %.1f mm, q=[%s]\n",post.distanceM*1000, ...
    sprintf(" %.3f",rad2deg(post.q)));

fprintf("\n--- RRT-Connect: qHome -> qPreWeld ---\n");
approach = localRrtConnect(scene,qHome,pre.q,options,"Approach");
fprintf("\n--- RRT-Connect: qPostWeld -> qHome ---\n");
exitPath = localRrtConnect(scene,post.q,qHome,options,"Exit");

fprintf("\n--- Independent high-density validation ---\n");
approach.validation = localValidatePath(scene,approach.qPath,options);
exitPath.validation = localValidatePath(scene,exitPath.qPath,options);
weldValidation = localValidatePath(scene,qWeld,options);
preWeldValidation = localValidatePath(scene,[pre.q;qWeld(1,:)],options);
postWeldValidation = localValidatePath(scene,[qWeld(end,:);post.q],options);

approach.pass = approach.success && approach.validation.pass;
exitPath.pass = exitPath.success && exitPath.validation.pass;
weldPass = weldResult.complete && weldValidation.pass;
preWeldPass = pre.edgeFree && preWeldValidation.pass;
postWeldPass = post.edgeFree && postWeldValidation.pass;
fullPass = approach.pass && preWeldPass && weldPass && postWeldPass && exitPath.pass;

fprintf("\n========================================\n");
fprintf("RRT-CONNECT RESULT\n");
fprintf("========================================\n");
fprintf("Approach RRT PASS      : %d\n",approach.pass);
fprintf("PreWeld -> Point 1     : %d\n",preWeldPass);
fprintf("Existing weld 1 -> 80  : %d (not replanned)\n",weldPass);
fprintf("Point 80 -> PostWeld   : %d\n",postWeldPass);
fprintf("Exit RRT PASS          : %d\n",exitPath.pass);
fprintf("Overall PASS           : %d\n",fullPass);

result = struct();
result.created = string(datetime("now","Format","yyyy-MM-dd HH:mm:ss"));
result.sourceWeldResultFile = options.resultFile;
result.scene = scene;
result.qHome = qHome;
result.qWeld = qWeld;
result.weldResult = weldResult;
result.preWeld = pre;
result.postWeld = post;
result.approach = approach;
result.exit = exitPath;
result.weldValidation = weldValidation;
result.preWeldValidation = preWeldValidation;
result.postWeldValidation = postWeldValidation;
result.fullPass = fullPass;
result.options = options;
result.note = "80-point weld path was read from the validated result and was not replanned.";
localSaveAndReport(result,options);
end

%% Candidate endpoint IK and transition checks

function transition = localFindTransitionPose(scene,Tweld,axisB,qRef,distances,options,label)
transition = localEmptyTransition();
transition.label = string(label);
transition.axisBase = axisB(:).';
allCandidates = repmat(localEmptyCandidate(),0,1);

for d = distances(:).'
    Ttarget = Tweld;
    Ttarget(1:3,4) = Tweld(1:3,4) - d*axisB(:);
    rng(options.randomSeed + round(d*1e6) + double(label=="PostWeld"),"twister");
    qSeeds = [qRef;scene.qMat0];
    for k = 1:options.numIKRestarts
        qSeeds(end+1,:) = randomConfiguration(scene.robot); %#ok<AGROW>
    end
    cand = localSolveFreeSpaceIK(scene,Ttarget,qSeeds,options);
    for k = 1:numel(cand)
        cand(k).distanceM = d;
        [cand(k).edgeFree,cand(k).edgeDiag] = ...
            localEdgeFree(scene,qRef,cand(k).q,options,localEmptyRrtStats());
    end
    allCandidates = [allCandidates;cand(:)]; %#ok<AGROW>
    free = cand([cand.stateFree] & [cand.edgeFree]);
    fprintf("  %s %.1f mm: pose-valid=%d, state-free=%d, edge-free=%d\n", ...
        label,d*1000,numel(cand),sum([cand.stateFree]),numel(free));
    if ~isempty(free)
        [~,best] = min(arrayfun(@(x)max(abs(rad2deg(localWrapRad(x.q-qRef)))),free));
        chosen = free(best);
        transition.success=true;
        transition.label=string(label);
        transition.distanceM=d;
        transition.q=chosen.q;
        transition.Ttarget=chosen.Ttarget;
        transition.Tactual=chosen.Tactual;
        transition.axisBase=axisB(:).';
        transition.positionErrorM=chosen.positionErrorM;
        transition.orientationErrorDeg=chosen.orientationErrorDeg;
        transition.edgeFree=chosen.edgeFree;
        transition.edgeDiag=chosen.edgeDiag;
        transition.candidates=allCandidates;
        return;
    end
end
transition.candidates=allCandidates;
end

function candidates = localSolveFreeSpaceIK(scene,Ttarget,qSeeds,options)
ik=inverseKinematics("RigidBodyTree",scene.robot); weights=[1 1 1 1 1 1];
limits=localJointLimits(scene.robot);
candidates=repmat(localEmptyCandidate(),0,1);
for k=1:size(qSeeds,1)
    seed=double(qSeeds(k,:));
    try, [qSol,~]=ik(scene.tcpBodyName,Ttarget,weights,seed); catch, continue; end
    qSol=double(qSol(:).');
    if numel(qSol)~=6 || any(~isfinite(qSol)), continue; end
    qSol=seed+localWrapRad(qSol-seed);
    if any(qSol<limits(:,1).'-1e-8) || any(qSol>limits(:,2).'+1e-8), continue; end
    Tactual=getTransform(scene.robot,qSol,scene.tcpBodyName);
    pErr=norm(Tactual(1:3,4)-Ttarget(1:3,4));
    Rerr=Ttarget(1:3,1:3).'*Tactual(1:3,1:3);
    oErr=acos(max(-1,min(1,(trace(Rerr)-1)/2)));
    if pErr>options.positionToleranceM || rad2deg(oErr)>options.orientationToleranceDeg, continue; end
    [free,diag]=localStateFree(scene,qSol,localEmptyRrtStats());
    c=localEmptyCandidate(); c.q=qSol; c.Ttarget=Ttarget; c.Tactual=Tactual;
    c.positionErrorM=pErr; c.orientationErrorDeg=rad2deg(oErr);
    c.stateFree=free; c.stateDiag=diag;
    candidates(end+1,1)=c; %#ok<AGROW>
end
candidates=localDeduplicateCandidates(candidates,0.5);
end

function out=localDeduplicateCandidates(in,tolDeg)
out=repmat(localEmptyCandidate(),0,1);
for i=1:numel(in)
    duplicate=false;
    for j=1:numel(out)
        if max(abs(rad2deg(localWrapRad(in(i).q-out(j).q))))<tolDeg
            duplicate=true; break;
        end
    end
    if ~duplicate, out(end+1,1)=in(i); end %#ok<AGROW>
end
end

%% Bidirectional RRT-Connect

function plan=localRrtConnect(scene,qStart,qGoal,options,label)
tic;
rng(options.randomSeed+double(label=="Exit")*7919,"twister");
limits=localJointLimits(scene.robot);
treeStart=localNewTree(qStart); treeGoal=localNewTree(qGoal);
stats=localEmptyRrtStats(); stats.start=qStart; stats.goal=qGoal;
connected=false; meetingStart=1; meetingGoal=1; swapped=false;

for iter=1:options.maxIterations
    stats.iterations=iter;
    if rand<options.goalBias, qRand=qGoal;
    else, qRand=limits(:,1).'+rand(1,6).*(limits(:,2)-limits(:,1)).'; end
    stats.samples=stats.samples+1;
    [treeStart,idxA,statusA,stats]=localExtend(treeStart,qRand,scene,options,stats);
    if strcmp(statusA,"trapped")
        [treeStart,treeGoal,swapped]=localSwapTrees(treeStart,treeGoal,swapped);
        continue;
    end
    qNew=treeStart.q(idxA,:);
    [treeGoal,idxB,statusB,stats]=localConnect(treeGoal,qNew,scene,options,stats);
    if strcmp(statusB,"reached")
        connected=true;
        if ~swapped, meetingStart=idxA; meetingGoal=idxB;
        else, meetingStart=idxB; meetingGoal=idxA; end
        break;
    end
    [treeStart,treeGoal,swapped]=localSwapTrees(treeStart,treeGoal,swapped);
end

stats.treeStartNodes=size(treeStart.q,1); stats.treeGoalNodes=size(treeGoal.q,1);
stats.planningTimeSec=toc; stats.success=connected;
plan=struct("label",string(label),"success",connected,"stats",stats, ...
    "qPath",zeros(0,6),"pass",false);
if connected
    qA=localTraceTree(treeStart,meetingStart); qB=localTraceTree(treeGoal,meetingGoal);
    if ~swapped, qPath=[qA;flipud(qB(1:end-1,:))];
    else, qPath=[qB;flipud(qA(1:end-1,:))]; end
    plan.qPath=qPath;
    plan.stats.pathLengthRad=sum(vecnorm(diff(qPath,1,1),2,2));
    if size(qPath,1)>1
        plan.stats.maxJointJumpDeg=max(abs(rad2deg(localWrapRad(diff(qPath,1,1)))),[],'all');
    else, plan.stats.maxJointJumpDeg=0; end
end
fprintf("%s: success=%d time=%.3f s iterations=%d samples=%d nodes=%d collisionChecks=%d edgeChecks=%d\n", ...
    label,plan.success,stats.planningTimeSec,stats.iterations,stats.samples, ...
    stats.treeStartNodes+stats.treeGoalNodes,stats.collisionChecks,stats.edgeChecks);
end

function tree=localNewTree(q)
tree.q=double(q(:).'); tree.parent=0;
end

function [tree,idx,status,stats]=localExtend(tree,qTarget,scene,options,stats)
qNearIdx=localNearest(tree.q,qTarget); qNear=tree.q(qNearIdx,:);
dq=localWrapRad(qTarget-qNear); maxDelta=max(abs(rad2deg(dq)));
if maxDelta<1e-8, idx=qNearIdx; status="reached"; return; end
scale=min(1,options.stepDeg/maxDelta); qNew=qNear+scale*dq;
[free,stats]=localEdgeFree(scene,qNear,qNew,options,stats);
if ~free, idx=qNearIdx; status="trapped"; return; end
tree.q(end+1,:)=qNew; tree.parent(end+1,1)=qNearIdx; idx=size(tree.q,1);
if scale>=1-1e-9, status="reached"; else, status="advanced"; end
end

function [tree,idx,status,stats]=localConnect(tree,qTarget,scene,options,stats)
idx=localNearest(tree.q,qTarget); status="advanced";
while true
    [tree,idxNew,statusExt,stats]=localExtend(tree,qTarget,scene,options,stats);
    if strcmp(statusExt,"trapped"), status="trapped"; idx=idxNew; return; end
    idx=idxNew;
    if strcmp(statusExt,"reached"), status="reached"; return; end
end
end

function idx=localNearest(Q,q)
[~,idx]=min(vecnorm(localWrapRad(Q-q),2,2));
end

function qPath=localTraceTree(tree,idx)
qPath=zeros(0,6);
while idx>0
    qPath(end+1,:)=tree.q(idx,:); %#ok<AGROW>
    idx=tree.parent(idx);
end
qPath=flipud(qPath);
end

function [a,b,swapped]=localSwapTrees(a,b,swapped)
tmp=a; a=b; b=tmp; swapped=~swapped;
end

%% Collision and validation

function [ok,stats]=localStateFree(scene,q,stats)
if nargin<3 || isempty(stats), stats=localEmptyRrtStats(); end
stats.collisionChecks=stats.collisionChecks+1;
limits=localJointLimits(scene.robot);
if any(q<limits(:,1).'-1e-8) || any(q>limits(:,2).'+1e-8), ok=false; return; end
try
    [env,~,~]=checkCollision(scene.robot,q,scene.environment, ...
        "IgnoreSelfCollision","on","Exhaustive","on");
catch
    env=checkCollision(scene.robot,q,scene.environment,"IgnoreSelfCollision","on");
end
if any(env(:)), ok=false; return; end
try
    self=checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",scene.selfCollisionSkipPairs,"Exhaustive","on");
catch
    self=checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",scene.selfCollisionSkipPairs);
end
ok=~any(self(:));
end

function [ok,stats]=localEdgeFree(scene,q1,q2,options,stats)
if nargin<5 || isempty(stats), stats=localEmptyRrtStats(); end
stats.edgeChecks=stats.edgeChecks+1;
dq=localWrapRad(q2-q1);
n=max(1,ceil(max(abs(rad2deg(dq)))/options.edgeResolutionDeg)); ok=true;
for k=0:n
    q=q1+(k/n)*dq; [free,stats]=localStateFree(scene,q,stats);
    if ~free, ok=false; return; end
end
stats.collisionFreeEdges=stats.collisionFreeEdges+1;
end

function validation=localValidatePath(scene,qPath,options)
validation=struct("pass",false,"nodeCount",size(qPath,1), ...
    "edgeCount",max(0,size(qPath,1)-1),"nodeCollisionFree",false, ...
    "edgeCollisionFree",false,"collisionNodes",[],"collisionEdges",[], ...
    "maxJointJumpDeg",NaN,"collisionChecks",0,"edgeChecks",0);
if isempty(qPath), return; end
stats=localEmptyRrtStats(); badNode=false(size(qPath,1),1);
for i=1:size(qPath,1)
    [free,stats]=localStateFree(scene,qPath(i,:),stats); badNode(i)=~free;
end
badEdge=false(max(0,size(qPath,1)-1),1);
for i=1:size(qPath,1)-1
    [free,stats]=localEdgeFree(scene,qPath(i,:),qPath(i+1,:), ...
        struct("edgeResolutionDeg",options.validationResolutionDeg),stats);
    badEdge(i)=~free;
end
validation.collisionNodes=find(badNode); validation.collisionEdges=find(badEdge);
validation.nodeCollisionFree=~any(badNode); validation.edgeCollisionFree=~any(badEdge);
validation.pass=validation.nodeCollisionFree && validation.edgeCollisionFree;
validation.collisionChecks=stats.collisionChecks; validation.edgeChecks=stats.edgeChecks;
if size(qPath,1)>1
    validation.maxJointJumpDeg=max(abs(rad2deg(localWrapRad(diff(qPath,1,1)))),[],'all');
else, validation.maxJointJumpDeg=0; end
end

%% Reporting and utility functions

function result=localFailureResult(weldResult,scene,qHome,qWeld,pre,post,options,note)
result=struct(); result.created=string(datetime("now","Format","yyyy-MM-dd HH:mm:ss"));
result.sourceWeldResultFile=options.resultFile; result.scene=scene; result.qHome=qHome;
result.qWeld=qWeld; result.weldResult=weldResult; result.preWeld=pre; result.postWeld=post;
result.approach=struct("success",false,"pass",false,"qPath",zeros(0,6));
result.exit=struct("success",false,"pass",false,"qPath",zeros(0,6));
result.fullPass=false; result.options=options; result.note=string(note);
end

function localSaveAndReport(result,options)
if isfield(options,"saveResult") && options.saveResult
    save(options.outputFile,"result","-v7.3"); fprintf("Saved MAT result: %s\n",options.outputFile);
end
fid=fopen(options.reportFile,"w");
if fid<0, warning("无法写入报告文件：%s",options.reportFile); return; end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"RRT-Connect approach/exit report\n");
fprintf(fid,"Created: %s\n",string(result.created));
fprintf(fid,"Source weld result: %s\n",result.sourceWeldResultFile);
fprintf(fid,"Overall PASS: %d\n",result.fullPass);
if isfield(result,"preWeld") && isfield(result.preWeld,"success")
    fprintf(fid,"PreWeld success=%d distance_m=%.6f edgeFree=%d\n", ...
        result.preWeld.success,localGet(result.preWeld,"distanceM",NaN),localGet(result.preWeld,"edgeFree",false));
end
if isfield(result,"postWeld") && isfield(result.postWeld,"success")
    fprintf(fid,"PostWeld success=%d distance_m=%.6f edgeFree=%d\n", ...
        result.postWeld.success,localGet(result.postWeld,"distanceM",NaN),localGet(result.postWeld,"edgeFree",false));
end
for label=["approach","exit"]
    if ~isfield(result,char(label)), continue; end
    p=result.(char(label));
    fprintf(fid,"%s success=%d pass=%d\n",label,localGet(p,"success",false),localGet(p,"pass",false));
    if isfield(p,"stats")
        st=p.stats;
        fprintf(fid,"  time_s=%.6f iterations=%d samples=%d nodes=%d collision_checks=%d edge_checks=%d path_length_rad=%.6f max_jump_deg=%.6f\n", ...
            localGet(st,"planningTimeSec",NaN),localGet(st,"iterations",0),localGet(st,"samples",0), ...
            localGet(st,"treeStartNodes",0)+localGet(st,"treeGoalNodes",0),localGet(st,"collisionChecks",0), ...
            localGet(st,"edgeChecks",0),localGet(st,"pathLengthRad",NaN),localGet(st,"maxJointJumpDeg",NaN));
    end
    if isfield(p,"validation")
        v=p.validation;
        fprintf(fid,"  validation nodes=%d edges=%d node_free=%d edge_free=%d\n", ...
            v.nodeCount,v.edgeCount,v.nodeCollisionFree,v.edgeCollisionFree);
    end
end
fprintf(fid,"Note: Existing 80-point weld path is read-only and was not replanned.\n");
fprintf("Saved text report: %s\n",options.reportFile);
end

function value=localGet(s,name,default)
if isstruct(s) && isfield(s,name), value=s.(name); else, value=default; end
end

function scene=localLoadScene(syncFile)
run(syncFile);
end

function limits=localJointLimits(robot)
limits=zeros(6,2); j=0;
for i=1:numel(robot.Bodies)
    jt=robot.Bodies{i}.Joint;
    if strcmpi(jt.Type,"revolute") || strcmpi(jt.Type,"continuous")
        j=j+1; limits(j,:)=double(jt.PositionLimits);
    end
end
if j~=6, error("UR5 revolute joint count is %d, expected 6.",j); end
end

function pairs=localStructuralSelfCollisionSkips(robot)
names=robot.BodyNames; pairs=cell(0,2);
for i=1:min(10,numel(names)-1), pairs(end+1,:)={names{i},names{i+1}}; %#ok<AGROW>
end
pairs(end+1,:)={"WeldingGunMount","wrist_3_link"};
pairs(end+1,:)={"WeldingGunMount","ee_link"};
pairs(end+1,:)={"WeldingGunMount","WeldTCP"};
end

function x=localWrapRad(x)
x=atan2(sin(x),cos(x));
end

function s=localMergeOptions(defaults,in)
s=defaults; f=fieldnames(defaults);
for i=1:numel(f)
    if isfield(in,f{i}) && ~isempty(in.(f{i})), s.(f{i})=in.(f{i}); end
end
end

function s=localEmptyTransition()
s=struct("success",false,"label","","distanceM",NaN,"q",zeros(1,6), ...
    "Ttarget",eye(4),"Tactual",eye(4),"axisBase",zeros(1,3), ...
    "positionErrorM",NaN,"orientationErrorDeg",NaN,"edgeFree",false, ...
    "edgeDiag",localEmptyRrtStats(),"candidates",repmat(localEmptyCandidate(),0,1));
end

function s=localEmptyCandidate()
s=struct("q",zeros(1,6),"Ttarget",eye(4),"Tactual",eye(4), ...
    "positionErrorM",NaN,"orientationErrorDeg",NaN,"stateFree",false, ...
    "stateDiag",localEmptyRrtStats(),"distanceM",NaN,"edgeFree",false, ...
    "edgeDiag",localEmptyRrtStats());
end

function s=localEmptyRrtStats()
s=struct("start",zeros(1,6),"goal",zeros(1,6),"iterations",0,"samples",0, ...
    "collisionChecks",0,"edgeChecks",0,"collisionFreeEdges",0, ...
    "treeStartNodes",0,"treeGoalNodes",0,"planningTimeSec",NaN, ...
    "success",false,"pathLengthRad",NaN,"maxJointJumpDeg",NaN);
end
