function info = check_branch_anchor_alignment()
%CHECK_BRANCH_ANCHOR_ALIGNMENT Read-only preflight for the HMI weld seam task.
%
% Confirms that weld_anchor_points.json qWeldStart really corresponds to
% point 1 of the 16-point, 25 mm ACCP seam (bottom x wall_y1) before it
% is used as an IK seed / branch reference.  This script plans nothing, sends
% no ROS2 command and does not modify the scene, the planner or the robot.

here = fileparts(mfilename("fullpath"));
rootDir = fileparts(here);
addpath(genpath(fileparts(fileparts(here))));
addpath(here);
setenv("RRT_HEADLESS","1");

anchorFile = fullfile(rootDir,"规划文件","weld_anchor_points.json");
A = jsondecode(fileread(anchorFile));
qWeldStart = double(A.qWeldStart(:).');
qWeldEnd   = double(A.qWeldEnd(:).');
qPreWeld   = double(A.qPreWeld(:).');
qPostWeld  = double(A.qPostWeld(:).');

scene = loadUR5JointScene(fullfile(here,"sync_ur5_joint_states.m"));
fprintf("tcpBodyName        : %s\n",scene.tcpBodyName);
fprintf("scene.qMat0 (deg)   : [%.3f %.3f %.3f %.3f %.3f %.3f]\n",rad2deg(scene.qMat0));
fprintf("qWeldStart (deg)    : [%.3f %.3f %.3f %.3f %.3f %.3f]\n",rad2deg(qWeldStart));
fprintf("qWeldEnd   (deg)    : [%.3f %.3f %.3f %.3f %.3f %.3f]\n",rad2deg(qWeldEnd));
fprintf("qPreWeld    (deg)    : [%.3f %.3f %.3f %.3f %.3f %.3f]\n",rad2deg(qPreWeld));
fprintf("qPostWeld   (deg)    : [%.3f %.3f %.3f %.3f %.3f %.3f]\n",rad2deg(qPostWeld));

Tstart = getTransform(scene.robot,qWeldStart,scene.tcpBodyName);
Tend   = getTransform(scene.robot,qWeldEnd,scene.tcpBodyName);
Tpre   = getTransform(scene.robot,qPreWeld,scene.tcpBodyName);
Tpost  = getTransform(scene.robot,qPostWeld,scene.tcpBodyName);

fprintf("\nFK of anchors (MATLAB base_link, m):\n");
fprintf("  qWeldStart : [%.6f %.6f %.6f]\n",Tstart(1:3,4));
fprintf("  qWeldEnd   : [%.6f %.6f %.6f]\n",Tend(1:3,4));
fprintf("  qPreWeld   : [%.6f %.6f %.6f]\n",Tpre(1:3,4));
fprintf("  qPostWeld  : [%.6f %.6f %.6f]\n",Tpost(1:3,4));

% Expected 16-point seam A geometry for a 25 mm end margin.
expectedStart = [-0.605000 -0.045000 0.005000];
expectedEnd   = [-0.605000  0.130000 0.005000];

dStart = norm(Tstart(1:3,4).'-expectedStart);
dEnd   = norm(Tend(1:3,4).'-expectedEnd);
fprintf("\nDistance to the historical seam A endpoints:\n");
fprintf("  qWeldStart vs seam start : %.6f mm\n",dStart*1000);
fprintf("  qWeldEnd   vs seam end   : %.6f mm\n",dEnd*1000);

fprintf("\nPreWeld/PostWeld retraction along the measured wire axis:\n");
preOffset  = Tpre(1:3,4)-Tstart(1:3,4);
postOffset = Tpost(1:3,4)-Tend(1:3,4);
fprintf("  PreWeld  offset norm : %.4f mm\n",norm(preOffset)*1000);
fprintf("  PostWeld offset norm : %.4f mm\n",norm(postOffset)*1000);

% Collision state of each anchor under the complete check.
info = struct();
info.qWeldStart = qWeldStart;
info.qWeldEnd = qWeldEnd;
info.qPreWeld = qPreWeld;
info.qPostWeld = qPostWeld;
info.qMat0 = double(scene.qMat0(:).');
info.anchorTcpStartMat = Tstart(1:3,4).';
info.anchorTcpEndMat = Tend(1:3,4).';
info.expectedSeamStartMat = expectedStart;
info.expectedSeamEndMat = expectedEnd;
info.seamStartErrorMm = dStart*1000;
info.seamEndErrorMm = dEnd*1000;
info.preWeldOffsetMm = norm(preOffset)*1000;
info.postWeldOffsetMm = norm(postOffset)*1000;

freeNames = ["qWeldStart","qWeldEnd","qPreWeld","qPostWeld","scene.qMat0"];
freeQ = [qWeldStart;qWeldEnd;qPreWeld;qPostWeld;double(scene.qMat0(:).')];
info.anchorFree = false(size(freeQ,1),1);
for i = 1:size(freeQ,1)
    free = localStateFree(scene,freeQ(i,:));
    info.anchorFree(i) = free;
    fprintf("  %-12s collision-free = %d\n",freeNames(i),free);
end

fprintf("\nSplice jumps without any 2pi lifting (deg):\n");
fprintf("  qPreWeld  -> qWeldStart : %.3f\n",max(abs(rad2deg(qWeldStart-qPreWeld))));
fprintf("  qWeldEnd  -> qPostWeld : %.3f\n",max(abs(rad2deg(qPostWeld-qWeldEnd))));
fprintf("  qWeldEnd   -> qPostWeld: %.3f\n",max(abs(rad2deg(qPostWeld-qWeldEnd))));
fprintf("  qWeldStart -> qWeldEnd : %.3f\n",max(abs(rad2deg(qWeldEnd-qWeldStart))));
info.splicePreToStartDeg = max(abs(rad2deg(qWeldStart-qPreWeld)));
info.spliceEndToPostDeg = max(abs(rad2deg(qPostWeld-qWeldEnd)));

fprintf("\nBRANCH_ANCHOR_CHECK done. seamStartErrorMm=%.4f seamEndErrorMm=%.4f\n", ...
    info.seamStartErrorMm,info.seamEndErrorMm);
end

function free = localStateFree(scene,q)
try
    [env,~,~] = checkCollision(scene.robot,q,scene.environment, ...
        "IgnoreSelfCollision","on","Exhaustive","on");
catch
    env = checkCollision(scene.robot,q,scene.environment,"IgnoreSelfCollision","on");
end
if any(env(:)), free = false; return; end
try
    self = checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",scene.selfCollisionSkipPairs,"Exhaustive","on");
catch
    self = checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",scene.selfCollisionSkipPairs);
end
free = ~any(self(:));
end
