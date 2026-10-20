function diagnose_weld_anchor_branch()
%DIAGNOSE_WELD_ANCHOR_BRANCH Read-only check of the weld-anchor joint branch.
%
% Reproduces the findings recorded in AGENT_HANDOFF.md section 150.3:
% the 80-point path produced by replan_weld_actual_axis_adaptive lands on a
% different UR5 IK branch than the PreWeld/PostWeld anchors stored in
% 规划文件/weld_anchor_points.json, so concatenating [qPre; qSeam; qPost]
% is discontinuous (~158 deg at the base).
%
% This script is strictly read-only.  It does not plan, does not write any
% file, does not start ROS2/CoppeliaSim, and does not command the robot.
%
% Usage:  matlab -batch "diagnose_weld_anchor_branch"

matlabDir = fileparts(mfilename('fullpath'));
rrtRoot = fileparts(fileparts(matlabDir));
addpath(genpath(rrtRoot));
addpath(matlabDir);
setenv('RRT_HEADLESS','1');

anchorFile = fullfile(fileparts(matlabDir),'规划文件','weld_anchor_points.json');
progressiveFile = fullfile(matlabDir,'weld_actual_axis_replan_adaptive_progressive.mat');
anchorSourceFile = fullfile(matlabDir,'weld_rrt_connect_approach_exit.mat');

fprintf('\n=== 1. anchors ===\n');
anchors = jsondecode(fileread(anchorFile));
fprintf('anchor fields : %s\n',strjoin(fieldnames(anchors)',', '));
if isfield(anchors,'qHome')
    fprintf('qHome present : [%s]\n',sprintf(' %.6f',anchors.qHome));
else
    fprintf('qHome MISSING -> plan_hmi_weld_request.m:274 silently falls back to zeros(1,6)\n');
end
qPre = double(anchors.qPreWeld(:).');
qPost = double(anchors.qPostWeld(:).');
fprintf('qPreWeld      : [%s]\n',sprintf(' %.6f',qPre));
fprintf('qPostWeld     : [%s]\n',sprintf(' %.6f',qPost));

fprintf('\n=== 2. scene (current) ===\n');
scene = loadUR5JointScene(fullfile(matlabDir,'sync_ur5_joint_states.m'));
fprintf('scene.qMat0   : [%s]\n',sprintf(' %.6f',scene.qMat0));

fprintf('\n=== 3. adaptive planner output (re-planned, cold start) ===\n');
if ~isfile(progressiveFile)
    fprintf('SKIP: %s not found\n',progressiveFile);
    return;
end
s = load(progressiveFile,'result');
r = s.result;
fprintf('complete      : %d\n',r.complete);
fprintf('seam startMat : [%s]\n',sprintf(' %.6f',r.seam.startMat));
fprintf('seam endMat   : [%s]\n',sprintf(' %.6f',r.seam.endMat));
qSeamStart = double(r.qPath(1,:));
qSeamEnd = double(r.qPath(end,:));
fprintf('qPath(1,:)    : [%s]\n',sprintf(' %.6f',qSeamStart));
fprintf('qPath(end,:)  : [%s]\n',sprintf(' %.6f',qSeamEnd));

fprintf('\n=== 4. branch comparison (wrapped, deg) ===\n');
dStart = rad2deg(localWrap(qSeamStart-qPre));
dEnd = rad2deg(localWrap(qSeamEnd-qPost));
fprintf('qPath(1)-qPreWeld   : [%s]\n',sprintf(' %8.2f',dStart));
fprintf('qPath(end)-qPostWeld: [%s]\n',sprintf(' %8.2f',dEnd));
fprintf('max |delta|         : %.2f deg (start) / %.2f deg (end)\n', ...
    max(abs(dStart)),max(abs(dEnd)));
fprintf('raw (unwrapped) jump to splice [qPre;qSeam;qPost]: %.2f deg\n', ...
    max(abs(rad2deg(qSeamStart-qPre))));

fprintf('\n=== 5. geometry: is qPreWeld a 30 mm wire-axis retract? ===\n');
T1 = getTransform(scene.robot,qSeamStart,scene.tcpBodyName);
Tp = getTransform(scene.robot,qPre,scene.tcpBodyName);
fprintf('FK(qPath(1,:)) position : [%s]\n',sprintf(' %.6f',T1(1:3,4)));
fprintf('FK(qPreWeld)   position : [%s]\n',sprintf(' %.6f',Tp(1:3,4)));
fprintf('offset                  : [%s]  |d| = %.6f m\n', ...
    sprintf(' %.6f',Tp(1:3,4)-T1(1:3,4)),norm(Tp(1:3,4)-T1(1:3,4)));
c = (trace(Tp(1:3,1:3).'*T1(1:3,1:3))-1)/2;
fprintf('orientation difference  : %.4f deg\n',rad2deg(acos(max(-1,min(1,c)))));

fprintf('\n=== 6. anchors are self-consistent and validated ===\n');
if isfile(anchorSourceFile)
    a = load(anchorSourceFile,'result');
    ar = a.result;
    fprintf('created              : %s\n',ar.created);
    fprintf('sourceWeldResultFile : %s\n',ar.sourceWeldResultFile);
    fprintf('fullPass             : %d\n',ar.fullPass);
    fprintf('weldValidation       : pass=%d nodeCount=%d edgeCount=%d maxJointJumpDeg=%.4f\n', ...
        ar.weldValidation.pass,ar.weldValidation.nodeCount,ar.weldValidation.edgeCount, ...
        ar.weldValidation.maxJointJumpDeg);
    fprintf('preWeldValidation    : pass=%d maxJointJumpDeg=%.4f\n', ...
        ar.preWeldValidation.pass,ar.preWeldValidation.maxJointJumpDeg);
    fprintf('qHome (= qOffset)    : [%s]\n',sprintf(' %.6f',ar.qHome));
    fprintf('max|qWeld(1)-preWeld.q| = %.3f deg\n', ...
        max(abs(rad2deg(localWrap(ar.qWeld(1,:)-ar.preWeld.q)))));
else
    fprintf('SKIP: %s not found\n',anchorSourceFile);
end

fprintf('\n=== 7. note on the two different Homes ===\n');
fprintf('MATLAB scene.qMat0 (= qOffset) is the CoppeliaSim zero mapped to ROS.\n');
fprintf('The HMI real_home in ur5_hmi/config/hmi_config.yaml is a DIFFERENT physical pose.\n');
fprintf('Do not use 轨迹文件/realhome_to_preweld_v1_report.json q_start_rad as RealHome.\n');
fprintf('\nDiagnostics complete; nothing was written or commanded.\n');
end

function x = localWrap(x)
x = atan2(sin(x),cos(x));
end
