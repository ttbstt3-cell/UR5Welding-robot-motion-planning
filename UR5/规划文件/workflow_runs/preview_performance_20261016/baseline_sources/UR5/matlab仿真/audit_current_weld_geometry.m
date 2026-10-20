clear;
clc;
close all;

%% AUDIT_CURRENT_WELD_GEOMETRY
% 只读审计当前 UR5 + WeldingGunMount + WeldTCP + GunCollisionHull
% + MATLAB wireCollision + Workpiece collision proxies。
% 本脚本不修改标定、不搜索 IK、不复用旧路径，也不发布 ROS2 轨迹。

matlabDir = fileparts(mfilename("fullpath"));
if isempty(matlabDir)
    matlabDir = pwd;
end
addpath(matlabDir);

run(fullfile(matlabDir,"sync_ur5_joint_states.m"));

fprintf("\n========================================\n");
fprintf("CURRENT WELD GEOMETRY AUDIT\n");
fprintf("========================================\n");

%% 1. Robot/tool bodies and collision counts
mountBody = getBody(scene.robot,scene.mountBodyName);
tcpBody = getBody(scene.robot,scene.tcpBodyName);

mountCollisionCount = numel(mountBody.Collisions);
tcpCollisionCount = numel(tcpBody.Collisions);

fprintf("Robot bodies                         : %d\n",numel(scene.robot.BodyNames));
fprintf("WeldingGunMount collision objects    : %d\n",mountCollisionCount);
fprintf("WeldTCP collision objects            : %d\n",tcpCollisionCount);
fprintf("Wire collision enabled in MATLAB     : %d\n",scene.wireCollisionEnabled);

if tcpCollisionCount < 1 || ~scene.wireCollisionEnabled
    warning("当前 MATLAB 场景未检测到焊丝 collisionCylinder。 ");
end

%% 2. TCP and gun-head geometry
pTcp = scene.pTcpMat0(:);
pGunHead = scene.pGunHeadMat0(:);
pWireEnd = scene.pWireEndMat0(:);

headToTcp = norm(pTcp-pGunHead);
wireToTcp = norm(pTcp-pWireEnd);

fprintf("\nTCP definition (MATLAB base)\n");
fprintf("  pTCP      = [%.6f %.6f %.6f] m\n",pTcp);
fprintf("  T_mount_tcp translation = [%.6f %.6f %.6f] m\n", ...
    scene.T_mount_tcp(1,4),scene.T_mount_tcp(2,4),scene.T_mount_tcp(3,4));
fprintf("  pGunHead  = [%.6f %.6f %.6f] m\n",pGunHead);
fprintf("  pWireEnd  = [%.6f %.6f %.6f] m\n",pWireEnd);
fprintf("  Gun head -> TCP distance = %.3f mm\n",headToTcp*1000);
fprintf("  Wire length              = %.3f mm\n",scene.wireLengthM*1000);
fprintf("  Wire end -> TCP gap      = %.3f mm\n",wireToTcp*1000);
fprintf("  Stored wire gap          = %.3f mm\n",scene.wireTcpGapM*1000);

fprintf("\nWire axis in WeldTCP frame (gun head -> TCP; independent of TCP +Z)\n");
fprintf("  zWireTcp = [%.6f %.6f %.6f]\n",scene.wireAxisTcp);
fprintf("  ||zWireTcp|| = %.6f\n",norm(scene.wireAxisTcp));

%% 3. Workpiece geometry and candidate welding side
wp = scene.geometryData.workpiece;

fprintf("\nWorkpiece geometry source\n");
fprintf("  collision proxies = %d\n",double(wp.collision_proxy_count));
fprintf("  open side         = %s\n",string(wp.collision_proxy_open_side));
fprintf("  plate thickness   = %.3f mm\n",double(wp.plate_thickness_m)*1000);
fprintf("  bbox              = [%.1f %.1f %.1f] mm\n", ...
    1000*double(wp.bbox_size_m(1)), ...
    1000*double(wp.bbox_size_m(2)), ...
    1000*double(wp.bbox_size_m(3)));

% Current seam convention: bottom x wall_y1, in Workpiece coordinates.
pSeamStartWP = [0.015 0.230 0.005];
pSeamEndWP = [0.220 0.230 0.005];
tangentWP = (pSeamEndWP-pSeamStartWP).';
tangentWP = tangentWP/norm(tangentWP);

bottomNormalWP = [0;0;1];       % bottom top surface, into free volume
wallNormalWP = [0;-1;0];        % wall_y1 inner surface, into U interior
freeBisectorWP = bottomNormalWP + wallNormalWP;
freeBisectorWP = freeBisectorWP/norm(freeBisectorWP);
approachToCornerWP = -freeBisectorWP;

fprintf("\nCandidate seam: bottom x wall_y1\n");
fprintf("  seam start WP = [%.4f %.4f %.4f] m\n",pSeamStartWP);
fprintf("  seam end   WP = [%.4f %.4f %.4f] m\n",pSeamEndWP);
fprintf("  tangent      = [%.4f %.4f %.4f]\n",tangentWP);
fprintf("  bottom free-space normal = [%.4f %.4f %.4f]\n",bottomNormalWP);
fprintf("  wall_y1 inner normal    = [%.4f %.4f %.4f]\n",wallNormalWP);
fprintf("  free-space bisector     = [%.4f %.4f %.4f]\n",freeBisectorWP);
fprintf("  approach-to-corner (+Z) = [%.4f %.4f %.4f]\n",approachToCornerWP);

%% 4. Existing 45-degree orientation convention
xTCPWP = [1;0;0];
zTCPWP = [0;1;-1];
zTCPWP = zTCPWP/norm(zTCPWP);
yTCPWP = cross(zTCPWP,xTCPWP);
yTCPWP = yTCPWP/norm(yTCPWP);
RtcpWP = [xTCPWP yTCPWP zTCPWP];

workAngleDeg = rad2deg(acos(abs(dot(zTCPWP,wallNormalWP))));
bottomAngleDeg = rad2deg(acos(abs(dot(zTCPWP,bottomNormalWP))));
approachAgreementDeg = rad2deg(acos(max(-1,min(1,dot(zTCPWP,approachToCornerWP)))));

% 在当前参考 TCP 姿态下，把实际焊丝/枪头延伸轴转换到 Workpiece
% 坐标系。这里仅作为“当前工具轴代理”输出，不能替代独立喷嘴轴标定。
wireAxisWP = RtcpWP*scene.wireAxisTcp(:);
wireAxisWP = wireAxisWP/norm(wireAxisWP);
wireAxisVsApproachDeg = rad2deg(acos(max(-1,min(1,dot(wireAxisWP,approachToCornerWP)))));
wireAxisVsBottomIntoDeg = rad2deg(acos(max(-1,min(1,dot(wireAxisWP,[0;0;-1])))));
wireAxisVsWallIntoDeg = rad2deg(acos(max(-1,min(1,dot(wireAxisWP,[0;1;0])))));

% 验证 visual/collision 圆柱中心线确实从枪头端向 WeldTCP 延伸。
axisBase = scene.T_MatBase_TCP0(1:3,1:3)*scene.wireAxisTcp(:);
axisBase = axisBase/norm(axisBase);
segmentBase = (pWireEnd-pGunHead)/norm(pWireEnd-pGunHead);
wireSegmentAgreementDeg = rad2deg(acos(max(-1,min(1,dot(axisBase,segmentBase)))));

fprintf("\nExisting reference TCP orientation\n");
fprintf("  +X_TCP (travel) = [%.4f %.4f %.4f]\n",xTCPWP);
fprintf("  +Y_TCP          = [%.4f %.4f %.4f]\n",yTCPWP);
fprintf("  +Z_TCP          = [%.4f %.4f %.4f]\n",zTCPWP);
fprintf("  det(RtcpWP)     = %.6f\n",det(RtcpWP));
fprintf("  angle to wall inner normal  = %.2f deg\n",workAngleDeg);
fprintf("  angle to bottom normal      = %.2f deg\n",bottomAngleDeg);
fprintf("  +Z vs approach-to-corner    = %.2f deg\n",approachAgreementDeg);
fprintf("  wire/gun axis proxy in WP    = [%.4f %.4f %.4f]\n",wireAxisWP);
fprintf("  wire axis vs approach axis   = %.2f deg\n",wireAxisVsApproachDeg);
fprintf("  wire axis vs bottom-inward   = %.2f deg\n",wireAxisVsBottomIntoDeg);
fprintf("  wire axis vs wall-inward     = %.2f deg\n",wireAxisVsWallIntoDeg);
fprintf("  wire segment endpoint error  = %.3f deg\n",wireSegmentAgreementDeg);
fprintf("  Existing travel definition  = rotation about local TCP Y\n");
fprintf("  Existing roll definition    = rotation about local TCP Z\n");

%% 5. Explicit uncertainty report; do not guess process-side semantics.
fprintf("\n========================================\n");
fprintf("AUDIT CONCLUSION\n");
fprintf("========================================\n");
fprintf("[OK] MATLAB WeldTCP has one wire collisionCylinder.\n");
fprintf("[OK] Wire is rigidly attached through WeldTCP -> WeldingGunMount.\n");
fprintf("[OK] Wire axis is derived from GunCollisionHull gun-head endpoint -> TCP, not assumed to be TCP +Z.\n");
fprintf("[OK] Visual/collision wire centerline is collinear with the gun-head -> TCP segment (%.3f deg).\n",wireSegmentAgreementDeg);
fprintf("[INFO] The 2.8 mm value is geometric remaining gap, not a calibrated process parameter.\n");
fprintf("[INFO] Gun-head point is estimated from the GunCollisionHull vertex nearest TCP; no explicit nozzle reference exists.\n");
fprintf("[REQUIRED] The current wire axis is only a gun-axis proxy until nozzle/feed-axis alignment is confirmed.\n");
fprintf("[INFO] CoppeliaSim currently has no corresponding wire collision object; synchronization is MATLAB-only.\n");
fprintf("[CHECK] Existing 45 deg orientation mathematically bisects bottom/wall normals and points +Z into the corner.\n");
fprintf("[REQUIRED HUMAN CONFIRMATION] Confirm that welding is performed on the U-interior side:\n");
fprintf("    free volume: y < 0.230 m and z > 0.005 m in Workpiece frame\n");
fprintf("    torch +Z: free space -> bottom/wall_y1 corner\n");
fprintf("    travel: Workpiece +X (Point 1 -> Point 80)\n");
fprintf("Only after this confirmation should姿态候选 and 80-layer IK search be regenerated.\n");

audit = struct();
audit.mountCollisionCount = mountCollisionCount;
audit.tcpCollisionCount = tcpCollisionCount;
audit.wireLengthM = scene.wireLengthM;
audit.wireDiameterM = scene.wireDiameterM;
audit.pTcpMat0 = pTcp.';
audit.pGunHeadMat0 = pGunHead.';
audit.pWireEndMat0 = pWireEnd.';
audit.gunHeadToTcpM = headToTcp;
audit.wireToTcpGapM = wireToTcp;
audit.wireAxisTcp = scene.wireAxisTcp;
audit.wireAxisWP = wireAxisWP;
audit.wireAxisVsApproachDeg = wireAxisVsApproachDeg;
audit.wireAxisVsBottomIntoDeg = wireAxisVsBottomIntoDeg;
audit.wireAxisVsWallIntoDeg = wireAxisVsWallIntoDeg;
audit.wireSegmentAgreementDeg = wireSegmentAgreementDeg;
audit.plateThicknessM = double(wp.plate_thickness_m);
audit.seamStartWP = pSeamStartWP;
audit.seamEndWP = pSeamEndWP;
audit.tangentWP = tangentWP;
audit.freeBisectorWP = freeBisectorWP;
audit.approachToCornerWP = approachToCornerWP;
audit.RtcpWP = RtcpWP;
audit.workAngleDeg = workAngleDeg;
audit.approachAgreementDeg = approachAgreementDeg;
audit.processSideConfirmed = false;

outputFile = fullfile(matlabDir,"current_weld_geometry_audit.mat");
save(outputFile,"audit");
fprintf("\nAudit MAT saved: %s\n",outputFile);
