function [RtcpWP, info] = make_actual_gun_axis_pose(scene, tangentWP, gunAxisWP, travelDeg, rollDeg)
%MAKE_ACTUAL_GUN_AXIS_POSE Build a TCP orientation using the measured wire axis.
%
% The wire centreline stored in scene.wireAxisTcp is treated as the physical
% gun/work axis.  It is deliberately not replaced by TCP +Z.  The returned
% rotation maps that tool-frame axis to gunAxisWP, while the TCP X direction
% follows the seam tangent as closely as possible.  travel changes the gun
% axis about the seam tangent and roll rotates the non-axisymmetric tool about
% the actual wire axis.

arguments
    scene
    tangentWP (3,1) double
    gunAxisWP (3,1) double
    travelDeg (1,1) double
    rollDeg (1,1) double
end

t = tangentWP / norm(tangentWP);
g0 = gunAxisWP / norm(gunAxisWP);
a = double(scene.wireAxisTcp(:));
a = a / norm(a);

% Use the measured wire-cylinder X axis as a deterministic tool-frame
% reference.  Re-orthogonalise it so numerical calibration noise cannot
% change the handedness of the generated frame.
rTool = double(scene.T_tcp_wire(1:3,1));
rTool = rTool - a*dot(a,rTool);
if norm(rTool) < 1e-9
    error("scene.T_tcp_wire 的参考轴与焊丝轴退化，无法定义工具姿态。");
end
rTool = rTool / norm(rTool);
sTool = cross(a,rTool);
sTool = sTool / norm(sTool);

% Travel angle is a rotation of the physical gun axis around the seam
% tangent.  The sign convention is explicit: positive travel rotates g0
% according to the right-hand rule about +t.
g = localRodrigues(t, deg2rad(travelDeg)) * g0;
g = g / norm(g);

% Project the seam tangent into the plane normal to the gun axis.  This is
% the zero-roll reference direction, so TCP X remains seam-aligned whenever
% geometry permits it.
r0 = t - g*dot(g,t);
if norm(r0) < 1e-9
    % A 90-degree travel angle would make the tangent parallel to the gun
    % axis.  Keep a stable fallback using the tool reference instead.
    r0 = rTool - g*dot(g,rTool);
end
r0 = r0 / norm(r0);

% Roll is about the actual physical gun axis, not TCP +Z.
r = localRodrigues(g, deg2rad(rollDeg)) * r0;
r = r / norm(r);
s = cross(g,r);
s = s / norm(s);

% [r s g] is the desired TCP frame in Workpiece coordinates and
% [rTool sTool a] is the calibrated tool frame in TCP coordinates.
RtcpWP = [r s g] * [rTool sTool a].';

if det(RtcpWP) < 0
    % This should never occur for a right-handed input, but fail loudly if
    % a malformed imported frame slips through.
    error("生成的 TCP 姿态不是右手旋转矩阵。");
end

info = struct();
info.tangentWP = t;
info.gunAxisWP = g;
info.gunAxisTcp = a;
info.travelDeg = travelDeg;
info.rollDeg = rollDeg;
info.rToolTcp = rTool;
info.sToolTcp = sTool;
info.RtcpWP = RtcpWP;
info.workAngleDeg = rad2deg(acos(max(-1,min(1,dot(g,[0;0;-1])))));
end

function R = localRodrigues(axis, angle)
axis = axis / norm(axis);
x = axis(1); y = axis(2); z = axis(3);
c = cos(angle); s = sin(angle); v = 1-c;
R = [x*x*v+c,   x*y*v-z*s, x*z*v+y*s; ...
     y*x*v+z*s, y*y*v+c,   y*z*v-x*s; ...
     z*x*v-y*s, z*y*v+x*s, z*z*v+c];
end
