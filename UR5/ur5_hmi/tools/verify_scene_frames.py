"""Verify the two independent frame transforms required by the HMI viewer.

  workpiece : CoppeliaSim UR5_base  ->  VTK base   p = Rz(-90) p + [0,0,dz]
  tcpPath   : MATLAB base_link      ->  VTK base   p = Rz(180) p

Both must land on the same physical geometry, so the seam derived from the
workpiece boxes and the planned TCP path must agree.
"""
import json
import math
from pathlib import Path

import numpy as np

UR = Path("/home/ttbstt/jixiebi/UR5")
DZ = 0.014649842865765095


def rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quat_to_R(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def to_base_from_ur5base(p):
    return (rz(-90.0) @ np.asarray(p, float)) + np.array([0.0, 0.0, DZ])


def to_base_from_base_link(p):
    return (rz(180.0) @ np.asarray(p, float))


geo = json.loads((UR / "scene_geometry.json").read_text())
wp = geo["workpiece"]
wp_p_R_ur5base = np.asarray(wp["pose_base"]["position"], float)
wp_R = quat_to_R(wp["pose_base"]["quaternion"])
assert np.allclose(wp_R, np.eye(3)), "Workpiece pose in UR5_base is expected to be identity rotation"

proxies = {p["name"]: p for p in wp["collision_proxies"]}
print("proxies:", list(proxies))
for name, p in proxies.items():
    print(f"  {name:34s} size={np.round(p['size_m'],5).tolist()} pos_wp={np.round(p['pose_workpiece']['position'],5).tolist()}")

# --- seam A = bottom x wall_y1, matching replan_weld_actual_axis_adaptive_core.localInferSeam
bottom = proxies["WorkpieceCollision_bottom"]
wall_x0 = proxies["WorkpieceCollision_wall_x0"]
wall_x1 = proxies["WorkpieceCollision_wall_x1"]
wall_y1 = proxies["WorkpieceCollision_wall_y1"]


def pose_np(proxy):
    pp = proxy["pose_workpiece"]
    return np.asarray(pp["position"], float), quat_to_R(pp["quaternion"])


b_p, b_R = pose_np(bottom)
x0_p, x0_R = pose_np(wall_x0)
x1_p, x1_R = pose_np(wall_x1)
y1_p, y1_R = pose_np(wall_y1)
bs = np.asarray(bottom["size_m"], float) / 2.0
y1s = np.asarray(wall_y1["size_m"], float) / 2.0

bottom_min = b_p - bs
bottom_max = b_p + bs
xMin = max(bottom_min[0], x0_p[0] + np.asarray(wall_x0["size_m"], float)[0] / 2)
xMax = min(bottom_max[0], x1_p[0] - np.asarray(wall_x1["size_m"], float)[0] / 2)
ySeam = y1_p[1] - y1s[1]
zSeam = bottom_max[2]
MARGIN = 0.025
print(f"\nseam A in Workpiece coords: x=[{xMin+MARGIN:.4f},{xMax-MARGIN:.4f}] y={ySeam:.4f} z={zSeam:.4f}")

# Workpiece -> UR5_base is pure translation
def wp_to_ur5base(p):
    return np.asarray(p, float) + wp_p_R_ur5base


seam_wp = np.array([[xMin + MARGIN, ySeam, zSeam], [xMax - MARGIN, ySeam, zSeam]])
seam_ur5base = np.array([wp_to_ur5base(p) for p in seam_wp])
seam_base = np.array([to_base_from_ur5base(p) for p in seam_ur5base])
print("seam endpoints in UR5_base (m):", np.round(seam_ur5base, 6).tolist())
print("seam endpoints in VTK base  (m):", np.round(seam_base, 6).tolist())

# --- planned TCP path from the validated headless run
from scipy.io import loadmat

mat = loadmat(UR / "planning_runs" / "hmi_weld_Weld_Seam" / "trajectory.mat", squeeze_me=True, struct_as_record=False)
tcp_bl = np.asarray(mat["tcpPath"], float)          # MATLAB base_link
q = np.asarray(mat["qPath"], float)
print(f"\ntcpPath from trajectory.mat: {tcp_bl.shape}")

tcp_base = np.array([to_base_from_base_link(p) for p in tcp_bl])

# rows: 1 = qPreWeld, 2..25 = weld seam (24 exec points), 26 = qPostWeld
weld_tcp_base = tcp_base[1:25]
weld_tcp_bl = tcp_bl[1:25]
print("weld segment TCP in VTK base, first/last:", np.round(weld_tcp_base[0], 6).tolist(), np.round(weld_tcp_base[-1], 6).tolist())
print("weld segment TCP in base_link, first/last:", np.round(weld_tcp_bl[0], 6).tolist(), np.round(weld_tcp_bl[-1], 6).tolist())

# deviation of the planned weld TCP from the ideal seam line (in VTK base)
a = seam_base[0]
b = seam_base[1]
d = b - a
d = d / np.linalg.norm(d)
rel = weld_tcp_base - a
t = rel @ d
perp = np.linalg.norm(rel - np.outer(t, d), axis=1)
print(f"\nperpendicular deviation of planned weld TCP from ideal seam: max {perp.max()*1000:.4f} mm  mean {perp.mean()*1000:.4f} mm")
print(f"along-seam span: {t.max()-t.min():.6f} m ; ideal seam length {np.linalg.norm(d)*1:.6f} m")

# workpiece box corners in VTK base
print("\nworkpiece box centre in VTK base (m):")
for name, p in proxies.items():
    pp, RR = pose_np(p)
    c_ur5base = wp_to_ur5base(pp)
    print(f"  {name:34s} {np.round(to_base_from_ur5base(c_ur5base),6).tolist()}")

# Overall bbox of the workpiece in VTK base
corners = []
for name, p in proxies.items():
    pp, RR = pose_np(p)
    half = np.asarray(p["size_m"], float) / 2.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                local = pp + RR @ np.array([sx * half[0], sy * half[1], sz * half[2]])
                corners.append(to_base_from_ur5base(wp_to_ur5base(local)))
corners = np.array(corners)
print("\nworkpiece bbox in VTK base (m):")
print("  min:", np.round(corners.min(axis=0), 5).tolist())
print("  max:", np.round(corners.max(axis=0), 5).tolist())
print("  extent:", np.round(corners.max(axis=0) - corners.min(axis=0), 5).tolist())
print(f"\nseam midpoint in VTK base: {np.round(seam_base.mean(axis=0),6).tolist()}")
print(f"weld TCP midpoint in VTK base: {np.round(weld_tcp_base.mean(axis=0),6).tolist()}")