"""Headless smoke test for the HMI online-planning wiring.

It builds the real MainWindow under the offscreen Qt platform, then exercises
the code paths that matter for this change without starting ROS2, MATLAB or
any robot motion:

  * the task selector and endpoint resolution for all four tasks
  * the online stage marker parser
  * report rendering for a real Full_Welding_Task report.json
  * the 3D view scene layers (workpiece boxes, seam, planned TCP path)
  * the slider -> robot pose + planned TCP correspondence
  * the final execution gate state after loading a trajectory

Nothing here publishes, subscribes, sends an action goal or calls
STOP MOTION.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np
from PyQt5.QtWidgets import QApplication

from gui.main_window import MainWindow
from gui.scene_geometry import base_link_to_base, load_scene_geometry

RUN_DIR = Path("/home/ttbstt/jixiebi/UR5/planning_runs/hmi_weld_Full_Welding_Task")
SEAM_RUN_DIR = Path("/home/ttbstt/jixiebi/UR5/planning_runs/hmi_weld_Weld_Seam")

failures: list[str] = []
checks = 0


def check(label: str, ok: bool, detail: str = "") -> None:
    global checks
    checks += 1
    status = "PASS" if ok else "FAIL"
    print(f"[{status}] {label}" + (f"  ({detail})" if detail else ""))
    if not ok:
        failures.append(label)


app = QApplication.instance() or QApplication(sys.argv)
win = MainWindow()

print("=" * 74)
print("1) task selector + endpoint resolution")
print("=" * 74)
task_index = {win.online_task_combo.itemData(i): i for i in range(win.online_task_combo.count())}
check("4 tasks present", len(task_index) == 4, ", ".join(sorted(task_index)))
# Use Saved RealHome as the start source: with no live /joint_states the
# "current" source is correctly refused by the stale-feedback gate.
win.online_start_combo.setCurrentIndex(win.online_start_combo.findData("real_home"))
for task in ("RealHome_to_PreWeld", "Weld_Seam", "PostWeld_to_RealHome", "Full_Welding_Task"):
    win.online_task_combo.setCurrentIndex(task_index[task])
    try:
        q0, q1, text = win._task_endpoints(task)
        ok = (
            np.asarray(q0).shape == (6,) and np.asarray(q1).shape == (6,)
            and np.isfinite(q0).all() and np.isfinite(q1).all()
        )
        check(f"endpoints resolve: {task}", ok, text)
    except Exception as exc:
        check(f"endpoints resolve: {task}", False, str(exc))

print()
print("=" * 74)
print("2) weld seam algorithm is fixed to A-ARC-IK+")
print("=" * 74)
for task in ("Weld_Seam", "Full_Welding_Task"):
    win.online_task_combo.setCurrentIndex(task_index[task])
    label = win.online_weld_algorithm_label.text()
    check(f"{task}: weld algorithm label", "A-ARC-IK+" in label, label)
free_algos = [win.online_algorithm_combo.itemData(i) for i in range(win.online_algorithm_combo.count())]
check(
    "free-space algorithms preserved",
    free_algos == ["RRT", "RRT-Connect", "Bi-RRT*", "V1 FastConnectAnytimeRRTStar"],
    ", ".join(free_algos),
)

print()
print("=" * 74)
print("3) stage marker parsing")
print("=" * 74)
for line, expect in [
    ("HMI_WELD_STAGE stage=PREPARING", "PREPARING"),
    ("HMI_WELD_STAGE stage=APPROACH", "PLANNING APPROACH"),
    ("HMI_WELD_STAGE stage=WELD", "PLANNING WELD"),
    ("HMI_WELD_STAGE stage=EXIT", "PLANNING EXIT"),
    ("HMI_WELD_STAGE stage=VALIDATING", "VALIDATING"),
    ("HMI_WELD_STAGE stage=EXPORTING", "EXPORTING"),
]:
    win._parse_online_stage_line(line)
    got = win.online_stage_label.text()
    check(f"stage {expect}", expect in got, got)

print()
print("=" * 74)
print("4) report rendering from a real Full_Welding_Task run")
print("=" * 74)
report = RUN_DIR / "report.json"
check("report.json exists", report.is_file(), str(report))
if report.is_file():
    payload = win._show_online_result(report)
    labels = win.online_result_labels
    for key in ("task", "algorithm", "planning_time", "path_points", "duration",
                "collision", "continuity", "tcp_error", "max_joint_jump",
                "seam_initial_points", "seam_final_points", "seam_exec_points",
                "seam_inserted_points", "seam_direction", "seam_result",
                "stage_approach_points", "stage_weld_points", "stage_exit_points",
                "stage_total_points"):
        value = labels[key].text().replace("\n", " | ")
        check(f"label {key}", value not in ("", "—"), value)
    check("task reported", payload.get("task") == "Full_Welding_Task", str(payload.get("task")))
    check("collision PASS", payload.get("collision_check_pass") is True)
    check("continuity PASS", payload.get("continuity_check_pass") is True)
    check("total points == 51", int(labels["stage_total_points"].text()) == 51,
          labels["stage_total_points"].text())

print()
print("=" * 74)
print("5) 3D scene layers")
print("=" * 74)
view = win.robot_view
status = view.scene_status()
check("scene geometry loaded", "unavailable" not in status, status)
is_vtk = hasattr(view, "_workpiece_actors")
if is_vtk:
    check("4 workpiece actors", len(view._workpiece_actors) == 4,
          str(len(view._workpiece_actors)))
    check("seam A/B/C actors", len(view._seam_actors) == 3,
          ",".join(sorted(view._seam_actors)))
else:
    print("  (offscreen matplotlib fallback in use; VTK actors not present)")
    check("fallback scene geometry loaded", view._scene is not None)
check("scene visibility switches", all(key in getattr(win, "scene_visibility_boxes", {})
      for key in ("robot", "workpiece", "seam", "tcp_path", "current_tcp", "preview_tcp", "gun", "wire", "collision")),
      ",".join(sorted(getattr(win, "scene_visibility_boxes", {}))))

# planned TCP path from the Weld_Seam run
from trajectory.trajectory_loader import load_trajectory

traj = load_trajectory(str(SEAM_RUN_DIR / "trajectory.mat"))
check("tcpPath loaded", traj.tcp_path is not None and len(traj.tcp_path) == traj.point_count,
      f"{traj.point_count} points")

print()
print("=" * 74)
print("6) workpiece vs seam vs planned TCP alignment (base frame)")
print("=" * 74)
scene = load_scene_geometry()
boxes = scene.workpiece_boxes()
seam_a = scene.seam("A")
tcp = np.asarray(traj.tcp_path, dtype=float)
tcp_base = base_link_to_base(tcp)
weld_segment = tcp_base[1:25]
a, b = seam_a.polyline_base[0], seam_a.polyline_base[-1]
d = b - a
d = d / np.linalg.norm(d)
rel = weld_segment - a
t = rel @ d
perp = np.linalg.norm(rel - np.outer(t, d), axis=1)
check("planned TCP on Seam A", perp.max() < 1e-3, f"max deviation {perp.max()*1000:.6f} mm")
check("seam span == 175 mm", abs((t.max() - t.min()) - 0.175) < 1e-6,
      f"{(t.max()-t.min())*1000:.3f} mm")
lo, hi = scene.workpiece_bbox_base()
# The four plates are axis-aligned in Workpiece coordinates, but the Workpiece
# frame is rotated by Rz(-90 deg) relative to base, so the base-frame bounding
# box is a permutation of 235 mm, not a cube.
expected = sorted([0.235, 0.235, 0.235])
check("workpiece bbox is a permuted 235 mm cube",
      sorted(np.round(hi - lo, 6).tolist()) == expected,
      f"extent {(hi - lo).round(4).tolist()} lo={lo.round(4).tolist()} hi={hi.round(4).tolist()}")
# The seam must sit on the bottom plate top face, 5 mm above its base.
bottom = next(b for b in boxes if b.name == "WorkpieceCollision_bottom")
b_lo = bottom.corners_base().min(axis=0)
b_hi = bottom.corners_base().max(axis=0)
check("seam z == bottom plate top face", abs(a[2] - b_hi[2]) < 1e-9,
      f"seam z={a[2]:.4f} bottom top z={b_hi[2]:.4f} bottom thickness={(b_hi[2]-b_lo[2])*1000:.1f} mm")
# The seam plane must be the wall_y1 inner face (Workpiece y -> base +x).
wall_y1 = next(b for b in boxes if b.name == "WorkpieceCollision_wall_y1")
w_corners = wall_y1.corners_base()
check("seam on wall_y1 inner face", abs(a[0] - w_corners[:, 0].min()) < 1e-9,
      f"seam x={a[0]:.4f} wall_y1 inner x={w_corners[:,0].min():.4f}")
check("seam inside workpiece bbox",
      bool(lo[0] <= a[0] <= hi[0] and lo[1] <= a[1] <= hi[1] and lo[2] <= a[2] <= hi[2]))

print()
print("=" * 74)
print("7) auto-load + preview correspondence")
print("=" * 74)
win._adopt_trajectory_file(SEAM_RUN_DIR / "trajectory.mat", auto_validate=True)
win._apply_planned_tcp_path()
check("trajectory loaded", win.loaded_trajectory is not None
      and win.loaded_trajectory.point_count == traj.point_count,
      f"{win.loaded_trajectory.point_count if win.loaded_trajectory else 0} points")
mismatch = 0
max_tcp_err = 0.0
for idx in (0, 1, 6, 12, 18, 24, 25):
    win._set_preview_pose(idx)
    shown = getattr(win.robot_view, "tcp", None)
    expected = base_link_to_base(traj.tcp_path[idx: idx + 1])[0]
    if shown is None or len(shown) != 3:
        mismatch += 1
        continue
    err = float(np.linalg.norm(np.asarray(shown, float) - expected)) * 1000.0
    max_tcp_err = max(max_tcp_err, err)
    q_shown = np.asarray(getattr(win.robot_view, "q", []), float)
    q_expected = np.asarray(traj.positions[idx], float)
    if q_shown.shape != q_expected.shape or float(np.abs(q_shown - q_expected).max()) > 1e-9:
        mismatch += 1
check("slider -> robot pose + TCP correspond", mismatch == 0, f"{mismatch} mismatches")
check("preview TCP matches planned sample", max_tcp_err < 1e-6, f"max {max_tcp_err:.9f} mm")

print()
print("=" * 74)
print("8) final execution gate (must still require the real checks)")
print("=" * 74)
gate_text = win.final_gate_details_label.text()
print(f"  gate: {gate_text}")
print(f"  execute button enabled: {win.execute_button.isEnabled()}")
check("execute blocked after auto-load", not win.execute_button.isEnabled(),
      gate_text)
check("motion not busy", not win._motion_busy())
check("no goal active", not win.goal_active)
check("planning process idle", win.planning_process is None)

print()
print("=" * 74)
print(f"RESULT: {checks - len(failures)}/{checks} checks passed")
if failures:
    print("FAILED:")
    for item in failures:
        print("  -", item)
else:
    print("ALL CHECKS PASSED  (no ROS2, no MATLAB, no robot motion)")
print("=" * 74)
# Hard-exit: QApplication teardown with live QTimers aborts on this Qt build.
sys.stdout.flush()
sys.stderr.flush()
os._exit(1 if failures else 0)
