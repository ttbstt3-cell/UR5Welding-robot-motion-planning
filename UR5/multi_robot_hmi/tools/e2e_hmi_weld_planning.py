"""End-to-end test of the HMI online-planning path.

Drives the real ``_start_online_planning()`` slot, which spawns MATLAB through
QProcess, streams the stage markers into the GUI, then auto-loads
trajectory.mat + report.json into the trajectory validator and 3D preview.

This DOES start MATLAB (the wrapper is the thing under test) but it never
starts ROS2, never sends a FollowJointTrajectory goal, never publishes a ROS2
message and never uses STOP MOTION.  Execution is not triggered: the Final
Execution Check must remain blocked.

Usage:
    QT_QPA_PLATFORM=offscreen python3 tools/e2e_hmi_weld_planning.py Weld_Seam
    QT_QPA_PLATFORM=offscreen python3 tools/e2e_hmi_weld_planning.py Full_Welding_Task
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np
from PyQt5.QtWidgets import QApplication

TASK = sys.argv[1] if len(sys.argv) > 1 else "Weld_Seam"
TIMEOUT_S = float(sys.argv[2]) if len(sys.argv) > 2 else 400.0

app = QApplication.instance() or QApplication(sys.argv)

from gui.main_window import MainWindow  # noqa: E402
from gui.scene_geometry import base_link_to_base  # noqa: E402

failures: list[str] = []


def check(label, ok, detail=""):
    print(f"[{'PASS' if ok else 'FAIL'}] {label}" + (f"  ({detail})" if detail else ""))
    if not ok:
        failures.append(label)
    sys.stdout.flush()


win = MainWindow()
win.online_start_combo.setCurrentIndex(win.online_start_combo.findData("real_home"))
win.online_task_combo.setCurrentIndex(
    win.online_task_combo.findData(TASK)
)
win.online_seed_spin.setValue(20260917)
win.online_budget_spin.setValue(30)

print(f"=== E2E task={TASK} ===")
print(f"start source: {win.online_start_combo.currentData()}")
print(f"weld algorithm label: {win.online_weld_algorithm_label.text()}")
check("weld algorithm fixed to ACCP", "ACCP" in win.online_weld_algorithm_label.text())
check("start snapshot available", "RealHome" in win.online_start_snapshot_label.text(),
      win.online_start_snapshot_label.text())

stages_seen: list[str] = []
original_parse = win._parse_online_stage_line


def spy(line):
    original_parse(line)
    text = win.online_stage_label.text()
    if text and (not stages_seen or stages_seen[-1] != text):
        stages_seen.append(text)


win._parse_online_stage_line = spy

t0 = time.time()
win._start_online_planning()
check("planning process started", win.planning_process is not None)
check("GUI responsive during planning (planning_busy)", win.planning_busy)

loop_done = False
while time.time() - t0 < TIMEOUT_S:
    app.processEvents()
    if win.planning_process is None and not win.planning_busy:
        loop_done = True
        break
    time.sleep(0.1)

elapsed = time.time() - t0
check("MATLAB process finished", loop_done, f"{elapsed:.1f} s")
if not loop_done:
    win._cancel_online_planning()
    for _ in range(50):
        app.processEvents()
        time.sleep(0.1)

print(f"\nstages observed: {stages_seen}\n")
expected_stage_tokens = {
    "Weld_Seam": ["PLANNING WELD", "VALIDATING", "EXPORTING"],
    "Full_Welding_Task": ["PLANNING APPROACH", "PLANNING WELD", "PLANNING EXIT", "VALIDATING", "EXPORTING"],
    "RealHome_to_PreWeld": ["PLANNING APPROACH", "VALIDATING", "EXPORTING"],
    "PostWeld_to_RealHome": ["PLANNING EXIT", "VALIDATING", "EXPORTING"],
}[TASK]
joined = " | ".join(stages_seen)
for token in expected_stage_tokens:
    check(f"stage seen: {token}", token in joined)
check("final stage SUCCESS", "SUCCESS" in win.online_stage_label.text(),
      win.online_stage_label.text())

print("\n--- result labels ---")
for key, label in win.online_result_labels.items():
    print(f"  {key:26s} {label.text().replace(chr(10), ' / ')}")

report_path = win.planning_report_path
mat_path = win.planning_output_path
check("report.json produced", report_path is not None and report_path.is_file(), str(report_path))
check("trajectory.mat produced", mat_path is not None and mat_path.is_file(), str(mat_path))

payload = json.loads(report_path.read_text(encoding="utf-8")) if report_path and report_path.is_file() else {}
check("report task matches", payload.get("task") == TASK, str(payload.get("task")))
check("report collision PASS", payload.get("collision_check_pass") is True)
check("report continuity PASS", payload.get("continuity_check_pass") is True)
check("report seam uses ACCP",
      "ACCP" in str(payload.get("seam_planner", "")) or payload.get("seam_point_count") in (0, 16),
      str(payload.get("seam_planner")))

print("\n--- auto-load ---")
traj = win.loaded_trajectory
check("trajectory auto-loaded", traj is not None,
      f"{traj.point_count if traj else 0} points")
check("validation ran", win.trajectory_validation is not None)
if win.trajectory_validation is not None:
    rep = win.trajectory_validation
    check("validation PASS", bool(getattr(rep, "ok", False)),
          f"ok={getattr(rep, 'ok', None)} issues={list(getattr(rep, 'issues', []))[:3]}")
check("time parameterized", traj is not None and traj.time_from_start is not None)
check("tcpPath present in MAT", traj is not None and traj.tcp_path is not None
      and len(traj.tcp_path) == traj.point_count)
check("collision metadata present",
      traj is not None and "collision_check_pass" in (traj.metadata or {}),
      str(sorted((traj.metadata or {}).keys())[:8]))

print("\n--- planned TCP path in the 3D view ---")
check("view has planned TCP points", win.robot_view.preview_point_base(0) is not None)
if traj is not None and traj.tcp_path is not None:
    shown = np.asarray(win.robot_view.preview_point_base(0), float)
    expect = base_link_to_base(np.asarray(traj.tcp_path, float)[0:1])[0]
    check("TCP path transformed into base frame", np.abs(shown - expect).max() < 1e-12,
          f"max {np.abs(shown-expect).max():.3e} m")
    # Seam alignment is checked on the WELD BLOCK only.  A Full task also
    # contains free-space approach and exit rows that are far from the seam.
    if payload.get("seam_point_count"):
        from gui.scene_geometry import load_scene_geometry
        seam = load_scene_geometry().seam("A")
        pts = base_link_to_base(np.asarray(traj.tcp_path, float))
        counts = payload.get("stage_point_counts") or {}
        if TASK == "Weld_Seam":
            block = pts[1:-1]
        else:
            n_ap = int(counts.get("approach", 0))
            block = pts[n_ap:n_ap + int(counts.get("weld", 0))]
        a, b = seam.polyline_base[0], seam.polyline_base[-1]
        d = b - a
        d /= np.linalg.norm(d)
        rel = block - a
        t = rel @ d
        perp = np.linalg.norm(rel - np.outer(t, d), axis=1)
        # The block includes the 30 mm PreWeld/PostWeld retraction, so allow it.
        check("weld block TCP stays near Seam A", perp.max() < 0.031,
              f"{len(block)} rows, max deviation {perp.max()*1000:.2f} mm")
        check("weld block spans the seam",
              abs((t.max() - t.min()) - 0.175) < 2e-3,
              f"span {(t.max()-t.min())*1000:.2f} mm")

print("\n--- preview ---")
win._start_preview()
for _ in range(40):
    app.processEvents()
    time.sleep(0.02)
win._stop_preview()          # stop the timer so sampling is not raced
app.processEvents()
mismatch = 0
for idx in range(0, traj.point_count, max(1, traj.point_count // 6)):
    win._set_preview_pose(idx)
    app.processEvents()
    q = np.asarray(getattr(win.robot_view, "q", []), float)
    if q.shape != (6,) or np.abs(q - traj.positions[idx]).max() > 1e-9:
        mismatch += 1
        # remember which index failed for the report
        print(f"      mismatch at index {idx}: "
              f"view={q.round(4).tolist()} traj={traj.positions[idx].round(4).tolist()}")
check("preview drives robot pose at every sampled index", mismatch == 0, f"{mismatch} bad")
win._stop_preview()

print("\n--- Final Execution Check (must stay blocked) ---")
gate = win.final_gate_details_label.text()
check("execute button disabled", not win.execute_button.isEnabled())
check("no motion", not win._motion_busy())
check("no action goal active", not win.goal_active)
print(f"  gate: {gate[:160]}")

print("\n--- archive ---")
archive_dir = Path(win.config.get("online_planning", {}).get("planning_file_dir", ""))
copies = sorted(archive_dir.glob(f"online_{TASK.lower()}*")) if archive_dir.is_dir() else []
check("archived to planning folder", len(copies) >= 3, f"{len(copies)} files")
for item in copies[-3:]:
    print(f"    {item.name}")

print(f"\nRESULT: {'ALL PASSED' if not failures else 'FAILURES'} ({len(failures)} failed)")
for item in failures:
    print("  FAILED:", item)
print("no ROS2 message, no FollowJointTrajectory, no robot motion was issued")
sys.stdout.flush()
os._exit(1 if failures else 0)
