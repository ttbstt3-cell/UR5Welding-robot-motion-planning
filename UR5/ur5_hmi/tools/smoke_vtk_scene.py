"""VTK-specific smoke test: verifies the real embedded 3D widget.

Runs under a real X11 display (QT_QPA_PLATFORM must not be ``offscreen``) so the
QVTK widget is constructed instead of the matplotlib fallback.  It only builds
rendering objects; it does not publish, subscribe or command anything.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.pop("QT_QPA_PLATFORM", None)
os.environ.setdefault("DISPLAY", ":0")

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np
from PyQt5.QtWidgets import QApplication

failures: list[str] = []
checks = 0


def check(label, ok, detail=""):
    global checks
    checks += 1
    print(f"[{'PASS' if ok else 'FAIL'}] {label}" + (f"  ({detail})" if detail else ""))
    if not ok:
        failures.append(label)


app = QApplication.instance() or QApplication(sys.argv)

from gui.vtk_robot_view import VtkRobotView  # noqa: E402
from gui.scene_geometry import base_link_to_base  # noqa: E402
from trajectory.trajectory_loader import load_trajectory  # noqa: E402

view = VtkRobotView()
print("backend: VTK (QVTKRenderWindowInteractor)")
print()

check("scene geometry loaded", "unavailable" not in view.scene_status(), view.scene_status())
check("4 workpiece actors", len(view._workpiece_actors) == 4, str(len(view._workpiece_actors)))
check("3 seam actors", len(view._seam_actors) == 3, ",".join(sorted(view._seam_actors)))
check("6 visibility keys", sorted(view.visibility()) == [
    "collision", "current_tcp", "robot", "seam", "tcp_path", "workpiece"])

# Each workpiece actor must span its true, rotated extent.
spans = {}
for actor in view._workpiece_actors:
    b = actor.GetBounds()
    spans[tuple(np.round(np.asarray(b), 5).tolist())] = True
check("actors have distinct bounds", len(spans) == 4, f"{len(spans)} distinct")

# Visibility switches must actually change VTK visibility.
view.set_visibility(workpiece=False)
hidden = all(not a.GetVisibility() for a in view._workpiece_actors)
view.set_visibility(workpiece=True)
shown = all(a.GetVisibility() for a in view._workpiece_actors)
check("workpiece toggle works", hidden and shown)
view.set_visibility(seam=False)
seam_hidden = all(not a.GetVisibility() for a in view._seam_actors.values())
view.set_visibility(seam=True)
check("seam toggle works", seam_hidden)

# Planned TCP path.
traj = load_trajectory("/home/ttbstt/jixiebi/UR5/planning_runs/hmi_weld_Weld_Seam/trajectory.mat")
view.set_planned_tcp_path(np.asarray(traj.tcp_path, float), frame="base_link")
check("planned TCP actor created", view._planned_tcp_actor is not None)
check("planned path stored in base frame",
      view._planned_tcp_points is not None
      and np.allclose(view._planned_tcp_points, base_link_to_base(traj.tcp_path)))
check("TCP path hidden until requested",
      not view._planned_tcp_actor.GetVisibility())
view.set_visibility(tcp_path=True)
check("TCP path toggle works", view._planned_tcp_actor.GetVisibility())

# Slider correspondence.
mismatch = 0
for idx in (0, 5, 13, 25):
    view.set_preview_state(traj.positions[idx].tolist(), idx)
    got = view.preview_point_base(idx)
    if got is None or np.abs(np.asarray(got) - base_link_to_base(traj.tcp_path[idx: idx + 1])[0]).max() > 1e-12:
        mismatch += 1
check("preview index -> planned TCP sample", mismatch == 0, f"{mismatch} mismatches")

# Framing with and without the workpiece.
view.fit_robot_view("iso")
cam_robot = np.array(view.renderer.GetActiveCamera().GetFocalPoint())
view.fit_scene_view("iso")
cam_scene = np.array(view.renderer.GetActiveCamera().GetFocalPoint())
check("combined framing shifts the focal point",
      not np.allclose(cam_robot, cam_scene),
      f"robot {cam_robot.round(3).tolist()} vs scene {cam_scene.round(3).tolist()}")

for name in ("iso", "top", "front", "side", "reset"):
    view.set_view_with_scene(name)
check("all camera presets accepted", True)

view.clear_planned_tcp_path()
check("planned TCP path cleared", view._planned_tcp_actor is None)

print()
print(f"RESULT: {checks - len(failures)}/{checks} checks passed")
if failures:
    for item in failures:
        print("  FAILED:", item)
else:
    print("ALL VTK CHECKS PASSED (rendering only; no ROS2, no robot motion)")
sys.stdout.flush()
sys.stderr.flush()
os._exit(1 if failures else 0)