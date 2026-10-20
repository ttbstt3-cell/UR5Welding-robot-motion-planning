"""Offline FK/VTK regression. No ROS worker or robot commands.

Run after audit_hmi_preview_display. --vtk uses a local X display for images.
"""
from pathlib import Path
import argparse
import json
import os
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
parser = argparse.ArgumentParser()
parser.add_argument("--vtk", action="store_true")
args = parser.parse_args()
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_workflow_mpl")
from trajectory.trajectory_loader import load_trajectory
from gui.preview_display import PreviewDisplayMixin
from gui.robot_view import UrdfKinematicModel
from gui.planning_workflow import PlanningWorkflow, PlanningStage
out = ROOT.parent / "规划文件/workflow_runs/preview_display_audit_20261014"
path = out / "full_raw_trajectory.mat"
t = load_trajectory(str(path))
audit = json.loads((out / "preview_display_audit.json").read_text())
class Probe(PreviewDisplayMixin):
    pass
p = Probe(); p.model = UrdfKinematicModel(); p.set_preview_trajectory(t)
assert p._preview_scene_snapshot is not None
maxima = {k: 0.0 for k in ("flangePosePath", "gunPosePath", "wirePosePath")}
for i, q in enumerate(t.positions):
    p._preview_q = q; p._preview_tcp_index = i
    d = p.preview_frame_diagnostics()
    for k in maxima:
        maxima[k] = max(maxima[k], d[k+"_position_error_mm"])
assert max(maxima.values()) < 0.001
state = PlanningWorkflow(); state.preview_complete(path, audit)
assert state.stage == PlanningStage.PREVIEW_WARNING
assert not audit["collision_coverage_pass"]
assert "forearm_link" in audit["missing_robot_collision_bodies"]
result = {"all_frame_fk_maximum_position_error_mm": maxima,
          "robot_motion": "NO", "collision_coverage_pass": False,
          "preview_state": state.stage.value}
if args.vtk:
    from PyQt5.QtWidgets import QApplication
    from gui.vtk_robot_view import VtkRobotView, _vtk_matrix
    import vtk
    app = QApplication([])
    v = VtkRobotView(); v._timer.stop(); v.resize(1100, 1000); v.show()
    v.set_preview_trajectory(t)
    live = np.deg2rad([10, -80, 20, -30, 40, 50]).tolist()
    v._live_q = live.copy()
    v.set_preview_state(t.positions[116], 116)
    d = v.preview_frame_diagnostics()
    def matrix(actor):
        m = actor.GetUserMatrix()
        return np.array([[m.GetElement(i,j) for j in range(4)] for i in range(4)])
    assert np.allclose(matrix(v._preview_gun_visual), d["gun_pose"], atol=1e-12)
    assert all(np.allclose(matrix(a), d["gun_pose"], atol=1e-12) for a in v._preview_gun_hulls)
    assert not v._gun_actor.GetVisibility()
    assert v._live_q == live
    result["frame117_reported_collision_pairs"] = d["collision_pairs"]
    previous = v._preview_tcp_fk.copy()
    different = np.eye(4); different[:3,3] = [.2,.1,.3]
    v.set_planning_tcp_transform(different)
    assert np.allclose(v._preview_tcp_fk, previous)
    # Visual surface crossings are diagnostic only, not planner collisions.
    visual_pairs = []
    gun_data = v._preview_gun_visual.GetMapper().GetInput()
    for name, (actor, _) in v._actors.items():
        f = vtk.vtkCollisionDetectionFilter()
        f.SetInputData(0, gun_data); f.SetMatrix(0, _vtk_matrix(d["gun_pose"]))
        f.SetInputData(1, actor.GetMapper().GetInput()); f.SetMatrix(1, actor.GetUserMatrix())
        f.SetCollisionModeToFirstContact(); f.Update()
        if f.GetNumberOfContacts():
            visual_pairs.append(name + " <-> WeldingGun_visual")
    result["frame117_visual_surface_pairs"] = visual_pairs
    result["surface_check_scope"] = "STL surface crossings only, not solid planning collision geometry"
    result["frame117"] = {k: value.tolist() for k,value in d.items() if isinstance(value, np.ndarray)}
    for view in ("iso", "front", "side"):
        v.fit_scene_view(view); app.processEvents(); v._refresh()
        image = vtk.vtkWindowToImageFilter(); image.SetInput(v.vtk_widget.GetRenderWindow()); image.Update()
        writer = vtk.vtkPNGWriter(); writer.SetFileName(str(out/f"frame117_{view}.png"))
        writer.SetInputConnection(image.GetOutputPort()); writer.Write()
    v.clear_preview_state(); v._refresh()
    assert v._gun_actor.GetVisibility()
    assert all(not a.GetVisibility() for a in v._preview_tool_actors)
    result["vtk_actor_test_pass"] = True
    v.close()
    # Integration screenshot and state test; the ROS worker never starts.
    from unittest.mock import patch
    from PyQt5.QtCore import QThread
    from gui.main_window import MainWindow
    with patch.object(QThread, "start", lambda self: None):
        win = MainWindow()
    win.showNormal(); win.resize(1920,1080)
    win.tabs.setCurrentIndex(win.online_tab_index)
    win._adopt_trajectory_file(path, auto_validate=False)
    win._set_preview_pose(116)
    assert win.planning_workflow.stage == PlanningStage.PREVIEW_WARNING
    assert "forearm_link" in win.online_preview_pose_label.text()
    assert win.online_result_labels["collision"].text() == "WARNING"
    assert not win._final_execution_gate_ok()
    app.processEvents(); win.robot_view.fit_scene_view("iso"); win.robot_view._refresh()
    app.processEvents()
    # Qt QWidget.grab does not reliably capture native VTK/X buffers. Read
    # that viewport directly and combine the two actual captured buffers.
    from PyQt5.QtGui import QImage, QPainter
    view_capture = vtk.vtkWindowToImageFilter()
    view_capture.SetInput(win.robot_view.vtk_widget.GetRenderWindow())
    view_capture.ReadFrontBufferOff(); view_capture.Update()
    writer = vtk.vtkPNGWriter(); writer.SetFileName(str(out/"hmi_vtk_viewport.png"))
    writer.SetInputConnection(view_capture.GetOutputPort()); writer.Write()
    shot = win.grab(); painter = QPainter(shot)
    viewport = win.robot_view.vtk_widget
    from PyQt5.QtCore import QRect
    painter.drawImage(QRect(viewport.mapTo(win, viewport.rect().topLeft()), viewport.size()),
                      QImage(str(out/"hmi_vtk_viewport.png")))
    painter.end(); shot.save(str(out/"hmi_frame117_warning.png"))
    result["hmi_warning_integration_pass"] = True
    win.close()
(out/"display_chain_test.json").write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
