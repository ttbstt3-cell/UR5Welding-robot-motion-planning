"""Actual HMI planning-button request; no ROS worker and no MATLAB launch."""
from pathlib import Path
from unittest.mock import patch
import hashlib
import json
import os
import sys
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_fixed_support_mpl")
project = Path("/home/ttbstt/jixiebi/UR5")
sys.path.insert(0, str(project / "ur5_hmi"))
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication
from gui.main_window import CONFIG, MainWindow
from gui.planning_workflow import PlanningStage
before = CONFIG.read_bytes()
app = QApplication.instance() or QApplication([])
with patch.object(QThread, "start", lambda self: None):
    win = MainWindow()
win.calibration_source_combo.blockSignals(True)
win.calibration_source_combo.setCurrentText("REAL CALIBRATION")
win.calibration_source_combo.blockSignals(False)
for i in range(win.online_start_combo.count()):
    if win.online_start_combo.itemData(i) != "current":
        win.online_start_combo.setCurrentIndex(i)
        break
for i in range(win.online_task_combo.count()):
    if win.online_task_combo.itemData(i) == "Full_Welding_Task":
        win.online_task_combo.setCurrentIndex(i)
        break
captured = {}
win._launch_workflow_process = lambda phase, entry, payload: captured.update(
    phase=phase, entry=entry, payload=payload)
win._start_preview_planning()
assert captured.get("entry") == "plan_hmi_preview_request", captured
out = Path(__file__).resolve().parent / "cold_full_preview_v2"
out.mkdir(exist_ok=True)
request = captured["payload"]
request.update(outputDir=str(out), request_id="FIXED_SUPPORT_CURRENT_HMI_COLD_OFFLINE")
assert request["task"] == "Full_Welding_Task"
assert request["calibration_binding"]["fixed_environment_contacts_sha256"] == hashlib.sha256(
    (project / "fixed_environment_contacts.json").read_bytes()).hexdigest()
(out / "request.json").write_text(json.dumps(request, indent=2))
if "--check-result" in sys.argv:
    live_before = list(win.state.positions)
    win.workflow_phase = "preview"
    win.workflow_canceled = False
    win.planning_process = None
    win.planning_report_path = out / "report.json"
    win.planning_output_path = out / "full_raw_trajectory.mat"
    win._online_process_finished(0, 0)
    assert win.planning_workflow.stage == PlanningStage.PREVIEW_WARNING, win.online_status_label.text()
    assert win.loaded_trajectory.point_count == 173
    assert win.loaded_trajectory.metadata["preview_only"]
    assert not win._final_execution_gate_ok()
    win._set_preview_pose(65)
    assert win.state.positions == live_before
    assert "WorkpieceCollision" in win.online_status_label.text()
    from trajectory.trajectory_loader import load_trajectory
    old = load_trajectory(str(project / '规划文件/workflow_runs/preview_20261014_190649_439402/full_raw_trajectory.mat'))
    stale_reasons = win._trajectory_binding_reasons(old)
    assert any("碰撞场景" in reason for reason in stale_reasons)
    test = {"pass": True, "stage": win.planning_workflow.stage.value,
            "warning": win.online_status_label.text(), "execution_gate_open": False,
            "live_state_unchanged": True, "old_trajectory_stale_reasons": stale_reasons,
            "robot_motion": "NO"}
    (out.parent / "hmi_completion_test.json").write_text(json.dumps(test, indent=2, ensure_ascii=False))
win.close()
app.processEvents()
assert before == CONFIG.read_bytes(), "Production Home configuration changed"
print(json.dumps({"request_file": str(out / "request.json"), "ros_started": False,
                  "robot_motion": "NO", "production_config_unchanged": True}))
