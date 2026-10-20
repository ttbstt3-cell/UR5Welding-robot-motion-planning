"""Capture the actual HMI planning-button request without ROS or motion."""
from pathlib import Path
from unittest.mock import patch
import hashlib
import json
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_environment_request_mpl")
PROJECT = Path("/home/ttbstt/jixiebi/UR5")
sys.path.insert(0, str(PROJECT / "ur5_hmi"))
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication
from gui.main_window import CONFIG, MainWindow
from gui.planning_workflow import PlanningStage

out = Path(__file__).resolve().parent / "current_online_replan"
out.mkdir(exist_ok=True)
protected = [CONFIG, PROJECT / "real_tcp_calibration_applied.json",
             PROJECT / "real_tcp_calibration_active.json",
             PROJECT / "real_gun_geometry/real_gun_geometry.json",
             PROJECT / "real_gun_mount_calibration.json",
             PROJECT / "workpiece_pose_real.json", PROJECT / "scene_geometry.json",
             PROJECT / "ur5_tool_calibration.mat", PROJECT / "ur5.ttt",
             PROJECT / "real_robot_description_calibrated.urdf",
             PROJECT / "matlab仿真/replan_weld_actual_axis_adaptive_core.m"]
before = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in protected if p.is_file()}
app = QApplication.instance() or QApplication([])
with patch.object(QThread, "start", lambda self: None):
    win = MainWindow()
win.calibration_source_combo.blockSignals(True)
win.calibration_source_combo.setCurrentText("REAL CALIBRATION")
win.calibration_source_combo.blockSignals(False)
win.online_start_combo.setCurrentIndex(win.online_start_combo.findData("realhome"))
# Do not select or modify a new endpoint: use the latest Saved RealHome.
if win.online_start_combo.currentData() != "realhome":
    for i in range(win.online_start_combo.count()):
        if win.online_start_combo.itemData(i) != "current":
            win.online_start_combo.setCurrentIndex(i)
            break
captured = {}
win._launch_workflow_process = lambda phase, entry, payload: captured.update(
    phase=phase, entry=entry, payload=payload)
win._real_planning_readiness_reasons = lambda: (_ for _ in ()).throw(
    AssertionError("preview invoked execution readiness"))
win._load_weld_anchors = lambda: (_ for _ in ()).throw(
    AssertionError("preview read formal anchors"))
win._start_preview_planning()
assert captured.get("entry") == "plan_hmi_preview_request", captured
request = captured["payload"]
request.update(outputDir=str(out), request_id="ENVIRONMENT_CURRENT_HMI_OFFLINE")
assert request["calibration_binding"]["environment_geometry_sha256"] == hashlib.sha256(
    (PROJECT / "environment_geometry_real.json").read_bytes()).hexdigest()
(out / "request.json").write_text(json.dumps(request, indent=2), encoding="utf-8")
if (out / "report.json").is_file():
    # Exercise the same completion handler that the real QProcess uses.
    win.workflow_phase = "preview"
    win.workflow_canceled = False
    win.planning_process = None
    win.planning_report_path = out / "report.json"
    win.planning_output_path = out / "full_raw_trajectory.mat"
    win._online_process_finished(0, 0)
    report = json.loads((out / "report.json").read_text())
    assert win.planning_workflow.stage == PlanningStage.PREVIEW_FAILED
    assert "EnvironmentCollision_Floor" in win.online_status_label.text()
    assert report["rrt_entered"] is False
    assert report["executionReady"] is False
    captured["result"] = {"status": win.planning_workflow.stage.value,
                          "reason": win.online_status_label.text(),
                          "start": win.online_start_state_label.text(),
                          "goal": win.online_goal_state_label.text()}
win.close()
app.processEvents()
after = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in protected if p.is_file()}
assert before == after, "GUI test unexpectedly changed protected production files"
summary = {"pass": True, "entry": captured["entry"], "request": request,
           "protected_files_unchanged": True, "protected_sha256": after,
           "ros_worker_started": False, "robot_motion": "NO"}
if "result" in captured:
    summary["hmi_result"] = captured["result"]
(out / "hmi_request_test.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))
print(json.dumps({"pass": True, "request_file": str(out / "request.json"),
                  "environment_sha256": request["calibration_binding"]["environment_geometry_sha256"],
                  "hmi_result": summary.get("hmi_result")}, ensure_ascii=False))
