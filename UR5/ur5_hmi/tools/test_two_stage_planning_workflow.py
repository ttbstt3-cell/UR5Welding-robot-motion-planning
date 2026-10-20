"""Offline GUI regression. ROS thread and MATLAB launch are replaced in tests."""
from __future__ import annotations
import json
import os
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_workflow_mpl")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from scipy.io import savemat
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication
from gui.main_window import MainWindow
from gui.planning_workflow import PlanningStage, PlanningWorkflow
from safety.joint_limits import JOINT_NAMES

app = QApplication.instance() or QApplication([])
with patch.object(QThread, "start", lambda self: None):
    win = MainWindow()  # Worker never starts: no ROS init, sockets or motion.
win.showNormal()
win.resize(1920, 1080)
win.tabs.setCurrentIndex(win.online_tab_index)
captured = {}

def launch(phase, entry, payload):
    captured.clear()
    captured.update(phase=phase, entry=entry, payload=payload)
    win.workflow_phase = phase
    win.workflow_canceled = False
    win.planning_busy = True

win._launch_workflow_process = launch
win._real_planning_readiness_reasons = lambda: (_ for _ in ()).throw(AssertionError("preview invoked real execution gate"))
win._load_weld_anchors = lambda: (_ for _ in ()).throw(AssertionError("preview loaded formal anchors"))
assert win.online_start_button.text() == "规划轨迹"
assert win.online_continuous_button.text() == "轨迹连续规划"
win._start_online_planning()
assert captured["entry"] == "plan_hmi_preview_request"
assert "execution_gate_reasons" not in captured["payload"]
assert "anchorFile" not in captured["payload"]
assert "seamCandidateFile" not in captured["payload"]
assert captured["payload"]["preview_only"] is True
print("PASS preview request starts without official anchors / execution gate")

with tempfile.TemporaryDirectory(prefix="ur5_workflow_test_") as tmp:
    temp = Path(tmp)
    q = np.tile(win._saved_real_home(), (5, 1))
    q[:, 0] += np.linspace(0, 0.002, 5)
    pose = np.empty(5, dtype=object)
    for i in range(5):
        pose[i] = np.eye(4)
        pose[i][:3, 3] = [0.1 + i*0.001, 0.2, 0.3]
    raw = temp / "full_raw_trajectory.mat"
    savemat(raw, {"qPath": q, "timeFromStart": np.arange(5.),
                 "tcpPath": np.array([p[:3, 3] for p in pose]), "tcpPosePath": pose,
                 "segmentLabel": np.array(["approach", "approach", "weld", "weld", "exit"], dtype=object),
                 "joint_names": np.array(JOINT_NAMES, dtype=object),
                 "metadata": {"preview_only": True, "executionReady": False, "workflow_stage": "PREVIEW", "tcp_frame": "base_link",
                              "realhome_version": captured["payload"]["realhome_version"],
                              "preweld_version": captured["payload"]["collision_scene_version"],
                              "collision_scene_version": captured["payload"]["collision_scene_version"]}})
    report = temp / "report.json"
    report.write_text(json.dumps({"previewReady": True, "executionReady": False,
                                 "task": "Full_Welding_Task", "algorithm": "UI fixture", "collision_check_pass": False,
                                 "success": True, "execution_point_count": 5, "duration": 4.0}))
    win.planning_output_path = raw
    win.planning_report_path = report
    live_before = list(win.state.positions)
    win._online_process_finished(0, 0)
    assert win.planning_workflow.stage == PlanningStage.PREVIEW_WARNING, win.online_status_label.text()
    assert win.loaded_trajectory.point_count == 5
    assert win.loaded_trajectory.metadata["preview_only"]
    assert not win._final_execution_gate_ok()
    win._set_preview_pose(3)
    assert win.state.positions == live_before
    assert "RPY" in win.online_preview_pose_label.text()
    assert "weld" in win.online_preview_pose_label.text()
    win._start_preview()
    assert win.preview_active
    win._pause_preview()
    print("PASS collision FAIL preview loads, animates, displays TCP pose/q; execution blocked")

    win._real_planning_readiness_reasons = lambda: ["test controller snapshot unavailable"]
    win._start_continuous_planning()
    assert captured["entry"] == "prepare_hmi_continuous_request"
    assert captured["payload"]["rawFile"] == str(raw)
    assert captured["payload"]["execution_gate_reasons"]
    assert captured["payload"]["minControllerSamples"] == 5000
    win.planning_report_path = report
    report.write_text(json.dumps({"executionReady": False, "failure_reason": "Boundary velocity discontinuity"}))
    win._online_process_finished(0, 0)
    assert win.planning_workflow.stage == PlanningStage.PREPARATION_FAILED
    assert win.planning_workflow.raw_path == raw
    assert win.loaded_trajectory.source_path == str(raw)
    assert "Boundary velocity discontinuity" in win.online_status_label.text()
    print("PASS execution preparation failure preserves usable preview and precise reason")

    state = PlanningWorkflow(raw_path=raw)
    try:
        state.preparation_complete(raw, {"executionReady": True})
    except ValueError:
        pass
    else:
        raise AssertionError("forged ready state accepted")
    try:
        state.preparation_complete(raw, {"executionReady": True,
            "controller_validation": {"complete": True, "collision_pass": True,
                                      "joint_limits_pass": True, "sample_count": 5000},
            "version_validation": {"pass": True}})
    except ValueError:
        pass
    else:
        raise AssertionError("offline workflow granted execution permission")
    assert state.ready_path is None
    print("PASS offline workflow cannot grant ready state even for numerical PASS")
    output = ROOT.parent / "规划文件" / "workflow_runs" / "gui_regression_20261014"
    output.mkdir(parents=True, exist_ok=True)
    app.processEvents()
    win.grab().save(str(output / "hmi_two_stage_preview.png"))
    print("GUI screenshot:", output / "hmi_two_stage_preview.png")
    # An actual MATLAB preview request, captured without starting a ROS worker.
    win.calibration_source_combo.blockSignals(True)
    win.calibration_source_combo.setCurrentText("REAL CALIBRATION")
    win.calibration_source_combo.blockSignals(False)
    binding = win._current_calibration_binding()
    offline_request = {"task": "Full_Welding_Task", "qStart": win._saved_real_home().tolist(),
        "qHome": win._saved_real_home().tolist(), "algorithm": "V1 FastConnectAnytimeRRTStar",
        "seed": 20260917, "maxPlanningTimeSec": 30.0, "planning_scene": "REAL_CALIBRATION",
        "calibration_binding": binding, "preview_only": True, "executionReady": False,
        "realhome_version": win._saved_real_home_record()["realhome_version"],
        "collision_scene_version": binding["collision_scene_version"], "preweld_version": None,
        "regenerate_preweld": True,
        "outputDir": str(output.parent / "real_preview_regression_20261014")}
    (output / "real_preview_request.json").write_text(json.dumps(offline_request, indent=2))
    actual_dir = output.parent / "real_preview_final_20261014"
    actual_raw = actual_dir / "full_raw_trajectory.mat"
    if actual_raw.is_file():
        prep = {"rawFile": str(actual_raw), "raw_sha256": win._file_sha256(actual_raw),
            "current_binding": binding,
            "execution_gate_reasons": ["OFFLINE_TEST_NO_FRESH_CONTROLLER_SNAPSHOT"],
            "maxBoundaryRefinementDepth": 2, "maxBoundaryIKCalls": 100,
            "maxBoundaryTimeSec": 30.0, "minControllerSamples": 5000,
            "outputDir": str(output.parent / "real_prepare_final_20261014")}
        (output / "real_prepare_request.json").write_text(json.dumps(prep, indent=2))
        win.planning_workflow.begin_preview()
        win.workflow_phase = "preview"
        win.workflow_canceled = False
        win.planning_output_path = actual_raw
        win.planning_report_path = actual_dir / "report.json"
        win._online_process_finished(0, 0)
        assert win.planning_workflow.stage == PlanningStage.PREVIEW_WARNING  # legacy raw lacks display/pair audit
        assert win.loaded_trajectory.point_count == 171
        win._refresh_online_goal_display()
        wp = json.loads(win.calibration_file.read_text())
        T = np.eye(4)
        T[:3,:3] = wp["rotation_matrix"]
        T[:3,3] = wp["position"]
        win.robot_view.set_workpiece_calibration(T)
        win._set_preview_pose(75)
        app.processEvents()
        win.grab().save(str(output / "hmi_real_preview_ready.png"))
        assert win.state.positions == live_before
        print("PASS actual 171-node MATLAB preview auto-loads; live joint state unchanged")
        summary = {"gui_tests_pass": True, "robot_motion": "NO",
            "real_preview": json.loads((actual_dir / "report.json").read_text()),
            "real_preparation_report": str(output.parent / "real_prepare_final_20261014" / "report.json"),
            "note": "GUI fixture tests and actual MATLAB preview loading; preparation acceptance uses its own report."}
        prep_report = output.parent / "real_prepare_final_20261014" / "report.json"
        if prep_report.is_file():
            summary["real_preparation"] = json.loads(prep_report.read_text())
        (output / "regression_summary.json").write_text(json.dumps(summary, indent=2))
win.close()
app.processEvents()
print("TWO_STAGE_GUI_TEST_PASS (synthetic display fixture; NO ROS / MATLAB / ROBOT MOTION)")
