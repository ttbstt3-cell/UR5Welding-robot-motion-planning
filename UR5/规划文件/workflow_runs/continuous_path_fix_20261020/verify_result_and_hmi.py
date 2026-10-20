"""Verify genuine cold-start results and replay completion in the real HMI.

ROS worker and TCP reads disabled; result publication isolated from user index.
"""
import hashlib
import json
import os
from pathlib import Path
import sys
from unittest.mock import patch
import warnings

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_pathfix_mpl")
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "ur5_hmi"))
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication
from gui.main_window import CONFIG, MainWindow
from gui.planning_workflow import PlanningStage


def no_tcp_reads(self):
    return None


def read(path):
    return json.loads(path.read_text())


def run():
    warnings.filterwarnings("ignore", message="Glyph.*missing from current font")
    before_launch = read(HERE / "before_launch.json")
    after_launch = read(HERE / "after_launch.json")
    failed = read(Path(before_launch["report"]))
    report = read(Path(after_launch["report"]))
    req = read(Path(after_launch["request"]))
    assert not failed["success"] and failed["failure_identifier"] == "MATLAB:UndefinedFunction"
    assert failed["failed_stage"] == "GEOMETRIC_CONSISTENCY"
    assert report["success"] and report["final_acceptance_pass"] and not report["executionReady"]
    assert report["stage"] == "PHYSICAL_VALIDATION_PENDING"
    assert all(report["offline_checks"].values())
    assert report["controller_validation"]["sample_count"] == 63701
    assert report["dependency_paths"]["environment_collision"] == str(ROOT.parent / "RRT/checkEnvironmentCollision.m")
    before = CONFIG.read_bytes()
    index = ROOT / "规划文件/当前轨迹/最新轨迹索引.json"
    index_before = index.read_bytes()
    app = QApplication.instance() or QApplication([])
    with patch.object(QThread, "start", lambda self: None), \
         patch.object(MainWindow, "_auto_read_robot_tcp", no_tcp_reads), \
         patch.object(MainWindow, "showMaximized", lambda self: None):
        win = MainWindow()
    win.robot_tcp_auto_timer.stop()
    sends = []
    win.worker.request_trajectory.connect(sends.append)
    try:
        win.trajectory_results_directory = HERE / "ISOLATED_CURRENT_TRAJECTORIES"
        win._real_planning_readiness_reasons = lambda: req["execution_gate_reasons"]
        win.planning_workflow.raw_path = Path(req["rawFile"])
        win.workflow_phase = "prepare"
        win.workflow_canceled = False
        win.planning_process = None
        win.planning_report_path = Path(before_launch["report"])
        win.planning_output_path = Path(before_launch["outputDir"]) / "offline_optimized_trajectory.mat"
        win._online_process_finished(0, 0)
        assert win.planning_workflow.stage == PlanningStage.CONTINUOUS_PLANNING_FAILED
        win.planning_workflow.begin_preparation()
        win.planning_report_path = Path(after_launch["report"])
        win.planning_output_path = Path(report["artifacts"]["trajectory_mat"])
        win._online_process_finished(0, 0)
        # A newly constructed test window defaults to SIMULATION. Confirm
        # that a real result is rejected there, then match the user's REAL UI.
        assert win.planning_workflow.stage == PlanningStage.CONTINUOUS_PLANNING_FAILED
        assert "Version mismatch" in win.online_status_label.text()
        win.calibration_source_combo.blockSignals(True)
        win.calibration_source_combo.setCurrentText("REAL CALIBRATION")
        win.calibration_source_combo.blockSignals(False)
        win.planning_workflow.begin_preparation()
        win._online_process_finished(0, 0)
        assert win.planning_workflow.stage == PlanningStage.PHYSICAL_VALIDATION_PENDING, (
            win.planning_workflow.stage, win.online_status_label.text(), win.planning_workflow.reason)
        assert win.loaded_trajectory.point_count == 638
        assert win.online_report_button.isEnabled() and win.online_save_analysis_button.isEnabled()
        assert not win.execute_button.isEnabled() and not win._final_execution_gate_ok()
        assert not sends and CONFIG.read_bytes() == before and index.read_bytes() == index_before
    finally:
        win.close()
        app.processEvents()
    protected = read(HERE / "protected_before.json")
    current = {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in protected}
    changed = [p for p in protected if current[p] != protected[p]]
    assert changed == [str(ROOT / "matlab仿真/prepare_hmi_continuous_request.m")], changed
    (HERE / "protected_verification.json").write_text(json.dumps({
        "only_expected_entry_changed": True, "changed": changed, "before": protected, "after": current},
        ensure_ascii=False, indent=2) + "\n")
    result = dict(before_failure_reproduced=True, actual_hmi_launch_command=True,
        full_current_request_pass=True, hmi_failure_and_success_replayed=True, wrong_scene_rejected=True,
        loaded_points=638, samples=report["controller_validation"]["sample_count"],
        planning_time=report["planning_time"], metrics=report["controller_validation"],
        production_changes=changed, motion_signals=0, user_config_and_index_unchanged=True,
        executionReady=False, robot_motion="NO")
    (HERE / "regression_results.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print({k: v for k, v in result.items() if k != "metrics"})


if __name__ == "__main__":
    run()
