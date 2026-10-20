"""Replay genuine MATLAB reports into a freshly loaded production HMI.

Complements real button/MATLAB end-to-end tests, especially display and
fail-closed mutations of audit evidence. No ROS worker or robot action.
"""
from __future__ import annotations
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch
import warnings

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_continuous_replay_mpl")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5.QtCore import QThread
from PyQt5.QtGui import QDesktopServices
from PyQt5.QtWidgets import QApplication
from gui.main_window import MainWindow
from gui.planning_workflow import PlanningStage


def run(e2e_path: Path, output: Path):
    warnings.filterwarnings("ignore", message="Glyph.*missing from current font")
    e2e = json.loads(e2e_path.read_text())
    report_path = Path(e2e["report"])
    request = json.loads(report_path.with_name("request.json").read_text())
    report = json.loads(report_path.read_text())
    app = QApplication.instance() or QApplication([])
    with patch.object(QThread, "start", lambda self: None):
        window = MainWindow()
    try:
        window.realhome_config_path = e2e_path.with_name(e2e_path.stem + "_historical_config.yaml")
        window.calibration_source_combo.blockSignals(True)
        window.calibration_source_combo.setCurrentText("REAL CALIBRATION")
        window.calibration_source_combo.blockSignals(False)
        window._refresh_online_goal_display()
        window._update_online_start_display()
        window._real_planning_readiness_reasons = lambda: request["execution_gate_reasons"]
        window.planning_workflow.raw_path = Path(request["rawFile"])
        window.workflow_phase = "prepare"; window.workflow_canceled = False
        window.planning_process = None
        window.planning_report_path = report_path
        window.planning_output_path = Path(report["artifacts"]["trajectory_mat"])
        window._online_process_finished(0, 0)
        assert window.planning_workflow.stage == PlanningStage.PHYSICAL_VALIDATION_PENDING
        for frame in (0, window.loaded_trajectory.point_count//2, window.loaded_trajectory.point_count-1):
            window._set_preview_pose(frame)
            assert window.planning_workflow.stage == PlanningStage.PHYSICAL_VALIDATION_PENDING
            d = window.robot_view.preview_frame_diagnostics()
            assert d["collision_status"] == "PASS", d
            assert not d["display_warning"], d
        with patch.object(QDesktopServices, "openUrl", return_value=True) as opened:
            window.online_report_button.click()
            assert opened.call_count == 1
            assert opened.call_args.args[0].toLocalFile().endswith("offline_analysis.html")
        assert "待验证" in window.online_execution_preparation_label.text()
        assert "离线 PASS" in window.online_result_labels["continuity"].text()
        assert not window.execute_button.isEnabled()
        window.showNormal(); window.resize(1920, 1080)
        window.tabs.setCurrentIndex(window.online_tab_index)
        app.processEvents()
        window.tabs.widget(window.online_tab_index).ensureWidgetVisible(window.online_save_analysis_button)
        app.processEvents(); window.grab().save(str(output.with_suffix(".png")))
        failures = []
        with tempfile.TemporaryDirectory() as temp:
            for name in ("sync", "orientation", "collision", "versions", "execution_claim"):
                bad = copy.deepcopy(report)
                bad["failure_reason"] = "TEST_REJECT_" + name
                if name == "sync": bad["controller_validation"]["max_tcp_time_sync_error_mm"] = .568271
                if name == "orientation": bad["controller_validation"]["max_tcp_orientation_error_deg"] = .548976
                if name == "collision": bad["controller_validation"]["collision_pass"] = False
                if name == "versions": bad["version_validation"]["pass"] = False
                if name == "execution_claim": bad["executionReady"] = True
                path = Path(temp)/"report.json"; path.write_text(json.dumps(bad))
                window.planning_report_path = path
                window._online_process_finished(0, 0)
                assert window.planning_workflow.stage == PlanningStage.CONTINUOUS_PLANNING_FAILED
                assert bad["failure_reason"] in window.online_status_label.text()
                assert not window.execute_button.isEnabled()
                assert not window.online_save_analysis_button.isEnabled()
                failures.append(name)
        output.write_text(json.dumps(dict(pass_all=True, actual_report=str(report_path),
            restart_load_pass=True, display_fk_and_collision_pass=True, report_button_pass=True,
            false_pass_rejected=failures, executionReady=False, robot_motion="NO"), indent=2))
        print("FRESH_HMI_REPLAY_PASS", failures)
    finally:
        window.close(); app.processEvents()


if __name__ == "__main__":
    run(Path(sys.argv[1]), Path(sys.argv[2]))
