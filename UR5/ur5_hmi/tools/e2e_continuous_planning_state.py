"""Actual HMI button/QProcess/MATLAB regression, with ROS worker disabled.

Only the recorded REAL readiness snapshot is replayed (no live servo claim).
No algorithm, MATLAB output, state transition or trajectory loader is mocked.
"""
from __future__ import annotations
import json
import os
from pathlib import Path
import sys
import time
import warnings
from unittest.mock import patch
import yaml

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_continuous_state_mpl")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication, QFileDialog
from gui.main_window import CONFIG, MainWindow
from gui.planning_workflow import PlanningStage
from trajectory.trajectory_loader import load_trajectory


def run(prior_request: Path, output: Path, expect_pass: bool) -> None:
    warnings.filterwarnings("ignore", message="Glyph.*missing from current font")
    request = json.loads(prior_request.read_text())
    raw = Path(request["rawFile"])
    app = QApplication.instance() or QApplication([])
    before = CONFIG.read_bytes()
    with patch.object(QThread, "start", lambda self: None):
        window = MainWindow()
    try:
        window.resize(1920, 1080)
        window.calibration_source_combo.blockSignals(True)
        window.calibration_source_combo.setCurrentText("REAL CALIBRATION")
        window.calibration_source_combo.blockSignals(False)
        window.tabs.setCurrentIndex(window.online_tab_index)
        # Replay the historical Home snapshot in a SEPARATE test YAML. The
        # user has since saved another Home; never rewrite production config
        # or remove version checks to make an old preview appear current.
        source = load_trajectory(str(raw))
        fixture = yaml.safe_load(before)
        q_home = list(map(float, source.metadata["real_home_q"]))
        fixture["real_home"] = q_home
        fixture["real_home_metadata"] = dict(q_rad=q_home,
            realhome_version=source.metadata["realhome_version"],
            collision_scene_version=source.metadata["collision_scene_version"],
            calibration_binding=request["current_binding"], source="RECORDED_OFFLINE_TEST_SNAPSHOT")
        test_config = output.with_name(output.stem + "_historical_config.yaml")
        test_config.write_text(yaml.safe_dump(fixture, allow_unicode=True))
        window.realhome_config_path = test_config
        window._refresh_online_goal_display()
        window._update_online_start_display()
        window.planning_workflow.raw_path = raw
        window._adopt_trajectory_file(raw, auto_validate=False)
        # Explicit recorded-scene replay, not a live safety/servo validation.
        window._real_planning_readiness_reasons = lambda: request["execution_gate_reasons"]
        live = list(window.state.positions)
        window._update_buttons()
        start = time.perf_counter()
        window.online_continuous_button.click()
        assert window.planning_busy and window.planning_process is not None, window.online_status_label.text()
        assert window.planning_workflow.stage == PlanningStage.CONTINUOUS_PLANNING
        print("ACTUAL_BUTTON_MATLAB_STARTED", str(window.planning_report_path), flush=True)
        while window.planning_busy:
            app.processEvents()
            if time.perf_counter()-start > 1500:
                raise TimeoutError("Continuous planning did not finish within 1500s")
            time.sleep(.02)
        app.processEvents()
        report = json.loads(window.planning_report_path.read_text())
        expected = (PlanningStage.PHYSICAL_VALIDATION_PENDING if expect_pass
                    else PlanningStage.CONTINUOUS_PLANNING_FAILED)
        assert window.planning_workflow.stage == expected, window.online_status_label.text()
        assert report["success"] is expect_pass
        assert report["continuous_planning_complete"] is expect_pass
        assert report["executionReady"] is False
        assert window.planning_workflow.ready_path is None
        assert not window._final_execution_gate_ok() and not window.execute_button.isEnabled()
        if expect_pass:
            assert window.loaded_trajectory.metadata["preview_only"]
            assert not window._scalar_bool(window.loaded_trajectory.metadata["executionReady"], True)
            assert "连续规划完成（离线验证通过）" in window.online_status_label.text()
            assert window.online_report_button.isEnabled() and window.online_save_analysis_button.isEnabled()
            window._set_preview_pose(window.loaded_trajectory.point_count//2)
            bundle = output.with_name("offline_analysis_saved_by_button.zip")
            with patch.object(QFileDialog, "getSaveFileName", return_value=(str(bundle), "")):
                window.online_save_analysis_button.click()
            assert bundle.is_file()
        else:
            assert "通过" not in window.online_stage_label.text()
            assert report["failure_reason"] in window.online_status_label.text()
            assert not window.online_report_button.isEnabled()
            assert window.loaded_trajectory.source_path == str(raw), "FAIL must preserve the raw preview"
        assert window.state.positions == live and CONFIG.read_bytes() == before
        run_dir = window.planning_report_path.parent
        assert not list(run_dir.glob("trajectory_execute_ready.*"))
        window.showNormal(); window.resize(1920, 1080)
        app.processEvents()
        window.grab().save(str(output.with_suffix(".png")))
        result = dict(test_pass=True, expected_offline_pass=expect_pass,
            scope="Actual HMI QPushButton -> QProcess MATLAB -> callback/state/load. ROS worker disabled; recorded readiness/Home replayed in isolated test YAML, not current live certification.",
            report=str(window.planning_report_path), stage=window.planning_workflow.stage.value,
            status=window.online_status_label.text(), execution_status=window.online_execution_preparation_label.text(),
            elapsed_sec=time.perf_counter()-start, live_state_preserved=True, config_unchanged=True,
            executionReady=False, execute_button_enabled=False, execution_permit_files=[], robot_motion="NO")
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2))
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    finally:
        window.close()
        app.processEvents()


if __name__ == "__main__":
    run(Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3] == "pass")
