"""Click production preview button and await real MATLAB, ROS worker disabled."""
from __future__ import annotations
import json
import os
from pathlib import Path
import sys
import time
from unittest.mock import patch
import warnings

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_preview_performance_e2e_mpl")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication
from gui.main_window import CONFIG, MainWindow
from gui.planning_workflow import PlanningStage


def run(output_file: Path, seed: int = 20260917) -> None:
    warnings.filterwarnings("ignore", message="Glyph.*missing from current font")
    before = CONFIG.read_bytes()
    app = QApplication.instance() or QApplication([])
    with patch.object(QThread, "start", lambda self: None):
        window = MainWindow()
    try:
        window.calibration_source_combo.blockSignals(True)
        window.calibration_source_combo.setCurrentText("REAL CALIBRATION")
        window.calibration_source_combo.blockSignals(False)
        window.online_start_combo.setCurrentIndex(window.online_start_combo.findData("real_home"))
        window.online_task_combo.setCurrentIndex(window.online_task_combo.findData("Full_Welding_Task"))
        window.online_seed_spin.setValue(seed)
        window.online_budget_spin.setValue(30)
        window.tabs.setCurrentIndex(window.online_tab_index)
        live_before = list(window.state.positions)
        started = time.perf_counter()
        window.online_start_button.click()
        assert window.planning_busy and window.planning_process is not None
        print("ACTUAL_PREVIEW_BUTTON_STARTED", window.planning_report_path, flush=True)
        while window.planning_busy:
            app.processEvents()
            if time.perf_counter() - started > 900:
                raise TimeoutError("Production HMI preview did not finish in 900 seconds")
            time.sleep(0.02)
        app.processEvents()
        report = json.loads(window.planning_report_path.read_text())
        request = json.loads(window.planning_request_path.read_text())
        assert window.planning_workflow.stage == PlanningStage.PREVIEW_READY, window.online_status_label.text()
        assert report["previewReady"] and report["executionReady"] is False
        assert window.loaded_trajectory.metadata["preview_only"]
        assert not window._final_execution_gate_ok()
        window._set_preview_pose(window.loaded_trajectory.point_count // 2)
        assert window.state.positions == live_before
        assert CONFIG.read_bytes() == before
        result = {"pass": True, "scope": "Actual QPushButton -> QProcess MATLAB -> HMI report/load/preview; ROS worker disabled",
                  "report": str(window.planning_report_path), "request": request,
                  "gui_stage": window.planning_workflow.stage.value,
                  "matlab_planning_sec": report["planning_time"],
                  "button_to_preview_sec": time.perf_counter() - started,
                  "transfer": json.loads(window.planning_report_path.with_name("hmi_transfer_performance.json").read_text()),
                  "node_count": window.loaded_trajectory.point_count, "live_state_preserved": True,
                  "config_unchanged": True, "executionReady": False, "robot_motion": "NO"}
        output_file.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        window.close()
        app.processEvents()


if __name__ == "__main__":
    run(Path(sys.argv[1]), int(sys.argv[2]) if len(sys.argv) > 2 else 20260917)
