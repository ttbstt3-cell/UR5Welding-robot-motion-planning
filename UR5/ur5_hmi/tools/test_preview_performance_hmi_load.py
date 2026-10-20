"""Measure production Qt completion/load path with ROS worker disabled."""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import time
from unittest.mock import patch
import warnings

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_preview_performance_mpl")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication
from gui.main_window import MainWindow, CONFIG
from gui.planning_workflow import PlanningStage


def run(directories: list[str]) -> None:
    warnings.filterwarnings("ignore", message="Glyph.*missing from current font")
    config_before = CONFIG.read_bytes()
    app = QApplication.instance() or QApplication([])
    results = []
    with patch.object(QThread, "start", lambda self: None):
        window = MainWindow()
    window.calibration_source_combo.blockSignals(True)
    window.calibration_source_combo.setCurrentText("REAL CALIBRATION")
    window.calibration_source_combo.blockSignals(False)
    try:
        for directory in directories:
            directory = Path(directory)
            before = list(window.state.positions)
            window.workflow_phase = "preview"
            window.workflow_canceled = False
            window.planning_workflow.begin_preview()
            window.planning_report_path = directory / "report.json"
            window.planning_output_path = directory / "full_raw_trajectory.mat"
            window.planning_process = None
            window.planning_busy = True
            started = time.perf_counter()
            window._online_process_finished(0, 0)
            app.processEvents()
            assert window.planning_workflow.stage == PlanningStage.PREVIEW_READY, window.online_status_label.text()
            assert window.loaded_trajectory.metadata["preview_only"]
            assert not window._final_execution_gate_ok()
            for i in (0, window.loaded_trajectory.point_count // 2, window.loaded_trajectory.point_count - 1):
                window._set_preview_pose(i)
                assert window.state.positions == before
            performance = json.loads((directory / "hmi_transfer_performance.json").read_text())
            results.append({"directory": str(directory), "pass": True,
                            "node_count": window.loaded_trajectory.point_count,
                            "completion_and_three_preview_frames_sec": time.perf_counter() - started,
                            "performance": performance, "live_state_preserved": True,
                            "executionReady": False, "robot_motion": "NO"})
    finally:
        window.close()
        app.processEvents()
    assert CONFIG.read_bytes() == config_before, "HMI load modified saved configuration"
    output = Path(directories[-1]).parent / "hmi_load_regression.json"
    output.write_text(json.dumps({"scope": "Production Qt completion/load/preview; offscreen renderer, ROS thread never started",
                                  "config_unchanged": True, "runs": results}, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    run(sys.argv[1:])
