"""Capture the REAL HMI button's fresh MATLAB command without starting ROS.

The captured command can subsequently be executed offline by the test operator.
No extra RRT path is injected by this harness; current index is never published.
"""
import hashlib
import json
import os
from pathlib import Path
import shlex
import sys
from unittest.mock import patch
import warnings

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_launch_paths_mpl")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5.QtCore import QProcess, QThread
from PyQt5.QtWidgets import QApplication
from gui.main_window import CONFIG, MainWindow
from gui.planning_workflow import PlanningStage


def no_tcp_reads(self):
    return None


def run(failed_request, output):
    warnings.filterwarnings("ignore", message="Glyph.*missing from current font")
    original = json.loads(failed_request.read_text())
    before = CONFIG.read_bytes()
    index = CONFIG.parents[2] / "规划文件/当前轨迹/最新轨迹索引.json"
    index_before = index.read_bytes()
    captured = {}

    class CaptureProcess(QProcess):
        def start(self, program, arguments):
            captured.update(program=program, arguments=list(arguments))

    app = QApplication.instance() or QApplication([])
    with patch.object(QThread, "start", lambda self: None), \
         patch.object(MainWindow, "_auto_read_robot_tcp", no_tcp_reads), \
         patch.object(MainWindow, "showMaximized", lambda self: None):
        window = MainWindow()
    window.robot_tcp_auto_timer.stop()
    sends = []
    window.worker.request_trajectory.connect(sends.append)
    try:
        window.trajectory_results_directory = output.parent / "ISOLATED_CURRENT_TRAJECTORIES"
        window.calibration_source_combo.blockSignals(True)
        window.calibration_source_combo.setCurrentText("REAL CALIBRATION")
        window.calibration_source_combo.blockSignals(False)
        window._adopt_trajectory_file(Path(original["rawFile"]), auto_validate=False)
        window.planning_workflow.raw_path = Path(original["rawFile"])
        # Offline test supplies the original request's recorded readiness list,
        # not a live controller/physical validation or execution permission.
        window._real_planning_readiness_reasons = lambda: original["execution_gate_reasons"]
        window._update_buttons()
        assert window.online_continuous_button.isEnabled()
        with patch("gui.planning_workflow.QProcess", CaptureProcess):
            window.online_continuous_button.click()
        assert captured and window.planning_workflow.stage == PlanningStage.CONTINUOUS_PLANNING
        generated = json.loads(window.planning_request_path.read_text())
        for key in ("rawFile", "raw_sha256", "motion_limits_file", "motion_limits_sha256", "minControllerSamples"):
            assert generated[key] == original[key], key
        assert generated["current_binding"] == original["current_binding"]
        assert hashlib.sha256(Path(generated["rawFile"]).read_bytes()).hexdigest() == original["raw_sha256"]
        assert CONFIG.read_bytes() == before and index.read_bytes() == index_before
        assert not sends
        captured.update(request=str(window.planning_request_path), outputDir=generated["outputDir"],
            report=str(window.planning_report_path), real_button_clicked=True,
            cold_start_only_paths="HMI matlab仿真 and RRT/benchmark; no genpath test setup",
            original_request_unchanged=True, robot_motion="NO", motion_signals=0)
        captured["shell_command"] = shlex.join([captured["program"], *captured["arguments"],
            "-logfile", str(output.parent / (output.stem + ".log"))])
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(captured, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps(captured, ensure_ascii=False, indent=2))
    finally:
        window.planning_busy = window.continuity_busy = False
        window.planning_process = None
        window.close()
        app.processEvents()


if __name__ == "__main__":
    run(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve())
