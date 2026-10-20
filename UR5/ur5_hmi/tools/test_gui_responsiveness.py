"""Real Qt buttons/process callbacks under 50 Hz synthetic state traffic.

Replay an existing genuine MATLAB result, not a new planning run. No ROS worker,
RTDE helper, successful execution confirmation or robot send is started. Use
--vtk for the production VTK view in a non-activating X11 test window.
"""
import json
import os
from pathlib import Path
import shutil
import sys
import threading
import time
from unittest.mock import patch
import warnings
import yaml

os.environ.setdefault("QT_QPA_PLATFORM", "xcb" if "--vtk" in sys.argv else "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_responsiveness_mpl")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from PyQt5.QtCore import QObject, QProcess, QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import QApplication
from gui.main_window import CONFIG, MainWindow
from gui.planning_workflow import PlanningStage, PlanningWorkflowPanel
from ros.state_monitor import RobotState


class StateSource(QObject):
    state = pyqtSignal(object)


def no_robot_tcp_reads(self):
    # Keep the function alive for the lifetime of the QTimer.singleShot bound
    # callback captured during widget construction (PyQt uses weak receivers).
    return None


def run(prepared, output):
    warnings.filterwarnings("ignore", message="Glyph.*missing from current font")
    output.mkdir(parents=True, exist_ok=True)
    config_before = CONFIG.read_bytes()
    source_report = prepared / "report.json"
    source_bytes = source_report.read_bytes()
    request = json.loads((prepared / "request.json").read_text())
    raw = Path(request["rawFile"])
    app = QApplication.instance() or QApplication([])
    traffic = StateSource()
    with patch.object(QThread, "start", lambda self: None), \
         patch.object(MainWindow, "_auto_read_robot_tcp", no_robot_tcp_reads), \
         patch.object(MainWindow, "showMaximized", lambda self: None):
        window = MainWindow()
    window.robot_tcp_auto_timer.stop()
    window.trajectory_results_directory = output / "CURRENT_TRAJECTORIES_TEST_ONLY"
    if "--vtk" in sys.argv:
        # QVTK requires a mapped X11 surface, not WA_DontShowOnScreen. Hidden
        # native render windows can segfault in this system VTK/GLX build.
        window.setAttribute(Qt.WA_ShowWithoutActivating)
        window.setWindowFlag(Qt.WindowDoesNotAcceptFocus)
        window.setWindowTitle("UR5 HMI UI TEST — ROS/RTDE disabled, NO ROBOT MOTION")
    else:
        window.setAttribute(Qt.WA_DontShowOnScreen)
    window.showNormal()
    window.resize(1920, 1080)
    window.tabs.setCurrentIndex(window.online_tab_index)
    sends = []
    window.worker.request_trajectory.connect(sends.append)
    traffic.state.connect(window._on_state)
    stop = threading.Event()
    count = [0]
    window.calibration_source_combo.blockSignals(True)
    window.calibration_source_combo.setCurrentText("REAL CALIBRATION")
    window.calibration_source_combo.blockSignals(False)
    window._adopt_trajectory_file(raw)
    historical = yaml.safe_load(config_before)
    historical["real_home"] = window.loaded_trajectory.metadata["real_home_q"].tolist()
    historical["real_home_metadata"] = dict(
        q_rad=historical["real_home"],
        realhome_version=window.loaded_trajectory.metadata["realhome_version"],
        collision_scene_version=window.loaded_trajectory.metadata["collision_scene_version"],
        calibration_binding=window.loaded_trajectory.metadata["calibration_binding"])
    window.realhome_config_path = output / "TEST_HISTORICAL_CONFIG.yaml"
    window.realhome_config_path.write_text(yaml.safe_dump(historical))
    window.planning_workflow.raw_path = raw
    q = window.loaded_trajectory.positions[0].tolist()
    # Synthetic non-motion-ready state. Readiness result below is a UI replay
    # fixture only and never enables the execution button.
    window._real_planning_readiness_reasons = lambda: []
    def produce():
        while not stop.is_set():
            traffic.state.emit(RobotState(positions=q, joint_stamp=time.monotonic(),
                robot_mode=5, program_running=False, safety_mode=1, robot_link_alive=True))
            count[0] += 1
            stop.wait(.02)
    producer = threading.Thread(target=produce, daemon=True)
    producer.start()
    beats = []
    launched = []
    heartbeat = QTimer()
    heartbeat.setInterval(20)
    heartbeat.timeout.connect(lambda: beats.append((time.monotonic(), window.planning_busy, len(launched))))
    heartbeat.start()
    original_launch = PlanningWorkflowPanel._launch_workflow_process
    class ReplayProcess(QProcess):
        def start(self, program, args):
            assert program == "matlab"
            run_dir = window.planning_report_path.parent
            fixture = json.loads(source_bytes)
            for name in ("offline_optimized_trajectory.mat", "offline_optimized_trajectory.json",
                         "offline_tracking_curves.csv"):
                shutil.copy2(prepared / name, run_dir / name)
            fixture["artifacts"]["trajectory_mat"] = str(run_dir / "offline_optimized_trajectory.mat")
            fixture["artifacts"]["trajectory_json"] = str(run_dir / "offline_optimized_trajectory.json")
            fixture["ui_test_scope"] = "REPLAY_ONLY_NOT_A_NEW_PLANNING_OR_EXECUTION_RESULT"
            window.planning_report_path.write_text(json.dumps(fixture))
            delay = 5. if len(launched) == 2 else .4
            super().start(sys.executable, ["-u", "-c",
                "import time; print('HMI_WELD_STAGE stage=BOUNDARIES',flush=True);"
                f"time.sleep({delay});"
                "print('HMI_WELD_STAGE stage=VELOCITY_OPTIMIZATION',flush=True);"
                f"time.sleep({delay});"
                "print('HMI_WELD_STAGE stage=PHYSICAL_VALIDATION_PENDING',flush=True)"])
    def launch(self, phase, entry, payload):
        launched.append(entry)
        # Isolate all replay outputs from actual workflow runs and calibration.
        with patch("gui.main_window.ROOT", output / "UI_FIXTURES" / "ur5_hmi"), \
             patch("gui.planning_workflow.QProcess", ReplayProcess):
            original_launch(self, phase, entry, payload)
    deadline = time.monotonic() + 20
    try:
        with patch.object(PlanningWorkflowPanel, "_launch_workflow_process", launch), \
             patch.object(MainWindow, "_auto_read_robot_tcp", no_robot_tcp_reads):
            window._update_buttons()
            start = time.monotonic()
            window.online_continuous_button.click()
            button_seconds = time.monotonic() - start
            assert window.planning_busy and not window.execute_button.isEnabled()
            while window.planning_busy and time.monotonic() < deadline:
                app.processEvents()
                time.sleep(.002)
            assert not window.planning_busy, "Completion notification starved"
            assert window.planning_workflow.stage == PlanningStage.PHYSICAL_VALIDATION_PENDING, window.online_status_label.text()
            assert not window._scalar_bool(window.loaded_trajectory.metadata["executionReady"], True)
            assert window.tabs.currentIndex() == window.trajectory_tab_index
            assert Path(window.loaded_path_name).parent == window.trajectory_results_directory
            assert Path(window.loaded_path_name).name.startswith("连续规划完成轨迹_")
            assert window.execution_report_file.name.endswith("_report.json")
            assert "VELOCITY_OPTIMIZATION" in window.online_output_view.toPlainText()
            # Check cancel by clicking the real cancel button of a second
            # test-owned process. Never touches the user's MATLAB/HMI process.
            window.online_continuous_button.click()
            assert window.planning_busy
            cancel_started = time.monotonic()
            QTimer.singleShot(250, window.online_cancel_button.click)
            while window.planning_busy and time.monotonic() < deadline:
                app.processEvents()
                time.sleep(.002)
            cancel_seconds = time.monotonic() - cancel_started
            assert window.planning_workflow.stage == PlanningStage.CANCELED
            assert cancel_seconds < 2
            assert window.planning_workflow.raw_path == raw
            assert not sends
            assert CONFIG.read_bytes() == config_before
            assert source_report.read_bytes() == source_bytes
            busy_gaps = [b[0]-a[0] for a,b in zip(beats, beats[1:]) if a[1] and b[1] and a[2] == b[2]]
            all_gaps = [b[0]-a[0] for a,b in zip(beats, beats[1:])]
            assert max(busy_gaps, default=0) < 1., busy_gaps
            result = dict(pass_all=True, view=type(window.robot_view).__name__,
                controller_input="SYNTHETIC non-motion-ready 50 Hz; ROS/RTDE disabled",
                matlab="Real QProcess callbacks, genuine saved result REPLAY; no new MATLAB optimization",
                button_return_s=button_seconds, max_heartbeat_gap_while_busy_s=max(busy_gaps,default=0),
                max_heartbeat_gap_including_completion_s=max(all_gaps,default=0),
                state_notifications=count[0], display_timer_interval_ms=window.state_display_timer.interval(),
                cancel_to_finished_s=cancel_seconds, completion_delivered=True,
                busy_execution_blocked=True, no_motion_signals=True, source_and_config_unchanged=True,
                executionReady=False)
            window.grab().save(str(output / "responsive_hmi.png"))
            (output / "responsiveness_tests.json").write_text(json.dumps(result, indent=2))
            print(json.dumps(result, indent=2))
    finally:
        stop.set()
        producer.join(1)
        heartbeat.stop()
        window.close()
        app.processEvents()


if __name__ == "__main__":
    run(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve())
