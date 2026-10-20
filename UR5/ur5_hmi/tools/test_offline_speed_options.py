"""Actual Qt buttons, no ROS thread/RTDE; optional real MATLAB result replay.

Writes diagnostics only. Never sends a motion, speed-slider or controller call.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import sys
import subprocess
import time
from unittest.mock import patch
import warnings

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_offline_speed_mpl")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from PyQt5.QtCore import QProcess, QThread, QByteArray
from PyQt5.QtTest import QSignalSpy
from PyQt5.QtWidgets import QApplication
from gui.main_window import CONFIG, MainWindow
from gui.planning_workflow import PlanningStage
from safety.execution_preparation import file_hash, offline_context
from safety.robot_motion_limits import check_motion
from trajectory.trajectory_loader import load_trajectory


def no_thread_start(self):
    pass


def no_auto_tcp(self):
    # Kept as a module-level callable: Qt also queues a one-shot startup
    # callback; a temporary patched lambda can die before that event runs.
    pass


def no_show_maximized(self):
    pass


def run(source, output, completed=None, read_controller=False):
    warnings.filterwarnings("ignore", message="Glyph.*missing from current font")
    initial_config = CONFIG.read_bytes()
    root = CONFIG.parents[2]
    index = root / "规划文件/当前轨迹/最新轨迹索引.json"
    initial_index = index.read_bytes()
    source_hash = file_hash(source)
    captured, sends = {}, []

    class CaptureProcess(QProcess):
        def start(self, program, arguments):
            captured.update(program=program, arguments=list(arguments))
        def readAllStandardOutput(self): return QByteArray()
        def readAllStandardError(self): return QByteArray()

    app = QApplication.instance() or QApplication([])
    with patch.object(QThread, "start", no_thread_start), \
         patch.object(MainWindow, "_auto_read_robot_tcp", no_auto_tcp), \
         patch.object(MainWindow, "showMaximized", no_show_maximized):
        window = MainWindow()
    window.robot_tcp_auto_timer.stop()
    spies = {name: QSignalSpy(signal) for name, signal in
        (("trajectory", window.worker.request_trajectory), ("speed", window.worker.request_speed_percent),
         ("jog", window.worker.request_jog), ("stop", window.worker.request_stop))}
    class Signals:
        def __len__(self): return sum(len(spy) for spy in spies.values())
        def __repr__(self): return repr({name:list(spy) for name,spy in spies.items() if len(spy)})
    sends = Signals()
    try:
        window.trajectory_results_directory = output.parent / "ISOLATED_CURRENT_TRAJECTORIES"
        window.calibration_source_combo.blockSignals(True)
        window.calibration_source_combo.setCurrentText("REAL CALIBRATION")
        window.calibration_source_combo.blockSignals(False)
        window._adopt_trajectory_file(source, auto_validate=False)
        if read_controller:
            # The native helper's default mode has an empty RTDE input recipe;
            # never pass --request or invoke the TCP synchronization callback.
            result = subprocess.run(["timeout", "6s", str(root / "ur5_hmi/bin/robot_tcp_helper"),
                "--robot-ip", "192.168.0.10"], capture_output=True, text=True, check=True)
            packet = json.loads(result.stdout)
            assert packet['operation'] == 'read' and packet['success'] and packet['output_only'] and packet['input_recipe_empty']
            window.robot_tcp_snapshot = packet['snapshot']
            window.robot_tcp_received = time.monotonic()
            captured['RTDE_output_only_read'] = packet
        else:
            # Pure GUI replay uses the source request's recorded empty list;
            # it is not a statement about live controller readiness.
            window._real_planning_readiness_reasons = lambda: []
            captured['readiness_scope'] = 'OFFLINE_GUI_REPLAY_NOT_LIVE_PERMISSION'
        window._execution_context()  # Real complete source + current versions, not a fabricated PASS.
        assert window.continuous_output_speed_spin.value() == 50
        assert window.offline_speed_spin.value() == 50
        assert window.offline_speed_button.isEnabled()
        assert not window.execute_button.isEnabled()
        old = load_trajectory(str(source))
        q, t, v = old.positions.copy(), old.time_from_start.copy(), old.velocities.copy()
        runtime_speed = window.real_speed_spin.value()
        window.offline_speed_spin.setValue(25)
        assert f"{t[-1]*4:.3f}" in window.offline_speed_estimate_label.text()
        window.offline_speed_spin.setValue(50)
        assert f"{t[-1]*2:.3f}" in window.offline_speed_estimate_label.text()
        assert np.array_equal(window.loaded_trajectory.positions, q)
        assert np.array_equal(window.loaded_trajectory.time_from_start, t)
        assert np.array_equal(window.loaded_trajectory.velocities, v)
        assert window.real_speed_spin.value() == runtime_speed and not sends

        # A source preview cannot invoke retiming or skip continuous planning.
        stage = window.loaded_trajectory.metadata["workflow_stage"]
        window.loaded_trajectory.metadata["workflow_stage"] = "PREVIEW"
        window._update_offline_speed_options()
        assert not window.offline_speed_button.isEnabled()
        window.loaded_trajectory.metadata["workflow_stage"] = stage
        window._update_offline_speed_options()
        # Busy motion must disable both timing controls.
        window.planning_busy = True; window._update_offline_speed_options()
        assert not window.offline_speed_button.isEnabled() and not window.offline_speed_spin.isEnabled()
        window.planning_busy = False; window._update_offline_speed_options()

        with patch("gui.planning_workflow.QProcess", CaptureProcess):
            window.offline_speed_button.click()
        assert captured and window.planning_workflow.stage == PlanningStage.CONTINUOUS_PLANNING
        request = json.loads(window.planning_request_path.read_text())
        assert request["offline_speed_percent"] == 50
        assert request["source_sha256"] == source_hash
        assert request["execution_gate_reasons"] == []
        assert request["minControllerSamples"] >= 5000
        assert "retime_hmi_continuous_request" in captured["arguments"][1]
        captured.update(request=str(window.planning_request_path), outputDir=request["outputDir"],
            report=str(window.planning_report_path), actual_button_clicked=True,
            no_ros_thread=True, motion_speed_or_stop_signals=len(sends), robot_motion="NO",
            source_unmodified=True, default_new_continuous_percent=50,
            default_loaded_file_retime_percent=50, estimates_checked=True,
            preview_and_busy_rejected=True)
        captured["shell_command"] = shlex.join([captured["program"], *captured["arguments"]])
        retime_capture = dict(captured)
        # Check the ordinary continuous-planning button uses the new default
        # too. It must not require a second audit or a separate button click.
        window.planning_busy = window.continuity_busy = False
        window.planning_process = None
        window._update_buttons()
        with patch("gui.planning_workflow.QProcess", CaptureProcess):
            window.online_continuous_button.click()
        default_request = json.loads(window.planning_request_path.read_text())
        assert default_request['offline_speed_percent'] == 50
        assert 'prepare_hmi_continuous_request' in captured['arguments'][1]
        default_capture = dict(program=captured['program'], arguments=captured['arguments'],
            request=str(window.planning_request_path), outputDir=default_request['outputDir'],
            default_new_continuous_percent=50, actual_button_clicked=True, robot_motion='NO')
        output.parent.mkdir(parents=True, exist_ok=True)
        output.with_name(output.stem + '_default_request.json').write_text(
            json.dumps(default_capture, ensure_ascii=False, indent=2)+'\n')
        captured.clear(); captured.update(retime_capture)
        captured['new_continuous_default_capture'] = str(output.with_name(output.stem + '_default_request.json'))

        if completed:
            # Complete the real production callback with the independently run
            # MATLAB result. Publication stays in an isolated result directory.
            finished = json.loads(completed.read_text())
            window.planning_report_path = completed
            window.planning_output_path = Path(finished["artifacts"]["trajectory_mat"])
            window._online_process_finished(0, QProcess.NormalExit)
            assert window.planning_workflow.stage == PlanningStage.PHYSICAL_VALIDATION_PENDING, window.online_status_label.text()
            scaled = window.loaded_trajectory
            assert np.array_equal(scaled.positions, q)
            assert np.array_equal(scaled.time_from_start, t*2)
            assert np.array_equal(scaled.velocities, v*.5)
            assert not scaled.metadata["executionReady"] and not window.execute_button.isEnabled()
            assert "保守调试轨迹_50pct" in scaled.source_path
            assert window.offline_speed_spin.value() == 100
            assert not window.offline_speed_button.isEnabled()
            assert window.continuous_output_speed_spin.value() == 50
            window._execution_context()
            reasons, metrics = check_motion(scaled.positions, scaled.time_from_start, scaled.velocities, execution=True)
            assert not reasons, reasons  # Only the numerical compatibility gate; no live authorization.
            captured.update(completion_callback_pass=True, auto_import_pass=True,
                unchanged_q_path=True, exact_time_doubling=True, exact_velocity_halving=True,
                numerical_compatibility_gate_pass=True, execution_button_enabled=False,
                already_scaled_file_defaults_to_100_no_accidental_second_halving=True,
                duration_sec=float(scaled.time_from_start[-1]),
                maximum_velocity_rad_s=float(max(metrics['max_velocity_per_joint'])),
                maximum_acceleration_rad_s2=float(max(metrics['max_acceleration_per_joint'])),
                published_path=scaled.source_path)
            # A genuinely failed report must not be turned into offline PASS.
            from gui.planning_workflow import PlanningWorkflow
            bad = dict(finished); bad["controller_validation"] = dict(finished["controller_validation"], collision_pass=False)
            try:
                PlanningWorkflow().preparation_complete(Path(finished["artifacts"]["trajectory_mat"]), bad)
                raise AssertionError("collision failure accepted")
            except ValueError:
                captured["collision_fail_closed"] = True
            window.showNormal(); window.resize(1920, 1080)
            window.tabs.setCurrentIndex(window.trajectory_tab_index)
            app.processEvents()
            shot = output.with_suffix('.png')
            window.grab().save(str(shot))
            captured['ui_screenshot'] = str(shot)
        assert file_hash(source) == source_hash and CONFIG.read_bytes() == initial_config
        assert index.read_bytes() == initial_index, 'production index changed'
        assert not sends, repr(sends)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(captured, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps(captured, ensure_ascii=False, indent=2))
    finally:
        window.planning_busy = window.continuity_busy = False
        window.planning_process = None
        window.close(); app.processEvents()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--completed", type=Path)
    parser.add_argument("--read-controller", action="store_true")
    args = parser.parse_args()
    run(args.source.resolve(), args.output.resolve(), args.completed.resolve() if args.completed else None, args.read_controller)
