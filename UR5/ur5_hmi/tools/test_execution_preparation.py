"""Offline UI/software preflight regression. Never starts ROS or sends motion.

Ready robot state is SYNTHETIC and tests button gating only. No successful
execution-confirmation/send is performed; real physical/servo PASS is not claimed.
"""
from __future__ import annotations
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from unittest.mock import patch
import warnings

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_execution_preparation_mpl")
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT))
import yaml
import numpy as np
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication, QDialog, QCheckBox, QDialogButtonBox
from gui.main_window import MainWindow, CONFIG
from ros.state_monitor import RobotState
from safety.execution_preparation import offline_context
from trajectory.trajectory_loader import load_trajectory


def run(path, output):
    warnings.filterwarnings("ignore", message="Glyph.*missing from current font")
    output.mkdir(parents=True, exist_ok=True)
    traj = load_trajectory(str(path)); source_bytes = path.read_bytes()
    report_file = path.parent / "report.json"; report = json.loads(report_file.read_text())
    binding = traj.metadata["calibration_binding"]; home = traj.metadata["realhome_version"]
    offline_context(traj, report_file, binding, home)
    rejected = []; config_before = CONFIG.read_bytes()
    with tempfile.TemporaryDirectory(prefix="ur5_simple_execution_TEST_") as td:
        folder = Path(td)
        for name, keys, value in (("collision", ("controller_validation", "collision_pass"), False),
            ("joint_limits", ("controller_validation", "joint_limits_pass"), False),
            ("sync_error", ("controller_validation", "max_tcp_time_sync_error_mm"), .6),
            ("orientation", ("controller_validation", "max_tcp_orientation_error_deg"), .6),
            ("acceleration", ("controller_validation", "max_joint_acceleration_rad_per_sec2"), .3),
            ("samples", ("controller_validation", "sample_count"), 4999),
            ("incomplete", ("continuous_planning_complete",), False),
            ("execution_claim", ("executionReady",), True),
            ("version_validation", ("version_validation", "pass"), False)):
            bad = copy.deepcopy(report); node = bad
            for key in keys[:-1]: node = node[key]
            node[keys[-1]] = value
            fixture = folder / "bad_report.json"; fixture.write_text(json.dumps(bad))
            try: offline_context(traj, fixture, binding, home)
            except ValueError: rejected.append(name)
            else: raise AssertionError("Invalid report accepted: " + name)
        for name, field in (("changed_q", "positions"), ("changed_velocity", "velocities"),
                            ("changed_time", "time_from_start")):
            bad = copy.deepcopy(traj); getattr(bad, field).flat[0] += .001
            try: offline_context(bad, report_file, binding, home)
            except ValueError: rejected.append(name)
            else: raise AssertionError(name)
        bad = copy.deepcopy(traj); bad.accelerations = np.zeros_like(bad.positions)
        try: offline_context(bad, report_file, binding, home)
        except ValueError: rejected.append("wrong_controller_fields")
        else: raise AssertionError("quintic message accepted")
        for name, b, h in (("changed_binding", dict(binding, urdf_version="changed"), home),
                           ("changed_home", binding, "changed")):
            try: offline_context(traj, report_file, b, h)
            except ValueError: rejected.append(name)
            else: raise AssertionError(name)
        bad = copy.deepcopy(traj); bad.metadata["workflow_stage"] = "PREVIEW"
        try: offline_context(bad, report_file, binding, home)
        except ValueError: rejected.append("raw_preview")
        else: raise AssertionError("raw preview accepted")
        app = QApplication.instance() or QApplication([])
        with patch.object(QThread, "start", lambda self: None): window = MainWindow()
        try:
            fixture = yaml.safe_load(config_before); q = list(map(float, traj.metadata["real_home_q"]))
            fixture["real_home"] = q
            fixture["real_home_metadata"] = dict(q_rad=q, realhome_version=home,
                collision_scene_version=traj.metadata["collision_scene_version"], calibration_binding=binding,
                source="SYNTHETIC_HMI_TEST_ONLY")
            config = folder / "test_config.yaml"; config.write_text(yaml.safe_dump(fixture))
            window.realhome_config_path = config
            window.calibration_source_combo.blockSignals(True)
            window.calibration_source_combo.setCurrentText("REAL CALIBRATION")
            window.calibration_source_combo.blockSignals(False)
            window._adopt_trajectory_file(path)
            emitted = []; window.worker.request_trajectory.connect(lambda msg: emitted.append(msg))
            def fresh():
                window.state = RobotState(joint_names=traj.joint_names, positions=traj.positions[0].tolist(),
                    joint_stamp=time.monotonic(), program_running=True, robot_mode=7, safety_mode=1,
                    speed_scaling=100., controller_active=True, action_server_ready=True, robot_link_alive=True)
            fresh(); assert not window.execute_button.isEnabled(), "Must explicitly check the loaded trajectory"
            window.validate_button.click()
            assert window._final_execution_gate_ok() and window.execute_button.isEnabled()
            # GUI cache must notice a new RealHome on disk, even after a
            # successful check. Production YAML is never touched here.
            valid_fixture = config.read_text()
            changed = yaml.safe_load(valid_fixture)
            changed["real_home_metadata"]["realhome_version"] = "TEST_CHANGED_HOME_VERSION"
            config.write_text(yaml.safe_dump(changed))
            assert window._execution_software_reasons()
            rejected.append("cached_context_home_changed")
            config.write_text(valid_fixture)
            assert not window._execution_software_reasons()
            # While planning, preflight work is deferred, NOT granted PASS.
            window.planning_busy = True
            with patch.object(window, "_execution_context", side_effect=AssertionError("busy UI performed preflight")):
                window._update_trajectory_gate(); window._update_buttons()
                assert not window.execute_button.isEnabled()
            window.planning_busy = False
            window._update_trajectory_gate(); window._update_buttons()
            assert window.loaded_trajectory.metadata["preview_only"] and not window.loaded_trajectory.metadata["executionReady"]
            assert "按轨迹速度" in window.execute_button.text()
            def inspect_then_cancel(dialog):
                boxes = dialog.findChildren(QCheckBox)
                assert len(boxes) == 4 and all(not box.isChecked() for box in boxes)
                confirm = next(b for b in dialog.findChild(QDialogButtonBox).buttons() if b.text() == "确认执行")
                assert not confirm.isEnabled()
                for box in boxes: box.setChecked(True)
                assert confirm.isEnabled()
                return QDialog.Rejected
            with patch.object(QDialog, "exec_", inspect_then_cancel): window._execute_path()
            assert not emitted
            for name, field, value in (("controller_inactive", "controller_active", False),
                    ("program_stopped", "program_running", False), ("unsafe_mode", "safety_mode", 3),
                    ("power_off", "robot_mode", 3), ("action_unready", "action_server_ready", False),
                    ("zero_scaling", "speed_scaling", 0.), ("link_down", "robot_link_alive", False)):
                fresh(); setattr(window.state, field, value); window._update_buttons()
                assert not window.execute_button.isEnabled(), name
                rejected.append(name)
            fresh(); window.state.joint_stamp -= 10; window._update_buttons()
            assert not window.execute_button.isEnabled(); rejected.append("stale_feedback")
            fresh(); window.state.positions[0] += .1; window._update_buttons()
            assert not window.execute_button.isEnabled(); rejected.append("start_mismatch")
            def disconnect_while_confirming():
                window.state.controller_active = False
                return True
            fresh()
            with patch.object(window, "_confirm_execute", side_effect=disconnect_while_confirming): window._execute_path()
            assert not emitted and not window._final_execution_gate_ok()
            window._adopt_trajectory_file(path)
            assert window.trajectory_validation is None and not window.execute_button.isEnabled()
            window.state = RobotState(); window._check_execution_preparation()
            window.showNormal(); window.resize(1920, 1080)
            window.tabs.setCurrentIndex(window.trajectory_tab_index)
            layouts = []
            for scale in (1., 1.25, 1.5):
                window.apply_ui_scale(scale); app.processEvents()
                window.trajectory_scroll.ensureWidgetVisible(window.execution_preparation_box)
                app.processEvents()
                screenshot = output / ("simple_execution_"+str(int(scale*100))+".png")
                window.grab().save(str(screenshot))
                width = window.execution_preparation_box.width(); viewport = window.trajectory_scroll.viewport().width()
                assert width <= viewport, (scale, width, viewport)
                layouts.append(dict(scale=scale, panel_width=width, viewport_width=viewport, screenshot=str(screenshot)))
            assert not emitted and CONFIG.read_bytes() == config_before and path.read_bytes() == source_bytes
            result = dict(pass_all=True, negative_cases=rejected, layouts=layouts,
                actual_saved_offline_result_checked=True, no_motion_signals=True,
                physical_or_servo_acceptance_claimed=False, no_execution_permit_files=True,
                source_and_config_unchanged=True,
                scope="Offline GUI/contract tests with SYNTHETIC READY robot state; no successful send invoked",
                robot_motion="NO")
            (output/"simple_execution_tests.json").write_text(json.dumps(result, ensure_ascii=False, indent=2))
            print(json.dumps(result, ensure_ascii=False, indent=2))
        finally:
            window.close(); app.processEvents()


if __name__ == "__main__":
    run(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve())
