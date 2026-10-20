"""Offline software tests. All field/servo evidence below is SYNTHETIC.

ROS threads never start. No successful send path is invoked; no motion signal
may be emitted. Synthetic authorizations exist only in a temporary directory.
"""
from __future__ import annotations
import copy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from unittest.mock import patch
import warnings

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_execution_preparation_mpl")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import yaml
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication, QDialog, QLineEdit, QCheckBox, QFileDialog
from gui.main_window import MainWindow, CONFIG
from ros.state_monitor import RobotState
from safety.execution_preparation import (draft_record, offline_context, file_hash,
    issue_authorization, validate_record, validate_authorization)
from trajectory.trajectory_loader import load_trajectory


def synthetic_record(context, folder):
    record = draft_record(context)
    record.update(status="REVIEWED", inspector="SYNTHETIC UNIT TEST - NOT A FIELD INSPECTION",
        qualification="TEST FIXTURE", procedure_reference="SOFTWARE_TEST_ONLY",
        performed_at_utc=datetime.now(timezone.utc).isoformat())
    for kind in ("physical", "servo"):
        path = folder / (kind + "_SYNTHETIC.txt")
        path.write_text("SYNTHETIC SOFTWARE TEST; NO ROBOT OR PHYSICAL VALIDATION", encoding="utf-8")
        record[kind].update(status="PASS", report_file=str(path), report_sha256=file_hash(path))
    record["physical"].update(minimum_verified_clearance_mm=20., uncertainty_mm=2., required_margin_mm=5.,
        installation_verified=True, workpiece_environment_verified=True, wire_approximation_reviewed=True)
    record["servo"].update(max_tcp_tracking_error_mm=.1, max_tcp_orientation_error_deg=.1,
        approved_velocity_rad_s=math.radians(5), approved_acceleration_rad_s2=.261799,
        actual_controller_and_scaling_verified=True)
    return record


def run(trajectory_path, output):
    warnings.filterwarnings("ignore", message="Glyph.*missing from current font")
    output.mkdir(parents=True, exist_ok=True)
    traj = load_trajectory(str(trajectory_path)); original_bytes = trajectory_path.read_bytes()
    report_path = trajectory_path.parent / "report.json"
    context = offline_context(traj, report_path, traj.metadata["calibration_binding"], traj.metadata["realhome_version"])
    original_config = CONFIG.read_bytes(); rejected = []
    with tempfile.TemporaryDirectory(prefix="ur5_execution_review_TEST_") as td:
        folder = Path(td); record = synthetic_record(context, folder)
        record_path = folder / "field_record.json"
        record_path.write_text(json.dumps(record), encoding="utf-8")
        validate_record(record, context, folder)
        permit = issue_authorization(record, context, record_path, "UNIT_TEST_OPERATOR")
        validate_authorization(permit, context)
        cases = [("draft", ("status",), "DRAFT"), ("no_inspector", ("inspector",), ""),
            ("no_qualification", ("qualification",), ""), ("no_procedure", ("procedure_reference",), ""),
            ("no_timezone", ("performed_at_utc",), "2026-10-19T09:00:00"),
            ("missing_evidence", ("physical", "report_file"), "missing.pdf"),
            ("evidence_hash", ("servo", "report_sha256"), "wrong"),
            ("physical_pending", ("physical", "status"), "PENDING"),
            ("servo_pending", ("servo", "status"), "PENDING"),
            ("clearance", ("physical", "minimum_verified_clearance_mm"), 1.),
            ("no_margin", ("physical", "required_margin_mm"), 0.),
            ("negative_uncertainty", ("physical", "uncertainty_mm"), -1.),
            ("wire_not_reviewed", ("physical", "wire_approximation_reviewed"), False),
            ("controller", ("servo", "controller"), "unknown"),
            ("quintic", ("servo", "interpolation_mode"), "quintic"),
            ("coordinates", ("servo", "joint_coordinate_convention"), "COPPELIA_DEG"),
            ("scaling_unverified", ("servo", "actual_controller_and_scaling_verified"), False),
            ("tracking_limit", ("servo", "max_tcp_tracking_error_mm"), .5),
            ("orientation_limit", ("servo", "max_tcp_orientation_error_deg"), .6),
            ("NaN", ("servo", "max_tcp_tracking_error_mm"), float("nan")),
            ("velocity_envelope", ("servo", "approved_velocity_rad_s"), .001),
            ("acceleration_envelope", ("servo", "approved_acceleration_rad_s2"), .001),
            ("stale_binding", ("context", "realhome_version"), "CHANGED")]
        for name, keys, value in cases:
            bad = copy.deepcopy(record); node = bad
            for key in keys[:-1]: node = node[key]
            node[keys[-1]] = value
            try: validate_record(bad, context, folder)
            except (ValueError, OSError, KeyError): rejected.append(name)
            else: raise AssertionError("Bad evidence accepted: " + name)
        for name, now in (("expired_permit", permit["expires_at"]+1),
                          ("future_permit", permit["issued_at"]-1)):
            try: validate_authorization(permit, context, now)
            except ValueError: rejected.append(name)
            else: raise AssertionError(name)
        record_path.write_text(json.dumps(dict(record, inspector="CHANGED")))
        try: validate_authorization(permit, context)
        except ValueError: rejected.append("record_changed")
        else: raise AssertionError("record changed")
        record_path.write_text(json.dumps(record))
        measured = Path(record["physical"]["report_file"]); original = measured.read_bytes()
        measured.write_bytes(b"changed")
        try: validate_authorization(permit, context)
        except ValueError: rejected.append("attachment_changed")
        else: raise AssertionError("attachment changed")
        measured.write_bytes(original)
        modified = copy.deepcopy(traj); modified.positions[0, 0] += .001
        try: offline_context(modified, report_path, traj.metadata["calibration_binding"], traj.metadata["realhome_version"])
        except ValueError: rejected.append("in_memory_nodes_changed")
        else: raise AssertionError("modified trajectory accepted")

        app = QApplication.instance() or QApplication([])
        with patch.object(QThread, "start", lambda self: None): window = MainWindow()
        try:
            fixture = yaml.safe_load(original_config)
            q_home = list(map(float, traj.metadata["real_home_q"]))
            fixture["real_home"] = q_home
            fixture["real_home_metadata"] = dict(q_rad=q_home,
                realhome_version=traj.metadata["realhome_version"], collision_scene_version=traj.metadata["collision_scene_version"],
                calibration_binding=traj.metadata["calibration_binding"], source="SYNTHETIC_HMI_TEST_ONLY")
            config = folder / "test_config.yaml"; config.write_text(yaml.safe_dump(fixture))
            window.realhome_config_path = config
            window._execution_review_directory = lambda: folder / "local_reviews"
            window.calibration_source_combo.blockSignals(True)
            window.calibration_source_combo.setCurrentText("REAL CALIBRATION")
            window.calibration_source_combo.blockSignals(False)
            window._adopt_trajectory_file(trajectory_path)
            emissions = []; window.worker.request_trajectory.connect(lambda msg: emissions.append(msg))
            def fresh():
                window.state = RobotState(joint_names=traj.joint_names, positions=traj.positions[0].tolist(),
                    joint_stamp=time.monotonic(), program_running=True, robot_mode=7, safety_mode=1,
                    speed_scaling=100., controller_active=True, action_server_ready=True, robot_link_alive=True)
            fresh(); window.preparation_buttons["check"].click()
            assert not window.execute_button.isEnabled()
            assert "缺少现场" in window.preparation_labels["field"].text()
            draft_path = folder / "draft.json"
            with patch.object(QFileDialog, "getSaveFileName", return_value=(str(draft_path), "")):
                window.preparation_buttons["draft"].click()
            assert json.loads(draft_path.read_text())["status"] == "DRAFT"
            assert not window.execute_button.isEnabled()
            record = synthetic_record(window._execution_context(), folder)
            record_path.write_text(json.dumps(record))
            fresh()
            with patch.object(QFileDialog, "getOpenFileName", return_value=(str(record_path), "")):
                window.preparation_buttons["import"].click()
            assert window.preparation_buttons["authorize"].isEnabled()
            assert not window.execute_button.isEnabled()
            def approve_test(dialog):
                dialog.findChild(QLineEdit).setText("SOFTWARE_TEST_ONLY")
                dialog.findChild(QCheckBox).setChecked(True)
                return QDialog.Accepted
            fresh()
            with patch.object(QDialog, "exec_", approve_test): window.preparation_buttons["authorize"].click()
            assert window.execution_authorization and window._final_execution_gate_ok()
            assert window.execute_button.isEnabled()
            assert window.loaded_trajectory.metadata["preview_only"] and not window.loaded_trajectory.metadata["executionReady"]
            with patch.object(window, "_confirm_execute", return_value=False): window._execute_path()
            assert not emissions
            def disconnect_in_confirmation():
                window.state.controller_active = False
                return True
            fresh()
            with patch.object(window, "_confirm_execute", side_effect=disconnect_in_confirmation): window._execute_path()
            assert not emissions and not window._final_execution_gate_ok()
            fresh(); saved_permit = copy.deepcopy(window.execution_authorization)
            window.execution_authorization["expires_at"] = time.time()-1
            window._update_buttons(); assert not window.execute_button.isEnabled()
            window.execution_authorization = saved_permit
            window.state.joint_stamp -= 10; window._update_buttons()
            assert not window.execute_button.isEnabled()
            fresh(); window._revoke_execution_authorization()
            assert not window.execute_button.isEnabled()
            assert window.execution_authorization is None
            window.execution_authorization = saved_permit
            window._adopt_trajectory_file(trajectory_path)
            assert window.execution_authorization is None and window.field_acceptance_path is None
            assert not emissions
            # Screenshots show the real pending state, NOT synthetic readiness.
            window.state = RobotState(); window._check_execution_preparation()
            window.showNormal(); window.resize(1920, 1080)
            window.tabs.setCurrentIndex(window.trajectory_tab_index)
            layouts = []
            for scale in (1., 1.25, 1.5):
                window.apply_ui_scale(scale); app.processEvents()
                window.trajectory_scroll.ensureWidgetVisible(window.execution_preparation_box)
                app.processEvents()
                filename = output / ("execution_preparation_"+str(int(scale*100))+".png")
                window.grab().save(str(filename))
                assert window.execution_preparation_box.width() <= window.trajectory_scroll.viewport().width()
                layouts.append(dict(scale=scale, panel_width=window.execution_preparation_box.width(),
                                    viewport_width=window.trajectory_scroll.viewport().width(), screenshot=str(filename)))
            assert CONFIG.read_bytes() == original_config and trajectory_path.read_bytes() == original_bytes
            result = dict(pass_all=True, real_offline_context_pass=True,
                negative_evidence_cases=rejected, GUI_import_draft_authorize_revoke_reload_pass=True,
                modal_condition_change_blocks_send=True, no_motion_signals=True, source_and_config_unchanged=True,
                scope="SYNTHETIC FIELD/SERVO RECORDS FOR SOFTWARE TESTS ONLY; NOT PHYSICAL VALIDATION",
                production_execution_authorizations=[], layouts=layouts, robot_motion="NO")
            (output/"tests.json").write_text(json.dumps(result, ensure_ascii=False, indent=2))
            print(json.dumps(result, ensure_ascii=False, indent=2))
        finally:
            window.close(); app.processEvents()


if __name__ == "__main__":
    run(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve())
