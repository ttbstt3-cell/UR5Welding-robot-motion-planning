"""Versioned RealHome regression; temporary YAML, mock joints/MATLAB, no ROS."""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_realhome_test_mpl")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
import yaml
from PyQt5.QtCore import QProcess, QThread
from PyQt5.QtWidgets import QApplication
from gui.main_window import CONFIG, MainWindow
from gui.realhome_workflow import read_realhome_record, save_realhome_record, store_realhome_validation
from gui.planning_workflow import PlanningStage


def run() -> None:
    app = QApplication.instance() or QApplication([])
    production_before = CONFIG.read_bytes()
    binding = {"planning_scene": "REAL_CALIBRATION", "urdf_version": "URDF-test",
               "tcp_calibration_version": "TCP-test", "gun_geometry_version": "GUN-test",
               "wire_model_version": "WIRE-test", "collision_scene_version": "SCENE-test"}
    with tempfile.TemporaryDirectory(prefix="ur5_realhome_regression_") as directory:
        temporary = Path(directory)
        config = temporary / "hmi_config.yaml"
        config.write_text(yaml.safe_dump({"real_home": [1, 2, 3, 4, 5, 6], "untouched": 42}), encoding="utf-8")
        legacy = read_realhome_record(config)
        assert legacy["realhome_version"].startswith("LEGACY-")
        assert legacy["validation"]["status"] == "UNKNOWN"
        first = save_realhome_record(config, np.zeros(6), binding)
        assert first["q_rad"] == [0.0] * 6
        assert first["validation"]["status"] == "PENDING"
        assert all(first[key] == value for key, value in binding.items() if key.endswith("version"))
        assert yaml.safe_load(config.read_text())["untouched"] == 42
        assert not store_realhome_validation(config, "OLD", "SCENE-test", {"status": "PASS"})
        second = save_realhome_record(config, [0.01] * 6, binding)
        assert second["realhome_version"] != first["realhome_version"]
        assert not store_realhome_validation(config, first["realhome_version"], "SCENE-test", {"status": "PASS"})
        assert store_realhome_validation(config, second["realhome_version"], "SCENE-test", {"status": "FAIL",
                    "collision_pairs": ["forearm_link ↔ wrist_2_link"]})
        print("PASS rad samples/version bindings, atomic save, legacy unknown, stale results rejected")

        with patch.object(QThread, "start", lambda self: None):
            window = MainWindow()
        window.realhome_config_path = config
        window.realhome_validation_run_root = temporary / "runs"
        window._current_calibration_binding = lambda: dict(binding)
        window._realhome_validation_output = lambda *args: None
        window._feedback_valid = lambda state: True
        window.state.positions = [0.0] * 6
        window.planning_workflow.stage = PlanningStage.EXECUTION_READY
        window.planning_workflow.raw_path = temporary / "old.mat"
        window.planning_workflow.ready_path = temporary / "ready.mat"
        window.full_raw_trajectory_path = temporary / "old.mat"
        window.trajectory_validation = object()
        launched = []
        before_refusal = config.read_bytes()
        window._current_calibration_binding = lambda: dict(binding, planning_scene="FROZEN_SIMULATION")
        with patch.object(QProcess, "start", lambda self, program, args: launched.append((program, args))):
            window._save_home()
        assert config.read_bytes() == before_refusal
        assert launched == []
        assert "请选择REAL CALIBRATION后保存" in window.home_validation_label.text()
        assert window.planning_workflow.stage == PlanningStage.EXECUTION_READY
        window._current_calibration_binding = lambda: dict(binding)
        print("PASS SIMULATION save refused with explicit REAL CALIBRATION prompt; saved sample unchanged")
        with patch.object(QProcess, "start", lambda self, program, args: launched.append((program, args))):
            window._save_home()
        saved = read_realhome_record(config)
        assert saved["q_rad"] == window.state.positions
        assert np.array_equal(window._saved_real_home(), window.state.positions)
        assert window.online_start_combo.currentData() == "real_home"
        assert window.planning_workflow.stage == PlanningStage.IDLE
        assert window.planning_workflow.raw_path is None and window.planning_workflow.ready_path is None
        assert window.full_raw_trajectory_path is None and window.trajectory_validation is None
        assert "PENDING" in window.home_validation_label.text()
        assert len(launched) == 1 and "validate_hmi_realhome_request" in launched[0][1][1]
        request = json.loads((window.realhome_validation_run / "request.json").read_text())
        assert request["qStart"] == saved["q_rad"]
        assert request["realhome_version"] == saved["realhome_version"]
        assert request["collision_scene_version"] == binding["collision_scene_version"]
        assert request["executionReady"] is False
        print("PASS save immediately refreshes planning start, clears old caches, launches offline validation")

        failure = {"realhome_version": saved["realhome_version"],
                   "collision_scene_version": binding["collision_scene_version"], "status": "FAIL",
                   "realhome_validation_status": "FAIL", "collision_pairs": ["forearm_link ↔ wrist_2_link"],
                   "endpoint_validation": {"valid": False, "collision_pairs": ["forearm_link ↔ wrist_2_link"]}}
        process = window.realhome_validation_process
        (window.realhome_validation_run / "report.json").write_text(json.dumps(failure), encoding="utf-8")
        window._realhome_validation_finished(process, saved, window.realhome_validation_run, 0)
        assert "FAIL" in window.home_validation_label.text() and "forearm_link" in window.home_validation_label.text()
        assert window.realhome_validation_busy is False

        window.state.positions = [0.02] * 6
        with patch.object(QProcess, "start", lambda *args: None):
            window._save_home()
        current = read_realhome_record(config)
        assert current["realhome_version"] != saved["realhome_version"]
        assert not window._accept_realhome_validation(saved, {"status": "PASS"})
        success = {"realhome_version": current["realhome_version"],
                   "collision_scene_version": binding["collision_scene_version"], "status": "PASS",
                   "success": True, "binding_valid": True,
                   "realhome_validation_status": "PASS", "collision_pairs": [],
                   "endpoint_validation": {"valid": True, "collision_pairs": []}}
        (window.realhome_validation_run / "report.json").write_text(json.dumps(success), encoding="utf-8")
        window._realhome_validation_finished(window.realhome_validation_process, current, window.realhome_validation_run, 0)
        assert "PASS" in window.home_validation_label.text()
        assert read_realhome_record(config)["validation"]["status"] == "PASS"
        captured = []
        window._launch_workflow_process = lambda phase, entry, payload: captured.append((phase, entry, payload))
        window._start_preview_planning()
        payload = captured[-1][2]
        assert payload["qStart"] == current["q_rad"]
        assert payload["qHome"] == current["q_rad"]
        assert payload["realhome_version"] == current["realhome_version"]
        assert payload["collision_scene_version"] == binding["collision_scene_version"]
        assert payload["preweld_version"] is None and payload["regenerate_preweld"]
        assert "seamCandidateFile" not in payload
        assert window.online_start_state_label.text() == "CHECKING"
        assert window.online_goal_state_label.text() == "NOT GENERATED"
        window._parse_online_stage_line("START COLLISION CHECK: FREE")
        assert window.online_start_state_label.text() == "PASS"
        window._parse_online_stage_line("GOAL COLLISION CHECK: COLLISION/INVALID")
        assert window.online_goal_state_label.text() == "FAIL"
        window.realhome_validation_busy = True
        window._start_preview_planning()
        assert len(captured) == 1
        window.realhome_validation_busy = False
        stale_memory = dict(window.config)
        fresh = save_realhome_record(config, [0.03] * 6, binding)
        window.config = stale_memory
        window._ui_scale_changed("100%")
        assert read_realhome_record(config)["realhome_version"] == fresh["realhome_version"]
        assert np.array_equal(window._saved_real_home(), [0.03] * 6)
        current = fresh
        print("PASS planning uses latest versioned Home, cold PreWeld request, endpoint labels, busy gating, UI save preserves fresh Home")
        binding["collision_scene_version"] = "SCENE-next"
        window._refresh_realhome_panel()
        assert "STALE" in window.home_validation_label.text()
        assert not window._accept_realhome_validation(current, {"status": "PASS"})
        print("PASS collision pair displayed, new sample rejects old validation, changed scene displays STALE")
        window.close()
        app.processEvents()
    assert CONFIG.read_bytes() == production_before
    print("REALHOME_WORKFLOW_TEST_PASS (production Home unchanged; NO ROS / MATLAB / ROBOT MOTION)")


if __name__ == "__main__":
    run()
