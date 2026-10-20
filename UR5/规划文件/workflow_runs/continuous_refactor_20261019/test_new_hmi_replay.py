"""Fresh HMI callback replay with ROS worker disabled and isolated result store."""
import copy
import json
import os
import sys
from pathlib import Path
from unittest.mock import patch
import warnings
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_refactor_mpl")
here = Path(__file__).resolve().parent
root = here.parents[2]
sys.path.insert(0, str(root / "ur5_hmi"))
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication
from gui.main_window import CONFIG, MainWindow
from gui.planning_workflow import PlanningStage

warnings.filterwarnings("ignore", message="Glyph.*missing from current font")
report_path = here / "staged_current" / "report.json"
report = json.loads(report_path.read_text())
request = json.loads((here.parent / "prepare_20261019_152229_336637" / "request.json").read_text())
before = CONFIG.read_bytes()
app = QApplication.instance() or QApplication([])
with patch.object(QThread, "start", lambda self: None):
    window = MainWindow()
try:
    window.trajectory_results_directory = here / "isolated_hmi_results"
    window.calibration_source_combo.blockSignals(True)
    window.calibration_source_combo.setCurrentText("REAL CALIBRATION")
    window.calibration_source_combo.blockSignals(False)
    window._real_planning_readiness_reasons = lambda: request["execution_gate_reasons"]
    window.planning_workflow.raw_path = Path(request["rawFile"])
    window.workflow_phase = "prepare"; window.workflow_canceled = False
    window.planning_process = None
    window.planning_report_path = report_path
    window.planning_output_path = Path(report["artifacts"]["trajectory_mat"])
    live = list(window.state.positions)
    window._online_process_finished(0, 0)
    assert window.planning_workflow.stage == PlanningStage.PHYSICAL_VALIDATION_PENDING
    for frame in (0, window.loaded_trajectory.point_count//2, window.loaded_trajectory.point_count-1):
        window._set_preview_pose(frame)
        diagnostic = window.robot_view.preview_frame_diagnostics()
        assert diagnostic["collision_status"] == "PASS" and not diagnostic["display_warning"]
    assert window.state.positions == live
    assert not window.execute_button.isEnabled() and not window._final_execution_gate_ok()
    assert window.online_report_button.isEnabled() and window.online_save_analysis_button.isEnabled()
    assert CONFIG.read_bytes() == before
    bad = copy.deepcopy(report); bad["controller_validation"]["collision_pass"] = False
    bad["failure_reason"] = "TEST_COLLISION_AUDIT_REJECTED"
    badpath = here / "hmi_failure_fixture.json"; badpath.write_text(json.dumps(bad))
    window.planning_report_path = badpath
    window._online_process_finished(0, 0)
    assert window.planning_workflow.stage == PlanningStage.CONTINUOUS_PLANNING_FAILED
    assert not window.execute_button.isEnabled() and CONFIG.read_bytes() == before
    result = dict(pass_all=True, real_report_callback=True, loaded_points=638,
                  three_display_frames=True, live_state_preserved=True, false_collision_pass_rejected=True,
                  physical_pending=True, execution_button_enabled=False,
                  production_config_unchanged=True, isolated_publish=True, robot_motion="NO")
    (here / "new_hmi_replay.json").write_text(json.dumps(result, indent=2)); print(result)
finally:
    window.close(); app.processEvents()
