"""Replay genuine post-fallback failure into real Qt HMI; no ROS/RTDE."""
import json
import os
from pathlib import Path
import sys
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication
from gui.main_window import CONFIG, MainWindow
from gui.planning_workflow import PlanningStage


def no_tcp_reads(self):
    return None


def run(report_file, output):
    report = json.loads(report_file.read_text())
    request = json.loads(report_file.with_name('request.json').read_text())
    assert report['success'] is False and report['hybrid_velocity_candidate']['candidate_accepted']
    raw = Path(request['rawFile'])
    index = raw.parent / '最新轨迹索引.json'
    original_config, original_index = CONFIG.read_bytes(), index.read_bytes()
    app = QApplication.instance() or QApplication([])
    with patch.object(QThread, 'start', lambda self: None), \
         patch.object(MainWindow, '_auto_read_robot_tcp', no_tcp_reads), \
         patch.object(MainWindow, 'showMaximized', lambda self: None):
        window = MainWindow()
    window.robot_tcp_auto_timer.stop()
    sends = []
    window.worker.request_trajectory.connect(sends.append)
    try:
        window.workflow_phase = 'prepare'; window.workflow_canceled = False
        window.planning_process = None
        window.planning_workflow.raw_path = raw
        window.planning_workflow.begin_preparation()
        window.planning_report_path = report_file
        window.planning_output_path = report_file.with_name('offline_optimized_trajectory.mat')
        window._online_process_finished(0, 0)
        assert window.planning_workflow.stage == PlanningStage.CONTINUOUS_PLANNING_FAILED
        assert report['failure_reason'] in window.online_status_label.text()
        assert 'Hybrid velocity generation failed' not in window.online_status_label.text()
        assert window.planning_workflow.raw_path == raw and window.planning_workflow.ready_path is None
        assert not window.execute_button.isEnabled() and not window.online_save_analysis_button.isEnabled()
        assert not sends
        assert CONFIG.read_bytes() == original_config and index.read_bytes() == original_index
        output.write_text(json.dumps(dict(pass_all=True, genuine_MATLAB_failure_replayed=True,
            displayed_reason=window.online_status_label.text(), preview_preserved=True,
            candidate_recovery_not_mistaken_for_full_task_pass=True,
            hardware_requests=0, executionReady=False, robot_motion='NO'), indent=2))
        print('REAL_QT_HYBRID_FALLBACK_FAILURE_PASS')
    finally:
        window.close(); app.processEvents()


if __name__ == '__main__':
    run(Path(sys.argv[1]), Path(sys.argv[2]))
