"""Read-only check of the actual retained latest trajectory/manual import UI."""
import json
import os
from pathlib import Path
import sys
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication, QFileDialog
from gui.main_window import CONFIG, MainWindow
from trajectory.result_store import latest_result, read_index
from trajectory.trajectory_loader import load_trajectory
from safety.execution_preparation import file_hash


def no_tcp_reads(self): return None


def run(output):
    before = CONFIG.read_bytes()
    app = QApplication.instance() or QApplication([])
    with patch.object(QThread, "start", lambda self: None), \
         patch.object(MainWindow, "_auto_read_robot_tcp", no_tcp_reads), \
         patch.object(MainWindow, "showMaximized", lambda self: None):
        win = MainWindow()
    sends = []; win.worker.request_trajectory.connect(sends.append)
    try:
        folder = win._trajectory_result_directory()
        latest = latest_result(folder)
        entry = read_index(folder)["preview"]
        original = Path(entry["source_path"])
        assert latest.is_file() and original.is_file()
        assert file_hash(latest) == file_hash(original)
        expected = load_trajectory(str(original))
        def select(*args):
            assert Path(args[2]) == latest, args
            return str(latest), ""
        with patch.object(QFileDialog, "getOpenFileName", side_effect=select):
            win.load_button.click()
        assert win.loaded_trajectory.point_count == expected.point_count
        assert win.loaded_path_name == str(latest)
        assert "仅预演" in win.path_label.text()
        assert win.planning_workflow.raw_path == latest
        assert win.execution_report_file == Path(entry["report"])
        assert win.trajectory_validation is None and not win.execute_button.isEnabled()
        assert not sends and CONFIG.read_bytes() == before
        result = dict(pass_all=True, latest_file=str(latest), node_count=expected.point_count,
            default_dialog_selects_latest=True, manual_import_resumes_preview_for_continuous_planning=True,
            byte_identical_to_original=True, named_report_associated=True,
            execution_disabled_until_checks_and_manual_confirmation=True,
            no_motion_signals=True, calibration_config_unchanged=True)
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2))
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        win.close()


if __name__ == "__main__": run(Path(sys.argv[1]).resolve())
