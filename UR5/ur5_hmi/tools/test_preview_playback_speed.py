"""Offline Qt playback-clock regression. No ROS worker, TCP reads or motion."""
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch
import warnings

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_preview_speed_mpl")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication
from gui.main_window import CONFIG, MainWindow, PREVIEW_SPEED_OPTIONS
from safety.joint_limits import JOINT_NAMES
from trajectory.trajectory_utils import TrajectoryData, trajectory_to_ros_message


def no_tcp_reads(self):
    return None


def run(output):
    warnings.filterwarnings("ignore", message="Glyph.*missing from current font")
    config_before = CONFIG.read_bytes()
    app = QApplication.instance() or QApplication([])
    with patch.object(QThread, "start", lambda self: None), \
         patch.object(MainWindow, "_auto_read_robot_tcp", no_tcp_reads), \
         patch.object(MainWindow, "showMaximized", lambda self: None):
        win = MainWindow()
    win.robot_tcp_auto_timer.stop()
    motion_signals = []
    speed_signals = []
    win.worker.request_trajectory.connect(motion_signals.append)
    win.worker.request_speed_percent.connect(speed_signals.append)
    try:
        assert win.preview_speed_combo.currentText() == win.online_preview_speed_combo.currentText() == "5x"
        assert tuple(win.preview_speed_combo.itemText(i) for i in range(win.preview_speed_combo.count())) == PREVIEW_SPEED_OPTIONS
        trajectory = TrajectoryData(joint_names=list(JOINT_NAMES),
            positions=np.zeros((2001, 6)), time_from_start=np.linspace(0, 200, 2001),
            velocities=np.zeros((2001, 6)), metadata={"diagnostic_only": True, "executionReady": False})
        win.loaded_trajectory = trajectory
        win.preview_slider.setRange(0, trajectory.point_count - 1)
        win.online_preview_slider.setRange(0, trajectory.point_count - 1)
        win.robot_view.set_preview_trajectory(trajectory)
        original = trajectory_to_ros_message(trajectory)
        positions = trajectory.positions.copy()
        stamps = trajectory.time_from_start.copy()
        velocities = trajectory.velocities.copy()
        live = win.state.positions
        real_speed = win.real_speed_spin.value()
        now = [100.0]
        with patch("gui.main_window.time.monotonic", side_effect=lambda: now[0]):
            win._start_preview()
            now[0] += 1
            win._preview_tick()
            assert win.preview_index == 50, win.preview_index  # 5 trajectory seconds.
            win.preview_speed_combo.setCurrentText("10x")
            win._preview_tick()
            assert win.preview_index == 50 and win.online_preview_speed_combo.currentText() == "10x"
            now[0] += 1
            win._preview_tick()
            assert win.preview_index == 150
            win.online_preview_speed_combo.setCurrentText("2x")
            win._preview_tick()
            assert win.preview_index == 150 and win.preview_speed_combo.currentText() == "2x"
            now[0] += 1
            win._preview_tick()
            assert win.preview_index == 170
            win._pause_preview()
            now[0] += 50
            win.online_preview_speed_combo.setCurrentText("20x")
            assert win.preview_index == 170 and not win.preview_timer.isActive()
            win._start_preview()
            now[0] += .5
            win._preview_tick()
            assert abs(win.preview_index - 270) <= 1
            win.preview_slider.setValue(500)
            win._preview_tick()
            assert abs(win.preview_index - 500) <= 1
            now[0] += 1
            win._preview_tick()
            assert abs(win.preview_index - 700) <= 1
            win._stop_preview()
            assert win.preview_index == 0 and not win.preview_active
            # A stopped/restarted animation uses the selected speed, not an old clock.
            win._start_preview()
            now[0] += 11
            win._preview_tick()
            assert win.preview_index == 2000 and not win.preview_timer.isActive()
            assert "完成" in win.preview_status_label.text()
            win._start_preview()
            assert win.preview_index == 0
            win._stop_preview()
            # During robot motion the existing preview lock must still apply.
            win.trajectory_client = SimpleNamespace(busy=True)
            win._update_buttons()
            assert not win.preview_speed_combo.isEnabled() and not win.online_preview_speed_combo.isEnabled()
            win._start_preview()
            assert not win.preview_active
            win.trajectory_client = None
        assert win.state.positions is live
        assert np.array_equal(positions, trajectory.positions)
        assert np.array_equal(stamps, trajectory.time_from_start)
        assert np.array_equal(velocities, trajectory.velocities)
        assert trajectory_to_ros_message(trajectory) == original
        assert win.real_speed_spin.value() == real_speed
        assert not motion_signals and not speed_signals
        assert CONFIG.read_bytes() == config_before
        result = dict(pass_all=True, default_preview_speed="5x", choices=list(PREVIEW_SPEED_OPTIONS),
            both_tabs_synchronized=True, no_jump_when_changing_rate=True,
            pause_resume_seek_stop_complete_pass=True, motion_lock_preserved=True,
            trajectory_and_ros_message_unchanged=True, live_state_and_robot_speed_unchanged=True,
            hardware_signals=0, robot_motion="NO")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        win.close()
        app.processEvents()


if __name__ == "__main__":
    run(Path(sys.argv[1]).resolve())
