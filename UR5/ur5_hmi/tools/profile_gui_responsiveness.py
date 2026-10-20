"""Read-only GUI refresh profile with synthetic states; no ROS thread or send."""
import cProfile
import json
import os
from pathlib import Path
import pstats
import sys
import time
from unittest.mock import patch
import warnings

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication
from gui.main_window import MainWindow
from ros.state_monitor import RobotState


def run(source, output):
    warnings.filterwarnings("ignore", message="Glyph.*missing from current font")
    output.mkdir(parents=True, exist_ok=True)
    app = QApplication.instance() or QApplication([])
    with patch.object(QThread, "start", lambda self: None):
        window = MainWindow()
    sends = []
    window.worker.request_trajectory.connect(sends.append)
    try:
        window.calibration_source_combo.blockSignals(True)
        window.calibration_source_combo.setCurrentText("REAL CALIBRATION")
        window.calibration_source_combo.blockSignals(False)
        window._adopt_trajectory_file(source)
        window.planning_busy = True
        profiler = cProfile.Profile()
        durations = []
        profiler.enable()
        for _ in range(30):
            state = RobotState(positions=window.loaded_trajectory.positions[0].tolist(),
                joint_stamp=time.monotonic(), program_running=False, robot_mode=5,
                safety_mode=1, robot_link_alive=True)
            start = time.monotonic()
            window._on_state(state)
            durations.append(time.monotonic() - start)
        profiler.disable()
        with (output / "refresh_profile.txt").open("w") as handle:
            pstats.Stats(profiler, stream=handle).sort_stats("cumulative").print_stats(45)
        result = dict(state_updates=30, mean_update_s=sum(durations)/len(durations),
            max_update_s=max(durations), no_motion_signals=not sends,
            loaded_nodes=window.loaded_trajectory.point_count, source=str(source))
        if hasattr(window, "_refresh_state_display"):
            paints = []
            paint_profiler = cProfile.Profile()
            paint_profiler.enable()
            for _ in range(30):
                start = time.monotonic()
                window._refresh_state_display()
                paints.append(time.monotonic() - start)
            paint_profiler.disable()
            with (output / "display_refresh_profile.txt").open("w") as handle:
                pstats.Stats(paint_profiler, stream=handle).sort_stats("cumulative").print_stats(25)
            result["mean_display_refresh_s"] = sum(paints) / len(paints)
            result["max_display_refresh_s"] = max(paints)
        (output / "refresh_profile.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result))
    finally:
        window.close()


if __name__ == "__main__":
    run(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve())
