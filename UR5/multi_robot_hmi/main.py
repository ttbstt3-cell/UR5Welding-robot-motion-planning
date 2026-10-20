#!/usr/bin/env python3
"""Entry point for the multi-robot welding HMI.

Run from a ROS 2 sourced shell:

    source /opt/ros/humble/setup.bash
    python3 /home/ttbstt/jixiebi/UR5/multi_robot_hmi/main.py

The application is deliberately monitor-first.  It never sends a trajectory
on startup.  When launched through run_hmi.sh, the wrapper may activate the
configured controller only after the read-only robot safety gates are ready;
the Qt application itself never sends motion on startup.
"""

from __future__ import annotations

import sys
import os

try:
    from PyQt5.QtCore import Qt
    from PyQt5.QtWidgets import QApplication
except ImportError as exc:  # pragma: no cover - user-facing startup guard
    raise SystemExit("PyQt5 is required. Install python3-pyqt5.") from exc

from gui.main_window import MainWindow


def main() -> int:
    # Enable Qt's device-independent scaling before QApplication is created.
    # This is important on 2K/4K displays and does not change robot commands.
    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    app.setApplicationName("Multi-Robot Welding HMI")
    window = MainWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
