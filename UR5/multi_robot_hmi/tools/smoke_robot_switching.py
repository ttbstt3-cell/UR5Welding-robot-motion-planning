"""Offline profile-switch and trajectory-provenance smoke test.

Runs the real Qt profile/view code with the ROS worker replaced by a no-op. It
does not initialize ROS, launch MATLAB, create an action client, or command a
robot.
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch

import numpy as np
from scipy.io import savemat
os.environ["QT_QPA_PLATFORM"] = "offscreen"
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from PyQt5.QtWidgets import QApplication
from gui.main_window import MainWindow


app = QApplication.instance() or QApplication(sys.argv)
checks = []


def check(label, condition):
    checks.append((label, bool(condition)))
    print(f"[{'PASS' if condition else 'FAIL'}] {label}")


def write_traj(path, robot_id, joint_names):
    path.write_text(json.dumps({
        "robot_id": robot_id,
        "joint_names": joint_names,
        "positions": [[0.0] * 6],
        "trajectory_name": f"{robot_id}_preview",
    }), encoding="utf-8")


with patch.object(MainWindow, "_start_ros_worker", lambda _self: None):
    win = MainWindow()
    win.show()
    app.processEvents()

    check("selector exposes UR5 and CR7",
          [win.robot_select_combo.itemData(i) for i in range(win.robot_select_combo.count())]
          == ["UR5", "CR7"])
    check("UR5 model selected initially",
          win.current_profile.robot_id == "UR5"
          and win.robot_view.model.revolute_names == list(win.current_profile.joint_names))

    win.planning_busy = True
    win.robot_select_combo.setCurrentIndex(win.robot_select_combo.findData("CR7"))
    check("switch blocked while planning_busy", win.current_profile.robot_id == "UR5")
    win.planning_busy = False

    cr7_index = win.robot_select_combo.findData("CR7")
    win.robot_select_combo.setCurrentIndex(cr7_index)
    app.processEvents()
    check("CR7 profile and 3D model loaded",
          win.current_profile.robot_id == "CR7"
          and win.robot_view.model.revolute_names == list(win.current_profile.joint_names))
    check("CR7 has no real controller/action",
          not win.current_profile.supports_ros_control
          and not win.current_profile.controller_action
          and not win.execute_button.isEnabled())
    check("CR7 HMI welding planner stays blocked", not win.online_start_button.isEnabled())

    with tempfile.TemporaryDirectory(prefix="robot-switch-smoke-") as tmp:
        tmp = Path(tmp)
        ur5_path = tmp / "ur5.json"
        cr7_path = tmp / "cr7.json"
        write_traj(ur5_path, "UR5", win.robot_profiles["UR5"].joint_names)
        write_traj(cr7_path, "CR7", win.robot_profiles["CR7"].joint_names)

        try:
            win._adopt_trajectory_file(ur5_path)
        except ValueError:
            pass
        check("UR5 trajectory blocked on CR7", win.loaded_trajectory is None
              and "robot_id=UR5" in win.trajectory_block_reason)
        check("Final Execution Check displays BLOCKED",
              "BLOCKED" in win.final_gate_label.text())

        win._adopt_trajectory_file(cr7_path)
        check("matching CR7 preview trajectory loads",
              win.loaded_trajectory is not None
              and win.loaded_trajectory.robot_id == "CR7")
        check("CR7 trajectory still cannot execute", not win.execute_button.isEnabled())

        cr7_mat = tmp / "cr7.mat"
        savemat(cr7_mat, {
            "robot_id": "CR7",
            "joint_names": np.asarray(win.robot_profiles["CR7"].joint_names, dtype=object),
            "qPath": np.zeros((2, 6)),
            "metadata": {"robot_id": "CR7", "source": "offline smoke"},
        })
        win._adopt_trajectory_file(cr7_mat)
        check("MAT metadata robot_id is preserved", win.loaded_trajectory is not None
              and win.loaded_trajectory.robot_id == "CR7")

        win.preview_active = True
        win.preview_timer.start()
        win.robot_select_combo.setCurrentIndex(win.robot_select_combo.findData("UR5"))
        app.processEvents()
        check("switch stops preview and unloads trajectory",
              win.current_profile.robot_id == "UR5"
              and not win.preview_active
              and win.loaded_trajectory is None)
        try:
            win._adopt_trajectory_file(cr7_path)
        except ValueError:
            pass
        check("CR7 trajectory blocked on UR5", win.loaded_trajectory is None
              and "robot_id=CR7" in win.trajectory_block_reason)

    check("profile data paths are distinct",
          win.robot_profiles["UR5"].real_home != win.robot_profiles["CR7"].real_home
          and win.robot_profiles["UR5"].tcp_calibration != win.robot_profiles["CR7"].tcp_calibration
          and win.robot_profiles["UR5"].workpiece_pose_file != win.robot_profiles["CR7"].workpiece_pose_file
          and win.robot_profiles["UR5"].planning_runs_dir != win.robot_profiles["CR7"].planning_runs_dir
          and win.robot_profiles["UR5"].trajectory_dir != win.robot_profiles["CR7"].trajectory_dir)

    win.close()
    app.processEvents()

passed = sum(ok for _, ok in checks)
print(f"RESULT: {passed}/{len(checks)} checks passed; no ROS, MATLAB, or robot motion")
sys.stdout.flush()
os._exit(0 if passed == len(checks) else 1)
