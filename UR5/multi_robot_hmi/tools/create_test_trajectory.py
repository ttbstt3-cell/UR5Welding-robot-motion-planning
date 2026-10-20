#!/usr/bin/env python3
"""Create a conservative 5-point MAT trajectory for HMI software testing.

This file never connects to ROS2 and never sends motion.  Pass the real
current joint angles in degrees with ``--q-deg`` before attempting validation;
the HMI start gate will reject a trajectory whose first point is not within
1 degree of live feedback.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from scipy.io import savemat


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("hmi_test_trajectory_5pt.mat"))
    parser.add_argument(
        "--q-deg",
        type=float,
        nargs=6,
        default=[0.0] * 6,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="current/live starting joints in degrees; default is all zero for preview testing",
    )
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()
    q0 = np.deg2rad(np.asarray(args.q_deg, dtype=float))
    offsets_deg = np.array([
        [0, 0, 0, 0, 0, 0],
        [0.5, 0, 0, 0, 0, 0],
        [0.5, 0, 0, 0, 0.5, 0],
        [0.5, 0, 0, 0, -0.5, 0],
        [0, 0, 0, 0, 0, 0],
    ])
    q_path = q0[None, :] + np.deg2rad(offsets_deg)
    times = np.linspace(0.0, float(args.duration), q_path.shape[0])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    savemat(args.output, {
        "qPath": q_path,
        "timeFromStart": times,
        "joint_names": np.asarray([
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        ], dtype=object),
        "trajectory_name": "HMI 5-point low-speed test",
        "description": "Software preview and final-gate test only; max joint offset 0.5 deg.",
        "source": "create_test_trajectory.py",
    })
    print(f"Wrote {args.output}")
    print(f"Start deg: {np.asarray(args.q_deg)}")
    print(f"Duration: {args.duration:.3f} s; points: {q_path.shape[0]}")


if __name__ == "__main__":
    main()
