"""Offline regression checks for the fixed-point TCP solver.

This uses synthetic flange poses only. It does not import ROS, connect to the
robot, or send a motion command.
"""
from __future__ import annotations

import numpy as np

from calibration.tcp_pivot import PivotCalibrationError, fit_pivot_calibration, verify_pivot_sample


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    C = 1.0 - c
    return np.array([
        [x*x*C+c, x*y*C-z*s, x*z*C+y*s],
        [y*x*C+z*s, y*y*C+c, y*z*C-x*s],
        [z*x*C-y*s, z*y*C+x*s, z*z*C+c],
    ])


def main():
    tcp = np.array([0.10, -0.02, 0.30])
    pivot = np.array([0.40, 0.20, 0.50])
    poses = []
    for axis, angle in [([1, 0, 0], .2), ([0, 1, 0], .4), ([0, 0, 1], .5),
                        ([1, 1, 0], .8), ([1, 0, 1], .7)]:
        T = np.eye(4)
        T[:3, :3] = rotation(axis, angle)
        T[:3, 3] = pivot - T[:3, :3] @ tcp
        poses.append(T)
    result = fit_pivot_calibration(poses)
    assert np.allclose(result.tcp_flange_m, tcp, atol=1e-10)
    assert np.allclose(result.pivot_base_m, pivot, atol=1e-10)
    assert result.matrix_rank == 6 and result.valid
    assert verify_pivot_sample(poses[-1], result) < 1e-10
    try:
        fit_pivot_calibration([poses[0], poses[0], poses[0], poses[0]])
    except PivotCalibrationError:
        pass
    else:
        raise AssertionError("duplicate poses must be rejected")
    print("TCP pivot solver: PASS")


if __name__ == "__main__":
    main()
