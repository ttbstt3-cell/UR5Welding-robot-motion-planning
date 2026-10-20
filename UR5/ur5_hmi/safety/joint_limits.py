from __future__ import annotations

import math


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# The CB3 driver accepts continuous UR joint angles.  These broad limits are
# deliberately not a substitute for the robot's own safety limits.
LOWER = [-2.0 * math.pi] * 6
UPPER = [2.0 * math.pi] * 6


def within_limits(q) -> bool:
    return len(q) == 6 and all(lo <= float(v) <= hi for v, lo, hi in zip(q, LOWER, UPPER))
