from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Optional

import numpy as np

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from safety.joint_limits import JOINT_NAMES


@dataclass
class TrajectoryData:
    """Canonical internal trajectory representation.

    Positions and optional derivatives are radians/rad-based SI values.  The
    time vector is deliberately optional after import: a path without timing
    may be inspected and previewed point-by-point, but it is never executable.
    """

    joint_names: list[str]
    positions: np.ndarray
    time_from_start: Optional[np.ndarray] = None
    velocities: Optional[np.ndarray] = None
    accelerations: Optional[np.ndarray] = None
    # Optional Cartesian TCP samples in the same base frame as the planning
    # scene.  They are display/diagnostic data only and are never sent to ROS.
    tcp_path: Optional[np.ndarray] = None
    metadata: dict[str, Any] = field(default_factory=dict)
    source_path: str = ""

    @property
    def shape(self):
        return self.positions.shape

    @property
    def point_count(self) -> int:
        return int(self.positions.shape[0])

    @property
    def duration(self) -> Optional[float]:
        if self.time_from_start is None or len(self.time_from_start) == 0:
            return None
        return float(self.time_from_start[-1])

    def __len__(self):
        return self.point_count

    def __getitem__(self, item):
        return self.positions[item]

    def __array__(self, dtype=None):
        return np.asarray(self.positions, dtype=dtype)


def duration_for_jog(delta_rad: float, speed_percent: float) -> float:
    # Conservative first-version timing: 1 degree at 5% takes 3 seconds.
    speed = max(1.0, float(speed_percent))
    return max(2.0, abs(float(delta_rad)) / math.radians(1.0) * 3.0 * 5.0 / speed)


def make_two_point_trajectory(q0, q1, duration: float) -> JointTrajectory:
    msg = JointTrajectory()
    msg.joint_names = list(JOINT_NAMES)
    p0 = JointTrajectoryPoint()
    p0.positions = [float(v) for v in q0]
    p0.time_from_start.sec = 0
    p1 = JointTrajectoryPoint()
    p1.positions = [float(v) for v in q1]
    p1.time_from_start.sec = int(duration)
    p1.time_from_start.nanosec = int(round((duration - int(duration)) * 1e9))
    msg.points = [p0, p1]
    return msg


def _as_matrix(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.ndim == 1 and arr.size == 6:
        arr = arr.reshape(1, 6)
    if arr.ndim != 2 or arr.shape[1] != 6:
        raise ValueError(f"{name} must be N x 6, got shape {arr.shape}")
    return arr


def _as_vector(value, name: str, n: int) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != n:
        raise ValueError(f"{name} must contain {n} values, got {arr.size}")
    return arr


def _as_optional_matrix(value, name: str, n: int) -> Optional[np.ndarray]:
    if value is None:
        return None
    return _as_matrix(value, name)


def _as_optional_tcp_path(value, name: str, n: int) -> Optional[np.ndarray]:
    if value is None:
        return None
    arr = np.asarray(value, dtype=float)
    if arr.ndim == 1 and arr.size == 3:
        arr = arr.reshape(1, 3)
    # MATLAB callers occasionally store 3xN Cartesian samples.  Normalize
    # that representation once at the import boundary.
    if arr.ndim == 2 and arr.shape == (3, n):
        arr = arr.T
    if arr.ndim != 2 or arr.shape != (n, 3):
        raise ValueError(f"{name} must be N x 3, got shape {arr.shape}")
    return arr


def _string_list(value) -> list[str]:
    if value is None:
        return []
    arr = np.asarray(value, dtype=object).reshape(-1)
    result = []
    for item in arr:
        if isinstance(item, bytes):
            item = item.decode("utf-8", errors="replace")
        result.append(str(item).strip())
    return result


def _first_key(mapping: dict, keys: tuple[str, ...]):
    for key in keys:
        if key in mapping:
            return mapping[key]
    return None


def normalize_trajectory(data, source_path: str = "") -> TrajectoryData:
    """Normalize MAT/YAML/JSON payloads without inventing timing."""
    if isinstance(data, TrajectoryData):
        return data
    if isinstance(data, dict):
        positions = _first_key(data, ("qPath", "q_path", "qTraj", "positions", "trajectory", "path"))
        if positions is None:
            raise ValueError("未找到 qPath/q_path/qTraj/positions 轨迹数据")
        metadata = data.get("metadata", {})
        if not isinstance(metadata, dict):
            metadata = {}
        # MATLAB files often save these as top-level scalar variables rather
        # than a struct.  Preserve them without inventing any trajectory data.
        for key in ("source", "trajectory_name", "trajectoryName", "description"):
            if key in data and key not in metadata:
                value = data[key]
                if isinstance(value, np.ndarray) and value.size == 1:
                    value = value.reshape(-1)[0]
                if isinstance(value, bytes):
                    value = value.decode("utf-8", errors="replace")
                metadata[key] = str(value)
        names = _first_key(data, ("joint_names", "jointNames"))
        time_value = _first_key(data, ("timeFromStart", "time_from_start", "time", "t"))
        velocities = _first_key(data, ("velocities", "velocity"))
        accelerations = _first_key(data, ("accelerations", "acceleration"))
        tcp_path = _first_key(data, ("tcpPath", "tcp_path", "tcp_positions", "tcpTrajectory"))
    else:
        positions = data
        metadata = {}
        names = None
        time_value = None
        velocities = None
        accelerations = None
        tcp_path = None

    q = _as_matrix(positions, "positions")
    if not np.isfinite(q).all():
        raise ValueError("trajectory contains NaN/Inf")
    joint_names = _string_list(names) if names is not None else list(JOINT_NAMES)
    if not joint_names:
        joint_names = list(JOINT_NAMES)
    if joint_names != list(JOINT_NAMES):
        raise ValueError(
            "joint_names 必须严格为 " + ", ".join(JOINT_NAMES)
        )
    t = None if time_value is None else _as_vector(time_value, "time_from_start", q.shape[0])
    v = _as_optional_matrix(velocities, "velocities", q.shape[0])
    a = _as_optional_matrix(accelerations, "accelerations", q.shape[0])
    tcp = _as_optional_tcp_path(tcp_path, "tcpPath", q.shape[0])
    if v is not None and not np.isfinite(v).all():
        raise ValueError("velocities contains NaN/Inf")
    if a is not None and not np.isfinite(a).all():
        raise ValueError("accelerations contains NaN/Inf")
    meta = dict(metadata)
    meta.setdefault("source", source_path or "unknown")
    meta.setdefault("trajectory_name", meta.get("trajectoryName", meta.get("name", "未命名轨迹")))
    meta.setdefault("description", "")
    return TrajectoryData(
        joint_names=list(JOINT_NAMES),
        positions=q,
        time_from_start=t,
        velocities=v,
        accelerations=a,
        tcp_path=tcp,
        metadata=meta,
        source_path=source_path,
    )


def trajectory_to_ros_message(traj: TrajectoryData) -> JointTrajectory:
    """Convert a timed canonical trajectory to FollowJointTrajectory input."""
    if traj.time_from_start is None:
        raise ValueError("TIME PARAMETERIZATION REQUIRED")
    msg = JointTrajectory()
    msg.joint_names = list(JOINT_NAMES)
    for i, row in enumerate(traj.positions):
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in row]
        if traj.velocities is not None:
            point.velocities = [float(v) for v in traj.velocities[i]]
        if traj.accelerations is not None:
            point.accelerations = [float(v) for v in traj.accelerations[i]]
        seconds = float(traj.time_from_start[i])
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int(round((seconds - int(seconds)) * 1e9))
        if point.time_from_start.nanosec >= 1_000_000_000:
            point.time_from_start.sec += 1
            point.time_from_start.nanosec -= 1_000_000_000
        msg.points.append(point)
    return msg


def normalize_loaded_path(data):
    """Backward-compatible numeric N x 6 accessor."""
    return normalize_trajectory(data).positions
