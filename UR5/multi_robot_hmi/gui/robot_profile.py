from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


HMI_ROOT = Path(__file__).resolve().parents[1]
PROFILE_DIR = HMI_ROOT / "config" / "robots"


@dataclass(frozen=True)
class RobotProfile:
    """Robot-specific HMI configuration and isolated data locations."""

    robot_id: str
    display_name: str
    joint_names: tuple[str, ...]
    joint_limits: tuple[tuple[float, float], ...]
    robot_description: Path
    controller_action: str
    controller_name: str
    real_home: Path
    tcp_calibration: Path
    planning_model: str
    collision_scene_file: Path | None
    scene_geometry_file: Path | None
    workpiece_pose_file: Path
    planning_runs_dir: Path
    trajectory_dir: Path
    log_dir: Path
    ros: dict[str, Any]
    joint_display_names: tuple[str, ...]
    supports_real_robot: bool
    supports_ros_control: bool
    supports_online_planning: bool
    matlab_model_root: Path
    matlab_scene_script: Path | None
    matlab_planning_entry: Path | None
    matlab_anchor_file: Path | None

    @classmethod
    def from_file(cls, path: Path | str) -> "RobotProfile":
        profile_path = Path(path).expanduser().resolve()
        data = yaml.safe_load(profile_path.read_text(encoding="utf-8")) or {}
        root = profile_path.parents[2]

        def resolve(value, *, optional=False):
            if value in (None, ""):
                return None if optional else root
            result = Path(str(value)).expanduser()
            return (result if result.is_absolute() else root / result).resolve()

        joints = tuple(str(v) for v in data["joint_names"])
        limits = tuple((float(pair[0]), float(pair[1])) for pair in data["joint_limits"])
        names = tuple(str(v) for v in data.get("joint_display_names", joints))
        if not joints or len(joints) != len(limits) or len(names) != len(joints):
            raise ValueError(f"{profile_path}: joint_names/display names/limits mismatch")
        return cls(
            robot_id=str(data["robot_id"]),
            display_name=str(data["display_name"]),
            joint_names=joints,
            joint_limits=limits,
            robot_description=resolve(data["robot_description"]),
            controller_action=str(data.get("controller_action") or ""),
            controller_name=str(data.get("controller_name") or ""),
            real_home=resolve(data["real_home"]),
            tcp_calibration=resolve(data["tcp_calibration"]),
            planning_model=str(data["planning_model"]),
            collision_scene_file=resolve(data.get("collision_scene_file"), optional=True),
            scene_geometry_file=resolve(data.get("scene_geometry_file"), optional=True),
            workpiece_pose_file=resolve(data["workpiece_pose_file"]),
            planning_runs_dir=resolve(data["planning_runs_dir"]),
            trajectory_dir=resolve(data["trajectory_dir"]),
            log_dir=resolve(data["log_dir"]),
            ros=dict(data.get("ros", {})),
            joint_display_names=names,
            supports_real_robot=bool(data.get("supports_real_robot", False)),
            supports_ros_control=bool(data.get("supports_ros_control", False)),
            supports_online_planning=bool(data.get("supports_online_planning", False)),
            matlab_model_root=resolve(data.get("matlab_model_root")),
            matlab_scene_script=resolve(data.get("matlab_scene_script"), optional=True),
            matlab_planning_entry=resolve(data.get("matlab_planning_entry"), optional=True),
            matlab_anchor_file=resolve(data.get("matlab_anchor_file"), optional=True),
        )


def load_robot_profiles(directory: Path | str = PROFILE_DIR) -> dict[str, RobotProfile]:
    profiles = {}
    for path in sorted(Path(directory).glob("*.yaml")):
        profile = RobotProfile.from_file(path)
        profiles[profile.robot_id] = profile
    if not profiles:
        raise RuntimeError(f"No robot profiles found in {directory}")
    return profiles


def check_trajectory_robot_id(trajectory, current_robot_id: str) -> None:
    """Fail closed when a trajectory lacks provenance or belongs to another robot."""
    robot_id = getattr(trajectory, "robot_id", None)
    if not robot_id:
        robot_id = (getattr(trajectory, "metadata", {}) or {}).get("robot_id")
    if str(robot_id or "") != str(current_robot_id):
        raise ValueError(
            f"BLOCKED: trajectory.robot_id={robot_id or 'MISSING'} "
            f"does not match current_robot_id={current_robot_id}"
        )
