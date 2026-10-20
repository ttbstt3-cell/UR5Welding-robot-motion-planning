"""Offline numeric inspection only: no exports and no robot interfaces."""
import json
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "ur5_hmi"))
from trajectory.trajectory_loader import load_trajectory
from safety.robot_motion_limits import check_motion, LEGACY_V, LEGACY_A


def inspect(trajectory, scale):
    reasons, metrics = check_motion(trajectory.positions, trajectory.time_from_start * scale,
                                    trajectory.velocities / scale, execution=True)
    return dict(uniform_time_scale=scale, duration_sec=trajectory.duration * scale,
                max_velocity_rad_s=float(np.max(metrics["max_velocity_per_joint"])),
                max_acceleration_rad_s2=float(np.max(metrics["max_acceleration_per_joint"])),
                max_velocity_deg_s=float(np.rad2deg(np.max(metrics["max_velocity_per_joint"]))),
                max_acceleration_deg_s2=float(np.rad2deg(np.max(metrics["max_acceleration_per_joint"]))),
                software_motion_limit_reasons=reasons)


if __name__ == "__main__":
    entry = json.loads((ROOT / "规划文件/当前轨迹/最新轨迹索引.json").read_text())["prepare"]
    trajectory = load_trajectory(entry["path"])
    report = json.loads(Path(entry["report"]).read_text())
    result = dict(trajectory=entry["path"], report=entry["report"], points=trajectory.point_count,
        offline_complete=report["continuous_planning_complete"],
        executionReady=report["executionReady"], recorded_motion_limits=report["motion_limits"],
        legacy_compatibility=dict(velocity_rad_s=LEGACY_V, acceleration_rad_s2=LEGACY_A,
            note="Software compatibility range, not UR5 hardware capability or physical certification"),
        original=inspect(trajectory, 1), slower_numeric_only=inspect(trajectory, 2),
        slower_case_not_fully_validated=True, no_trajectory_exported=True,
        robot_motion="NO")
    (HERE / "trajectory_diagnosis.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print({key: result[key] for key in ("original", "slower_numeric_only", "robot_motion")})
