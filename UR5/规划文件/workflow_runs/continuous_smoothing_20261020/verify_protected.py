"""Record protected production inputs; never modifies them or publishes a trajectory."""
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PRIOR = HERE.parent / "continuous_refactor_20261019/protected_verification.json"
EXTRA = [
    "UR5/matlab仿真/prepare_hmi_continuous_request.m",
    "UR5/matlab仿真/hmi_continuous_geometry.m",
    "UR5/matlab仿真/hmi_continuous_acceptance.m",
    "UR5/matlab仿真/hmi_continuous_reference.m",
    "UR5/matlab仿真/hmi_continuous_pose_at_s.m",
    "UR5/matlab仿真/hmi_export_offline_continuous_result.m",
    "UR5/matlab仿真/check_scene_environment_collision.m",
    "RRT/checkEnvironmentCollision.m",
    "UR5/matlab仿真/accp_cubic_derivative_metrics.m",
]


def snapshot():
    paths = list(json.loads(PRIOR.read_text())["after"]) + EXTRA
    return {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in paths}


if __name__ == "__main__":
    current = snapshot()
    if sys.argv[1] == "before":
        result = current
        destination = HERE / "protected_before.json"
    else:
        before = json.loads((HERE / "protected_before.json").read_text())
        changes = [p for p in before if before[p] != current[p]]
        result = {"unchanged_all": not changes, "checked_files": len(before),
                  "changes": changes, "before": before, "after": current}
        destination = HERE / "protected_verification.json"
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(destination)
    if sys.argv[1] != "before":
        assert not changes, changes
