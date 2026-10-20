"""Read-only animation snapshot. Never changes live TCP or tool calibration."""
import hashlib
import json
from pathlib import Path
import numpy as np


class PreviewDisplayMixin:
    def set_preview_trajectory(self, trajectory):
        self._preview_trajectory = trajectory
        self._preview_scene_snapshot = None
        self._preview_snapshot_error = ""
        self._preview_environment_components = []
        self._preview_environment_error = ""
        self._preview_environment_coverage_warning = ""
        info = trajectory.metadata.get("preview_display", {})
        if not info:
            self._preview_snapshot_error = "缺少预览场景快照/逐帧碰撞审计"
            installer = getattr(self, "_install_preview_tool_actors", None)
            if callable(installer):
                installer()
            return
        try:
            path = Path(str(info["snapshot_file"]))
            content = path.read_bytes()
            if hashlib.sha256(content).hexdigest() != info["snapshot_sha256"]:
                raise ValueError("预览显示快照哈希不一致")
            data = json.loads(content)
            if data.get("frame_id") != "base":
                raise ValueError("预览快照坐标系不是 URDF base")
            from gui.robot_view import _validated_tcp_transform
            for name in ("T_flange_to_gun", "T_flange_to_mount", "T_flange_to_tcp",
                         "T_flange_to_wire", "wire_collision_shape_pose"):
                data[name] = _validated_tcp_transform(data[name])
            self._preview_scene_snapshot = data
            from gui.scene_geometry import environment_components, environment_coverage_warning
            self._preview_environment_coverage_warning = environment_coverage_warning(
                data.get("environment_geometry_load", {}).get("source_coverage"))
            if "environment" not in data:
                self._preview_environment_error = "预览快照缺少环境碰撞几何；不会用当前实时环境替代旧快照"
            else:
                self._preview_environment_components = environment_components(data["environment"], "base")
                if not self._preview_environment_components:
                    self._preview_environment_error = "预览快照未包含工件以外的环境碰撞几何"
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self._preview_snapshot_error = str(exc)
        installer = getattr(self, "_install_preview_tool_actors", None)
        if callable(installer):
            installer()

    def preview_frame_diagnostics(self, transforms=None):
        snapshot = getattr(self, "_preview_scene_snapshot", None)
        result = {"collision_status": "UNKNOWN", "collision_pairs": [],
                  "display_warning": getattr(self, "_preview_snapshot_error", "")}
        if snapshot is None or self._preview_q is None:
            return result
        transforms = transforms or self.model.link_transforms_in_base(self._preview_q)
        flange = transforms["flange"]
        gun = flange @ snapshot["T_flange_to_gun"]
        wire = flange @ snapshot["T_flange_to_wire"]
        tcp = flange @ snapshot["T_flange_to_tcp"]
        length = float(snapshot["wire_length_m"])
        start = (wire @ [0, 0, -length/2, 1])[:3]
        end = (wire @ [0, 0, length/2, 1])[:3]
        collision = wire @ snapshot["wire_collision_shape_pose"]
        cl = float(snapshot["wire_collision_length_m"])
        result.update(flange_pose=flange, gun_pose=gun, wire_pose=wire, tcp_pose=tcp,
                      gun_tip_world=start, wire_tip_world=end,
                      collision_start=(collision @ [0, 0, -cl/2, 1])[:3],
                      collision_end=(collision @ [0, 0, cl/2, 1])[:3],
                      T_flange_to_gun=snapshot["T_flange_to_gun"])
        traj = self._preview_trajectory
        i = int(self._preview_tcp_index or 0)
        records = traj.metadata.get("collisionResult", [])
        if isinstance(records, dict):
            records = [records]
        if i < len(records):
            record = records[i]
            result["collision_status"] = str(record.get("collision_status", "UNKNOWN"))
            def pairs(value):
                if isinstance(value, str): return [value] if value else []
                return [str(x) for x in np.asarray(value, dtype=object).reshape(-1)]
            result["collision_pairs"] = pairs(record.get("collision_pairs", [])) + pairs(record.get("outgoing_edge_pairs", []))
        # Display FK must agree with MATLAB, not merely look plausible.
        warnings = []
        for key, actual in (("flangePosePath", flange), ("gunPosePath", gun), ("wirePosePath", wire)):
            path = traj.metadata.get(key, [])
            if i < len(path):
                expected = np.asarray(path[i], dtype=float)
                pos = np.linalg.norm(actual[:3, 3]-expected[:3, 3])*1000
                rot = np.degrees(np.arccos(np.clip((np.trace(actual[:3,:3].T @ expected[:3,:3])-1)/2, -1, 1)))
                result[key+"_position_error_mm"] = float(pos)
                if pos > 0.001 or rot > 0.001:
                    warnings.append(f"{key} FK mismatch {pos:.6f} mm / {rot:.6f}°")
            else:
                warnings.append(f"{key} missing")
        if warnings:
            result["collision_status"] = "WARNING"
            result["display_warning"] = "; ".join(warnings)
        if not snapshot.get("collision_coverage_pass", False):
            result["collision_status"] = "PREVIEW WARNING"
            result["display_warning"] = "规划快照缺少 UR5 本体碰撞几何：" + ", ".join(snapshot.get("missing_robot_collision_bodies", []))
        if getattr(self, "_preview_snapshot_error", ""):
            result["collision_status"] = "WARNING"
            result["display_warning"] += "; " + self._preview_snapshot_error
        environment_error = getattr(self, "_preview_environment_error", "")
        if environment_error:
            result["collision_status"] = "PREVIEW WARNING"
            result["display_warning"] += "; " + environment_error
        coverage_warning = getattr(self, "_preview_environment_coverage_warning", "")
        if coverage_warning:
            # Source coverage is informational. A deferred fence must not
            # become an extra collision/planning gate for a floor-only scope.
            result["environment_source_coverage_note"] = coverage_warning
        result["environment_object_count"] = len(getattr(self, "_preview_environment_components", []))
        visual_pairs = getattr(self, "_preview_visual_pair_cache", {}).get(i, [])
        result["visual_surface_pairs"] = visual_pairs
        if visual_pairs:
            result["collision_status"] = "WARNING"
            result["collision_pairs"] = result["collision_pairs"] + visual_pairs
            result["display_warning"] += "; STL 表面交叉诊断，不是规划器实体碰撞认证"
        return result

    def _preview_tcp_from_snapshot(self):
        if getattr(self, "_preview_scene_snapshot", None) is None:
            return self._fk_tcp_from_flange(self._preview_q)[1]
        return self.preview_frame_diagnostics()["tcp_pose"][:3, 3].copy()
