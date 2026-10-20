"""Output-only controller TCP mirror regression; no ROS node or robot I/O.

Run from the project root after sourcing /opt/ros/humble/setup.bash. Synthetic
same-packet RTDE poses exercise fixed 6D frame conversion; GUI mixins are called
on lightweight fakes, not on a running HMI or a connected robot.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import time
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

HMI_ROOT = Path(__file__).resolve().parents[1]
PROJECT_UR5 = HMI_ROOT.parent
sys.path.insert(0, str(HMI_ROOT))

from calibration.controller_tcp_sync import (
    AUTHORITY, fixed_flange_tcp, make_controller_tcp_record,
    sync_controller_tcp_record, validate_controller_tcp_record,
)
from calibration.tcp_frames import transform_to_ur_pose, ur_pose_to_transform
from safety.joint_limits import JOINT_NAMES


class SyntheticModel:
    """Ten deliberately different flange poses with a nontrivial fixed tool0."""
    revolute_names = list(JOINT_NAMES)

    def __init__(self):
        self.flange_tool0 = ur_pose_to_transform([.004, -.007, .012, .3, -.2, .4])
        self.poses = [ur_pose_to_transform([
            -.23 + .021 * i, .19 - .009 * i, .4 + .005 * i,
            .07 + .04 * i, -.11 + .017 * i, .13 - .021 * i,
        ]) for i in range(10)]

    def link_transforms_in_base(self, q):
        flange = self.poses[int(q[0])].copy()
        return {"flange": flange, "tool0": flange @ self.flange_tool0}

    def snapshot(self, index=0, offset=None):
        offset = [.01353, .01713, .37983, -.9562, 2.4566, 1.2418] if offset is None else offset
        q = [index, 0, -.4, .7, -.3, .1]
        actual = self.link_transforms_in_base(q)["tool0"] @ ur_pose_to_transform(offset)
        return {"frame_id": "base", "actual_q": q,
                "actual_qd": [0] * 6, "actual_TCP_pose": transform_to_ur_pose(actual).tolist(),
                "tcp_offset": list(offset), "timestamp": 1000.0 + index,
                "robot_mode": 7, "safety_mode": 1, "runtime_state": 1}


class ControllerMirrorTests(unittest.TestCase):
    def setUp(self):
        self.model = SyntheticModel()
        self.urdf_hash = "a" * 64

    def test_ten_poses_use_same_fixed_full_6d_transform(self):
        reference = self.model.flange_tool0 @ ur_pose_to_transform(self.model.snapshot()["tcp_offset"])
        versions = set()
        for index in range(10):
            sample = self.model.snapshot(index)
            record = make_controller_tcp_record(self.model, sample, self.urdf_hash)
            np.testing.assert_allclose(record["T_flange_tcp"], reference, atol=1e-12)
            np.testing.assert_allclose(fixed_flange_tcp(self.model, sample["tcp_offset"]), reference, atol=1e-12)
            np.testing.assert_allclose(validate_controller_tcp_record(record, self.model, self.urdf_hash), reference, atol=1e-12)
            np.testing.assert_allclose(ur_pose_to_transform([
                *record["position_flange_tcp_m"], *record["rotation_flange_tcp_rotvec_rad"]]), reference, atol=1e-12)
            self.assertEqual(record["tcp_authority"], AUTHORITY)
            self.assertEqual(record["quality_level"], "CONTROLLER_DEFINED")
            self.assertEqual(record["physical_accuracy_status"], "NOT_MEASURED_BY_HMI")
            self.assertNotIn("fit", record)  # Never invent a pivot RMS residual.
            versions.add(record["version"])
        self.assertEqual(len(versions), 1)

    def test_pose_motion_and_bounded_read_jitter_do_not_churn_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first, changed = sync_controller_tcp_record(root, self.model, self.model.snapshot(), self.urdf_hash)
            self.assertTrue(changed)
            path = root / "real_tcp_calibration_active.json"
            raw = path.read_bytes()
            before = hashlib.sha256(raw).hexdigest()
            modified_ns = path.stat().st_mtime_ns
            for index in range(10):
                sample = self.model.snapshot(index)
                if index == 9:
                    sample = self.model.snapshot(index, np.asarray(sample["tcp_offset"]) + 1e-9)
                record, changed = sync_controller_tcp_record(root, self.model, sample, self.urdf_hash)
                self.assertFalse(changed)
                self.assertEqual(record, first)
                self.assertEqual(path.read_bytes(), raw)
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), before)
                self.assertEqual(path.stat().st_mtime_ns, modified_ns)
            self.assertEqual(len(list((root / "real_tcp_calibration_versions").glob("*.json"))), 1)

    def test_position_and_orientation_changes_archive_and_reversion(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            # Legacy and frozen records must survive unchanged.
            legacy = root / "real_tcp_calibration_applied.json"
            frozen = root / "ur5_tool_calibration.mat"
            legacy.write_bytes(b'{"source":"four-point historical"}\n')
            frozen.write_bytes(b"frozen simulation fixture\x00")
            active = root / "real_tcp_calibration_active.json"
            one, _ = sync_controller_tcp_record(root, self.model, self.model.snapshot(), self.urdf_hash)
            one_raw = active.read_bytes()
            position_offset = np.asarray(self.model.snapshot()["tcp_offset"]) + [.002, 0, 0, 0, 0, 0]
            two, changed = sync_controller_tcp_record(root, self.model, self.model.snapshot(1, position_offset), self.urdf_hash)
            self.assertTrue(changed)
            self.assertNotEqual(one["version"], two["version"])
            two_raw = active.read_bytes()
            orientation_offset = position_offset + [0, 0, 0, .02, -.01, .03]
            three, changed = sync_controller_tcp_record(root, self.model, self.model.snapshot(2, orientation_offset), self.urdf_hash)
            self.assertTrue(changed)
            self.assertNotEqual(two["version"], three["version"])
            archives = [p.read_bytes() for p in (root / "real_tcp_calibration_versions").glob("before_auto_sync_*.json")]
            self.assertIn(one_raw, archives)
            self.assertIn(two_raw, archives)
            self.assertEqual(len(archives), 2)
            self.assertEqual(legacy.read_bytes(), b'{"source":"four-point historical"}\n')
            self.assertEqual(frozen.read_bytes(), b"frozen simulation fixture\x00")

    def test_invalid_samples_never_replace_active(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            sync_controller_tcp_record(root, self.model, self.model.snapshot(), self.urdf_hash)
            active = root / "real_tcp_calibration_active.json"
            raw = active.read_bytes()
            samples = []
            for field in ("tcp_offset", "actual_TCP_pose", "actual_q"):
                for value in (float("nan"), float("inf")):
                    sample = self.model.snapshot()
                    sample[field][0] = value
                    samples.append(sample)
            sample = self.model.snapshot(); sample["frame_id"] = "base_link"; samples.append(sample)
            sample = self.model.snapshot(); sample["actual_TCP_pose"][0] += .01; samples.append(sample)
            sample = self.model.snapshot(); sample["actual_TCP_pose"][3] += .1; samples.append(sample)
            for sample in samples:
                with self.subTest(sample=sample), self.assertRaises(ValueError):
                    sync_controller_tcp_record(root, self.model, sample, self.urdf_hash)
                self.assertEqual(active.read_bytes(), raw)

    def test_invalid_saved_active_is_not_trusted_or_legacy_fallback(self):
        base = make_controller_tcp_record(self.model, self.model.snapshot(), self.urdf_hash)
        variants = []
        for key, value in (("tcp_authority", "HMI_PIVOT"), ("valid", False), ("status", "DRAFT"),
                           ("urdf_sha256", "b" * 64)):
            record = deepcopy(base); record[key] = value; variants.append(record)
        record = deepcopy(base); record["coordinate_frames"]["flange"] = "tool0"; variants.append(record)
        record = deepcopy(base); record["T_flange_tcp"][0][3] += .001; variants.append(record)
        record = deepcopy(base); record["T_flange_tcp"][0][0] *= 2; variants.append(record)
        record = deepcopy(base); record["position_flange_tcp_m"][0] += .001; variants.append(record)
        for record in variants:
            with self.subTest(record=record), self.assertRaises(ValueError):
                validate_controller_tcp_record(record, self.model, self.urdf_hash)


class FakeLabel:
    def __init__(self):
        self.text = ""

    def setText(self, text):
        self.text = text


class FakeView:
    def __init__(self):
        self.planning_transform = None
        self.live_position = None
        self.live_q = None

    def set_planning_tcp_transform(self, transform):
        self.planning_transform = np.asarray(transform).copy()

    def set_controller_tcp_pose(self, position, q, _frame):
        self.live_position = np.asarray(position).copy()
        self.live_q = np.asarray(q).copy()


class FakeProcess:
    def __init__(self, payload):
        self.payload = payload
        self.deleted = False

    def readAllStandardOutput(self):
        return json.dumps(self.payload).encode()

    def readAllStandardError(self):
        return b""

    def deleteLater(self):
        self.deleted = True


class ControllerGuiOrchestrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import gui.tcp_robot_panel as panel_module
            import gui.main_window as main_module
        except ModuleNotFoundError as exc:
            raise unittest.SkipTest("Source ROS2 environment for GUI mixin imports: " + str(exc))
        cls.panel_module = panel_module
        cls.main_module = main_module

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.model = SyntheticModel()
        self.urdf_hash = hashlib.sha256(self.panel_module.URDF_PATH.read_bytes()).hexdigest()
        self.labels = {name: FakeLabel() for name in ("state", "current", "alignment")}
        self.logs = []
        self.invalidated = []
        self.panel = SimpleNamespace(
            robot_tcp_snapshot=self.model.snapshot(), robot_tcp_received=time.monotonic(),
            robot_tcp_sync_error="", robot_tcp_sync_stamp=0.0,
            robot_tcp_busy=False, robot_tcp_mode="read", robot_tcp_report_path=None,
            robot_tcp_auto_sync=True, robot_tcp_auto_last_error="",
            goal_active=False, _jog_pending=False, planning_busy=False,
            tcp_fk_model=self.model, tcp_fk_sha256=self.urdf_hash,
            tcp_calibration_active_file=self.root / "real_tcp_calibration_active.json",
            tcp_calibration_applied_file=self.root / "real_tcp_calibration_applied.json",
            calibration_file=self.root / "workpiece_pose_real.json",
            calibration_source_combo=SimpleNamespace(currentText=lambda: "REAL CALIBRATION"),
            robot_view=FakeView(), robot_tcp_labels=self.labels,
            _motion_busy=lambda: False,
            _invalidate_tcp_trajectories=lambda: self.invalidated.append("STALE"),
            _log=self.logs.append, _refresh_online_goal_display=lambda: None,
            _refresh_scene_status_label=lambda: None, _update_robot_tcp_controls=lambda: None,
            _update_buttons=lambda: None,
            _launch_robot_tcp_helper=lambda *args: self.fail("Tests must never launch a helper or writer"),
            _format_tcp6=self.panel_module.TcpRobotPanel._format_tcp6,
            _file_sha256=self.main_module.MainWindow._file_sha256,
        )
        self.panel._sync_controller_tcp_to_planning = MethodType(
            self.panel_module.TcpRobotPanel._sync_controller_tcp_to_planning, self.panel)
        self.addCleanup(patch.stopall)
        patch.object(self.panel_module, "HMI_ROOT", self.root / "ur5_hmi").start()
        patch.object(self.main_module, "ROOT", self.root / "ur5_hmi").start()
        (self.root / "real_robot_description_calibrated.urdf").write_bytes(
            self.panel_module.URDF_PATH.read_bytes())
        self.panel.calibration_file.write_text(json.dumps({
            "valid": True, "geometry_valid": True, "quality_level": "GOOD", "residual_mm": 1.0,
        }))
        gun_dir = self.root / "real_gun_geometry"
        gun_dir.mkdir()
        (gun_dir / "real_gun_geometry.json").write_bytes(
            (PROJECT_UR5 / "real_gun_geometry/real_gun_geometry.json").read_bytes())

    def _finish_read(self, payload, code=0):
        self.panel.robot_tcp_process = FakeProcess(payload)
        self.panel.robot_tcp_busy = True
        self.panel_module.TcpRobotPanel._robot_tcp_finished(
            self.panel, code, self.panel_module.QProcess.NormalExit)

    def _readiness(self):
        return self.main_module.MainWindow._real_planning_readiness_reasons(self.panel)

    def test_fresh_read_automatically_mirrors_full_pose_without_robot_write(self):
        sample = self.model.snapshot()
        self._finish_read({"success": True, "snapshot": sample})
        record = json.loads(self.panel.tcp_calibration_active_file.read_text())
        self.assertEqual(record["tcp_authority"], AUTHORITY)
        np.testing.assert_allclose(self.panel.robot_view.planning_transform, record["T_flange_tcp"])
        np.testing.assert_array_equal(self.panel.robot_view.live_position, sample["actual_TCP_pose"][:3])
        np.testing.assert_array_equal(self.panel.robot_view.live_q, sample["actual_q"])
        self.assertEqual(self.invalidated, ["STALE"])
        self.assertEqual(self._readiness(), [])
        self.assertIn("SYNCED", self.labels["state"].text)

    def test_restart_restores_full_view_transform_without_new_version(self):
        self.assertTrue(self.panel._sync_controller_tcp_to_planning())
        before = self.panel.tcp_calibration_active_file.read_bytes()
        self.invalidated.clear()
        del self.panel.robot_tcp_view_version
        self.panel.robot_view.planning_transform = None
        self.assertTrue(self.panel._sync_controller_tcp_to_planning())
        self.assertEqual(before, self.panel.tcp_calibration_active_file.read_bytes())
        np.testing.assert_allclose(self.panel.robot_view.planning_transform,
                                   json.loads(before)["T_flange_tcp"])
        self.assertEqual(self.invalidated, [])

    def test_planning_busy_defers_file_change_but_change_invalidates_old_trajectory(self):
        self.assertTrue(self.panel._sync_controller_tcp_to_planning())
        before = self.panel.tcp_calibration_active_file.read_bytes()
        self.invalidated.clear()
        self.panel.planning_busy = True
        new_offset = np.asarray(self.model.snapshot()["tcp_offset"]) + [.01, 0, 0, 0, 0, .03]
        sample = self.model.snapshot(1, new_offset)
        self._finish_read({"success": True, "snapshot": sample})
        self.assertEqual(self.panel.tcp_calibration_active_file.read_bytes(), before)
        self.assertEqual(self.invalidated, ["STALE"])
        self.assertTrue(any("不一致" in item or "STALE" in item for item in self._readiness()))
        self.panel.planning_busy = False
        self.assertTrue(self.panel._sync_controller_tcp_to_planning())
        self.assertNotEqual(self.panel.tcp_calibration_active_file.read_bytes(), before)
        self.assertEqual(self._readiness(), [])

    def test_all_busy_states_and_old_snapshot_defer_sync(self):
        for flag in ("planning_busy", "goal_active", "_jog_pending", "robot_tcp_busy"):
            setattr(self.panel, flag, True)
            with self.subTest(flag=flag):
                self.assertFalse(self.panel._sync_controller_tcp_to_planning())
                self.assertFalse(self.panel.tcp_calibration_active_file.exists())
            setattr(self.panel, flag, False)
        self.panel._motion_busy = lambda: True
        self.assertFalse(self.panel._sync_controller_tcp_to_planning())
        self.panel._motion_busy = lambda: False
        self.panel.robot_tcp_received = time.monotonic() - 31.0
        self.assertFalse(self.panel._sync_controller_tcp_to_planning())
        self.assertFalse(self.panel.tcp_calibration_active_file.exists())
        self.assertTrue(any("新鲜" in item or "UNKNOWN" in item for item in self._readiness()))

    def test_failed_read_clears_live_source_and_preflight_remains_blocked(self):
        self.assertTrue(self.panel._sync_controller_tcp_to_planning())
        before = self.panel.tcp_calibration_active_file.read_bytes()
        self._finish_read({"success": False, "reason": "offline read timeout"}, 1)
        self.assertIsNone(self.panel.robot_tcp_snapshot)
        self.assertEqual(self.panel.robot_tcp_received, 0.0)
        self.assertIn("UNKNOWN", self.labels["current"].text)
        self.assertTrue(any("新鲜" in item or "UNKNOWN" in item for item in self._readiness()))
        self.assertEqual(self.panel.tcp_calibration_active_file.read_bytes(), before)

    def test_invalid_active_and_missing_active_never_use_historical_record(self):
        self.assertTrue(self.panel._sync_controller_tcp_to_planning())
        active = self.panel.tcp_calibration_active_file
        historical = self.panel.tcp_calibration_applied_file
        historical.write_bytes(active.read_bytes())
        record = json.loads(active.read_text()); record["T_flange_tcp"][0][3] += .01
        active.write_text(json.dumps(record))
        self.assertTrue(self._readiness())
        active.unlink()
        self.assertTrue(self._readiness())


if __name__ == "__main__":
    unittest.main(verbosity=2)
