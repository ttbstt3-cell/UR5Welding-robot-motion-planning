"""Offline tests for controller TCP frame conversion and quality gates."""
from __future__ import annotations

from copy import deepcopy
import unittest

import numpy as np

from calibration.tcp_frames import (
    calibration_write_quality, controller_snapshot_to_flange_tcp,
    flange_offset_to_controller_tcp, transform_to_ur_pose, ur_pose_to_transform,
    validate_rtde_model_alignment, validate_saved_pivot_payload,
)
from calibration.tcp_pivot import fit_pivot_calibration
from safety.joint_limits import JOINT_NAMES


class SyntheticModel:
    revolute_names = list(JOINT_NAMES)

    def __init__(self):
        self.flange_tool0 = np.eye(4)
        self.flange_tool0[:3, :3] = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]
        self.poses = []
        tcp = np.array([.377, -.032, .007])
        pivot = np.array([-.30, .36, .20])
        for rotvec in ([.3, 0, 0], [0, .5, 0], [0, 0, .7], [.6, .3, 0], [0, .4, .4]):
            T = ur_pose_to_transform([0, 0, 0, *rotvec])
            T[:3, 3] = pivot - T[:3, :3] @ tcp
            self.poses.append(T)

    def link_transforms_in_base(self, q):
        flange = self.poses[int(q[0])].copy()
        return {"flange": flange, "tool0": flange @ self.flange_tool0}


class TCPFrameTests(unittest.TestCase):
    def setUp(self):
        self.model = SyntheticModel()

    def test_exact_flange_mapping_and_orientation_unchanged(self):
        original = [.01353, .01713, .37983, -.9562, 2.4566, 1.2418]
        converted = flange_offset_to_controller_tcp(self.model, [.3776839156, -.0317121363, .0071119466], original)
        np.testing.assert_allclose(converted[:3], [-.0317121363, .0071119466, .3776839156], atol=1e-14)
        np.testing.assert_array_equal(converted[3:], original[3:])
        np.testing.assert_allclose(self.model.flange_tool0 @ np.r_[converted[:3], 1],
                                   [.3776839156, -.0317121363, .0071119466, 1], atol=1e-14)

    def test_conversion_accounts_for_frame_translation(self):
        self.model.flange_tool0[:3, 3] = [.01, -.02, .03]
        converted = flange_offset_to_controller_tcp(self.model, [.377, -.032, .007], np.zeros(6))
        np.testing.assert_allclose(self.model.flange_tool0 @ np.r_[converted[:3], 1], [.377, -.032, .007, 1])

    def test_alignment_removes_active_tcp_offset(self):
        # Identity FK lets the composed actual pose equal the current TCP;
        # using actual_TCP_pose directly as tool0 would fail this test.
        self.model.poses[0] = np.linalg.inv(self.model.flange_tool0)
        offset = [.01353, .01713, .37983, -.9562, 2.4566, 1.2418]
        sample = {"actual_q": [0] * 6, "actual_TCP_pose": offset, "tcp_offset": offset}
        report = validate_rtde_model_alignment(self.model, sample)
        self.assertTrue(report["pass"])
        self.assertLess(report["position_error_mm"], 1e-8)
        self.assertLess(report["orientation_error_deg"], 1e-5)
        sample["actual_TCP_pose"] = np.array(offset) + [.0031, 0, 0, 0, 0, 0]
        self.assertFalse(validate_rtde_model_alignment(self.model, sample)["pass"])

    def test_controller_snapshot_derives_complete_flange_tcp(self):
        local = ur_pose_to_transform([.0, 0, 0, .2, -.1, .3])
        local[:3, 3] = [.31, -.02, .04]
        flange = self.model.poses[0]
        actual = transform_to_ur_pose(flange @ local)
        sample = {"actual_q": [0] * 6, "actual_TCP_pose": actual.tolist(),
                  "tcp_offset": [0, 0, 0, 0, 0, 0]}
        converted = controller_snapshot_to_flange_tcp(self.model, sample)
        np.testing.assert_allclose(converted["T_flange_tcp"], local, atol=1e-9)
        np.testing.assert_allclose(converted["position_flange_tcp_m"], local[:3, 3], atol=1e-9)

    def _payload(self):
        fit = fit_pivot_calibration(self.model.poses)
        return {
            "valid": True, "urdf_sha256": "a" * 64,
            "coordinate_frames": {"base": "URDF base", "flange": "URDF flange"},
            "position_flange_tcp_m": fit.tcp_flange_m.tolist(), "fit": fit.to_dict(),
            "pose_samples": [{"q_rad": [i, 0, 0, 0, 0, 0]} for i in range(5)],
            "independent_verification_mm": [.4],
        }

    def test_saved_fit_rebuilt_with_model_fingerprint(self):
        result = validate_saved_pivot_payload(self._payload(), self.model, "a" * 64)
        self.assertTrue(result["valid"])
        self.assertLess(result["offset_reproduction_error_mm"], 1e-8)
        self.assertFalse(result["independent_verification_recomputed"])

    def test_weak_pose_coverage_can_fit_well_but_block_writes(self):
        poses = []
        noises = [[.0006, 0, 0], [-.0006, 0, 0], [0, .0006, 0], [0, 0, -.0006]]
        for rotvec, noise in zip(([.12, 0, 0], [0, .12, 0], [0, 0, .12], [.12, .06, 0]), noises):
            T = ur_pose_to_transform([0, 0, 0, *rotvec])
            T[:3, 3] = np.array([.3, .3, .2]) - T[:3, :3] @ [.38, -.03, .007] + noise
            poses.append(T)
        result = fit_pivot_calibration(poses)
        self.assertTrue(result.valid)
        quality = calibration_write_quality(poses, result)
        self.assertEqual(quality["matrix_rank"], 6)
        self.assertLess(quality["condition_number"], 100)
        self.assertLess(quality["max_residual_mm"], 1)
        self.assertGreater(quality["max_std_mm"], 2)
        self.assertFalse(quality["pass"])

    def test_saved_valid_flag_cannot_bypass_checks(self):
        base = self._payload()
        for key, value in (("urdf_sha256", "b" * 64),
                           ("position_flange_tcp_m", [.2, 0, 0]),
                           ("position_flange_tcp_m", [float("nan"), 0, 0]),
                           ("independent_verification_mm", [float("inf")])):
            payload = deepcopy(base)
            payload[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                validate_saved_pivot_payload(payload, self.model, "a" * 64)
        payload = deepcopy(base)
        payload["independent_verification_mm"] = [2.01]
        self.assertTrue(validate_saved_pivot_payload(payload, self.model, "a" * 64)["valid"])
        payload = deepcopy(base)
        payload["fit"]["condition_number"] = 101
        with self.assertRaises(ValueError):
            validate_saved_pivot_payload(payload, self.model, "a" * 64)
        payload = deepcopy(base)
        payload["pose_samples"] = [payload["pose_samples"][0]] * 5
        with self.assertRaises(ValueError):
            validate_saved_pivot_payload(payload, self.model, "a" * 64)

    def test_saved_independent_poses_recomputed(self):
        payload = self._payload()
        tcp = np.array([.377, -.032, .007])
        pivot = np.array([-.30, .36, .20])
        T = ur_pose_to_transform([0, 0, 0, .4, -.6, .2])
        T[:3, 3] = pivot - T[:3, :3] @ tcp + [.0005, 0, 0]
        self.model.poses.append(T)
        payload["verification_samples"] = [{"q_rad": [5, 0, 0, 0, 0, 0], "timestamp": "test"}]
        payload["independent_verification_mm"] = [.5]
        report = validate_saved_pivot_payload(payload, self.model, "a" * 64)
        self.assertTrue(report["independent_verification_recomputed"])
        self.assertAlmostEqual(report["independent_verification_max_mm"], .5, places=7)
        payload["independent_verification_mm"] = [.4]
        with self.assertRaises(ValueError):
            validate_saved_pivot_payload(payload, self.model, "a" * 64)

    def test_independent_records_reject_duplicate_or_failed_pose(self):
        payload = self._payload()
        payload["verification_samples"] = [{"q_rad": [0] * 6}]
        payload["independent_verification_mm"] = [0]
        with self.assertRaises(ValueError):
            validate_saved_pivot_payload(payload, self.model, "a" * 64)
        T = ur_pose_to_transform([0, 0, 0, -.3, .6, .2])
        T[:3, 3] = np.array([-.30, .36, .20]) - T[:3, :3] @ [.377, -.032, .007] + [.003, 0, 0]
        self.model.poses.append(T)
        payload["verification_samples"] = [{"q_rad": [5, 0, 0, 0, 0, 0]}]
        payload["independent_verification_mm"] = [.5]
        with self.assertRaises(ValueError):
            validate_saved_pivot_payload(payload, self.model, "a" * 64)
        payload["verification_samples"] = []
        with self.assertRaises(ValueError):
            validate_saved_pivot_payload(payload, self.model, "a" * 64)
    def test_invalid_joint_order_and_frames_rejected(self):
        self.model.revolute_names = list(reversed(JOINT_NAMES))
        with self.assertRaises(ValueError):
            flange_offset_to_controller_tcp(self.model, [0, 0, .3], np.zeros(6))
        with self.assertRaises(ValueError):
            validate_rtde_model_alignment(self.model, {"frame_id": "base_link"})


if __name__ == "__main__":
    unittest.main()
