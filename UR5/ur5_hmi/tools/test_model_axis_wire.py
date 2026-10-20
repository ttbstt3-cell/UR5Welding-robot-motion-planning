"""Offline wire schema, frame and negative-input regression; no ROS calls."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calibration.model_axis_wire import build_model_axis_wire, WIRE_STATUS

ROOT = Path(__file__).resolve().parents[2]


class ModelAxisWireTest(unittest.TestCase):
    def setUp(self):
        self.manifest = json.loads((ROOT / 'real_gun_geometry/real_gun_geometry.json').read_text())
        # Independent unit fixture: do not activate the real staged package.
        self.manifest['status'] = 'ACTIVE'
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        # Offline geometry fixture only. The source-authority/conversion gate
        # is tested separately in test_controller_tcp_sync.py; never fabricate
        # an ACTIVE calibration in the real project directory for this test.
        self.tcp = json.loads((ROOT / 'real_tcp_calibration_applied.json').read_text())
        self.tcp['version'] = 'OFFLINE-WIRE-ENDPOINT-FIXTURE'
        (self.root / 'real_tcp_calibration_active.json').write_text(json.dumps(self.tcp))

    def test_real_endpoint_and_fixed_geometry(self):
        w = build_model_axis_wire(self.root, self.manifest)
        T = np.asarray(self.manifest['visual']['T_flange_source'])
        tcp = self.tcp
        self.assertEqual(w['status'], WIRE_STATUS)
        self.assertEqual(w['diameter_m'], 0.0012)
        self.assertGreater(w['length_m'], 0)
        np.testing.assert_allclose(np.linalg.norm(w['axis_source']), 1, atol=1e-12)
        np.testing.assert_allclose((T @ np.r_[w['wire_end_source'], 1])[:3],
                                   tcp['position_flange_tcp_m'], atol=1e-12)
        np.testing.assert_allclose(np.asarray(w['tip_source']) + w['length_m'] * np.asarray(w['axis_source']),
                                   w['tcp_source'], atol=1e-12)
        # Display-ball size and obsolete endpoint data must have no influence.
        m = copy.deepcopy(self.manifest)
        m['wire'].update(tcp_marker_radius_m=99, wire_end_source=[999]*3, axis_source=[0]*3, length_m=-1)
        self.assertEqual(build_model_axis_wire(self.root, m), w)

    def test_staged_wire_preview_never_authorizes_planning(self):
        self.manifest['status'] = 'STAGED_ALIGNMENT_REQUIRED'
        with self.assertRaises(ValueError):
            build_model_axis_wire(self.root, self.manifest)
        wire = build_model_axis_wire(self.root, self.manifest, preview_only=True)
        self.assertTrue(wire['preview_only'])
        self.assertEqual(wire['status'], 'MODEL_AXIS_TO_TCP_PREVIEW')

    def test_no_reference_no_fallback(self):
        self.manifest['wire'] = {}
        with self.assertRaises(ValueError):
            build_model_axis_wire(self.root, self.manifest)

    def test_missing_active_never_uses_historical_four_point_tcp(self):
        (self.root / 'real_tcp_calibration_active.json').unlink()
        (self.root / 'real_tcp_calibration_applied.json').write_text(json.dumps(self.tcp))
        with self.assertRaises(FileNotFoundError):
            build_model_axis_wire(self.root, self.manifest)

    def test_controller_active_version_precedes_old_four_point_record(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            old = json.loads((ROOT / 'real_tcp_calibration_applied.json').read_text())
            active = copy.deepcopy(old)
            active['version'] = 'CONTROLLER-TCP-TEST'
            active['position_flange_tcp_m'][0] += 0.002
            (root / 'real_tcp_calibration_applied.json').write_text(json.dumps(old))
            (root / 'real_tcp_calibration_active.json').write_text(json.dumps(active))
            wire = build_model_axis_wire(root, self.manifest)
            T = np.asarray(self.manifest['visual']['T_flange_source'])
            np.testing.assert_allclose((T @ np.r_[wire['wire_end_source'], 1])[:3],
                                       active['position_flange_tcp_m'], atol=1e-12)
            self.assertEqual(wire['tcp_calibration_version'], active['version'])
            # An invalid active version must never silently select the old TCP.
            active['valid'] = False
            (root / 'real_tcp_calibration_active.json').write_text(json.dumps(active))
            with self.assertRaises(ValueError):
                build_model_axis_wire(root, self.manifest)

    def test_degenerate_and_wrong_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            T = np.asarray(self.manifest['visual']['T_flange_source'])
            p = (T @ np.r_[self.manifest['wire']['gun_tip_model'], 1])[:3]
            record = {'valid': True, 'applied_to_planning_model': True,
                      'coordinate_frames': {'flange': 'URDF flange'},
                      'position_flange_tcp_m': p.tolist()}
            # Set exact zero in source frame to avoid inverse roundoff.
            self.manifest['visual']['T_flange_source'] = np.eye(4).tolist()
            record['position_flange_tcp_m'] = self.manifest['wire']['gun_tip_model']
            path = root / 'real_tcp_calibration_active.json'
            path.write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError, 'length'):
                build_model_axis_wire(root, self.manifest)
            record['coordinate_frames']['flange'] = 'tool0'
            path.write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError, '坐标系'):
                build_model_axis_wire(root, self.manifest)


if __name__ == '__main__':
    unittest.main()
