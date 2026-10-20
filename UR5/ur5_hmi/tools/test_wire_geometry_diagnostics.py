#!/usr/bin/env python3
"""Offline VTK diagnostic smoke test; synthetic feedback, never robot I/O."""
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
import yaml
from PyQt5.QtWidgets import QApplication

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ur5_hmi'))
from gui.vtk_robot_view import VtkRobotView


def main():
    protected = [ROOT / name for name in (
        'real_tcp_calibration_active.json', 'workpiece_pose_real.json',
        'real_robot_description_calibrated.urdf',
        'real_gun_geometry/real_gun_geometry.json')]
    hashes = [hashlib.sha256(p.read_bytes()).hexdigest() for p in protected]
    app = QApplication(sys.argv)
    view = VtkRobotView()
    view.resize(1000, 900)
    view._timer.stop()
    assert not view.real_wire_geometry_diagnostics()['available']
    q = np.asarray(yaml.safe_load((ROOT / 'ur5_hmi/config/hmi_config.yaml').read_text())['real_home'])
    tcp = np.asarray(json.loads(protected[0].read_text())['T_flange_tcp'])
    view.set_planning_tcp_transform(tcp)
    # A local test fixture only; no ROS subscriptions or controller connection.
    view._live_q = q.copy()
    view._live_tcp_fk = (view.model.link_transforms_in_base(q)['flange'] @ tcp)[:3, 3]
    view._live_tcp_source = 'controller RTDE actual_TCP_pose (OFFLINE TEST FIXTURE)'
    view._live_tcp_rtde_stamp = time.monotonic()
    before_q = q.copy()
    before_tcp = view._live_tcp_fk.copy()
    view._refresh()
    d = view.real_wire_geometry_diagnostics()
    assert d['available'] and d['display_mode'] == 'LIVE'
    assert d['start_to_nozzle_error_mm'] < 1e-9
    assert d['display_end_to_live_tcp_error_mm'] < 1e-9
    assert d['display_start_error_mm'] < 1e-9
    assert abs(np.linalg.norm(d['wire_axis']) - 1) < 1e-12
    assert view._wire_start_marker.GetProperty().GetColor() == (0.1, 1.0, 0.2)
    assert view._wire_start_marker.GetVisibility()
    view._wire_diagnostic_button.setChecked(True)
    app.processEvents()
    assert 'Nozzle tip:' in view._wire_diagnostic_text.toPlainText()
    print('OFFLINE fixture:', json.dumps(d))
    # Nozzle datum mismatch is reported, never corrected.
    view._wire_nozzle_source += np.array([0.001, 0, 0])
    mismatch = view.real_wire_geometry_diagnostics()
    assert abs(mismatch['start_to_nozzle_error_mm'] - 1) < 1e-8
    # LIVE reads must not borrow trajectory preview q or TCP.
    view.set_preview_state(q + 0.01, 0)
    preview = view.real_wire_geometry_diagnostics()
    assert preview['display_mode'] == 'PREVIEW'
    assert preview['display_end_to_live_tcp_error_mm'] is None
    assert np.allclose(preview['tcp_live_world'], before_tcp)
    assert np.array_equal(view._live_q, before_q)
    assert np.array_equal(view._live_tcp_fk, before_tcp)
    view._live_tcp_rtde_stamp = 0
    assert not view.real_wire_geometry_diagnostics()['available']
    # FK fallback is explicitly UNKNOWN, not reported as LIVE measurement.
    view._live_tcp_source = 'HMI flange FK fallback (controller TCP stale/unavailable)'
    assert not view.real_wire_geometry_diagnostics()['available']
    assert hashes == [hashlib.sha256(p.read_bytes()).hexdigest() for p in protected]
    print('PASS: endpoints, nozzle mismatch, axis, marker, panel, preview isolation, stale/UNKNOWN and protected hashes')
    view.close()


if __name__ == '__main__':
    main()
