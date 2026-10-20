"""Rendering/FK only, synthetic controller sample. Never creates a ROS node."""
import json
import os
from pathlib import Path
import sys

import numpy as np
import vtk
from PyQt5.QtWidgets import QApplication

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gui.vtk_robot_view import VtkRobotView

ROOT = Path(__file__).resolve().parents[2]


def main():
    reference = json.loads((ROOT / 'diagnostics/model_axis_wire_to_tcp_20261013.json').read_text())
    app = QApplication(sys.argv)
    view = VtkRobotView()
    view.resize(1100, 900)
    view.show()
    tcp = json.loads((ROOT / 'real_tcp_calibration_applied.json').read_text())
    view.set_display_tcp_offset(tcp['position_flange_tcp_m'])
    assert view._wire_actor is not None
    assert view._wire_tip_actor is None and view._wire_tip_label is None
    assert len(view._gun_collision_actors) == reference['gun_component_count']
    errors = []
    for i, q in enumerate(reference['q_rad']):
        view.set_live_state(q)
        view._refresh()
        start = np.asarray(view._wire_collision_line_source.GetPoint1())
        end = np.asarray(view._wire_collision_line_source.GetPoint2())
        errors.append(max(np.linalg.norm(start - reference['wire_start_base_m'][i]),
                          np.linalg.norm(end - reference['wire_end_base_m'][i])) * 1000)
    assert max(errors) < 1e-6, errors
    # Visual wire must reach the red controller marker; fixed planning
    # cylinder must not be silently modified by the same display update.
    end_before = np.asarray(view._wire_collision_line_source.GetPoint2())
    start_before = np.asarray(view._wire_collision_line_source.GetPoint1())
    fake_controller = end_before + [0.035197, 0, 0]
    view.set_controller_tcp_pose(fake_controller, reference['q_rad'][-1])
    np.testing.assert_allclose(view._live_tcp_fk, fake_controller)
    np.testing.assert_allclose(view._wire_line_source.GetPoint2(), fake_controller)
    np.testing.assert_allclose(view._tcp_actor.GetPosition(), fake_controller)
    np.testing.assert_allclose(view._wire_collision_line_source.GetPoint2(), end_before)
    np.testing.assert_allclose(view._wire_line_source.GetPoint1(), start_before)
    assert abs(view._wire_live_planning_error_mm - 35.197) < 1e-6
    assert '需确认同步规划 TCP' in view.scene_status()
    assert view._tcp_actor.GetMapper().GetInputConnection(0, 0).GetProducer().GetRadius() == 0.003
    live_q = list(view._live_q)
    live_tcp = view._live_tcp_fk.copy()
    view.set_preview_state(reference['q_rad'][0], 0)
    view._refresh()
    np.testing.assert_allclose(view._live_q, live_q)
    np.testing.assert_allclose(view._live_tcp_fk, live_tcp)
    np.testing.assert_allclose(view._wire_line_source.GetPoint2(), view._preview_tcp_fk)
    view.clear_preview_state()
    # Deliberately mismatched applied/controller offset: capture the visual
    # reaching TCP with Collision Geometry OFF, not a fake synchronized model.
    view.set_controller_tcp_pose(fake_controller, reference['q_rad'][-1])
    view.set_visibility(robot=False, workpiece=False, seam=False, collision=True)
    view.set_visibility(collision=False)
    view._refresh()
    assert not view._wire_collision_actor.GetVisibility()
    assert view._wire_actor.GetVisibility()
    view.fit_gun_view('iso')
    app.processEvents()
    view._tcp_label.SetInput('TCP TEST (offline)')
    view.vtk_widget.GetRenderWindow().Render()
    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(view.vtk_widget.GetRenderWindow())
    capture.ReadFrontBufferOff(); capture.Update()
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(ROOT / 'diagnostics/live_wire_tcp_display_offline_20261013.png'))
    writer.SetInputConnection(capture.GetOutputPort()); writer.Write()
    result = {'status': 'PASS', 'poses': 10,
                      'max_hmi_matlab_endpoint_error_mm': max(errors),
                      'no_yellow_tip': True, 'LIVE_preview_isolated': True,
                      'visual_wire_reaches_controller_tcp': True,
                      'controller_difference_does_not_change_collision_wire': True,
                      'synthetic_controller_planning_difference_mm': 35.197,
                      'robot_motion': 'NO'}
    (ROOT / 'diagnostics/live_wire_tcp_display_offline_20261013.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
    view.close()
    os._exit(0)


if __name__ == '__main__':
    main()
