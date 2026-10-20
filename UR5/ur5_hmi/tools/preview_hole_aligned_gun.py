"""Render the saved four-hole mounting draft without starting ROS."""
import os
import sys
import json
from pathlib import Path
import numpy as np
from PyQt5.QtWidgets import QApplication
import vtk

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ur5_hmi'))
from gui.vtk_robot_view import VtkRobotView


def main():
    app = QApplication(sys.argv)
    view = VtkRobotView(); view.resize(1100, 900); view.show()
    record = json.loads((ROOT / 'real_gun_mount_calibration.json').read_text())
    view.set_gun_source_basis(record['T_flange_source_nominal'])
    view.set_gun_mount_adjustment(record['roll_deg'], record['translation_flange_mm'])
    # A fixed offline view pose only; no command or live-state replacement.
    q = np.deg2rad([-58, -88.5, 2, -89.76, 3, -.43])
    view.set_preview_state(q, 0)
    view.set_visibility(workpiece=False, seam=False, current_tcp=False, preview_tcp=False)
    view._refresh(); app.processEvents()
    expected = np.asarray(record['T_flange_source'])
    actual = np.asarray(view.gun_mount_adjustment()['T_flange_source'])
    assert np.allclose(expected, actual, atol=1e-12)
    assert all(np.allclose(x, expected, atol=1e-12) for x in view._gun_collision_mount_transforms)
    frames = view.model.link_transforms_in_base(q)
    flange = frames['flange']
    center = flange[:3, 3]
    # Keep the wrist/flange and complete gun visible, with annotations hidden.
    for label, _ in view._joint_labels.values():
        label.SetVisibility(False)
    for actor in view._gun_collision_actors:
        actor.SetVisibility(False)
    bounds = np.array(view._gun_actor.GetBounds())
    wrist = view._actors['wrist_3_link'][0].GetBounds()
    for i in range(3):
        bounds[2*i] = min(bounds[2*i], wrist[2*i])
        bounds[2*i+1] = max(bounds[2*i+1], wrist[2*i+1])
    focus = np.array([(bounds[2*i] + bounds[2*i+1]) / 2 for i in range(3)])
    camera = view.renderer.GetActiveCamera()
    camera.ParallelProjectionOn()
    camera.SetFocalPoint(*focus)
    camera.SetPosition(*(focus + flange[:3, :3] @ np.array([.4, -.5, .35])))
    camera.SetViewUp(*(flange[:3, :3] @ [0, 0, 1]))
    view.renderer.ResetCamera(bounds.tolist())
    view.renderer.ResetCameraClippingRange()
    def capture(name):
        view.vtk_widget.GetRenderWindow().Render()
        image = vtk.vtkWindowToImageFilter(); image.SetInput(view.vtk_widget.GetRenderWindow())
        image.ReadFrontBufferOff(); image.Update()
        writer = vtk.vtkPNGWriter(); writer.SetFileName(str(ROOT / 'real_gun_geometry' / name))
        writer.SetInputConnection(image.GetOutputPort()); writer.Write()
    capture('hmi_hole_aligned_gun.png')
    camera.SetFocalPoint(*center)
    camera.SetPosition(*(center + flange[:3, :3] @ np.array([.16, -.22, .16])))
    camera.SetParallelScale(.065)
    view.renderer.ResetCameraClippingRange()
    capture('hmi_hole_aligned_flange_closeup.png')
    print(json.dumps({'loaded_transform_pass': True, 'component_count': len(view._gun_collision_actors),
                      'shared_rigid_transform_pass': True, 'robot_motion': 'NO'}), flush=True)
    view.close(); os._exit(0)


if __name__ == '__main__':
    main()
