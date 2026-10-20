"""Rendering-only replacement STL smoke test; no ROS node or robot calls."""
import json
import os
from pathlib import Path
import sys
import numpy as np
import yaml
import vtk
from PyQt5.QtWidgets import QApplication

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
os.environ.setdefault('DISPLAY',':0')
from gui.vtk_robot_view import VtkRobotView


def main():
    app=QApplication.instance() or QApplication(sys.argv)
    view=VtkRobotView(); view.resize(1100,900); view.show()
    manifest=json.loads((ROOT.parent/'real_gun_geometry/real_gun_geometry.json').read_text())
    assert manifest['collision']['method']=='CoppeliaSim simConvex.vhacd'
    assert len(view._gun_collision_actors)==len(manifest['collision']['components'])
    assert view._wire_actor is not None and view._wire_tip_tcp is not None
    assert view.gun_mount_adjustment()['geometry_preview']
    home=yaml.safe_load((ROOT/'config/hmi_config.yaml').read_text())['real_home']
    view.set_preview_state(home,0)
    view.set_visibility(robot=False,workpiece=False,seam=False,current_tcp=False,preview_tcp=False)
    folder=ROOT.parent/'real_gun_geometry'
    results=[]
    for angle in (0,90,-90):
        old_live_q=view._live_q
        old_tcp=view._live_tcp_fk
        view.set_gun_mount_adjustment(angle,[0,0,0])
        view._refresh()
        view.fit_gun_view('iso')
        app.processEvents()
        view.vtk_widget.GetRenderWindow().Render()
        transform=np.asarray(view.gun_mount_adjustment()['T_flange_source'])
        for T in view._gun_collision_mount_transforms:
            assert np.allclose(T,transform,atol=1e-12)
        assert view._live_q is old_live_q and view._live_tcp_fk is old_tcp
        capture=vtk.vtkWindowToImageFilter()
        capture.SetInput(view.vtk_widget.GetRenderWindow())
        capture.ReadFrontBufferOff(); capture.Update()
        writer=vtk.vtkPNGWriter()
        writer.SetFileName(str(folder/f'hmi_new_gun_preview_{angle:+d}.png'))
        writer.SetInputConnection(capture.GetOutputPort()); writer.Write()
        results.append({'angle_deg':angle,'components':len(view._gun_collision_actors),
                        'shared_transform_pass':True,'LIVE_state_unchanged':True,
                        'wire_model_defined_active':True,'installation_confirmed':True})
    # Scene reload must not leave duplicate old/new actors.
    count=view.renderer.GetActors().GetNumberOfItems()
    view._load_scene_geometry(); view._refresh()
    assert len(view._gun_collision_actors)==len(manifest['collision']['components'])
    assert view.renderer.GetActors().GetNumberOfItems()==count
    (folder/'hmi_geometry_smoke.json').write_text(json.dumps({'checks':results,'reload_no_duplicate_actors':True,'robot_motion':'NO'},indent=2)+'\n')
    print(json.dumps(results),flush=True)
    view.close()
    # Avoid VTK/X11 destructor ordering (existing smoke pattern).
    os._exit(0)


if __name__=='__main__': main()
