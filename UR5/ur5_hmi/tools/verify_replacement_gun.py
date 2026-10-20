#!/usr/bin/env python3
"""Render and numerically inspect the replacement gun; no ROS/no motion."""
import hashlib
import json
import os
from pathlib import Path
import sys

import numpy as np
import vtk
import yaml
from PyQt5.QtWidgets import QApplication

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ur5_hmi'))
from gui.vtk_robot_view import VtkRobotView
from calibration.controller_tcp_sync import validate_controller_tcp_record


def main():
    folder=ROOT/'real_gun_geometry'
    manifest=json.loads((folder/'real_gun_geometry.json').read_text())
    T=np.asarray(manifest['preview']['T_flange_source'])
    config=ROOT/'ur5_hmi/config/hmi_config.yaml'
    home=np.asarray(yaml.safe_load(config.read_text())['real_home'])
    poses=np.tile(home,(10,1))
    for i in range(1,10):
        poses[i]+=np.deg2rad(10)*np.sin(np.arange(6)+i)
    app=QApplication(sys.argv)
    view=VtkRobotView(); view.resize(1100,900); view.show()
    assert view._gun_geometry_preview == (manifest['status']!='ACTIVE')
    assert len(view._gun_collision_actors)==manifest['collision']['component_count']
    active=json.loads((ROOT/'real_tcp_calibration_active.json').read_text())
    urdf_hash=hashlib.sha256((ROOT/'real_robot_description_calibrated.urdf').read_bytes()).hexdigest()
    tcp=validate_controller_tcp_record(active,view.model,urdf_hash)
    view.set_planning_tcp_transform(tcp)
    samples=[]
    wire_errors=[]
    for q in poses:
        view.set_preview_state(q,0)
        view._refresh()
        frames=view.model.link_transforms_in_base(q)
        source=frames['flange']@T
        for component_T in view._gun_collision_mount_transforms:
            assert np.allclose(component_T,T,atol=1e-12)
        centers=[(source@np.r_[np.mean(c['vertices_m'],axis=0),1])[:3].tolist()
                 for c in manifest['collision']['components']]
        samples.append(dict(q_rad=q.tolist(),T_base_flange=frames['flange'].tolist(),
                            T_base_source=source.tolist(),component_centers_base_m=centers))
        if view._wire_actor is not None and view._wire_end_source is not None:
            expected=(frames['flange']@tcp)[:3,3]
            visual=np.asarray(view._wire_line_source.GetPoint2())
            collision=np.asarray(view._wire_collision_line_source.GetPoint2())
            wire_errors.append(dict(visual_error_mm=float(np.linalg.norm(visual-expected)*1000),
                                    collision_error_mm=float(np.linalg.norm(collision-expected)*1000)))
            assert np.linalg.norm(visual-expected)<1e-10
            assert np.linalg.norm(collision-expected)<1e-10
            assert view._wire_tube.GetRadius()==.0006
    assert view._live_q is None, 'Preview must never overwrite real feedback'
    view.set_preview_state(home,0)
    view._refresh()
    view.set_visibility(workpiece=False,seam=False,current_tcp=False,preview_tcp=False)
    for actor in (view._calibrated_tcp_actor,view._calibrated_tcp_line_actor,view._calibrated_tcp_label,
                  view._axes,*view._axis_label_actors):
        if actor is not None: actor.SetVisibility(False)
    for label,_ in view._joint_labels.values(): label.SetVisibility(False)
    # Isolate the wrist/flange and complete tool for installation review.
    for name, (actor, _origin) in view._actors.items():
        actor.SetVisibility(name=='wrist_3_link')
    gun_bounds=np.asarray(view._gun_actor.GetBounds())
    wrist_bounds=np.asarray(view._actors['wrist_3_link'][0].GetBounds())
    bounds=gun_bounds.copy()
    bounds[::2]=np.minimum(gun_bounds[::2],wrist_bounds[::2])
    bounds[1::2]=np.maximum(gun_bounds[1::2],wrist_bounds[1::2])
    if view._wire_end_source is not None:
        point=np.asarray(view._preview_tcp_fk)
        bounds[::2]=np.minimum(bounds[::2],point-.003)
        bounds[1::2]=np.maximum(bounds[1::2],point+.003)
    focus=(bounds[::2]+bounds[1::2])/2
    flange=view.model.link_transforms_in_base(home)['flange']
    camera=view.renderer.GetActiveCamera(); camera.ParallelProjectionOn()
    camera.SetFocalPoint(*focus)
    camera.SetPosition(*(focus+flange[:3,:3]@np.array([.4,-.6,.4])))
    camera.SetViewUp(*(flange[:3,:3]@np.array([0,0,1])))
    view.renderer.ResetCamera(bounds.tolist()); view.renderer.ResetCameraClippingRange()
    app.processEvents()
    def capture(name):
        view.vtk_widget.GetRenderWindow().Render()
        filt=vtk.vtkWindowToImageFilter(); filt.SetInput(view.vtk_widget.GetRenderWindow())
        filt.ReadFrontBufferOff(); filt.Update()
        writer=vtk.vtkPNGWriter(); writer.SetFileName(str(folder/name)); writer.SetInputConnection(filt.GetOutputPort())
        writer.Write()
    # CAD exterior, and independent orange collision overlay.
    for actor in view._gun_collision_actors: actor.SetVisibility(False)
    capture('Areal_visual_preview.png')
    for actor in view._gun_collision_actors: actor.SetVisibility(True)
    capture('Areal_collision_preview.png')
    if wire_errors:
        # Explicit offline preview marker, never fake red LIVE controller data.
        for actor in view._gun_collision_actors: actor.SetVisibility(False)
        view._wire_collision_actor.SetVisibility(False)
        view._preview_tcp_actor.SetVisibility(True)
        view._preview_tcp_actor.SetScale(.15,.15,.15)
        view._preview_tcp_line_actor.SetVisibility(False)
        view._preview_tcp_label.SetInput('TCP PREVIEW (pendant calibration)')
        view._preview_tcp_label.SetVisibility(True)
        capture('Areal_wire_preview.png')
        wire_report=dict(status='OFFLINE_PREVIEW_PASS',pose_count=len(wire_errors),
                         errors=wire_errors,diameter_mm=1.2,collision_cylinder_present=True,
                         live_feedback_unchanged=True,execution_authorized=False,robot_motion='NO')
        (folder/'Areal_wire_hmi_report.json').write_text(json.dumps(wire_report,indent=2)+'\n')
    request=dict(q_rad=poses.tolist(),qHome=home.tolist(),qHome_source=str(config),
                 T_flange_source=T.tolist(),source_sha256=manifest['source']['sha256'],
                 hmi_samples=samples,robot_motion='NO')
    (folder/'Areal_consistency_request.json').write_text(json.dumps(request,indent=2)+'\n')
    report=dict(status='OFFLINE_PREVIEW_PASS_NOT_PLANNING_APPROVAL',component_count=len(samples[0]['component_centers_base_m']),
                source_sha256=manifest['source']['sha256'],visual_and_collision_same_transform=True,
                real_feedback_unchanged=True,TCP_file_unchanged=True,
                wire_status=manifest['wire']['status'],
                installation_confirmed=manifest['mounting']['status']=='CONFIRMED',robot_motion='NO')
    (folder/'Areal_hmi_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)
    view.close()
    os._exit(0)


if __name__=='__main__': main()
