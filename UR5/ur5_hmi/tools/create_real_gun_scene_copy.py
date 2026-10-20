#!/usr/bin/env python3
"""Create a stopped, script-free UR5 scene copy for manual new-gun review."""
import json
from pathlib import Path
import sys
import argparse
import numpy as np
from scipy.io import loadmat
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,'/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src')
sys.path.insert(0,str(ROOT/'ur5_hmi'))
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from gui.robot_view import UrdfKinematicModel


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--port',type=int,default=23070)
    args=parser.parse_args()
    client=RemoteAPIClient(port=args.port); sim=client.require('sim'); client.socket.RCVTIMEO=600000
    if sim.getSimulationState()!=sim.simulation_stopped:
        raise RuntimeError('Dedicated review instance must be STOPPED')
    folder=ROOT/'real_gun_geometry'
    export=json.loads((folder/'vhacd_collision_export.json').read_text())
    anchor=sim.getObject('/STL_SOURCE_UNCONFIRMED')
    sim.setModelProperty(anchor,0)
    model_file=folder/'new_gun_source_unconfirmed.ttm'
    sim.saveModel(anchor,str(model_file))
    # Read the frozen source, but ONLY save to the explicitly different path.
    sim.loadScene(str(ROOT/'ur5.ttt'))
    if sim.getSimulationState()!=sim.simulation_stopped:
        raise RuntimeError('Unexpected running source scene')
    # This review copy has no simulation/control scripts. Never run ROS here.
    scripts=sim.getObjectsInTree(sim.handle_scene,sim.sceneobject_script,0)
    if scripts: sim.removeObjects(scripts)
    mount=sim.getObject('/UR5/connection/WeldingGunMount')
    old=[]
    for h in sim.getObjectsInTree(mount,sim.sceneobject_shape,0):
        name=sim.getObjectAlias(h)
        if name.startswith('GunCollision') or name.startswith('WeldingGun_visual'):
            old.append(h)
    if old: sim.removeObjects(old)
    new=sim.loadModel(str(model_file))
    sim.setObjectParent(new,mount,True)
    model=UrdfKinematicModel(); zero=model.link_transforms_in_base(np.zeros(6))
    cal=loadmat(str(ROOT/'ur5_tool_calibration.mat'))
    T_flange_mount=np.linalg.inv(zero['flange'])@zero['tool0']@cal['T_tool0_mount']
    T_mount_source=np.linalg.inv(T_flange_mount) # same explicit identity source-at-flange preview as HMI
    sim.setObjectPose(new,mount,np.r_[T_mount_source[:3,3],Rotation.from_matrix(T_mount_source[:3,:3]).as_quat()].tolist())
    out=folder/'ur5_real_gun_alignment_preview.ttt'
    sim.saveScene(str(out))
    result={'scene':str(out),'removed_old_gun_shapes':len(old),'removed_scene_scripts':len(scripts),
            'visual_note':export['coppelia_preview_visual'],
            'simulation_running':False,'status':'UNCONFIRMED_SOURCE_AT_FLANGE',
            'T_mount_source_preview':T_mount_source.tolist(),'frozen_scene_overwritten':False}
    (folder/'scene_copy_report.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))


if __name__=='__main__': main()
