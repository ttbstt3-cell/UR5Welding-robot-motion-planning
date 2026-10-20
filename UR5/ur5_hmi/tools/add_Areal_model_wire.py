#!/usr/bin/env python3
"""Define this CAD's annular nozzle centre and the existing model-to-TCP wire.

No robot/RTDE access, TCP writes, geometry activation, or nearest-hull guess.
"""
import hashlib
import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from align_gun_mount_holes import mesh, boundaries

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ur5_hmi'))
from calibration.model_axis_wire import build_model_axis_wire


def save(path,data):
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


def main():
    folder=ROOT/'real_gun_geometry'
    path=folder/'real_gun_geometry.json'
    m=json.loads(path.read_text())
    source=Path(m['source']['stl'])
    expected='06f38186eddd712eb239f069878da1d9d307171dc693df3a83c8b086b8d2e21c'
    if hashlib.sha256(source.read_bytes()).hexdigest()!=expected or m['source']['sha256']!=expected:
        raise ValueError('This nozzle extraction is specific to A焊枪real.STL')
    v,f=mesh(source)
    # Local nozzle direction is evidenced by the end annulus in this CAD.
    n=np.array([0.,-1.,1.])/np.sqrt(2)
    x=np.array([1.,0.,0.]); y=np.cross(n,x)
    basis=np.array([x,y,n]); p=v@basis.T
    plane=210.4804 # mm, audited terminal annulus plane; not a hull vertex
    loops=boundaries(p,f,2,plane,.03)
    outer=next((l for l in loops if 10<l['radius']<10.6 and l['rms']<.001),None)
    inner=next((l for l in loops if 8<l['radius']<9 and l['rms']<.001),None)
    if outer is None or inner is None or np.linalg.norm(outer['center']-inner['center'])>.001:
        raise ValueError('No trusted concentric nozzle annulus in this source')
    points=outer['points']; plane=float(np.mean(points[:,2]))
    center=basis.T@np.r_[outer['center'],plane]
    end_plane=float(p[:,2].max())
    evidence=dict(source_sha256=expected,method='Terminal CAD annulus planar boundary circle fits',
                  nozzle_center_source_mm=center.tolist(),nozzle_axis_source=n.tolist(),
                  outer_radius_mm=outer['radius'],inner_radius_mm=inner['radius'],
                  outer_fit_rms_mm=outer['rms'],inner_fit_rms_mm=inner['rms'],
                  concentricity_error_mm=float(np.linalg.norm(outer['center']-inner['center'])),
                  plane_projection_mm=plane,embedded_wire_extent_mm=end_plane-plane,
                  embedded_wire_diameter_mm=.8,physical_measurement=False,
                  note='CAD reference only. Added cylinder follows nozzle centre to synchronized TCP, not an axis acceptance test.')
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    history=folder/'wire_versions'; history.mkdir(exist_ok=True)
    shutil.copy2(path,history/('before_Areal_wire_'+stamp+'.json'))
    m['wire']=dict(status='MODEL_AXIS_TO_TCP_DEFINED',coordinate_frame='STL_SOURCE',
                   gun_tip_model=(center*.001).tolist(),tip_source=(center*.001).tolist(),
                   tip_definition='Areal CAD terminal annulus concentric circle centre; not nearest hull vertex',
                   nozzle_axis_source=n.tolist(),diameter_m=.0012,source_sha256=expected,
                   physical_measurement_required=False,contact_tip_calibration_required=False,
                   evidence_file=str(folder/'Areal_nozzle_reference.json'))
    preview=m['status']!='ACTIVE'
    wire=build_model_axis_wire(ROOT,m,preview_only=preview)
    m['wire'].update(wire)
    m['wire_axis_status']=wire['status']
    m['version']=str(m['version'])+'-MODEL-WIRE-'+stamp
    m['requires_replan']=True
    save(folder/'Areal_nozzle_reference.json',evidence)
    save(folder/'Areal_wire_definition.json',wire)
    save(path,m)
    print(json.dumps(dict(nozzle_center_mm=center.tolist(),length_mm=wire['length_m']*1000,
                         diameter_mm=1.2,status=wire['status'],parent_status=m['status']),ensure_ascii=False))


if __name__=='__main__': main()
