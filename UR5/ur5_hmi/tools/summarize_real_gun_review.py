"""Aggregate offline evidence. No robot connection or configuration writes."""
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
FOLDER=ROOT/'real_gun_geometry'


def main():
    read=lambda name:json.loads((FOLDER/name).read_text())
    geometry=read('geometry_review.json')
    collision=read('vhacd_collision_export.json')
    smoke=read('hmi_geometry_smoke.json')
    matlab={str(a):read(f'matlab_review_{a:+d}.json') for a in (0,90,-90)}
    errors=[]; flange_errors=[]
    for h,m in zip(geometry['hmi_samples'],matlab['0']['samples']):
        assert np.allclose(h['q_rad'],m['q_rad'],atol=1e-12)
        errors.extend(np.linalg.norm(np.array(h['component_centers_base_m'])-
                                    np.array(m['component_centers_base_m']),axis=1))
        flange_errors.append(np.max(np.abs(np.array(h['T_base_flange'])-
                                           np.array(m['T_base_flange']))))
    backup=ROOT/'backups/gun_geometry_20261010_184312'
    files={'scene_geometry.json':'scene_geometry.json',
           'ur5_tool_calibration.mat':'ur5_tool_calibration.mat',
           'real_tcp_calibration.json':'real_tcp_calibration.json',
           'real_tcp_calibration_applied.json':'real_tcp_calibration_applied.json',
           'workpiece_pose_real.json':'workpiece_pose_real.json',
           'real_robot_description_calibrated.urdf':'real_robot_description_calibrated.urdf',
           'ur5.ttt':'ur5_frozen.ttt'}
    hashes={}
    for current,previous in files.items():
        sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
        hashes[current]={'current':sha(ROOT/current),'backup':sha(backup/previous)}
        hashes[current]['unchanged']=hashes[current]['current']==hashes[current]['backup']
    assert all(v['unchanged'] for v in hashes.values())
    manifest=read('real_gun_geometry.json')
    manifest['activation_policy']='STAGED geometry may be explicitly previewed; REAL planning requires confirmed physical mounting and accepted collision coverage.'
    manifest['collision']['coverage_review']='LOCAL_UNDERCOVERAGE_REQUIRES_REVIEW'
    manifest['collision']['coverage_report']=str(FOLDER/'geometry_review.json')
    (FOLDER/'real_gun_geometry.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    report={'schema_version':'real_gun_replacement_report_v2','status':'STAGED_ALIGNMENT_AND_COVERAGE_REVIEW_REQUIRED',
            'source_stl':manifest['source'],'backup_dir':str(backup),
            'collision':{k:collision[k] for k in ('method','component_count','vertex_count','face_count','parameters','elapsed_seconds','component_frame_error_m','collision_input','coppelia_preview_visual')},
            'collision_input_audit_experimental_derivative_not_used_in_final_full_source_run':read('collision_input_audit.json'),
            'coverage':{k:geometry[k] for k in ('surface_vertex_sample_count','sample_fraction_inside_union','max_convex_halfspace_violation_mm','coverage_note')},
            'HMI_MATLAB_10_pose_comparison':{'component_center_max_error_mm':float(max(errors)*1000),
                    'component_center_RMS_error_mm':float(np.sqrt(np.mean(np.square(errors)))*1000),
                    'flange_transform_max_abs_error':float(max(flange_errors)),
                    'relative_transform_max_fro_error':max(s['relative_transform_error'] for s in matlab['0']['samples'])},
            'saved_home_collision_preview':{a:{k:r[k] for k in ('self_collision','environment_collision','workpiece_review_status','collision_acceptance')} for a,r in matlab.items()},
            'HMI_smoke':smoke,'HMI_panel_smoke':read('hmi_panel_smoke.json'),'frozen_hash_checks':hashes,
            'wire_status':str(manifest.get('wire',{}).get('status','UNAVAILABLE')),'old_real_trajectories':'STALE',
            'activation_blockers':['STL units/flange mounting datum must be physically confirmed',
                                   'Local V-HACD undercoverage requires conservative collision-envelope review',
                                   'Recheck Saved RealHome/environment collision with confirmed mounting',
                                   'Existing DRAFT workpiece requires its own acceptance; this review does not bypass it'],
            'real_TCP_unchanged':True,'robot_motion_sent':False}
    (FOLDER/'replacement_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report['HMI_MATLAB_10_pose_comparison'],indent=2))


if __name__=='__main__':main()
