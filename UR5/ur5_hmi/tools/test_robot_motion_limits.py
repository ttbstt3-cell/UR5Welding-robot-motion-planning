"""Offline limit/profile/analytic-extrema/worker gate tests; no ROS node."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from safety.robot_motion_limits import PROFILE, URDF, load_limits, validate_contract, cubic_extrema, check_motion, message_motion_reasons
from trajectory.trajectory_loader import load_trajectory
from trajectory.trajectory_utils import trajectory_to_ros_message
from trajectory.result_store import read_index
from gui.main_window import RosWorker
from gui.planning_workflow import PlanningWorkflow, PlanningStage
from gui.execution_preparation_panel import ExecutionPreparationPanel


def run(output, trajectory_file=None):
    output.mkdir(parents=True, exist_ok=True)
    limits=load_limits(); rejected=[]; facts={}
    assert limits['scalar_velocity_rad_s']==np.pi and limits['scalar_acceleration_rad_s2']==40
    assert limits['normal_joint_speed_evidence_present'] and limits['normal_tcp_speed_m_s']==1.5
    assert not limits['installation_safety_verified'] and not limits['payload_acceleration_verified']
    def expect(name, q, t, v, phrase):
        reasons, _=check_motion(q,t,v)
        assert any(phrase in r for r in reasons), (name,reasons)
        rejected.append(name)
    q=np.zeros((2,6)); q[1,0]=2.2; v=np.zeros_like(q)
    expect('between_node_velocity_peak',q,[0,1],v,'VELOCITY_LIMIT')
    q=np.zeros((2,6)); q[1,0]=.4
    expect('between_node_acceleration_peak',q,[0,.2],v,'ACCELERATION_LIMIT')
    q=np.zeros((2,6)); q[:,5]=6.2; v=np.zeros_like(q);v[0,5]=1;v[1,5]=-1
    expect('between_node_position_overshoot',q,[0,1],v,'POSITION_LIMIT')
    expect('cartesian_speed_bound',np.zeros((2,6)),[0,1],np.full((2,6),.5),'TCP_SPEED_BOUND_LIMIT')
    for name,q,t,v in [('nonfinite',np.full((2,6),np.nan),[0,1],np.zeros((2,6))),
                       ('zero_dt',np.zeros((2,6)),[0,0],np.zeros((2,6))),
                       ('missing_velocities',np.zeros((2,6)),[0,1],None)]:
        try:cubic_extrema(q,t,v)
        except ValueError:rejected.append(name)
        else:raise AssertionError(name)
    original=PROFILE.read_bytes(); urdf_before=URDF.read_bytes()
    with tempfile.TemporaryDirectory(prefix='ur5_limits_TEST_ONLY_') as temp:
        file=Path(temp)/'limits.json'; base=json.loads(original)
        for name,mutate in [('urdf_version',lambda d:d.update(urdf_sha256='0'*64)),
                            ('controller_array',lambda d:d['controller']['acceleration_rad_s2'].__setitem__(0,100)),
                            ('controller_hash',lambda d:d['controller'].update(raw_configuration_sha256='0'*64)),
                            ('wrong_joint_order',lambda d:d['joint_names'].reverse()),
                            ('photo_hash',lambda d:d['pendant_evidence'].update(general_photo_sha256='0'*64))]:
            d=copy.deepcopy(base);mutate(d);file.write_text(json.dumps(d))
            try:load_limits(file)
            except ValueError:rejected.append(name)
            else:raise AssertionError(name)
        d=copy.deepcopy(base);d['installation_safety_verified']=d['payload_acceleration_verified']=True
        file.write_text(json.dumps(d));assert not load_limits(file)['payload_acceleration_verified']
        facts['flipping_json_booleans_cannot_approve_execution']=True
    root=Path(__file__).resolve().parents[2]
    if trajectory_file is None:
        entry=read_index(root/'规划文件/当前轨迹').get('prepare')
        if not entry: raise ValueError('Latest result is preview; provide an explicit immutable completed trajectory for this test')
    else: entry={'path':str(trajectory_file.resolve(strict=True))}
    traj=load_trajectory(entry['path'])
    assert not check_motion(traj.positions,traj.time_from_start,traj.velocities,execution=True)[0]
    scaled=copy.deepcopy(traj);scaled.time_from_start=traj.time_from_start/2;scaled.velocities=traj.velocities*2
    assert not check_motion(scaled.positions,scaled.time_from_start,scaled.velocities)[0]
    reasons,metrics=check_motion(scaled.positions,scaled.time_from_start,scaled.velocities,execution=True)
    assert any('PAYLOAD_ACCELERATION_PENDING' in reason for reason in reasons)
    panel=SimpleNamespace(loaded_trajectory=traj,_motion_limit_cache=None)
    assert not ExecutionPreparationPanel._trajectory_motion_limit_reasons(panel)
    panel.loaded_trajectory=scaled
    assert any('PAYLOAD_ACCELERATION_PENDING' in r for r in ExecutionPreparationPanel._trajectory_motion_limit_reasons(panel))
    facts['GUI_blocks_speed_increase_before_confirmation_and_invalidates_numeric_cache']=True
    # Status/schema fixtures only: no fake optimized MAT is generated.
    report_file=Path(entry['path']).with_name(Path(entry['path']).stem+'_report.json')
    modern=json.loads(report_file.read_text()); modern['motion_limits']=limits
    modern['controller_validation']['max_joint_velocity_rad_per_sec']=float(max(metrics['max_velocity_per_joint']))
    modern['controller_validation']['max_joint_acceleration_rad_per_sec2']=float(max(metrics['max_acceleration_per_joint']))
    modern['controller_validation']['acceptance'].update(max_joint_velocity_rad_per_sec=limits['scalar_velocity_rad_s'],
        max_joint_acceleration_rad_per_sec2=limits['scalar_acceleration_rad_s2'])
    state=PlanningWorkflow();state.preparation_complete(Path(entry['path']),modern)
    assert state.stage==PlanningStage.PHYSICAL_VALIDATION_PENDING and state.ready_path is None
    facts['synthetic_new_limit_offline_status_not_misreported_as_algorithm_failure']=True
    for name,mutate in [('false_pass_over_robot_velocity',lambda d:d['controller_validation'].update(max_joint_velocity_rad_per_sec=3.2)),
                        ('false_pass_over_robot_acceleration',lambda d:d['controller_validation'].update(max_joint_acceleration_rad_per_sec2=40.1)),
                        ('false_pass_unbound_profile',lambda d:d['motion_limits'].update(scalar_acceleration_rad_s2=100)),
                        ('legacy_cannot_claim_new_limits',lambda d:d.pop('motion_limits'))]:
        bad=copy.deepcopy(modern);mutate(bad)
        try:PlanningWorkflow().preparation_complete(Path(entry['path']),bad)
        except ValueError:rejected.append(name)
        else:raise AssertionError(name)
    msg=trajectory_to_ros_message(scaled)
    sends=[];done=[];worker=RosWorker('/TEST_ONLY')
    worker.node=SimpleNamespace(send=lambda *a,**kw:sends.append(a))
    worker.goal_done.connect(lambda *args:done.append(args));worker._send_trajectory(msg)
    assert not sends and done and 'MOTION_LIMITS_BLOCKED' in done[-1][1]
    fake=copy.deepcopy(msg);fake.joint_names.reverse();assert message_motion_reasons(fake)
    assert PROFILE.read_bytes()==original and URDF.read_bytes()==urdf_before
    contract=copy.deepcopy(limits);contract['scalar_acceleration_rad_s2']=100
    try:validate_contract(contract)
    except ValueError:rejected.append('modified_report_contract')
    else:raise AssertionError('Unbound contract accepted')
    result=dict(pass_all=True,robot_motion='NO',ROS_node_started=False,real_network_calls=0,
        robot_send_calls=len(sends),limits=limits,negative_cases=rejected,
        baseline_remains_legal=True,uniform_2x_controller_urdf_ceiling_pass=True,
        uniform_2x_execution_blocked_pending_payload=True,
        two_x_max_velocity_per_joint=metrics['max_velocity_per_joint'].tolist(),
        two_x_max_acceleration_per_joint=metrics['max_acceleration_per_joint'].tolist(),
        two_x_tcp_speed_upper_bound_m_s=metrics['tcp_linear_speed_upper_bound_m_s'],
        original_profile_model_trajectory_unchanged=True,**facts)
    (output/'motion_limits_tests.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':run(Path(sys.argv[1]).resolve(),Path(sys.argv[2]) if len(sys.argv)>2 else None)
