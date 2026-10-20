"""Genuine full-task report -> production HMI contract, no ROS or motion."""
from __future__ import annotations
import copy
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gui.planning_workflow import PlanningWorkflow, PlanningStage
from gui.offline_planning_report import render_report
from trajectory.trajectory_loader import load_trajectory


def run(report_file: Path, output: Path):
    report = json.loads(report_file.read_text())
    request = json.loads(report_file.with_name('request.json').read_text())
    assert report['hybrid_velocity_candidate']['source'] == 'C_ACCURACY_FALLBACK'
    assert report['hybrid_velocity_candidate']['nonoptimal_fallback'] is True
    if report['success'] is False:
        # Genuine full-task failure after C recovery must stay a failure.
        raw = Path(request['rawFile'])
        assert report['velocity_optimization']['pass'] is False
        assert report['executionReady'] is False and not report['continuous_planning_complete']
        assert 'controller_validation' not in report  # NOT RUN, not a collision PASS.
        rejected = []
        for label in ('genuine full task failure', 'partial success disguised as complete'):
            case = copy.deepcopy(report)
            if label.startswith('partial'):
                case.update(success=True, continuous_planning_complete=True,
                            continuous_planning_status='PASS', stage='PHYSICAL_VALIDATION_PENDING')
            state = PlanningWorkflow(raw_path=raw)
            state.begin_preparation()
            try:
                state.preparation_complete(raw, case)  # Existing path: absence is not the rejection reason.
            except ValueError as exc:
                state.fail('prepare', str(exc))
                assert report['failure_reason'] in str(exc)
            else:
                raise AssertionError('Partial task accepted: ' + label)
            assert state.stage == PlanningStage.CONTINUOUS_PLANNING_FAILED
            assert state.raw_path == raw and state.ready_path is None and state.optimized_path is None
            rejected.append(label)
        assert not report.get('artifacts')
        assert not report_file.with_name('offline_optimized_trajectory.mat').exists()
        assert not report_file.with_name('trajectory_execute_ready.mat').exists()
        output.write_text(json.dumps(dict(pass_all=True, full_task_pass=False,
            report=str(report_file), full_task_failure=report['failure_reason'],
            candidate_recovery_pass=True, HMI_correctly_failed_closed=True,
            final_dense_collision_audit='NOT_RUN', rejected_partial_pass_cases=rejected,
            executionReady=False, robot_motion='NO'), indent=2))
        print('HYBRID_FALLBACK_HMI_FAILURE_CONTRACT_PASS')
        return
    path = Path(report['artifacts']['trajectory_mat'])
    state = PlanningWorkflow(raw_path=Path(request['rawFile']))
    state.begin_preparation()
    state.preparation_complete(path, report)
    assert state.stage == PlanningStage.PHYSICAL_VALIDATION_PENDING and state.ready_path is None
    assert report['hybrid_velocity_candidate']['nonoptimal_fallback'] is True
    assert report['hybrid_velocity_candidate']['source'] == 'C_ACCURACY_FALLBACK'
    trajectory = load_trajectory(path)
    assert not trajectory.metadata['executionReady'] and trajectory.metadata['diagnostic_only']
    assert trajectory.metadata['weld_velocity_candidate']['source'] == 'C_ACCURACY_FALLBACK'
    assert trajectory.metadata['weld_velocity_candidate']['nonoptimal_fallback']
    view = render_report(report_file).read_text()
    assert 'C_ACCURACY_FALLBACK' in view and 'NOT a D optimizer success' in view
    cases = []
    for key in ('collision_pass', 'joint_limits_pass', 'analytic_extrema_pass', 'complete'):
        bad = copy.deepcopy(report); bad['controller_validation'][key] = False
        cases.append((key, bad))
    for key in ('max_tcp_contour_error_mm', 'max_tcp_time_sync_error_mm', 'max_tcp_orientation_error_deg'):
        bad = copy.deepcopy(report); bad['controller_validation'][key] = .500001
        cases.append((key, bad))
    for key in ('max_joint_velocity_rad_per_sec', 'max_joint_acceleration_rad_per_sec2'):
        bad = copy.deepcopy(report)
        bad['controller_validation'][key] = report['controller_validation']['acceptance'][key] + 1e-5
        cases.append((key, bad))
    bad = copy.deepcopy(report); bad['offline_checks']['interface_c2'] = False
    cases.append(('interface C2 fail', bad))
    bad = copy.deepcopy(report); bad['version_validation']['pass'] = False
    cases.append(('version mismatch', bad))
    bad = copy.deepcopy(report); bad['executionReady'] = True
    cases.append(('execution claim', bad))
    bad = copy.deepcopy(report); bad['controller_validation']['sample_count'] = 4999
    cases.append(('incomplete audit', bad))
    bad = copy.deepcopy(report); bad['controller_validation']['max_tcp_time_sync_error_mm'] = float('nan')
    cases.append(('NaN', bad))
    rejected = []
    for label, bad in cases:
        failed = PlanningWorkflow(raw_path=Path(request['rawFile']))
        try:
            failed.preparation_complete(path, bad)
        except ValueError as exc:
            failed.fail('prepare', str(exc))
        else:
            raise AssertionError('Invalid report accepted: ' + label)
        assert failed.stage == PlanningStage.CONTINUOUS_PLANNING_FAILED
        assert failed.raw_path == Path(request['rawFile']) and failed.ready_path is None
        rejected.append(label)
    output.write_text(json.dumps(dict(pass_all=True, report=str(report_file),
        fallback_provenance_retained=True, HMI_offline_pass_not_execution_ready=True,
        rejected_bad_full_task_reports=rejected, executionReady=False, robot_motion='NO'), indent=2))
    print('HYBRID_FALLBACK_HMI_CONTRACT_PASS', len(rejected))


if __name__ == '__main__':
    run(Path(sys.argv[1]), Path(sys.argv[2]))
