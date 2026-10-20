"""Check actual isolated experiment evidence; never publish to the HMI index."""
import hashlib
import json
from pathlib import Path
import statistics

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def read(name):
    return json.loads((HERE / name).read_text())


def timing(pattern):
    rows = [read(file.name) for file in sorted(HERE.glob(pattern))]
    assert len(rows) == 3, (pattern, len(rows))
    seconds = [row['elapsed_sec'] for row in rows]
    return dict(n=len(rows), QP_converged=sum(row.get('exitflag', 0) > 0 for row in rows),
                seconds=seconds, mean_sec=statistics.mean(seconds),
                sample_std_sec=statistics.stdev(seconds),
                exitflags=[row.get('exitflag') for row in rows])


protected = {
    'matlab仿真/optimize_hmi_cubic_node_velocities.m': '850a632c0337b6d1f49257986fb12afb709cbeb8b3dcf2d8d8250f0c9280ee82',
    'matlab仿真/optimize_hmi_controller_continuity.m': 'bb676dc571b4c059c909b6fce4e3b4142108a194e9364fa04e9eccb01ebb1bdf',
    'matlab仿真/validate_accp_controller_interpolation.m': '486d15bffcd1b669509f515bc8b075c76644773be3797791b6205981df791451',
    'robot_motion_limits.json': '8eaa422c8c8856ecc5f7745a8add8716de6cbe6983813a4d6d1afa5651dd03a3',
    'ur5_hmi/config/hmi_config.yaml': 'b4b77f1cf92186ce0299dd6219dd99cd24f996c657451ca9e0e5ab36cb771234',
}
for name, expected in protected.items():
    assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == expected, name
assert read('transformation_tests.json')['pass'] is True
nodes = read('fixed_node_audit.json')
assert nodes['all_bad_nodes_are_inserted']
assert len(nodes['nodes_above_0p5mm']) == 178
assert nodes['original_node_position_error_max_mm'] < .5
base = read('initial_raw_interior-point-convex.json')
improved = read('initial_scaled_eliminated_interior-point-convex.json')
assert base['exitflag'] == improved['exitflag'] == -2
assert read('initial_raw_linprog.json')['exitflag'] == -2
relaxed = read('last_scaled_eliminated_interior-point-convex_pos0.650_ori0.460.json')
assert relaxed['exitflag'] > 0 and relaxed['original_constraints_pass']
assert relaxed['official_QP_inequality_violation'] > 0
assert relaxed['exact_formal_precision_pass'] is False
assert relaxed['formal_final_collision'] == 'NOT_RUN'
relaxed_runs = [relaxed] + [read(file.name) for file in sorted(HERE.glob(
    'last_scaled_eliminated_interior-point-convex_pos0.650_ori0.460_repeat*.json'))]
assert len(relaxed_runs) == 3
assert all(row['exitflag'] > 0 and not row['exact_formal_precision_pass'] for row in relaxed_runs)
assert not list(HERE.rglob('trajectory_execute_ready.*'))
report = dict(
    scope='FROZEN_CURRENT_REQUEST_DIAGNOSTICS_NOT_PRODUCTION_REPLACEMENT',
    production_unchanged=True, protected_sha256=protected,
    fixed_threshold_raw=timing('initial_raw_interior-point-convex_repeat*.json'),
    fixed_threshold_equivalent_scaled=timing('initial_scaled_eliminated_interior-point-convex_repeat*.json'),
    Hessian_condest=read('initial_attribution.json')['matrix_stats'],
    fixed_node_inconsistency=dict(original_node_count=nodes['original_node_count'],
        inserted_node_count=nodes['inserted_node_count'], bad_inserted_nodes=178,
        original_node_max_mm=nodes['original_node_position_error_max_mm'],
        inserted_node_max_mm=nodes['max_fixed_node_TCP_error_mm']),
    threshold_sensitivity=dict(internal_position_mm=.65, internal_orientation_deg=.46,
        QP_success_count=3, trials=3, formal_precision_pass_count=0,
        sync_mm=[row['exact_tracking']['position_max_mm'] for row in relaxed_runs],
        contour_mm=[row['exact_tracking']['contour_max_mm'] for row in relaxed_runs]),
    minimum_initial_linearized_position_bound_mm=read('minimum_position_attribution.json')['minimum_position_bound_mm'],
    conclusion='REPAIR_REFERENCE_AND_INSERTED_NODE_CONSISTENCY_BEFORE_RELAXING_ANY_GATE',
    general_success_rate_claimed=False, executionReady=False, robot_motion='NO')
(HERE / 'comparison_summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
print('FROZEN_QP_COMPARISON_VERIFIED')
