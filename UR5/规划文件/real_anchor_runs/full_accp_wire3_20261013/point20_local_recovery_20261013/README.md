# Point19 -> Point20 local recovery diagnostic

This directory contains an independent diagnostic for the fixed real scene
wire_collision_shorten_mm=3. It reads the saved ACCP MAT, existing Point20
audit, and current real calibration files. It does not modify formal ACCP,
CBi-RRT*, TCP, gun/wire/workpiece geometry, collision filters, trajectory files,
or weld_anchor_points_real.json. No robot motion was sent.

## Snapshot check

scene_snapshot_check.json reports CURRENT:

- TCP calibration, real gun geometry, gun mount, real workpiece, and calibrated
  URDF SHA-256 values match the failed ACCP report;
- wire_collision_shorten_mm=3 in the saved scene and ACCP options;
- execution_authorized=false.

A mismatch returns STALE and aborts recovery.

## Original Point19 -> Point20

The original connection remains NO_CONNECTED_CANDIDATE:

- 258 IK calls, 258 numeric successes, 230 pose-valid;
- 102 direction rejects and 84 workpiece node-collision rejects;
- 6 continuity rejects: 2 joint-step and 4 edge-collision;
- 0 final connected candidates;
- all four edge failures are WeldingGunWire <-> WorkpieceCollision_bottom;
- the main node-level blocker is WeldingGunMount <-> WorkpieceCollision_wall_x1.

## 101-sample edge audit

point19_point20_edge_audits.json contains 101 samples (including endpoints) for
each of the three edge candidates. Every sample performs complete node
environment/self-collision checks:

| candidate | first collision fraction | first collision pair | last safe fraction | colliding samples |
|---|---:|---|---:|---:|
| edge 1 | 0.05 | WeldingGunWire <-> WorkpieceCollision_bottom | 1.00 | 91 |
| edge 2 | 0.24 | WeldingGunWire <-> WorkpieceCollision_bottom | 1.00 | 53 |
| edge 3 | 0.06 | WeldingGunWire <-> WorkpieceCollision_bottom | 1.00 | 91 |

Each record includes the first colliding 6-joint vector, TCP 4x4 pose, the last
safe pose, all pairs, and nearest positive distance. MATLAB checkCollision does
not expose signed penetration depth, so that field is explicitly unavailable.

## Two joint-step candidates

point19_point20_step_candidate_audit.json reports:

| candidate | max joint change | over-15-degree joint changes | local Cartesian recovery |
|---|---:|---|---|
| step 1 | 31.5835 deg | J1=20.8439, J5=31.5835 | legal diagnostic branch found |
| step 2 | 17.6808 deg | J3=17.6808, J5=15.0365 | legal diagnostic branch found |

The branches are obtained by new Cartesian targets and fresh IK, not by splitting
the original joint line and checking endpoints only.

## Local Cartesian recovery

local_recovery_summary.json reports DIAGNOSTIC_PASS, but executionReady=false:

- 25%, 50%, and 75% Cartesian targets with linear position interpolation and
  quaternion SLERP orientation;
- fresh numeric IK at every new target;
- 101 samples on every adjacent edge with full workpiece, gun, wire, and robot
  self-collision checks;
- first successful branch: 3 new nodes, 20 IK calls, 4 edge checks;
- maximum joint change 3.5049 deg;
- maximum node TCP position error 0.000027 mm;
- minimum available positive collision gap 0.4034 mm;
- all nodes and edges pass;
- no official trajectory or real anchor is written.

The remaining endpoint branches were also audited; both original step-limit
candidates independently found legal local Cartesian branches. Detailed stage
failure reasons remain in stage_statistics.failure_reason.

## Files

- point19_point20_edge_audits.json: three 101-sample edge audits;
- point19_point20_step_candidate_audit.json: per-joint step deltas and branch result;
- local_recovery_summary.json: original/local comparison and cost statistics;
- full_accp_report.json: read-only copy of the complete failed ACCP report;
- scene_snapshot_check.json: TCP/gun/workpiece/URDF/wire snapshot check;
- MATLAB visualization: UR5/matlab仿真/visualize_accp_point20_edge_audit.m;
- diagnostic entry point: UR5/matlab仿真/diagnose_accp_point19_point20_local_recovery.m.

Example:

~~~
visualize_accp_point20_edge_audit([], [], 1)
~~~

The viewer shows Point19, the first collision interpolation pose, and Point20,
including real gun, wire and workpiece collision geometry. It is offline only.

