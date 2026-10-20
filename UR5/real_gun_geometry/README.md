# Real welding-gun geometry (independent offline review)

This directory contains the user-supplied replacement STL and a reproducible
audit/collision-proxy package.  It is intentionally **not active** yet.

## Source audit

`stl_audit.json` reports the source mesh without modifying it.  The binary STL
contains 1,251,936 triangles and has raw bounds
`[38.13095, 0.36275, 7.91431]` to `[390.53653, 196.85893, 73.91431]`.
The size is consistent with millimetres, but the unit decision remains a
human-confirmed convention; the source file is never silently centred,
rotated, or rewritten.

The mesh preview contains the L-shaped mounting bracket, so the supplied STL
is treated as one complete visual assembly.  The exact flange mounting datum
is not encoded in the STL and therefore remains unset.

## Collision status

The installed CoppeliaSim 4.10 includes `libsimConvex.so` and
`simConvex.vhacd`. The former 16 slab review proxies are archived in
`archive/slab_review_proxy_not_vhacd.json`; they are NOT V-HACD and are NOT
accepted real collision geometry. `tools/decompose_real_gun_coppelia.py`
uses the genuine plugin and reuses `export_scene_geometry.mesh_record` to
export each compound component separately. Exact parameters and source hash
are recorded in `vhacd_collision_export.json` after decomposition completes.

The manifest must not be changed to `ACTIVE` until a human confirms:

1. the source-frame-to-URDF-flange rigid transform (including any confirmed
   J6 rotation, with no automatic centring),
2. the V-HACD result or an accepted collision decomposition, and
3. review of self/environment collisions with the confirmed mounting pose.

Wire precision is independent: the unreliable nearest-old-hull-vertex wire is
`DISABLED_UNVERIFIED` in REAL geometry, not silently inherited or counted as a
collision PASS. This does not block gun/bracket visualization or decomposition.
TCP parameters remain entirely independent and unchanged.

## HMI manual alignment

After genuine V-HACD export, restart HMI and open **焊枪安装调整**. The new STL
is shown even while installation is unconfirmed. Its source origin is initially
placed at the flange ONLY AS AN EXPLICIT PREVIEW, not a measured installation.
Use STL source-basis R/P/Y, J6-axis ±90°/1°/0.1° controls and flange-local XYZ
translation (mm) to align the mounting interface. All components share the
same `T_flange_source`; red LIVE TCP and joint feedback are never changed.
Hide Robot and enable Collision to inspect the complete gun+L bracket.

Save a draft or confirm the physical mount only after measurement. The record
is source-hash-bound; old-hull mount records are rejected for this STL. Saving
does not automatically activate MATLAB geometry or execute a trajectory.
Previously saved records are backed up under `mount_versions/`.

## MATLAB / tests

`sync_ur5_joint_states.m` remains the FROZEN SIMULATION builder, even when a
real manifest is active. `apply_real_gun_geometry.m` explicitly replaces only
gun visuals/collisions in a calibrated REAL URDF scene exposing `flange`.
Each component has its own `collisionMesh(vertices)`; no combined giant hull.
The source-to-mount matrix is derived from the exact flange-to-mount chain.
Unconfirmed transforms require `previewOnly=true`; no planning approval.

`check_real_gun_geometry_offline.m` and the Python review/smoke tools compare
10 identical joint poses, Saved RealHome self/environment collisions, and
HMI transforms. Reports and ±90° screenshots stay in this folder. Results
with an unconfirmed mount or a DRAFT workpiece are diagnostics, not acceptance.

All old real trajectories remain STALE. Frozen JSON/MAT/TTT/URDF/TCP files,
ROS2 control, robot joints and planner algorithms are not overwritten.

## Superseded minimum-X face candidate (2026-10-12)

The candidate below was rejected: it selected a circular gun-body face,
not the installation plate. Its numerical centre residual did not verify
the physical interface. Do not restore it as the current mounting datum.

The STL has a candidate circular mounting face on the minimum-X plane. Its
geometric centre is `(38.130955, 166.858932, 40.914322) mm`; the candidate
transform maps that point to the URDF `flange` origin and keeps the field-
confirmed right-hand J6 roll at `+90 deg`. The centre residual is `0.00000057
mm` numerically, with a proper rigid rotation (determinant 1).

This is stored as `../real_gun_mount_calibration.json` with status `DRAFT`.
The previous confirmed record is backed up under
`mount_versions/before_face_center_alignment_20261012_093442.json`.
The draft is a geometric preview only: the bolt pattern, contact face and
physical flange seating still need manual confirmation. It has not been
applied to MATLAB REAL planning, and it does not modify the live TCP or robot.
See `flange_face_alignment_review.json` for the calculation.

## Current four-hole alignment draft (2026-10-12)

The actual back contact face is at source X=49.23116 mm, with a 63 mm
outer disc centred at Y=31.858934585, Z=40.914309240 mm. Four 6 mm bolt
holes form a square on a 50 mm pitch circle. The matching four holes
were extracted from the URDF wrist3 mesh in flange coordinates.
Circle fits and rigid registration give RMS=0.031352 mm, max=0.040254 mm
hole-centre disagreement, and zero contact-plane gap. These are mesh
comparisons, not measured real installation precision. The low polygon
robot face is X=0.025055 mm in flange coordinates.

The fourfold pattern cannot determine the unique installation azimuth;
the branch near the user's +90 degree field confirmation was selected
(effective angle 89.979188 degrees). Source STL is unchanged. All 26
collision components use the exact visual source-to-flange transform.
The result is saved as DRAFT for HMI preview only; no MATLAB activation.
The rejected draft is backed up as
`mount_versions/before_hole_alignment_20261012.json`.

Evidence: `mount_hole_alignment.json`, `mount_hole_inspection.png`,
`mount_holes_aligned.png`, `hmi_hole_aligned_gun.png`, and
`hmi_hole_aligned_flange_closeup.png`. Reproduce offline with
`ur5_hmi/tools/align_gun_mount_holes.py` and
`ur5_hmi/tools/preview_hole_aligned_gun.py`.

## Completed offline review (2026-10-10)

The final run used the **full original STL**, not the simplified experimental
collision input: 26 V-HACD components, 853 vertices, 1,602 triangles;
decomposition time 12.308987539 s. Parameters: resolution 1,000,000,
concavity 0.0025, plane/hull downsampling 4, max vertices 64,
alpha/beta 0.05, min volume 0.00001, PCA off, voxels on.

The 20,000-source-vertex coverage diagnostic is 99.91% inside the union;
maximum normalized half-space violation is 0.885761 mm. This is NOT a
complete surface-distance guarantee or a conservative-envelope PASS. Local
undercoverage must be reviewed before accepting this collision model.
The source has 12 combined boundary/non-manifold edges; no automatic repair.

For 10 identical joint poses, HMI/MATLAB component-center maximum difference
is 2.83e-13 mm. At the explicit unconfirmed source-at-flange transforms
0/+90/-90 degrees, Saved RealHome had no detected self/environment collision.
This cannot validate actual installation or explain the former RealHome
collision: the physical mount is unknown and the workpiece record is DRAFT.

Artifacts:

- `replacement_report.json`: aggregate results and seven frozen-file hashes.
- `geometry_review.json`, `matlab_review_+0.json`, `matlab_review_+90.json`,
  `matlab_review_-90.json`, `matlab_geometry_review.log`: numerical evidence.
- `hmi_geometry_smoke.json`: separate visual/collision layers, rigid transforms,
  LIVE-state isolation and actor reload checks.
- `hmi_panel_smoke.json`: 100/125/150% UI control smoke tests; ROS startup mocked.
- `hmi_new_gun_preview_+0.png`, `hmi_new_gun_preview_+90.png`,
  `hmi_new_gun_preview_-90.png`: rendering-only installation previews.
- `ur5_real_gun_alignment_preview.ttt`: independent, stopped UR5 scene copy
  with full new visual and compound hull; old gun shapes/control script removed.
  Do not execute it as a validated real scene.

The long initial attempts were blocked by exporting huge visual arrays over
the remote API. Bounding-box-only inspection now avoids that transfer; only
the small convex components use the existing JSON mesh exporter. The bounding
box reorientation changes metadata only, never mesh vertices/shape frame.
