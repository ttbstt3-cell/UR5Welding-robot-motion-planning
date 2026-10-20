# 2-D RRT Planner Phase 1--3

The original `RRT_basic_animation.m`, `Bi_RRT_animation.m`,
`collisionFree.m`, and `collisionFreeJointEdge.m` remain at the project root.
The new planners use the same root-level `collisionFree.m` and do not modify
the UR5/ROS2/CoppeliaSim chain.

## Run the new planners directly

From the project root, these commands work without manually adding any
subdirectories:

```matlab
RRT_connect
RRT_star
Q_RRT_star
GD_RRT_star
Classified_RRT_star
APF_Bi_RRT_star
```

No-argument calls open the animated demo and stop at the first feasible path.
The root-level launcher files add their own `core/` and `planners/`
dependencies. Benchmark calls can pass `config.optimizeIterations > 0` to
continue the RRT* optimization phase after the first solution. Its exact
meaning is:

```text
optimizationIterations = N
=> run exactly N additional iterations after firstSolutionIteration,
   unless maxIterations is reached first.
```

## Run in benchmark mode

```matlab
addpath(genpath('/home/ttbstt/jixiebi/RRT'));

config = defaultRRTConfig();
config.showAnimation = false;   % required for benchmark mode
config.seed = 1;

connectResult = RRT_connect(config);
starResult = RRT_star(config);
```

Set `config.showAnimation = true` for the single-run visualization. Plotting,
`drawnow`, and `pause` are outside the planner timing accumulator.

## Implemented in Phase 2--3

- `RRT_connect`: alternating bidirectional trees with a repeated CONNECT loop
  (`ADVANCED` until `REACHED` or `TRAPPED`). It stops at the first feasible
  connection.
- `RRT_star`: `RandomSample`, `Nearest`, `Steer`, radius-based `Near`,
  `ChooseParent`, and `Rewire`. It continues after the first feasible path for
  `config.optimizeIterations` additional iterations, subject to
  `config.maxIterations`.
- `Bi_RRT_star`: two alternating RRT* trees, cost-aware near-tree connection,
  shared `ChooseParent`, shared `Rewire`, and descendant cost propagation.
  Its connection cost includes the start-tree root branch, the inter-tree
  edge, and the goal-tree root branch.
- `Informed_RRT_star`: standard RRT* before the first solution, then direct
  sampling inside the prolate ellipse defined by `c_min` and the current
  `c_best`. It reports uniform/informed sample counts and ellipse-boundary
  rejection counts.
- `Q_RRT_star`: standard single-tree RRT* with ancestor candidates and
  Quick-Rewire. `config.qDepth` controls how many parent levels are added to
  the candidate set; supported values are 1, 2, and 3 (the default is 2).
  Q-RRT* deliberately uses uniform sampling and does not apply goal bias,
  informed sampling, Gaussian sampling, APF, or bidirectional growth.
- `GD_RRT_star`: Zhao-style Gaussian/Depth adaptive RRT*. It reuses the
  Q-RRT* ancestor mechanism, updates collision frequency every ten base
  expansions, and supports paper-literal initial-path mode plus an explicitly
  labeled extended optimization mode.
- `Classified_RRT_star`: Xiong-style C-RRT* with four-way node
  classification, a goal-tree branch set, reverse-backtracking ChooseParent,
  sequential Rewire, and random offset recovery after a blocked extension.
- `APF_Bi_RRT_star`: independent APF-Bi-RRT* with dynamic target bias,
  improved APF direction, bidirectional RRT* growth, greedy simplification,
  and collision-validated clamped B-spline smoothing. `apfPaperMode=true`
  stops at the first connection; extended mode uses the shared extra-iteration
  benchmark semantics. Reproduction details and paper/project scaling notes
  are in `docs/APF_Bi_RRT_star_reproduction_notes.md`.

RRT* uses the standard two-dimensional radius form

```text
r_n = min(nearGamma * sqrt(log(n) / n), nearRadiusMax)
```

The engineering defaults are `nearGamma = 30` and `nearRadiusMax = 25`; both
are explicit configuration fields so benchmark experiments can report them.

Every planner result contains the common fields requested for comparison,
including `firstPath`, `firstSolutionTime`, `collisionChecks`, and
`rewireCount`. Optimization-capable planners also return
`optimizationIterationsRequested` and `optimizationIterationsExecuted`.
Planner-side collision calls are counted while all geometric checks are
delegated to the shared continuous `collisionFree.m`.

This direct-demo versus benchmark convention is shared by the standard RRT*
family planners: direct demos stop at the first feasible path; benchmark calls
continue for the configured additional optimization budget. GD-RRT* is the
documented exception: its default `gdPaperMode=true` is reserved for the
paper's initial-path reproduction, while `gdPaperMode=false` explicitly opts
into the project's extended optimization convention.

## Verification

Run the phase test from the project root:

```matlab
addpath(genpath(pwd));
test_phase1_phase3;
```

Run the Bi-RRT* test with:

```matlab
test_phase4_bi_rrt_star;
```

Run the Informed RRT* test with:

```matlab
test_phase5_informed_rrt_star;
```

Run the Q-RRT* test with:

```matlab
test_phase6_qrrt_star;
```

Run the GD-RRT* test with:

```matlab
test_phase7_gd_rrt_star;
```

Run the Classified-RRT* test with:

```matlab
test_phase8_classified_rrt_star;
```

Run the APF-Bi-RRT* test with:

```matlab
test_phase9_apf_bi_rrt_star;
```

The Phase 5 test also writes
`results/phase5_rrtstar_vs_informed_convergence.png` at 300 dpi.

The Phase 6 test writes
`results/phase6_rrtstar_informed_qrrt_convergence.png` at 300 dpi and checks
ancestor depths 1/2/3, cycle prevention, cost consistency, and exact
optimization budgets of 0/100/500/1000 iterations.

Phase 7 uses the paper-literal interpretation of `TargetSample`: when
`alpha=false`, the sample is exactly `config.goal`; when `alpha=true`, the
Gaussian mixture is used. It does not silently add a goal-bias probability.
`gdPaperMode=true` stops at the first validated path, matching the paper's
initial-path algorithm. Set `gdPaperMode=false` only for the project's
explicit extended optimization experiments. Full ambiguity and equation
notes are in
`docs/GD_RRT_star_reproduction_notes.md`.

Phase 8 records its paper interpretation in
`docs/Classified_RRT_star_reproduction_notes.md`. `crrtPaperMode=true` uses a
fixed total horizon (`crrtPaperTotalIterations`, default 1000) for the paper's
initial/within-horizon comparison. `crrtPaperMode=false` uses the project's
first-solution plus `optimizeIterations` benchmark convention. The Phase 8
test writes both an after-first-solution convergence figure and an absolute
1000-iteration paper-horizon figure.

Phase 9 records the APF equations, paper parameters, explicit engineering
assumptions, and paper-scale versus project-normalized settings in
`docs/APF_Bi_RRT_star_reproduction_notes.md`. Its test checks the dynamic bias,
union-area occupancy, APF finite-difference gradient, opposite tree targets,
greedy simplification, B-spline safety validation, raw-path comparison, and
exact extended optimization budget. It is not a formal 100-run benchmark.
The checked-in demonstration figure is
`results/phase9_apf_birrt_paths.png`.

The test uses fixed seeds, verifies that Basic RRT and Bi-RRT files are still
present, validates every returned path edge, and checks RRT* rewiring and
reproducibility.

## Phase 12A: offline UR5 6-DOF migration

Phase 12A-1 migrates only four 2-D search structures to the current UR5 joint
space: `RRTConnectJoint`, `BiRRTStarJoint`, `ClassGoalLiteJoint`, and
`ClassifiedRRTStarJoint`. APF is intentionally not migrated yet because a
2-D point-to-rectangle potential is not equivalent to a robot link/tool
configuration-space potential.

All four planners use the common `jointspace/` metric, shortest revolute
angle differences, explicit UR5 joint limits, and the same
`collisionFreeJointEdge` interface. The first implementation uses a 2-degree
discrete interpolated edge resolution and does not claim continuous
swept-volume checking. It plans only free-space transitions around the weld:
`qHome -> qPreWeld` and `qPostWeld -> qHome`; the 80-point weld segment stays
with the existing continuous IK/ACCP path.

Run the six-case functional validation with:

```matlab
addpath(genpath('/home/ttbstt/jixiebi/RRT'));
report = run_phase12a_functional;
```

The canonical UR5 scene and weld states are defined in
`/home/ttbstt/jixiebi/AGENT_HANDOFF.md`, not in the 2-D map files. The latest
validated functional run is stored in
`results/phase12a_functional.mat` and
`results/phase12a_functional_summary.csv`: 24/24 runs succeeded and 24/24
paths passed independent endpoint, limit, and edge-collision validation.

For later collision-resolution sensitivity checks, the functional runner also
persists every qPath in `report.paths`. Use
`benchmark/validate_phase12a_resolution.m` to recheck those saved paths at
2, 1, and 0.5 degrees; this script never calls a planner. The original
Phase 12A-1 artifact predates path persistence, so it must not be presented as
a completed resolution-sensitivity experiment.

The archival rerun is stored separately in
`results/phase12a_functional_archival.mat`; its 24-row comparison against the
historical summary passes after allowing `1e-7` CSV round-off for path cost.
The saved-path resolution check found `70/72` PASS rows: the
`C05_Home_to_Point1 / RRT-Connect / seed=5` path passes at 2 degrees but fails
at both 1 and 0.5 degrees on an intermediate environment-collision state.
This is evidence of discrete-resolution leakage, not a continuous safety
proof. Do not start the 480-run Phase 12A-2 pilot until the formal edge
resolution is raised and revalidated.

Phase 12A-1.5 then replanned the same 24 cases at 1 degree and independently
validated the saved paths at 0.5 degree. All 24 planners succeeded, but only
23/24 paths passed the 0.5-degree check: the same C05/RRT-Connect diagnostic
case still produced an intermediate environment collision on the newly found
1-degree path. The 1-degree rerun is stored in
`results/phase12a_resolution1deg_functional.mat`; its validation and
1-vs-2-degree comparison are stored under the corresponding
`phase12a_resolution1deg_*` result files. The 480-run pilot remains blocked.
