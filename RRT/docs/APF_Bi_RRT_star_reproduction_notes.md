# APF-Bi-RRT* reproduction notes

## Source and scope

The implementation follows Section 3.3 of the supplied thesis
`面向船舶狭小空间焊接作业的机械臂避障运动规划研究`, especially equations
(3.12)--(3.19), Algorithm 3.3.4, and Table 3.1. The source PDF is the local
Zotero copy:

`/home/ttbstt/Zotero/storage/WR9CSYAU/面向船舶狭小空间焊接作业的机械臂避障运动规划研究_许家伟.pdf`

This project is a 2-D validation layer. It does not claim to reproduce the
thesis' UR5 6-D collision model or its 3-D sphere/cuboid environment.

## Reproduced equations

The target-bias rule is Eq. (3.12):

```text
xRand = target,          if pRand <= m
xRand = uniform sample,  otherwise
```

The dynamic threshold uses Eq. (3.13):

```text
m = pMax - (pMax - pMin) * (Vobs / Vtotal)^k
```

The thesis defines `Vobs/Vtotal` as a 3-D volume ratio. In this 2-D project it
is implemented as the exact union area of the axis-aligned rectangles divided
by the map area. Overlapping rectangles are counted once and are clipped to
the map bounds.

The attraction and improved repulsive potentials follow Eqs. (3.14), (3.17),
and (3.18):

```text
Fatt = katt_mot * (target - p)
katt_mot = katt_min + lambda * norm(target - p)

Urep* = 0.5 * krep * (1/d - 1/d0)^2 * norm(p - target)^a,  d <= d0
Urep* = 0,                                                   d > d0
```

`d` is the distance to the nearest point on the clearance-expanded rectangle,
not the distance to its centre. The implementation uses the analytically
derived vector `Frep = -gradient(Urep*)`, including the gradient of the goal
distance factor. The thesis gives the scalar potential but does not print the
complete vector derivative, so that vector expression is explicitly a
derivation rather than a verbatim paper formula.

## Parameters and assumptions

Table 3.1 gives the following paper-scale values. The shared project defaults
remain normalized for the existing 0--100 2-D benchmarks (`stepSize=4` and
`nearRadiusMax=25`); the paper-scale values are set explicitly in the scale
test below:

| Parameter | Value | Status |
| --- | ---: | --- |
| `stepSize` (paper scale) | 8 | paper value |
| `nearRadiusMax` | 10 | paper value |
| `maxIterations` | 2000 | paper value |
| `apfKattMin` | 0.2 | paper value |
| `apfLambda` | 0.05 | paper value |
| `apfKrep` | 0.1 | paper value |
| `apfInfluenceDistance` | 15 | paper value |
| `apfGoalBiasMax` | 0.30 | engineering assumption; not specified in extracted text |
| `apfGoalBiasMin` | 0.05 | engineering assumption; not specified in extracted text |
| `apfGoalBiasExponent` | 1.0 | engineering assumption; not specified in extracted text |
| `apfRepulsiveExponent` | 2.0 | engineering assumption; not specified in extracted text |
| `bsplineDegree` | 3 | engineering assumption; the thesis gives the basis form but no degree |

The project-normalized direct demo overrides only `stepSize` to 1 on the
shared 0--100 map. This is needed because the map contains wall-like
obstacles and the literal paper-scale step is too coarse for the 2-D demo.
Paper-scale tests set the 0--150 map, start `[20 20]`, goal `[120 120]`,
`stepSize=8`, and `nearRadiusMax=10` explicitly.

## Search and post-processing semantics

The start tree uses `goal` as its APF target; the goal tree uses `start`.
The sampled `xRand` is used only for `Nearest`. The new point is generated
from the APF force at `xNear`, with no random-direction or GD fallback. A
near-zero force rejects the iteration and increments `apfZeroForceCount`.

After insertion, the implementation uses the existing RRT* ChooseParent,
Rewire, and descendant-cost propagation helpers. Paper mode checks the other
tree's nearest node and stops at the first validated connection. Extended mode
may use `best_near` and continues exactly `optimizeIterations` iterations
after the first solution.

The raw connection path is then processed independently:

1. Greedy goal-to-start line-of-sight simplification.
2. Clamped uniform B-spline generation with exact endpoints.
3. Collision, endpoint, finite-value, and map-bound validation of every sampled
   smoothing segment.

If smoothing is invalid, `result.path` falls back to the validated simplified
polyline and retains `smoothedPathValid=false` plus the diagnostic smoothed
path. Search and post-processing collision checks are reported separately.

## Reproducibility commands

Direct demo:

```matlab
addpath(genpath(pwd));
result = APF_Bi_RRT_star();
```

Project-normalized, non-animated run:

```matlab
config = defaultRRTConfig();
config.showAnimation = false;
config.apfPaperMode = true;
config.stepSize = 1;
result = APF_Bi_RRT_star(config);
```

Paper-scale, non-animated run:

```matlab
config = defaultRRTConfig();
config.showAnimation = false;
config.apfPaperMode = true;
config.mapBounds = [0 150 0 150];
config.start = [20 20];
config.goal = [120 120];
config.stepSize = 8;
config.nearRadiusMax = 10;
result = APF_Bi_RRT_star(config);
```

The exact result is environment- and obstacle-dependent. No formal 100-run
benchmark is part of Phase 9.
