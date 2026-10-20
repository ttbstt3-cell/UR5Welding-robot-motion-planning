# GD-RRT* reproduction notes

This document records the implementation choices for the Phase 7
reproduction of Zhao et al., 2026, *An improved RRT* path planning algorithm
combining Gaussian distributed sampling and depth strategy for robotic arm of
fruit-picking robot*.

## 1. TargetSample ambiguity

Algorithm 1 is interpreted literally in Phase 7:

```text
alpha == false  ->  TargetSample() returns xGoal
alpha == true   ->  Gaussian mixture sampling
```

The surrounding prose also says target bias and Gaussian sampling are used
with probability.  Those statements are not identical to the pseudocode.
This implementation does not silently invent a target-bias probability.  The
configuration records `gdSamplingInterpretation = 'paper_literal'`, and the
result reports separate `targetSampleCount` and `gaussianSampleCount` fields.
A probabilistic target-bias variant is intentionally not part of Phase 7.

## 2. p and pcf

Every ten base expansion attempts form one window.  `p` is the number of
collision-free base expansion edges in that window.  Therefore:

```text
pcf = (10 - p) / 10
```

`pcf` is collision frequency.  A larger value means that the local region has
more blocked expansion attempts.  It is not the collision-free success rate.

The discrete update rule uses `pcf > 0.5` for the Gaussian branch.  Since a
ten-attempt window produces only 0.0, 0.1, ..., 1.0, this is equivalent to the
paper's `pcf >= 0.6` wording.

## 3. ParamUpdate

With `baseStep = config.stepSize`:

| pcf | alpha | current step | current Depth |
|---:|:---:|---:|---:|
| `pcf <= 0.2` | false | `2*baseStep` | 1 |
| `0.2 < pcf <= 0.5` | false | `baseStep` | 2 |
| `pcf > 0.5` | true | `baseStep` | 3 |

The initial state is `alpha=false`, `Depth=1`, `currentStep=baseStep`, and
`p=0`.  The first ten attempts therefore use the target sample before the
first parameter update.

## 4. Gaussian frame

The frame origin is the world start point.  Local y points from start to goal:

```matlab
ey = (goal-start) / norm(goal-start);
ex = [ey(2), -ey(1)];
R = [ex(:), ey(:)];
```

For row-vector points:

```matlab
local = (world-origin) * R;
world = local * R' + origin;
```

The implementation uses `computeGaussianFrame`, `gaussianWorldToLocal`, and
`gaussianLocalToWorld`; it never treats world x/y as Gaussian x/y.

## 5. Gaussian mixture equations

The most direct implementation of Eq.(4)--Eq.(8) is:

```text
sigma11 = sigma12 = 0.5 * currentStep * exp(pcf)
mu11 = xgmax + sigma11
mu12 = xgmin - sigma12
k1 = k2 = 0.5
```

For Eq.(9)--Eq.(14):

```text
sigma21 = sigma22 = currentStep * exp(pcf)
mu21 = ygmax + sigma21
mu22 = ygmax - sigma22
l1 = 1 - pcf^2
l2 = pcf^2
```

The x mixture is sampled with equal component probability.  The y mixture's
second component is the backward component, so its theoretical probability is
`pcf^2`.  Parameters are computed from the most recent ten successfully added
tree nodes, or all available successfully added nodes when fewer than ten
exist.  They are not computed from ten random samples or all tree nodes.

Gaussian samples are not clipped to an artificial rectangle.  Samples outside
the map are retried up to `gdGaussianMaxRetries`; after that, a uniform map
sample is used and `gaussianFallbackCount` is incremented.

## 6. Q-RRT* and GD-RRT* differences

Both planners use the shared ancestor-aware ChooseParent and
`propagateDescendantCosts`.  Their Rewire implementations are intentionally
separate:

- Q-RRT*: considers `xNew` and `Parent(xNew)` as Quick-Rewire candidates in
  increasing candidate-cost order.
- GD-RRT*: follows the paper's priority: test `Parent(xNew)` first; only if it
  does not improve the cost or its edge is blocked, test `xNew`.

Both implementations reject cycle-forming parent changes.

## 7. Paper mode and extended mode

`config.gdPaperMode = true` is the paper-reproduction mode.  It returns as
soon as the first feasible goal path is validated, matching Algorithm 1's
initial-path role.  The root command `GD_RRT_star` uses this mode by default.

`config.gdPaperMode = false` enables the project's extended optimization mode:
after the first solution it continues for `config.optimizeIterations` extra
iterations, subject to `maxIterations`.  This mode is explicitly an internal
benchmark extension and is not labeled as the original paper algorithm.

## 8. Boundary and timing engineering details

`gdGaussianMaxRetries` prevents an unbounded loop when the Gaussian tail keeps
leaving the map.  Plotting, `drawnow`, and animation pauses are outside the
planner timing accumulator.  Collision checks, Gaussian generation, pcf
updates, parameter updates, ancestor processing, rewiring, and cost
propagation are included in `planningTime`.
