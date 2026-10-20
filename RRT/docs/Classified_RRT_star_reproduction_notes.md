# Classified-RRT* reproduction notes

This document records the implementation choices for the Phase 8
reproduction of Xiong et al., 2026, *Classified-RRT*: Node classification
RRT*-based algorithm with improved solution performance and convergence rate.

## 1. xRand versus xNew classification

The paper's prose says that C-RRT* classifies `xnew`, while an OCR version of
Algorithm 3 shows a classifier call with `(xrand, xstart, xgoal)`. This
implementation classifies the point that is actually inserted into the search
tree, `xNew`. That is the object whose parent, Rewire behavior, and goal-tree
branch are defined by the rest of the algorithm.

## 2. Four node types

For every successfully extended `xNew`, the implementation checks both direct
edges and maps them as follows:

| Type | start to `xNew` | `xNew` to goal |
|---:|:---:|:---:|
| 1 | free | free |
| 2 | free | blocked |
| 3 | blocked | free |
| 4 | blocked | blocked |

All classification collision checks are included in `collisionChecks`.

## 3. Goal tree G

`G` has the goal as its root:

```text
G.nodes(1,:) = goal
G.sourceTreeIndex(1) = 0
G.costToGoal(1) = 0
```

Each Type 1 or Type 3 `xNew` is added as a direct goal branch because its
`xNew`-to-goal edge has already been verified collision-free. Its
`costToGoal` is the direct Euclidean distance to the goal.

## 4. CheckConnection engineering interpretation

The paper names `CheckConnection-C-RRT*(G,xNew)` but does not provide enough
detail for an unambiguous implementation. For Type 2 and Type 4 nodes, this
implementation scans the existing goal-tree branch nodes. For every branch it
computes:

```text
tree.cost(xNew) + norm(xNew-GNode) + G.costToGoal(GNode)
```

Only collision-free `xNew`-to-`GNode` edges participate; the lowest total cost
is selected. The goal-tree root is not redundantly scanned because the direct
`xNew`-to-goal edge was already tested during classification.

## 5. Random offset sampling

When the ordinary `Nearest`-to-`Steer` edge is blocked, C-RRT* samples on the
circle centered at `xNearest` with the original extension radius:

```text
theta = 2*pi*rand
xOffset = xNearest + radius*[cos(theta), sin(theta)]
```

The angle is uniform. A candidate must be inside the map and collision-free.
The paper does not specify a retry count, so the paper setting is
`crrtOffsetAttempts = 1`. More retries are available for sensitivity studies
but are not used in the paper-mode results. Offset attempts, successes,
failures, and optional sample points are reported separately.

## 6. Reverse-backtracking ChooseParent

Type 1 and Type 2 use `start` as their parent and do not call ChooseParent.
Type 3 and Type 4 first select `xmin` from `Nearest` plus `Near` using the
lowest cost-improving collision-free candidate. They then check ancestors in
reverse order:

```text
depth = dbmax, dbmax-1, ..., 1, 0
```

The first collision-free ancestor edge is accepted. The actual available
depth is capped naturally at the root, and zero indices are never produced.
The default `crrtBacktrackDepth = 2` follows the paper's recommended value,
but 0 through 4 are supported for sensitivity tests.

## 7. Sequential Rewire

The C-RRT* Rewire implementation is independent of Q-RRT* and GD-RRT*.
For each `xNear`, it tests in this order:

```text
Parent(xNew) -> xNew -> Parent(Parent(xNear))
```

The next candidate is tested when the current candidate either does not lower
the cost or is in collision. Rewire stops for that `xNear` only after a real
parent change succeeds. The third candidate also requires a lower cost and a
collision-free edge. Every candidate is checked for invalid indices, self
parenting, and cycle formation before it is considered.

## 8. Paper and extended modes

`crrtPaperMode = true` runs a fixed total horizon of
`crrtPaperTotalIterations` (default 1000). It is not interpreted as “first
solution plus 1000 iterations”. The result keeps both `firstPathLength` and
the best path found within that total horizon as `paperOptimalPathLength`.

`crrtPaperMode = false` is the project benchmark mode. It stops at the first
solution for `optimizeIterations = 0`, or continues for exactly the configured
additional optimization iterations after the first solution.

The no-argument root launcher uses the direct first-solution demo behavior.

## 9. Empty obstacle maps

An explicitly supplied empty obstacle matrix is preserved as an obstacle-free
map by `normalizeRRTConfig`. This is needed for deterministic Type 1 unit
tests and is also the natural interpretation of `config.obstacles = []`.
