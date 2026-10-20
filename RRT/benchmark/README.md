# Phase 10 benchmark

`run_phase10a_pilot` is the fixed-map pilot entry point. With no argument it
runs Experiment A only:

```matlab
addpath(genpath(pwd));
out = run_phase10a_pilot();
```

This is exactly `4 maps x 20 seeds x 10 planners = 800` planner runs. Each
run uses the common project-normalized workspace, fixed map, `stepSize = 1`,
`nearRadiusMax = 25`, `clearance = 0`, and `maxIterations = 5000`. Warm-up
calls are excluded from the CSV files.

Experiment A stops at the first feasible path. The separate Experiment B
implementation can be enabled explicitly after the pilot is reviewed:

```matlab
out = run_phase10a_pilot(true);
```

That appends the six optimization-capable planners with
`optimizeIterations = 1000`; it is intentionally not enabled by the default
Phase 10A command, so the pilot remains the requested 800-run experiment.

Outputs are written under `results/`:

- `pilot_raw_results.csv`: one row per planner run, including failures and
  algorithm-specific counters.
- `benchmark_summary.csv`: algorithm/map/experiment summaries with Wilson
  success-rate intervals and bootstrap intervals for continuous metrics.
- `phase10a_postprocess_results.csv` and
  `phase10a_postprocess_summary.csv`: APF native and common
  Greedy+B-spline post-processing, including smoothing validity.
- `benchmark_figures/`: 300-dpi pilot figures.

The four maps are defined only in `benchmark/maps/phase10Maps.m`; obstacles
are never regenerated during a benchmark.

## Phase 10B formal first-solution benchmark

After the 20-seed pilot is accepted, run the formal first-solution experiment
without the optimization pilot:

```matlab
addpath(genpath(pwd));
out = run_phase10a_pilot(false,1:100,'formal_first_solution');
```

This produces `4 maps x 100 seeds x 10 planners = 4000` runs. Every planner
stops at its first feasible path; no extra 1000-iteration optimization is
performed. Formal outputs use the `formal_first_solution_*` filenames and the
`results/benchmark_figures/formal_first_solution/` directory so the 20-seed
pilot files remain intact.

After the run, generate pooled diagnostics and per-map Pareto flags with:

```matlab
out = analyze_phase10b_formal();
```

The Map x Algorithm summary is the primary result. Pooled diagnostics and
Pareto flags are auxiliary and should not be used to declare a universal
winner.

## Phase 10A.5 optimization pilot

After Experiment A has been reviewed, run the separate fixed-budget pilot:

```matlab
addpath(genpath(pwd));
out = run_phase10a5_optimization_pilot();
```

This runs 7 extended planners (`RRT*`, `Bi-RRT*`, `Informed RRT*`,
`Q-RRT*`, `Classified-RRT*`, `APF-Bi-RRT*`, and `GD-RRT* Extended`) over
`4 maps x 20 seeds = 560` runs. Every successful run requests and checks
exactly 1000 iterations after its first solution. GD is explicitly labeled
Extended and uses `gdPaperMode = false`; Classified and APF use their
extended modes. No post-processing path is used as the Experiment B final
path.

The B Pilot writes `pilot_optimization_raw_results.csv`, the per-map
`pilot_optimization_summary.csv`, the pooled diagnostic table, anomaly table,
long-form convergence data, and four map-specific convergence figures under
`results/benchmark_figures/`.

## Phase 11A mechanism ablation

Phase 11A is a first-solution mechanism ablation. Every run stops at the first
complete collision-free path; it does not use `optimizeIterations`, and the
primary metrics do not include Greedy/B-spline post-processing.

Run it with:

```matlab
addpath(genpath(pwd));
out = run_phase11a_ablation();
save('results/phase11a_ablation_output.mat','out','-v7.3');
```

The experiment contains 9 variants and 2160 runs total:

- APF: `Bi-RRT*`, `Bias-only Bi-RRT*`, `APF-only Bi-RRT*`,
  `Bias+APF Bi-RRT*`;
- Classified: `Full`, `No Offset`, `No Classification+GoalTree`,
  `No Backtracking`, `No Sequential Rewire`;
- Map1/Map2 use 20 seeds each for sanity checking;
- Map3/Map4 use 100 seeds each for the formal ablation.

The primary report is separated by Map and Variant. It records success rate,
first-solution time, first-path length, node count, collision checks, and first
solution iteration. `results/phase11a_paired_statistics.csv` compares each
variant to its same-family Full baseline using paired Map+Seed differences,
Wilcoxon p-values, and rank-biserial effects.

Outputs:

- `results/phase11a_ablation_raw_results.csv`;
- `results/phase11a_ablation_summary.csv`;
- `results/phase11a_paired_statistics.csv`;
- `results/phase11a_ablation_anomalies.csv`;
- `results/phase11a_ablation.mat` and `results/phase11a_ablation_output.mat`;
- `results/benchmark_figures/phase11a_ablation/Map3_ablation.png`;
- `results/benchmark_figures/phase11a_ablation/Map4_ablation.png`.

The completed run produced 2160/2160 successful, valid runs, with no
exceptions, path-collision failures, required non-finite values, or iteration
truncations. Do not start UR5 migration or Phase 11B automatically after this
experiment. Use the Map3/Map4 paired results to select mechanisms first, then
run the fixed wall-clock budget experiment at 25/50/100/200 ms.

## Phase 11B fixed wall-clock budget experiment

Phase 11B compares six selected variants at equal total planning-time budgets.
Each planner run executes once through 200 ms and records best-so-far snapshots
at 25, 50, 100, and 200 ms. It does not run four independent repetitions and
does not use `optimizeIterations`, Greedy, or B-spline post-processing.

Run the formal experiment with:

```matlab
addpath(genpath(pwd));
out = run_phase11b_time_budget(1:100);
save('results/phase11b_time_budget_output.mat','out','-v7.3');
```

Variants:

- `RRT-Connect`;
- `Bi-RRT*`;
- `Full Classified-RRT*`;
- `ClassGoal-Lite`: Classification+GoalTree on, Offset/Backtracking/Sequential Rewire off;
- `APF-only Bi-RRT*`: dynamic Goal Bias off, APF direction on;
- `Full APF-Bi-RRT*`: dynamic Goal Bias and APF direction on.

The formal scale is `6 × 4 maps × 100 seeds = 2400 planner runs`, producing
9600 snapshot rows. The time-budget mode is opt-in through
`config.timeBudgetMs`; default planner calls preserve the previous behavior.
RRT-Connect keeps its first path fixed while consuming the time horizon; the
other five variants continue their native tree growth/connection/rewiring logic.

Outputs:

- `results/phase11b_time_budget_raw.csv`;
- `results/phase11b_time_budget_summary.csv`;
- `results/phase11b_time_budget_paired_statistics.csv`;
- `results/phase11b_time_budget_pareto.csv`;
- `results/phase11b_time_budget_anomalies.csv`;
- `results/phase11b_time_budget.mat` and `results/phase11b_time_budget_output.mat`;
- `results/benchmark_figures/phase11b_time_budget/`.

The completed formal run had 0 anomalies, 0 path-validity failures, 0
best-so-far monotonicity violations, median budget overshoot 0.103 ms, and
total wall time 732.791 s. Use Map3/Map4 as the main constrained-space
analysis. Do not declare a final algorithm or begin UR5 migration from this
experiment alone.
