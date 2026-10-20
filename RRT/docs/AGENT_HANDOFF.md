# RRT 项目 Agent 交接文档

更新时间：2026-09-26（Asia/Shanghai）

项目根目录：`/home/ttbstt/jixiebi/RRT`

本文档用于让下一位 agent 快速恢复当前 MATLAB RRT 项目的上下文、实验约束、代码状态和后续工作。

## 当前用户目标

用户已经明确改变主线：

1. 不再把“额外 1000 次后谁的路径更短”作为主实验。
2. 当前主实验只评价一次规划得到首条合法路径的性能。
3. 先完成 Phase 10B：4 张固定地图 × 100 seeds × 10 algorithms = 4000 runs。
4. 正式结果后筛选 3–4 个算法进入 UR5 六维关节空间。
5. 之后做必要机制消融，再迁移到完整焊接任务：
   `安全点 → 起焊点 → 焊缝运动 → 焊接终点 → 安全点`。
6. 暂不启动 UR5 迁移，不继续堆新的二维算法。

主指标：

- Success Rate
- First Solution Time
- First Path Length
- Node Count
- Collision Checks

辅助指标：

- First Solution Iteration
- Planning Time 的 95% CI

结果必须优先按 `Map × Algorithm` 统计；四张地图 pooled 结果只能作为辅助诊断，不作主结论。

## UR5 仿真环境的唯一参考来源

RRT 项目后续涉及 UR5、焊枪、船舶工件、焊缝起止点、碰撞模型或 ROS 2/CoppeliaSim
联调时，必须先阅读并以以下文档为准：

`/home/ttbstt/jixiebi/AGENT_HANDOFF.md`

该文档是具体 UR5 仿真环境的权威交接文档，当前 RRT 项目文档不重复维护一份可能过期的
UR5 坐标副本。特别注意：二维 RRT 的 Map 坐标不是 UR5 的真实工件坐标，不能直接当作
焊缝端点或机械臂目标位姿。

当前应从该文档读取的关键内容包括：

- CoppeliaSim 场景：`/home/ttbstt/jixiebi/UR5/ur5.ttt`；ROS 2、MATLAB 与外部
  `rclpy`/ZMQ Remote API 的连接架构；
- 场景中的 `GunCollisionHull`、`Workpiece` 碰撞模型，以及 WeldTCP 和实际焊枪轴
  `wireAxisTcp`；
- 焊缝为 `Workpiece` 的 `bottom × wall_y1` 交线，焊接侧为 U 形工件内部；
- 当前实际轴规划使用的焊缝边界和 `endMarginM = 0.025 m`。历史章节中的零边界或旧 TCP
  端点只作诊断/历史记录，不能未经核对直接用于当前规划；精确当前端点应从当前规划结果
  `/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan.mat` 或对应当前脚本读取；
- 完整任务链：`qHome → qPreWeld → Point 1–80 → qPostWeld → qHome`。其中前后转场使用
  RRT，焊缝中间段继续使用现有连续 IK / A-ARC-IK 路线，不用二维 RRT 点列替代。

在未重新读取上述权威文档并核对当前 MAT/脚本之前，不得自行假设 UR5 的焊缝起点、终点、
TCP 偏置、焊枪轴方向或碰撞几何参数。

## 算法清单

当前二维平台有 10 种算法：

1. Basic RRT
2. Bi-RRT
3. RRT-Connect
4. RRT*
5. Bi-RRT*
6. Informed RRT*
7. Q-RRT*
8. GD-RRT*
9. Classified-RRT*
10. APF-Bi-RRT*

正式首解实验中：

- 所有算法均 `showAnimation=false`。
- 首次找到合法路径立即停止。
- `optimizeIterations=0`。
- GD-RRT* 使用 `gdPaperMode=true`。
- APF-Bi-RRT* 的 Experiment A 只统计 raw planner 首解，不把 Greedy/B-spline 当作主路径质量。
- Classified-RRT* 使用其首解模式，不把 extended 1000 次结果混进首解实验。

## 算法冻结规则

Phase 1–9 的采样、Steer、ChooseParent、Rewire、APF、Gaussian、Classified node logic、goal tree、offset sampling 和 collisionFree 逻辑视为冻结。

允许的改动仅限：

- 统一 planner(config) 接口；
- 补充统计字段；
- benchmark wrapper/runner；
- 明确的 bug 修复。

最近增加了统计字段 `result.collisionChecksAtFirstSolution`，用于计算首解之后的碰撞检测开销；没有改变搜索逻辑或随机数顺序。

## 已完成的阶段

### Phase 1–9

10 种算法及其测试已经完成。Phase 1–9 回归在 Phase 10A、Phase 10A.5 之前和之后都通过。

统一回归命令：

```matlab
addpath(genpath(pwd));
test_phase1_phase3;
test_phase4_bi_rrt_star;
test_phase5_informed_rrt_star;
test_phase6_qrrt_star;
test_phase7_gd_rrt_star;
test_phase8_classified_rrt_star;
test_phase9_apf_bi_rrt_star;
test_phase10_wrappers;
```

### Basic RRT / Bi-RRT 统一接口

新增/保留文件：

- `RRT_basic.m`
- `Bi_RRT.m`
- `planners/RRT_basic_impl.m`
- `planners/Bi_RRT_impl.m`
- `tests/test_phase10_wrappers.m`

旧动画版本与新 wrapper 在 seed=1 下路径行为一致：

- Basic RRT：iteration=646，pathLength=264.808321284，nodes=470。
- Bi-RRT：iteration=87，pathLength=194.326188895，startTree=33，goalTree=31。

### Phase 10A 首解 Pilot

规模：

```text
4 maps × 20 seeds × 10 algorithms = 800 runs
```

结果：

- 800/800 success；
- 0 planner exception；
- 0 path collision；
- 必需字段无 NaN/Inf；
- 0 maxIterations truncation；
- Pilot 前后回归均通过；
- 总耗时约 85.8 秒。

主要输出：

- `results/pilot_raw_results.csv`
- `results/benchmark_summary.csv`
- `results/phase10a_postprocess_results.csv`
- `results/phase10a_postprocess_summary.csv`
- `results/benchmark_figures/`

Experiment C 后处理结果必须保留：

- Bi-RRT* common Greedy+B-spline 有效率：60.0%；
- APF-Bi-RRT* native/common B-spline 有效率：51.25%。

不要为了提高这个有效率而修改平滑算法；这是需要保留的工程现象。

### Phase 10A.5 优化 Pilot

规模：7 个 extended planner × 4 maps × 20 seeds = 560 runs。

参与算法：

- RRT*
- Bi-RRT*
- Informed RRT*
- Q-RRT*
- Classified-RRT*
- APF-Bi-RRT*
- GD-RRT* Extended（名称必须带 Extended，`gdPaperMode=false`）

结果：

- 560/560 success；
- 每条成功样本 `optimizationIterationsExecuted=1000`；
- 0 truncation；
- 0 path collision；
- 0 final-longer-than-first；
- 0 NaN/Inf；
- 收敛数据 6160/6160 完整；
- 总耗时约 153 秒。

主要输出：

- `results/pilot_optimization_raw_results.csv`
- `results/pilot_optimization_summary.csv`
- `results/pilot_optimization_pooled_diagnostic.csv`
- `results/pilot_optimization_convergence.csv`
- `results/pilot_optimization_anomalies.csv`
- `results/benchmark_figures/Map1_optimization_convergence.png` 至 `Map4_optimization_convergence.png`

重要诊断：Classified-RRT* extended 模式的 goal-tree connection checks / total collision checks 中位占比约为：

- Map 1：95.38%
- Map 2：97.82%
- Map 3：89.02%
- Map 4：93.59%

这解释了其 extended 优化计算量高；不要在当前阶段优化 goal-tree 搜索。

## 固定地图和公共参数

地图唯一来源：`benchmark/maps/phase10Maps.m`

- workspace：`[0,100] × [0,100]`
- start：`[10,10]`
- goal：`[90,90]`
- clearance：`0`
- `stepSize=1`
- `nearRadiusMax=25`
- Phase 10A 首解：`maxIterations=5000`
- Phase 10A.5 B Pilot：`maxIterations=6000`
- seeds：由实验明确传入，不在算法内部随机生成地图。

## Phase 10B 正式首解实验

正式命令已经启动：

```bash
matlab -batch "addpath(genpath(pwd)); out=run_phase10a_pilot(false,1:100,'formal_first_solution'); save('results/formal_first_solution_output.mat','out','-v7.3');"
```

含义：

```text
4 maps × 100 seeds × 10 algorithms = 4000 runs
Experiment A only
first feasible solution immediately stop
```

正式输出目标：

- `results/formal_first_solution_raw_results.csv`
- `results/formal_first_solution_summary.csv`
- `results/formal_first_solution_postprocess_results.csv`
- `results/formal_first_solution_postprocess_summary.csv`
- `results/formal_first_solution_convergence.mat`
- `results/formal_first_solution_output.mat`
- `results/benchmark_figures/formal_first_solution/`

状态：已完成。

- session `24665` 已正常结束；
- 4000/4000 runs 完成；
- 10 个算法各 400 次；
- 总耗时：458.125 秒；
- 0 failure、0 planner exception、0 path-validity failure、0 maxIterations truncation。

formal CSV 已生成。后续 agent 不要重复启动同一组 4000 runs，先检查 formal 输出并进行统计/候选筛选。

最终验收：

- 原始 CSV：4000 条数据行；
- Summary：40 个 Map×Algorithm 分组，每组 100 runs；
- 4000/4000 success；
- 0 failure、0 path collision、0 truncation；
- 必需字段 4000/4000 finite；
- 正式总耗时：458.125 秒；
- 正式结果后 Phase 1–9 + wrapper regression 全部通过。

formal 分析脚本和输出：

- `benchmark/analyze_phase10b_formal.m`
- `results/formal_first_solution_pooled_diagnostic.csv`
- `results/formal_first_solution_pareto.csv`
- `results/benchmark_figures/formal_first_solution/formal_first_solution_pareto.png`

Pareto 点数量（按首解时间、首路径长度、节点数、碰撞检测和成功率的 Map 内多目标判定）：Map1=3、Map2=3、Map3=5、Map4=4。

## Phase 10B 完成后的验收清单

1. 检查 `formal_first_solution_raw_results.csv` 应有 4001 行（含 header）。
2. 检查每个 `algorithm × map` 应有 100 runs，共 40 个分组。
3. 检查 `success=false`、planner exception、NaN/Inf、pathCollisionFree=false、maxIterationsTruncated。
4. 主统计只使用 `formal_first_solution_summary.csv` 的 Map×Algorithm 行。
5. pooled 结果只作辅助，不输出“冠军”。
6. 计算 Planning Time 的 95% CI；当前共享 summary 已有 bootstrap CI 结构，可按正式文件复用。
7. 生成 Pareto 辅助分析：首解时间、首路径长度、碰撞检测、成功率共同考虑。
8. 特别检查 Map 3 / Map 4；它们更接近狭小空间焊接场景。
9. 正式实验后再跑一次完整 Phase 1–9 回归。
10. 到此停止，不自动开始 UR5，也不再追加 1000 次主实验。

## Pareto 分析建议

不要用单指标选择算法。至少保留以下四个维度：

- success rate 越高越好；
- first solution time 越低越好；
- first path length 越低越好；
- collision checks 越低越好。

建议先按 Map 3 / Map 4 找候选，再结合 Map 1 / Map 2 的稳定性。可先保留：

- 一个最快基线：通常是 RRT-Connect；
- 一个速度/质量折中：候选为 Bi-RRT*；
- 一个 Classified 或 APF 候选；
- GD-RRT* 只有在复杂地图收益足够时再进入 UR5。

这些只是候选方向，必须以 100-seed formal 数据为准。

## 后续 UR5 迁移边界

100-seed 正式结果筛选前，不要迁移 UR5。

迁移后规划空间是：

```text
q = [q1 q2 q3 q4 q5 q6]
```

每条关节空间边 `qa → qb` 使用关节插值并调用 `collisionFreeJointEdge.m`，检查：

- 机械臂与工件；
- 机械臂与障碍物；
- 焊枪与障碍物；
- 机械臂自碰撞。

UR5 阶段新增指标：

- planning time；
- success rate；
- joint-space path length；
- minimum clearance；
- execution time；
- `T_total = T_plan + T_exec`。

最终焊接任务链：

```text
安全点 → 起焊点 → A-ARC-IK 焊缝运动 → 焊接终点 → 安全点
```

## 重要文件索引

- `benchmark/run_phase10a_pilot.m`：A Pilot 与可复用正式首解 runner。
- `benchmark/run_phase10a5_optimization_pilot.m`：B Pilot 专用 runner。
- `benchmark/maps/phase10Maps.m`：固定 4 张地图。
- `benchmark/README.md`：benchmark 运行说明。
- `core/initRRTResult.m`：统一结果字段，含 `collisionChecksAtFirstSolution`。
- `tests/test_phase10_wrappers.m`：Basic/Bi wrapper 行为一致性测试。
- `results/benchmark_summary.csv`：20-seed A Pilot 汇总。
- `results/pilot_optimization_summary.csv`：20-seed B Pilot 汇总。

## 接手时的原则

- 先读本文档，再检查运行中的 MATLAB session。
- 不重复启动已经运行的 formal benchmark。
- 不覆盖 Phase 10A/10A.5 结果。
- 不修改算法核心机制来改善统计数字。
- 正式 100-seed 结果未验收前，不进入 UR5。

## Phase 11A：机制消融已完成

Phase 11A 的目标是识别哪些机制值得进入后续固定 wall-clock 时间预算实验，
不是继续做额外 1000 次迭代优化，也不是选择一个跨地图的单一冠军。
所有运行均为首解即停：第一次得到完整无碰撞路径后立即结束；
`optimizeIterations = 0`；主指标没有使用 Greedy/B-spline 后处理。

运行入口：

```matlab
addpath(genpath(pwd));
out = run_phase11a_ablation();
save('results/phase11a_ablation_output.mat','out','-v7.3');
```

实验设计：

- APF 组：`Bi-RRT*`、`Bias-only Bi-RRT*`、`APF-only Bi-RRT*`、`Bias+APF Bi-RRT*`；
- Classified 组：`Full`、`No Offset`、`No Classification+GoalTree`、
  `No Backtracking`、`No Sequential Rewire`；
- Map1/Map2：每组 20 seeds sanity check；
- Map3/Map4：每组 100 seeds 正式消融；
- 总量：9 variants × (20 + 20 + 100 + 100) = 2160 runs；
- APF 组固定使用 `apfPaperMode = false`、`apfPaperConnectionMode = 'best_near'`，
  以隔离 Goal Bias 与 APF force；
- Classified 组通过独立开关逐项关闭机制，默认配置仍保持原算法行为。

输出文件：

- `benchmark/run_phase11a_ablation.m`：运行器；
- `results/phase11a_ablation_raw_results.csv`：2160 条原始运行记录；
- `results/phase11a_ablation_summary.csv`：Map×Variant 汇总；
- `results/phase11a_paired_statistics.csv`：同 Map + Seed 的配对差异、Wilcoxon p 和 rank-biserial effect；
- `results/phase11a_ablation_anomalies.csv`：异常记录；
- `results/phase11a_ablation.mat`、`results/phase11a_ablation_output.mat`：MATLAB 结果对象；
- `results/benchmark_figures/phase11a_ablation/Map3_ablation.png`；
- `results/benchmark_figures/phase11a_ablation/Map4_ablation.png`。

验收结果：

- 2160/2160 success；
- 每个 variant 240/240 success；
- 0 planner exception；
- 0 path-validity failure；
- 0 required NaN/Inf；
- 0 max-iteration truncation；
- 总耗时 258.338 s；
- Phase 1–9 + wrapper regression 在消融前后均通过，消融后最后一次回归标记为 `REGRESSION_AFTER_PHASE11A_OK`。

Map3/Map4 的首解中位数（时间单位 ms；路径为二维原始路径长度）：

| family | variant | Map3 time | Map3 path | Map4 time | Map4 path |
|---|---|---:|---:|---:|---:|
| APF | Bi-RRT* | 14.6445 | 128.6711 | 19.1670 | 138.9486 |
| APF | Bias-only Bi-RRT* | 17.7800 | 128.8233 | 25.9265 | 141.4561 |
| APF | APF-only Bi-RRT* | 15.8365 | 131.2397 | 33.5280 | 131.0760 |
| APF | Bias+APF Bi-RRT* | 14.9480 | 131.6528 | 38.0220 | 131.4433 |
| Classified | Full | 34.4205 | 130.0137 | 21.0640 | 126.5228 |
| Classified | No Offset | 30.3720 | 131.2993 | 16.4270 | 127.0449 |
| Classified | No Classification+GoalTree | 142.5205 | 125.4264 | 92.0465 | 127.1646 |
| Classified | No Backtracking | 32.5455 | 130.2923 | 22.1075 | 126.5228 |
| Classified | No Sequential Rewire | 18.0865 | 130.0137 | 10.1760 | 126.5228 |

配对统计采用同一 Map、同一 seed，差值定义为“变体 − Full 基线”。重点结论：

- Classified 的 `No Classification+GoalTree` 在 Map3/4 上分别比 Full 慢约
  108.962 ms / 71.3005 ms，且 Wilcoxon `p < 1e-16`；该机制对快速建立首解非常关键。
- `No Sequential Rewire` 在 Map3/4 上分别快约 15.804 ms / 10.894 ms，
  首路径中位数不变；对首解阶段它主要增加计算成本，未显示路径收益。
- `No Offset` 在 Map3/4 上分别快约 3.3745 ms / 5.023 ms，
  但路径中位数增加约 1.0052 / 0.6461；当前地图下 Offset 不利于首解速度，
  质量代价较小但不是零。
- `No Backtracking` 对路径几乎没有影响；Map3 略快、Map4 略慢，属于次优先级机制。
- APF 组中，`Bias-only` 在 Map3/4 分别慢约 3.2545 / 6.3165 ms，
  Map4 路径还变长约 2.6195；动态 Goal Bias 没有显示出稳定收益。
- `APF-only` 在 Map4 将首路径中位数降低约 7.3820，但时间增加约 13.9845 ms；
  `Bias+APF` 没有比 APF-only 带来额外收益，反而在 Map4 更慢且路径略长。

这些结论是当前四张固定地图、首解即停目标下的机制证据，不应直接外推为所有场景的普适结论。
尤其是 APF 的“更短路径换更长时间”需要在 Phase 11B 的 25/50/100/200 ms
best-so-far 曲线中继续验证。当前建议的后续候选不是预先定死的：
Classified 保留 Classification+GoalTree，首解核心暂不默认保留 Sequential Rewire；
APF 倾向先保留 APF-only 作为时间—质量候选，动态 Goal Bias 暂不视为必选机制。

当前阶段边界：Phase 11A 完成后停止；不要自动开始 Phase 11B、UR5 六维迁移或焊接任务接入，
除非用户明确要求继续。

## Phase 11B：固定 wall-clock 时间预算已完成

用户明确要求从“额外固定迭代次数”转为“相同总规划时间下的 best-so-far
路径质量”比较。本阶段仍然是二维实验，不进入 UR5，不执行 Greedy/B-spline，
也不构造加权总分。

正式方案（6 variants）：

- `B0 RRT-Connect`：首条路径保持不变；time-budget 模式继续运行到最大 checkpoint，
  但不加入 RRT* 优化；
- `B1 Bi-RRT*`：标准双向 RRT*；
- `B2 Full Classified-RRT*`：Offset、Classification+GoalTree、Backtracking、Sequential Rewire 全开；
- `B3 ClassGoal-Lite`：Classification+GoalTree 开启，Offset/Reverse Backtracking/
  Sequential Rewire 关闭，Type 3/4 使用标准 `chooseParentRRTStar` 和 `rewireRRTStar`；
- `B4 APF-only Bi-RRT*`：动态 Goal Bias 关闭、改进 APF direction 开启；
- `B5 Full APF-Bi-RRT*`：动态 Goal Bias 与改进 APF direction 均开启。

每次 planner run 只执行一次，checkpoint 为 `25/50/100/200 ms`；一次 run 到 200 ms，
保存四个 best-so-far snapshot。单次迭代不被中断，记录实际 snapshot 时间和 overshoot。
time-budget 分支通过 `config.timeBudgetMs` 显式启用；没有该字段时，原有 planner
行为保持不变。

运行入口：

```matlab
addpath(genpath(pwd));
out = run_phase11b_time_budget(1:100);
save('results/phase11b_time_budget_output.mat','out','-v7.3');
```

正式规模：`6 variants × 4 maps × 100 seeds = 2400 planner runs`，共 9600 个
budget snapshots。运行器文件：`benchmark/run_phase11b_time_budget.m`。

输出文件：

- `results/phase11b_time_budget_raw.csv`：9600 条 checkpoint 记录；
- `results/phase11b_time_budget_summary.csv`：Map×Variant×Budget 汇总，含成功率、路径中位数、IQR、bootstrap 95% CI、collision checks、node count、overshoot；
- `results/phase11b_time_budget_paired_statistics.csv`：同 Map+Seed+Budget 的 Wilcoxon、rank-biserial effect 和 McNemar 统计；
- `results/phase11b_time_budget_pareto.csv`：以成功率为硬约束的 Pareto 标记；
- `results/phase11b_time_budget_anomalies.csv`：异常记录；
- `results/phase11b_time_budget.mat`、`results/phase11b_time_budget_output.mat`：含 snapshot paths 的 MATLAB 结果；
- `results/benchmark_figures/phase11b_time_budget/Map1_time_budget.png` 至 `Map4_time_budget.png`；
- `results/benchmark_figures/phase11b_time_budget/Map3_TimePath_Pareto.png`、`Map4_TimePath_Pareto.png`；
- `benchmark/analyze_phase11b_time_budget.m`：重新计算 Pareto 标记的只读分析脚本。

正式验收：

- 2400/2400 planner runs 完成；
- 9600/9600 snapshot 时间不早于对应预算；
- 9260/9600 snapshot 已找到合法路径；
- 0 path-validity failure；
- 0 anomaly rows；
- best-so-far 路径单调性违规为 0；
- 最大 overshoot 2.399 ms，中位 overshoot 0.103 ms；
- 总耗时 732.791 s；
- Phase 1–9 + wrapper regression 最后一次通过，标记为 `REGRESSION_AFTER_PHASE11B_OK`。

Map3/Map4 的主要结果（格式：成功率；best path median；collision checks median）：

| Map | Budget | RRT-Connect | Bi-RRT* | Full Classified | ClassGoal-Lite | APF-only | Full APF |
|---|---:|---|---|---|---|---|---|
| Map3 | 25 ms | 100%; 144.808; 2379 | 99%; 125.496; 1311 | 9%; 127.102; 1277 | 91%; 127.404; 1531 | 100%; 125.522; 858 | 100%; 125.450; 861 |
| Map3 | 50 ms | 100%; 144.808; 5041 | 100%; 124.834; 2891 | 90%; 124.580; 2948 | 100%; 124.725; 4140 | 100%; 125.495; 1729 | 100%; 125.367; 1741 |
| Map3 | 100 ms | 100%; 144.808; 10469 | 100%; 124.678; 5988 | 100%; 122.764; 7815 | 100%; 123.915; 10215 | 100%; 125.495; 3446 | 100%; 125.367; 3441 |
| Map3 | 200 ms | 100%; 144.808; 21397 | 100%; 124.633; 12018 | 100%; 122.035; 19387 | 100%; 122.991; 23268 | 100%; 125.495; 6425 | 100%; 125.367; 6352 |
| Map4 | 25 ms | 100%; 158.132; 4227 | 92%; 134.483; 1728 | 84%; 126.155; 818 | 100%; 126.002; 2900 | 0%; —; — | 0%; —; — |
| Map4 | 50 ms | 100%; 158.132; 9764 | 100%; 132.403; 3829 | 100%; 125.743; 4181 | 100%; 125.923; 7954 | 100%; 130.339; 2164 | 95%; 130.339; 2148 |
| Map4 | 100 ms | 100%; 158.132; 21090 | 100%; 132.328; 8051 | 100%; 125.743; 13922 | 100%; 125.879; 19303 | 100%; 130.227; 4136 | 100%; 130.227; 4240 |
| Map4 | 200 ms | 100%; 158.132; 43922 | 100%; 132.328; 15656 | 100%; 125.730; 35549 | 100%; 125.803; 42806 | 100%; 130.227; 7834 | 100%; 130.227; 8310 |

解释边界：

- RRT-Connect 在四个 checkpoint 的路径保持首解不变；其 collision checks 随继续运行增加，
  不能把这些额外检查误认为路径优化收益。
- Map3 上 Full Classified 在 100/200 ms 得到最短路径，但 25/50 ms 首解成功率较低；
  ClassGoal-Lite 早期成功率明显更高，100/200 ms 路径略长，且 collision checks 更高，
  因此它是早期响应候选，不是已证明的全面替代。
- Map4 上 ClassGoal-Lite 在 25 ms 达到 100% 成功率，而 Full Classified 为 84%；
  50 ms 以后 Full Classified 路径略短且检查数更低。
- APF-only 与 Full APF 在 Map4 的 100/200 ms 路径几乎相同；APF-only 的 collision checks 更低，
  且 50 ms 成功率为 100% 对 Full APF 的 95%。动态 Goal Bias 没有显示出稳定的固定预算收益。
- Bi-RRT* 相比 RRT-Connect 在 Map3/4 付出更高的早期失败风险（25 ms），但成功后路径明显更短；
  50 ms 后两张复杂地图均达到 100% 成功率。

Phase 11B 之后不宣布最终算法，也不再追加二维组合实验。Phase 12A-1 已完成如下六维迁移
功能验证；后续进入 Phase 12B 前，仍需先由用户确认是否继续，不得自动启动 ROS2/CoppeliaSim。

## Phase 12A-1：UR5 六维迁移功能验证（2026-09-27）

本阶段只迁移四个候选：

1. `RRTConnectJoint`：RRT-Connect 速度基线；
2. `BiRRTStarJoint`：双向 RRT* 质量基线；
3. `ClassGoalLiteJoint`：Classification + GoalTree，关闭 Offset、Backtracking、Sequential Rewire；
4. `ClassifiedRRTStarJoint`：Full Classified-RRT*，包含 6D hypersphere Offset、Reverse Backtracking
   和 Sequential Rewire。

APF 没有迁移。二维 APF 的点—矩形距离不能直接替换成 UR5 关节角距离；后续若迁移，必须
单独建立 workspace obstacle force、link-wise distance 或 Jacobian-transpose 模型。

实现位置：

- `jointspace/jointDistance.m`：六维加权 L2 距离，内部使用最短 revolute angle delta；
- `jointspace/jointWrapDelta.m`：处理 `179° → -179°` 的 2° 差值；
- `jointspace/steerJoint.m`：沿最短角方向、统一 norm step 扩展，不 clamp 越界状态；
- `jointspace/sampleJointConfig.m`、`deriveUR5JointLimits.m`、`normalizeJointPlannerConfig.m`；
- `jointspace/jointPlannerCore.m`：四个 planner 共用核心；
- `collisionFreeJointState.m` 和 `collisionFreeJointEdge.m`：所有算法共用同一 UR5 状态/边碰撞接口。

UR5 场景由权威文档和现有脚本提供：

- 权威环境：`/home/ttbstt/jixiebi/AGENT_HANDOFF.md`；
- 场景脚本：`/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m`；
- 使用当前 `GunCollisionHull`、焊丝碰撞模型、四个 `Workpiece` proxy、机器人自碰撞和结构连接
  跳过表；没有修改 ROS2、CoppeliaSim 或 UR5 场景文件；
- 状态单位为 rad、形状为 1×6 row；当前边检查分辨率为 2°，明确属于 discrete interpolated
  joint-edge collision checking，不宣称连续 swept-volume 检查；
- 真实任务只测试自由空间转场：`qHome → qPreWeld`、`qPostWeld → qHome`，以及起/收焊过渡。
  80 点焊缝段从已验收结果读取，没有用 RRT 重规划焊缝点列。

功能批测入口：`benchmark/run_phase12a_functional.m`。

本次使用 6 个案例 × 4 个算法，共 24 次离线 MATLAB 运行，配置为 `maxIterations=300`、
`maxJointStepRad=25°`、`jointEdgeResolutionRad=2°`：

- `C01_Home_to_PreWeld`；
- `C02_PostWeld_to_Home`；
- `C03_PreWeld_to_Point1`；
- `C04_Point80_to_PostWeld`；
- `C05_Home_to_Point1`（靠近工件/焊枪的诊断案例）；
- `C06_Simple_5deg`（简单局部自由空间案例）。

验收结果：`24/24 success`、`24/24 pathValid`、`0 joint-limit violation`。结果文件：

- `results/phase12a_functional.mat`；
- `results/phase12a_functional_summary.csv`。

注意：最初一次 Phase 12A-1 结果只保存了统计字段，没有持久化 24 条 `qPath`。因此该旧结果
不能直接用于“不重新规划”的 2°/1°/0.5°敏感性复核。`run_phase12a_functional.m` 已更新为
同时保存 `report.paths`；后续必须先用完全相同的 6 cases、4 algorithms 和 seed 重新生成一次
可审计路径归档，再运行 `benchmark/validate_phase12a_resolution.m`。在此之前不得把新规划结果
冒充旧路径的分辨率复核，也不得开始 480-run Phase 12A-2 Pilot。

代表性结果（规划时间包含离线 MATLAB 碰撞检查，不包含场景加载；单位 ms）：

| Case | RRT-Connect | Bi-RRT* | ClassGoal-Lite | Full Classified |
|---|---:|---:|---:|---:|
| Home → PreWeld | 6093 | 2567 | 31870 | 37591 |
| PostWeld → Home | 2439 | 10169 | 2141 | 1593 |
| PreWeld → Point1 | 727 | 212 | 215 | 583 |
| Point80 → PostWeld | 528 | 441 | 82 | 182 |
| Home → Point1 | 5378 | 20579 | 31684 | 39802 |
| Simple 5° | 245 | 281 | 326 | 344 |

这些结果只证明四个搜索器已经能够在共同碰撞框架下输出合法 6D joint path，不能作为最终
算法排名或执行时间结论。尤其当前 Full Classified 的 6D Offset 是高维自然扩展，仍需单独
做机制合理性与效率分析。

新增单元测试：

- `tests/test_joint_distance.m`；
- `tests/test_joint_wrap.m`；
- `tests/test_joint_steer.m`；
- `tests/test_joint_limits.m`；
- `tests/test_joint_edge_collision.m`；
- `tests/test_phase12a_joint_planners.m`。

Phase 1–11 原有回归全部通过，最后标记为 `REGRESSION_PHASE1_11_PASS`。Phase 12A-1 完成后
停止：不要迁移 APF、不要做 100-seed 六维 benchmark、不要做 trajectory timing，也不要启动
ROS2/CoppeliaSim；下一阶段应由用户明确确认后再做。

## Phase 12A-1 archival rerun 与分辨率复核（2026-09-27）

由于旧的功能验证 MAT 没有保存 qPath，按原 6 cases × 4 algorithms、相同 seed 和相同配置
进行了仅用于补档的 archival rerun。唯一变化是保存 `report.paths` 和路径元数据；没有修改
planner、案例、seed 或碰撞模型，也没有覆盖旧结果。归档对照结果为 `24/24`：成功状态、路径
合法性、节点数、collision checks、path state count 和首解迭代均一致；`jointPathCostL2`
只存在不超过 `1e-7` 的历史 CSV 有限精度差异。RRT 目录不是 Git 仓库，因此 code version
记录为 unavailable。

归档文件：

- `results/phase12a_functional_archival.mat`：24 条 qPath 和元数据；
- `results/phase12a_functional_archival_summary.csv`；
- `results/phase12a_functional_archival_comparison.csv`：`PASS=1`；
- `benchmark/refresh_phase12a_archival_comparison.m`：不调用 planner 的对照刷新脚本。

随后只读取归档 qPath，使用同一 UR5 场景在 2°、1°、0.5°离散插值分辨率下复核，共 72 条
检查通过 `70/72`。23 条路径（按算法/案例组合计）在三种分辨率均通过；唯一失败组合为：

- `C05_Home_to_Point1 / RRT-Connect / seed=5`；
- 第 26 条边；
- 2°：PASS；1°：FAIL；0.5°：FAIL；
- 1°/0.5°均在同一中间构型发现 `environmentCollision=1`、`selfCollision=0`。

因此当前结果已经出现“2° PASS、细分辨率 FAIL”的漏检证据。该结果只能说明当前离散边检查
分辨率不足以支持后续正式实验的安全假设，不能宣称连续 swept-volume 安全。不得启动
Phase 12A-2 的 480-run Pilot；下一步应先提高正式规划/验证的边分辨率并重新确认计算代价。

复核文件：

- `results/phase12a_resolution_sensitivity_archival.mat`；
- `results/phase12a_resolution_sensitivity_archival.csv`；
- `results/phase12a_resolution_sensitivity_archival_wide.csv`；
- `benchmark/validate_phase12a_resolution.m`：只验证已保存路径，不调用 planner。

## Phase 12A-1.5：1°规划定标（2026-09-27）

基于上述 2°漏检反例，使用完全相同的 6 cases、4 algorithms、seed 和 planner 参数，仅将
planner 内部 `jointEdgeResolutionRad` 从 2°改为 1°，重新规划 24 次。结果为：

- `24/24` planner success；
- `24/24` endpoint/path-valid；
- 没有修改 UR5、焊枪、工件、环境、joint limits 或 planner mechanism；
- 新结果另存为 `results/phase12a_resolution1deg_functional.mat`。

随后只读取这 24 条新 qPath，以 0.5°重新做独立 edge validation，结果为 `23/24 PASS`。失败仍为
`C05_Home_to_Point1 / RRT-Connect / seed=5`，但由于 1°规划产生了不同路径，当前新路径的失败位置为
第 24 条边、失败采样点 20，`environmentCollision=1`、`selfCollision=0`。边端点和实际碰撞构型
已保存在 `results/phase12a_resolution1deg_validation05.mat` 的 rows 字段中。

因此目前不能接受“1° planning + 0.5° validation”作为已经通过的正式配置，也不能启动
Phase 12A-2 的 480-run Pilot。当前证据只能说明 1°比 2°更严谨，但在这个窄空间案例上仍有
离散漏检；它仍不是 continuous swept-volume collision detection。

1°相对原 2°功能结果的中位数开销：planning time ratio `1.6666`，离散 collision configuration
check ratio `1.7850`；均值分别为 `1.6523` 和 `1.7663`。四个算法的中位时间比约为：RRT-Connect
1.65、Bi-RRT* 1.66、ClassGoal-Lite 1.68、Full Classified-RRT* 1.74。只有 C05/RRT-Connect
的首条路径代价/节点数发生明显变化，其余配对结果保持一致。

新增统计字段：

- `edgeCollisionChecks`：调用 edge checker 的次数；
- `collisionConfigurationChecks`：实际送入离散 joint-state collision check 的构型数量。

结果文件：

- `results/phase12a_resolution1deg_functional_summary.csv`；
- `results/phase12a_resolution1deg_validation05.csv`；
- `results/phase12a_resolution1deg_validation05_wide.csv`；
- `results/phase12a_resolution1deg_vs_2deg.csv`；
- `benchmark/run_phase12a_1p5.m`；
- `benchmark/compare_phase12a_resolution1deg.m`。

截至本阶段，Phase 12A-2 尚未启动。Phase 1–11、Phase 12A 单元测试和本阶段回归均通过，标记为
`PHASE12A15_REGRESSION_PASS`。

## Phase 12A-2：六维速度/路径长度 Pilot（部分完成，未形成正式 480-run 结论）

按用户冻结的评价目标启动了 6 cases × 4 algorithms × 20 seeds Pilot：

- planner edge resolution：`0.5 deg`；
- independent final validation：`0.25 deg`；
- 只统计 planning speed 与 joint-space path length；
- 未进行 trajectory timing、APF、ROS2 或 CoppeliaSim。

并行分片中，C01、C02、C03、C04、C06 已完成并保存，共 `400/480` 条记录：

- success：`392/400`；
- 0.25° final validation：`391/400`；
- C01 中已发现一条 `planner success=1` 但 `finalValidationPass=0` 的记录
  （Bi-RRT*、seed 18），因此 `0.5° planning + 0.25° validation` 不能表述为已证明无漏检。

C05（`Home_to_Point1`，靠近工件/焊枪的诊断案例）运行中 Full Classified-RRT* 出现极端长尾：
多个 seed 达到约 400–940 s，seed 18 在超过 4 h 后仍未返回。为避免继续无界占用计算资源，
该单次运行被安全中止；由于旧版 runner 只在整个 case 完成后写出最终 MAT/CSV，C05 已完成的
中间记录没有落盘。因此当前不能把已有数据称为完整 480-run Pilot，也不能据此做最终算法排名。

部分汇总文件（仅用于诊断，不用于正式论文结论）：

- `results/phase12a_pilot_partial_400.mat`；
- `results/phase12a_pilot_partial_400_runs.csv`；
- `results/phase12a_pilot_partial_400_summary.csv`；
- `results/phase12a_pilot_partial_400_figures/`。

部分结果的工程信号已经很清楚：RRT-Connect/Bi-RRT* 通常为几十秒量级；ClassGoal-Lite 与
尤其 Full Classified 在 C01/C05 出现数百秒到约 15 min 的长尾，且 C05 存在多次失败。
这些时间是离线 MATLAB 碰撞检查的 planning time，不是机器人执行时间。

合并器同时修复了旧分片中 `cases` 行/列方向不一致的问题：
`benchmark/merge_phase12a_pilot_shards.m` 现在会统一按列拼接 cases。

当前下一步不是直接宣布算法优劣，而是先改造 Pilot runner：

1. 每个 run 保存 checkpoint，而不是只在整个 case 完成后保存；
2. 增加明确的单次规划 wall-clock 上限/超时状态；
3. 重新从 C05 未完成部分开始，或将 C05 单独作为“超时/不可实时”诊断实验；
4. 只有所有 case 的记录齐全后，才生成正式 480-run 汇总。

因此当前状态标记为 `PHASE12A2_PARTIAL_400_C05_INCOMPLETE`，不要把
`phase12a_pilot_partial_400` 当作完整 Pilot，也不要进入 Phase 12B。

## Phase 12A-2R：Seam A 两段任务定标（2026-09-28）

根据研究问题重新冻结 benchmark 任务，不再围绕 C01--C06 比较。当前 Seam A 的焊接段
`Point1 -> Point80` 保持已有 80 点连续路径，不使用 RRT 重新规划。自由空间规划只负责：

- `T01_Safe_to_Point1`：`qHome(qSafe) -> qWeld(1,:)`；
- `T02_Point80_to_Safe`：`qWeld(end,:) -> qHome(qSafe)`。

四个比较算法仍为 RRT-Connect、Bi-RRT*、ClassGoal-Lite 和 Full Classified-RRT*。
Phase 12A-2R 的预定定标配置为：planner 内部 `0.25 deg` 离散 joint-edge 检查、独立
`0.125 deg` 路径复核、每次运行统一 `2 s` wall-clock 上限、每个任务 5 个 seed，共 40 次。
每个运行结果明确分为 `VALID_SUCCESS`、`TIMEOUT`、`PLANNER_FAIL` 或 `INVALID_PATH`。

40-run 定标已完成，但结果不是“通过”：

- `40/40` planner timeout；
- `0/40` 首条合法路径；
- `0/40` final validation pass；
- 两个任务的起点和终点均无碰撞；
- 两条任务的直连边均碰撞，因此不是端点定义错误；
- 额外进行的 8 个单 seed、10 s 可行性诊断也为 `0/8` success。

当前观测到的主要瓶颈是 UR5 + 焊枪模型的 `checkCollision` 构型检查：在 `0.25 deg`
插值下，10 s 诊断也只完成约 2.7k--3.2k 次构型检查，尚未完成一次有效绕障连接。这说明
`0.25 deg planning + 2 s` 在当前 MATLAB 离线碰撞实现和这条 Seam A 任务上尚不能作为可执行
定标配置。该结论不是算法排名，也不能用于比较四种算法优劣。

本阶段结果文件：

- `results/phase12a_2r_calibration_40.mat`；
- `results/phase12a_2r_calibration_40_runs.csv`；
- `results/phase12a_2r_calibration_40_summary.csv`；
- `results/phase12a_2r_calibration_40.mat.checkpoint.mat`。

运行器已支持每个 run checkpoint、明确 timeout 状态和 Seam A 两任务定义：
`benchmark/run_phase12a_pilot.m`、`benchmark/run_phase12a_2r_calibration.m`、
`benchmark/run_phase12a_2r_pilot.m`。Phase 12 benchmark 运行默认设置 `RRT_HEADLESS=1`，
不会创建场景 figure；需要人工显示时才显式设置 `RRT_SHOW_SCENE=1`。场景加载脚本中的显示
区段已改为条件执行，实际检查 `HEADLESS_FIGURES=0`。

当前状态标记为 `PHASE12A2R_CALIBRATION_TIMEOUT_NOT_PASSED`。在用户重新确定“碰撞分辨率与
时间预算”的折中前，不启动 160-run Pilot，不进入 Phase 12B，不迁移 APF，也不做最终算法结论。

## Phase 12A-2R2：两级碰撞检测定标（2026-09-28）

根据上一阶段结果，搜索与安全认证拆开：planner 使用较粗的内部离散分辨率；每个内部连通
候选路径立即使用 `0.125 deg` 独立离散插值复核。只有通过该复核的路径才算
`VALID_SUCCESS`，认证耗时也计入同一个 `2 s` wall-clock deadline。认证失败的候选路径不计
成功，搜索继续到超时。

公共 planner core 新增 `candidateValidatorFcn(path,abortFcn)` 接口，并记录：
`firstCandidateTime`、`firstCertifiedTime`、`candidateCount`、`candidateValidationFailures`。
严格认证的构型/边检查也计入统一 collision statistics。新增运行器：
`benchmark/run_phase12a_2r2.m`。

定标任务仍只有：

- `T01_Safe_to_Point1`：`qHome -> Point1`；
- `T02_Point80_to_Safe`：`Point80 -> qHome`。

四个算法、5 个 seed、2 个任务分别测试了 planner 内部 `0.5 deg`、`0.75 deg`、`1.0 deg`，
每组都是 40 runs；三组的初始结果均为 `0/40 certified success`。其中：

- `0.5 deg`：`0/40` certified，40 次均在形成候选前 timeout；
- `0.75 deg`：`0/40` certified，40 次均在形成候选前 timeout；
- `1.0 deg`：初始组 `0/40` certified，仅出现 1 个被 `0.125 deg` 拒绝的候选；
- 对 1.0 deg 做了同一算法/任务/seed 的缓存优化后完整重跑，仍为 `0/40` certified；
  Task B 出现若干候选，但严格认证全部拒绝，随后 timeout。

因此当前不是“0.125 deg 认证太严格导致少量失败”的问题：Task A 主要无法在 2 s 内形成
候选，Task B 的候选也无法通过严格认证。0.5/0.75/1.0 deg 都没有找到可进入 160-run
Pilot 的正式搜索配置，不能启动 160-run，也不能做算法排名。

Profiler 代表性结果（T01、seed 1、1 deg、2 s）：

- RRT-Connect：约 `331` 个实际构型检查；
- Full Classified-RRT*：约 `329` 个实际构型检查；
- 主要耗时在 `collisionFreeJointState -> checkCollision`，每个构型包含环境和自碰撞检查；
- Full Classified 额外耗时集中在 repeated edge / parent / goal-tree 相关检查。

在不改变碰撞几何判定的前提下，planner 内部新增了单次 run 的：

- state collision cache；
- undirected edge result cache；
- `stateCacheHits`、`edgeCacheHits` 统计。

缓存后 1.0 deg、2 s 的实际构型检查能力有所提升，但仍为 `0/40` certified。严格
`0.125 deg`认证不复用搜索缓存，保持独立安全验收。joint-edge 和 6D planner 回归测试已通过。

结果文件：

- `results/phase12a_2r2_0p5deg_40.mat`；
- `results/phase12a_2r2_0p75deg_40.mat`；
- `results/phase12a_2r2_1p0deg_40.mat`；
- `results/phase12a_2r2_1p0deg_cached_40.mat`；
- `results/phase12a_2r2_profile_RRTConnectJoint.mat`；
- `results/phase12a_2r2_profile_ClassifiedRRTStarJoint.mat`。

说明：0.5/0.75 deg 和缓存修复前的 1.0 deg 文件保留作分辨率诊断；它们的
`certifiedSuccess/outcome` 可用于本阶段结论，但碰撞计数以修复计数逻辑后的
`phase12a_2r2_1p0deg_cached_40.mat` 为准。

当前状态标记为 `PHASE12A2R2_NO_CERTIFIED_SOLUTION_2S`。在继续实验前，应优先进行
collision-check profiling/优化（重复边缓存、批量/低层碰撞调用、Full Classified 的
goal-tree/rewire 检查），或重新确定可接受的时间预算；不要继续把内部离散分辨率机械地压细，
也不要启动 160-run Pilot、Phase 12B、APF、ROS2 或 CoppeliaSim。

## 恢复主线：Phase 12A Seam approach/exit Pilot（2026-09-28）

Phase 12A-2R2 已降级为碰撞分辨率诊断，不再作为主实验入口。重新恢复已经实际跑通的
Seam A 任务结构：

```text
qHome(qSafe) -> qPreWeld       RRT自由空间规划
qPreWeld -> Point1             已冻结的局部安全连接
Point1 -> Point80              已验证的80点焊缝路径，不重新规划
Point80 -> qPostWeld           已冻结的局部安全连接
qPostWeld -> qHome(qSafe)      RRT自由空间规划
```

原始脚本 `/home/ttbstt/jixiebi/UR5/matlab仿真/plan_weld_approach_exit_rrt_connect.m`
已用新输出文件复现，未覆盖历史结果：

- Approach `qHome -> qPreWeld`：`12.220 s`，PASS；
- Exit `qPostWeld -> qHome`：`6.787 s`，PASS；
- PreWeld/PostWeld 30 mm 局部连接 PASS；
- 0.5° dense validation PASS；
- Overall PASS。

复现结果：

- `results/phase12a_seam_legacy_rrtconnect_repro.mat`；
- `results/phase12a_seam_legacy_rrtconnect_repro_report.txt`。

新建四算法共同 wrapper：`benchmark/run_phase12a_seam_approach_exit_pilot.m`。四个算法只替换
planner，任务、UR5/焊枪/工件、joint limits、metric、碰撞接口和随机种子规则保持共同配置。
本 Pilot 使用历史 RRT-Connect 的碰撞规则：

- planner edge interpolation：`2°`；
- independent dense validation：`0.5°`；
- joint step：`12°`；
- goal bias：`0.20`；
- max iterations：`8000`；
- 每次运行 `30 s` 仅作为防卡死保护，不是算法准入目标；
- 2 tasks × 4 algorithms × 5 seeds = 40 runs。

40-run 结果如下，必须按 Task 分开解释：

| Task / Algorithm | Success | Median planning time | P90 planning time | Median Lq |
|---|---:|---:|---:|---:|
| Safe -> PreWeld / RRT-Connect | 4/5 | 7.600 s | 13.325 s | 8.8953 rad |
| Safe -> PreWeld / Bi-RRT* | 5/5 | 10.643 s | 17.544 s | 5.7271 rad |
| Safe -> PreWeld / ClassGoal-Lite | 0/5 | 30.002 s | 30.004 s | -- |
| Safe -> PreWeld / Full Classified-RRT* | 0/5 | 30.003 s | 30.004 s | -- |
| PostWeld -> Safe / RRT-Connect | 4/5 | 0.756 s | 1.180 s | 5.4410 rad |
| PostWeld -> Safe / Bi-RRT* | 5/5 | 6.150 s | 14.992 s | 5.1473 rad |
| PostWeld -> Safe / ClassGoal-Lite | 5/5 | 0.721 s | 2.255 s | 4.5951 rad |
| PostWeld -> Safe / Full Classified-RRT* | 3/5 | 1.136 s | 1.913 s | 4.5702 rad |

总体为 `26/40` valid success、`10/40` timeout、`4/40` planner-success 但 0.5° dense
validation 失败。失败路径不计入 success，也不改变实验主线；它们作为原始 2°规划与更密验证
之间的安全诊断记录保存。碰撞分辨率问题不再升级为 2 秒准入门槛。

结果文件：

- `results/phase12a_seam_approach_exit_pilot_40.mat`；
- `results/phase12a_seam_approach_exit_pilot_40_runs.csv`；
- `results/phase12a_seam_approach_exit_pilot_40_summary.csv`；
- `results/phase12a_seam_approach_exit_pilot_40.mat.checkpoint.mat`。

当前有效状态标记为 `PHASE12A_SEAM_APPROACH_EXIT_PILOT_40_COMPLETE`。下一步若继续，只能在
这条恢复主线上扩大到 20 seeds；暂不做 2 秒硬门槛、0.125°认证循环、Phase 12B、APF、ROS2
或 CoppeliaSim 扩展。

## Phase 12A-0R：原始基线与恢复版差异审计（2026-09-28）

用户要求先读取并核对主交接文档 `/home/ttbstt/jixiebi/AGENT_HANDOFF.md`，暂停新的
benchmark。审计结果已保存至：

- `docs/phase12a0r_original_vs_restored_audit.md`

关键纠正：原始规划脚本并非完全没有 `0.5°` 后验验证。其默认参数明确为：

```text
RRT 搜索边插值：2°
独立路径 validation：0.5°
动画自由空间帧插值：独立参数 stepDeg，当前默认 1°
```

所以旧文档没有写清楚 `0.5°` 的来源，但不能再说 `0.5°` 只来自动画。与此同时，原始脚本
与当前恢复版 wrapper 仍有重要实现差异：当前 wrapper 使用共享 `jointPlannerCore`，增加了
state/edge cache、内部候选路径复核和 30 s wall-clock guard；当前 `runConnect` 的树选择/交换
代码也不是原始本地 `localRrtConnect` 的逐行复现。故此前 40-run 结果只能标记为：

```text
diagnostic / restored-pilot attempt under current shared core
```

不能直接作为 `original RRT-Connect ground-truth benchmark`。当前审计阶段没有启动新的
40-run、160-run 或 100-seed 实验。后续应先决定是否把原始 RRT-Connect 封装为严格 ground-truth
实现，再让其他算法仅替换搜索器并共用相同碰撞接口、插值规则和成功判据。

## Phase 12A anytime 首次方法验证（2026-09-28）

研究目标进一步固定为：在原始 Seam A 环境和碰撞条件不变的情况下，同时记录首解时间/首解路径
长度，以及统一 30 s 预算内的 best-so-far 路径长度。新增入口：

```text
benchmark/run_phase12a_anytime_home_preweld.m
```

固定条件：任务 `qHome -> qPreWeld`；RRT 搜索边 `2 deg`；独立后验验证 `0.5 deg`；关节扩展步长
`12 deg`；目标偏置 `0.20`；总预算 `30 s`。RRT-Connect 在首个认证路径后停止；三个 RRT*
变体继续搜索，只有通过同一后验验证的候选才更新 best path。入口设置 `RRT_HEADLESS=1`，不会
弹出 MATLAB 场景窗口。

`seed=1` 单次方法验证结果如下，暂不作为最终排名：

```text
RRT-Connect       T_first=3.058 s, L_first=L_best@30=5.7018 rad
Bi-RRT*           T_first=8.524 s, L_first=6.3495 rad, L_best@30=5.1845 rad
ClassGoal-Lite    30 s 内未找到认证首解
Full Classified   30 s 内未找到认证首解
```

Bi-RRT* 的 checkpoint 为：`L@10=6.3495`、`L@15=6.3495`、`L@20=5.2487`、
`L@30=5.1845 rad`；RRT-Connect 首解后保持 `5.7018 rad`。

结果文件：

```text
/home/ttbstt/jixiebi/RRT/results/phase12a_anytime_home_preweld_seed1.mat
/home/ttbstt/jixiebi/RRT/results/phase12a_anytime_home_preweld_seed1.csv
```

时间—路径质量图：

```text
/home/ttbstt/jixiebi/RRT/results/phase12a_anytime_home_preweld_seed1_time_quality.png
```

回归已运行：`test_joint_distance`、`test_joint_wrap`、`test_joint_steer`、`test_joint_limits`、
`test_joint_edge_collision` 和 `test_phase12a_joint_planners`；默认 planner 仍保持首解模式，
RRT-Connect/Bi-RRT* smoke 路径有效。

这只是方法验证。后续如继续，只增加固定 seed 和 `qPostWeld -> qHome` 任务，不改变原始碰撞
分辨率，也不恢复 0.25/0.125 deg 认证支线。

## Phase 12A anytime 回放与原始 RRT 对照（2026-09-28）

> 后续修正（2026-09-28，见文末“Phase 12A anytime 完整任务链视频与配色修正”）：本节列出的两个
> `phase12a_anytime_home_preweld_*.avi` 有两个问题——Figure 无光照导致配色偏离主规划场景，
> 且只回放了 `Home -> PreWeld` 一段，没有 `PostWeld -> Home` 退出段。它们已移入
> `results/superseded/`，现行交付物是四个 `phase12a_anytime_full_sequence_*.avi`。

用户只要求保留本次 30 s 方法验证中两个成功算法的视频，不再生成原始 RRT 基线视频。视频均为
MATLAB 离线回放，任务是 `Home -> PreWeld`，使用同一 UR5 + 焊枪 + 工件场景，不发送
ROS2/CoppeliaSim 指令：

```text
results/phase12a_anytime_home_preweld_RRTConnect.avi   （已废弃，见上）
results/phase12a_anytime_home_preweld_BiRRT.avi        （已废弃，见上）
```

其中 RRT-Connect 回放首条认证路径；Bi-RRT* 回放 30 s 结束时的 best-so-far 认证路径。
视频生成入口为：

```text
benchmark/animate_phase12a_anytime_home_preweld.m
```

为响应用户提供的历史 RRT 表格，已生成数值对照：

```text
results/phase12a_anytime_vs_original_rrt.csv
results/phase12a_anytime_vs_original_rrt.png
benchmark/compare_phase12a_anytime_with_original_rrt.m
```

对照中保留原始 RRT-Connect 的两段基线：当前重现值为 `Home -> PreWeld = 12.219553 s / 19.842372 rad`，
`PostWeld -> Home = 6.786776 s / 17.255930 rad`；截图中的 `11.898 s / 6.753 s` 是历史运行时间，
差异属于运行耗时波动。30 s 方法验证只包含 `Home -> PreWeld`：RRT-Connect 首解为 `3.058 s / 5.7018 rad`；
Bi-RRT* 首解为 `8.524 s / 6.3495 rad`，30 s best-so-far 为 `5.1845 rad`。这些值用于方法验证和
与历史 RRT 基线的说明性对照，暂不作为多 seed 最终排名。


## Phase 12A anytime 完整任务链视频与配色修正（2026-09-28）

用户复核指出两点：(1) 视频颜色不对；(2) 视频只覆盖安全点到焊缝起点，焊接终点回到安全点
没有做。两条均已定位并修正。

### 配色基准与根因

主规划场景（`sync_ur5_joint_states.m` 建立的 Figure）只有 1 个光源：
`Position [1.5 0 1.5]`、`Style 'local'`、`Color [1 1 1]`。MATLAB 的 `show()` 只有在**它自己
创建 axes** 时才会加这个光源；如果先 `axes(fig)` 建了空 axes 再 `show(...,'Parent',ax)`，
Figure 就是无光照的，patch 会直接显示原始 FaceColor。同一帧的像素均值实测：

| 来源 | 机器人绿 RGB 均值 | 工件橙 RGB 均值 |
| --- | --- | --- |
| 主规划场景（基准） | (42, 111, 36) | (114, 58, 1) |
| 修复前 `home_preweld` 视频 | (69, 172, 62) | (252, 129, 5) |
| 修复后 `full_sequence` 视频 | (41–46, 101–112, 36–41) | (107–119, 56–65, 3–7) |

`animate_phase12a_anytime_home_preweld.m` 里还有第二个同源问题：每轮循环开头的
`cla(ax,'reset')` 会把光源一起删掉。修正方式：

- 改成让 `show()` 自己创建 axes（与主场景一致），不再预先 `axes(fig)`；
- 用新的 `localClearDrawn` 只删 line/patch/text/surface/quiver，替代 `cla(ax,'reset')`；
- 新增 `localRequireSceneLight`，录制前若 `findall(ax,'Type','light')` 为空直接报错，
  拒绝按错误配色出片。

同类问题一并修正：

- `benchmark/animate_weld_rrt_connect_sequence.m`：原来依赖 `gcf/gca` 恰好带着光源，
  现在缺光源时显式补 `light(ax,'Position',[1.5 0 1.5],'Style','local','Color',[1 1 1])`。
- `UR5/matlab仿真/record_three_weld_videos.m`：`camlight('headlight')` 正面打光，把工件照得
  比主场景亮约 65%（橙 183–190 vs 114），已替换为同一个主场景光源；同时把默认 `resultDir`
  从已改名的 `fair_task_pre5_runs` 改到 `task_level_arc_vs_gao_runs`，并保留旧目录回退。

### 完整任务链回放

`run_phase12a_anytime_seamA_two_successful.m` 本来就同时规划了两段自由空间转场
（`Home -> PreWeld` 与 `PostWeld -> Home`），两条路径都通过 `0.5 deg` 独立后验验证
（CSV：`results/phase12a_anytime_seamA_two_successful.csv`，四行 success=1）。缺的只是回放。
新增回放入口：

```text
benchmark/animate_phase12a_anytime_sequence.m
```

回放链：`Home -> PreWeld -> Point 1 -> 焊缝 1..80 -> Point 80 -> PostWeld -> PostWeld -> Home`。
自由空间段按关节插值（1 deg），焊接段按 TCP 弧长重参数化（43 mm/s，与 §84 一致）。
RRT-Connect 回放首条认证路径，Bi-RRT* 回放 30 s 预算结束时的 best-so-far 认证路径。
脚本在缺接近段或缺退出段时直接报错，不再可能产出只有半条任务链的视频。
每次录制前会把出图区固定为要求的 `1394×840`（首帧前先 `drawnow` 稳定窗口并校正
Figure 尺寸），避免窗口未映射时首帧变成 `1394×818` 而把整段视频拖小。

分帧统计与产物（全部 `1394×840`、`30 fps`）：

```text
RRT-Connect  Home -> PreWeld 242 | PreWeld -> Point 1  4 | 焊缝 1..80 124 |
             Point 80 -> PostWeld 4 | PostWeld -> Home 266 | 合计 640 帧 ≈ 21.33 s
Bi-RRT*      Home -> PreWeld 187 | PreWeld -> Point 1  4 | 焊缝 1..80 124 |
             Point 80 -> PostWeld 4 | PostWeld -> Home 211 | 合计 530 帧 ≈ 17.67 s

results/phase12a_anytime_full_sequence_RRTConnect_perspective.avi
results/phase12a_anytime_full_sequence_RRTConnect_top.avi
results/phase12a_anytime_full_sequence_BiRRT_perspective.avi
results/phase12a_anytime_full_sequence_BiRRT_top.avi
```

配色复测（逐帧像素均值，基准为主规划场景的绿 `(42,111,36)` / 橙 `(114,58,1)`）：

```text
perspective  RRT-Connect  green=(42,109,37)  orange=(111,57,3)
perspective  Bi-RRT*      green=(43,106,37)  orange=(112,59,5)
top          RRT-Connect  green=(42, 99,38)  orange=(169,88,7)
top          Bi-RRT*      green=(44,101,39)  orange=(173,93,11)
```

俯视图的橙色更亮是主场景自身的光照几何，不是配色错误：用同一个主规划场景 Figure（同光源、
同材质）把视角切到近俯视后实测橙色为 `(169.3, 86.1, 2.3)`，与新视频的 `(169–173)` 一致；
俯视时工件上表面正对 `[1.5 0 1.5]` 的局部光源，本来就比斜视图更亮。


### 旧视频归档

以下视频已移入 `results/superseded/`，不再作为交付物：

| 文件 | 问题 |
| --- | --- |
| `phase12a_anytime_home_preweld_RRTConnect.superseded.avi` | 无光照配色 + 只有接近段 |
| `phase12a_anytime_home_preweld_BiRRT.superseded.avi` | 无光照配色 + 只有接近段 |

同名 `.unlit-home_preweld_only.avi` 是上述文件的逐字节备份。旧入口
`animate_phase12a_anytime_home_preweld.m` 保留（配色已修），但只回放接近段，
新工作应使用 `animate_phase12a_anytime_sequence.m`。

### 三焊缝视频的配色

`weld_seam_videos/`（43 mm/s）与 `weld_seam_videos_100cmmin/`（100 cm/min）共 12 个 AVI
在修复前均为 `camlight('headlight')` 打光，工件橙色均值 `(183–190, 92–95, 2–4)`，
与主场景 `(114, 58, 1)` 明显不符（机器人绿色只差 ~7%）。这 12 个视频已按 §136 的
“保留原场景颜色”要求用主场景光源重录，原始文件备份为同目录下的 `.before-light-fix.avi`。

另外 `weld_seam_videos_40cmmin/Seam_A_AARC_IK_40cmmin_perspective.avi` 是一次中断的运行
留下的未关闭 AVI（`VideoReader`/OpenCV 均无法解析帧数与帧率），已重命名为
`.incomplete-aborted-run.avi` 保留，不作为交付物。

## 三个规划代码的截图格式对比（2026-09-28）

按用户提供的格式生成了三个代码的并列对比：

```text
RRT（原始基线）
RRT-Connect（当前共享接口）
Bi-RRT*
```

每个算法分别列出 `Home -> PreWeld` 和 `PostWeld -> Home` 的结果、规划时间、迭代次数、
碰撞检查和关节空间路径长度。首解版与 30 s best-so-far 版分别输出：

```text
results/phase12a_three_algorithm_first_solution_comparison.png
results/phase12a_three_algorithm_30s_best_comparison.png
results/phase12a_three_algorithm_comparison.csv
benchmark/make_phase12a_three_algorithm_comparison.m
```

## 115. V1 Fast-Connect Anytime RRT*（2026-09-30）

V1 只实现“RRT-Connect 快速首解 + 双树 RRT* 首解后优化”，没有修改原有
`RRTConnectJoint`、`BiRRTStarJoint`、UR5 场景、碰撞模型、ROS2、CoppeliaSim 或焊缝路径。

新增文件：

```text
jointspace/FastConnectAnytimeRRTStarJoint.m
benchmark/run_v1_fast_connect_anytime_taskA.m
```

固定任务为 Seam A 的 Task A：`qHome -> qPreWeld`；搜索边使用原始 `2 deg`，最终独立
dense validation 使用原始 `0.5 deg`。V1 Stage 1 保持双树交替 Extend/Connect；首次连接后
进入 Stage 2，保留两棵树并执行 Bi-RRT* 风格 ChooseParent、Rewire、连接代价更新。

`seed=20260917` 单 seed、30 s 结果：

| 算法 | 首解时间/s | 首解路径/rad | 5 s | 10 s | 15 s | 20 s | 30 s | 0.5°复核 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| RRT-Connect | 0.900 | 5.701772 | 5.701772 | 5.701772 | 5.701772 | 5.701772 | 5.701772 | PASS |
| Bi-RRT* | 6.558 | 6.349535 | — | 5.276655 | 5.184512 | 5.184512 | 5.184512 | PASS |
| V1 FastConnectAnytimeRRTStar | 0.766 | 5.701772 | 5.212353 | 5.212353 | 5.173362 | 5.173362 | 5.173362 | PASS |

V1 improvement events：

```text
0.766 s   5.701772 rad
1.103 s   5.399815 rad
1.352 s   5.212353 rad
15.255 s  5.173362 rad
```

V1 Stage 1 使用 `35` 次迭代；独立 RRT-Connect 在同一 seed/端点/参数下也是 `35` 次迭代、
首解路径 `5.701772 rad`，首解行为对齐。三种算法最终路径均通过 `0.5 deg` dense validation。

结果文件：

```text
results/v1_fast_connect_anytime_taskA_seed20260917.mat
results/v1_fast_connect_anytime_taskA_seed20260917.csv
results/v1_fast_connect_anytime_taskA_seed20260917_improvement_events.csv
results/v1_fast_connect_anytime_taskA_time_quality.png
```

本阶段只完成 Task A 单 seed 功能验证；暂不自动扩展 Task B、多 seed、V2 自适应采样或 V3
选择性重连。

首解版使用 Bi-RRT* 首次通过统一后验验证的路径；30 s 版使用 Bi-RRT* 在 30 s 内通过验证的
best-so-far 路径。原始 RRT 基线没有继续优化，因此两个版本的原始 RRT 数值相同。

### 三焊缝任务链的范围

需要说明：三条焊缝的 A-ARC-IK 任务结果里 `approachPathPlanned=false`、
`exitPathPlanned=false`——任务级 benchmark 本身只定义了焊缝段，没有安全点到起焊点、收焊点
到安全点的转场。§136 的视频规格也只要求回放焊缝段。若要把三条焊缝也做成
`安全点 -> 起焊点 -> 焊缝 -> 收焊点 -> 安全点` 的完整任务链，需要给任务级 benchmark 增加
两段 RRT 转场并重新规划，这会改变已冻结的对比实验，需用户明确同意后再做。

## 116. V1 Seam A 双向任务验证（2026-09-30）

用户确认正式研究必须同时覆盖完整焊接自由空间任务的两个方向：

```text
Task A: qHome -> qPreWeld
        + 原有确定性 PreWeld -> Point1 局部接近

Task B: Point80 -> qPostWeld 局部退出
        + qPostWeld -> qHome
```

RRT 竞争只统计两个自由空间段；局部连接和 Point1 -> Point80 的已有焊缝路径不进入算法竞争。
新增入口：

```text
benchmark/run_v1_fast_connect_anytime_taskB.m
benchmark/compare_v1_taskA_taskB.m
```

Task B 使用与 Task A 完全相同的场景、seed、关节步长、2 deg 搜索边、0.5 deg 后验 dense
validation 和 30 s anytime 预算。seed `20260917` 的单次结果如下：

| 算法 | 任务 | 首解时间/s | 首解路径/rad | 10 s best/rad | 30 s best/rad | 0.5 deg 复核 |
|---|---|---:|---:|---:|---:|---|
| RRT-Connect | qPostWeld -> qHome | 1.896 | 6.688920 | 6.688920 | 6.688920 | PASS |
| Bi-RRT* | qPostWeld -> qHome | 8.170 | 5.140525 | 5.140525 | 5.095463 | PASS |
| V1 FastConnectAnytimeRRTStar | qPostWeld -> qHome | 1.784 | 6.688920 | 4.979579 | 4.729628 | PASS |

Task A + Task B 的联合辅助汇总：

| 算法 | 两段首解时间合计/s | 两段首解路径合计/rad | 10 s best 合计/rad | 30 s best 合计/rad | 两段均 PASS |
|---|---:|---:|---:|---:|---|
| RRT-Connect | 2.795 | 12.390692 | 12.390692 | 12.390692 | YES |
| Bi-RRT* | 14.728 | 11.490060 | 10.417184 | 10.279975 | YES |
| V1 FastConnectAnytimeRRTStar | 2.551 | 12.390692 | 10.191926 | 9.902990 | YES |

结果文件：

```text
results/v1_fast_connect_anytime_taskB_seed20260917.mat
results/v1_fast_connect_anytime_taskB_seed20260917.csv
results/v1_fast_connect_anytime_taskB_seed20260917_improvement_events.csv
results/v1_fast_connect_anytime_taskB_time_quality.png
results/v1_seamA_taskA_taskB_comparison.csv
results/v1_seamA_taskA_taskB_combined.csv
results/v1_seamA_taskA_taskB_combined.mat
```

本次只完成固定 seed 的 Task A/Task B 联合验证，不开始 V2、多 seed 正式排名或 Phase 12B
执行时间参数化。运行通过 `RRT_HEADLESS=1`，不会弹出 MATLAB 图形窗口；生成的 time-quality
图使用隐藏 figure，仅保存到文件。

## 117. V1 Seam A 完整路径规划视频（2026-09-30）

根据用户要求，使用已保存的 V1 Task A/Task B qPath 离线录制完整 Seam A 序列，没有重新规划，
也没有发送 ROS2/CoppeliaSim 命令。视频五段均存在：

```text
Home -> PreWeld
PreWeld -> Point 1
Welding Point 1 -> 80
Point 80 -> PostWeld
PostWeld -> Home
```

V1 共 527 帧，30 fps，时长约 17.57 s。录制脚本使用主场景光源，不添加 headlight，不覆盖
机器人/工件原有颜色；日志确认 `light=1`，机器人/工件颜色保持正常。输出：

```text
results/v1_fast_connect_anytime_videos/v1_seamA_full_sequence_V1FastConnectAnytimeRRTStar_perspective.avi
results/v1_fast_connect_anytime_videos/v1_seamA_full_sequence_V1FastConnectAnytimeRRTStar_top.avi
```

两个 AVI 均通过 `VideoReader`/`ffprobe` 验证：1394×840、Motion JPEG、30 fps、527 帧，
可正常读取。可复现入口：

```text
benchmark/make_v1_seamA_video_report.m
benchmark/animate_phase12a_anytime_sequence.m
results/v1_seamA_video_report.mat
```

本次视频明确包含 `PostWeld -> Home` 退出段，已不再是只有安全点到起焊点的单向演示。
