# Literature-guided improvements for ACCP

日期：2026-09-28  
范围：只读 Zotero 文献精读后的独立改进方案。当前正式 benchmark 和稳定 ROS2/CoppeliaSim 链路不修改。

## 当前基线

当前正式任务级 benchmark 已冻结：

- 3 条焊缝 × 30 个任务 × 2 种算法；
- ACCP 总体 PASS@60 s：87/90；
- ACCP 成功样本 TFF 中位数：4.048 s；
- 中位 IK calls：380；
- 中位最大关节跳变：7.460°；
- 中位最大段间 TCP 偏差：0.332 mm。

基线文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/replan_weld_actual_axis_adaptive.m
/home/ttbstt/jixiebi/UR5/matlab仿真/replan_weld_actual_axis_adaptive_core.m
/home/ttbstt/jixiebi/UR5/matlab仿真/task_level_arc_vs_gao.csv
```

正式数据不得被下面的实验覆盖。

## 精读文献及可用结论

### 1. 奇异位形和分支切换

**Boschi et al., “Handling Transitions Across Singularities for UR-Like Serial Robots”**  
Zotero key：`YS8CJXS4`

UR-like 6R 机器人可用肩部、腕部、肘部三个符号索引描述 8 个解析 IK 分支。文中用 `s3`、`s5` 和第三个姿态因子监视分支唯一性域；接近奇异位形时，不是全局扩大 beam，而是只评估相邻的替代符号分支，并选择与上一状态关节距离最小的解。腕部奇异 `s5≈0` 需要特殊的腕部四杆连续化处理。

对本项目的直接改进：

1. 在每个已接受状态记录 `branchSignature` 和奇异指标；
2. 只有 `min(abs(s3),abs(s5),abs(P3))` 低于阈值时，才扩展替代 IK 分支；
3. 分支选择代价使用关节位移、TCP 误差和碰撞间隙，而不是只用 IK 残差；
4. 保持现有 15° 关节跳变、焊接侧、焊丝碰撞和段边碰撞硬约束；
5. 对 `s5≈0` 先做诊断标记，暂不在没有 UR5 专用解析公式验证前强行替换数值 IK。

这项改进最可能提高当前 Seam A/C 困难任务的恢复成功率，而且正常样本几乎不增加计算量。

### 2. 局部候选库和最少重构连接

**Wang, Sifferman, Gleicher, “IKLink: End-Effector Trajectory Tracking with Minimal Reconfigurations”**  
Zotero key：`4Q4SHSAX`

论文在每个 waypoint 生成少量多样 IK 候选，再用动态规划连接候选。候选生成采用上一候选的贪心传播、随机补充和聚类去重；连接代价优先减少重构，再最小化关节运动。

对本项目的直接改进：

- 正常 ACCP 仍保持 `beam=1`；
- 只有局部失败窗口才建立候选表，例如每层 4--8 个候选；
- 用分支签名和关节距离去重，避免状态膨胀；
- 局部 DP 代价建议为：

```text
J = wq * ||Δq||²
  + wtcp * TCP_error
  + wclear * clearance_penalty
  + wbranch * branch_switch
```

- `CHECK_CONT` 必须复用现有的关节阈值、插值碰撞和 TCP 误差检查；
- 不允许用“候选存在”替代“整条边通过”。

这比全局 layered graph 更符合当前按需恢复结构。

### 3. 预测式连续 IK 和非均匀速度缩放

**Woliński and Wojtyra, “Predictive inverse kinematics with trajectory scaling for redundant manipulators based on quadratic optimization”**  
Zotero key：`3ABDWD5D`

PQPIK-S 用短预测窗口预先观察未来若干步，同时把关节位置、速度、加速度约束放进 QP；只有在未来约束无法同时满足时才局部降低轨迹进度。论文显示预测窗口可以避免单步缩放造成的加速度尖峰和突然的缩放因子变化，但会增加计算量和 TCP 误差。

对本项目不建议直接移植完整 QP，而建议只借鉴前瞻机制：

1. 在当前候选状态上预测后面 2--3 个焊缝点的 `qPred`；
2. 将预测的关节限位余量、奇异指标和最小碰撞间隙加入候选评分；
3. 如果预测窗口内的余量持续下降，提前触发局部分支候选，而不是到碰撞后才恢复；
4. 速度缩放只用于执行时间参数化，不得用它掩盖 MATLAB 规划阶段的节点/边失败。

### 4. 船舶狭小空间的分层碰撞和姿态反馈

**Du et al., “Spatial Obstacle Avoidance for Dual-Arm Welding Robots in Ship Hull Subassemblies and Intermediate Assemblies”**  
Zotero key：`J5XQXFXT`

论文先按焊缝局部几何构造默认枪姿，再用 GJK 距离评估姿态；碰撞后区分末端局部碰撞和中后段连杆碰撞。末端碰撞优先改枪姿，中后段碰撞才改变整体构型/IK 初值；同时采用分层的保守包络进行快速筛选。

对本项目的直接改进：

- 保留当前精确碰撞检测作为最终判定；
- 增加廉价 broad-phase：先用代理包围盒/线段-凸体距离筛掉明显安全或明显危险状态；
- 将碰撞来源分为 `tool/wire`、`wrist/end-link`、`mid/rear-link`、`self-collision`；
- `tool/wire` 触发姿态候选，`mid/rear-link` 触发 IK 分支/起始 seed 候选；
- 任何 broad-phase 结果都不能直接替代最终插值碰撞检查。

### 5. 由焊缝几何协同生成焊枪姿态

**Wang et al., “Innovation in robot welding trajectory control: Online synchronous planning of welding paths and torch postures for structures with unknown spatial geometries”**  
Zotero key：`H37G9RPF`

论文从局部焊缝几何建立截面平面，使用焊缝两侧/根部点构造角平分方向，使工具轴与接头几何协同变化，而不是把 RX/RY/RZ 当作互相独立的参数。文中对角焊缝的路径 RMSE 约 0.278 mm、姿态误差约 0.5--1.1°。

对本项目的直接改进：

- 保持当前实际焊丝轴作为真实工作轴；
- 为每个焊点保存 `tangent`、两个板面法向和焊接侧；
- 用局部截面/法向计算允许姿态走廊，`travel` 和 `roll` 只在走廊内采样；
- 将焊接侧硬约束放在 IK 之前过滤，减少无效 IK 调用；
- 继续用当前 WeldTCP 标定，不因姿态生成而改变 TCP。

## 推荐的独立实验版本

不要修改正式基线。新增实验版本应按以下顺序实现：

### Phase 1：只做诊断，不改规划结果

新增一个后处理脚本，沿现有最终 `qPath` 计算：

- 每个点的 Jacobian 最小奇异值；
- `s3`、`s5`、`P3` 或等价的 UR5 腕部/肘部指标；
- 关节限位余量；
- 最小工具/焊丝/工件间隙；
- 分支签名变化位置。

先确认 90 s 预算内失败样本是否集中在奇异指标过小、关节边界或工具碰撞附近。

### Phase 2：奇异触发的局部分支恢复

只在触发窗口生成 4--8 个候选：

```text
qPred
qPrev
qPred ± δq4
qPred ± δq5
qPred ± δq6
相邻 branch signature（仅当奇异指标低于阈值）
```

候选经过现有硬约束后，用局部 DP 连接 2--3 层前瞻窗口。正常区段仍是单分支 continuation。

### Phase 3：预测式触发

在当前点之外预看 2--3 个点；若预测到：

```text
奇异指标持续下降
或关节限位余量过小
或最小间隙下降
```

则提前启动 Phase 2。否则不增加 IK 调用。

## 验收方法

新实验只能写入新的文件，例如：

```text
literature_guided_arc_ik_experiment.m
literature_guided_arc_ik_results.csv
```

不得覆盖：

```text
task_level_arc_vs_gao.csv
weld_actual_axis_replan*.mat
```

先用正式 benchmark 之外的新 seeds 做 20 个任务预实验，再决定是否纳入正式数据。比较：

- PASS@60 s；
- TFF；
- 触发率；
- IK calls；
- 最大关节跳变；
- 最大段间 TCP 偏差；
- 工具/焊丝/连杆碰撞来源；
- 与冻结基线的回归差异。

## 结论

当前最值得实现的是：

> **奇异位形感知的局部分支扩展 + IKLink 风格的局部候选 DP + 2--3 点前瞻触发。**

不建议现在移植 Gao 的完整 RRT*、不建议全局 beam 增大，也不建议直接移植 PQPIK-S 的完整 QP。前者会破坏当前速度优势，后两者需要重新验证工具标定、时间参数化和 TCP 精度。

