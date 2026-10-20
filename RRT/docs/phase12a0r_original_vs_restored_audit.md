# Phase 12A-0R：原始 RRT-Connect 与当前恢复版 wrapper 差异审计

更新时间：2026-09-28（Asia/Shanghai）  
项目：`/home/ttbstt/jixiebi/RRT`  
参照交接文档：`/home/ttbstt/jixiebi/AGENT_HANDOFF.md`

## 结论先行

1. 原始 RRT-Connect 的任务定义确实是：
   - `qHome → qPreWeld`
   - `qPostWeld → qHome`
   然后固定连接 `PreWeld → Point 1`、已有 `Point 1 → Point 80`、固定连接 `Point 80 → PostWeld`。

2. `0.5°` **不是只来自动画**。原始规划脚本
   `UR5/matlab仿真/plan_weld_approach_exit_rrt_connect.m` 的默认参数明确包含：

   ```matlab
   edgeResolutionDeg       = 2
   validationResolutionDeg = 0.5
   ```

   原始 RRT 搜索边使用 `2°` 插值；规划完成后，`localValidatePath` 使用 `0.5°` 重新验证路径。因此更准确的说法是：旧交接文档没有把这个 `0.5°` 规则写清楚，而不是原始代码完全没有它。

3. 动画中的 `stepDeg` 是另一件事。当前动画默认 `stepDeg=1.0°`，自由空间动画帧使用该值插值；焊接段还使用 `denseStepDeg=max(stepDeg/8,0.05)` 做 FK/弧长重参数化。它不参与 RRT 搜索成功判定。

4. 当前恢复版 wrapper 与原始基线在“名义碰撞分辨率”上相同（规划 `2°`、后验验证 `0.5°`），但**并不是原始 RRT-Connect 的逐行等价实现**。最重要的实现差异是：
   - 当前 RRT-Connect 改用共享 `jointPlannerCore`；
   - 当前 core 增加了状态缓存和边缓存；
   - 当前 core 增加了 `considerCandidate/validatePath` 内部路径复核；
   - 当前 core 增加了 30 s wall-clock guard；
   - 当前 `runConnect` 的树交换/主动树选择写法与原始本地实现不同，不能直接宣称是原始算法的严格复现。

5. 因此此前 40-run 结果应继续标记为 `diagnostic / restored-pilot attempt`，不能当成原始基线下的正式四算法比较。当前审计阶段没有启动新的 benchmark。

## 证据来源

### 原始交接文档

`/home/ttbstt/jixiebi/AGENT_HANDOFF.md` 第 84 节记录了：

- `qHome → qPreWeld` 和 `qPostWeld → qHome`；
- `step=12°`、双向 RRT-Connect、固定种子 `20260917`；
- 历史规划时间约 `11.898240 s` 和 `6.752958 s`；
- 搜索阶段 `edge checks` 与独立 `dense validation` 均通过；
- 五段完整路径后来被发送到 CoppeliaSim 并回到 Home。

该文档没有明确写出 `dense validation` 的角度分辨率，这也是此前概念串线的来源。

### 原始规划脚本

文件：

`/home/ttbstt/jixiebi/UR5/matlab仿真/plan_weld_approach_exit_rrt_connect.m`

关键位置：

- 第 25–30 行：`stepDeg=12`、`goalBias=0.20`、`maxIterations=8000`、`edgeResolutionDeg=2`、`validationResolutionDeg=0.5`、`randomSeed=20260917`；
- 第 226–272 行：原始本地双树 RRT-Connect；
- 第 318–350 行：节点和边碰撞检查；
- 第 352–370 行：独立路径验证。

### 当前恢复版 wrapper

文件：

`benchmark/run_phase12a_seam_approach_exit_pilot.m`

关键位置：

- 第 13–24 行：当前默认参数；
- 第 46–50 行：从冻结的原始结果加载 `qHome/qPreWeld/qPostWeld`；
- 第 72–79 行：调用共享四算法 wrapper，然后再执行当前 wrapper 的路径验证；
- 第 153–176 行：当前后验验证函数。

### 当前共享规划 core

文件：

`jointspace/jointPlannerCore.m`

关键位置：

- 第 35–48 行：每次运行建立 state/edge cache；
- 第 180–209 行：当前 `runConnect`；
- 第 282–320 行：候选路径内部复核和可选严格认证接口；
- 第 594–654 行：缓存包装的状态/边碰撞检查；
- 第 656–669 行：当前 core 的内部路径复核；
- 第 671–680 行：wall-clock timeout。

## 逐项差异表

| 项目 | 原始 RRT-Connect | 当前恢复版 wrapper / core | 结论与影响 |
|---|---|---|---|
| 任务端点 | `qHome→qPreWeld`；`qPostWeld→qHome` | 从原始重跑 MAT 冻结加载同样的 `qHome/qPreWeld/qPostWeld` | 任务定义一致；当前 wrapper 不重新执行 Pre/PostWeld IK 选择 |
| Pre/PostWeld 选择 | 每次脚本运行时从 Point 1/80 重新做候选 IK，选 30 mm 等候选并检查端点边 | 直接读取原始成功结果中的冻结姿态 | 本次 40-run 的几何端点与原始重跑一致，但选择过程没有纳入每个 run |
| 关节单位与角度处理 | rad；`localWrapRad` 使用最短角差 | rad；`jointWrapDelta/jointDistance` 使用最短角差 | 名义上等价 |
| 关节步长 | `stepDeg=12` | `maxJointStepRad=deg2rad(12)` | 一致 |
| 目标偏置 | `goalBias=0.20` | `goalBias=0.20` | 一致 |
| 最大迭代 | `8000` | `8000` | 一致 |
| 随机种子 | Approach=`20260917`；Exit=`20260917+7919` | seed 1 时同样使用这两个有效种子；其他 seed 在其上递增 | seed 1 可对齐；批量 seed 是扩展实验，不是历史单次复现 |
| 搜索边插值 | 每条边 `n=max(1,ceil(max(abs(dq_deg))/2))`，检查 `k=0:n` 共 `n+1` 个状态 | `collisionFreeJointEdge` 使用同样的 `ceil(max(abs(dq))/resolution)` 和端点包含规则 | 几何采样规则一致 |
| 搜索边分辨率 | `edgeResolutionDeg=2` | `plannerResolutionDeg=2` | 一致 |
| 状态碰撞顺序 | 先环境碰撞，再非结构性自碰撞；均优先 `Exhaustive='on'`，失败后 fallback | `collisionFreeJointState` 使用同样的环境→自碰撞顺序和参数 | 碰撞谓词基本一致 |
| 结构性自碰撞排除 | 原始脚本内置相邻 body 对和 WeldingGunMount 附着对 | scene loader/`deriveStructuralSelfCollisionSkips` 提供同一张排除表 | 当前场景表已核对为同构；应在正式复现前打印并逐项比较 |
| 搜索停止条件 | 双树一旦 `CONNECT` 到达就停止并返回候选路径 | 连接后先经过 core 的 `considerCandidate`/内部 `validatePath`，通过后返回 | 当前多了一层 core 内部路径复核；不应与原始“连接即停”混称 |
| 后验 dense validation | `localValidatePath`：先逐节点检查，再用 `validationResolutionDeg=0.5` 检查所有边 | wrapper 对路径边调用 `collisionFreeJointEdge`，默认 `validationResolutionDeg=0.5`；端点和限位另查，但没有独立的逐节点循环 | 0.5°名义分辨率一致；统计方式和节点检查实现不完全相同 |
| `0.5°` 的真实来源 | 原始规划脚本的 `validationResolutionDeg=0.5`；不是动画参数 | 当前 wrapper 默认也写成 `0.5` | “0.5°只来自动画”的说法被源代码纠正；应改为“原始代码有0.5°后验验证，旧文档未说明” |
| 动画插值 | 动画脚本单独用 `stepDeg` 生成播放帧；焊接段还做更细 FK/弧长采样 | 不影响当前 wrapper 的 planner 成功 | 与搜索/验证规则分离 |
| 状态缓存 | 原始本地实现每次调用都直接增加 `collisionChecks` 并调用 `checkCollision` | current core 使用 state cache 和 edge cache；缓存命中不再调用 `checkCollision` | 这是实际计算时间和 collision count 的实质差异；当前 40-run 不能作为原始开销基线 |
| 碰撞计数含义 | `collisionChecks` 基本等于每次状态检查调用数 | `collisionConfigurationChecks` 只统计实际未命中缓存的状态检查；另有 `edgeCollisionChecks` | 两者不能直接横向比较 |
| Timeout | 原始脚本没有 wall-clock timeout，只有 8000 次迭代 | wrapper 默认 30 s；core 在多个循环/状态检查点检查 timeout | 30 s 是防卡死保护，不是原始算法规则；当前超时样本不能解释为原始算法在同规则下失败 |
| 搜索引擎 | `localRrtConnect/localExtend/localConnect` | `RRTConnectJoint → jointPlannerCore/runConnect` | 不是逐行同一实现；需先修复/验证后再称为原始基线复现 |
| 当前 40-run 的成功判定 | 原始：`approach.pass = approach.success && approach.validation.pass` | 当前：`plannerSuccess && validationPass && ~timedOut` | 高层逻辑相近，但 current core 的内部验证、缓存和 timeout 使实际判定链不同 |

## 需要特别修正的概念

### 1. 不能说原始系统没有 0.5°后验验证

从原始源码可直接确认：

```matlab
"edgeResolutionDeg",2, ...
"validationResolutionDeg",0.5, ...
```

并且：

```matlab
approach.validation = localValidatePath(scene,approach.qPath,options);
exitPath.validation = localValidatePath(scene,exitPath.qPath,options);
```

`localValidatePath` 又把 `options.validationResolutionDeg` 传给边插值。因此：

- `2°` 是原始 RRT 搜索时的边检查分辨率；
- `0.5°` 是原始路径生成后的独立后验复核分辨率；
- `0.5°` 不是动画播放的 `StepDeg`；
- 原始代码没有“2 s 内通过0.125°认证”的规则。

### 2. 当前 40-run 为什么仍不能直接作为正式比较

原因不是单纯的 `0.5°`，而是当前恢复 wrapper 还改变了：

- RRT-Connect 的实际实现；
- 缓存策略；
- 内部路径复核流程；
- wall-clock 终止条件；
- 碰撞检查计数的定义。

所以当前结果的正确归档名称是：

> `diagnostic / restored-pilot attempt under current shared core`

不能写成：

> `original RRT-Connect ground-truth benchmark`

## 原始基线重现状态

已用原始脚本重新运行，输出另存为：

- `/home/ttbstt/jixiebi/RRT/results/phase12a_seam_legacy_rrtconnect_repro.mat`
- `/home/ttbstt/jixiebi/RRT/results/phase12a_seam_legacy_rrtconnect_repro_report.txt`

本次结果：

```text
PreWeld  = 30 mm
PostWeld = 30 mm
Overall PASS = 1

Approach: 12.219553 s, 31 iterations, 32 nodes,
          317 collision checks, 60 edge checks,
          path length 19.842372 rad,
          0.5 deg validation: 21 nodes / 20 edges, PASS

Exit:     6.786776 s, 11 iterations, 22 nodes,
          181 collision checks, 30 edge checks,
          path length 17.255930 rad,
          0.5 deg validation: 19 nodes / 18 edges, PASS
```

与交接文档记录的 `11.898240 s / 6.752958 s` 接近，路径长度、迭代数、节点数、碰撞检查数和边检查数一致，说明原始脚本和场景基线仍可复现。

## 后续动作边界

在这份审计完成前，不启动新的 40-run、160-run 或 100-seed benchmark。

后续正确顺序：

1. 以原始 `plan_weld_approach_exit_rrt_connect.m` 为 ground-truth implementation；
2. 明确决定是：
   - 让四算法都接入同一套原始碰撞接口，但保留各自搜索结构；还是
   - 先把原始 RRT-Connect 封装成共享接口，并用同一随机数/同一碰撞计数规则复现历史输出；
3. 对齐后只比较 `qHome→qPreWeld` 和 `qPostWeld→qHome` 的规划时间、成功率、路径长度；
4. 原始 `0.5°` 后验验证可以保留，但必须在实验定义中明确写成“独立后验验证”，不能再称作动画分辨率，也不能升级成0.125°认证门槛；
5. 当前 40-run 结果保留为诊断材料，不覆盖原始基线结果。

