# 轨迹接口圈数修复与时间参数化独立诊断

2026-10-15。全部离线，NO ROBOT MOTION。

## 结论

**接近→焊接接口已合法衔接，原18个接口工件碰撞采样消失。完整修复预览通过碰撞及节点限位检查，但时间参数化仍未通过，executionReady=false。**

- 真实 TCP、URDF、焊枪、焊丝、工件、地面、碰撞过滤和阈值未改。
- ACCP 的47个 q 节点、TCP路径及时间戳保留；未重算IK、未修改ACCP或CBi-RRT*算法。
- 不把周期等价解释成可以跳过实际旋转。若选用原始数值路径中的大角度旋转，先对该真实长边逐点验碰，再加密用于预览。
- 修复预览359点：接近165、焊接47、退出147。219.147993s是**预览时钟**，不是已通过验收的执行时间。
- 当前保留的接近段有约329.109496°的单条原始实际J6长边；全部实际插值已经验证。不是最短路径，也不是消除了所有绕转。不能把它藏在取模或“零姿态差”中。

## 1. 根因与6轴数值

源数据：`fixed_base_floor_contact_20261015/cold_full_preview_v2`，未覆盖。

| 轴 | 修复前接近末端 ° | ACCP焊接首点 ° | 原接口差 ° | 修复后接近末端 ° |
|---|---:|---:|---:|---:|
| J1 | -95.025144 | -95.025144 | 0 | -95.025144 |
| J2 | -66.308856 | -66.308856 | 0 | -66.308856 |
| J3 | 78.786650 | 78.786650 | 0 | 78.786650 |
| J4 | -85.563579 | -85.563579 | 0 | -85.563579 |
| J5 | -51.103956 | -51.103956 | 0 | -51.103956 |
| J6 | **371.227115** | **11.227115** | **-360** | **11.227115** |

旧接口TCP位置/姿态相同，但数值关节轨迹存在一整圈差。`plan_hmi_preview_request/freePath` 原先无条件调用 `lift_continuous_joint_path`，该辅助函数不校验全路径真实限位或最终目标的数值圈数。结果接近段末4个点超过J6上限；动画按数值插值，在狭窄工件附近旋转一圈，产生18个碰撞采样。

还发现旧退出段最终J6为-169.198133°，而当前RealHome为+190.801867°，同样相差-360°。本轮同时固定返回目标数值，不改Home。

当前场景限位（rad转换为deg）：J1/J2/J4/J5/J6为±360°，J3为±180°；验证还与已加载URDF关节限位取交集。没有扩限、钳位或简单取模。

## 2. 为什么不能直接整段±2π平移

- 焊接段J6范围[-6.802349°, 11.227115°]。整段+360°后最大371.227115°越限；整段-360°后最小-366.802349°越限。焊接段不能整体换圈。
- 把旧lift后的接近段整体-360°，起点会从真实保存的190.801867°变成-169.198133°，不再是请求起点；整体+360°则越限。
- 只改接近末点为11.227°、其他点不变，会在接近末段制造实际长旋转；不允许仅凭端点FK相同就接受。
- 同样不能把退出段整体+360°后直接接到焊接末点，这会把完整一圈转移到另一个接口。

因此采用**起终点数值钉定＋真实边插值验证**，不强制最短角度：周期lift只是候选；若越限或改变端点就拒绝。原始数值路径作为另一候选，只有对其实际长旋转也完整验碰通过才可保留。若候选均失败，有限次重调用原CBi-RRT*，目标始终是当前ACCP有效分支。

## 3. 修复实现与重新规划

生产入口已接入：

- [plan_hmi_preview_request.m](/home/ttbstt/jixiebi/UR5/matlab仿真/plan_hmi_preview_request.m:235)：不再无条件lift。
- [hmi_plan_interface_free_path.m](/home/ttbstt/jixiebi/UR5/matlab仿真/hmi_plan_interface_free_path.m:1)：最多3次原规划器调用，保留每次seed和失败证据。参数和硬约束沿用原值，只有独立重试seed变化，不改搜索核心。优先检查没有额外长绕转的有效分支；所有候选均需实际碰撞验证。
- [hmi_checked_free_path_branch.m](/home/ttbstt/jixiebi/UR5/matlab仿真/hmi_checked_free_path_branch.m:1)：固定数值start/goal、检查限位、逐条实际边验证。加密成不大于3°的自由空间预览间隔之前，已经检查原始长边；不是拆开后只查端点。
- [hmi_audit_joint_interface_path.m](/home/ttbstt/jixiebi/UR5/matlab仿真/hmi_audit_joint_interface_path.m:1)：每条原始边至少101点，并保证实际角度采样不粗于0.5°；绝不wrap插值。

离线恢复实验中，接近段seed20260918/19/20的候选未通过实际边检查，涉及Gun↔wall_x0/wall_x1，被明确拒绝。退出段也拒绝了存在实际工件碰撞的候选，没有把RRT返回success直接当成整个任务PASS。

保留的是经完整数值旋转验证的合法路径；接近原始数值J6最大边329.109496°，1973次状态采样全部通过。其J6累计实际运动约478.644240°，退出段累计约425.188596°。这些是明确记录的真实绕转，不是最优路径结论。

另外独立执行了**HMI正式新入口**的CBi-RRT*调用，得到同样的165点接近路径，与本次修复预览逐值差为0。证据：[workflow_entry_replan](/home/ttbstt/jixiebi/UR5/规划文件/workflow_runs/interface_branch_repair_20261015/workflow_entry_replan/validated_approach.mat)。这不是只改离线文件而未接入后续规划。

## 4. 几何验证结果

| 项目 | 修复后结果 |
|---|---|
| 接近末点→焊接首点6轴数值差 | 全0 |
| 该接口TCP位置差 | 0mm |
| 该接口姿态差 | 数值舍入约0.0000017° |
| 焊接末点→退出首点6轴差 | 全0 |
| 返回终点→保存RealHome数值差 | 仅浮点舍入，J6约2.5e-14° |
| 接口相邻3条实际边加密 | 303次，0碰撞，0限位失败 |
| 自由空间原始边 | 每条≥101，长边采样≤0.5°；含实际大角度旋转 |
| 完整预览节点/边采样 | **7877次，0碰撞** |
| 自碰撞、Robot-Gun、Robot-Wire | PASS，原skip表不变 |
| 工件、地面、枪/丝环境碰撞 | PASS，原固定底座接触策略不变 |
| 全部输出节点关节限位 | PASS |
| 输出最大相邻关节变化 | 7.118992°，来自保留的焊接段，不再有360°接口差 |
| 原47个焊接q节点及TCP路径 | 逐值完全不变 |
| 最小采样正间距 | 约0.231724mm |

**0.231724mm不是独立实物安全余量认证。**3mm焊丝碰撞缩短保留原近似，未修改。离散采样PASS不等于连续路径的数学安全证明。没有将这些结果标成真机可执行。

场景快照与当前依赖文件在验证前后均一致：[summary.json](/home/ttbstt/jixiebi/UR5/规划文件/workflow_runs/interface_branch_repair_20261015/summary.json)。全部碰撞对、首次失败姿态和每次候选在各attempt MAT、[interface_diagnosis.json](/home/ttbstt/jixiebi/UR5/规划文件/workflow_runs/interface_branch_repair_20261015/replanned/interface_diagnosis.json)及[逐帧审计](/home/ttbstt/jixiebi/UR5/规划文件/workflow_runs/interface_branch_repair_20261015/replanned/preview_display_audit.json)。原18个失败采样记录仍保存在旧目录。

## 5. 接口通过后，时间失败的独立根因

已实际调用现有第二阶段，进入 `VELOCITY_OPTIMIZATION`，不是仍卡在接口。

当前冷规划焊缝虽然也是47点，但q/TCP/time不等于历史D快照，不能仅凭点数相同就复用旧速度。当前焊接时长为25.135842s，保持时间戳和q不变，现有速度模块结果：

| 速度策略 | 轮廓mm | 同步mm | 姿态° | 最大速度rad/s | 解析最大加速度rad/s² | 碰撞 | 总验收 |
|---|---:|---:|---:|---:|---:|---|---|
| 原前段平均速度规则 | 0.406375 | 2.055985 | 0.738561 | 0.091121156 | 0.848252593 | PASS | FAIL |
| A 中心差分 | 0.277386 | 3.289951 | 0.679224 | 0.083423386 | 0.174027119 | PASS | FAIL |
| B 五次参考导出速度，再按cubic验收 | 0.147303 | 4.139510 | 1.002891 | 0.107046991 | 0.274170004 | PASS | FAIL |
| C 现有controller-aware节点速度 | **0.182934** | **0.568271** | **0.548976** | **0.084158503** | **0.261796382** | PASS | **FAIL** |

阈值不变：同步/轮廓<0.5mm，姿态<0.5°，速度≤0.087266463rad/s，加速度≤0.261799rad/s²。峰值按解析cubic极值检查，不拿分段平均加速度代替。原报告0.091117459/0.835528804是密集采样值；本表基线0.091121156/0.848252593是更严格的解析峰值，差异并非修改约束或场景。

**独立原因：当前47点与固定时间戳下，现有C节点速度的同步和姿态约束仍不满足。**Hybrid函数要求C是已通过的初始解；因此它不能继续生成D。以前会得到泛化的`C is not a valid initial snapshot`断言，现在包装层保存完整输入诊断并返回：

`PREPARE:TimingInput — sync=0.568271mm; orientation=0.548976deg`

没有修改速度算法或时间参数，没有为了得到PASS而调用旧D速度、调整阈值。单焊缝各策略每段101点，共4601点；**完整任务≥5000点cubic控制器验收尚未进入**。全程7877点是几何预览线性审计，不能冒充完整时间轨迹验收。

证据：[时间报告](/home/ttbstt/jixiebi/UR5/规划文件/workflow_runs/interface_branch_repair_20261015/timing/timing_summary.json)、[完整日志](/home/ttbstt/jixiebi/UR5/规划文件/workflow_runs/interface_branch_repair_20261015/timing.log)、`timing/velocity_profile_input_diagnostic.mat`。没有生成`trajectory_execute_ready.json/.mat`。

## 6. 回归与状态

- 合法小步数值路径通过；周期等价但起点数值不符拒绝；越限等价角拒绝。
- 故意恢复的原360°接口按实际长边721点检查，检出限位和碰撞，不被取模为0。
- 旧越限预览在速度优化前返回`PREPARE:InterfaceLimits`，不会送给Cartesian Bridge尝试“补一圈”。纯整数圈接口差也有单独`PREPARE:PeriodicBranch`错误。
- HMI现有MAT载入、节点校验、状态机回归PASS：359点可加载；`Preview Ready → Preparation Failed`后预览保留，执行路径为空，`executionReady=false`。未启动GUI机器人工作线程、未创建ROS节点或action。
- MATLAB静态检查无语法错误，仅格式/性能及多余suppression提示。
- 16份受保护文件SHA复验全部不变，包含算法核心、碰撞后端、TCP/URDF/几何/固定接触策略、HMI ROS发送与配置、控制器YAML。

证据：[回归](/home/ttbstt/jixiebi/UR5/规划文件/workflow_runs/interface_branch_repair_20261015/regression/regression.json)、[HMI状态回归](/home/ttbstt/jixiebi/UR5/规划文件/workflow_runs/interface_branch_repair_20261015/hmi_state_regression.json)、`protected_before.sha256`。

## 7. 修改范围与结果使用

生产修改仅2份：

1. `plan_hmi_preview_request.m`：自由空间输出接入数值圈数/实际路径验证适配器。
2. `prepare_hmi_continuous_request.m`：旧接口错误前置拒绝；保存速度输入诊断并精确报告既有算法失败，不改变优化数学。

新增7个接口/离线诊断辅助文件：`hmi_audit_joint_interface_path.m`、`hmi_checked_free_path_branch.m`、`hmi_plan_interface_free_path.m`、`diagnose_hmi_interface_branch_repair.m`、`diagnose_hmi_interface_timing.m`、`test_hmi_interface_branch_repair.m`、`summarize_hmi_interface_repair.m`。交接文档同步记录。

可在现有HMI加载本轮[修复预览](/home/ttbstt/jixiebi/UR5/规划文件/workflow_runs/interface_branch_repair_20261015/replanned/full_raw_trajectory.mat)，仅用于观察。下一次规划会使用新适配器并独立检查新场景，不会自动信任本轮旧快照。

未覆盖正式anchor、冻结轨迹或原诊断数据。后续若优化固定时间下的速度精度，或减少自由空间大幅绕转，应另行处理；本轮不越过接口修复与时间失败诊断范围。
