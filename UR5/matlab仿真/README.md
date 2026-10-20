# UR5 MATLAB 仿真脚本

本目录集中存放当前使用的 MATLAB 脚本和函数。场景文件、标定文件、JSON 和
CoppeliaSim/Python 文件仍保留在上一级 `UR5` 目录，不改变 ROS2 控制链路。

## 文件

- `sync_ur5_joint_states.m`：加载 MATLAB UR5、冻结标定、焊枪凸包和工件碰撞代理，生成 `scene` 并显示当前场景。
- `test_ur5_ros2_control.m`：通过现有 `/joint_trajectory` 接口发送测试轨迹，同时接收 `/joint_states`。
- `build_weld_seam.m`：根据 Workpiece 局部坐标生成直线焊缝。
- `make_actual_gun_axis_pose.m`：根据实际焊丝轴线构造每个焊点的目标姿态。
- `generate_actual_axis_ik_candidates.m`：为实际焊丝轴线目标生成并筛选 IK 候选。
- `replan_weld_actual_axis.m`：基于实际焊丝轴线完成 Point 1–80 的连续重规划，并检查节点、边、焊枪和焊丝碰撞。
- `animate_weld_actual_axis_replan.m`：播放已保存的 Point 1–80 规划结果并生成多视角视频。
- `publish_weld_trajectory.m`：将已验收的轨迹发布到现有 `/joint_trajectory` 接口。
- `validate_weld_path.m`：检查机器人与环境碰撞及关节跳变。
- `inspect_workpiece_geometry.m`：接收 `scene` 并输出 Workpiece 及其碰撞代理的坐标、尺寸和姿态。
- `audit_current_weld_geometry.m`：只读核对当前场景几何和焊缝定义。
- `diagnose_weld_collisions.m`：通用碰撞诊断工具。

## 运行

在 MATLAB 中可从任意当前工作目录执行：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m')
```

ROS2 控制测试（脚本会持续运行，停止时使用 Ctrl+C）：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/test_ur5_ros2_control.m')
```

`sync_ur5_joint_states.m` 会自动从其所在目录定位上一级的：

```text
../scene_geometry.json
../ur5_tool_calibration.mat
```

工件检查函数的使用方式：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m')
wpSummary = inspect_workpiece_geometry(scene);
```

场景几何更新仍在上一级执行：

```bash
cd /home/ttbstt/jixiebi/UR5
PYTHONPATH=/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src \
python3 export_scene_geometry.py
```

当前实际焊丝轴线重规划：

```matlab
opts = struct('animate', false, 'publish', false, ...
    'outputFile', 'weld_actual_axis_replan.mat');
result = replan_weld_actual_axis(opts);
```

当前已验收结果保存在：

```text
weld_actual_axis_replan.mat
```

动画播放与视频输出示例：

```matlab
animate_weld_actual_axis_replan( ...
    '/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan.mat', ...
    StepDeg=1, Speed=100, SaveVideo=true, ...
    VideoFile='weld_actual_axis_replan_weld_inside.avi', ...
    View='allWeld');
```

旧 TCP+Z、多 IK、局部 corridor 和旧 RRT/IK 框架脚本及其 `.mat` 数据已清理；
临时 `smoke`、`pre5` 结果和旧场景同步归档已移入系统回收站，正式 benchmark、当前验收结果和历史记录仍保留在本目录及 `AGENT_HANDOFF.md`。

### 时间参数化 TCP 诊断（只读）

`verify_time_parameterized_trajectory.m` 用于定位时间参数化后 TCP 误差的来源，
不修改 ACCP、TCP、碰撞模型、时间戳或 ROS2 执行链路。它分别报告：

1. `DIRECT_REFINED_QPATH`：对 refined/exported `qPath` 每个航点直接做 FK；
2. `TIME_PARAMETERIZED_TRAJECTORY`：按导出的 `timeFromStart` 和 `velocities` 重建
   控制器等价的 cubic Hermite 采样（没有 velocities 时使用线性关节插值）。

两者都与同长度的 `tcpPosePath`/`tcpTargetExecution`（位置线性插值、姿态四元数
SLERP）逐采样比较，输出最大/RMS TCP 位置误差、最大姿态误差及对应时间点。完整样本
同时写入 CSV/MAT，便于后续论文审计；没有期望姿态时会明确报告姿态指标不可用。

示例：

```matlab
report = verify_time_parameterized_trajectory( ...
    'trajectory_with_time.mat', expectedResult, ...
    'time_parameterized_tcp_diagnostic.json');
```

离线回归脚本 `test_time_parameterized_trajectory_diagnostic.m` 使用保存的 ACCP 结果
复现当前基线：直接 refined qPath 最大误差约 `0.000028 mm`，时间参数化轨迹最大
误差约 `2.331394 mm`（`10.696707 s`），RMS 约 `0.376481 mm`，最大姿态误差
约 `2.518817 deg`（`8.394341 s`）。这只是诊断结果，不代表已优化或允许真机执行。
### ACCP 时间参数化离线验证

`compare_accp_time_parameterizations.m` 对 ACCP 的31点 refined 路径进行
3001点以上密集采样，分别报告焊缝轮廓误差、时间同步误差、姿态误差、速度/加速度、
碰撞和限位。`validate_accp_timed_output.m` 是独立的显式门控；未配置有限时间同步
门或未通过任一检查时，结果保持不可执行。该模块不修改 ACCP 搜索、TCP、碰撞模型
或 ROS2 执行链路。

`compare_accp_time_parameterizations.m` 还提供
`adaptive_time_scaled_positions_only`：按每段 Δq、TCP步长、姿态变化和曲率特征
分配节点间隔，并优化相邻段速度。当前结果总时长21.246303 s，密集节点加速度
估计5.358140 rad/s²，连续段加速度估计0.053581 rad/s²；TCP轮廓0.361549 mm、
时间同步0.423961 mm、碰撞PASS。该候选仍是离线诊断，不会自动写入或发送 ROS2
trajectory。

### ACCP 内部三阶段输出

正式 ACCP 核心 `replan_weld_actual_axis_adaptive_core.m` 已将时间阶段内置为：

```text
Continuous IK -> Cartesian Refinement -> Adaptive Time Scaling
```

Cartesian refinement 生成的 31 个关节节点保持不变，Adaptive Time Scaling 只分配
节点时间，最终返回 `qPath`、`tcpPath/tcpPosePath`、`timeFromStart`、`velocities` 和
`accelerations`。通过门要求 TCP 轮廓与同时间 TCP 同步误差均小于 0.5 mm、碰撞/限位
通过、速度/加速度满足限制、总时长不超过 50 s。返回的速度/加速度用于离线验证和
记录，现有 ROS2 trajectory 导出和控制器插值接口不变，不会自动发送运动。
`maxAccelerationRadPerSec2` 采用返回分段速度斜率的 `piecewise_linear_segment_slope`
指标；节点折点的有限差分峰值只作为诊断字段保留。

用 `test_accp_adaptive_time_scaling_integration.m` 可对既有 31 点结果做离线回归；
历史 positions-only 候选结果为 21.246303 s、最大分段速度 0.070576 rad/s、最大分段
加速度 0.053581 rad/s²、TCP轮廓 0.361549 mm、时间同步 0.423961 mm、碰撞 PASS。
这只是分段线性候选的历史指标；最新 ACCP 还必须通过控制器等价插值复核，不能据此
单独设置 `executionReady=true`。

### ACCP Cartesian Bridge 回退（2026-10-13，离线）

`replan_weld_actual_axis_adaptive_core.m` 已将 Point19→Point20 的局部笛卡尔恢复作为
可配置失败回退：只有相邻层返回 `NO_CONNECTED_CANDIDATE` 时才尝试；25/50/75%目标
使用位置线性插值和姿态四元数 SLERP，新增节点重新调用现有 IK，所有节点和边沿用
硬方向/限位/步长/碰撞约束。桥接边至少101个关节空间样本；最大深度2、IK预算3000、
时间预算60s，成功后将中间状态插回搜索图并继续后续焊点。`wire_collision_shorten_mm=3`
仍是碰撞近似参数，必须另做物理安全验证。

同一真实场景冷运行结果保存在：
`UR5/规划文件/real_anchor_runs/accp_bridge_comparison_20261013/`。
原始21点 ACCP 在Point20失败；桥接后扩展为24层，第20点恢复，选中3个中间节点、配置链
24/24、最终可达状态3，节点/边碰撞PASS。选中路径最小正间隙0.579816mm；所有失败尝试
最近近邻0.007115mm仅作为诊断，不是成功路径余量。汇总见 `comparison_report.json`，
所有结果均 `executionReady=false`。

最终时间轨迹不再只用分段平均加速度判断。新增
`validate_accp_controller_interpolation.m`，按已核对的 ROS2
`joint_trajectory_controller` positions+velocities cubic Hermite 规则进行至少101点/边
密集 FK、姿态、限位、速度、加速度和碰撞验证。当前47点 refined 输出的几何TCP轮廓
最大0.076950mm、碰撞PASS，但控制器等价时间同步最大2.885364mm、姿态0.694330°、
峰值加速度0.323359rad/s²，未通过时间阶段门，因此整条结果仍不可执行，不得送入真机。

### Controller-aware 时间优化（2026-10-13，独立离线模块）

`optimize_accp_controller_timing.m` 接收 qPath、tcpPosePath、timeFromStart 和场景，
仅优化段时长；q 节点、TCP 目标、标定和几何不变。速度沿用现有
`[0;diff(q)./diff(t)]` 规则重算；桥接邻域及高姿态变化段优先优化。
三次 Hermite 速度极值检查包含段内驻点，加速度检查包含每个节点左右两侧峰值，
最终对 4601 个样本执行精确 FK、限位和完整碰撞复核，不以平均加速度替代峰值。

`test_accp_controller_timing_optimization.m` 复用已通过构型/边验证的47点快照。
第二轮候选结果：18.606468→17.099156 s；TCP轮廓0.187600→0.073211 mm；
时间同步2.885364→0.863661 mm；姿态0.694330→0.453679°；峰值速度
0.074037→0.087119 rad/s（限值0.087266）；解析峰值加速度
0.328283→0.261798 rad/s²（限值0.261799）；碰撞/限位PASS。
解析基线峰值高于旧采样报告0.323359，因为同时检查节点左右极限。

当前速度规则使第一段 `q(u)=q0+(u-(u^3-2u^2+u))*Δq` 与段时长无关，
所以第一段同步误差至少0.863661 mm，纯时间优化无法满足0.5 mm同步要求。
候选为 `numericalAcceptancePass=false`、`executionReady=false`，不生成正式真实轨迹。
若要消除此下界，需另行授权调整速度生成规则；本模块未做该修改。

报告、完整MAT候选、MATLAB日志保存在
`UR5/规划文件/real_anchor_runs/controller_aware_timing_20261013_v2/`。
该候选仅对应 positions+velocities 三次插值，不导出 accelerations；本机控制器
positions+velocities+accelerations 对应五次插值，不能混用上述验收结果。
最小采样正间隙0.583667 mm不代表物理安全余量或连续碰撞证明，焊丝缩短3 mm仍需
物理安全验证。未改ACCP、CBi-RRT*、ROS2或执行链路，无机器人运动。

### Adaptive Velocity Profile Generation（2026-10-13，独立离线）

`generate_accp_adaptive_velocity_profile.m` 仅改变节点速度，复用 Controller-aware
时间优化候选的47个 q 节点、TCP目标和17.099156 s时间戳；没有时间再缩放、IK或几何变更。
可用 `test_accp_adaptive_velocity_profile.m` 重现实验：

* A：非均匀时间网格的加权中心差分，端点采用邻接段斜率。
* B：端点静止的最小积分jerk、C2五次多项式曲线，提取其节点速度。注意控制器并不
  执行该五次曲线；取出的速度仍通过三次Hermite验证，不发送加速度字段。
* C：允许非零入口/出口速度，使用固定TCP路径FK/Jacobian目标优化节点速度。
  加速度线性端点约束，加上速度采样约束和解析段内速度极值补充约束。
  全部方案最后都用精确FK及4601点完整碰撞验证，解析检查所有段速度/加速度峰值。

| 方案 | 同步误差 mm | 轮廓误差 mm | 姿态 ° | 最大速度 rad/s | 最大加速度 rad/s² | 数值验收 |
|---|---:|---:|---:|---:|---:|---|
| 原前段平均速度 | 0.863661 | 0.073211 | 0.453679 | 0.087119263 | 0.261798476 | FAIL |
| A 中心差分 | 0.543578 | 0.045823 | 0.302694 | 0.087112452 | 0.177026986 | FAIL |
| B 五次导出速度 | 1.229321 | 0.033386 | 0.306707 | 0.086337104 | 0.175337720 | FAIL |
| C 非零入口优化 | 0.238287 | 0.013041 | 0.212377 | 0.087265027 | 0.261796382 | PASS |

四者时长均17.099156 s，碰撞/限位均PASS。C同步RMS0.078519 mm、轮廓RMS0.002150 mm，
最小采样正间隙0.628018 mm。第一次C候选曾在网格间速度极值略超限，保留失败报告，
通过新增解析极值约束后才获得上述PASS，不放宽速度限值。

最终 `report.json`、`comparison.csv`、四组速度曲线CSV、六关节曲线PNG和完整MAT保存于
`UR5/规划文件/real_anchor_runs/adaptive_velocity_profile_20261013_final/`。
输出始终 `executionReady=false`，PASS仅指本段离线数值验收。非零入口和出口必须另做
接近/离开段及实际控制器首尾状态衔接验证，不可从静止直接发送本段候选。采样碰撞通过
不是连续安全证明，小间隙及焊丝缩短3 mm仍需独立物理验证。未接入ROS2导出/执行链路。

### Hybrid Adaptive Velocity Optimization（2026-10-14，独立离线对比）

新增 `optimize_accp_hybrid_velocity.m`。固定47个 q 节点、TCP目标及17.099156 s节点
时间，仅求节点速度。A中心差分和B五次最小jerk参考构成平滑参考代价，C是可行初始解；
D使用带约束二次优化，不是将A/B/C简单加权平均。代价包括相对A/B参考速度、解析分段
jerk积分、节点加速度跳变、连续加速度积分及节点速度曲率。优先把峰值加速度约束到
0.9倍软目标；最后仍按原硬限制及精确FK验证。所有候选用positions+velocities控制器
等价三次Hermite，4601个采样点重新检查TCP、姿态、碰撞和限位。

| 策略 | TCP轮廓最大 mm | 同步最大 mm | 姿态最大 ° | 速度最大 rad/s | 加速度最大 / RMS rad/s² | jerk最大 / RMS rad/s³ | 节点加速度跳变最大 rad/s² | 碰撞/限位 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| A 中心差分 | 0.045823 | 0.543578 | 0.302694 | 0.087112 | 0.177027 / 0.026212 | 2.208399 / 0.295479 | 0.094015 | PASS/PASS |
| B 五次参考 | 0.033386 | 1.229321 | 0.306707 | 0.086337 | 0.175338 / 0.027088 | 1.164653 / 0.189855 | 0.143568 | PASS/PASS |
| C 精度优先 | 0.013041 | 0.238287 | 0.212377 | 0.087265 | 0.261796 / 0.044657 | 3.517053 / 0.612044 | 0.418127 | PASS/PASS |
| D Hybrid | 0.109436 | 0.355067 | 0.420968 | 0.087265 | 0.148927 / 0.029294 | 0.717414 / 0.119425 | 0.180123 | PASS/PASS |

各策略总时长不变，均为17.099156 s。D的同步/轮廓/姿态精度均在0.5 mm、0.5 mm、
0.5°硬门内。D相比C的加速度峰值下降43.1%、峰值jerk下降79.6%、RMS jerk下降80.5%、
节点加速度跳变峰值下降56.9%，且通过0.9倍加速度软目标。本结果选为更优离线候选；
它的TCP误差高于C，但仍通过全部指定数值验收。节点加速度仍有0.180123 rad/s²跳变，
故不声称加速度连续。cubic段内jerk为常量；节点跳变单独计算，未将其伪报为有限jerk峰值。

入口/出口衔接为 `BLOCKED_NO_CURRENT_BOUND_APPROACH_EXIT`。旧接近规划缺少当前真实
场景绑定，最近真实接近规划为 `PLAN_FAILED_REAL_SCENE_OR_STALE_GOAL`，没有当前绑定
退出轨迹。D入口/出口速度不是零；接近/焊缝/离开段的位置、速度、加速度接口连续性
尚未验证。虽然D段本身数值通过，整体仍不可执行。控制器源码默认 `splines`，本机
scaled controller YAML 使用 position command interface；positions+velocities采用三次
样条。运行时参数、scaled time与真实UR内部伺服行为未读取/验证，因此这只是ROS2层
插值等价离线结果。碰撞检查是密集采样，不是连续安全证明；最小正间隙0.569161 mm、
焊丝缩短3 mm仍须物理安全验证。

结果目录 `UR5/规划文件/real_anchor_runs/hybrid_velocity_20261014_final/` 包括JSON/MAT、
汇总CSV、全六轴速度/加速度/分段jerk CSV与图、节点加速度跳变CSV与图、J3/J6在
12–15 s的放大对比图和MATLAB日志。D仅为离线比较候选，`executionReady=false`；没有
覆盖C、生成正式轨迹或发送运动。
