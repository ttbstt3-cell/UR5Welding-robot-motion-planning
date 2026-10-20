# UR5内置轨迹规划能力与现有系统对比报告

调查日期：2026-10-15，Asia/Shanghai。现场只读快照：12:34:41 +08:00。

范围：UR5 CB3 / PolyScope 3.15.4、ROS2 Humble、当前 Python HMI、MATLAB CBi-RRT* + ACCP，以及实际安装的驱动和控制器。仅新增本报告及证据文件；未修改业务代码、控制器参数、标定、碰撞模型或轨迹，未启动或切换运动控制器，未发送任何运动指令。

## 1. 结论先行

1. **【官方文档确认】UR 已有运动指令内部的加减速、部分运动段之间的 blend，以及实时伺服跟随能力。**这不等于它会替上位机完成任意焊接任务的构型连续搜索、焊缝精度约束和全场景避障。[CB3 3.15.4 手册](https://www.universal-robots.com/download/manuals-cb-series/script/script-manual-cb-series-sw3154/)、[CB3 blend 说明](https://www.universal-robots.com/articles/ur/robot-care-maintenance/protective-stop-cb3-service-note/)
2. **【项目源码确认】本项目轨迹执行走 `FollowJointTrajectory`，不是把 MATLAB 路径转换为 `movej/movel/movep`。**配置的目标是 scaled 控制器；正常执行设计为 ROS 侧插值、驱动发送周期关节位置、机器人执行 `servoj`。因此当前没有调用 UR 的 MoveJ/MoveL/MoveP 原生运动规划来替代后处理。
3. **【项目源码确认（含只读运行态）】调查时没有激活的轨迹控制器。**scaled、普通 JTC、passthrough 均为 `inactive`；UR 硬件组件为 `active`。不能把“硬件连接”和“轨迹控制器激活”混为一谈，也不能声称本次已验证轨迹在真机运行。
4. **【官方文档确认＋项目源码确认】当前 JTC 的插值不是自动时间优化。**P 采用线性，P+V 采用三次，P+V+A 采用五次；它使用上层给定的节点、导数和时间，并不自动优化焊接速度、jerk、TCP 轮廓或碰撞。[Humble 轨迹插值](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)
5. **【推断／技术建议】不应删除“轨迹连续规划”，应保留它作为“完整任务执行准备与控制器感知验证”。**可以不在 HMI 自己实时重做伺服插值；但仍应保留三段边界衔接、时间和节点速度设计、TCP 精度、解析动力学指标、碰撞及版本检查。
6. **【推断／推荐方案】选择方案 C：MATLAB 负责焊接工艺、构型、时间和安全约束，ROS2/UR 负责已验证轨迹的低层插值与执行。**当前先沿用 scaled 链路；passthrough 可作为后续单独验证的执行后端，不是自动平滑优化器，也不是本轮建议直接切换的配置。

## 2. 证据口径与版本边界

本文标记含义：

- **官方文档确认**：UR 官方匹配版本手册，或官方 ROS/UR 源码、文档直接支持。
- **项目源码确认**：本机项目、安装文件、包版本，以及只读 ROS 服务实际返回；运行态另明确注明时间。
- **推断**：由已确认实现和数学性质得出的工程判断，不冒充厂商保证或实测。
- **待验证**：需要后续 URSim、运行记录或经批准的真机测试，本轮没有验证。

### 2.1 本机版本，而非最新网页版本

**【项目源码确认】包管理器返回：**

| 项目 | 实际安装版本 |
|---|---|
| `ur_robot_driver` | 2.14.0-1jammy.20260908.194018 |
| `ur_controllers` | 2.14.0-1jammy.20260908.143119 |
| `ur_client_library` | 2.15.0-1jammy.20260905.032610 |
| `joint_trajectory_controller` | 2.54.0-1jammy.20260908.124006 |
| `controller_manager` | 2.54.2-1jammy.20260908.115549 |
| `control_msgs` | 4.9.0-1jammy.20260907.212955 |

**【项目源码确认（运行态）】**只读 `/dashboard_client/get_polyscope_version` 返回 **3.15.4.106291**；`/ur_configuration_controller/get_robot_software_version` 亦返回 3.15.4。不是仅照抄用户输入。硬件类为 `ur_robot_driver/URPositionHardwareInterface`，非 MockSystem。

证据：[现场快照](/home/ttbstt/jixiebi/UR5/技术调查/20261015_UR内置轨迹能力/runtime_readonly_snapshot.json:1)、[版本及调查证据](/home/ttbstt/jixiebi/UR5/技术调查/20261015_UR内置轨迹能力/investigation_evidence.json:1)。对 UR 驱动、UR controllers、UR client library 和 JTC 安装文件执行包完整性只读检查，未返回文件被修改的条目；这不是对控制器闭源固件的审计。

**【官方文档确认】**取得的是 UR 官网链接的 `scriptManual_3.15.4.pdf`，不是 SW5/SW10 手册。下载内容 872,555 bytes，SHA256：`a9000a6bedcdf87e90ac33560312e40ad09ab57559c645e287fa7a862feea410`。只在内存提取文字，未执行其中程序。

**【待验证／范围限制】**未读取闭源 UR 固件内部规划器实现；不能由文档未承诺某种连续性，就断言其物理运动必然不平滑。反过来，也不能把“运行看起来平滑”升级为对任意路径的 jerk、碰撞或 0.5 mm 焊接精度保证。

## 3. UR 原生功能能做什么

### 3.1 MoveJ、MoveL、MoveP 与 blend

下表基本指令行为均为**【官方文档确认】**，依据匹配的 [CB3 3.15.4 Script Manual](https://s3-eu-west-1.amazonaws.com/ur-support-site/124999/scriptManual_3.15.4.pdf) 对应函数章节；仅概述，不复制手册。

| 指令 | 轨迹与速度参数 | 对本项目的含义 |
|---|---|---|
| `movej` | 关节空间点到点；`v`、`a` 是主导关节速度和加速度；`t` 指定运动时间，优先于 v/a；`r` 为 blend 半径 | 内置点到点时间规划，但 TCP 不因此走直线 |
| `movel` | 工具空间直线；v 为 m/s，a 为 m/s²；支持 t、r | 有笛卡尔直线运动能力，不等于复现指定 ACCP 关节分支 |
| `movep` | 过程运动，直线配合工具空间圆弧 blend，趋于恒定工具速度；参数 a/v/r | 适用于过程走刀；不是任意关节轨迹的通用优化入口 |

**【官方文档确认】**在支持的相邻运动组合和几何条件下，非零 blend 可避免在每个点停车；blend 会改变过点附近的实际路径。过大的、相互重叠的 blend 可能跳过路点，而非严格经过所有输入点。[UR CB3 官方说明](https://www.universal-robots.com/articles/ur/robot-care-maintenance/protective-stop-cb3-service-note/)

**【推断】**可以让 UR 规划某些相邻动作的速度衔接，但不能据此把离散、已避障的 CBi-RRT*/ACCP 路径直接换为任意半径 blend：圆角可能切入工件、底面或机器人本体；焊缝轮廓、姿态及 IK 分支也可能变化。任何此类后端切换都要对新插值后的完整路径重新验证。

**【官方文档确认】**UR 官方还明确指出 CB3 与 e-Series 的 MoveP blend 行为存在版本差异。[CB3→e-Series 迁移说明](https://www.universal-robots.com/articles/ur/interface-communication/cb3-to-e-series-programs-transferring-quick-guide/)。因此本报告不把新版 MoveP/OptiMove 的行为当作 CB3 3.15.4 的能力。

### 3.2 ServoJ、SpeedJ 不是上层路径优化器

**【官方文档确认】**`servoj` 是在线关节位置伺服。CB3 手册默认周期为 0.008 s；gain 控制跟随响应，lookahead 用于前瞻和抑制过冲。该版手册明确 a/v 不是当前版本使用的运动规划参数。高 gain、短 lookahead、低频或噪声目标可能引起振荡。[3.15.4 手册](https://www.universal-robots.com/download/manuals-cb-series/script/script-manual-cb-series-sw3154/)、[UR ServoJ 专题](https://www.universal-robots.com/articles/ur/programming/servoj-command/)

**【官方文档确认】**`speedj` 接收关节速度目标，以指定主导轴加速度改变速度；t 控制调用返回时间。它不是接收整条 Cartesian 焊缝后自动求解姿态、碰撞和时间约束的接口。[匹配版本手册](https://www.universal-robots.com/download/manuals-cb-series/script/script-manual-cb-series-sw3154/)

**【推断】**两者都具有低层跟随/加减速作用，但不是“输入任意 q 点便自动得到全局最优且 jerk 连续的安全焊接轨迹”。不能靠 ServoJ 的 lookahead 修复上层速度尖峰或错误边界；滤波对误差和相位的影响还需实测。

### 3.3 能否保证 v/a/jerk 与焊接精度

| 问题 | 调查结论 | 证据等级 |
|---|---|---|
| 是否自带自动加减速 | 原生 Move 指令有；SpeedJ 有速度变化过程；ServoJ 是伺服目标跟随，不是同一种规划接口 | 官方文档确认 |
| 是否能非停顿地通过相邻动作 | 满足运动组合和 blend 条件时可以，不能泛化为任意三条 action 自动衔接 | 官方文档确认＋推断 |
| 是否保证任意输入的关节加速度、jerk 连续 | 未找到匹配 CB3 手册对任意输入路径的这类保证；不能把“平滑”解释为数学 C2/C3 连续 | 待验证，不能宣称保证 |
| 是否保证 TCP 相对 CAD 焊缝误差 <0.5 mm、姿态 <0.5° | 不能从这些接口推出；还取决于标定、轨迹、负载、跟踪、速度缩放等 | 推断；物理精度待验证 |
| 是否知道新 STL、焊丝、真实工件、地面全部碰撞几何 | 本机发送接口未把这些规划场景几何交给 UR 固件 | 项目源码确认；因此不能由 UR 代替场景避障为推断 |

**【推断】**机器人保护停机、关节/安全平面限制，不等于针对本项目完整工具和工件模型的预测性无碰撞规划；不能把“撞了会保护停”当作离线路径碰撞验证的替代。

## 4. 当前项目真正执行链路

### 4.1 从 HMI 到机器人：逐段证据

| 环节 | 实际行为与证据 | 结论标记 |
|---|---|---|
| HMI 目标 action | [hmi_config.yaml](/home/ttbstt/jixiebi/UR5/ur5_hmi/config/hmi_config.yaml:3)：`/scaled_joint_trajectory_controller/follow_joint_trajectory` | 项目源码确认 |
| 执行按钮 | [main_window.py](/home/ttbstt/jixiebi/UR5/ur5_hmi/gui/main_window.py:3345)：通过最终检查、人工确认后，对加载轨迹调用消息转换；没有 MoveJ/MoveL 转换 | 项目源码确认 |
| 工作线程 | [main_window.py](/home/ttbstt/jixiebi/UR5/ur5_hmi/gui/main_window.py:208)：再次检查状态后交给 node.send | 项目源码确认 |
| ROS 包装 | [ros_node.py](/home/ttbstt/jixiebi/UR5/ur5_hmi/ros/ros_node.py:217)：直接调用 trajectory.send | 项目源码确认 |
| Action 类型 | [trajectory_client.py](/home/ttbstt/jixiebi/UR5/ur5_hmi/ros/trajectory_client.py:69)：`FollowJointTrajectory`；[发送处](/home/ttbstt/jixiebi/UR5/ur5_hmi/ros/trajectory_client.py:152) 将整份 trajectory 放入 goal | 项目源码确认 |
| JTC 命令接口 | [ur_controllers.yaml](/opt/ros/humble/share/ur_robot_driver/config/ur_controllers.yaml:109)：position 命令、position/velocity 状态；现场查询一致 | 项目源码确认（含运行态） |
| scaled 插值 | [官方 2.14.0 scaled 源码](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.14.0/ur_controllers/src/scaled_joint_trajectory_controller.cpp)：推进缩放后的轨迹时间，调用 Trajectory.sample，再输出关节 position | 官方文档确认（官方源码） |
| 驱动写入 | [官方 2.14.0 hardware_interface.cpp](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.14.0/ur_robot_driver/src/hardware_interface.cpp) 的 write()：position 模式选择 `MODE_SERVOJ`，不是 movej | 官方文档确认（官方源码） |
| 机器人侧跟随 | 本机 [external_control.urscript](/opt/ros/humble/share/ur_client_library/resources/external_control.urscript:220)：servoThread 调用 servoj；不是在这一分支重新做全路径 MoveJ 规划 | 项目源码确认 |

**【项目源码确认＋官方文档确认（官方源码）】**预期的正常执行链路是：MATLAB 节点/时间/导数 → HMI JointTrajectory → FollowJointTrajectory → scaled JTC 的 ROS 样条采样 → UR 驱动周期关节位置 → 机器人侧 ServoJ → 内部伺服。应区分“ROS 轨迹插值”“URScript 跟随程序”“UR 固件闭环伺服”三个层次。

### 4.2 哪个控制器实际激活

**【项目源码确认（现场只读运行态）】**12:34:41 的 `/controller_manager/list_controllers` 返回：

| 控制器 / 硬件 | 状态 |
|---|---|
| scaled_joint_trajectory_controller | inactive |
| joint_trajectory_controller | inactive |
| passthrough_trajectory_controller | inactive |
| forward_position / forward_velocity | inactive |
| joint_state、速度缩放、TCP 等 broadcaster | active |
| UR5 硬件组件 URPositionHardwareInterface | active |

**【项目源码确认】**这与本项目 [start_ur5_real.sh](/home/ttbstt/jixiebi/UR5/start_ur5_real.sh:60) 显式 `activate_joint_controller:=false` 相容。安装 launch [控制器装载逻辑](/opt/ros/humble/share/ur_robot_driver/launch/ur_control.launch.py:406) 把运动控制器先列为 inactive，是否激活由启动参数决定；[默认控制器](/opt/ros/humble/share/ur_robot_driver/launch/ur_control.launch.py:581) 是 scaled。本轮没有为了调查而激活它。

**【待验证】**历史上每次执行究竟激活了哪个控制器、发送了哪个轨迹文件、实际运行多久，需要对应时刻的记录；当前快照不能反推历史运行状态。

### 4.3 MATLAB 的速度、加速度与时间戳是否传到 ROS

**【项目源码确认】**[trajectory_utils.py](/home/ttbstt/jixiebi/UR5/ur5_hmi/trajectory/trajectory_utils.py:134) 保留载入的 q、t、v、a；[消息转换](/home/ttbstt/jixiebi/UR5/ur5_hmi/trajectory/trajectory_utils.py:208) 分别复制到 `positions`、`velocities`、`accelerations`、`time_from_start`。没有时间戳会报错，不会凭空补一套自动时间规划。

**【项目源码确认】**第二阶段 [prepare_hmi_continuous_request.m](/home/ttbstt/jixiebi/UR5/matlab仿真/prepare_hmi_continuous_request.m:113) 刻意只导出 **P+V+t**；不把加速度作为命令字段导出，避免把已验证的 cubic 变成 quintic。加速度仍然用于验证，并不是“不检查加速度”。

**【项目源码确认／离线测试】**使用现有载入归一化与消息转换函数、3 个合成节点，P/PV/PVA 三种格式均通过：字段分别保持为 6/0/0、6/6/0、6/6/6；0、0.123456789、2.5 s 原样转换到秒/纳秒；没有创建 ROS 节点、action 或 publisher。测试只证明数据路径，不证明已在机器人上执行。

**【官方文档确认（官方源码）＋推断】**硬件只接收 position，**不等于消息中的 velocity 被丢弃**：velocity 先决定 ROS 样条形状，形状采样后才输出 position。若换了导数字段，就必须换用对应的离线插值模型重新验证。

**【项目源码确认】**本机运行参数：`interpolation_method=splines`、`open_loop_control=false`、`update_rate=125`、`allow_nonzero_velocity_at_trajectory_end=true`；硬件描述中的 ServoJ gain=2000、lookahead=0.03 s。仅记录值，未调整。

### 4.4 时间戳与速度缩放不能混淆

**【官方文档确认（官方源码）】**scaled 控制器依据机器人速度缩放推进轨迹的有效时间；不是忽略 time_from_start。参见 [scaled 2.14.0 源码](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.14.0/ur_controllers/src/scaled_joint_trajectory_controller.cpp) 的 `traj_time_` 更新和 sample 调用，以及 [UR 控制器说明](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_controllers/doc/index.html)。

**【推断】**速度缩放小于 1 时，墙钟完成时间变长。因此需要区分沿有效轨迹时间的同步误差，和按真实墙钟计算的工艺进度。动态缩放还会影响实际加减速，不能仅凭离线常速缩放模型保证实机峰值。

**【项目源码确认／需注意】**HMI 的百分比滑块在 [_on_speed_changed](/home/ttbstt/jixiebi/UR5/ur5_hmi/gui/main_window.py:2331) 只更新显示；它被[点动入口](/home/ttbstt/jixiebi/UR5/ur5_hmi/gui/main_window.py:2360) 用于点动时长计算。整条轨迹的 [_execute_path](/home/ttbstt/jixiebi/UR5/ur5_hmi/gui/main_window.py:3355) 原样使用加载轨迹，并不按该滑块重定时，也没有在该分支写机器人速度滑块。因此确认框中的“Command Speed 5%”不能当作整条轨迹又限速到 5% 的证据。本轮没有修复此界面差异。

## 5. JTC、scaled JTC 与导数字段的准确含义

### 5.1 当前插值规则

**【官方文档确认＋项目源码确认】**以下适用于本机已查询为 `splines` 的 JTC/scaled JTC。依据 [Humble 官方说明](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)、[2.54.0 trajectory.cpp](https://github.com/ros-controls/ros2_controllers/blob/2.54.0/joint_trajectory_controller/src/trajectory.cpp)，以及本机 [trajectory.hpp](/opt/ros/humble/include/joint_trajectory_controller/joint_trajectory_controller/trajectory.hpp:105)。

| 相邻端点字段 | 使用的插值 | 数学连续性及局限 |
|---|---|---|
| positions | 分段线性关节插值 | 位置 C0；节点处通常速度跳变，不自动获得平滑速度 |
| positions + velocities | 分段 cubic Hermite | 共享节点速度时 C1；加速度可跳变，jerk 不保证连续 |
| positions + velocities + accelerations | 分段 quintic | 共享节点导数时 C2；不自动保证 jerk 连续或峰值满足上限 |
| 两端字段不一致 | 按可用的共同信息降阶 | 不应在同一正式轨迹随意混用不同字段组合 |

表中连续性为**【官方文档确认】**；由此判断“仍需峰值和几何验证”为**【推断】**。本机 `allow_integration_in_goal_trajectories` 的安装默认值为 false，本文不把仅速度、仅加速度的积分路径作为当前标准执行方式。

**【推断】**三次插值的段内 jerk 有限，不代表节点加速度跳变已经消失；分段线性在段内计算得到加速度 0，也不代表节点过渡的真实峰值为 0。不能用段平均加速度替代峰值验收。

**【官方文档确认】**Humble 文档展示了不一致导数字段可能产生过冲，并没有声称样条会自动检查全部应用约束。[官方轨迹示例](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)

**【项目源码确认】**控制器 YAML 的 joint trajectory/goal 容差为 0.2/0.1 rad（[配置](/opt/ros/humble/share/ur_robot_driver/config/ur_controllers.yaml:126)）；它们是关节跟随容差，不是焊缝 TCP 的 0.5 mm 误差门，也不是本项目规划关节速度/加速度限制。不能把 action 成功直接解释为焊接精度合格。

### 5.2 哪些是重复计算，哪些不是

| 当前计算 | 是否与控制器重复 | 调查判断 |
|---|---|---|
| 上层给出 q、t、节点 v | 否：这是样条边界条件的设计，不是控制器替我们生成的最优解 | 项目源码确认＋推断 |
| MATLAB 离线重建 cubic 做 FK、误差和碰撞验证 | 公式与控制器相似，但用途是验收，并非第二次实时发令 | 项目源码确认 |
| HMI 根据时间播放 3D 动画 | 仅预览显示，不构成又一次机器人速度控制 | 项目源码确认 |
| ROS 周期样条 + UR ServoJ | 两级不同职责；一个采样目标，一个伺服跟随；不是两套独立全局轨迹优化器 | 官方文档确认（官方源码）＋推断 |
| 单焊接段 Hybrid 后，再做全任务边界速度优化 | 有重复求解的可能，但全任务新边界是新增问题；不能因为控制器有插值就删除 | 项目源码确认＋推断 |

**【项目源码确认】**当前第二阶段在 [prepare_hmi_continuous_request.m](/home/ttbstt/jixiebi/UR5/matlab仿真/prepare_hmi_continuous_request.m:36) 对完全相同 q/TCP/time 尝试复用 D，否则重新调用 Hybrid；随后做完整任务处理与验证。这是未来可按版本缓存精简的计算，不是当前控制器已代做的工作。

**【项目源码确认／限制】**HMI 通用 [trajectory_validator.py](/home/ttbstt/jixiebi/UR5/ur5_hmi/trajectory/trajectory_validator.py:69) 主要使用相邻节点差分作估计；正式第二阶段才在 [prepare_hmi_continuous_request.m](/home/ttbstt/jixiebi/UR5/matlab仿真/prepare_hmi_continuous_request.m:82) 做密集控制器等价验证与解析极值检查。前者不能替代后者。

## 6. Passthrough 与 motion primitive：本机是否能用

### 6.1 Passthrough 已安装，但不是“自动优化”

**【项目源码确认】**本机确有 `ur_controllers/PassthroughTrajectoryController` 插件：[controller_plugins.xml](/opt/ros/humble/share/ur_controllers/controller_plugins.xml:32)；YAML [参数区](/opt/ros/humble/share/ur_robot_driver/config/ur_controllers.yaml:138) 和现场服务都确认已加载，但为 inactive。说“当前 Humble 没有 passthrough”是不准确的。

**【官方文档确认】**该控制器仍接受 FollowJointTrajectory，把轨迹点交给机器人侧执行，降低 PC 周期流式发送的实时压力；不因此承担上层焊缝优化或场景避障。[官方 passthrough 说明](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_controllers/doc/index.html)

**【项目源码确认】**安装的 [external_control.urscript](/opt/ros/humble/share/ur_client_library/resources/external_control.urscript:321) 实现 cubicSplineRun；[jointSplineStep](/opt/ros/humble/share/ur_client_library/resources/external_control.urscript:477) 按样条计算关节速度并调用 speedj；[分派](/opt/ros/humble/share/ur_client_library/resources/external_control.urscript:667) 区分 cubic/quintic。这不是自动把 FJT 换成 MoveP，也不是“UR 黑盒自动重新求最优速度”。

**【官方文档确认（官方源码）／重要差异】**2.14.0 的 [hardware_interface.cpp](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.14.0/ur_robot_driver/src/hardware_interface.cpp) 在 `check_passthrough_trajectory_controller()` 中，把缺少速度和加速度的点送为**零端点速度的 spline**；有 P+V 时送 cubic，有 P+V+A 时送 quintic，只有 A 没 V 的组合会报错。不能假设 positions-only 切到 passthrough 后仍等于当前 JTC 的线性插值。此分支也证明必须审源码，不能只根据控制器名字或概览文档认定等价。

**【推断／兼容性初判】**当前 CB3、已装 UR 驱动和脚本具备尝试标准 passthrough 的软件基础；没有证据支持笼统称其“仅 e-Series 可用”。**【待验证】**仍未在本机激活并运行该后端；URCap/脚本组合、起止速度、速度缩放、轨迹量化、取消与跟踪误差需先在匹配的 URSim 3.15.4 上验证。安装成功和接口存在不等于已验收可用于真实焊接。

**【项目源码确认＋推断】**本机脚本还包含对末段非零速度的减速处理（[源码](/opt/ros/humble/share/ur_client_library/resources/external_control.urscript:420)）；因此切换后端可能改变时间行为，已有 cubic 离线报告不能直接当作 passthrough 实机证书。UR 文档所述 goal 容差功能也不能替代本项目全程碰撞检查。

### 6.2 新 motion primitive controller 不是现有功能

**【官方文档确认】**Rolling 文档的 `motion_primitive_forward_controller` 可把 MoveJ/MoveL/MoveC 类型运动序列交给原生运动规划，接口为 `ExecuteMotionPrimitiveSequence`，不是目前 HMI 使用的 FJT。[Rolling 原生运动接口](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_robot_driver/ur_robot_driver/doc/usage/position_velocity_control.html)

**【项目源码确认】**本机没有安装 `ros-humble-motion-primitives-controllers`，当前 UR controllers 插件清单、YAML、launch 和 HMI 都没有接入该控制器。因此“新文档存在”不能解释为“本机已经可以直接启用”。低层 UR client library 有运动原语能力，也不等于 ROS Humble 已有对应 action 插件。

**【项目源码确认】**本机脚本对 OptiMoveJ/OptiMoveL 有明确版本条件：PolyScope 5.21+ 或 X 10.8+，见 [external_control.urscript](/opt/ros/humble/share/ur_client_library/resources/external_control.urscript:693)。CB3 3.15.4 不满足。**【推断】**不能把这些新优化功能作为当前删除 MATLAB 后处理的依据。

**【待验证】**移植新 motion primitive 后端到当前 Humble/CB3 所需的依赖、API、驱动能力与兼容性没有完成验收。本轮不安装、不升级、不切换。即使未来可移植，blend 后的路径仍需要重新进行 TCP 和碰撞验证。

## 7. 三种方案对比

下表为基于已确认实现的**【推断／工程评估】**，不是新的真机实验结果。

| 维度 | A：MATLAB 自主连续性规划 + ROS2 | B：MATLAB 仅 q 路径，现有控制器插值 | C：MATLAB 工艺/精度/安全 + ROS2/UR 低层执行 |
|---|---|---|---|
| 速度连续性 | 可优化并验证完整任务的节点速度和边界 | 无时间戳时当前接口不接受；即使补 t，positions-only 仍有节点速度跳变 | 上层设计边界，底层按既定导数插值，职责清晰 |
| 加速度、jerk | 可做解析极值、节点跳变与 jerk 优化；仍非物理实测保证 | 现有 JTC 不自动生成满足应用限制的最优导数 | 保留必要上层约束和验证；不重写实时伺服 |
| TCP 焊缝精度 | 可用已标定 FK 验证轮廓/同步/姿态 | 关节插值不等于 TCP 插值；没有自动 0.5 mm 保证 | 工艺验收留在 MATLAB，真实跟踪另验 |
| 路径偏离、碰撞 | 验证最终样条，而非只查节点 | 仅节点无碰撞不足；下层插值/跟随可能改变中间状态 | 同一模型、版本、最终插值检查；变化就失效重验 |
| 实时性 | 优化可离线，但 scaled 流式执行仍需要 PC 周期稳定 | 省去离线优化，不消除流式发送实时要求 | 现阶段 scaled；后续可验证 passthrough 减少 PC 实时压力 |
| 当前兼容性 | 已有代码链路；完整任务仍须全部通过 | 仅 q 不能直接发送；不是“一键自动平滑” | 最小改动且与现有链路一致；本轮不改变任何配置 |
| 论文价值 | 需证明控制器感知与受约束优化价值，而非仅重复样条公式 | 有价值的基线，不适合未经验证就作为最终焊接方案 | 最适合研究：构型连续、恢复、几何精度、时序与控制器协同 |

**【推断】**若 A 指“上层输出 q/t/v 并离线验证”，它实际上已经是推荐 C 的重要部分；若 A 指“在 HMI 再写一套实时机器人伺服/滤波”，这部分没有必要与官方驱动重复。B 也可以改成“上层给定恒速 Cartesian MoveP 序列”，但那已不是现有控制器插值方案，而是另一条需单独验证的执行后端。

## 8. 最适合当前焊接研究的技术方案

### 8.1 保留的功能

以下均为**【推断／技术建议】**，本轮没有实施：

1. CBi-RRT* 保持负责全局接近/退出避障；ACCP 保持负责焊缝构型连续、局部桥接与笛卡尔精度。
2. 保留完整任务的 PreWeld/PostWeld 边界检查：关节分支、位置、姿态、节点速度、两侧加速度。非零入口/出口速度必须和相邻段统一设计。
3. 保留时间分配和 Hybrid 节点速度优化。优化结果是提供给样条的边界条件，不是和样条插值重复。
4. 保留与**实际消息字段及控制器版本匹配**的验证：轮廓误差、同步误差、姿态、解析 v/a 极值、段内 jerk 与节点加速度跳变、全部几何碰撞和版本绑定。
5. 保留执行前实际起点、控制器状态、新鲜反馈、人工确认和物理安全检查；Preview Ready、离线 Execution Ready 和真实焊接验收不是一个状态。

**【项目源码确认】**目前两阶段入口分别为 [规划预览](/home/ttbstt/jixiebi/UR5/ur5_hmi/gui/planning_workflow.py:131) 与 [轨迹连续规划](/home/ttbstt/jixiebi/UR5/ur5_hmi/gui/planning_workflow.py:165)。后者本来就不仅是“平滑”，还包含边界、验证和版本检查，不能因控制器有样条就整体删除。

### 8.2 可以精简的部分

以下为**【推断／技术建议】**：

- 将“轨迹连续规划”的概念明确为“执行轨迹准备与验证”，避免让用户误以为 UR 完全不会加减速。
- 一个任务生成一份统一时间轴、统一导数的 action，三段仅用 segment label 区分。不要寄望三条独立 action 自动 blend；本机允许非零末速度，只代表消息可接受，不证明下一段能连续接入。
- 对完全相同 q/TCP/time/场景/控制器版本复用已通过的优化结果，但不能复用过期安全结论。
- 保留离线控制器等价采样作为验收，不必再把加密样条样本当作第二套独立优化路径送出；以实际需要和误差验收决定节点密度。
- 后续若实时抖动成为主要瓶颈，先在 URSim 单独评估本机已有 passthrough。它可以迁移插值位置，不能替代工艺约束、时间设计和最终安全验收。

### 8.3 关于论文创新的边界

**【推断／研究建议】**普通 cubic/quintic 插值、UR 自带 blend 不宜作为新的算法创新。可以研究并通过消融实验论证：受限空间下连续构型与局部桥接、笛卡尔精度保持、控制器感知的边界/速度/jerk 优化，以及统一几何和版本后的完整任务验证。建议比较 P-only、常规 PV、Hybrid PV，并区分轮廓误差与时间同步误差。

**【待验证】**本报告不作学术新颖性认证，也不把既有单焊接段 D 的离线 PASS 当作完整任务或真机焊接已通过。需要另行验证接近/焊接/退出的全程连续性和实际跟踪。

## 9. 尚未完成的验证与建议顺序

以下全部为**【待验证】**，不属于本轮已执行工作：

1. 匹配 URSim 3.15.4：相同 q/t/v 在 scaled 与 passthrough 下的 P/PV/PVA、起止速度、时间量化、速度缩放、暂停/取消差异。
2. 验证实际发出的 goal 字段、控制器状态、期望/实际关节轨迹和 TCP 反馈日志一致，而不只看文件中有 velocities。
3. 记录真实墙钟和有效缩放时间，评估焊接速度受示教器缩放/暂停影响时的工艺处理。
4. 在独立授权和物理安全验收后，才做实机跟踪误差、负载、工具挠曲、安装及标定不确定性验证。仿真采样无碰撞不是连续安全的数学证明。
5. 另行评估本报告发现的 HMI“5%”显示与轨迹实际限速语义不一致，不在只读调查中修改。

## 10. 调查方式、来源与只读审计

**【项目源码确认】**本轮执行了：包版本/完整性检查，文件与行号读取，ROS ListControllers/ListHardwareComponents/GetParameters/GetPolyScopeVersion/GetRobotSoftwareVersion 只读查询，以及不创建 ROS 节点的离线消息转换测试。没有调用 SwitchController、set_parameters、任何 motion action、脚本发送或 TCP 写入接口。

**【项目源码确认】**仅新增本目录报告、`runtime_readonly_snapshot.json`、`investigation_evidence.json`。后者保存关键受保护文件 SHA256；结束时再次核对 **11/11 均未改变**，用于确认报告整理没有改变被调查的执行链路。

主要一手来源与适用边界：

| 来源 | 用途 / 版本边界 |
|---|---|
| [UR CB3 SW3.15.4 手册页面](https://www.universal-robots.com/download/manuals-cb-series/script/script-manual-cb-series-sw3154/)及[原始 PDF](https://s3-eu-west-1.amazonaws.com/ur-support-site/124999/scriptManual_3.15.4.pdf) | 本机固件匹配的 Move/Servo/Speed 定义 |
| [UR ServoJ 说明](https://www.universal-robots.com/articles/ur/programming/servoj-command/) | CB3 8 ms、gain/lookahead 的作用和风险 |
| [UR CB3 保护停机与 blend 说明](https://www.universal-robots.com/articles/ur/robot-care-maintenance/protective-stop-cb3-service-note/) | blend 的连续过点用途及重叠风险，不当作避障规划证明 |
| [CB3/e-Series 迁移说明](https://www.universal-robots.com/articles/ur/interface-communication/cb3-to-e-series-programs-transferring-quick-guide/) | 防止混用不同代际 MoveP 行为 |
| [Humble JTC 轨迹说明](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)及[2.54.0 实现](https://github.com/ros-controls/ros2_controllers/blob/2.54.0/joint_trajectory_controller/src/trajectory.cpp) | 与本机 JTC 版本和参数交叉核对，不使用 Rolling 新插值特性作当前证据 |
| [UR 驱动 2.14.0 scaled 控制器](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.14.0/ur_controllers/src/scaled_joint_trajectory_controller.cpp) | 时间缩放、sample、position 输出；本次读取内容 SHA256 为 b93ae342cb3d4fcf3391b70e4cfb7da72fe95f9434f8ac4861de967cf6ff2e56 |
| [UR 驱动 2.14.0 硬件接口](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.14.0/ur_robot_driver/src/hardware_interface.cpp)及[passthrough 控制器](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.14.0/ur_controllers/src/passthrough_trajectory_controller.cpp) | 流式与转发分支、导数字段、时间处理；结合本机已安装脚本核对 |
| [UR 控制器概览](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_controllers/doc/index.html) | 功能说明；文档会更新，具体行为以已装版本源码为准 |
| [Rolling position/velocity control](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_robot_driver/ur_robot_driver/doc/usage/position_velocity_control.html) | 只用于识别新 motion primitive 功能，不作为本机已具备该插件的证据 |

方法说明：采用事实核查流程，以版本匹配的一手资料、安装文件和只读运行态互证。报告由 AI 辅助整理；“推断”是工程评估，不是独立认证。未派生独立评审人，未把项目代码上传给第三方分析服务。

**最终建议【推断】：保留“轨迹连续规划”的约束优化和完整任务验证价值，交给 ROS2/UR 去做它们已经擅长的实时插值、跟随与伺服。不要用“UR 能平滑”替代“这条焊接轨迹已满足精度和碰撞要求”。**
