# 上位机调速、真机同步显示及2倍速度离线验证

日期：2026-10-19。结论：调速/显示实现及隔离回归PASS；当前固定轨迹2倍速度离线FAIL。没有机器人运动、真实调速调用、控制器切换或执行许可导出。

## 1. 当前限制与实现

只读检查时真实倍率反馈为100%，不能再通过UR示教器倍率接口提速至200%。本机包为UR controllers 2.14.0、joint_trajectory_controller 2.54.0、ur_msgs 2.6.0。

- 轨迹执行页新增目标1%～100%及“应用真机倍率”，调用已有`/io_and_status_controller/set_speed_slider`服务，不新增驱动或修改配置。启动、拖动不会发送请求；运行中应用必须明确确认。
- 目标与实际倍率分开显示。实际值来自`/speed_scaling_state_broadcaster/speed_scaling`，包含UR控制器的实际限制；服务ACK不代替反馈。过期/失败显示UNKNOWN，超时保留未决请求，不自动重试或允许旧/新请求乱序。
- 发送轨迹前和所有运动忙状态退出预演；机器人、焊枪及焊丝使用既有真实关节/FK显示链，10Hz刷新，不用预演计时代表真机状态。
- action反馈只读解码，比较同一反馈中的desired/actual；不再用未计入速度缩放的墙钟线性插值报告跟踪误差。JTC 2.54反馈的`desired.time_from_start`是下一节点时间，并非精确缩放时钟，因此只显示段进度下界和倍率估计剩余时间。
- 原始JointTrajectory字段完整保留。取消请求不是终止结果，取消待确认/状态未知时继续保留实际反馈显示，不假报已停止。
- 没有验证动态倍率变化的实际加减速或伺服精度，也没有宣称软件检查等于物理安全验收。

代码入口：

| 文件 | 作用 |
| --- | --- |
| `UR5/ur5_hmi/gui/execution_live_panel.py` | 调速界面、人工确认、目标/反馈区分、实时显示状态 |
| `UR5/ur5_hmi/gui/main_window.py` | worker请求、禁止预演覆盖、真实反馈日志/进度、取消待返回状态 |
| `UR5/ur5_hmi/ros/speed_slider.py` | 非阻塞服务客户端，范围/新鲜度/状态检查、超时与迟到响应 |
| `UR5/ur5_hmi/ros/ros_node.py` | 服务可用性、倍率时间戳、action反馈回调连接 |
| `UR5/ur5_hmi/ros/state_monitor.py` | 独立倍率时间戳/服务状态快照 |
| `UR5/ur5_hmi/ros/trajectory_client.py` | 同goal只读反馈，旧goal和无效反馈忽略；不改发送字段 |
| `UR5/ur5_hmi/trajectory/execution_feedback.py` | 标准关节顺序/有限值与反馈时间语义 |
| `UR5/matlab仿真/diagnose_hmi_uniform_speed_scaling.m` | 独立离线2倍时间缩放和完整审计，输出诊断JSON而非轨迹 |
| `UR5/ur5_hmi/tools/test_execution_speed_live.py` | 假服务/假action及真实Qt/VTK回归，无真实ROS节点 |
| `UR5/ur5_hmi/docs/continuous_planning_and_execution.md` | 操作及限制 |

CBi-RRT*、ACCP/IK、TCP、机器人/焊枪/焊丝/工件/环境几何、碰撞阈值和过滤、控制器YAML以及trajectory_utils.py未修改。相关旧生产文件保存在`before/`。

## 2. 2倍候选输入与完整结果

来源：`UR5/规划文件/当前轨迹/连续规划完成轨迹_2026-10-19_12-14-01_560570.mat`，471个节点（焊接段47点）；配套同名report，场景与所有标定/模型依赖从绑定快照核对，RealHome单独匹配。原始轨迹SHA256：`7bab695b30f6069dbc0408a37423c3e1eb01f57fbc6271f6bea3bb41c488ce0c`。

保持q和TCP目标不变：`t2=t/2; V2=2*V`。P+V cubic Hermite段内解析极值与每段101点密集FK/碰撞审计均执行，未因速度超限提前跳过碰撞。

| 指标 | 原结果 | 2倍候选 | 判定 |
| --- | ---: | ---: | --- |
| 时长 | 294.880033s | 147.440017s | 2倍时间目标 |
| 最大关节速度 | 0.084673742rad/s | 0.169347484rad/s | FAIL，上限0.087266463（5°/s） |
| 最大关节加速度 | 0.261718677rad/s² | 1.046874709rad/s² | FAIL，上限0.261799 |
| TCP轮廓最大/RMS | 0.291503/0.031893mm | 同左 | PASS，最大≤0.5mm |
| TCP同步最大/RMS | 0.462008/0.059983mm | 同左 | PASS，最大≤0.5mm |
| 最大姿态误差 | 0.414448° | 同左 | PASS，≤0.5° |
| 最大段内jerk | 3.583708rad/s³ | 28.669662rad/s³ | 统计，不声称节点加速度连续 |
| RMS段内jerk | 0.187197rad/s³ | 1.497574rad/s³ | 统计 |
| 最大节点加速度跳变 | 0.439865rad/s² | 1.759458rad/s² | 统计，与段内jerk区分 |
| 碰撞采样/关节限位 | 47,001点PASS | 47,001点PASS | 通过不等于连续安全数学证明 |
| 最小正间隙 | 0.459441mm | 同左 | 小间隙，模型/标定不确定性及现场余量未认证 |
| 离线完整验收 | PASS | FAIL | 速度、加速度超限 |
| executionReady | false | false | 均不声称物理/伺服已验收 |

2倍候选最大同步误差时间81.632259s；最大关节速度出现在J6、143.523391s；最大加速度出现在J5、73.763586s。完整位姿、关节状态、误差位置及绑定见[诊断JSON](speed_2x_offline_rerun/speed_2x_validation.json)。审计耗时276.394510s。

等比例缩放满足`h2*V2=h*V`，因此理想Hermite的q(u)不变，几何/碰撞指标与原结果相同。真机动态响应并未验证，不能推断真机误差也不变。

### 不是只换一种时间分配就能达到2倍

保留每个数值关节节点和5°/s限速时，每段至少需要`max(abs(Δq))/vLimit`，所有段相加为**225.857235s**。这是必要下界，不是可行优化结果，也未计加速度、精度及碰撞限制。因此147.440017s不可能通过单纯时间重分配实现。

J6累计绝对行程约1076.31°（范围367.47°），对总时长影响明显；未取模、压缩整圈或修改其实际运动路径。若仍追求2倍，后续需单独评估新的路径/运动约束方案及实际能力，不能直接扩大倍率或放宽现有限制。

初版诊断因raw预览附带request-only Home/PreWeld键而比较整个binding失败，未产生候选；修正独立诊断入口为核对优化结果的每个依赖键及独立Home版本后重新运行。没有放宽场景版本检查或改动正式规划器。

## 3. 回归与范围

- [调速/反馈/VTK测试](vtk_tests/speed_live_tests.json)：假服务端点1/100%、无启动/拖动写入、非法值/重叠/故障/过期拒绝、超时及迟到响应、ACK不伪造反馈PASS；假action中原始轨迹所有ROS字段不变，旧goal/无效反馈拒绝。
- 同一测试用实际VTK，5组不同q反馈核对`T_world_gun=FK(q)_flange*T_flange_gun`及焊丝末端更新；预演停止且不能覆盖运动画面，反馈UNKNOWN不按墙钟伪造，取消请求不是终止PASS。没有ROS节点、真机服务或真实action调用。
- [执行软件门回归](execution_regression/simple_execution_tests.json)：26个反例全部拒绝；100/125/150%布局PASS；原轨迹、生产YAML不变，发送0次。
- [响应性回归](gui_responsiveness/responsiveness_tests.json)：生产VTK、50Hz合成状态、真实QProcess回放既有MATLAB结果PASS。按钮返回0.043s，计算中最大心跳间隔0.049s，取消0.262s；完成图表加载仍有约2.597s短停顿，未宣称完全异步或重新优化成功。
- Python源语法检查PASS。ROS CDR序列化含对齐填充字节，假action测试比较所有ROS消息字段，不以非语义填充字节作数据一致性依据。

## 4. 官方源码核对

- [UR 2.14 GPIO控制器](https://raw.githubusercontent.com/UniversalRobots/Universal_Robots_ROS2_Driver/2.14.0/ur_controllers/src/gpio_controller.cpp)：`set_speed_slider`接口及0.01～1.0范围，非200%接口。
- [UR 2.14倍率广播](https://raw.githubusercontent.com/UniversalRobots/Universal_Robots_ROS2_Driver/2.14.0/ur_controllers/src/speed_scaling_state_broadcaster.cpp)：发布百分数；[scaled JTC](https://raw.githubusercontent.com/UniversalRobots/Universal_Robots_ROS2_Driver/2.14.0/ur_controllers/src/scaled_joint_trajectory_controller.cpp)：按实际scaling推进轨迹时钟并反馈desired/actual。
- [JTC 2.54 trajectory.cpp](https://raw.githubusercontent.com/ros-controls/ros2_controllers/2.54.0/joint_trajectory_controller/src/trajectory.cpp)：P+V三次插值及`output_state.time_from_start=next_point.time_from_start`，故不把该字段伪称精确执行时钟。

本地对应srv/action定义及已安装包版本与上述匹配；运行时仅查询类型/倍率反馈，不调用变速或运动服务。

## 5. 使用

机器人停止、没有活动任务后重开HMI。轨迹执行页选择倍率→“应用真机倍率”，运行中需确认；看实际反馈，不把预演倍速或Jog速度当作真机倍率。100%仍是294.880s这一轨迹的原速。当前2倍候选不提供导入执行。

本轮未关闭用户原HMI或驱动、未覆盖源轨迹/标定、未更改实际倍率，未生成可发送的2倍轨迹或执行许可。NO ROBOT MOTION。
