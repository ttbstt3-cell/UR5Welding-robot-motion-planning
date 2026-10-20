# UR5 Safe HMI

这是一个独立的 PC 端 PyQt5 + ROS 2 上位机界面。它不修改 UR ROS2 Driver、CoppeliaSim 或 MATLAB 文件，启动时也不会自动发送轨迹。

## 启动

先确保 UR5 驱动和示教器 External Control 已按现场安全流程运行，再在 Ubuntu 终端执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/ttbstt/jixiebi/UR5/ur5_hmi
python3 main.py
```

也可以使用：

```bash
/home/ttbstt/jixiebi/UR5/ur5_hmi/run_hmi.sh
```

当前第一版默认连接：

```text
JointState: /joint_states
TCP pose: /tcp_pose_broadcaster/pose
Action: /scaled_joint_trajectory_controller/follow_joint_trajectory
Controller: scaled_joint_trajectory_controller
```

## 真机重启后的可靠连接

真机网口使用 NetworkManager 配置 `ur5-static`：电脑有线网卡 `eno1` 为
`192.168.0.20/24`，UR5 为 `192.168.0.10`。不要在 Wi-Fi 网卡上运行 Driver。

推荐直接运行整合后的单入口：

```bash
cd /home/ttbstt/jixiebi/UR5/ur5_hmi
./run_hmi.sh
```

`run_hmi.sh` 会调用 `../start_ur5_real.sh`：激活 `ur5-static`、等待 `eno1` 和
`192.168.0.10` 可达、启动 ROS2 Driver，然后启动 HMI。如果检测到已有 Driver，
会复用而不会重复启动。Driver 参数显式固定为
`reverse_ip=192.168.0.20`、`reverse_port=50001`、`script_sender_port=50002`，并启用
Dashboard client。若网线未连接，脚本会超时退出而不会留下发布缓存状态的 Driver。

整个入口仍然只会等待网络和启动软件，不会上电、释放刹车、激活运动控制器或发送轨迹。

HMI 还会对 `192.168.0.10` 的 30001/29999 做只读 TCP 探测。网线断开时不会把
ros2_control 继续发布的最后一帧关节状态显示为 Connected 或实时姿态，电源状态显示为
`电源：未知（通信中断）`。
安全模式按 `ur_dashboard_msgs/msg/SafetyMode` 枚举显示，例如 `9` 明确显示为
`FAULT`，不会再把有效故障码误标成“not available”。

## 安全行为

- 不自动激活控制器、不自动运动。
- Jog 每次基于最新 `/joint_states` 生成，只改变一个关节；默认 0.5°、5%，并使用至少 2 秒的轨迹。
- Jog/执行轨迹要求：关节反馈新鲜、`robot_program_running=true`、`robot_mode=RUNNING`、`safety_mode=NORMAL`、速度倍率大于 0、scaled controller active、action server 就绪。
- 轨迹执行还要求轨迹首点与最新真实关节角误差不超过 1.0°（`trajectory.start_tolerance_deg`）；不做 `wrapToPi`。
- `STOP MOTION` 只取消 FollowJointTrajectory goal，不替代示教器急停。
- 读取 `.mat/.yaml/.json` 只做结构与关节限位检查；HMI 不声称已完成碰撞验证。
- 左侧 3D 视图优先使用 VTK 加载 URDF 对应的 7 个 UR5 collision STL 网格，并按 URDF 的关节原点/轴和 `/joint_states` 实时更新；它只用于显示，不参与控制、IK 或碰撞检测。若 VTK Qt 集成不可用，会回退到 Matplotlib 骨架视图。
- “打开 RViz 3D 模型”按钮启动 `config/ur5_hmi.rviz`，显示 `/robot_description`、`/tf`、`/tf_static` 和 `/joint_states` 对应的 RobotModel/TF/Grid。
- 页面内提供“等轴/俯视/正视/侧视/重置”视角按钮。VTK 视图也支持鼠标：左键拖动旋转，中键拖动平移，右键拖动或滚轮缩放；如果转乱，点击“重置”即可恢复。

## 当前限制

HMI 做关节 Jog、状态监控、MAT 轨迹结构验证、只读 3D 预演和二次确认后发送；不做笛卡尔 Jog、MoveIt、碰撞检测或自动规划。`robot_program_running` 没有有效样本时运动按钮会保持禁用，这是有意的安全策略。

## 界面显示

- 程序启动前启用 Qt 高 DPI 缩放，适配 2K/4K 屏幕。
- 默认界面缩放为 `100%`，可在右侧“界面缩放”选择 `100%/125%/150%`；修改后重启 HMI 完整生效。
- 上方为大尺寸 UR5 3D 模型和状态/TCP/安全卡片，下方为“关节点动、轨迹执行、RealHome、运行日志”标签页。
- 3D 模型下方提供等轴、俯视、正视、侧视和自动适配；模型中的 J1–J6 与 TCP 标注仅用于显示。
- 3D 自动适配只根据 UR5 网格包围盒计算相机，不把坐标轴、文字和 TCP 标记算入范围；默认保留约 12%–15% 留边，机器人约占视窗高度 65%–72%，因此高 DPI 下顶部和底座不会贴边裁切。
- Jog 区以角度（deg）为主显示，速度滑块是指令速度系数，机器人实际 speed scaling 单独显示。
- 关节点动页右侧显示 Motion/Action、Start Check、Control Source、指令速度和真实 speed scaling；
  这些是状态显示，不会新增控制路径。
- UI 缩放会同步更新状态行、关节行、Jog 按钮、Tab 和间距；默认 125%。
- 右侧状态/TCP 区采用可滚动的独立行布局；100%/125%/150% 会同步调整状态行、TCP 行、STOP 警告、表格和按钮尺寸，关节表启动时自动回到顶部以保证 J1–J6 可访问。
- 关节点动主页现在只显示“当前角度/连续值/Jog”，rad 保存在对应单元格 tooltip 中；TCP 主页拆成 POSITION 和 ORIENTATION 两张卡片，位置显示 mm、姿态显示 deg，原始 m/rad 可通过 tooltip 查看。
- 顶部空间重新分配为约 40% 3D/状态区、60% 关节点动区；右侧安全控制固定在状态面板最上方，避免 STOP MOTION 被 TCP 内容推到滚动区域下方。
- 右侧状态/TCP/安全卡片已进一步紧凑化；在当前 1600×1000 Qt 窗口下，100%/125%/150% 均不需要右侧纵向滚动条即可看到基础状态和 STOP 控件。
- 掉电/未知机器人模式下，HMI 不再把最后缓存的关节角和 TCP 当作实时数据；模式 3（POWER_OFF）会显示 Stopped、Joint State stale、TCP `—`，并隐藏 3D 实时姿态。ROS2 Driver 仍显示 Connected 仅表示 PC 端 ROS2 进程在线。
- 安全控制卡片新增独立的“电源”状态芯片：`电源：已上电`（robot_mode 4/5/6/7）、`电源：已断电`（robot_mode 3）或 `电源：未知`。它与 ROS2 Driver 的 Connected 状态分开显示，避免把电脑端连接误认为机器人已上电。
- 掉电或尚未收到首帧关节反馈时，3D 视图仍显示完整的 UR5 参考构型（零位或最后一个已知构型）；仅标记为非实时并隐藏 TCP 实时标记，不再让所有连杆停留在各自局部原点而重叠成一个圆柱。
- 关节表固定按六行内容高度布局，行高随 UI Scale 为约 56/58/60 px；不再把表格最大高度放大到 620 px，避免 1080p 窗口底部裁掉 J6。
- 关节表关闭水平/垂直滚动条并固定六行内容区，保证 J1–J6 在关节点动页同时可见。

## MATLAB 轨迹导入、预演与执行门

轨迹页统一使用以下内部数据：

```text
joint_names: shoulder_pan_joint ... wrist_3_joint
positions: N×6 rad
time_from_start: N（必须严格递增才允许执行）
velocities/accelerations: optional
metadata: source / trajectory_name / description
```

MATLAB `.mat` 支持 `qPath`、`q_path`、`qTraj`、`positions`，时间支持
`timeFromStart`、`time_from_start`、`time`、`t`。没有时间的文件仍可加载和拖动查看，
但页面会显示 `TIME PARAMETERIZATION REQUIRED`，不能执行。

轨迹验证包括 NaN/Inf、N×6、严格关节顺序、关节限位、相邻跳变、时间、估计速度/加速度，
以及实时关节与第一点的 1° 起点门。HMI 不做碰撞检测，也不会把“验证 PASS”解释为碰撞 PASS。

正式 MATLAB/RRT 轨迹还必须在 MAT 的 `metadata` 中提供：

```text
collision_verified = true
collision_check_pass = true
coordinate_space = ros_real
```

否则轨迹可以预览和结构验证，但 Final Execution Check 会保持 BLOCKED。来源含 `test` 或
`commissioning` 的调试轨迹不冒充碰撞 PASS，且仍不能绕过实时状态、起点误差、控制器和人工确认门。

“3D轨迹预演”只更新左侧 UR5 模型，不发送 ROS2 指令；支持播放、暂停、停止、0.5x/1x/2x
和按点拖动。当前没有可靠的 TCP 工具模型时只做关节预演，不绘制错误 TCP 轨迹。

执行前必须同时满足 ROS/External Control、RUNNING、NORMAL、speed scaling、scaled controller、
新鲜关节反馈、轨迹验证、起点误差≤1°和 action 空闲；随后还必须勾选四项人工安全确认。
执行仍复用 `/scaled_joint_trajectory_controller/follow_joint_trajectory`。`run_hmi.sh` 会在真机
已经上电、External Control 正在运行、`robot_mode=7`、`safety_mode=1` 且速度倍率大于 0
时，自动重试激活 `scaled_joint_trajectory_controller`；这一步只取得控制器接口，不发送运动。
如果这些条件不满足，控制器保持 inactive，HMI 继续阻止点动和轨迹执行。
不做 `wrapToPi`。执行期间禁用 Jog、RealHome 和重新加载轨迹，STOP MOTION 仍可取消 action。

可生成 5 点、10 秒、每关节最大偏移 0.5° 的软件测试轨迹（不连接机器人）：

```bash
cd /home/ttbstt/jixiebi/UR5/ur5_hmi
python3 tools/create_test_trajectory.py --output /tmp/hmi_test_trajectory_5pt.mat
```

若要让测试轨迹首点接近真机，请将 HMI 当前六个角度（度）传入 `--q-deg`；即使误填，
最终 1° 起点门也会阻止执行。

执行日志保存到 `logs/trajectory_runs/`，包含 CSV 采样和 JSON summary（结果、持续时间、
每关节最大/RMS/最终跟踪误差、q_command、q_actual、TCP、speed scaling 和 safety mode）。

### 现有 RealHome → PreWeld 导出

MATLAB 已有的 RRT-Connect 结果位于：

```text
matlab仿真/weld_rrt_connect_approach_exit.mat
```

仅导出其中已独立验证通过的 `result.approach.qPath`，不重新规划：

```matlab
cd('/home/ttbstt/jixiebi/UR5/matlab仿真');
addpath(pwd);
export_realhome_to_preweld_hmi;
```

输出：

```text
轨迹文件/realhome_to_preweld.mat
轨迹文件/realhome_to_preweld_report.json
```

导出保留 ROS/真实机器人连续关节分支，不执行 `wrapToPi`。若原 RRT 文件在 ±2π 边界使用了
等价表示，导出器只通过整圈平移恢复相邻点的连续表示，并在 JSON 报告中记录调整量。当前
离线结果为 21 点、47.588 s，原始节点/边碰撞验证 PASS；尚未发送真机运动。

时间由 `parameterize_joint_path.m` 显式生成，HMI 不会猜测执行时长；没有时间的 MAT 只能预览，
显示 `TIME PARAMETERIZATION REQUIRED`。

### 最新 V1 版本

当前旧的 `realhome_to_preweld.mat` 来自本地 `RRT-Connect`。最新 V1 结果来自
`Constraint-aware Bidirectional RRT* (CBi-RRT*)`，可单独导出而不会覆盖旧文件：

```matlab
cd('/home/ttbstt/jixiebi/UR5/matlab仿真');
addpath(pwd);
export_v1_realhome_to_preweld_hmi;
```

输出：

```text
轨迹文件/realhome_to_preweld_v1.mat
轨迹文件/realhome_to_preweld_v1_report.json
```

V1 Task-A 是同一个 `qHome → qPreWeld` 任务，保存结果的首次可行解约 0.766 s，最终路径
长度 5.173362 rad，dense validation PASS。HMI 预演时请选择带 `_v1` 后缀的文件；本轮仍禁止
直接执行，先确认真实机器人当前姿态与该轨迹起点一致。

### 在线规划 Tab（离线 MATLAB，多算法对比）

在线规划页只负责读取起点、调用 MATLAB、保存结果、加载到轨迹页并预演；不会自动发送
`FollowJointTrajectory`。起点可选最新 `/joint_states` 或 HMI 配置中的 `real_home`，目标
`PreWeld` 来自 `轨迹文件/realhome_to_preweld_v1_report.json` 的 `task.goal`。

算法下拉框统一使用以下 MATLAB 实现：

```text
RRT                         RRT/jointspace/RRTJoint.m
RRT-Connect                 RRT/jointspace/RRTConnectJoint.m
Bi-RRT*                     RRT/jointspace/BiRRTStarJoint.m
Constraint-aware Bidirectional RRT* (CBi-RRT*) RRT/jointspace/FastConnectAnytimeRRTStarJoint.m
```

统一入口：`/home/ttbstt/jixiebi/RRT/benchmark/plan_hmi_request.m`。
所有算法共享同一场景、关节限制和碰撞接口，并统一进行 dense 节点/边验证与时间参数化；不
使用 `wrapToPi`。每次请求保存到 `UR5/planning_runs/<request_id>/` 下的 `request.json`、
`trajectory.mat` 和 `report.json`。成功后 HMI 自动加载到轨迹页，但仍需人工验证和3D预演，
不自动发送真机运动。

同一 Saved RealHome → PreWeld（seed `20260917`、30 s 上限）的离线对比：

| Algorithm | Success | First solution | Planning | Path length (rad) | Points | Collision |
|---|---:|---:|---:|---:|---:|---:|
| RRT | 0 | — | 30.075 s | — | — | FAIL/not generated |
| RRT-Connect | 1 | 0.973 s | 1.034 s | 5.035870 | 26 | PASS |
| Bi-RRT* | 1 | 6.760 s | 6.807 s | 4.362400 | 8 | PASS |
| Constraint-aware Bidirectional RRT* (CBi-RRT*) | 1 | 0.928 s | 30.059 s | 4.364048 | 11 | PASS |

结果目录：`/home/ttbstt/jixiebi/UR5/planning_runs/comparison_saved_realhome_*`。以上均为离线结果，没有发送
ROS2 或真机运动指令。

### 规划结果归档

在线规划完成后，HMI 会将结果复制到：

```text
/home/ttbstt/jixiebi/UR5/规划文件/
  online_<algorithm>_<request_id>.mat
  online_<algorithm>_<request_id>_report.json
  online_<algorithm>_<request_id>_request.json
```

`规划文件` 是集中查看/加载目录；`planning_runs/<request_id>` 仍保留原始请求和审计记录。
归档复制不会绕过轨迹验证、起点检查或 Final Execution Check，也不会自动发送真机运动。

### 主界面 3D 布局与相机

主界面上部与下方 Tab 使用可拖动的上下 `QSplitter`，默认约为 58% / 42%；上部
3D 模型与状态区使用约 70% / 30% 的左右 `QSplitter`。3D 区的视图按钮为：

```text
等轴 | 俯视 | 正视 | 侧视 | 机器人 | 机器人+工件
```

`机器人` 只按 UR5 mesh 适配，`机器人+工件` 按 UR5 mesh 与
`scene_geometry.json` 工件几何适配。VTK 相机适配只使用机器人/工件 bounds，
不把 TCP、文字、坐标轴、焊缝标记或规划路径计入 bounds，并在适配后执行约 3.5 倍 zoom，
使“机器人+工件”视图中的实体模型约占 70%--80% 视窗高度。
所有操作只改变显示，不改变任何模型坐标或 ROS2 控制状态。

布局截图保存在 `screenshots/main_layout_100.png`、`main_layout_125.png` 和
`main_layout_150.png`；VTK 原生“机器人+工件”渲染截图为
`screenshots/main_layout_vtk_scene_100.png`。离屏 smoke test 通过 `55/55`，本轮没有发送真机运动指令。

关节点动页在选中时会自动把主分割器调整为约 46% / 54%，并将 J1--J6 行高按
100%/125%/150% 缩放为约 50/56/62 px，确保六行和 Jog 按钮完整可见；切换到其它
页面后恢复约 58% / 42% 的监视布局。该调整只影响显示布局。

`run_hmi.sh` 启动时还会做只读的 RTDE/Dashboard 状态一致性检查。若检测到机器人重启后
Dashboard 已是 RUNNING、但旧 `ur_ros2_control_node` 仍发布 POWER_OFF/旧程序状态，脚本
会只重启 ROS2 Driver 会话，然后重新等待 50001/50002 监听；不会上电、释放抱闸或发送轨迹。

### 主界面左右分栏

主窗口现在使用一个可拖动的水平 `QSplitter`：默认左侧约 50% 为完整 UR5 3D 场景，
右侧约 50% 为 STOP MOTION、简化机器人状态和功能 Tab。右侧不再长期显示 TCP 数值、
ROS 底层模式、控制器和 Jog 状态卡；这些详细诊断保留在“运行日志”页的“详细状态”区域。

右侧 Tab 包含：关节点动、轨迹执行、在线规划、工件标定（只读）、RealHome、运行日志。
工件标定页只显示现有 `scene_geometry.json` 来源与坐标说明，不编辑任何几何或坐标。
左侧视图按钮和六个显示开关保持原有功能，未修改机器人、工件、TCP 或碰撞几何。

3D 参考坐标轴的 X/Y/Z 仅使用紧凑的显示标签，坐标轴长度、方向、原点和所有模型坐标
保持不变；VTK 和 Matplotlib fallback 的标签字号已统一缩小。

运行日志页采用“关键诊断状态 + 事件日志”布局：保留 ROS2、程序、模式、安全、速度、
控制器和关节反馈状态；TCP 数值卡片与 Jog 当前指令卡片不再长期展开，避免窄屏重叠，
底层状态变量和安全门逻辑未删除。

### 工件三点示教标定

“工件标定”页只读取最新的实时 TCP，不发送任何机器人运动。依次用 TCP 示教并读取
`P0`（工件原点）、`PX`（+X 方向点）、`PY`（+Y 方向点），再按需要重复读取 P0 作为
`P_CHECK`。计算使用右手系 `X=normalize(PX-P0)`、`Z=normalize(cross(X,PY-P0))`、
`Y=cross(Z,X)`；三点几何有效即可计算和保存，`P_CHECK` 仅用于建议验证，不再是保存门槛。

有效结果保存为 `/home/ttbstt/jixiebi/UR5/workpiece_pose_real.json`，包括
`frame_id`、`position`、`rotation_matrix`、`rpy`、`timestamp`、`residual_mm`、
`quality_band`、`review_status`、`valid`、`source` 以及四个示教点的位置、坐标帧和时间戳
（位置单位为 m，残差单位为 mm）。误差提示为 ≤2 mm 绿色、2–5 mm 黄色、>5 mm 红色；
没有 P_CHECK 或误差较大时仍可保存，但标记为 `DRAFT`。保存后 3D 工件、焊缝和已加载的规划 TCP 路径使用
同一个显示变换；“恢复仿真位姿”只恢复显示，不删除标定文件，也不改变机器人/TCP坐标。
重启后只有操作者在位姿源下拉框明确选择 `REAL CALIBRATION` 时才会读取这个 JSON；
MATLAB 规划器如需使用实测工件位姿，也应显式读取该 JSON；本轮没有自动改写 MATLAB 场景。

3D 视图在“工件”图层开启时显示洋红色球和“工件原点”文字，并附带小型工件坐标轴；
该标记不参与相机 fit，也不是碰撞体。标定切换时标记与工件、焊缝同步移动。

### 普通四点 TCP 标定（当前界面规则）

“TCP标定”页现在按普通四点流程工作：采集 4 组不同法兰姿态即可计算，允许额外采集
第 5/6 组增加冗余。计算使用 `R_i*p_flange_tcp+t_i=p_pivot_base` 的最小二乘法，
仍只使用最新关节角经过已标定 URDF 得到的 `base→flange`，不使用已经包含 TCP 偏移的
TCP 话题。

RMS 只作为界面质量提示：`≤2 mm` 绿色（精度较好）、`2–5 mm` 黄色（误差偏大，建议
重新标定）、`>5 mm` 红色（质量较差）。这不是 UR 官方精度标准。只要姿态跨度、矩阵秩和
数值求解有效，结果就可以保存；未做独立验证的文件会记录 `review_status=DRAFT`。
独立验证是可选的，按钮不会因没有额外姿态而失效。应用到软件规划模型仍需二次人工确认，
并会使旧轨迹失效、要求重新规划和完整碰撞验证；真机写入继续使用更严格的覆盖/条件数/
残差/静止状态门；矩阵秩不足、condition>100 或坐标模型不匹配时禁止应用，不因保存草稿而放宽安全检查。保存新结果前，旧的
`real_tcp_calibration.json` 会先归档到 `UR5/tcp_calibration_history/`，避免覆盖历史参数。
# TCP 定点旋转标定

“TCP标定”页与“工件标定”并列。现场人员在机器人安全静止且 External
Control 正常运行时，手动把焊枪 TCP 对准同一个固定点，在 4 个不同腕部姿态
分别点击“采集姿态”（可选第 5/6 组增加冗余）。每次采集只保存最新 `/joint_states`，并通过
`real_robot_description_calibrated.urdf` 的 `base -> flange` FK 计算法兰位姿；
`/tcp_pose_broadcaster/pose` 不参与求解。

求解模块为 `calibration/tcp_pivot.py`，使用
`R_i p_flange_tcp + t_i = p_pivot_base` 的最小二乘模型，报告姿态跨度、矩阵秩、
条件数、RMS/最大残差。需要至少 4 组且矩阵秩为 6；RMS 只用于 HMI 的绿/黄/红质量提示，
独立验证是可选的；应用前仍需人工确认并重新做规划/碰撞验证。

结果保存到 `/home/ttbstt/jixiebi/UR5/real_tcp_calibration.json`，另有应用记录
`real_tcp_calibration_applied.json`。文件记录 `base`、`flange` 坐标系、位置偏移、
现有工具姿态来源、URDF 哈希、样本和残差。MATLAB 通过
`/home/ttbstt/jixiebi/UR5/matlab仿真/load_real_tcp_calibration.m` 显式读取；它不会
自动改写冻结的 `ur5_tool_calibration.mat`。

点击“应用到软件显示 / 导出”需要人工确认。它只更新 HMI 的对比显示和 JSON
输入，不修改真机。TCP 页另外提供“读取 UR5 当前 TCP”和“写入真机当前 TCP（运行时）”
按钮：读取通过官方 URCL/RTDE 获取同一数据包中的 `tcp_offset`、`actual_q`、
`actual_TCP_pose`、robot/safety/runtime 状态；写入前要求程序由现场人员手动停止、机器人
处于 RUNNING/NORMAL 且静止，并弹出人工确认。写入只保留当前 RX/RY/RZ，调用受限的
`set_tcp` secondary program，随后用递增 RTDE `tcp_offset` 样本读回确认；没有运动、上电、
停机、重试或自动回滚。写入失败或读回不确定时，旧轨迹会被清除并要求重新读取、规划和
碰撞验证。它设置的是当前激活 TCP 的运行时值，不等价于示教器安装配置中的永久 TCP；
重启或重新运行程序后是否保留，必须在示教器中人工保存/确认。

TCP 只读同步：TCP 页默认开启“自动同步示教器 TCP（只读）”，每 5 秒通过现有
`robot_tcp_helper --read` 的独立 RTDE 输出会话读取控制器当前生效的
`tcp_offset`、`actual_TCP_pose` 和 `actual_q`。读取失败时显示 `UNKNOWN`，不会伪造
数据，也不会调用 `set_tcp`。检测到 `tcp_offset` 改变会自动使已加载轨迹失效并要求
重新验证；不会自动把新参数写入 MATLAB 规划模型。

3D 红色 `TCP LIVE` 标记优先使用 ROS2 `/tcp_pose_broadcaster/pose`（URDF `base`
坐标系，1.5 s 内的新鲜数据），因此跟随控制器当前生效 TCP；话题过期或坐标系不受
支持时才回退到 HMI 已应用的 flange FK 偏移，并在诊断中标明回退状态。预演蓝色
TCP 与实时红色 TCP 状态隔离。HMI 仍仅显示控制器当前 TCP，不修改示教器永久安装配置。

真机写入仍要求标定姿态覆盖质量门通过（估计 XYZ 标准差不超过 2 mm）、新鲜快照、
静止状态和人工确认；独立验证姿态是可选的。若覆盖/残差门不通过，按钮仍显示 BLOCKED，
即使结果已经可以作为 DRAFT 保存。

历史姿态可在 TCP 页点击“加载已保存标定”恢复。若旧文件不符合当前质量门，HMI 会恢复
其中的 `pose_samples` 供继续采集/重算，并明确显示“仅恢复姿态，禁止写入真机”；不会把旧
的 `valid` 标志当成新的写入许可。

### 工件三点标定质量规则（当前界面）

工件标定仍使用 `P0 / PX / PY` 三点建立右手工件坐标系，`P_CHECK` 改为建议验证点。
只要三点非共线、数据有限、旋转矩阵有效，就可以保存记录；没有 `P_CHECK` 或误差较大的
结果会保存为 `DRAFT`，并按误差显示绿色（≤2 mm）、黄色（2–5 mm）或红色（>5 mm）
质量提示；其中 5–6 mm 通过人工确认后可申请应用，6–10 mm 仍可作为 DRAFT 保存并做
3D 诊断预览但禁止真实规划，超过 10 mm 建议重新标定。
保存同时保留四个点的位置、坐标帧、时间戳、误差和变换矩阵，并将旧文件归档到
`UR5/workpiece_calibration_history/`。

保存不等于应用。选择 `REAL CALIBRATION` 时仍必须完成 P_CHECK 独立几何验证、满足当前
应用精度（当前 ≤6 mm）、坐标帧一致、旋转矩阵有效并通过人工确认；不满足条件时保持 `SIMULATION`，
继续 BLOCKED。该应用门与保存质量提示分开，规划和碰撞检测逻辑不变。

工件标定页提供“加载已保存标定”和“显示到3D（预览）”两个独立操作。前者只恢复
JSON 中的位姿、采样点、时间戳和质量信息，不改变场景；后者复用现有真实工件应用门，
通过验证和人工确认后，使用同一 `T_base_workpiece_real` 更新 3D 工件、焊缝、工件原点
及已加载 TCP 路径的显示。未通过真实规划应用门的 DRAFT/低质量结果也可以在人工确认后
以 `REAL CALIBRATION PREVIEW` 诊断状态显示，但不会启用规划位姿或真机执行。恢复或显示
均不发送机器人运动指令。

### 焊枪安装姿态调整（离线）

“焊枪安装调整”页用于让 HMI 3D 中的 GunCollisionHull、焊枪显示和 wireCollision
与真实法兰安装状态对齐。角度范围为 ±180°，提供 1°/0.1° 微调和 ±90° 快捷按钮；
XYZ 平移是 flange 局部毫米偏差。所有几何作为一个刚体绕 URDF J6/flange 原点变换，
不会改变真实 TCP、关节反馈、示教器参数或冻结 MATLAB 场景。页面同时显示焊丝物理
末端与示教器 LIVE TCP 的 XYZ 偏差和距离，并可隐藏机器人查看焊枪。

预览不影响规划。`保存草稿` 和 `确认安装标定` 写入独立的
`/home/ttbstt/jixiebi/UR5/real_gun_mount_calibration.json`；确认会让已加载旧轨迹
标记 `STALE`，但不会自动把变换应用到 MATLAB 或发送机器人运动。MATLAB 规划脚本
可显式调用 `matlab仿真/load_real_gun_mount_calibration.m` 读取该文件，之后必须
重新做碰撞、TCP 一致性和路径验证；`scene_geometry.json` 与
`ur5_tool_calibration.mat` 保持冻结不变。
只有 `CONFIRMED` 文件才进入轨迹的 `gun_mount_sha256` 版本绑定；草稿仅用于预览。

### 焊丝末端与控制器 TCP 离线诊断（2026-10-10）

`tools/diagnose_gun_tcp_alignment.py` 使用同一条
`base→flange→tool0→mount→TCP` 链，离线比较 J6 轴右手旋转 `0°/+90°/-90°`。
红色 `TCP LIVE` 优先使用新鲜 ROS2 `/tcp_pose_broadcaster/pose`（控制器当前生效
TCP）；该话题过期时才使用 HMI 的 `real_tcp_calibration_applied.json` 或冻结 MAT
偏移做 FK 回退，回退不会被当作实测 TCP 做焊丝偏差诊断。

脚本把 `current_weld_geometry_audit.mat` 的 18.0 mm 长度、1.2 mm 直径作为审计输入；
焊枪头位置仍是 `GunCollisionHull` 中离冻结 TCP 最近的顶点估计，不是实测导电嘴中心。
报告保存于 `UR5/diagnostics/gun_tcp_alignment_latest.json`，不会修改标定或场景文件。

VTK 视图增加黄色 `WIRE TIP`、红色 `TCP LIVE` 和洋红色偏差线/数值；偏差线只有在
控制器 TCP 话题新鲜时显示，预演 q 不会覆盖真实关节/TCP快照。安装版本变化后，旧
轨迹仍须保持 `STALE` 并重新执行碰撞和 TCP 一致性验证。

### 采用示教器当前 TCP（只读候选/人工确认）

TCP 页的“采用示教器当前 TCP（生成候选）”只读取同一 RTDE 数据包中的
`actual_q`、`actual_TCP_pose` 和 `tcp_offset`，用已标定 URDF 的
`base→flange` FK 反算完整 `T_flange_tcp`（位置+旋转向量），不把 raw
`tcp_offset` 平移直接当作 URDF flange 坐标。候选不会覆盖旧标定，也不会写入示教器。

人工确认“示教器 TCP 确实定义在焊丝/导电嘴尖端”后，保存到
`UR5/real_tcp_calibration_versions/tcp_from_pendant_*.json`，并保留
`real_tcp_calibration_from_pendant_latest.json` 作为独立最新记录。再次人工确认后，
“应用到真实规划模型”才会写入独立的 `real_tcp_calibration_active.json`，使 HMI 与
MATLAB 真实规划读取同一份完整变换并将旧轨迹标为 `STALE`；
`real_tcp_calibration_applied.json`、`real_tcp_calibration.json` 和冻结
`ur5_tool_calibration.mat` 不会被覆盖。没有 active 文件时 MATLAB 继续使用原有文件。

### 焊枪可视/导电嘴几何审计

`tools/audit_gun_visual_geometry.py` 的只读报告为
`UR5/diagnostics/gun_visual_geometry_audit.json`。当前 `visual_path` 只是 CoppeliaSim
对象路径，工程内没有可信的 STL/CAD 文件；GunCollisionHull 是 4 个凸组件（104 顶点、
192 三角面）的碰撞代理，不能证明导电嘴中心。WeldTCP 是参考点，但尚未由实物核实为
导电嘴尖端。18.0 mm 伸出量和 1.2 mm 直径来自审计 MAT 的记录值、不是本次现场实测；
真实送丝轴线、导电嘴中心和伸出长度保持 `PENDING_PHYSICAL_MEASUREMENT`，不会自动猜测。
