# UR5 Safe HMI

这是一个独立的 PC 端 PyQt5 + ROS 2 上位机界面。它不修改 UR ROS2 Driver、CoppeliaSim 或 MATLAB 文件，启动时也不会自动发送轨迹。

当前两阶段规划、独立速度对比实验及真机执行限制，见
[连续规划与执行说明](docs/continuous_planning_and_execution.md)。
`PHYSICAL_VALIDATION_PENDING`不是物理/伺服验收PASS。按用户要求，轨迹页现提供完整软件检查及人工二次确认后的现场调试执行，
不需要外部验收报告或独立授权文件；不会自动发送运动。日常连续规划已移除四方案对比，完整最终验证保留。

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

### 最新 CBi-RRT* 版本

当前旧的 `realhome_to_preweld.mat` 来自本地 `RRT-Connect`。最新 CBi-RRT* 结果来自
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

CBi-RRT* Task-A 是同一个 `qHome → qPreWeld` 任务，保存结果的首次可行解约 0.766 s，最终路径
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

该脚本中的 18.0 mm 记录只保留为冻结诊断历史；当前 REAL 规划不再使用最近凸包
顶点或历史长度估计，而读取 manifest 中 CAD 枪尖到已应用真实 TCP 的参数化圆柱
（51.126 mm × 1.2 mm）。
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
`UR5/diagnostics/gun_visual_geometry_audit.json`。当前真实视觉模型来自独立的
`real_gun_geometry/welding_gun_source.STL`；GunCollisionHull 是该 STL 的独立
V-HACD 组件，不能把最近凸包顶点当作导电嘴中心。真实送丝几何采用 manifest 中的
`MODEL_DEFINED_ACTIVE` CAD 终端面/当前真实 TCP 固定轴定义（51.126 mm、1.2 mm），不要求额外
`PENDING_PHYSICAL_MEASUREMENT` 或 contact-tip calibration；仍必须通过有限值、刚体
变换和碰撞组件完整性检查。
# New real-gun STL and independent collision layer

The replacement welding-gun package is staged in `../real_gun_geometry/`.
`vtk_robot_view.py` reads its manifest independently from the frozen
`scene_geometry.json`. After actual CoppeliaSim V-HACD export, the new complete
gun+L-bracket STL is visible as **UNCONFIRMED PREVIEW**, with an independent
orange translucent compound collision layer. It does not require precision
nozzle/wire measurements to be previewed. The unreliable old wire is explicitly
disabled in REAL geometry, not inherited as verified geometry.

Open **焊枪安装调整** to adjust source-basis R/P/Y, right-hand J6 rotation and
flange-local XYZ in mm. The starting source-at-flange pose is not a physical
alignment claim. Save draft/confirm only after checking the real flange mounting
interface. Old mount records with a different STL hash are rejected; records
include the complete `T_flange_source` and preserve previous versions.

Saving/preview does not automatically change MATLAB collision geometry. The
independent MATLAB `apply_real_gun_geometry.m` loads the same components and
transform explicitly in a calibrated REAL scene; the default frozen builder
is untouched geometrically. TCP, joint states, ROS2 and algorithms are unchanged.
Old trajectories remain STALE and unconfirmed geometry remains execution-blocked.
## 新焊枪完整 STL / V-HACD 离线接入（2026-10-10）

新 STL（含 L 支架）现可在“焊枪安装调整”页预览；外观采用完整原 STL，
橙色半透明层为独立的 26 组件 V-HACD，不再使用旧 16 个分段审查代理。
可隐藏机器人、选择焊枪视图、调整 CAD 源基准 R/P/Y、法兰轴角度和局部 XYZ，
保存带源 STL 哈希及完整 `T_flange_source` 的草稿。源原点放在法兰仅为明确标记的
初始预览，不代表真实安装已对齐；旧源模型的安装记录不能直接套用。

当前焊枪安装记录已完成人工确认并激活为 REAL `ACTIVE`；真实工件仍需人工确认，
凸包覆盖仍需审查；该段历史记录中的焊丝轴线曾标记为 `UNVERIFIED`，当前
manifest 已切换为 `MODEL_DEFINED_ACTIVE`。
旧真实轨迹保持 STALE；LIVE TCP、关节反馈、TCP 标定不随模型调整改变。
旧焊丝估算圆柱为 `DISABLED_UNVERIFIED`，不再用于当前 REAL 场景；当前模型定义
焊丝使用 manifest 固定点/轴/尺寸。详细证据见
`../real_gun_geometry/replacement_report.json` 和该目录 README。
# REAL 场景同步说明（2026-10-12）

REAL MATLAB 场景入口为 `matlab仿真/build_real_planning_scene.m`，严格模式只接受
人工确认后的 `real_gun_geometry.json`、已应用 `real_tcp_calibration_applied.json`
和工件 `review_status=CONFIRMED`。冻结仿真场景不被覆盖。

本轮用 HMI 当前保存的 `config/hmi_config.yaml` 中 RealHome 作为规划起点，真实工件
位姿与真实 TCP 已加载并生成版本绑定。离线 ACCP 在真实场景第 2 个焊点没有
可用的碰撞自由状态（初始16点；IK调用296/506，不能完成焊缝），因此没有发布
`weld_anchor_points_real.json`，也没有伪造 qPreWeld。诊断输出位于
`规划文件/real_scene_active_diagnostic_20261012/`，仅供分析，禁止执行。

新焊枪的 STL/V-HACD 与安装矩阵已由独立激活流程写入 ACTIVE manifest；真实工件仍需确认；
确认和重新规划前，所有旧轨迹保持 STALE。全程 NO ROBOT MOTION。

## 模型定义焊丝（2026-10-12，已废弃的中间版本）

真实焊枪 manifest 的焊丝几何使用 `MODEL_DEFINED_ACTIVE`：以 CAD 枪尖和同一安装链
中的当前真实 TCP 两点定义轴线，长度为两点欧氏距离（当前约 51.126 mm），直径为 1.2 mm。
这一路径不要求 physical measurement 或 contact-tip calibration，但仍要求
tip/tcp/axis/长度/尺寸为有限有效值、法兰安装变换有效、V-HACD 碰撞组件已加载。
HMI REAL preflight、MATLAB `load_real_gun_geometry.m` 和
`apply_real_gun_geometry.m` 使用同一份 manifest；真实场景中焊丝是
`flange → WeldingGunMount → WeldingGunWire` 的固定子对象，同时提供 visual 与
collisionCylinder。冻结仿真场景和旧轨迹不被修改。

历史参数化焊丝为 CAD 枪尖到已应用真实 TCP 的固定连线，长度约 51.126 mm、直径 1.2 mm；
不再要求物理导电嘴测量。MATLAB REAL 场景中的固定父子接触
`WeldingGunMount → WeldingGunWire` 已加入结构性自碰撞跳过表，但焊丝与其它
机器人连杆、真实工件的碰撞检查仍保持启用。

### 当前显示规则：保留原始几何，不生成 TCP 连接焊丝

更新：用户随后明确要求采用简化直线焊丝。当前 REAL 显示恢复枪尖到新鲜控制器
TCP LIVE 球心的直线圆柱，直径 1.2 mm；不显示 WIRE TIP 球或 ΔTCP 偏差文字。
预演连接预演 TCP，不混用真实关节状态。MATLAB 使用 manifest 的固定圆柱加入碰撞，
并检查其终点与已应用规划 TCP 一致；不一致时必须同步标定并重建，禁止使用旧端点。
当前已应用规划 TCP 对应长度 51.126 mm；这不是对现场实时 TCP 已同步的声明。
该模型为用户指定的简化避障几何，不代表已认证的原始 CAD 枪嘴轴。
下方规则保留为上一轮历史说明。

已删除黄色 WIRE TIP 球、偏差连接线、ΔTCP 标签和安装页面的偏差卡片。
真实 STL 保持原形并继承 flange 下的安装刚体变换；红色 TCP LIVE 的数据来源不变。
HMI 不再从枪尖到 TCP 两点生成圆柱，也不再从最近凸包顶点推测焊丝。
当前没有独立、已确认的原始焊丝网格，故不补画焊丝；STL 内已有外形仍完整保留。
错误的 manifest wire 记录停用为 `DISABLED_NO_ORIGINAL_WIRE`，历史数值仅供审计。
MATLAB 仍执行原有有效性检查，因此不能继续使用该错误圆柱规划；旧真实轨迹版本过期。
需要原始焊丝几何或 CAD 独立枪嘴轴定义后再接入，禁止为了重合而改变 TCP 或弯曲焊丝。

## 当前焊丝端点规则（2026-10-12）

焊丝仍为直径 1.2 mm 的直线圆柱，但只延伸到红色 TCP LIVE 标记球面，
不进入 TCP 球心。`tcp_source` 保存球心参考，`wire_end_source` 保存沿枪尖→
球心方向、距球心 25 mm 的接触点；HMI 与 MATLAB 均使用该点，实际长度为
26.125812 mm。旧的 51.125812 mm 端到球心规则已停用，manifest 变化后旧真实
轨迹继续保持 STALE；NO ROBOT MOTION。

## ACCP 候选评分与离线诊断（2026-10-12）

`online_planning.candidate_scoring_mode: true` 启用综合评分保留 beam；设为
`false` 恢复原累计关节距离排序。`candidate_score_weights: [1,1,0.1,0.1]`
分别对应关节距离（rad）、轴角误差（rad）、平均逆归一化关节限位余量、
`10mm/max(collision_clearance_mm,0.001)`。未知间隙单独标记 UNKNOWN，
不伪造数值，排序成本取保守值。评分仅排序已通过原节点/连接硬约束的候选。
底层实验函数默认仍关闭评分；真实 anchor 生成支持第三参数 selectionOptions。

每次 core 输出同目录 `<core文件名>_candidate_selection_diagnostics.json`，
记录所有节点合格候选（重复 seed 命中次数单独记录）、上一点/RealHome 距离、
轴误差、六关节限位余量、实际自/环境碰撞间隙、J 分项与是否保留。
下一点预测使用独立 IK 和原硬约束；有限9姿态搜索失败不等于不可达，
不作为新增硬门。未访问点明确为 NOT_VISITED。RealHome 来自配置文件。
诊断恢复 RNG，不改变正式求解器状态；连接池未包含某候选可能是去重，
不能单凭该字段判断碰撞。各轮单独文件避免覆盖。

本轮固定 seed=20260917，16点离线 core 对比：冻结场景两种排序均完成，
节点/边碰撞通过；真实场景两种排序均 Point1 零有效候选，尚未解除阻塞。
先前轴约束比较加载的是冻结适配器，不能作为真实场景已通过的证据。
测试和完整日志保存在 `UR5/planning_runs/accp_candidate_selection_20261012*`。
这些是诊断结果，不是可执行轨迹；未生成新真实 anchor，未发送运动。

### 轴方向容差灵敏度（2026-10-12）

新增可选 `axis_angle_tolerance_deg`（core/feasibility/candidate generator）。
未传或传空时仍使用原 `sideToleranceDeg=5°`，不修改正式默认参数和HMI配置。
该参数仅改变轴夹角允许值，不改变IK姿态精度、焊接侧位置、关节限位或碰撞门。
候选缓存key包含有效轴容差，防止不同容差复用错误结果。

离线入口 `test_accp_axis_sensitivity(outputDir)` 固定REAL场景、seed20260917、
16点及与上一轮一致的搜索/评分设置，保持工件、焊枪、焊丝碰撞全部ON。
容差5/10/15/20/30°下，Point1 IK成功均154，轴通过51/51/103/154/154，
碰撞通过与最终候选均0，16点均FAIL。20/30°下工件碰撞拒绝118、焊枪相关
自碰撞拒绝36。说明本搜索集内即使所有轴候选通过，碰撞仍阻塞；不能将
失败仅归因于5°轴约束，不代表已经证明所有可能姿态均不可行。

报告 `UR5/planning_runs/accp_axis_sensitivity_20261012/comparison.json`、
`comparison.csv`，每档保存Point1全部姿态合格候选和core诊断，完整日志
`UR5/planning_runs/accp_axis_sensitivity_20261012.log`。未生成anchor或执行轨迹。

### Point1 碰撞对象细分（2026-10-12）

`diagnose_accp_point1_collision_pairs(inputMat,outputDir)` 复用20°测试保存的
全部154候选，检查当前版本绑定一致，并按26个真实V-HACD凸组件分别诊断。
组件隔离仅发生在机器人副本，组件局部安装变换、UR5、焊丝和原skip pairs不变。
环境及自碰撞分别与完整机器人结果核对；统计为每对象涉及的候选数，不能相加。

关键结果：`component_04 ↔ WireCollision` 在154/154候选碰撞；36个候选无
工件碰撞，只有该固定工具内部碰撞；118个还与工件相撞。工件bottom/x0/x1/y1
涉及15/84/22/6候选。固定源坐标检查发现焊丝起点在component_04凸包内部，
最大外向半空间距离 -0.358933mm（这是点的半空间距离，不是圆柱穿透深度）。

每对记录最近分离距离；相交实体距离为0，原MATLAB返回NaN明确标识相交。
`checkCollision`不提供穿透深度，`penetration_depth_mm=null`并标记
`UNAVAILABLE_CHECKCOLLISION_API`，禁止把0、半空间距离或未知值冒充穿透深度。
最近10候选按碰撞对数少、非碰撞最小间隙大、距RealHome近的顺序排序，
原始连续关节值0.5°去重；同时保存原始top10。均仍BLOCKED，不是可执行姿态。
报告、完整对象计数CSV、top10和局部重叠证据保存在
`UR5/planning_runs/accp_point1_collision_pairs_20261012/`。未修改任何正式模型或门。

### 固定工具内部自碰撞过滤（2026-10-12）

经用户确认，仅新增 `WeldingGunMount ↔ WeldingGunWire` 固定装配对到
真实场景与ACCP core自碰撞skip列表。当前26个GunCollisionHull组件均属于
WeldingGunMount，因此这一对覆盖它们与焊丝的内部结构接触。
环境碰撞以及gun/wire与其他UR5连杆的检查不变；不修改IK、TCP、轴容差或阈值。

固定20°测试的154候选，工具内部碰撞154→0，工件碰撞118→118，
环境未碰撞后的焊枪自碰撞拒绝36→0，最终通过碰撞的原始候选0→36。
冷启动ACCP Point1得到22个去重候选，beam保留2个；16点仍在Point4中断。
这不是完整轨迹PASS，也不将20°设为默认。过滤对比报告和离线core保存在
`UR5/planning_runs/accp_fixed_wire_filter_20261012/`。外部自碰撞距离矩阵
（除新增内部对外）对154个姿态全部验证一致。NO ROBOT MOTION。

额外正式5°复测：Point1去重候选11，16点core节点/边碰撞PASS；最大插值TCP
误差1.482825mm，尚未通过0.5mm执行精度验收，approach/exit未规划，不可直接执行。
复测暴露已有RealHome 6×1输入与row DataFormat不匹配，core仅将其reshape为1×6，
逐值断言不变；不改变关节值、IK、TCP或约束。详见default5_core.mat和复测日志。

### 焊缝 Cartesian refinement（2026-10-12）

生产入口 `replan_weld_actual_axis_adaptive_feasibility` 的精度阶段现调用
`refine_accp_cartesian_path`：从ACCP选中状态读取原始TCP目标，位置线性插值、
四元数最短弧SLERP，对新增点复用 `generate_actual_axis_ik_candidates`。
原16点关节值逐值保留，不对路径wrapToPi；新点保持种子附近连续关节分支，
使用原IK精度、轴/焊接侧、关节限位及全部环境/自碰撞检查。每段至少插入1点，
必要时增加，仍沿用maxIntermediatePoints=4预算。失败保持complete=false。

密集复查每条关节边至少21样本（包含中点，大角度时进一步加密），最终误差
必须严格小于0.5mm才precision PASS。旧代码3°采样可能只检查端点，且只在
插点成功后检查碰撞，没有强制复查最终TCP误差；四元数也使用了NLERP。
现在使用真正SLERP并显式复查。RRT、安全门、TCP与工具几何未改变。
HMI wrapper继续读取qPathExecution，报告新增seam_cartesian_refinement；
max_tcp_error_mm显示焊缝插值误差，原节点FK/tcpPath自一致误差另存
max_tcp_node_consistency_error_mm，避免界面误显示约0mm。

真实场景离线结果：16→31点，新增15点，最大路径误差0.423961mm；独立每边
101样本复核相同，节点最大误差0.000027737mm，原16点全部保留，关节最大
raw步长6.986289°，节点/密集边碰撞PASS，正式5°轴约束不变。更密采样原路径
为1.667204mm，而旧3°采样报告1.482825mm；不是TCP标定变化。
法兰→TCP固定变换变化的Frobenius范数最大1.81e-15。

输出 `UR5/planning_runs/accp_cartesian_refinement_20261012/`：
original_qPath.mat、refined_qPath.mat、report.json；测试入口
`test_accp_cartesian_refinement(inputMat,outputDir)`。这些是未时间参数化的离线
诊断路径，不是已授权执行轨迹，不覆盖旧anchor、轨迹或标定。

插值方式需区分：HMI预演/本项误差采用线性关节边；现有导出器自动添加速度，
控制器positions+velocities对应三次插值。独立离线Hermite审计几何偏离
0.384872mm，但相同时间相位下TCP差2.331394mm。不能将路径几何精度或SLERP
节点精度宣称为时间跟踪/真机精度；本轮未修改速度生成、控制器或执行逻辑。
详见export_interpolation_audit.json。NO ROBOT MOTION。

生产feasibility入口另做冷规划回归：原16点PASS，细化31点、0.423961mm、
Precision/Overall PASS。三次导出插值额外630配置碰撞和关节限位采样PASS。
没有测试真机执行；上述精度结论不替代实时Start Check或最终人工执行确认。

### ACCP内部精度阶段与统一输出（2026-10-12）

Cartesian Refinement正式作为ACCP内部的精度增强机制，不作为第三个算法。
ACCP（Adaptive Configuration Continuation Planning，自适应构型连续规划算法）
包含四个阶段：焊缝姿态约束生成、自适应构型连续搜索、笛卡尔细化及增量IK、
连续性与密集碰撞验证。先选出稀疏可行分支，再细化，避免对大量密集TCP点
提前开展全量多分支搜索。CBi-RRT*仍负责自由空间全局避障，未修改。

`replan_weld_actual_axis_adaptive_core`拥有完整ACCP过程，内部调用现有精度
helper。`replan_weld_actual_axis_adaptive_feasibility`仅协调稀疏点二分/恢复，
不再外部调用细化；每次成功core调用的精度阶段只执行一次。

统一结果：

| 字段 | 含义 |
| --- | --- |
| qPathInitial | 原稀疏构型链，诊断用，不是默认执行输出 |
| qPath | 通过内部精度验证的高精度关节轨迹，rad |
| tcpPath | 对应密集目标TCP位置，N×3 m，robot root frame |
| tcpPosePath | 对应完整4×4目标TCP位姿cell，包含SLERP姿态 |
| qPathExecution | qPath兼容别名，保留旧loader接口 |
| configurationComplete | 稀疏构型搜索是否成功 |
| complete | 构型搜索与内部精度/连续性/碰撞验证全部通过 |

精度失败时complete=false，正式qPath/qPathExecution为空；失败候选只放在
cartesianRefinement诊断记录内。历史配置搜索测试可显式使用
configurationOnlyDiagnostic=true，但不能publish/animate，不能返回正式ACCP
complete=true或提供qPathExecution；诊断报告用configurationComplete判定粗搜索。
HMI和真实anchor生成器已改读主qPath，仍要求全部原安全门，不自动应用或执行。

算法耗时为构型搜索耗时＋内部精度耗时；外层协调器不重复计时。此次是架构
整合，保留既有精度策略/约束/几何，并不解决上节已记录的时间相位误差。
离线回归入口 `test_accp_embedded_precision(baselineFile,outputDir)`，输出保存在
`UR5/planning_runs/accp_embedded_precision_20261012/`，旧实验文件不覆盖。
论文可表述：ACCP consists of weld pose constraint generation, adaptive
configuration continuation, Cartesian trajectory refinement, and continuous
collision validation. 该表述是实现结构描述，不代替算法创新性或性能对比证据。

整合回归PASS：16→31点、最大0.423961mm，与上一轮密集qPath逐值差0；粗
构型搜索亦逐值相同，精度阶段只执行1次，碰撞和场景版本绑定PASS。搜索
14.932s＋精度129.862s＝144.794s。report.json与粗/密MAT均保存在上述新目录。
仅配置诊断验证整体complete=false，不能作为正式轨迹输出。NO ROBOT MOTION。

## 2026-10-13 REAL 模型枪口到 TCP 的参数化焊丝

真实场景使用 `MODEL_AXIS_TO_TCP_ACTIVE`。HMI 与 MATLAB 分别通过
`calibration/model_axis_wire.py`、`model_axis_wire_to_tcp.m` 只读生成同一定义：
现有 ACTIVE STL 的 CAD 枪口参考点（manifest `wire.gun_tip_model`，STL_SOURCE，m）
→ `real_tcp_calibration_applied.json` 的 URDF flange TCP。先转换 TCP 到 STL_SOURCE，
再求单位方向、两点距离及 Ø1.2 mm 圆柱。无需独立端点文件、物理导电嘴测量或黄色端点球。
红色球仅为控制器 LIVE TCP 参考；其显示半径不影响焊丝或碰撞几何。

当前模型定义长度 `51.125812 mm`。这是用户指定的参数化模型，不是已实测枪嘴轴线认证。
LIVE TCP 如与已应用标定不同，不会拉伸焊丝掩盖偏差，需要人工同步规划 TCP。
焊丝固定挂在 `WeldingGunMount`，与枪体、工件、其它 UR5 连杆的原有碰撞规则继续生效；
仅沿用既有固定枪体—焊丝装配接口过滤。模型版本更新使旧 anchor/trajectory 保持 STALE。

离线回归：`tools/test_model_axis_wire.py`、`test_model_axis_wire_to_tcp.m`、
`tools/smoke_model_axis_wire.py`；结果与离线截图在 `UR5/diagnostics/model_axis_wire_*20261013*`。
不修改 TCP、冻结仿真基准、ACCP、CBi-RRT* 或 ROS2 执行链路。NO ROBOT MOTION。

### 当前控制器 TCP 是真实规划的唯一目标来源

在线 REAL 规划必须核对新鲜 RTDE `tcp_offset` 与已确认的控制器版本
`real_tcp_calibration_active.json`。该文件是控制器 TCP 转换到 URDF flange 后的镜像，
不是第二个独立 TCP。旧四点 `real_tcp_calibration_applied.json` 仅作历史/离线兼容记录，
不能在当前控制器版本未同步时冒充在线规划输入。采用当前值使用 TCP Tab 的
“采用示教器当前 TCP → 确认并保存新 TCP 版本 → 应用到真实规划模型”，不需重新四点标定。
变化后旧轨迹 STALE，保留所有质量/碰撞/人工确认要求，不自动写入示教器。

黄色焊丝表示当前 LIVE/预演 TCP 的显示连接；橙色半透明焊丝是已确认版本的固定
规划碰撞体（Collision Geometry 开关）。不一致时明确提示，显示连上不等于碰撞模型已同步。

### 2026-10-13：控制器 TCP 作为唯一规划来源（当前生效策略）

在线 REAL 规划的 TCP 现在唯一来自 UR5 控制器 RTDE 输出 `tcp_offset`。HMI
启动/轮询时通过输出只读 `robot_tcp_helper` 读取同一数据包中的
`tcp_offset`、`actual_TCP_pose` 和 `actual_q`，用固定的 URDF
`flange <- tool0` 变换生成完整 6D `T_flange_tcp`，并自动原子写入
`UR5/real_tcp_calibration_active.json`。该文件是规划输入的本地镜像，不是第二套
TCP，也不会写回示教器或发送机器人运动。

镜像的 `tcp_authority` 为 `UR_CONTROLLER_ACTIVE_TCP`，质量状态为
`CONTROLLER_DEFINED`（表示来源和坐标系一致，不伪造四点标定 RMS，也不声称实物
尖端精度）。保存的四点文件 `real_tcp_calibration_applied.json` 只作历史/离线诊断，
不能替代在线控制器 TCP。控制器 TCP 改变时旧轨迹自动标记 `STALE`，必须重新生成
焊点、重新做自碰撞/工件/焊枪碰撞和轨迹验证。

同步在运动或在线规划期间延后，避免半途更换场景版本；读取失败、坐标核对失败或
镜像过期时显示 `UNKNOWN/BLOCKED`，不会使用旧值掩盖问题。MATLAB REAL 场景和
参数化焊丝同样只接受这个 ACTIVE 控制器镜像；冻结仿真 TCP 和原始工件场景不变。

### 2026-10-13：替换为 A焊枪real.STL

当前视觉/碰撞包位于 `UR5/real_gun_geometry/`，来源为
`焊枪模型/A焊枪real.STL`（397,910 面）。复用 CoppeliaSim 实际 V-HACD 分解：
10 个独立凸组件，429 顶点、818 面；原始 STL 不居中、不旋转、不改写。
安装变换重新按新 CAD 的 Z=0 安装面与四个 6 mm 螺栓孔拟合，不能套用旧源坐标。
HMI 显示新模型及独立橙色碰撞层；目前为 `STAGED_ALIGNMENT_REQUIRED`，安装草稿
需在“焊枪安装调整”中人工核对并确认。重启 HMI 后读取新包。

10 姿态 HMI/MATLAB 凸包位姿一致性通过。Saved RealHome 的枪体+机器人+真实工件
离线检查无碰撞，**该检查不包含新模型焊丝，不代表完整规划通过**。
旧 CAD 焊丝参考点不迁移到新源；待新枪嘴参考确认后再生成固定焊丝。
抽样碰撞包络仍有约 0.751 mm 最大半空间覆盖偏差，需核对，未自动宣称包络验收。
旧轨迹因模型版本变化保持 STALE；TCP、URDF、工件标定和冻结场景未修改。

回滚备份：`UR5/backups/before_Areal_gun_20261013T065647Z/`，包含旧模型整包、安装、
TCP、工件记录。需回滚时恢复旧 `real_gun_geometry` 与旧安装记录，重启并重新校验。
离线结果：`Areal_visual_preview.png`、`Areal_collision_preview.png`、
`Areal_hmi_report.json`、`Areal_matlab_report.json`（均在当前几何包目录）。

### A焊枪模型的参数化焊丝（2026-10-13）

已通过新CAD枪嘴出口环面同心圆拟合定义 `gun_tip_model`，source坐标为
`[33.330001,59.362932,357.000221] mm`，证据在 `Areal_nozzle_reference.json`。
不使用旧凸包最近顶点、不复用旧模型参考点。按最新规则从该点到示教器同步的
ACTIVE规划TCP生成直圆柱，直径1.2mm，当前长度59.703019mm（随TCP版本重新计算）。
HMI黄色焊丝及独立碰撞显示层使用同一端点定义，MATLAB建立固定
`WeldingGunMount → WeldingGunWire` 子对象和collisionCylinder。

新模型安装尚待确认，焊丝状态为 `MODEL_AXIS_TO_TCP_PREVIEW`；显式离线预览可用，
普通规划加载仍要求枪体ACTIVE。未自动激活、未修改TCP/安装变换、安全门或规划算法。
CAD本身还包含约12.5mm×0.8mm细丝，源STL未删除/改写；该CAD细丝不作为新增焊丝的
末端依据。新增焊丝是按用户规则的模型连接，不声称已验证实物伸出长度/送丝轴。
6项Python焊丝测试通过；10姿态HMI/Matlab端点校验通过（MATLAB最大约1.67e-13mm）。
报告：`Areal_wire_hmi_report.json`、`Areal_wire_matlab_report.json`；
`Areal_wire_preview.png`为离线预演，蓝色点不是实时红色LIVE数据。

### A焊枪人工确认激活（2026-10-13，当前状态）

用户已明确确认新焊枪安装位置、方向和碰撞包络。当前manifest为 `ACTIVE`，
安装/10个V-HACD组件为 `CONFIRMED`，焊丝为 `MODEL_AXIS_TO_TCP_ACTIVE`。
确认弹窗已改为读取实际组件数和参数化焊丝长度/直径，不再固定显示旧18mm/26组件。
MATLAB正式REAL场景离线加载通过，包含示教器镜像TCP、真实工件、新枪体和焊丝
碰撞圆柱；Saved RealHome自碰撞/环境碰撞均false（`Areal_activation_scene_report.json`）。
这不是路径规划或执行验收。旧轨迹仍STALE，真实焊点文件仍需重新生成。
重启HMI加载已激活版本；未发送机器人运动、未修改TCP/安装矩阵/安全门。
# 只读焊丝几何诊断（2026-10-13）

3D视图底部的“焊丝几何诊断”可展开查看真实枪口、TCP LIVE、wire_start、
wire_end的world坐标（URDF base，mm）、长度和单位轴方向。绿色点为焊丝起点，
红色点仍为控制器LIVE TCP，黄色线为现有焊丝。诊断额外报告CAD枪嘴轴夹角和
端点误差，只报告，不修改工具安装或规划几何。没有新鲜控制器TCP时显示UNKNOWN。
预演与LIVE分开标注，预演端点不当作真机反馈。CAD枪口基准不等同实测精度。

### 两阶段规划流程（2026-10-14）

在线规划使用两个独立按钮：

1. **规划轨迹**：生成仿真/显示用 `full_raw_trajectory.mat`。REAL场景使用当前已完成的
   47点ACCP-D候选作为明确的焊接段输入，调用现有CBi-RRT*生成接近/退出路径。
   没有该候选时调用现有ACCP入口；SIMULATION显式使用冻结场景。不会读取正式
   `weld_anchor_points_real.json`，不会调用真机执行准备安全门。原始边界、重复点保留，
   碰撞结果供显示；后验碰撞FAIL仍可预览。算法无法产生路径或模型数据损坏时仍报告失败。
   候选默认读取最新`hybrid_velocity_*/hybrid_velocity_diagnostic.mat`；可用HMI配置
   `online_planning.accp_d_candidate_file`显式选择。选择来源不等于通过执行版本门。
2. **轨迹连续规划**：读取原始轨迹及其不可变场景快照，检查两处Δq、TCP/姿态差、
   速度/加速度跳变。位置断裂时有限次Cartesian线性位置+SLERP+连续IK桥接，节点/边
   碰撞和关节步长仍检查；保持原始q节点。调用现有Hybrid策略，以D为参考，只调整接口
   附近2或4个焊接节点速度以及自由空间段速度，固定原始节点及时间。用positions+velocities
   cubic Hermite验证至少5000点，另检查解析速度/加速度/位置极值及jerk和节点加速度跳变。
   最后检查TCP、URDF、枪体、焊丝、工件和碰撞场景版本，以及真实场景准入结果。

仅全部通过才生成 `trajectory_execute_ready.mat/.json` 并显示蓝色 **Execution Ready**。
加速度放在诊断报告中，不加入ROS trajectory的acceleration字段，以保持三次插值模型。
这仍是离线准备状态；既有Final Execution Check、最新起点检查和人工Execute继续生效。
仅完整显示/碰撞审计通过才显示绿色 **Preview Ready**。碰撞失败、碰撞覆盖缺失或旧轨迹
没有逐帧审计时显示 **PREVIEW WARNING**，仍可播放；预览MAT始终携带 `preview_only=true/executionReady=false`。

状态迁移：`IDLE → PLANNING PREVIEW → Preview Ready → PREPARING EXECUTION → Execution Ready`。
规划失败为`PREVIEW FAILED`；执行准备失败为`PREPARATION FAILED`，保留可播放的原始预览；
取消为`CANCELED`。`planning_busy`覆盖MATLAB过程，`continuity_busy`标识第二阶段，与motion独立。

原始MAT包含qPath、完整4×4 tcpPosePath、weldingPosePath、robotState、segmentLabel、
approach/weld/exit分段数据、各段来源绑定及raw_scene.mat快照。拼接播放时间仅标记为
DISPLAY_ONLY；第二阶段使用各段原始时间。结果全部保存到`UR5/规划文件/workflow_runs/<阶段_请求ID>/`。
真实TCP路径已经是MATLAB场景坐标，显示时只做既有base_link→base转换，不再重复施加工件位姿。

GUI函数位于`gui/planning_workflow.py`：`_start_preview_planning`、`_start_continuous_planning`、
`_finish_planning_workflow`。MATLAB入口位于`UR5/matlab仿真/plan_hmi_preview_request.m`和
`prepare_hmi_continuous_request.m`，规划器、TCP/枪体/工件及ROS控制代码保持原样。

离线回归：`tools/test_two_stage_planning_workflow.py`禁用ROS线程并验证GUI路由、碰撞FAIL预览、
失败保留预览及准备状态校验；`test_hmi_two_stage_workflow`测试版本门、局部IK/边碰撞、QP和
真实D单段5015采样。完整任务是否通过以该次`report.json`为准，不把单段PASS当作整程PASS。

### 预览工具链与穿模审计（2026-10-14）

`audit_hmi_preview_display.m`只读取保存的`raw_scene.mat`，不修改场景、URDF、凸包、TCP或
碰撞跳过规则。它导出逐帧`flangePosePath/gunPosePath/wirePosePath/collisionResult`，
并创建独立的`preview_display_snapshot.json`（固定法兰→枪体/TCP/焊丝变换、原凸组件、
原焊丝长度与3mm缩短后的碰撞长度、既有skip pairs及哈希）。动画的机器人FK和全部工具
actor在同一次刷新中更新；预览使用快照，LIVE TCP/安装调整与预览状态隔离。
界面显示当前帧TCP、枪口、焊丝末端、法兰XYZ/RPY；完整法兰矩阵和`T_flange_to_gun`
可在提示框中查看，安装矩阵也写入运行日志。

重要：审计用户`preview_20261014_121436_257478`发现UR5本体7个实际网格连杆的
`Collisions`全部为空，只有枪体10个凸组件和焊丝1个碰撞体。因此此前该真实预览的
collision PASS不具备枪体↔UR5/UR5自碰撞覆盖，不能作为安全验收证据。
第117帧STL表面交叉诊断检出`forearm_link ↔ WeldingGun_visual`；这不是FK脱节。
本轮按限制未修复MATLAB碰撞几何加载，也未改变路径。

VTK额外执行只读STL表面交叉诊断并缓存对象对；明确区分此显示诊断与规划器的实体
碰撞认证。现有法兰安装接口skip pairs仅用于解释安装接触，不放宽规划过滤。
缺失碰撞几何始终WARNING，即使原生查询返回零命中。旧预览缺少快照也为UNKNOWN/WARNING。
独立诊断副本、JSON和三视图截图：`规划文件/workflow_runs/preview_display_audit_20261014/`。
回归脚本：`tools/test_preview_display_chain.py --vtk`，使用本地显示，显式禁用HMI ROS线程。

### 预览规划性能与延迟时间优化（2026-10-16）

`plan_hmi_preview_request` 默认使用几何预览优化：不变关节限位在 Cartesian Refinement
密集循环外读取；同一不可变场景内使用完整 IEEE754 关节值缓存原生 FK/碰撞结果；CBi-RRT*
在首条通过实际数值分支、真实限位及密集物理边审计的路径后结束预览优化。首解分支失败时
继续有预算的尝试/anytime恢复，不降低碰撞密度、误差要求或过滤规则。自由空间路径可能
比 anytime 优化路径更长，Preview Ready 不表示最短路径或更短的执行时间。

ACCP Continuous IK、Cartesian Bridge、Cartesian Refinement 与几何碰撞验收继续在预览中
完成。执行用 Adaptive Time Scaling/控制器插值审计延迟到“轨迹连续规划”；新预览的
焊接段带 `time_parameterization_deferred=true` 和显示专用时钟。第二阶段通过
`hmi_prepare_deferred_weld_timing` 调用原时间算法，然后继续原有边界、速度、控制器与
版本检查。其它直接调用 ACCP 的实验默认仍执行时间阶段；预览不生成执行就绪轨迹。

缓存仅活于单次请求/函数，绑定 RealHome、起止关节值、TCP/URDF/枪/丝/工件/环境版本、
场景快照、限位、skip表及碰撞规则源哈希；不做取模、近似匹配或跨请求合法性复用。
结束时再次核对场景和规则变化，变更返回 STALE。完整审计仍覆盖当前已补齐的 UR5 本体、
焊枪、焊丝、工件与地面；历史缺失本体碰撞的预览不能借用新结果升级。

每次 `report.json` 的 `performance`、`accp_performance` 和 `audit_performance` 记录
分阶段耗时、查询数和精确缓存命中；全部 RRT 尝试均在 `interfaceTrials` 中记录。
`hmi_transfer_performance.json` 记录实际 QProcess/界面文件采用耗时。离线请求设置
`performance_mode="baseline"` 可复现旧预览工作量作对照，保持所有硬约束相同。

完整剖析、同种子重复运行、逐值焊接节点比较、关闭缓存复审以及真实 HMI 按钮回归见
`UR5/规划文件/workflow_runs/preview_performance_20261016/report.md`。

