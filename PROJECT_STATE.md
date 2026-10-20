# 项目状态与权威入口

更新时间：2026-10-20（Asia/Shanghai）。这是稳定上下文；动态进度见`HANDOFF.md`。

## 架构与职责

- 主项目`UR5/`，共享规划器`RRT/`；`cr7/`、`jakaA5/`是其他机器人/迁移实验，保留但非当前UR5真机入口。
- UR5 CB3，PolyScope 3.15.4；Ubuntu/ROS2 Humble；MATLAB R2025b；CoppeliaSim 4.10。
- Python/PyQt5 HMI：`UR5/ur5_hmi/main.py`、`gui/main_window.py`。
- REAL任务：RealHome → CBi-RRT*接近 → ACCP焊接 → CBi-RRT*退出 → RealHome。
- CBi-RRT* = Constraint-aware Bidirectional RRT*，负责自由空间避障；旧V1/FastConnect名称仍可能存在于函数/文件标识，不能为术语统一改变接口。
- ACCP = Adaptive Configuration Continuation Planning，负责连续焊接构型；保留Continuous IK、Cartesian Bridge、Cartesian Refinement。历史A-ARC-IK+标识不是另一个现役算法。
- 日常连续规划只有三个所有者：几何一致性 → 统一时间/速度参数化 → 控制器等价与全程验收。旧C/Hybrid-D/QP实验保留，不重新串联为日常流程。

## 场景、坐标与版本

- REAL构建：`UR5/matlab仿真/build_real_planning_scene.m`；本体是校准URDF `UR5/real_robot_description_calibrated.urdf`，加载7组本体碰撞mesh，不可只用MATLAB默认无碰撞体模型。
- 机器人标定：`UR5_real/ur5_calibration.yaml`。关节标准顺序为shoulder_pan、shoulder_lift、elbow、wrist_1、wrist_2、wrist_3；计算/ROS用rad、m，界面deg、mm。
- `base_link`与`base`有固定Rz(180°)转换；工件记录frame_id=`base`。按URDF/函数完整刚体变换，不直接复制坐标或混用RPY与旋转向量。
- TCP权威：示教器当前生效完整6D `tcp_offset`，只读RTDE同步到`UR5/real_tcp_calibration_active.json`，经tool0/控制器输出法兰 → URDF flange转换。见`calibration/tcp_frames.py`、`controller_tcp_sync.py`；真实TCP不随模型安装或预演被改写。
- 当前TCP版本记录在上述JSON（CONTROLLER-TCP-27c4…，URDF SHA17eed…）；实时新鲜度/控制器实际状态不能从磁盘记录推断。
- REAL焊枪清单`UR5/real_gun_geometry/real_gun_geometry.json`为ACTIVE，源SHA06f38186…（A焊枪real版本），真实V-HACD **10组件**。实际安装以`real_gun_mount_calibration.json`的T_flange_source、CONFIRMED版本为准；L支架含在整体STL，不重复添加。
- 不信任几何README早期“未激活/26组件”段落作为当前值。视觉与碰撞分别加载同一安装变换，不能以碰撞凸包替代外观。
- 当前焊丝由`calibration/model_axis_wire.py`及MATLAB REAL适配器按CAD枪口参考点与ACTIVE TCP解析为直圆柱，直径1.2mm；不是实测枪嘴轴线。manifest内嵌旧wire端点可能引用旧TCP，运行时解析器才是依据。collision shorten固定3mm，视觉不因此变短。
- 地面：`environment_geometry_real.json`，当前1个启用碰撞对象；清单明确缺安全栅栏/工作台/夹具，不得宣称完整现场环境。
- 唯一固定支撑例外：`fixed_environment_contacts.json`中的base_link_inertia ↔ EnvironmentCollision_Floor；未独立实测安装，不扩大skip表。工具内部合法固定邻接过滤也不得扩展到机器人运动link或环境。
- 冻结基准`UR5/scene_geometry.json`、`ur5_tool_calibration.mat`、`ur5.ttt`不可覆盖；冻结Coppelia→MATLAB qOffset=[180,-90,0,-90,0,0]°不可套到REAL反馈关节。
- TCP、URDF、枪、丝、工件、地面/场景与RealHome版本绑定；变更后旧轨迹STALE。工件保存/质量评级不等于应用或执行许可。

## 两阶段HMI与关键文件

- `gui/planning_workflow.py`：规划预览与连续规划分离；预览可展示失败/碰撞但不能冒充Collision PASS。
- MATLAB：`plan_hmi_preview_request.m`、`prepare_hmi_continuous_request.m`。
- 三模块：`hmi_continuous_geometry.m`、`hmi_continuous_parameterize.m`、`hmi_continuous_acceptance.m`；验收只读，不反向优化。
- 新连续规划默认50%保守输出：`hmi_scale_continuous_candidate.m`在唯一验收前统一时间膨胀；已验证文件可经`retime_hmi_continuous_request.m`另存调速版，q/TCP不变且重新完整验收。
- 新结果、同名报告和最新索引在`UR5/规划文件/当前轨迹/`。源请求、JSON参考、场景快照在`workflow_runs/`；不能只保留单个MAT删除依赖。
- RealHome由`ur5_hmi/config/hmi_config.yaml`的real_home和real_home_metadata共同读取，保存后重新端点验证；不使用旧qPreWeld缓存或旧Home验收。

## 已验证结论与限制

- 2026-10-20保守版回归：638节点、原47焊接节点不变；约454秒、峰值4.858°/s与9.345°/s²；63701全程采样碰撞/限位PASS，TCP轮廓约0.2485mm、同步0.2863mm、姿态0.2614°。
- 三次离线结果/真实Qt按钮回放与反例证据：`UR5/规划文件/workflow_runs/conservative_speed_20261020/report.md`。这些是既有实测离线记录，本次维护配置不重新运行规划。
- 最新结果/当前Home可能后续更新，应读索引/配置；不能拿上例旧Home的PASS当当前执行许可。
- 精度门保持轮廓/同步<0.5mm、姿态≤0.5°；每边至少101点，总采样≥5000，并检查解析关节极值、节点加速度跳变及接口C²。
- 分段Jerk与节点加速度不连续分别统计，不声称全局C²或数学连续安全证明；小间隙约0.3mm和wire缩短3mm仍需物理验证。
- 软件旧兼容范围5°/s、0.261799rad/s²不是UR5额定上限；Primary配置40rad/s²也不是当前负载可用加速度认证。e-Series能力不能直接套CB3。
- 输出保持PHYSICAL_VALIDATION_PENDING / executionReady=false；离线PASS不等于伺服/物理验收。当前软件支持完整软件门与人工确认后的现场调试，不自动放行。

## 不随维护任务改变的执行链

HMI → trajectory_to_ros_message（文件positions+velocities及时间戳）→ FollowJointTrajectory
`/scaled_joint_trajectory_controller/follow_joint_trajectory` → UR ROS2 Driver → CB3。
不是HMI直接发送movej/movel；离线模型核对的是已安装ROS2控制器P+V三次Hermite，不等于已验证UR内部伺服。
稳定接口：`trajectory/trajectory_utils.py`、`ros/trajectory_client.py`、`safety/robot_motion_limits.py`。
反馈`/joint_states`、`/tcp_pose_broadcaster/pose`；预演与真实关节/TCP状态隔离，动画默认5x不改真机速度。
`start_ur5_real.sh`不主动激活运动控制器；`ur5_hmi/run_hmi.sh`在现场已就绪时有条件watchdog会激活控制器，因此维护/备份检查不运行它。
运行必须另核对反馈、External Control、RUNNING/NORMAL、正倍率、controller active、action、起点与版本等门及人工二次确认；物理急停不替代这些检查。

## 历史排错索引（按需）

- RealHome YAML解析/缓存、Point20 Bridge、J6整圈接口、工具内部碰撞、地面固定接触：在`AGENT_HANDOFF.md`按关键词检索，不整篇读。
- 精度/平稳性与等覆盖并行审计：`workflow_runs/continuous_smoothing_20261020/`。
- 冷启动缺checkEnvironmentCollision路径：`workflow_runs/continuous_path_fix_20261020/`，入口已显式加载并核对项目RRT适配器。
- 未完成负载验收接入原型：`workflow_runs/tool_dynamics_acceptance_20261020/isolated_draft/`，非生产，不得复制激活。
- 本地维护/恢复/远程人工同步：`maintenance/BACKUP_WORKFLOW.md`；状态由维护工具实查，不把计划当已备份。
