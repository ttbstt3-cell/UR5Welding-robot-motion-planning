# HMI端点诊断与失败返回修复

2026-10-14；离线运行；robot_motion=NO，executionReady=false。

使用失败请求 `preview_20261014_143725_608305` 保存的 `raw_scene.mat`、request.json及同一份47点ACCP-D缓存。机器人、工具、工件几何、原self-collision skip表和3mm焊丝碰撞缩短设置均保持原样。场景与输入哈希、版本和完整结果见 [endpoint_diagnostic.json](endpoint_diagnostic.json)。

## finishResult修复

原端点失败分支传入7个参数，finishResult实际接收6个。已修正参数个数，并处理搜索树尚未创建时的空treePair，避免继发索引异常。只修复失败返回及诊断，搜索、连接、重连、IK及碰撞计算未修改。

端点碰撞现在返回结构体：success=false，path为空0×6矩阵，零搜索迭代/树节点；保留startStateValid、goalStateValid、两端碰撞检查细节，以及以下失败原因：

- invalid_start_collision
- invalid_goal_collision
- invalid_start_and_goal_collision

主RRT目录与UR5源码副本同时修复。两份实现各验证仅起点碰撞、仅目标碰撞、两端碰撞，6项全部PASS。原输入场景和轨迹未覆盖；回归结果保存在planner_failure_return_1_1.mat至planner_failure_return_2_3.mat。

## 指定姿态与实际请求起点

| 关节姿态（deg） | 本体自碰撞 | 工件碰撞 | forearm_link ↔ wrist_2_link |
| --- | --- | --- | --- |
| [180,-90,0,-90,0,0] | 无 | 无 | 无，距离14.431392mm |
| [0,0,0,0,0,0] | 无 | 无 | 无，距离15.218746mm |
| 本次实际请求起点 | 有 | 无 | 碰撞 |

实际请求起点：

```text
[305.286944,-99.668542,64.495017,83.969068,104.327246,-9.883301] deg
```

三组姿态均满足当前关节限位。实际起点唯一未过滤自碰撞pair是forearm_link ↔ wrist_2_link。结果为当前MATLAB碰撞模型检测，不等同于现场物理测量。碰撞接口没有提供有符号穿透深度，未编造距离。

## qOffset转换链

冻结Coppelia适配器使用qMat=qCop+qOffset，其中qOffset_deg=[180,-90,0,-90,0,0]。日志中的Coppelia零关节值因此对应冻结适配器初始化姿态[180,-90,0,-90,0,0]。

REAL场景随后导入已标定URDF，并将scene.qMat0替换为保存的RealHome。本次qStart、qHome、REAL scene.qMat0一致。HMI真实/joint_states或SavedRealHome直接生成qStart（rad）；preview将它直接传入cfg.start，缓存qPreWeld直接传入cfg.goal。该链没有额外加qOffset。

HMI将同一关节角直接代入URDF FK。显示转换只使用URDF root→base刚体变换：T_base_pose=inv(T_root_base)*T_root_pose，Rz180°变换不修改六关节值。

四个诊断姿态的HMI/MATLAB flange/base FK最大矩阵元素偏差≤2.220446e-16。诊断性地额外叠加qOffset会导致实际起点法兰位置偏差459.903mm，qPreWeld法兰位置偏差551.548mm；这项比较未写入模型或参数。

## qPreWeld

目标来源是 `UR5/规划文件/real_anchor_runs/hybrid_velocity_20261014_final/hybrid_velocity_diagnostic.mat` 的r.qPath第一行，非正式weld_anchor_points_real.json。

```text
qPreWeld_deg = [278.862708,-4.717265,-56.812594,-18.026429,-71.924571,8.263081]
TCP XYZ(base, mm) = [-356.494887,592.054404,7.315590]
```

完整TCP位姿，URDF base坐标系，平移单位m：

```text
[-0.750587914 -0.649045115  0.123928295 -0.356494887
  0.504373064 -0.683928828 -0.527114002  0.592054404
  0.426878901 -0.333139305  0.840709585  0.007315590
  0           0            0            1]
```

关节限位PASS；本体自碰撞未检出；环境碰撞FAIL。唯一环境碰撞pair为forearm_link ↔ WorkpieceCollision_wall_x1。本次该姿态未检出Gun/Wire ↔ Workpiece碰撞，检测到的是机器人前臂进入工件碰撞面。

缓存焊缝候选曾在本体碰撞几何缺失的场景中验证；补齐本体碰撞后，该入口不能继续沿用此前的碰撞PASS结论。

## 结论与修改范围

指定两组零位均未检测到碰撞。实际失败请求起点是SavedRealHome/真实反馈姿态，非日志中的冻结初始化q0。当前起点自碰撞、缓存目标前臂与工件碰撞，规划器在搜索前拒绝端点；finishResult参数错误原先掩盖了这两个原因。

未调整qOffset、Home、PreWeld、TCP、碰撞过滤/阈值、ACCP-D、机器人或工具/工件模型、ROS2执行链。未生成正式anchor或真实执行轨迹。

修改文件为两份FastConnectAnytimeRRTStarJoint.m的失败处理，以及新增只读诊断入口 `UR5/matlab仿真/diagnose_hmi_preview_endpoints.m`。本轮结果另写入项目交接文档。
