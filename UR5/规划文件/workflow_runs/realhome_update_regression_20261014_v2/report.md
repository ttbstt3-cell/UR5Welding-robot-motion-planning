# RealHome 更新及当前场景 PreWeld 流程报告

2026-10-14。离线验证；NO ROBOT MOTION；executionReady=false。

## 结果

RealHome保存、版本绑定、端点验证、旧缓存隔离和HMI状态管理已实现。当前生产配置已由现场更新为新的无碰撞Home，本轮没有改变其关节值。当前场景端点验证PASS。

真实焊缝重新生成仍未完成：冷启动ACCP在Point14出现NO_CONNECTED_CANDIDATE，进入Cartesian Bridge时触发既有结构字段重复异常。新PreWeld未生成，未进入CBi-RRT*，未生成预览或执行轨迹。未加载旧47点候选作为目标。

## 当前 RealHome

```text
q_rad = [-1.276243034993307, -2.0179079214679163, 1.5659332275390625,
         -1.3189204374896448, -1.3801425139056605, 3.3301687240600586]
q_deg = [-73.123340, -115.617607, 89.721365,
         -75.568575, -79.076341, 190.804613]
```

关节值不wrap、不加qOffset。当前真实TCP、焊枪、3mm缩短后的焊丝碰撞体、工件和完整UR5本体collision参与检查：关节限位、RobotSelfCollision、RobotGunCollision、RobotWorkpieceCollision及Wire相关碰撞全部PASS，collision_pairs为空。

实际HMI请求的原生端点验证见 [hmi_current_home_validation/report.json](hmi_current_home_validation/report.json)。场景加载前后源文件快照一致。

## 保存行为

“保存当前姿态为RealHome”需要当前REAL CALIBRATION源及新鲜真实关节反馈。只读取姿态，不发送运动。原子保存yaml，兼容已有real_home列表，并新增real_home_metadata：

- 六关节q_rad、单位、来源、时间戳；
- 唯一realhome_version；
- URDF/robot、TCP、gun/tool、wire及collision_scene版本；
- 完整calibration_binding与本次endpoint验证记录。

保存后立即重新读取磁盘记录，切换SavedRealHome起点，清空旧raw/ready引用并将旧加载轨迹STALE；自动启动独立MATLAB端点检查。RealHome页显示PASS/FAIL及碰撞pair；旧样本/旧场景报告不接受。SIMULATION下明确提示先选择REAL CALIBRATION，不改源或保存值。

每次规划从单次读取的最新Home记录取q和版本。UI缩放保存也先读取最新配置，避免旧内存配置覆盖新Home。保存与测试均没有自动改写Home为诊断q0。

## 规划行为

```text
最新RealHome/显式Current Robot
→ 版本与场景加载快照检查
→ Endpoint START
→ 当前场景冷启动ACCP生成新的焊缝/PreWeld
→ Endpoint GOAL
→ CBi-RRT* approach/exit
→ 预览后验审计
```

HMI请求不再包含seamCandidateFile；MATLAB历史缓存分支已删除。旧调用即使传入此字段也不会读取对应文件。历史MAT仅用于复用现有参数配方，不使用历史qPath。新PreWeld只有当前场景规划完整、端点PASS后才能写入独立run的preweld_current.json。

请求包含realhome_version、collision_scene_version和preweld_version=null，明确要求重新生成。生成成功的PreWeld版本绑定当前collision_scene_version和RealHome版本；旧版本目标不能继续使用。START/GOAL失败均在RRT之前返回。HMI新增Start State/Goal State，实时输出及最终报告均更新状态。规划期间Home或场景改变，完成结果标记STALE；第二阶段同样拒绝旧Home/PreWeld版本。

碰撞场景指纹现在包含七个UR5实际碰撞mesh的内容哈希与加载覆盖状态，解决“同一URDF字节但本体collision缺失”沿用相同scene版本的问题。HMI与MATLAB当前mesh指纹、wire及scene版本逐项一致。

场景加载前后比较实际依赖源文件内容（17项，包括URDF、TCP、gun安装/碰撞数据、工件、冻结适配数据和本体STL）。加载期间内容变化则返回STALE及changed_files。仅新增版本/加载一致性检查，没有改变任何几何。

## 测试

| 测试 | 结果 |
| --- | --- |
| 临时配置保存模拟反馈、完整metadata、立即刷新起点 | PASS |
| 旧RealHome验证回包及变化场景拒收 | PASS |
| 保存后raw/ready缓存失效、旧轨迹STALE | PASS |
| SIMULATION保存拒绝且配置不变 | PASS |
| GUI请求无旧candidate、版本字段、START/GOAL显示 | PASS |
| 生产Home不被GUI测试或诊断q0覆盖 | PASS |
| 当前SavedRealHome原生端点验证 | PASS / FREE |
| 历史14:37碰撞起点 | FAIL，准确拒绝forearm_link ↔ wrist_2_link，未进入ACCP/RRT |
| 场景版本不匹配 | STALE，未进入ACCP/RRT |
| 元数据绑定兼容/缺失字段/变化检测8项 | PASS |
| 源依赖快照5类变化测试（临时副本） | PASS |
| 既有两阶段GUI回归 | PASS，旧预览仅WARNING、执行受阻 |
| 实际HMI请求冷启动当前场景ACCP | FAIL，Point14回退结构重复字段 |
| 是否加载旧qPreWeld缓存 | 否 |
| 是否生成新PreWeld | 否 |
| 是否进入CBi-RRT* | 否 |

GUI测试禁用ROS worker并mock MATLAB进程；写入临时配置。原生MATLAB验证及冷启动均离线运行。保护文件SHA256前后相同：生产Home配置、TCP、焊枪manifest、工件、URDF、ACCP核心及CBi-RRT*。

## 当前剩余阻塞

实际HMI请求日志：

```text
START COLLISION CHECK: FREE
Point14: NO_CONNECTED_CANDIDATE
localCartesianBridgeFallback, line1642:
Duplicate field name "selected_fractions"
Start State: PASS
Goal State: NOT_GENERATED
rrt_entered=false
seam_candidate_reused=false
```

`replan_weld_actual_axis_adaptive_core.m`中bridge结果struct重复声明selected_level/selected_fractions。原有缺连接候选应进入Bridge回退，但结构创建异常先中断了回退；因此还不能判断Bridge能否恢复当前Point14。按本轮禁止修改ACCP-D的限制，该核心文件保持原字节，不关闭Bridge、不放宽碰撞/方向/步长约束、不回退历史轨迹。

## 文件和证据

- GUI：gui/realhome_workflow.py、gui/planning_workflow.py、main_window.py中RealHome/在线状态与配置保存部分。
- MATLAB：plan_hmi_preview_request.m、validate_hmi_endpoint.m、validate_hmi_realhome_request.m、hmi_workflow_binding.m、hmi_request_binding_mismatches.m、hmi_planning_dependency_snapshot.m、hmi_planning_dependency_changes.m。
- 回归入口：tools/test_realhome_workflow.py、tools/test_two_stage_planning_workflow.py；MATLAB test_hmi_realhome_flow.m、test_hmi_request_binding_semantics.m及快照回归。
- [集成摘要](integration_summary.json)、[MATLAB流程回归](regression_report.json)、[绑定回归](binding_semantics_report.json)、[源快照回归](dependency_snapshot_test_report.json)。
- [实际HMI请求](hmi_current_home_preview/request.json)、[冷启动报告](hmi_current_home_preview/report.json)、[完整日志](hmi_current_home_preview/matlab.log)、[ACCP日志](hmi_current_home_preview/accp_cold_start.log)、[GUI状态验证](hmi_current_home_preview/gui_integration_report.json)。

现有HMI进程需要重新打开以加载新Python代码。TCP、工具/工件模型、碰撞阈值及skip规则、ACCP-D、CBi-RRT*、IK和ROS2控制/执行链未修改。所有旧真实轨迹继续保持STALE；未生成executionReady轨迹。
