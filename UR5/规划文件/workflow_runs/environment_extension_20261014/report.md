# REAL 地面环境加载及在线入口离线复验

日期：2026-10-14。按用户后续确认，本轮只加入地面；安全栅栏以后添加。NO ROBOT MOTION。

## 结论

已从停止的 `UR5/ur5.ttt` 导出实际地面，接入 MATLAB REAL `scene.environment` 与 HMI 独立环境显示层。新增 **1 个**地面碰撞对象，环境总数 **5 个**：原 4 个 WorkpieceCollision 保持不变，加 `EnvironmentCollision_Floor`。

当前规划请求版本核对 PASS，但 Start State FAIL：`base_link_inertia ↔ EnvironmentCollision_Floor`。没有进入 CBi-RRT* 或 ACCP，没有生成新执行轨迹，`executionReady=false`。这不是规划算法报错。

旧 369 帧预览路径在加入地面后也存在动态穿地碰撞，不能继续沿用旧 Collision PASS。它只被复制到独立诊断目录，原 trajectory 不覆盖，正式版本保持 STALE。

## 导出来源与几何

- 读取独立、停止的 CoppeliaSim 实例；源场景 SHA256：`1e2b295f479d96ea51489e7bea7425dc832748695b2c0941ce69315afd642644`。未运行仿真、修改关节或保存源场景。
- 完整源 shape 清单 19 个：17 个机器人/工具/工件明确排除，环境只有 `/Floor` 与 `/Floor/box`。
- 主地面原生 cuboid：`5 × 5 × 0.20000018179416656 m`。保留源 pose、尺寸及原网格，不自动居中、缩放或移动。
- `/Floor/box` 为主地面内部的重复 slab/棋盘格顶面，保留原视觉几何记录，不添加第二个重复碰撞体；零厚度顶面也没有伪造为立体凸包。
- 源地面未启用 Coppelia collidable 位。按本轮明确要求，仅在独立规划环境清单中启用物理地面；原 `.ttt` 属性未改。
- 源场景没有 fence/workbench/fixtures；不编造其位置与尺寸。HMI仅提示源覆盖范围，不因用户暂缓栅栏而新增执行门。

新清单：[`environment_geometry_real.json`](../../../environment_geometry_real.json)。环境版本：`environment_dd72e9f8d5fbd1db`。最终文件 SHA256：`02f81207182e3275f7ce69a3aac7e06f2cd9c26fcddefeeef3ec7c00611dbc55`。

## 坐标、模型和版本

环境坐标链为 `T_URDFroot_environment = inv(T_CopBase_MatBase) * T_UR5base_environment`，再转换到 URDF `base` 供 HMI。清单保存精确 `T_base_UR5base`、冻结标定与真实 URDF 指纹；MATLAB独立验证变换及文件 SHA。没有使用显示拟合偏移。

最终当前绑定 MATLAB 快照与 HMI：环境名称、box尺寸、类型完全一致，最大矩阵元素偏差 `1.110223e-16`，最大位置偏差 `1.570092e-13 mm`。显示环境保持固定，不随 FK 或工件标定改变；实时关节/TCP未被预演覆盖。环境显示开关 PASS，camera fit 不把 5 m 地面算入机器人 fit bounds。

原 UR5 collision、工具凸组件、gun mount、wire长度/直径、原4个工件位姿与尺寸、qOffset、关节限位和 self skip 表逐项比较一致，全部 body FK 矩阵差 0。CBi-RRT*、ACCP-D、IK、TCP、轨迹优化及 ROS2 执行链未修改。既有固定 Gun-Wire 结构排除仍报告 `SKIPPED_FIXED_ATTACHMENT`，不是该 pair 碰撞检查 PASS；所有工具/机器人对新增地面的外部碰撞均检查。

环境 JSON SHA 纳入 collision_scene_version，并在加载前后依赖快照中检查源环境 JSON 及 `.ttt`。旧无地面轨迹与新版碰撞场景版本不同，保持 STALE。当前 HMI/MATLAB scene version：`SCENE-87e91e20d190130e8cc41c8c701c80fcdf7ada4419de88cee69399b54690ee92`。

## 当前在线按钮离线重跑

通过实际 `_start_preview_planning()` 捕获最新 Saved RealHome 请求，显式禁用 ROS worker，并拦截 QProcess；随后只把该请求提交本机 MATLAB 离线入口。未读正式 anchor，未调用执行准备门，未写生产 Home。

- binding_mismatches：空；加载前后 dependency validation：PASS。
- 环境总数：5；新地面数：1。
- START COLLISION CHECK：COLLISION/INVALID。
- pair：`base_link_inertia ↔ EnvironmentCollision_Floor`。
- Goal State：NOT_GENERATED；`rrt_entered=false`。
- HMI真实完成回调：`PREVIEW FAILED`，显示同一具体 pair，不显示 Collision PASS。
- `executionReady=false`，没有 execute-ready 输出。

请求、完整日志、端点和GUI回包测试：[`current_online_replan/`](current_online_replan/)。

基座 STL 最低 z 约 `-0.002399320 mm`，导入地面顶面 z 约 `+0.000246541 mm`，垂直 bounds 重叠约 `0.002646 mm`。这是源基座/地面安装接触及网格精度候选问题，**不是**原生接口返回的 penetration depth。没有为获得 PASS 而下移地面、放宽阈值或跳过基座检测；该接触是否应作为固定安装接触处理需单独确认。

## 原预览路径重新审计

输入：`workflow_runs/preview_20261014_190649_439402/full_raw_trajectory.mat`。369 帧，每条播放边 21 个采样，共 8097 次原生碰撞查询；全部样本存在固定基座与地面接触。另有动态碰撞：

| 对象 pair | 首次样本 | 时间 | 命中采样数 |
|---|---|---:|---:|
| base_link_inertia ↔ Floor | 帧1节点 | 0 s | 8097 |
| WeldingGunMount ↔ Floor | 帧141出边，u=0.9 | 80.653043 s | 1870 |
| WeldingGunWire ↔ Floor | 帧150出边，u=0.55 | 85.499510 s | 2 |
| wrist_2_link ↔ Floor | 见逐帧JSON | — | 2031 |
| wrist_3_link ↔ Floor | 见逐帧JSON | — | 1930 |

还有 forearm/upper_arm/wrist_1 ↔ Floor，以及旧路径部分边的自碰撞/工具-工件碰撞。完整 pair、q、时间与边插值比例保存在 [`real_scene_audit/preview_display_audit.json`](real_scene_audit/preview_display_audit.json)。状态 WARNING，不可执行。最小有限正距离 `0.109465 mm` 仅为**未相交 pair**的距离，不能当作整条碰撞路径的安全间隙。原生相交距离 NaN，未虚构 signed penetration。

首次完整审计绑定环境文件旧 SHA `04f45e…`；之后追加全源inventory元数据，最终 SHA 变为 `02f812…`，几何版本不变。旧审计报告不改哈希。重新构建当前场景逐项确认5个 native collider 的几何/pose及原模型完全相同，并输出新的当前绑定一帧快照；详见 [`current_binding_audit/environment_binding_refresh.json`](current_binding_audit/environment_binding_refresh.json)。不能把旧完整审计冒充当前文件SHA认证。

## 回归测试

- MATLAB synthetic loader PASS：独立测试fixture检出 Robot/Gun/Wire ↔ Environment；原4工件、工具与skip不变；1 mm错误坐标映射被STALE拒绝。测试fixture不用于真实场景。
- 真实 native 模型比较 PASS；实际地面引起的碰撞按 FAIL/WARNING保留，不把测试程序成功等同于轨迹安全。
- 最终 HMI VTK snapshot一致性、开关、FK固定环境、live/preview隔离 PASS；报告 [`final_current_matlab_hmi_display/display_test_report.json`](final_current_matlab_hmi_display/display_test_report.json)。
- 既有两阶段 GUI 离线回归 PASS：没有正式anchor仍可发起预览，碰撞失败预览不被误标为Ready，执行准备失败保留预览且禁止执行。
- 修改Python文件 compile/py_compile PASS；MATLAB代码分析无语法/执行错误。
- 本轮GUI测试前后生产配置、TCP、真实枪、工件、URDF、冻结 scene/标定/ttt 与 ACCP核心文件指纹一致。

截图：[地面整体视图](final_current_matlab_hmi_display/environment_overview.png)、[机器人近景](final_current_matlab_hmi_display/preview_environment.png)。

## 修改文件

```text
UR5/export_environment_geometry.py
UR5/environment_geometry_real.json
UR5/environment_inventory_raw.json
UR5/environment_all_shape_inventory.json
UR5/matlab仿真/load_real_environment_geometry.m
UR5/matlab仿真/hmi_environment_names.m
UR5/matlab仿真/test_real_environment_geometry_loader.m
UR5/matlab仿真/audit_real_environment_scene.m
UR5/matlab仿真/refresh_environment_audit_snapshot.m
UR5/matlab仿真/build_real_planning_scene.m
UR5/matlab仿真/validate_hmi_endpoint.m
UR5/matlab仿真/audit_hmi_preview_display.m
UR5/matlab仿真/hmi_workflow_binding.m
UR5/matlab仿真/hmi_request_binding_mismatches.m
UR5/matlab仿真/hmi_planning_dependency_snapshot.m
UR5/ur5_hmi/gui/scene_geometry.py
UR5/ur5_hmi/gui/preview_display.py
UR5/ur5_hmi/gui/vtk_robot_view.py
UR5/ur5_hmi/gui/robot_view.py
UR5/ur5_hmi/gui/main_window.py
UR5/ur5_hmi/gui/planning_workflow.py
UR5/ur5_hmi/tools/test_environment_display.py
AGENT_HANDOFF.md
```

现有 HMI 需重新打开以加载新显示/版本代码。原算法、控制链及源几何均未改变；未发送机器人运动指令。
