# 连续规划离线成功与真机执行准备状态分离

日期：2026-10-16。范围：MATLAB返回契约、HMI状态管理、非执行用途结果展示与导出。NO ROBOT MOTION。

## 结论

用户报告对应的连续优化及离线验证已经完成。原来的 `PREPARATION FAILED` 是状态分类错误，不是本次优化或碰撞验证失败。

真实HMI按钮→QProcess→MATLAB→完成回调的重新运行通过：

`CONTINUOUS_PLANNING → PHYSICAL_VALIDATION_PENDING`

界面显示“连续规划完成（离线验证通过）”，另行显示“真机执行准备：待验证”。`executionReady=false`、`ready_path=None`、执行按钮禁用。此离线入口没有授予 `EXECUTION_READY` 或导出执行许可的分支。

## 准确根因与源码行号

修改前源文件保存在本目录 `before/`，不是根据修改后的行号倒推：

- `before/prepare_hmi_continuous_request.m:149`：已经通过controller audit和版本检查，却因 `requires_independent_physical_safety_validation=true` 主动抛出 `PREPARE:OfflineOnly`。随后第170行的catch把 `success` 改为false。
- `before/planning_workflow.py:272`：HMI又把任何 `executionReady != true` 直接视为连续规划失败；第57行的状态类也只能接受Execution Ready，没有“离线完成/物理待验收”的状态。
- MATLAB原第156–168行另有本轮不应该到达的execution-ready导出分支。现已删除，不是将物理待验证条件绕开后走该分支。

原始证据：`../prepare_20261016_140015_078106/report.json` 中controller complete/collision/joint-limits/analytic extrema/version均true，但failure_identifier是PREPARE:OfflineOnly。

## 重新运行的离线分项结果

新完整MATLAB报告：`../prepare_20261016_145333_894143/report.json`。

| 检查 | 结果 |
|---|---|
| 两处任务段接口C² | PASS；最大加速度跳变5.55937e-11 rad/s²，公共节点位置/速度一致 |
| 所有内部节点是否C² | 不保证；全局最大加速度跳变0.405732520 rad/s²，明确保留报告 |
| TCP轮廓误差 | 最大0.223461146 mm，RMS 0.023218212 mm，PASS |
| TCP时间同步误差 | 最大0.462406673 mm，RMS 0.053454208 mm，PASS |
| TCP姿态误差 | 最大0.273888491°，PASS |
| 速度 | 最大0.0834013475 rad/s，限制0.0872664626，PASS |
| 加速度 | 最大0.261687136 rad/s²，限制0.261799，解析极值PASS |
| 控制器等价插值 | positions+velocities cubic Hermite，41301点，PASS |
| 全程碰撞 | 41301采样PASS；使用原UR5、枪、丝、工件、地面和原过滤规则 |
| 关节限位 | 节点、采样、解析位置极值PASS |
| 场景与标定版本 | PASS |
| 段内jerk | 最大4.312066613、RMS 0.181580035 rad/s³；节点加速度跳变单独统计 |
| 轨迹 | 414优化节点，总时长254.954031717 s |

新运行数值与原用户报告相同，未更改优化器、阈值或节点选择规则。MATLAB计算503.383922 s；按钮到显示510.222416 s。本轮不是性能优化。

最小有限正碰撞间隙0.195796703 mm并非独立物理安全余量证明。wire_collision_shorten_mm=3仍是已有碰撞近似。采样PASS不证明数学连续安全，也不代表UR实际伺服跟踪已经验收。不能将“接口C² PASS”说成“全程C² PASS”。

## 状态与结果契约

- A：`CONTINUOUS_PLANNING`：保持现有边界修复、速度/时间优化、控制器等价验证、完整碰撞、限位、版本门。
- A通过：`success=true`、`continuous_planning_complete=true`、`continuous_planning_status=PASS`、各项offline_checks全部true。
- B：`stage=PHYSICAL_VALIDATION_PENDING`、物理与伺服status=PENDING、`executionReady=false`。这是已完成A，不是算法失败。
- A失败：`CONTINUOUS_PLANNING_FAILED`，保留failed_stage、failure_identifier、failure_reason、stack。不得仅凭某一个controller PASS字段覆盖其他失败。
- C：`EXECUTION_READY`保留为独立状态，但本轮离线入口没有此转换，也不创建许可文件。未来必须独立完成物理/伺服/版本等验收。

HMI在接受离线结果前独立核对success、complete、每项硬检查、>=5000采样、真实数值误差/速度/加速度、版本和非执行元数据。`executionReady=true`在这个离线接口也会被拒绝。

## 可查看和保存的非执行用途结果

新运行目录 `../prepare_20261016_145333_894143/`：

- `offline_optimized_trajectory.mat/.json`：preview_only=true、diagnostic_only=true、executionReady=false；不输出accelerations命令字段以免改变插值含义。
- `offline_tracking_curves.csv`、`offline_tracking_errors.png`：独立FK显示采样；最终验收数值来自41301点原controller audit，不能以图表采样替代碰撞验证。
- `offline_analysis.html`、`report.json`、完整`matlab.log`。
- `controller_continuity_diagnostic.mat`：完整诊断候选及审计。

HMI新增“查看离线报告 / 误差曲线”“另存离线分析…”；保存为带OFFLINE_ONLY说明的白名单ZIP，不收集execution-ready文件。按钮实际保存测试产物为本目录 `offline_analysis_saved_by_button.zip`。

新的优化节点数/顺序与原预览不同，因此非执行显示产物重新按同一已验证scene计算每帧flange/gun/wire姿态；节点碰撞状态明确来源于完整controller audit（包含各节点u=0/1），不复用旧预览帧数组，也不重新标定工具。HMI未对已变换的TCP路径重复施加工件变换。

## 回归范围与证据

1. **真实成功流程**：`hmi_actual_pass.json`。实际MainWindow按钮、实际QProcess MATLAB、实际优化与41301点验证、实际回调/加载/保存ZIP；未模拟MATLAB或PASS结果。
2. **真实失败流程**：`hmi_actual_fail.json`。独立TEST COPY故意给接近段一个节点增加20 rad，实际MATLAB在INTERFACE_GEOMETRY拒绝，返回PREPARE:InterfaceLimits；HMI显示CONTINUOUS_PLANNING_FAILED，保留原预览、不生成许可。没有改原轨迹、参数或模型。
3. **16项契约反例**：`contract_tests.json`。拒绝success=false、complete=false、碰撞/限位/解析极值失败、同步/姿态/速度/加速度超限、NaN、采样不足、版本失败、C²证据缺失、错误插值和executionReady=true。
4. **全新HMI实例加载及显示回归**：`fresh_hmi_replay.json`。读取上述真实MATLAB结果，首/中/末帧FK及碰撞显示一致；报告按钮可用；5项故意破坏报告的反例均不能显示完成或解锁执行。`fresh_hmi_replay.png`为offscreen骨架回归截图，不冒充实际VTK桌面截图。
5. Python编译通过；受保护ACCP核心、连续优化器、原controller validator、REAL场景loader、TCP、枪、工件和ROS发送层8项SHA保持不变。

**历史快照限制**：用户在原报告之后于14:42再次保存了RealHome，版本从REALHOME-1758…变为REALHOME-00fce…。旧预览在当前真实配置下正确报STALE，不应改写版本强行复用。因此成功回归采用原报告输入的真实历史场景/Home，放在隔离测试YAML中；只重放此前已有的readiness快照，禁用ROS线程，没有读取/控制真机。本报告的PASS不能作为当前新RealHome或实时控制器的执行认证。当前生产YAML逐字节未修改。

## 修改文件

生产代码：

- `UR5/matlab仿真/prepare_hmi_continuous_request.m`：只改返回状态及非执行导出入口，保留原验证。
- `UR5/matlab仿真/hmi_export_offline_continuous_result.m`：新增离线证据汇总和诊断导出。
- `UR5/ur5_hmi/gui/planning_workflow.py`：分离三阶段；非执行结果接受/显示/保存。
- `UR5/ur5_hmi/gui/main_window.py`：状态文字、两个诊断按钮、显示路径阶段识别、未运行检查不误报碰撞失败；不改变执行门判定。
- `UR5/ur5_hmi/gui/offline_planning_report.py`：本地分析页面、误差图、白名单ZIP。

测试：`test_hmi_continuous_state_failure_fixture.m`、`e2e_continuous_planning_state.py`、`test_continuous_offline_contract.py`、`test_continuous_offline_hmi.py`，以及更新旧two-stage测试中“仅凭离线审计授予Execution Ready”的过时断言。

未修改CBi-RRT*、ACCP搜索、Cartesian Bridge、IK、连续优化算法、TCP、几何、碰撞阈值/过滤和ROS2执行链。未写示教器参数、未发机器人运动、未生成trajectory_execute_ready文件。重启HMI使新Python状态机生效；旧FAILED报告保留为历史证据，不直接改写为PASS。
