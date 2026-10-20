# 轨迹连续规划三阶段重构与离线对比

日期：2026-10-19。已先读最新AGENT_HANDOFF，再完成独立副本、同名入口暂存测试及生产最小切换。所有结果executionReady=false，未启动ROS节点、未发送运动。当前真实Home、原始预览、最新轨迹索引未改。

## 结论

日常入口现在只有三个核心阶段：几何一致性 → 统一局部时间/速度参数化 → 控制器等价与全程验收。旧C、Hybrid-D、全局QP和各实验函数保留，未删除或修改，不再在日常入口互相覆盖同一条轨迹。没有自动回退到旧C/D/QP或旧PreWeld。

新入口在隔离目录完成全流程：`staged_current/report.json`，402.180953秒，PHYSICAL_VALIDATION_PENDING。HMI实际回调、MAT加载、三帧显示及17个假PASS反例通过；17项受保护文件SHA前后相同。正式安装9个MATLAB文件与已验收暂存源码逐字节相同；生产路径另做8项快速契约测试，不谎称又运行一次完整400秒审计。

## 原调用链与故障

旧包装：deferred timing → 边界IK → C → C失败局部QP / C成功Hybrid-D → D失败采用C → merge → 全局QP与三轮细分 → 四轮时间/速度QP → 接口C² QP → 最终验收。多个模块可修改时间、节点和速度。

此前已确认的两项问题：

1. 旧`optimize_hmi_controller_continuity/subdivide`分别线性插值关节和Cartesian目标，导致717个新增固定节点中178个节点的FK误差超过0.5mm，最大0.637527mm。调整速度不能修正固定节点。
2. 返回Home要求零速度，却把最后43.285mm段的目标按每段匀速线性时间推进；该参考在终点仍要求约71mm/s，与静止末端冲突。

本轮不是通过放宽约束证明旧QP有解。旧QP数值实验与算法源码均保留，故障建模在新的单向数据所有权中修复。

## 三阶段职责及代码

| 核心模块 | 数据所有权与职责 |
|---|---|
| `hmi_continuous_geometry.m` | 合并三段，固定所有原q和47个原焊接TCP目标；几何样条超差时只在此模块做局部Cartesian/SLERP+IK细化。新节点检查限位、合法分支及原生场景碰撞。 |
| `hmi_continuous_parameterize.m` | 冻结几何后统一分配局部时间、生成节点速度；控制器采样只能取同一冻结曲线，不重新定义几何目标。解析修正首尾静止和三段接口C²。 |
| `hmi_continuous_acceptance.m` | 只读候选；验证原0.5mm/0.5°、解析运动极值、>=101点/边且>=5000、全场景碰撞、Home与全部版本。失败保留分项和精确原因，不修轨迹。 |

入口`prepare_hmi_continuous_request.m`从246行降到64行。两个纯参考助手为`hmi_continuous_reference.m`、`hmi_continuous_pose_at_s.m`；几何参考始终来自原始Cartesian/SLERP目标。单调局部时钟与静止五次进度在最终验收前声明，目标不使用候选实际FK反算。参考契约保存进metadata，误差曲线使用同一时钟。

共享边界若携带独立FK产生的微小矩阵差，以原ACCP目标为准；47个原焊接q和TCP目标逐值完全相同，所有原三段q同样逐值保留。自由空间没有免除0.5mm门。

几何最多6轮/1800节点/150秒；参数化最多12轮/2400节点。预算耗尽明确失败。C²只保证三段接口与静止首尾，不宣称所有内部节点C²或整体jerk最优；加速度跳变与段内jerk分别报告。

## 当前失败请求回归

输入：`prepare_20261019_152229_336637/request.json`，原预览SHA `bb3473ea3317a71c5495b1a388dcb0007c1f055499cd176dc3359d85ddd309b3`，Home `REALHOME-a3b74…`。使用请求原有motion limit profile：速度πrad/s、加速度40rad/s²配置上界、TCP保守界1.5m/s；不是新的物理运行许可。

| 指标 | 原完整恢复尝试 | 新三阶段完整入口 |
|---|---:|---:|
| 离线结果 | FAIL，全局QP未通过 | PASS，物理/伺服待验证 |
| 规划计算时间 | 962.479s | 402.181s |
| 原焊接q/TCP目标 | 47 | 47，逐值保留 |
| 最终节点 | 1077，仅失败诊断 | 638 |
| 最大轮廓误差 | 0.614167mm，仅FK诊断 | 0.251530mm |
| 最大同步误差 | 0.637527mm，仅FK诊断 | 0.287995mm |
| 最大姿态误差 | 0.280062°，仅FK诊断 | 0.262226° |
| 密集碰撞 | 最终审计未进入 | 63701样本零碰撞 |
| 关节限位 | 未完成全收尾 | 节点、采样和解析极值PASS |

新入口分阶段：输入1.431s；几何11.341s；时间/速度1.041s；控制器/FK/全碰撞382.598s；重新加载当前场景与版本3.738s；离线导出1.943s。剩余耗时主要在严格完整验收，没有减少采样密度。对比是同输入单次实测，存在并行离线进程资源共享，不作为统计加速比。

接口C²最大加速度跳变7.11e-15rad/s²。全节点最大跳变0.005476rad/s²。最大节点相邻关节变化4.487699°，没有取模消除原运动过程。最大速度0.230830rad/s；最大加速度14.465545rad/s²；段内jerk最大1600.279845rad/s³、RMS15.681107rad/s³；TCP全局保守速度界0.733044m/s。新任务运动时长207.842390s，与402.181s的计算耗时不同。

**重要限制：高jerk并未优化到旧D水平，14.47rad/s²通过的是配置上界，不是当前工具的物理验收。** HMI现有真实执行检查对这个实际MAT仍返回`MOTION_LIMITS_PAYLOAD_ACCELERATION_PENDING`，按钮不允许发送。未降低或绕过该门。

最小正间隙0.319238mm；wire shorten仍3mm。标定/凸包不确定性未认证，采样PASS不等于连续安全数学证明。

## 历史成功请求回归

输入`prepare_20261019_121401_560570/request.json`，其冻结旧Home与旧5°/s、0.261799rad/s²门不变。它是历史几何/时序回归，不是当前Home准入；新正式入口会拒绝过期Home。

| 指标 | 原历史成功结果 | 新独立副本 |
|---|---:|---:|
| 计算时间 | 478.566s | 298.797s |
| 轨迹运动时长 | 294.880s | 325.087s |
| 节点/原焊接节点 | 471 / 47 | 478 / 47 |
| 最大轮廓/同步误差mm | 0.291503 / 0.462008 | 0.271158 / 0.303025 |
| 最大姿态误差° | 0.414448 | 0.274799 |
| 最大速度rad/s | 原门PASS | 0.082903 |
| 最大加速度rad/s² | 原门PASS | 0.247550 |
| 段内jerk最大/RMS rad/s³ | 3.583708 / 0.187197 | 4.478462 / 0.161065 |
| 最大加速度跳变rad/s² | 0.439865 | 0.200930 |
| 密集碰撞 | 47001样本PASS | 47701样本PASS |

新副本几何7.226s、参数化1.087s、最终验收290.376s；最小正间隙0.428923mm。运动约慢10.2%，峰值jerk约高25%，不是所有指标都优于旧方案。统一倍率原型虽PASS却耗时1749s，已归档`copy_uniform/`与`*_uniform/`，没有接生产。

## 验证证据

- `current/report.json`、`historical/report.json`：两个独立副本完整验收，均executionReady=false。
- `staged_current/report.json`：同名暂存入口从输入到新鲜Home/场景版本及离线导出完整PASS；MAT/JSON/曲线位于同目录，不改用户最新轨迹索引。
- `contract_tests.json`：13项原节点、参考独立性、静止首尾、接口及坏几何拒绝。
- `production_contracts.json`：8项正式路径快速测试，含过期Home提前拒绝、精确失败原因、失败不导出、旧参考默认语义不变。
- `new_workflow_contract.json`：真实MAT加载和软件上下文通过；17个假PASS反例拒绝；真实执行仍被现有加速度待验证门阻止。
- `new_hmi_replay.json`：生产MainWindow真实回调/导入/3帧显示、失败状态拒绝；ROS线程禁用、隔离结果目录，不是现场或VTK真机验收。
- `legacy_workflow_contract.json`：旧报告16项失败反例继续拒绝。
- `installed_controller_equivalence.json`：直接调用安装JTC动态库，15150个关节样本P/V/A与cubic Hermite差异<=1.2e-14；没有ROS init或动作。
- `protected_verification.json`：17个CBi/ACCP/C/D/QP、模型、Home、限值、ROS发送与当前索引SHA完全不变。

## 修改与回滚

新增五文件：`hmi_continuous_geometry.m`、`hmi_continuous_parameterize.m`、`hmi_continuous_acceptance.m`、`hmi_continuous_reference.m`、`hmi_continuous_pose_at_s.m`。

修改四文件：`prepare_hmi_continuous_request.m`改为三阶段编排；`validate_accp_controller_interpolation.m`增加可选独立时钟参考（缺省旧行为不变）；`hmi_cubic_tracking_diagnostics.m`增加同一可选参考；`hmi_export_offline_continuous_result.m`仅保存参考并用其出曲线。没有向ROS消息加入加速度。

完整代码差异`production_changes.patch`，改前四文件在`before/`。安装源码与`staged/`逐字节相同；旧优化器文件仍在原位置。下次HMI“轨迹连续规划”新MATLAB进程直接使用新入口，无须重新生成原始预览；本轮未关闭用户HMI或驱动，未自动替换用户当前结果。

文档修改：项目根`AGENT_HANDOFF.md`已追加当前架构、数据、风险与证据；`UR5/ur5_hmi/docs/continuous_planning_and_execution.md`已更新日常流程及旧实验适用范围。
