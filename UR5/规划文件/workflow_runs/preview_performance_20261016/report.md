# HMI 预览规划性能调查与优化报告

同一 REAL 场景、RealHome、TCP、焊枪、焊丝、工件、地面和随机种子 20260917。所有结果 executionReady=false，NO ROBOT MOTION。

## 结果

优化前均值 **596.204 ± 1.311 s**；优化后均值 **189.660 ± 3.759 s**，减少 **68.19%**，加速 **3.14 倍**。± 为样本标准差。

基准包含用户原始实际 HMI 记录 595.277s 和一次新跑的未剖析基准 597.130s。优化有 3 次运行；最后一轮通过实际 HMI 按钮→QProcess→MATLAB→HMI 回调运行，ROS 工作线程禁用。这里只验证同种子重复性，不外推为多种子总体成功率。

实际 HMI 按钮回归：MATLAB 规划 192.420s，点击到显示 Preview Ready 197.537s，生产文件采用 0.056s；实时关节状态与原 YAML 均未改变。

| 运行 | 总耗时 s | 总节点 | 完整碰撞采样 | 预览 |
|---|---:|---:|---:|---|
| 用户原始 HMI | 595.277 | 359 | 7877 | PASS |
| 新跑旧流程 | 597.130 | 359 | 7877 | PASS |
| 优化预览 1 | 191.181 | 416 | 9131 | PASS |
| 优化预览 2 | 185.378 | 416 | 9131 | PASS |
| 优化预览 3 | 192.420 | 416 | 9131 | PASS |

## 分阶段耗时

旧流程列为新跑未剖析基准，优化列为优化重复运行均值。父阶段与 IK/碰撞子耗时存在包含关系，不能把所有子耗时再次相加。

| 阶段 | 旧流程 s | 优化均值 s |
|---|---:|---:|
| REAL 场景与版本绑定 | 4.727 | 4.273 |
| ACCP 构型搜索 | 95.266 | 81.313 |
| ACCP Cartesian Refinement | 187.933 | 13.751 |
| ACCP Adaptive Time Scaling / 控制器验证 | 28.298 | 0.000 |
| 接近段搜索与物理分支审计 | 125.610 | 17.549 |
| 退出段搜索与物理分支审计 | 105.181 | 15.476 |
| 完整预览碰撞审计 | 46.486 | 53.527 |

每请求调用口径：REAL 场景建立 1 次、ACCP 搜索 1 次、Cartesian Refinement 1 次；原预览时间缩放/控制器验证各 1 次，优化预览均为 0 次（在第二阶段恢复）。CBi-RRT* 接近/退出由各 3 次变成各 1 次，完整审计与 HMI 采用各 1 次。主搜索表仍为 2426 IK；全部实际 IK 为 3384 次。

优化实测的纯 IK 调用区间平均约 21.856s，与候选函数整段耗时区分；它包括全部 3384 次实际求解，不把碰撞/筛选包装时间都记作 IK。

第一轮优化的原生查询与缓存命中如下；这些是各阶段的子调用，不是额外阶段耗时：

| 阶段 / 查询 | 逻辑查询次数 | 原生计算次数 | 精确命中 |
|---|---:|---:|---:|
| 构型搜索 / 环境碰撞 | 12582 | 8675 | 3907 |
| 构型搜索 / 自碰撞 | 11579 | 7987 | 3592 |
| 构型搜索 / TCP FK | 3250 | 2878 | 372 |
| Cartesian Refinement / 环境碰撞 | 3177 | 1808 | 1369 |
| Cartesian Refinement / 自碰撞 | 3160 | 1791 | 1369 |
| Cartesian Refinement / TCP FK | 3177 | 1808 | 1369 |
| 完整预览 / 环境碰撞 | 9131 | 8272 | 859 |
| 完整预览 / 自碰撞 | 9131 | 8272 | 859 |

CBi-RRT* 接近与退出合计的具体计数（新跑旧流程六次尝试，对比第一轮优化两次尝试）：

| 指标 | 旧流程 | 优化 |
|---|---:|---:|
| 随机采样 | 461 | 23 |
| 原生状态检测 | 30663 | 290 |
| 连接边检测 | 3030 | 95 |
| 重连尝试 | 2267 | 0 |
| 重连成功 | 19 | 0 |
| 各尝试树节点合计 | 404 | 48 |

文件快照保存、轨迹组装保存、最终元数据处理合计约 0.378s；ACCP 内场景快照载入、结果装配及保存的剩余时间单独由 ACCP wall 与算法时间之差保留。HMI 文件读取/场景采用约 0.04s（Qt 离线回调），不构成主要瓶颈。实际进程启动到界面就绪耗时见 hmi_button_e2e.json 的 button_to_preview_sec 与 transfer；不把 MATLAB 启动/退出算作算法耗时。

## 瓶颈排名与依据

1. Cartesian Refinement 的不变限位读取：3094 次 hardState 反复访问 rigidBodyTree.Bodies；全程剖析共记录约 81 万次刚体对象构造，主要来自这种重复读取。剖析中 hardState 266.159s / refinement 303.567s，约 88%。优化只把不变限位移到密集循环外；密度、轴约束、误差阈值保留。
2. CBi-RRT* anytime 与重试：原记录接近/退出各 3 次，每次约 30s，总搜索约 180.017s；首解仅约 0.64–1.40s。原报告选中 planner 的 30s 不是所有尝试之和。优化预览在首个经过实际数值边审计的合法路径处结束；先验证关节分支/真实限位/物理边，失败不接受，必要时保留有预算的 anytime 恢复。
3. 构型搜索重复密集边检查：图构建和 Bridge 都会再次查询已检查状态。保留全部节点/边采样，用精确缓存减少重复原生查询。
4. 预览提前执行时间缩放与控制器插值审计：原 ACCP 约 28s 为这部分；剖析中 47.140s，其中 controller validation 45.591s，fminsearch 仅 0.058s。它已延迟到第二阶段，未删除；几何细化仍在预览执行。
5. 完整模型碰撞审计仍是主要必要成本；优化只复用完全相同状态和向量化最近正距离提取。

剖析会放大耗时，整次剖析约 886s，不纳入优化前后均值。以下为剖析 inclusive 时间，用于定位调用来源，不能直接与非剖析时间相加：

| 函数 | 调用次数 | 剖析 inclusive s |
|---|---:|---:|
| rigidBodyTree>rigidBodyTree.get.Bodies | 50550 | 270.505 |
| refine_accp_cartesian_path>hardState | 3094 | 266.159 |
| replan_weld_actual_axis_adaptive_core>localBuildGraph | 1 | 71.858 |
| validate_accp_controller_interpolation | 1 | 45.591 |
| replan_weld_actual_axis_adaptive_core>localCartesianBridgeFallback | 1 | 40.237 |
| inverseKinematics>inverseKinematics.solve | 3384 | 30.706 |

原日志 2426 是主构型统计；完整剖析实际 IK solve 为 3384 次，包含评分/恢复/细化。优化中的 configuration_queries 与 refinement_queries 给出实际 IK、FK、环境碰撞、自碰撞次数、原生耗时及命中数，全部保留在各 report.json。

原 595.277s 的日志没有完整阶段计时，不能精确倒推每一项历史剩余耗时；可确认约 180.017s 来自六次 RRT 搜索，其余由场景、物理分支审计、完整碰撞审计和 I/O 构成。新跑基准的 performance.stages 给出不重叠分解与总和，可直接核对原约 288s 的算法外成本。

新跑未剖析基准中，ACCP 算法外的 285.632s 具体为：RRT 六次搜索 180.026s；实际数值分支密集审计 50.765s；完整预览审计 46.486s；场景/版本建立 4.727s；ACCP 快照载入/结果装配/保存剩余 2.708s；端点、组装、文件与最终元数据等其余 0.922s。以上不重复计算，总和与阶段账本一致。

## 几何、连续性、版本与失败状态

- 原 47 个焊接 q 节点与 TCP 目标逐值相同，最大差为 0；不是仅比较节点数量。
- TCP 最大几何误差保持 0.244253mm，最大节点误差约 0.000143mm；几何姿态误差约 0.018086°。
- 最大相邻数值关节步长约 7.119°；真实限位、自碰撞、焊枪、焊丝、工件、地面采样全部通过，接口没有取模或跳过整圈运动。
- 首解策略的路径为 188 接近 + 47 焊接 + 181 退出 = 416 点；原路径为 165+47+147=359 点。它减少搜索优化时间，允许自由空间路径更长；不宣称路径最短或执行时间更短。
- 同一优化轨迹关闭缓存后重新审计 9131 个相同采样点，逐帧状态、碰撞对、最近对象名称与距离一致；最大逐帧间隙差 0mm。
- 本轮新增的场景查询缓存为单请求、单不可变模型的内存对象，使用六关节 IEEE754 完整 num2hex，不取模、不舍入、不邻近匹配。绑定 RealHome/qStart/qHome、TCP/URDF/枪/丝/工件/环境版本、快照 SHA、skip 表、限位和碰撞规则源 SHA。请求结束丢弃；没有跨场景、跨请求合法性缓存。
- 新请求、1e-12rad 差异不命中旧 q；缓存结果与原生环境/自碰撞矩阵、FK 完全相同。规划结束还重新核对场景与规则源；变更返回 STALE。
- 源码 SHA 证明原生碰撞后端、REAL 场景构建、时间缩放数学与控制器插值验证函数未改。TCP、URDF、焊枪/焊丝/工件/地面几何和控制链未写入。
- 同种子预览成功率为基准 2/2、优化 3/3；这里只说明该固定场景的重复性。

## 两阶段流程验证

预览保留 Continuous IK、Cartesian Bridge、Cartesian Refinement 和密集几何碰撞验证；输出显示专用时钟，并明确标注 time_parameterization_deferred。点击“轨迹连续规划”时 hmi_prepare_deferred_weld_timing 才调用原 Adaptive Time Scaling，再继续原速度/边界/控制器验证流程。

延迟阶段回归：q/TCP 不变，重新计算的时间戳及速度与原算法结果逐值相同；同一内存段二次进入不重复计算。原时间方案的 complete=False，失败原因原样保留，没有把尚未通过的时间方案冒充可执行。

本轮未运行会发布/执行轨迹的 ROS 入口，未生成 trajectory_execute_ready 文件。所有预览、对比与诊断保持 executionReady=false。

## 修改文件

生产入口/工作流：plan_hmi_preview_request.m、prepare_hmi_continuous_request.m、gui/planning_workflow.py。
性能适配：replan_weld_actual_axis_adaptive_core.m、generate_actual_axis_ik_candidates.m、refine_accp_cartesian_path.m、HmiExactSceneQueries.m、hmi_preview_query_binding.m、hmi_prepare_deferred_weld_timing.m。
预览首解与审计：RRT/jointspace/FastConnectAnytimeRRTStarJoint.m、hmi_plan_interface_free_path.m、hmi_audit_joint_interface_path.m、audit_hmi_preview_display.m。
基准/回归：benchmark_hmi_preview_performance.m、compare_hmi_preview_performance.m、test_hmi_exact_scene_queries.m、test_hmi_deferred_preview_timing.m、verify_hmi_preview_native_reaudit.m、tools/test_preview_performance_hmi_load.py、tools/e2e_preview_performance.py、tools/report_preview_performance.py。README 与 AGENT_HANDOFF 同步记录。

## 证据与回滚

- comparison.json：逐值节点、目标、硬约束参数、版本与成功率验证。
- baseline_profile/baseline_01/matlab_profiler.mat 与 matlab_profiler_summary.json：完整调用剖析。
- baseline_unprofiled/baseline_01/：未剖析旧流程完整日志与阶段耗时。
- optimized/optimized_01、optimized_02/：两次优化完整日志、每次 planner 尝试、原生查询统计。
- native_reaudit/ 与 native_reaudit_regression.json：关闭缓存的独立完整复审。
- deferred_timing_regression.json：第二阶段时间重建与失败状态保留。
- exact_cache_regression.json、optimized/hmi_load_regression.json、hmi_button_e2e.json：缓存与实际 HMI 链路验证。
- baseline_sources/、baseline_source_manifest.json：优化前 MATLAB/RRT 源码副本；实验输出全部在独立目录。

可在离线请求设置 performance_mode='baseline' 回到原 preview anytime/提前时间计算流程作对照；默认 HMI 预览使用优化流程。此开关不改变碰撞或精度门限。
