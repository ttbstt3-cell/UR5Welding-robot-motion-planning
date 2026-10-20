# Hybrid-D预算退出后的候选恢复与独立求解器对比

日期：2026-10-19。原失败请求：`prepare_20261019_152229_336637`。

## 结论

**候选交接缺陷已修复；当前完整任务仍未通过。** D达到预算不再立即抹掉已验证的C；本次实际选择了`C_ACCURACY_FALLBACK`，进入原有全局连续化。后续全局QP在既有预算内未找到可接受结果，TCP同步/轮廓误差仍超0.5mm，所以可靠失败关闭。不能把本次修复称为完整规划PASS，也不能据此证明所有可能的全局轨迹在数学上均无解。

`executionReady=false`，未导出优化轨迹或执行许可，未发送机器人指令；未改CBi-RRT*、ACCP构型搜索/Bridge/Refinement、IK、TCP、几何、碰撞规则/阈值、限值profile、ROS2链路和当前HMI索引/Home。

## 1. 最小生产变更

- `matlab仿真/optimize_accp_hybrid_velocity.m`：记录原始exitflag/iterations/message/elapsed，区分`ITERATION_LIMIT`、`QP_INFEASIBLE`、`SOLVER_FAILURE`及调用异常；未收敛QP迭代值始终丢弃。显式返回已通过焊缝审计的C非最优候选来源。D数学目标、矩阵、硬约束、active-set默认及1000次预算不变。
- 新`hmi_quadprog_exit_status.m`：集中退出分类，0不再冒称-2。
- 新`hmi_select_hybrid_velocity_candidate.m`：核对不可变q/TCP/time、版本绑定、完整焊缝审计、解析极值；D有效则用D，否则只允许原样且验证有效的C。拒绝用未收敛QP输出替换C，任何缺失/非有限/不合格证据均拒绝。
- `prepare_hmi_continuous_request.m`：保存Hybrid诊断，采用上述选择器，原有全任务连续优化/插值/碰撞/版本验证继续必需。历史复用也须明确`hybridCandidateFound=true`，不得把以前回退C的第4槽误称D。增加`GLOBAL_CONTINUITY`阶段定位，不修改全局优化器。
- `hmi_export_offline_continuous_result.m`及`gui/offline_planning_report.py`：最终离线输出若存在，保留种子来源并明确C回退不是D优化成功；不改变执行许可。
- 操作说明、交接文档及专项测试更新。`before/`保留4个原生产源文件；新增文件与具体差异可直接核对。

## 2. 当前请求完整回归：恢复成功，后续可靠失败

请求副本和未简化日志：`full_prepare/request.json`、`full_recovery.log`；真实结果：`full_prepare/report.json`。
采用原预览SHA `bb3473ea3317a71c5495b1a388dcb0007c1f055499cd176dc3359d85ddd309b3`、同一场景/47焊接节点/新Home/限值版本，不重新运行几何规划。

1. C焊缝审计PASS：同步0.428674657mm，轮廓0.073083256mm，姿态0.280061645°，4601采样碰撞/限位PASS；这是焊缝段证据，不是整任务许可。
2. 正式active-set六次均exitflag=0、1000次迭代，准确标为`ITERATION_LIMIT`。
3. 成功交接原样C，`nonoptimal_fallback=true`，进入原有`optimize_hmi_controller_continuity`。
4. 全局恢复实际运行4次细化试验、4次时间/速度试验，QP均返回-2“未找到可行解”；细化增加717个**诊断**执行节点，原47焊接q/TCP及其余原节点仍保持。没有把这些未通过节点导出为优化/执行轨迹。
5. 末次FK诊断107601样本：同步最大0.637526939mm（approach，t=4.361637546s，扩展诊断边135，u=0.02）；轮廓最大0.614167135mm（approach，t=19.134844754s，边428）；姿态最大0.280061645°（weld）。诊断时长237.714414s，不是可执行时长。
6. 失败候选解析vmax0.211625180rad/s、amax26.714223245rad/s²、段内jerk最大3756.883464rad/s³；这些仅描述失败候选，绝非已验收运动性能。全局QP未通过，接口最终C²/独立最终密集碰撞/版本收尾未进入，碰撞状态`NOT_CHECKED`，不能沿用预览或C焊缝段的PASS冒充整任务PASS。
7. 返回`CONTROLLER_FK_VELOCITY_TIMING_NOT_FEASIBLE`，耗时962.479387s，`success=false / executionReady=false`；没有`offline_optimized_trajectory`或`trajectory_execute_ready`文件。该时间包含与独立对比同时运行的资源竞争，不当作标准性能基准。

本次完整运行启动时MATLAB已解析入口，原宽阶段名`VELOCITY_OPTIMIZATION`保留在真实报告中；运行中新增的`GLOBAL_CONTINUITY`只改变后续请求的日志/阶段命名，不改计算或重写旧报告。实际失败位置由报告的`velocity_optimization`与`PREPARE:Velocity`明确定位。

## 3. 独立副本：active-set与内点法

副本`solver_copy/hybrid_solver_comparison_copy.m`仅参数化算法名；与生产源的差异为函数名、独立`solverAlgorithm`配置及求解器名称记录。相同47个q、TCP、时间、目标、约束和1000迭代上限，未放宽约束、未替换正式默认。全部详情`solver_comparison.json`及两个`solver_*_report.json/mat`。

| 项目 | active-set | interior-point-convex |
| --- | ---: | ---: |
| D是否找到 | 否；保留C | 是 |
| 退出 | 六次0，各1000迭代 | 首次1，17迭代 |
| 含组装/审计总耗时 | 47.907538s | 504.030547s |
| 单次内点QP求解耗时 | — | 444.111493s |
| 表中速度来源 | C回退，不是D | 新D |
| 同步最大误差mm | 0.428674657 | 0.488369722 |
| 轮廓最大误差mm | 0.073083256 | 0.464747480 |
| 姿态最大误差° | 0.280061645 | 0.422572829 |
| 速度最大rad/s | 0.237020201 | 0.218274090 |
| 加速度最大rad/s² | 1.616888386 | 0.658433750 |
| 段内jerk最大rad/s³ | 9.054497555 | 1.803635432 |
| 段内jerk RMS rad/s³ | 1.116583041 | 0.288672821 |
| 节点加速度跳变最大rad/s² | 2.017315231 | 0.544000541 |
| 焊缝段碰撞/限位 | PASS | PASS |
| 整任务验收 | 不由此对比授予 | 未做 |

内点法成功进一步证明该D矩阵并非无解，但它本次耗时显著更高，并且精度余量变小。该独立D没有自动接入全任务，也未成为日常默认。两个方法不是相同求解次数，且存在共享主机负载；本表是这一次固定输入的结果，不能外推所有请求的成功率或速度。

## 4. 回归和失败关闭证据

- `candidate_contract_tests.json`：真实C/真实超时D选择PASS；19种反例全部拒绝，包括q/TCP/time变化、绑定不符、执行许可伪称、停止迭代值冒充C、NaN、碰撞/限位/精度/解析门失败、采样不够、错误插值及无效D。
- `hmi_contract_tests.json`：真实全局失败报告正确FAIL；即使把局部成功伪称整任务成功，缺乏最终审计仍拒绝。
- `hmi_actual_failure.json`：真实Qt生产回调重放此MATLAB结果，显示后续失败原因而非旧Hybrid错误；保留预览、执行禁用、无轨迹/调速指令、配置/当前索引逐字节未变。没有启动ROS线程或RTDE读取。
- `historical_D_positive_regression.json`：历史已知D成功输入重新调用当前生产优化器及选择器，D仍成功/被正确选用；只是历史焊缝段正例，不代表当前新Home整任务PASS。第一次历史测试因旧快照没有`scene.jointLimits`而停止；重跑仅在测试内从同一rigidBodyTree读取原限位，未改任何生产模型/配置，错误日志保留。
- Python语法检查PASS。没有为制造PASS而改动模型、门限或原报告。

## 5. 后续建议

本次合同修复已经避免可选D超时直接终止，并证实真正未通过全任务时仍失败关闭。下一步应单独分析approach的全局FK线性化QP与时间输入，定位为何既有4+4恢复预算未找到可接受结果；不要重新做ACCP搜索、切换旧非法PreWeld、强行改角度或放宽0.5mm要求。当前结果不得执行。
