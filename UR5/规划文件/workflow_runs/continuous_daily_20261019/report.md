# 日常连续规划移除四方案实验对比

2026-10-19；全部测试离线，NO ROBOT MOTION。

## 改动

- `prepare_hmi_continuous_request.m`显式关闭四策略对比，保留C初始候选审计及原有恢复/Hybrid分支。
- `generate_accp_adaptive_velocity_profile.m`增加`compareStrategies`选项，默认true保留独立实验；日常false只审计C。A/B仍作为数学参考，未审计项标记NOT_RUN_COMPARISON_ONLY，不冒充PASS或碰撞FAIL。
- `optimize_accp_hybrid_velocity.m`增加`compareReferences`选项，默认true；日常false不运行A/B对比审计，仍验证C初始解及D候选。数值求解器、目标、约束和阈值均未改。
- 新增显式离线入口`diagnose_hmi_velocity_comparison(preparationDir,outputDir)`，使用记录的时间轨迹和场景快照，检查哈希/绑定，只向新诊断目录导出四方案实验。不由HMI按钮调用。
- 新增`test_hmi_daily_velocity_result.m`、`test_hmi_velocity_comparison_modes.m`；README链接到`ur5_hmi/docs/continuous_planning_and_execution.md`。
- 原三个生产源文件保存在本目录`before/`，可逐项回滚。

最终完整轨迹验证函数、每段至少101点密度、接口连续性、解析关节速度/加速度极值、碰撞/限位/版本门均未改。CBi-RRT*、ACCP、TCP、机器人/枪/丝/工件/地面几何、Home和ROS2执行链未改。

## 同一输入实际HMI按钮回归

基线：`prepare_20261019_091210_620739`。
修改后：`prepare_20261019_093204_279082`。
两次使用同一`preview_20261019_090735_341342/full_raw_trajectory.mat`，同一不可变场景、Home及全部版本绑定。

| 指标 | 修改前 | 修改后 |
|---|---:|---:|
| MATLAB规划时间/s | 546.832251 | 469.661168 |
| 最终节点 | 469 | 469 |
| 原ACCP焊接节点 | 47 | 47 |
| 最终碰撞审计采样 | 46801 | 46801 |
| 全流程控制器碰撞审计状态数 | 69806 | 56003 |
| 最大TCP轮廓误差/mm | 0.292624131563 | 0.292624131563 |
| 最大TCP同步误差/mm | 0.461103141151 | 0.461103141151 |
| 最大姿态误差/deg | 0.346849579022 | 0.346849579022 |
| 最大关节速度/rad/s | 0.086485054763 | 0.086485054763 |
| 最大关节加速度/rad/s² | 0.260089123865 | 0.260089123865 |
| 最终碰撞、限位、接口、版本 | PASS | PASS |
| executionReady | false | false |

单次同输入减少77.171083s，14.1124%；不是多次均值或普遍性能保证。
只取消基线/A/B的13803个对比审计状态；C必要审计、初始时间缩放验收及完整最终验收均保留。
其余约8分钟开销仍存在，本轮未合并重复FK或改变密集碰撞验证。

`numerical_regression.json`严格`isequal`确认四组生成参考速度、C审计、最终q/TCP/时间/速度以及完整controller_validation结果逐值一致。

`hmi_actual_pass.json`记录实际MainWindow按钮→QProcess→MATLAB→回调/自动载入/预演/另存离线分析，耗时476.488s；ROS工作线程禁用，仅回放记录的场景/安全状态，不是物理或伺服验收。生产YAML逐字节不变、实时关节状态未覆盖、execute按钮禁用、无执行许可文件。

`offline_contract_after.json`的16种假PASS/超限/采样不足/版本/执行声明反例全部拒绝。
`hmi_restart.json`确认新建HMI能载入结果、首中末帧显示一致、查看报告正常，另5种坏报告拒绝。
`comparison_modes_unit.json`是独立合成保持轨迹单元测试，验证默认四方案实验仍可运行、日常仅审计C、Hybrid仍验证C/D，且D速度与审计逐值一致；不是当前UR5场景或物理安全认证。初版测试夹具未显式指定默认parent过滤产生重复MATLAB警告；现已只对合成夹具显式指定parent并重跑通过，真实过滤规则未改。

## 真机执行说明

当前结果仍是PHYSICAL_VALIDATION_PENDING。当前系统没有对这种离线产物进行现场物理/伺服验收签署并生成执行许可的完整入口，所以不能直接执行此文件，也不能修改元数据替代验收。

现有“轨迹执行”页仅供已批准的正式轨迹：加载→验证轨迹→最终执行门通过→人工二次确认→FollowJointTrajectory。现场验收、当前版本/起点和安全条件缺一不可。
“低速执行（5%）”按钮文字不等于自动重定时；当前转换函数直接传递文件时间戳和速度，不能只根据标签判断实际速度。本轮不改发送链路，也不自动授予许可。

完整说明见`UR5/ur5_hmi/docs/continuous_planning_and_execution.md`。
