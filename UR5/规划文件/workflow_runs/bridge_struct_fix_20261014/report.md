# Cartesian Bridge fallback结构体修复

2026-10-14。仅修复数据结构；NO ROBOT MOTION；executionReady=false。

## 修改

`replan_weld_actual_axis_adaptive_core.m`的`localCartesianBridgeFallback`在构造bridge结构体时两次声明`selected_level`和`selected_fractions`。MATLAB在执行任何桥接搜索前抛DuplicateFieldName。

生产代码仅删除第二次重复声明的一行，保留第一次定义及原先成功/失败返回字段。与[修改前备份](core_before_fix.m)逐字对比，差异只有这一行。搜索、IK、轴约束、碰撞、阈值、预算与参数均不改变。成功分支的重复赋值合法，本轮未扩展修改。

原返回结构保持：

- `bridge.success`
- `bridge.failure_reason`
- `bridge.paths(k).intermediate_states(j).Ttarget / Tactual`：插入TCP位姿
- `bridge.paths(k).intermediate_states(j).q`：插入关节节点
- `bridge.paths(k).endpoint_state`：桥接终点

JSON测试报告另外将所有返回分支插入数据整理为`inserted_poses`和`inserted_q_nodes`，没有修改生产接口。

## Point14失败重放

从上一轮实际HMI的只读场景快照及同一参数配方冷启动，在14.750s到达Point14：node=6、connected=0、2090次IK调用，触发NO_CONNECTED_CANDIDATE。

测试使用独立源副本，在原调用前保存完整输入及RNG状态后中止捕获；公开wrapper调用与生产逐字相同的私有bridge函数。重放前恢复当时RNG，逐项确认已传配方参数一致；不stub IK或collision，不强制PASS，不改变模型。

| 指标 | 结果 |
| --- | --- |
| 结构体创建与正常返回测试 | PASS |
| Point14 bridge.success | true |
| failure_reason | 空字符串 |
| 返回分支 | 2条 |
| 每条分支的插入位姿/q节点 | 各3个 |
| 全部返回分支插入数据 | 6个位姿、6个q候选 |
| 选中路径插入比例 | 25%、50%、75% |
| 选中路径新增节点 | 3 |
| Bridge IK调用 | 580 |
| Bridge边检查 | 26 |
| 每条检查边采样 | 101点 |
| 选中路径最大关节变化 | 3.515457° |
| 选中路径最大节点TCP误差 | 0.000009862mm |
| 选中路径最小有限正间隙 | 0.506720mm |
| Bridge运行时间 | 15.926s |

间隙来自采样碰撞接口，仅描述本次选中路径；全部尝试中的更小间隙包含失败尝试，不能当成该路径间隙或物理安全证明。本轮只验证局部Bridge返回，没有继续验证后续完整焊缝。

## 失败返回测试

上一层为空的测试正常返回：success=false，failure_reason=NO_PREVIOUS_LAYER，paths为空、inserted_layer_count=0。未触发重复字段或索引异常，插入位姿和q输出为空。

## 证据与保护

- [JSON报告](bridge_return_report.json)：success、插入位姿/q、failure reason及完整桥接诊断。
- [MAT结果](point14_bridge_return.mat)：原bridge、summary和归一化插入数据。
- [Point14输入fixture](point14_input_fixture.mat)：scene、seam、上一层、候选池、参数与RNG。
- [完整日志](point14_capture_and_replay.log)。
- 可复跑入口：`UR5/matlab仿真/test_cartesian_bridge_struct_return.m`。

测试保护断言通过：当前Home、真实TCP、焊枪、焊丝、工件、URDF及源依赖未变化；生产核心在测试期间未变化，源副本后缀逐字等于修复后的核心。生产核心SHA256：ffae45d1f8415fc8a2bb6fbcf125504a06fb31d8bbbdb4802c14362a43aa09bd。

没有生成或覆盖正式anchor/执行轨迹，没有修改ROS2或发送机器人指令。局部Bridge通过不替代完整ACCP、时间参数化、整程碰撞或执行验收。
