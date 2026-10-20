# 完整 ACCP 离线运行：wire collision shorten = 3 mm

本目录是 2026-10-13 当前真实场景的完整离线 ACCP 运行结果。正式真实焊缝入口
显式固定 `wire_collision_shorten_mm=3`：只缩短 MATLAB `WeldingGunWire` 的
`collisionCylinder` 长度，保持视觉焊丝、真实 TCP、枪体几何、IK、ACCP 方向约束、
碰撞过滤和安全门不变。没有发送 ROS2 或真机运动指令。

## 结果

- 实际运行：16 点初始焊缝，最多 4 轮自适应细分，并执行有限 beam/angular recovery。
- 碰撞圆柱缩短：3.000 mm；视觉焊丝保持 55.941479 mm，碰撞圆柱为 52.941479 mm。
- Point1：32 次数值 IK，23 个位姿成功，11 个候选通过全部 Point1 过滤；
  `WireCollision` 不再是 Point1 的阻塞项。
- 完整焊缝：未通过。最终细分到 21 点时，第 20 层没有连续可行候选；
  已到达第 19 层，仍不能完成整条焊缝。
- 总 ACCP 算法时间：约 52.284 s；总墙钟时间约 60.690 s。
- `collision_check_pass=false`，`success=false`，因此没有生成或复制
  `weld_anchor_points_real.json`，也没有授权后续执行。

## 文件

- `report.json`：最终 `PLAN_FAILED` 报告与版本绑定。
- `real_seam.mat`：最终不完整 ACCP 结果，`complete=false`，仅用于诊断。
- `real_seam_round_*.mat`：各细分轮次与 recovery 结果。
- `accp_point1_diagnostics.json`：Point1 过滤统计。

失败原因是后续连续构型/环境碰撞可行性，不是 3 mm 参数未生效；不得以该不完整
结果作为可执行轨迹。
