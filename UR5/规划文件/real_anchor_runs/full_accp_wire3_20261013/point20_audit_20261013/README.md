# ACCP Point20 离线候选过滤审计

本目录对应 `real_seam_round_recovery_beam3.mat` 的 Point19 → Point20
失败位置。审计只读复放现有 MATLAB IK、关节限位、方向约束、节点碰撞和边碰撞
检查；没有修改 ACCP、TCP、焊枪/焊丝几何、碰撞过滤或 `wire_collision_shorten_mm`。
没有连接 ROS2，也没有发送机器人运动。

## 统计

| 项目 | 数量 |
|---|---:|
| IK calls | 258 |
| 数值 IK 成功 | 258 |
| 位姿误差通过 | 230 |
| 方向约束拒绝 | 102 |
| 关节限位拒绝 | 0 |
| 工件节点碰撞拒绝 | 84 |
| 焊枪节点碰撞拒绝 | 0 |
| 焊丝节点碰撞拒绝 | 0 |
| 自碰撞拒绝 | 0 |
| 连续性拒绝 | 6 |
| └ 其中最大关节步长拒绝 | 2 |
| └ 其中边碰撞拒绝 | 4 |
| └ 边碰撞涉及焊丝 | 4 |
| 通过节点过滤 | 6（去重后 5 个主要候选） |
| 通过 Point19→Point20 连接 | 0 |

节点工件碰撞的主要 pair 为 `WeldingGunMount <->
WorkpieceCollision_wall_x1`；四条连续边的具体阻塞 pair 为
`WeldingGunWire <-> WorkpieceCollision_bottom`。节点级未发现独立焊枪、自碰撞或
焊丝碰撞拒绝；边级焊丝碰撞属于连续性检查的一部分，不能被解释为放宽安全门的理由。
`checkCollision` 不提供有符号 penetration depth，本审计仅记录 pair 和距离矩阵的
最近正距离。

## 输出文件

- `point20_filter_summary.json`：完整计数、版本绑定、状态。
- `point20_all_candidates.json`：全部位姿有效 IK 候选及碰撞 pair、TCP pose、与 Point19 的关节差。
- `point20_top10_nearest.json` / `.csv`：node-valid 候选中去重后的最近候选；本次实际得到 5 个，均未通过连接门。
- `point20_filter_counts.csv`：计数表。

Top 候选排序仅用于诊断：节点有效优先，其次碰撞 pair 数、正距离余量、与 Point19
关节距离；不会把任何候选提升为可执行轨迹。
