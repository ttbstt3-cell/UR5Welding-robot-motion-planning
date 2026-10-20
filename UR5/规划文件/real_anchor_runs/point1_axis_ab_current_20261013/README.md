# 当前REAL场景 ACCP Point1方向/碰撞 A/B诊断

NO ROBOT MOTION。未修改TCP、焊枪模型、工件、碰撞过滤/阈值、正式方向参数。
上一轮CAD轴替换请求已被本轮诊断要求中断，未落实任何轴定义修改。
当前仍采用枪口→TCP连线定义，wire长度55.941479mm。

先用原失败run的real_seam_round_01.mat中的实际options冷生成当前场景候选，
搜索seed/beam/姿态网格/限位/方向阈值/碰撞条件均保持原值。仅输出路径和
configurationOnlyDiagnostic用于隔离诊断，不生成精度/时间轨迹或可执行anchor。
然后对同一完整IK成功候选池逐对检查全部环境与自碰撞。
A/B是Point1的两组过滤评估，B只移除方向比较；不是两次整条焊缝规划。
保留枪头焊接侧位置检查，不改方向目标或阈值。

| 组 | IK调用 | 数值求解返回 | 限位/位姿有效IK | 方向通过 | 碰撞通过 | 最终可行 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A 当前方向约束5deg | 264 | 264 | 144 | 55 | 0 | 0 |
| B 仅诊断跳过方向检查 | 264 | 264 | 144 | 144 | 0 | 0 |

144个限位/位姿有效候选逐个保存q_rad/q_deg、axis_angle_deg、orientation_error_deg、
TCP 4x4位姿、全部碰撞pair、最近距离及最小非碰撞间隙。其余120个求解尝试
未通过前置限位/位姿检查，不是满足IK要求的候选；计数保留在current_core.mat。
TCP与碰撞位置属于MATLAB规划base坐标系，而非STL_SOURCE或TCP局部坐标系。
矩阵及版本记录来源于同一次scene，审计开始时scene_is_current=true。

所有144候选都有WireCollision↔WorkpieceCollision_bottom及↔wall_y1。
枪体附加碰撞：component04↔wall_x0=55，component10↔wall_x0=50，
component03↔wall_x0=35，component09↔wall_x0=24，component08↔wall_x0=13，
component07↔wall_x0=5，component01/02↔wall_x1各15。对象计数允许重叠。
结论仅针对当前Point1候选池：关闭方向约束也不能产生可行候选，焊丝—工件
碰撞是独立阻塞因素，不应通过关闭方向约束解除。
不据此直接认定物理焊丝必然穿透：还需检查圆柱端部与焊缝接触定义/轴线关系。

相交最近距离按集合距离记0，原MATLAB距离NaN也保留；API不返回穿透深度。
nearest_distance_mm=0不代表已量出穿透深度，min_positive_clearance_mm是其他
非碰撞pair间隙，不是焊丝脱离工件所需位移。

完整数据：collision_pairs.json；Top10：closest_10.json/closest_10.csv；
计数：comparison.csv/object_counts.csv；冷生成原始数据：current_core.mat。
完整日志：上一目录point1_axis_ab_current_20261013.log。
全部诊断输出execution_authorized=false，不生成weld_anchor_points_real.json。
