# ACCP Point1：55个工件碰撞候选审计

仅审计 real_seam.mat 中第一次拒绝原因为 workpiece_collision 的55个候选。
使用其保存时的 robot/environment/安装变换/TCP/焊丝/碰撞过滤列表，不重新IK。
检查结果逐候选与原完整机器人环境/自碰撞矩阵一致，55个均环境碰撞，0个自碰撞。
碰撞过滤规则没有修改，不生成anchor或运动轨迹，不发送机器人运动。

## 场景版本警告

保存后的控制器TCP镜像已改变；当前构建的焊丝约55.9mm，快照为59.703019mm。
scene_is_current=false。本报告只解释这55个旧候选，不能当作当前TCP下的验收。
候选保持STALE。快照与当前版本绑定均记录在collision_pairs.json。

## 碰撞对统计

| 组件 | 工件面 | 涉及候选数 |
| --- | --- | ---: |
| WireCollision | WorkpieceCollision_bottom | 55 |
| WireCollision | WorkpieceCollision_wall_y1 | 55 |
| GunCollisionHull/component_03 | WorkpieceCollision_wall_x0 | 20 |
| GunCollisionHull/component_04 | WorkpieceCollision_wall_x0 | 20 |
| GunCollisionHull/component_09 | WorkpieceCollision_wall_x0 | 12 |
| GunCollisionHull/component_10 | WorkpieceCollision_wall_x0 | 12 |

同一候选可涉及多个组件和工件面，计数不能相加作为拒绝总数。
27个候选有额外枪体碰撞，28个仅有上述两对焊丝碰撞。wall_x1没有碰撞。
Top10依次为原候选51、55、54、52、53、62、63、61、60、64，均只有焊丝与
bottom、wall_y1的两对碰撞。完整连续分支关节角保存在closest_10.csv及JSON，未wrap。

## 排序含义与限制

先按碰撞对数量升序，再按非碰撞对中的最小正间隙降序，最后按Saved RealHome
关节距离升序；按原始连续关节角每关节0.5deg去重，不合并2pi分支。
此为诊断启发式，不代表可执行，不代表最小穿透。Top10仍全部碰撞。
所列13.168557～12.815743mm是其他非碰撞对间隙，不是焊丝与工件间隙。
相交集合距离记0；MATLAB checkCollision对相交返回NaN，不提供穿透深度，
因此penetration_depth_mm=null，不推测深度。

文件：collision_pairs.json（55个候选完整逐对信息）、closest_10.json、
closest_10.csv（关节角/排序/具体碰撞对）、object_counts.csv。
完整成功日志在上一目录collision_audit_55_final.log；首次版本检查失败和
MATLAB字符串类型处理失败的日志也保留，未覆盖失败证据。

后续应检查焊丝圆柱与焊缝接触几何关系，并在当前TCP版本重新生成候选。
本报告不授权忽略焊丝—工件碰撞、缩短焊丝或改动任何正式约束。
