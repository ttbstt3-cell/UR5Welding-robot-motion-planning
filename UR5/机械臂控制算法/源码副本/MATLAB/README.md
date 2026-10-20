# UR5 MATLAB 仿真脚本

本目录集中存放当前使用的 MATLAB 脚本和函数。场景文件、标定文件、JSON 和
CoppeliaSim/Python 文件仍保留在上一级 `UR5` 目录，不改变 ROS2 控制链路。

## 文件

- `sync_ur5_joint_states.m`：加载 MATLAB UR5、冻结标定、焊枪凸包和工件碰撞代理，生成 `scene` 并显示当前场景。
- `test_ur5_ros2_control.m`：通过现有 `/joint_trajectory` 接口发送测试轨迹，同时接收 `/joint_states`。
- `build_weld_seam.m`：根据 Workpiece 局部坐标生成直线焊缝。
- `make_actual_gun_axis_pose.m`：根据实际焊丝轴线构造每个焊点的目标姿态。
- `generate_actual_axis_ik_candidates.m`：为实际焊丝轴线目标生成并筛选 IK 候选。
- `replan_weld_actual_axis.m`：基于实际焊丝轴线完成 Point 1–80 的连续重规划，并检查节点、边、焊枪和焊丝碰撞。
- `animate_weld_actual_axis_replan.m`：播放已保存的 Point 1–80 规划结果并生成多视角视频。
- `publish_weld_trajectory.m`：将已验收的轨迹发布到现有 `/joint_trajectory` 接口。
- `validate_weld_path.m`：检查机器人与环境碰撞及关节跳变。
- `inspect_workpiece_geometry.m`：接收 `scene` 并输出 Workpiece 及其碰撞代理的坐标、尺寸和姿态。
- `audit_current_weld_geometry.m`：只读核对当前场景几何和焊缝定义。
- `diagnose_weld_collisions.m`：通用碰撞诊断工具。

## 运行

在 MATLAB 中可从任意当前工作目录执行：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m')
```

ROS2 控制测试（脚本会持续运行，停止时使用 Ctrl+C）：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/test_ur5_ros2_control.m')
```

`sync_ur5_joint_states.m` 会自动从其所在目录定位上一级的：

```text
../scene_geometry.json
../ur5_tool_calibration.mat
```

工件检查函数的使用方式：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m')
wpSummary = inspect_workpiece_geometry(scene);
```

场景几何更新仍在上一级执行：

```bash
cd /home/ttbstt/jixiebi/UR5
PYTHONPATH=/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src \
python3 export_scene_geometry.py
```

当前实际焊丝轴线重规划：

```matlab
opts = struct('animate', false, 'publish', false, ...
    'outputFile', 'weld_actual_axis_replan.mat');
result = replan_weld_actual_axis(opts);
```

当前已验收结果保存在：

```text
weld_actual_axis_replan.mat
```

动画播放与视频输出示例：

```matlab
animate_weld_actual_axis_replan( ...
    '/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan.mat', ...
    StepDeg=1, Speed=100, SaveVideo=true, ...
    VideoFile='weld_actual_axis_replan_weld_inside.avi', ...
    View='allWeld');
```

旧 TCP+Z、多 IK、局部 corridor 和旧 RRT/IK 框架脚本及其 `.mat` 数据已清理；
临时 `smoke`、`pre5` 结果和旧场景同步归档已移入系统回收站，正式 benchmark、当前验收结果和历史记录仍保留在本目录及 `AGENT_HANDOFF.md`。
