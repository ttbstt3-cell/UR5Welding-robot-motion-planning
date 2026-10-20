# 源码副本说明

本目录是根据 2026-10-10 项目状态复制的参考源码快照，便于集中查看、归档和交接。

## 来源映射

```text
MATLAB/      ← /home/ttbstt/jixiebi/UR5/matlab仿真/
RRT/         ← /home/ttbstt/jixiebi/RRT/（只复制源代码，不复制 results/）
HMI/         ← /home/ttbstt/jixiebi/UR5/ur5_hmi/
ROS2/        ← /home/ttbstt/jixiebi/ros2_ws/src/ur5_controller/
场景几何/     ← UR5/ur5.ttt、scene_geometry.json、export_scene_geometry.py、
               ur5_tool_calibration.mat、规划文件/weld_anchor_points.json
```

## 副本内容

- `MATLAB/`：MATLAB 场景、焊缝规划、ACCP、RRT 接口、轨迹导出和诊断脚本；
- `RRT/`：关节空间规划器、采样/树操作/碰撞验证核心和 HMI MATLAB 入口；
- `HMI/ur5_hmi/`：Python HMI、ROS 状态、轨迹验证、安全门、校准和 3D 显示源码；
- `ROS2/ur5_controller/`：仿真用 `rclpy` 控制节点源码；
- `场景几何/`：CoppeliaSim 场景与 MATLAB 场景几何/标定输入参考。

本次副本规模：MATLAB 83 个文件、RRT 86 个源文件、HMI 41 个源文件、ROS2 控制包 8 个文件、
场景几何 5 个文件。

不复制 MATLAB `.mat/.csv` 实验结果、视频、HMI 截图和日志；这些仍保留在原工程目录。

## 使用规则

这里的文件是**参考副本，不是运行源**。运行和修改应针对原始路径；若确实修改了副本，
必须人工比对后再同步回原始工程，并更新 `AGENT_HANDOFF.md`。副本中的相对路径可能不适合
直接启动，尤其是 HMI、MATLAB `addpath` 和 RRT 结果目录。

复制时间：2026-10-10（Asia/Shanghai）。
