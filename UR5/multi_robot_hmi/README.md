# Multi-Robot Welding Planning & Control HMI

此目录是从原 `UR5/ur5_hmi` 复制出的独立版本。原 HMI 保留不变。本阶段加入 UR5/CR7 profile 选择、独立数据目录、CR7 只读关节状态和 3D 模型切换，以及轨迹 `robot_id` 强校验。

## 启动

```bash
source /opt/ros/humble/setup.bash
cd /home/ttbstt/jixiebi/UR5/multi_robot_hmi
python3 main.py
```

右上角选择 UR5 或 CR7。UR5 保留原有 HMI 安全门和人工确认流程；切换设备本身不发送运动命令。CR7 只订阅 `/cr7/joint_states` 更新关节表和 3D 模型，不创建控制 action client。CR7 真机 controller/action 尚未配置，Jog、执行、RealHome 采集和在线焊接规划均保持阻断。

## RobotProfile 与数据隔离

- `config/robots/ur5.yaml` 和 `config/robots/cr7.yaml` 是设备配置入口。
- RealHome、TCP 标定、工件标定、轨迹归档、规划运行和日志目录均按 profile 指向独立路径。
- 切换时若存在 motion goal 或 MATLAB 规划进程会拒绝切换；成功切换会停止预演、卸载轨迹、清除 Final Execution Check，再加载对应 URDF 和配置。不会自动运行或发送轨迹。
- MAT/YAML/JSON 轨迹必须带 `robot_id`，并与所选 profile 完全一致；缺失或不一致时加载结果为 BLOCKED。
- MATLAB 入口 `matlab/plan_hmi_robot_request.m` 按 `plan_request.robot_id` 路由到现有 UR5 焊接规划入口或 CR7 RRT-Connect 模型。CR7 目前只提供离线关节空间规划入口；HMI 的 CR7 在线焊接规划在路径与碰撞验收前保持关闭。

## 范围

本副本仅扩展设备选择、模型/规划模型解析、配置隔离和轨迹身份校验。开发验证不连接真机、不发送 Jog、轨迹或其他运动指令。`ur5_hmi` 原目录作为原版基线继续保留。
