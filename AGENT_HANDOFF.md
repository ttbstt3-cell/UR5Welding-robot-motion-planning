# UR5 CoppeliaSim / ROS 2 / MATLAB 项目交接文档

> **历史档案入口调整（2026-10-20）**：本文件保留完整历史，不删除旧记录。今后启动只读根目录`AGENTS.md`、`PROJECT_STATE.md`、`HANDOFF.md`；当前任务进度更新HANDOFF，不再在本文件重复追加。下方9月首页及早期README不代表当前架构；历史按日期/关键词定向检索。备份/恢复状态见`maintenance/BACKUP_WORKFLOW.md`与维护工具实查。

更新时间：2026-09-15（Asia/Shanghai）  
工作区：`/home/ttbstt/jixiebi`

## 快速状态（优先读取）

- 当前场景：[UR5/ur5.ttt](/home/ttbstt/jixiebi/UR5/ur5.ttt)，仿真停止。
- 当前架构：MATLAB → `/joint_trajectory` → 外部 `rclpy` → ZMQ Remote API → CoppeliaSim；CoppeliaSim 只发布 `/joint_states`。
- 不要恢复 CoppeliaSim Python `simROS2` subscriber，也不要继续改 `functional.lua`。
- 焊枪层级：`connection/WeldingGunMount/{WeldingGun_visual, GunCollision_*, WeldTCP}`。
- `WeldingGunMount` 相对 `connection`：位置 `[0,0,0]`，四元数 `[0,0,0,1]`。
- **当前冻结 TCP 基准**（相对 `WeldingGunMount`）：位置 `[-0.066774822, 0.019489205, 0.382330220] m`，四元数 `[0,0,0,1]`。这是已按 18 mm 作用点调整并完成数值验收的版本；不得自动拟合或擅自再次加入焊丝伸出量。
- 几何同步已完成：`UR5/scene_geometry.json`、`UR5/export_scene_geometry.py`、`UR5/load_scene_geometry.m`、`UR5/show_scene_geometry.m`。
- 当前阶段：已完成独立 WeldTCP 工作空间、逐点多 IK、前向/后向状态图和插值边碰撞可达性复核；整条焊缝连续状态图仍在中段断开，暂不实现 RRT、Bi-RRT/RRT*/平滑，也不接 D435i。
- 注意：MATLAB `universalUR5` 与当前 CoppeliaSim UR5 的关节运动学坐标尚未对齐；ROS 2 轨迹接口已能接受/执行，但实际 WeldTCP 沿焊缝尚未通过端到端验证。

本文用于将当前工作状态交接给后续 agent。除非用户明确要求回滚，否则不要覆盖当前场景或恢复已经放弃的 `simROS2` Python subscriber 方案。

## 变更同步规则

后续每一步都必须同步写入本文档，包括但不限于：

- CoppeliaSim 场景树、对象位姿、颜色、碰撞属性和 TCP 变更；
- ROS 2 节点、topic、控制器和启动方式变更；
- MATLAB 脚本、碰撞模型、规划参数和接口变更；
- 新建的备份、导出文件、预览图和验证结果。

每次变更至少记录：日期、修改内容、涉及文件、关键数值、验证方式和可回滚备份。若某次尝试被撤销，也要记录撤销原因和恢复到的基准版本。当前最新用户手动调整的 `WeldTCP` 是唯一基准，任何后续 agent 都必须先读取本文档再操作。

## 1. 当前目标与完成状态

项目目标是用 MATLAB 完成 UR5 的 IK、Bi-RRT/Bi-RRT*、路径平滑和时间参数化，再通过 ROS 2 控制 CoppeliaSim 中的 UR5，并为后续真机/MoveIt2 保留一致的控制接口。

当前已经完成：

- ROS 2 Humble 环境和 MATLAB ROS Toolbox 通信验证。
- MATLAB → `/joint_trajectory` → 外部 `rclpy` 控制节点 → CoppeliaSim ZMQ Remote API → UR5 执行链路验证。
- CoppeliaSim 仅发布 `/joint_states`，MATLAB 通过 callback 持续接收状态。
- 焊枪已安装到 UR5 `connection`，并建立 `WeldingGunMount`。
- 高面数焊枪 STL 与碰撞代理已分离。
- 焊枪视觉模型已改为深灰色。
- `WeldTCP` 已建立；当前采用用户人工调好的位置，不再使用网格推断或假定焊丝伸出量。

当前焊缝基线已完成。后续若用户确认，才通过现有 `/joint_trajectory` 接口执行；不要重新折腾 CoppeliaSim 内嵌 Python subscriber。RRT 系列算法留待后续阶段。

## 2. 环境与启动方式

- Ubuntu 22.04
- ROS 2 Humble：`/opt/ros/humble`
- CoppeliaSim 4.10：`/opt/coppeliaSim`
- MATLAB R2025b + ROS Toolbox
- 工作区：`/home/ttbstt/jixiebi`
- CoppeliaSim 场景：`/home/ttbstt/jixiebi/UR5/ur5.ttt`
- ZMQ Remote API 默认端口：`23000`
- Python Remote API 客户端目录：
  `/opt/coppeliaSim/programming/zmqRemoteApi/clients/python`

必须从已经 source ROS 2 的终端启动 CoppeliaSim，否则 `libsimROS2.so` 可能因找不到 ROS 运行库而加载失败：

```bash
cd /opt/coppeliaSim
source /opt/ros/humble/setup.bash
./coppeliaSim -f /home/ttbstt/jixiebi/UR5/ur5.ttt
```

Remote API Python 命令需要设置：

```bash
export PYTHONPATH=/opt/coppeliaSim/programming/zmqRemoteApi/clients/python
```

当前 CoppeliaSim 场景应保持停止状态再做对象编辑。保存场景必须显式传入文件名：

```python
sim.saveScene('/home/ttbstt/jixiebi/UR5/ur5.ttt')
```

`sim.saveScene()` 不带文件名只返回序列化 buffer，不会更新磁盘场景文件。

## 3. 当前 ROS 2 控制架构

当前采用的架构：

```text
MATLAB（路径规划 / IK / 优化）
        |
        | /joint_trajectory
        v
rclpy ur5_controller
        |
        | ZMQ Remote API
        v
CoppeliaSim UR5
        |
        | simROS2 publisher
        v
/joint_states  --> MATLAB callback
```

ROS 2 控制包：

```text
/home/ttbstt/jixiebi/ros2_ws/src/ur5_controller/
```

核心控制脚本：

[controller.py](/home/ttbstt/jixiebi/ros2_ws/src/ur5_controller/ur5_controller/controller.py)

该节点：

- 节点名：`ur5_controller`
- 订阅：`/joint_trajectory`，类型 `trajectory_msgs/msg/JointTrajectory`
- 默认以 50 Hz 插值并通过 ZMQ 设置 6 个 UR5 关节目标。
- 默认会发送一个 10 点、2 秒的 demo 轨迹。
- MATLAB 外部控制时使用 `send_demo:=false`，避免 demo 与 MATLAB 命令同时发送。

构建/运行：

```bash
cd /home/ttbstt/jixiebi/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 run ur5_controller controller --ros-args -p send_demo:=false
```

MATLAB 测试脚本：

[test_ur5_ros2_control.m](/home/ttbstt/jixiebi/UR5/test_ur5_ros2_control.m)

运行：

```matlab
run('/home/ttbstt/jixiebi/UR5/test_ur5_ros2_control.m')
```

曾验证的目标关节角：

```text
J1 =  10°
J2 = -30°
J3 =  45°
J4 = -30°
J5 =  20°
J6 =  10°
```

通信链路已成功使 UR5 到达该目标。后续 MATLAB 应直接发布 `JointTrajectory`，不要回到高频 `JointState` 目标发布。

## 4. 已放弃的 simROS2 Python subscriber 方案

早期尝试在 CoppeliaSim Python sandbox 中使用：

```python
simROS2.createSubscription(..., 'jointCommandCallback')
```

虽然 `simROS2` publisher 正常，`/joint_states` 可以被 MATLAB 收到，且 `/sim_ros2_interface` subscriber 存在，但订阅回调没有可靠执行，并出现：

```text
External call to simCallScriptFunction failed ('_S.reifiedFunctions....'):
error in script function.
```

即使把 callback 函数对象改成字符串，subscriber callback 调度仍不稳定。因此当前 CoppeliaSim 内嵌脚本只负责发布 `/joint_states`，控制订阅由外部 `rclpy` 节点承担。

不要重新加入 CoppeliaSim Python subscriber，也不要为了它继续修改 `functional.lua`。

`/opt/coppeliaSim/lua/functional.lua` 曾被做过兼容性修改，并且已有备份；它是全局安装文件，升级 CoppeliaSim 时可能被覆盖。

## 5. 当前 CoppeliaSim 场景树

关键层级如下：

```text
/UR5
  └─ connection
      └─ WeldingGunMount
          ├─ WeldingGun_visual
          ├─ GunCollision_mount
          ├─ GunCollision_body
          ├─ GunCollision_neck1
          ├─ GunCollision_neck2
          ├─ GunCollision_nozzle
          └─ WeldTCP

/Workpiece                 # 独立环境对象
```

`connection` 的对象句柄在重载场景后可能变化，不要硬编码句柄；应始终用对象路径查询。

### 5.1 Mount 与视觉模型

`WeldingGunMount` 相对 `connection`：

```text
position    = [0, 0, 0] m
quaternion  = [0, 0, 0, 1]
```

`WeldingGun_visual` 相对 `WeldingGunMount`：

```text
position    = [-0.07765445115395112, -0.03181499980192973, 0] m
quaternion  = [0, 0, 0, 1]
```

这个非零平移是正常的，因为 STL 原点不在真实安装法兰中心。不要把它改回 `[0,0,0]`。

视觉模型属性：

- alias：`WeldingGun_visual`
- 颜色：约 `[0.06, 0.06, 0.07]`
- 只保留 renderable 属性
- 不作为碰撞模型

### 5.2 碰撞代理

碰撞代理均为 `WeldingGunMount` 的直接子对象，保持静态、可碰撞、不可渲染。当前通过 `sim.setObjectSpecialProperty` 设置为 `objectspecialproperty_collidable`，并将 visibility layer 设为 `0`。它们用于显式碰撞检测，不用于动力学响应。

当前尺寸和局部位姿（单位：m）：

| 对象 | 类型 | 尺寸 `[x,y,z]` | 相对 Mount 位置 | 四元数 |
|---|---|---:|---:|---:|
| `GunCollision_mount` | cylinder | `[0.085, 0.085, 0.025]` | `[0, 0, 0.0125]` | `[0,0,0,1]` |
| `GunCollision_body` | cuboid | `[0.085, 0.090, 0.160]` | `[0, 0.130, 0.150]` | `[0,0,0,1]` |
| `GunCollision_neck1` | cylinder | `[0.070, 0.070, 0.140]` | `[-0.010, 0.130, 0.265]` | `[0,0,0,1]` |
| `GunCollision_neck2` | cylinder | `[0.058, 0.058, 0.105]` | `[-0.048, 0.055, 0.330]` | `[0,0,0,1]` |
| `GunCollision_nozzle` | cylinder | `[0.050, 0.050, 0.065]` | `[-0.048, 0.055, 0.375]` | `[0,0,0,1]` |

圆柱尺寸使用 `[diameter, diameter, length]` 记录。当前代理是保守包络，后续如果 MATLAB 碰撞过于保守，只能在确认真实几何后微调，不要擅自改动 Mount 或视觉 STL。

### 5.3 WeldTCP 当前位姿（最新用户人工调好的版本）

> **基准冻结：** 以下位姿是用户在 CoppeliaSim 中手动调好的最终基准。后续 MATLAB 工具变换、碰撞代理同步和路径规划必须使用它；不得用 STL 自动拟合结果覆盖，也不得擅自加入焊丝伸出量或旋转。

`WeldTCP` 相对 `WeldingGunMount` 的当前位姿：

```text
Position (m):
[-0.0603,
  0.0310,
  0.3701]

Position (mm):
[-60.300, 31.000, 370.100]

Quaternion [x,y,z,w]:
[0, 0, 0, 1]

Euler orientation [alpha,beta,gamma] (deg):
[0, 0, 0]
```

当前约定：

- 该位姿来自用户在 CoppeliaSim 中的人工调节，优先级高于网格几何推断。
- 不要在没有用户确认的情况下加入 10–15 mm 焊丝伸出量。
- TCP 原点暂按用户调好的喷嘴/焊丝作用点使用。
- TCP `+Z` 暂保持当前 Mount 坐标系方向；后续如需真实工具轴标定，应先让用户确认。
- `WeldingGunMount` 与 `connection` 零位姿，因此 `Mount → WeldTCP` 就是当前需要给 MATLAB 的固定工具变换。

不要根据 STL 的包络或自动拟合结果覆盖该人工位姿。真实焊丝伸出量确认后，再由用户决定是否沿工具轴修改。

## 6. 当前场景文件与备份

当前有效场景：

[ur5.ttt](/home/ttbstt/jixiebi/UR5/ur5.ttt)

关键备份：

- [ur5-before-rclpy-20260911.ttt](/home/ttbstt/jixiebi/UR5/ur5-before-rclpy-20260911.ttt)
- [ur5-before-weldinggun-zero-20260914.ttt](/home/ttbstt/jixiebi/UR5/ur5-before-weldinggun-zero-20260914.ttt)
- [ur5-before-weldinggun-mount-20260914.ttt](/home/ttbstt/jixiebi/UR5/ur5-before-weldinggun-mount-20260914.ttt)
- [ur5-before-weldinggun-translation-20260914.ttt](/home/ttbstt/jixiebi/UR5/ur5-before-weldinggun-translation-20260914.ttt)
- [ur5-before-gun-proxies-20260914.ttt](/home/ttbstt/jixiebi/UR5/ur5-before-gun-proxies-20260914.ttt)
- [ur5-before-weldtcp-calibration-20260914.ttt](/home/ttbstt/jixiebi/UR5/ur5-before-weldtcp-calibration-20260914.ttt)
- [ur5-before-weldtcp-restore-20260914.ttt](/home/ttbstt/jixiebi/UR5/ur5-before-weldtcp-restore-20260914.ttt)
- [ur5-before-latest-tcp-save-20260914.ttt](/home/ttbstt/jixiebi/UR5/ur5-before-latest-tcp-save-20260914.ttt)

TCP 标记预览图（红点是临时预览对象，不在场景中）：

[weldtcp_marker_preview.png](/home/ttbstt/jixiebi/UR5/weldtcp_marker_preview.png)

黑色焊枪预览：

[weldinggun_black_visual_closeup2.png](/home/ttbstt/jixiebi/UR5/weldinggun_black_visual_closeup2.png)

## 7. 常用检查命令

查询当前对象位姿：

```bash
PYTHONPATH=/opt/coppeliaSim/programming/zmqRemoteApi/clients/python python3 - <<'PY'
from zmqRemoteApi import RemoteAPIClient
import math

s = RemoteAPIClient('localhost', 23000).getObject('sim')
mount = s.getObject('/UR5/connection/WeldingGunMount')
tcp = s.getObject('/UR5/connection/WeldingGunMount/WeldTCP')

print('simulation state:', s.getSimulationState())
print('TCP position:', s.getObjectPosition(tcp, mount))
print('TCP quaternion:', s.getObjectQuaternion(tcp, mount))
print('TCP Euler deg:', [x * 180 / math.pi for x in s.getObjectOrientation(tcp, mount)])
PY
```

查询碰撞代理尺寸和位姿时，使用对象路径，不要使用固定句柄：

```python
names = [
    'GunCollision_mount',
    'GunCollision_body',
    'GunCollision_neck1',
    'GunCollision_neck2',
    'GunCollision_nozzle',
]
for name in names:
    obj = sim.getObject('/UR5/connection/WeldingGunMount/' + name)
    pose = sim.getObjectPose(obj, mount)
    size, bbPose = sim.getShapeBB(obj)
```

注意：不要通过 ZMQ 调用 `sim.getShapeMesh` 传输整个高面数 STL，之前会因数据量过大使 Remote API 卡住。需要网格统计时，使用 CoppeliaSim 进程内的临时 Lua customization script 调用 `sim.getShapeViz`，只返回统计量。

## 8. 后续 MATLAB 同步方案

后续导出脚本应自动读取：

- 5 个代理对象的 alias
- primitive 类型
- `getShapeBB` 尺寸
- 相对 `WeldingGunMount` 的位置和四元数
- `WeldTCP` 的位置和四元数

建议导出为 JSON 或 MATLAB `.m` 数据文件，避免人工抄写。例如 MATLAB 侧概念上：

```matlab
gunCollision.mount = collisionCylinder(...);
gunCollision.mount.Pose = T_mount;

gunCollision.body = collisionBox(...);
gunCollision.body.Pose = T_body;

gunCollision.neck1 = collisionCylinder(...);
gunCollision.neck1.Pose = T_neck1;
```

这些代理不是环境障碍物，而是机器人末端工具的一部分：

```text
UR5 links
  + WeldingGun collision proxies
        vs.
Workpiece / 船舱壁 / 其它环境障碍物
```

每个 Bi-RRT* 采样状态 `q` 都应通过：

```text
q
→ UR5 正运动学
→ flange pose
→ 固定 flange→WeldTCP / gun proxy 变换
→ 机器人和焊枪代理碰撞检测
```

## 9. 交接注意事项

1. 不要删除 `WeldingGunMount`，它是固定的法兰安装坐标系。
2. 不要把 `WeldingGun_visual` 的局部位置改回零；STL 原点不在安装法兰中心。
3. 不要让高面数视觉 STL重新参与 RRT 碰撞检测。
4. 不要在 CoppeliaSim Python sandbox 中恢复 `/ur5_joint_command` subscriber。
5. 不要继续修改全局 `functional.lua`，除非出现新的、可复现的兼容性问题并先建立备份。
6. 任何场景写操作前先复制当前 `ur5.ttt`；保存时使用显式文件名。
7. 代理碰撞检测当前使用 `collidable` 属性而非动态 `respondable` 接触；这是路径规划所需的设置。
8. 当前 WeldTCP 使用用户人工调好的位姿；不要擅自用 STL 拟合结果、12 mm 假设或自动旋转覆盖它。

## 10. 几何同步阶段（2026-09-15）

本阶段只完成几何同步和 MATLAB 碰撞模型，没有修改 `controller.py`、MATLAB ROS2 控制脚本或 CoppeliaSim 控制脚本，也没有实现 RRT。

### 新增文件

- [export_scene_geometry.py](/home/ttbstt/jixiebi/UR5/export_scene_geometry.py)：通过 CoppeliaSim ZMQ Remote API 读取场景真值并生成 JSON。
- [scene_geometry.json](/home/ttbstt/jixiebi/UR5/scene_geometry.json)：统一以 `UR5_base` 为参考帧的导出数据。
- [load_scene_geometry.m](/home/ttbstt/jixiebi/UR5/load_scene_geometry.m)：加载 JSON，建立 MATLAB `universalUR5`、固定 `WeldingGunMount` 和 5 个焊枪碰撞代理。
- [show_scene_geometry.m](/home/ttbstt/jixiebi/UR5/show_scene_geometry.m)：显示 UR5、焊枪碰撞模型、Workpiece 三个 MATLAB 碰撞盒和 WeldTCP 标记。

### 当前 JSON 内容

- `WeldTCP` 相对 `WeldingGunMount` 的最新人工位姿。
- 5 个 `GunCollision_*` 的类型、尺寸、Mount 相对位姿和 UR5_base 当前位姿。
- `Workpiece` 相对 UR5_base 的位姿：约 `[-0.075, 0.375, -0.01465] m`，姿态为单位四元数。
- Workpiece 包围盒：`0.235 × 0.235 × 0.235 m`。
- MATLAB-only Workpiece proxies：
  - `WorkpieceCollision_bottom`：`0.235 × 0.235 × 0.005 m` 底板；
  - `WorkpieceCollision_wall_x0`：`0.005 × 0.235 × 0.235 m`；
  - `WorkpieceCollision_wall_x1`：`0.005 × 0.235 × 0.235 m`。

当前 Workpiece 网格为简单 U 形三板结构，侧板位于 Workpiece 局部 `x=0` 和 `x=Lx`。这些碰撞盒只在 MATLAB 中创建，CoppeliaSim 没有新增 Workpiece 碰撞对象。

### 运行导出器

先确保 CoppeliaSim 从 ROS 2 sourced 终端启动并保持运行：

```bash
cd /opt/coppeliaSim
source /opt/ros/humble/setup.bash
./coppeliaSim -f /home/ttbstt/jixiebi/UR5/ur5.ttt
```

另开终端执行：

```bash
PYTHONPATH=/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src \
python3 /home/ttbstt/jixiebi/UR5/export_scene_geometry.py
```

### 运行 MATLAB 几何加载/显示

```bash
matlab -batch "addpath('/home/ttbstt/jixiebi/UR5'); run('/home/ttbstt/jixiebi/UR5/show_scene_geometry.m')"
```

已验证：

- `load_scene_geometry` 成功加载 `universalUR5`；
- 5 个焊枪碰撞代理已附加到固定 `WeldingGunMount` body；
- 3 个 Workpiece `collisionBox` 已转换到 `UR5_base`；
- `checkCollision(robot, zeros(1,6), scene.environment)` 返回 `[0 0]`（无自碰撞、无环境碰撞）。

注意：MATLAB 使用 `[qw qx qy qz]`，JSON/CoppeliaSim 使用 `[qx qy qz qw]`；转换已集中在 `load_scene_geometry.m` 的 `poseStructToTform` 中。

本阶段备份：

- [ur5-before-geometry-sync-20260914.ttt](/home/ttbstt/jixiebi/UR5/ur5-before-geometry-sync-20260914.ttt)

## 11. 直线角焊缝基线阶段（2026-09-15）

本阶段在不修改 CoppeliaSim 场景、ROS 2 控制包、现有 MATLAB 控制脚本的前提下，新增了“人工焊缝 → 固定 TCP 姿态 → 顺序 IK → 节点和插值碰撞检查 → 可选 JointTrajectory 发送”流程。没有实现 RRT，也没有接入 D435i。

### 新增 MATLAB 文件

- [generate_weld_seam.m](/home/ttbstt/jixiebi/UR5/generate_weld_seam.m)：从现有 MATLAB Workpiece 代理自动计算 `wall_x0` 与底板的内角交线；默认 80 个焊缝点，端部各留 20 mm，并在两端各增加 50 mm `PreWeld` / `PostWeld` 点。默认总路径点数为 82。
- [generate_weld_tcp_poses.m](/home/ttbstt/jixiebi/UR5/generate_weld_tcp_poses.m)：对整条路径固定 TCP 姿态。默认直接使用 `scene.weldTCPBaseCurrent` 的最新人工标定基座姿态，仅沿焊缝改变位置；IK 目标通过 `scene.weldTCP`（Mount→WeldTCP 固定变换）换算为 `WeldingGunMount` 目标。
- [solve_weld_path_ik.m](/home/ttbstt/jixiebi/UR5/solve_weld_path_ik.m)：调用 MATLAB `inverseKinematics`；第一点默认零位种子，后续点使用前一点解作为 seed；返回 `qPath`，尺寸 `N×6`，单位 rad。
- [check_weld_path_collision.m](/home/ttbstt/jixiebi/UR5/check_weld_path_collision.m)：检查 UR5 与 5 个焊枪代理相对于 Workpiece 的环境碰撞、机器人非父子刚体自碰撞；每段按默认最大 2° 关节步长插值检查，避免节点间漏碰撞。`SkippedSelfCollisions="parent"` 明确跳过关节处预期的父子接触。
- [run_weld_path_demo.m](/home/ttbstt/jixiebi/UR5/run_weld_path_demo.m)：串联加载、焊缝生成、TCP 姿态、IK、碰撞检查和可选动画；`Execute` 默认 false。
- [send_qpath_joint_trajectory.m](/home/ttbstt/jixiebi/UR5/send_qpath_joint_trajectory.m)：仅作为后续执行 helper，复用已稳定的 `/joint_trajectory` `trajectory_msgs/JointTrajectory` 接口；没有修改 `/home/ttbstt/jixiebi/ros2_ws/src/ur5_controller/ur5_controller/controller.py`。

### 关键参数和验证结果

- 选用焊缝：`WorkpieceCollision_wall_x0` 内侧面与 `WorkpieceCollision_bottom` 上表面的交线。
- 焊缝长度：约 `0.1950 m`；焊缝点数 `80`；总路径点数 `82`（PreWeld + 80 个焊缝点 + PostWeld）。
- Pre/Post 长度：各 `0.050 m`；端部避让 `EndMargin=0.020 m`。
- TCP 姿态：直接使用当前 `scene_geometry.json` 的 `welding_gun.weld_tcp.pose_base_current` 旋转；没有自动拟合、旋转或加入焊丝伸出量。当前人工 Mount→TCP 位姿仍为 `[-0.0603, 0.0310, 0.3701] m`、四元数 `[0,0,0,1]`。
- 插值碰撞步长：`MaxJointStep=deg2rad(2)`。
- MATLAB R2025b batch 验证：`IK success=82/82`；所有节点无碰撞；所有插值段无碰撞；最大相邻关节跳变 `0.194936 rad`（`11.169°`）；结果 `PASS`。

验证命令：

```bash
matlab -batch "addpath('/home/ttbstt/jixiebi/UR5'); warning('off','all'); out=run_weld_path_demo('Animate',false,'Execute',false); fprintf('PASS=%d\\n',out.pass);"
```

预期输出摘要：

```text
Weld points: 80 (plus PreWeld/PostWeld; total path points: 82)
IK success: 82/82
All nodes collision-free: 1
All interpolated segments collision-free: 1
RESULT: PASS
```

### 实现中修正的事项

初版用 MATLAB 零位的末端旋转重建 TCP 固定姿态，虽能求出 IK，但产生 UR5 自碰撞和约 `3π` 的关节翻转。已改为直接使用导出的最新人工 `pose_base_current` 旋转后重新验证，当前基线 PASS。不得把该修正改回零位重建，除非用户明确提供新的坐标标定。

### 当前未执行事项

- `run_weld_path_demo` 默认不发送轨迹；本节后续的受控执行验证已在 CoppeliaSim 中完成一次 8 秒轨迹执行。
- 但由于 MATLAB 内置 UR5 与 CoppeliaSim UR5 运动学不一致，实际 TCP 沿焊缝的端到端验证仍未完成。
- 未修改 `controller.py`、`test_ur5_ros2_control.m`、CoppeliaSim 场景或 `functional.lua`。
- 未实现 RRT、Bi-RRT、RRT*、Bi-RRT*、路径平滑、时间优化或 D435i。

### 受控执行验证（2026-09-15）

为验证现有控制接口，曾临时启动现有 controller：

```bash
source /opt/ros/humble/setup.bash
source /home/ttbstt/jixiebi/ros2_ws/install/setup.bash
ros2 run ur5_controller controller --ros-args -p send_demo:=false
```

通过 ZMQ Remote API 临时启动仿真，再运行：

```bash
matlab -batch "addpath('/home/ttbstt/jixiebi/UR5'); warning('off','all'); out=run_weld_path_demo('Animate',false,'Execute',true,'TotalExecutionTime',8.0); fprintf('PASS=%d sent=%d\\n',out.pass,out.execution.sent);"
```

结果：MATLAB 代理模型仍为 `IK 82/82`、节点和插值段均无碰撞、`RESULT: PASS`，并报告 `sent=1`；controller 日志显示接受 `82 points, duration 8.000 s`，随后 `Trajectory execution completed`。但是进一步从 CoppeliaSim 读取实际 `WeldTCP` 后发现，按现有 MATLAB `universalUR5` 模型求出的末态并未落在预期 `PostWeld` 点：实际 TCP 约为 `[-0.4874, -0.1596, 0.7736] m`，预期约为 `[-0.0700, 0.6400, -0.00965] m`。因此这次只能记为“ROS 2 轨迹发送/执行接口通过”，不能记为“实际 TCP 沿焊缝通过”。验证结束后已停止仿真并退出临时 controller；磁盘场景没有保存运行时关节姿态变化。

控制器日志中出现过若干 `sequence size exceeds remaining buffer` 的 Remote API 警告，但未阻止轨迹接受或完成；后续若需降低日志噪声，再单独排查，不能修改已稳定 topic/接口。

## 13. 用户删除本次新增 MATLAB 脚本（2026-09-15）

根据用户要求，已删除本次生成的 6 个 MATLAB 脚本：

- `UR5/generate_weld_seam.m`
- `UR5/generate_weld_tcp_poses.m`
- `UR5/solve_weld_path_ik.m`
- `UR5/check_weld_path_collision.m`
- `UR5/send_qpath_joint_trajectory.m`
- `UR5/run_weld_path_demo.m`

保留不变：`load_scene_geometry.m`、`show_scene_geometry.m`、`test_ur5_ros2_control.m`、`scene_geometry.json`、CoppeliaSim 场景、ROS 2 controller、`functional.lua` 以及本交接文档。之后不要再假设上述 6 个脚本存在，除非用户重新创建或要求恢复。

## 14. 用户复核后的同步问题优先级（2026-09-15）

用户复核现有文件后确认以下问题需要优先处理；本次只记录，不修改 MATLAB 文件：

1. `show_scene_geometry.m` 中写死 `q=zeros(1,6)`，没有读取当前 `/joint_states`，因此显示的 MATLAB UR5 不代表 CoppeliaSim 当前姿态。
2. `test_ur5_ros2_control.m` 的 callback 只打印关节角，没有按 `msg.name` 映射并保存给 MATLAB 模型。
3. `export_scene_geometry.py` 生成的 `matlab_mount_to_last_body_assumption=identity` 未经过验证；不能据此断言 MATLAB 最后一个 body 等于 CoppeliaSim `/UR5/connection`。应在相同关节状态下计算并验证 `last body → WeldingGunMount` 固定变换。
4. 当前 MATLAB `universalUR5` 与 CoppeliaSim UR5 的关节零位/连杆运动学已有实际 TCP 不一致证据。仅同步 q 数值只能验证关节消息，不足以保证末端位姿一致；必须先比较 q=0 和至少一个非零 q 的 FK。
5. `WeldingGun_visual` 目前只是 JSON 中的路径/位姿记录，没有导入 MATLAB 的 STL visual geometry；MATLAB 侧可见的主要是 UR5 自带 visual 和简化 collision proxy。
6. Workpiece 的 `bbox_pose_object` 与代理局部坐标假设需要单独核对，不能默认三块 `collisionBox` 就与原始 U 型 STL 完全重合。

碰撞体添加顺序暂不认定为根因；先执行：

```matlab
body = getBody(scene.robot, "WeldingGunMount");
numel(body.Collisions)
```

若结果为 5，说明碰撞体已经挂载，添加顺序不是当前主要问题。

## 15. 旧场景同步脚本归档（2026-09-15）

根据用户要求，旧场景同步文件没有永久删除，而是移动到：

```text
/home/ttbstt/jixiebi/UR5/archive_old_scene_sync/
```

归档内容：

- `export_scene_geometry.py`
- `load_scene_geometry.m`
- `scene_geometry.json`
- `show_scene_geometry.m`

保留在 `UR5/` 根目录的稳定控制测试脚本：

- `test_ur5_ros2_control.m`

后续重构按三层进行：

1. 只同步 UR5 六关节 `/joint_states`，验证 MATLAB `universalUR5` 与 CoppeliaSim FK；
2. 再单独同步 `WeldingGunMount/WeldTCP`；
3. 最后拆分导出工具几何和环境几何，分别使用 `tool_geometry.json` 与 `environment_geometry.json`。

本次只完成归档，没有创建新的同步脚本，也没有修改 ROS 2 控制链路、CoppeliaSim 场景或 `test_ur5_ros2_control.m`。

## 16. 新增 UR5 关节状态同步脚本（2026-09-15）

根据用户提供的最小方案，新建：

```text
/home/ttbstt/jixiebi/UR5/sync_ur5_joint_states.m
```

脚本功能：

- 加载 MATLAB `universalUR5`；
- 订阅 `/joint_states`；
- 根据 `msg.name` 建立六个 UR5 关节的名称映射；
- 按映射读取关节角并实时显示 MATLAB UR5；
- 不发送控制命令，不修改 CoppeliaSim 场景，不修改 ROS 2 controller。

MATLAB `checkcode` 语法检查已通过。运行前需启动 CoppeliaSim 并确保 `/joint_states` 正在发布：

```matlab
run('/home/ttbstt/jixiebi/UR5/sync_ur5_joint_states.m')
```

## 17. WeldingGunMount / WeldTCP 帧读取（2026-09-15）

新增只读脚本：

```text
/home/ttbstt/jixiebi/UR5/read_tool_frames.py
```

运行时读取 `/UR5/connection`、`WeldingGunMount` 和 `WeldTCP` 的相对位姿，不修改场景。当前读取结果为：

```text
connection relative base
position = [-0.1850380406, -0.0000001774, 0.9865366852]
quaternion [qx qy qz qw] = [-0.0000000382, 0.7071376509, 0.0000000577, -0.7070759101]

mount relative connection
position = [0.0, 0.0, 0.0]
quaternion [qx qy qz qw] = [0.0, 0.0, 0.0, 1.0]

TCP relative mount
position = [-0.0603000000, 0.0310000000, 0.3701000000]
quaternion [qx qy qz qw] = [0.0, 0.0, 0.0, 1.0]

TCP relative base
position = [-0.5551327732, 0.0309998411, 0.9262043745]
quaternion [qx qy qz qw] = [-0.0000000382, 0.7071376509, 0.0000000577, -0.7070759101]
```

再次只读执行 `read_tool_frames.py` 时，CoppeliaSim 当前六关节实际仍接近零位：

```text
q ≈ [0, 0, 0, 0, 0, 0] deg
TCP relative base position = [-0.5551327732, 0.0309998411, 0.9262043745]
quaternion [qx qy qz qw] ≈ [-0.0000000382, 0.7071376509, 0.0000000577, -0.7070759101]
```

该位置与 MATLAB 零位 TCP 位置一致；四元数与 `[0,-0.7071,0,0.7071]` 只差整体符号，代表同一旋转。因此本次没有取得用户指定的非零 `qCop≈[0,30,20,0,0,0]°` 对照数据；需要在 CoppeliaSim 保持该非零姿态后再次运行读取脚本。

## 18. 非零姿态 TCP 数值验收（2026-09-15）

为排除仿真运行时关节控制器回写造成的误差，先通过 ZMQ 停止 CoppeliaSim 仿真，在停止状态下直接设置 3 组 CoppeliaSim 关节角，并读取每组关节回读值及 `WeldTCP` 相对 `/UR5` 的位姿。关节回读均与目标一致。MATLAB 预测值使用当前 `sync_ur5_joint_states.m` 中的固定标定关系：

```text
qMat = qCop + [180, -90, 0, -90, 0, 0] deg
T_CopBase_TCP = T_CopBase_MatBase * T_MatBase_WeldTCP
```

姿态误差使用四元数绝对点积计算，忽略四元数整体正负号等价性。

| case | qCop [deg] | MATLAB 预测位置 [m] | CoppeliaSim 实际位置 [m] | 位置误差 | 姿态误差 |
|---|---|---|---|---:|---:|
| 1 | `[0, 30, 20, 0, 0, 0]` | `[-0.5551327733, 0.5592939342, 0.6930970405]` | `[-0.5550500820, 0.5592710949, 0.6931634814]` | `0.109 mm` | `0.005°` |
| 2 | `[-20, 10, -15, 25, 10, -20]` | `[-0.5133103049, 0.2182252692, 0.9392927429]` | `[-0.5131786946, 0.2184030889, 0.9392762956]` | `0.222 mm` | `0.005°` |
| 3 | `[35, -20, 30, -10, -25, 15]` | `[-0.4868056288, -0.1891821546, 0.8886426531]` | `[-0.4865496145, -0.1895740986, 0.8886873249]` | `0.470 mm` | `0.005°` |

结论：当前 3 组非零姿态均满足毫米级位置一致性和极小姿态误差，`WeldTCP` 标定层通过验收，可以冻结 `qOffset`、`T_CopBase_MatBase`、`T_tool0_mount` 和 `T_mount_tcp`，后续不要再修改这些标定量，除非重新执行同样的数值验收。

本次验收没有修改 CoppeliaSim 场景、ROS 2 控制代码或用户的 `sync_ur5_joint_states.m`。测试结束后已将 CoppeliaSim 六个关节恢复为零位；仿真保持停止状态。

## 19. 从当前 CoppeliaSim 场景重新导出几何（2026-09-15）

根目录原先没有导出脚本；旧版 `export_scene_geometry.py` 和旧版 JSON 位于 `archive_old_scene_sync/`，本次没有直接复用旧脚本或旧 JSON。新建当前场景专用导出器：

```text
/home/ttbstt/jixiebi/UR5/export_scene_geometry.py
```

该脚本只读 CoppeliaSim 当前场景，不创建或修改任何场景对象，导出：

- 5 个当前 `GunCollision_*` 的类型、尺寸、WeldingGunMount 相对位姿和 UR5_base 相对位姿；
- 当前 `WeldTCP` 的 mount/base 位姿；
- `WeldingGun_visual` 的当前位姿；
- `Workpiece` 的 base/world 位姿、网格边界和包围盒信息；
- 根据当前 Workpiece 网格边界生成的 MATLAB-only 底板/两侧板碰撞盒。

执行命令：

```bash
cd /home/ttbstt/jixiebi/UR5
PYTHONPATH=/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src \
python3 export_scene_geometry.py
```

已生成新文件：

```text
/home/ttbstt/jixiebi/UR5/scene_geometry.json
```

导出结果摘要：

```text
schema_version = 2
reference_frame = UR5_base
gun proxies = 5
workpiece proxies = 3
WeldTCP relative mount = [-0.0603, 0.0310, 0.3701] m, identity quaternion
Workpiece relative UR5_base = [-0.0750, 0.3750, -0.01465] m, identity quaternion
```

固定的 `qOffset`、`T_CopBase_MatBase`、`T_tool0_mount`、`T_mount_tcp` 没有写入或覆盖；它们仍由 `ur5_tool_calibration.mat` / `sync_ur5_joint_states.m` 管理。

## 20. MATLAB JSON cell/struct 兼容修复（2026-09-15）

当前 MATLAB `jsondecode` 的实际结果为：

```text
gunSpecs = cell(1,5)
wpSpecs  = struct(1,3)
```

因此在以下文件中修复了两个代理循环：

```text
/home/ttbstt/jixiebi/UR5/sync_ur5_joint_states.m
```

循环现在兼容两种容器：

```matlab
if iscell(specs)
    spec = specs{k};
else
    spec = specs(k);
end
```

已对焊枪代理和工件代理分别应用，并打印实际容器类型。MATLAB 短时运行验证通过：

```text
gunSpecs class = cell
1..5 GunCollision_* 成功创建
WeldingGunMount collision count = 5

wpSpecs class = struct
1..3 WorkpieceCollision_* 成功创建
Environment collision object count = 3

[FREE] 当前零位 Robot/Gun 与 Workpiece 无碰撞。
PLANNING SCENE READY
```

本次只修复 JSON 容器访问，不修改 ROS 2 控制链路、CoppeliaSim 场景或冻结的 TCP 标定参数。

## 21. 焊枪碰撞代理需要在 CoppeliaSim 重新拟合（2026-09-15）

当前 `scene_geometry.json` 中 5 个焊枪碰撞代理的数据读取和 MATLAB 导入均正常，但其几何定义仍不符合真实黑色焊枪外形：

- `GunCollision_mount`、`body`、`neck1`、`neck2`、`nozzle` 当前导出的局部四元数均为 `[0,0,0,1]`；
- 因此三个圆柱代理在 MATLAB 中保持平行，没有表达真实焊枪枪颈和喷嘴的弯曲方向；
- 当前代理位置是人工初始估计值，MATLAB 只是忠实复现，没有额外旋转或扭曲；
- `visibility_layer = 0` 说明这些代理在 CoppeliaSim 中被隐藏，当前视图看到的是黑色 visual STL，不是碰撞代理。

结论：不修改已冻结的 TCP 标定、MATLAB 导入流程或 ROS 2 控制链路。下一步应在 CoppeliaSim 中临时显示 5 个 `GunCollision_*`，以黑色焊枪为参照重新调整每个代理的尺寸、位置和局部旋转，重点保证：

1. `GunCollision_mount` 覆盖安装法兰；
2. `GunCollision_body` 覆盖焊枪主体最粗区域；
3. `GunCollision_neck1`、`neck2` 沿真实弯曲枪颈方向布置；
4. `GunCollision_nozzle` 覆盖喷嘴并靠近 `WeldTCP`；
5. 代理略大于真实外轮廓，保留碰撞安全裕度。

调整完成后重新运行：

```bash
cd /home/ttbstt/jixiebi/UR5
PYTHONPATH=/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src \
python3 export_scene_geometry.py
```

再用 MATLAB 重新加载 JSON 验证代理姿态；在代理拟合完成前不进入 RRT。

## 22. 焊枪碰撞模型拟改为整体 Convex Hull（2026-09-15）

为避免 5 个人工 box/cylinder 代理方向拟合不准确，计划将焊枪碰撞模型简化为：

```text
WeldingGunMount
├─ WeldingGun_visual   # 真实 STL，仅显示
├─ GunCollisionHull    # 单个整体凸包，仅碰撞
└─ WeldTCP
```

当前阶段不删除原有 `GunCollision_*`，先保留为回退方案。必须先在 CoppeliaSim 中复制 visual mesh、生成凸包、命名为 `GunCollisionHull` 并挂到 `WeldingGunMount`，确认凸包完整包住黑色焊枪且不改变 `WeldTCP` 位姿。

凸包对象创建并验证后，再改造：

1. `export_scene_geometry.py`：读取 `GunCollisionHull` 的局部 mesh vertices/faces 和相对 Mount 位姿；导出三角面索引时转换为 MATLAB 的 1-based 索引；
2. `sync_ur5_joint_states.m`：优先创建 `collisionMesh(vertices,faces)` 并挂到 `WeldingGunMount`；若 JSON 暂无 hull，则保留当前 5 代理回退路径；
3. 重新导出 `scene_geometry.json`，验证 MATLAB 凸包和 CoppeliaSim 凸包位姿一致。

在 `GunCollisionHull` 尚未出现在当前场景前，不修改现有稳定的 5 代理加载逻辑，也不进入 RRT。

## 23. GunCollisionHull 已创建并接入 MATLAB（2026-09-15）

用户已在当前 CoppeliaSim 场景创建：

```text
/UR5/connection/WeldingGunMount/GunCollisionHull
```

只读检查结果：

```text
handle = 52
pose relative WeldingGunMount = [-0.07765445, -0.03181500, 0] m
mesh vertices = 104
mesh indices = 576 -> 192 triangular faces
```

已更新 `/home/ttbstt/jixiebi/UR5/export_scene_geometry.py`：

- 读取当前 `GunCollisionHull` mesh；
- 导出 `vertices_m` 为 N×3 顶点；
- 导出 `faces` 为 MATLAB 1-based M×3 三角面索引；
- 导出 hull 相对 Mount/base 的位姿；
- 保留旧 5 个 `GunCollision_*` 记录作为 JSON 回退信息。

重新执行导出后，`scene_geometry.json` 中新增：

```text
data.welding_gun.collision_hull
type = convex_hull
vertex_count = 104
face_count = 192
```

已更新 `/home/ttbstt/jixiebi/UR5/sync_ur5_joint_states.m`：

- 若 JSON 存在 `collision_hull`，优先使用整体凸包；
- 使用 MATLAB R2025b 实际支持的 `collisionMesh(vertices)` 接口；
- JSON 中的 faces 保留用于审计，但不作为第二个构造参数；
- 若无 hull 字段，回退到原 5 个 box/cylinder 代理；
- `scene.gunCollisionMode` 记录当前模式。

MATLAB 短时运行验证通过：

```text
GunCollisionHull: 104 vertices, 192 triangular faces
WeldingGunMount collision count = 1
Environment collision object count = 3
[FREE] 当前零位 Robot/Gun 与 Workpiece 无碰撞。
PLANNING SCENE READY
```

本次没有修改 ROS 2 controller、CoppeliaSim 对象位姿、UR5/TCP 冻结标定参数或旧 5 个代理对象。

## 24. 修正 GunCollisionHull 的复合组件处理（2026-09-15）

复查当前场景后发现：`GunCollisionHull` 并不是单一 mesh，而是 CoppeliaSim 的 4 组件 convex compound：

```text
sim.getShapeGeomInfo(GunCollisionHull) = (5, 0, ...)
result bit 0 = compound
result bit 2 = convex
shapeintparam_component_cnt = 4
```

之前若把 `getShapeMesh` 返回的全部顶点一次性传给 MATLAB `collisionMesh(vertices)`，会把 4 个组件重新包成一个更大的凸包，不能忠实表示当前 CoppeliaSim 几何。

已修正：

- `export_scene_geometry.py` 现在通过 `getIntArrayProperty(shape, "meshes")` 读取每个组件；
- JSON 的 `collision_hull` 增加 `type = convex_compound`、`component_count = 4` 和 `components[]`；
- 每个组件分别导出自己的 `vertices_m`、`faces`（faces 为 MATLAB 1-based）；
- `sync_ur5_joint_states.m` 现在逐组件创建 `collisionMesh(component.vertices_m)`，每个组件使用同一个 hull pose 添加到 `WeldingGunMount`；
- 不再将所有组件顶点合并为单一 MATLAB 凸包。

当前重新导出结果：

```text
GunCollisionHull type = convex_compound
components = 4
combined vertices = 104
combined triangular faces = 192
```

MATLAB 验证：

```text
GunCollisionHull: 4 components, 104 vertices, 192 triangular faces
WeldingGunMount collision count = 4
Environment collision object count = 3
[FREE] 当前零位 Robot/Gun 与 Workpiece 无碰撞。
PLANNING SCENE READY
```

因此当前场景实际采用的是 4 个凸组件，属于合理的 convex decomposition 形式；它比错误地合并成一个大凸包更准确。ROS 2 控制链路、场景位姿和 TCP 标定均未修改。

## 25. 工件碰撞代理由 3 块更新为 4 块（2026-09-15）

用户要求不在 MATLAB 中手动补板，而是继续由 CoppeliaSim/导出 JSON 提供工件几何来源。已备份：

```text
/home/ttbstt/jixiebi/UR5/export_scene_geometry.py.bak-20260915-workpiece3
/home/ttbstt/jixiebi/UR5/scene_geometry.json.bak-20260915-workpiece3
/home/ttbstt/jixiebi/UR5/matlab仿真/_backup_before_reorg/sync_ur5_joint_states.m.bak-20260915-workpiece3
```

已修改 `/home/ttbstt/jixiebi/UR5/export_scene_geometry.py`：

- `make_workpiece_proxies()` 由 3 个 box 改为 4 个 box；
- 现有三块保持为 `bottom`、`wall_x0`、`wall_x1`；
- 新增 `WorkpieceCollision_wall_y1`，尺寸为 `[Lx, thickness, height]`，中心为 `[Lx/2, Ly-thickness/2, height/2]`；
- 简化模型约定开口方向为 `y=0`，并将该约定写入 JSON 的 `geometry_description` 与 `collision_proxy_open_side`；
- 当前场景已经没有旧的五个 `GunCollision_*` 对象，导出器改为“存在则导出、缺失则跳过”，不影响新的 `GunCollisionHull`。

已重新生成 `/home/ttbstt/jixiebi/UR5/scene_geometry.json`。验收结果：

```text
Workpiece proxies: 4
1. WorkpieceCollision_bottom
2. WorkpieceCollision_wall_x0
3. WorkpieceCollision_wall_x1
4. WorkpieceCollision_wall_y1
Gun hull: 104 vertices, 192 triangular faces
Gun hull type: convex_compound, components: 4
```

MATLAB `jsondecode` 读取该 JSON 已确认 `numel(d.workpiece.collision_proxies) == 4`。`sync_ur5_joint_states.m` 仅更新了数量注释和提示（预期 4），仍按 JSON 数量自动创建环境碰撞体，没有在 MATLAB 中手动补板。ROS2 controller、CoppeliaSim 场景对象位姿、UR5/TCP 标定均未修改。

注意：当前实际 CoppeliaSim Workpiece mesh 的可见边界可能包含更多表面；本次按用户指定的“四块简化模型”采用 `bottom + x0 + x1 + y1`，如果后续确认应保留 `y0` 而不是 `y1`，只需在导出器中替换该代理名称和中心坐标。

## 26. MATLAB 文件整理到 `UR5/matlab仿真`（2026-09-15）

用户要求将 UR5 MATLAB 代码集中管理，同时保持 ROS2 控制链路不变。已将以下 6 个活动 `.m` 文件从 `UR5` 根目录移动到：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/
```

```text
build_weld_seam.m
main_welding_rrt.m
solve_weld_ik.m
sync_ur5_joint_states.m
test_ur5_ros2_control.m
validate_weld_path.m
```

同时新增：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/README.md
```

整理前的 MATLAB 副本位于：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/_backup_before_reorg/
```

路径调整：

- `sync_ur5_joint_states.m` 不再写死 `/home/ttbstt/jixiebi/UR5`，通过 `mfilename("fullpath")` 定位自身目录，再用上一级目录读取 `scene_geometry.json` 和 `ur5_tool_calibration.mat`；
- `main_welding_rrt.m` 自动把自身目录加入 MATLAB path，并调用同目录的 `sync_ur5_joint_states.m`；删除了对不存在的旧 `main_ur5_planning_scene.m` 的硬编码引用；
- `build_weld_seam.m`、`solve_weld_ik.m`、`validate_weld_path.m` 由 `main_welding_rrt.m` 通过同目录 path 调用；
- `test_ur5_ros2_control.m` 的 ROS2 topic 和消息接口未修改；
- `scene_geometry.json`、`ur5_tool_calibration.mat`、`export_scene_geometry.py`、`read_tool_frames.py`、`ur5.ttt` 保留在 `UR5` 根目录。

验证：MATLAB Code Analyzer 对 6 个活动脚本/函数未发现语法错误；从 `matlab仿真` 路径解析上一级 JSON 和标定文件的检查通过。

## 27. 复核 `sync_ur5_joint_states.m` 正确版本（2026-09-15）

用户误改后请求恢复。已对比当前活动文件与整理前备份：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m
/home/ttbstt/jixiebi/UR5/matlab仿真/_backup_before_reorg/sync_ur5_joint_states.m
```

当前活动文件与此前整理后的正确版本一致，差异仅为已确认的两项：

- 从 `mfilename("fullpath")` 定位 `matlab仿真`，再读取上一级 JSON/标定文件；
- 工件碰撞代理数量说明为 4。

没有发现用户误改的其它代码。MATLAB Code Analyzer 复核通过（无语法错误）。当前应使用：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m
```

## 12. 新发现：MATLAB UR5 与 CoppeliaSim UR5 的运动学坐标不一致（2026-09-15）

受控执行后的实际 TCP 核对表明，现有 `load_scene_geometry.m` 使用的 MATLAB 内置 `universalUR5` 与当前 CoppeliaSim UR5 的关节零位/连杆坐标约定不一致。不能仅凭 MATLAB 代理模型 PASS 就宣称 CoppeliaSim 中的 WeldTCP 已沿焊缝运动。

通过 ZMQ 在 CoppeliaSim 关节为 0 时读取到的串联相对变换（单位 m；各旋转为相邻关节帧的固定旋转）为：

```text
joint1: p=[ 0.000000, 0, 0.008500], R=I
joint2: p=[-0.070300, 0, 0.066050], R=Ry(-90°)
joint3: p=[ 0.425100, 0, 0       ], R=I
joint4: p=[ 0.392150, 0, 0       ], R=I
joint5: p=[ 0.045575, 0, 0.039700], R=Ry(+90°)
joint6: p=[ 0.014425, 0, 0.049175], R=Ry(-90°)
connection: p=[0,0,0.089375], R=I
```

各 CoppeliaSim revolute joint 的局部转轴为 local Z。用这些变换临时构造的 MATLAB `rigidBodyTree` 能让 MATLAB IK 的 TCP 目标落在 `[-0.0700, y, -0.00965]` 的焊缝线上；但该临时模型尚未整合进项目，也尚未完成可靠的 UR5 连杆碰撞代理。因此后续 agent 应先完成“CoppeliaSim 真值运动学模型 + 碰撞几何”的正式整合，再恢复实际轨迹执行验证。

## 29. 再次统一 UR5 MATLAB 文件位置（2026-09-15）

复核发现用户最新版本的 `sync_ur5_joint_states.m` 和 `inspect_workpiece_geometry.m` 曾被放回 `UR5` 根目录。已将它们原样移动到：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/
```

当前活动 MATLAB 文件共 7 个：

```text
build_weld_seam.m
inspect_workpiece_geometry.m
main_welding_rrt.m
solve_weld_ik.m
sync_ur5_joint_states.m
test_ur5_ros2_control.m
validate_weld_path.m
```

`UR5` 根目录已无活动 `.m` 文件。移动前的用户版本备份为：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/_backup_before_reorg/sync_ur5_joint_states.root-before-move-20260915.m
/home/ttbstt/jixiebi/UR5/matlab仿真/_backup_before_reorg/inspect_workpiece_geometry.root-before-move-20260915.m
```

路径验证通过：移动后的 `sync_ur5_joint_states.m` 能从自身目录定位上一级的 `scene_geometry.json` 与 `ur5_tool_calibration.mat`；`inspect_workpiece_geometry.m` 能定位同目录的场景脚本；7 个活动 MATLAB 文件均可通过 `which` 找到。ROS2 控制代码、CoppeliaSim 场景和 Python 导出器未修改。

## 30. 修复最新同步脚本未识别 `GunCollisionHull`（2026-09-15）

用户运行日志显示，最新的 `sync_ur5_joint_states.m` 能正确读取 JSON 和 4 个工件代理，但输出焊枪碰撞数量为 0。原因是该版本只查找：

```text
welding_gun.collision_meshes
welding_gun.collision_proxies
```

而当前 `scene_geometry.json` 的焊枪碰撞几何位于：

```text
welding_gun.collision_hull
```

已备份：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/_backup_before_reorg/sync_ur5_joint_states.before-hull-fix-20260915.m
```

已修改 `/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m`：

- 优先识别 `collision_hull`；
- 对 `type=convex_compound` 的 4 个 `components` 分别创建 MATLAB collision mesh；
- 每个组件继承 hull 的 `pose_mount`；
- 支持 `convex_hull` / `convex_mesh` 类型；
- 对不同 MATLAB 版本兼容 `collisionMesh(vertices,faces)` 与 `collisionMesh(vertices)`，当前版本自动采用后者。

MATLAB 实际运行验证通过：

```text
Welding-gun collision objects = 4 (convex_compound)
WeldingGunMount collision count = 4
Workpiece collision objects = 4
Initial collision = 0
```

旧报错中对 `/home/ttbstt/jixiebi/UR5/main_ur5_planning_scene.m` 的调用来自移动前的旧 `main_welding_rrt.m`。当前正确入口是：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/main_welding_rrt.m')
```

若当前 MATLAB 会话仍命中旧文件，先执行：

```matlab
clear main_welding_rrt sync_ur5_joint_states
rehash path
addpath('/home/ttbstt/jixiebi/UR5/matlab仿真')
which -all main_welding_rrt
which -all sync_ur5_joint_states
```

确认 `which` 只指向 `UR5/matlab仿真/`。ROS2 控制链路、CoppeliaSim 场景、JSON 和冻结标定未修改。

## 31. 清理 `collisionMesh(vertices,faces)` 警告（2026-09-15）

用户运行同步场景后，功能已成功但每个 GunCollisionHull 组件都出现：

```text
collisionMesh(vertices,faces) 不可用，改用 vertices-only
```

原因是当前 MATLAB 版本的 `collisionMesh` 只接受 `collisionMesh(vertices)`。已修改：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m
```

现在直接对每个凸组件调用 `collisionMesh(vertices)`；JSON 中的 `faces` 仍保留在 `scene.data` 中用于审计和索引校验，不再触发无效的双参数调用。

重新运行验证无警告：

```text
Welding-gun collision objects = 4 (convex_compound)
WeldingGunMount collision count = 4
Workpiece collision objects = 4
Initial collision = 0
```

## 32. 恢复 MATLAB 场景显示窗口（2026-09-15）

用户运行 `sync_ur5_joint_states.m` 后只看到 `PLANNING SCENE READY`，没有图形窗口。原因是当前同步脚本只构建 `scene` 和碰撞模型，未包含 `figure/show` 绘图代码；旧的 `show_scene_geometry.m` 位于归档目录且依赖旧接口，不能直接复用。

已新增：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/show_scene_geometry.m
```

该脚本会调用同目录 `sync_ur5_joint_states.m`，并显示：

- MATLAB `universalUR5` 可视模型；
- `GunCollisionHull` 的 4 个 convex compound 组件；
- JSON 中的 4 个 Workpiece collisionBox；
- 当前 `qMat0` 下的 `WeldTCP` 标记。

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/show_scene_geometry.m')
```

MATLAB 实际验证通过：`Gun collision objects = 4`、`Environment objects = 4`、`Initial collision = 0`，显示脚本 Code Analyzer 无消息。`sync_ur5_joint_states.m` 保持为场景构建脚本，不再强制打开窗口。若 MATLAB 以 `-nodisplay` 或纯 batch 模式启动，图形窗口仍不会在桌面显示，应在正常 MATLAB Desktop 会话中运行上述命令。

## 33. 将显示功能重新整合进 `sync_ur5_joint_states.m`（2026-09-15）

用户确认希望恢复原先行为：直接运行 `sync_ur5_joint_states.m` 就显示场景。已在：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m
```

加入显示段，显示当前 `qMat0` 下的：

- UR5 可视模型；
- GunCollisionHull 的 4 个 convex compound 组件；
- Workpiece 的 4 个碰撞代理；
- WeldTCP 红色标记及坐标标签。

`show_scene_geometry.m` 已改为兼容入口，只调用同步脚本一次，避免重复创建第二个窗口。直接运行：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m')
```

MATLAB 实际验证得到 `figures = 1`、`Gun collision objects = 4`、`Environment objects = 4`、`Initial collision = 0`。显示功能不会修改 ROS2 控制或 CoppeliaSim 场景；在 `-nodisplay`/纯 batch 会话中仍无法看到桌面窗口。

## 34. 恢复 V-HACD 凸组件版同步脚本（2026-09-15）

用户明确要求恢复之前带 V-HACD/GunCollisionHull 凸块和原始显示逻辑的版本。已从整理前备份恢复：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/_backup_before_reorg/sync_ur5_joint_states.m
```

恢复后的活动文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m
```

仅将旧的根目录硬编码路径改为从 `mfilename("fullpath")` 定位上一级 `UR5` 目录；V-HACD 凸组件加载和原始显示逻辑保持不变。当前 JSON 中：

```text
GunCollisionHull: 4 components, 104 vertices, 192 triangular faces
Gun collision proxies: 4
Workpiece proxies: 4
Initial collision: 0
```

独立的 `matlab仿真/show_scene_geometry.m` 已删除，显示功能只保留在 `sync_ur5_joint_states.m`。当前 MATLAB 直接运行同步脚本即可得到原来的规划场景窗口。

## 35. 整理 `inspect_workpiece_geometry.m` 为函数并移除根目录 Shadow 文件（2026-09-15）

检查发现两个版本：

- `UR5/inspect_workpiece_geometry.m`：正确函数，首行为 `function summary = inspect_workpiece_geometry(scene)`；
- `UR5/matlab仿真/inspect_workpiece_geometry.m`：旧脚本，会执行 `clear/clc/run`，不能作为函数调用。

已将正确函数原样迁移到：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/inspect_workpiece_geometry.m
```

并删除 `UR5/inspect_workpiece_geometry.m` 根目录副本；旧脚本和函数副本均已放入：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/_backup_before_reorg/
```

验证通过：

```text
which -all inspect_workpiece_geometry
只指向 UR5/matlab仿真/inspect_workpiece_geometry.m
nargin('inspect_workpiece_geometry') = 1
summary_count = 4
```

调用方式：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m')
wpSummary = inspect_workpiece_geometry(scene);
```

## 36. 焊缝几何测试改为复用唯一主 Figure（2026-09-15）

用户指出 `test_weld_seam_geometry.m` 重新创建 Figure 并再次 `show` UR5/Workpiece，导致颜色、光照、材质和工件立体感与主场景不一致。已备份原文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/_backup_before_reorg/test_weld_seam_geometry.before-overlay-fix-20260915.m
```

已修改：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/test_weld_seam_geometry.m
```

## 42. 新增末端翻枪四姿态基线测试（2026-09-15）

根据当前焊枪碰撞主要集中在枪体方向、且希望优先利用第 6 轴绕工具轴翻转的要求，新增：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/test_weld_roll_orientation.m
```

该脚本固定实际焊缝：

```matlab
pStartWP = [0.015 0.230 0.005];
pEndWP   = [0.220 0.230 0.005];
```

并固定 `+Z_TCP` 工作轴，只测试 TCP 局部 Z 轴的：

```text
roll = 0°, 90°, 180°, 270°
```

每个候选姿态均重新调用 `solve_weld_ik`，不直接修改 `q(6)`；随后调用 `validate_weld_path` 检查 80 个节点的碰撞和关节跳变，并输出 q6 范围。结果保存在 MATLAB 变量 `results` 中。

静态检查通过：

```text
test_weld_roll_orientation messages=0
```

运行前需先重新打开已更新的 `ur5.ttt`，然后执行：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/test_weld_roll_orientation.m')
```

该测试不修改 `optimize_weld_orientation.m`，不修改 ROS2 控制链路，也不直接改变场景关节角。

现在该脚本：

- 继续调用 `sync_ur5_joint_states.m` 建立主场景；
- 使用 `fig = gcf`、`ax = gca` 获取同步脚本刚创建的 Figure/axes；
- 不再重新 `figure`、`axes`、`show(robot)` 或 `show(environment)`；
- 只叠加 80 点红色焊缝、绿色起点、蓝色终点和 RGB TCP 坐标轴；
- 不重新改变主场景的视角、坐标轴、光照或材质。

MATLAB 实际运行验证：

```text
SEAM_OVERLAY_VERIFY figures=1 lines=80
```

所以后续焊缝、RRT 树和规划路径都应继续叠加到 `sync_ur5_joint_states.m` 创建的唯一主 Figure 上。

## 37. 修复焊枪姿态搜索 fprintf 兼容性（2026-09-15）

用户在运行 `optimize_weld_orientation.m` 时遇到 MATLAB 报错“文件标识符无效”。原因是 `fprintf` 的格式字符串使用了双引号 string 数组和方括号拼接，部分 MATLAB 版本会将其错误解析为文件标识符。

已备份原文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/_backup_before_reorg/optimize_weld_orientation.before-fprintf-fix-20260915.m
```

已修改：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/optimize_weld_orientation.m
```

将 `NEW BEST` 输出改为标准 character vector 格式：先构造 `fmt = ['...' ...]`，再调用 `fprintf(fmt, ...)`。搜索算法、姿态候选范围、IK 和碰撞验证逻辑均未修改。

静态检查通过：

```text
checkcode messages=0
```

重新加载并运行：

```matlab
clear optimize_weld_orientation
rehash path

qSeed = qWeld(1,:);
bestOrientation = optimize_weld_orientation( ...
    scene, ...
    pStartWP, ...
    pEndWP, ...
    qSeed);
```

如果 `scene`、`qWeld`、`pStartWP` 和 `pEndWP` 仍在 MATLAB 工作区，不需要重复加载场景或重新求 IK。该修复不涉及 ROS2 控制链路。

## 38. 将 18 mm TCP standoff 纳入焊缝轨迹（2026-09-15）

用户确认实际焊接头距离几何焊缝约 18 mm。此前 `build_weld_seam.m` 把几何焊缝点直接作为 TCP 目标点，会使焊枪碰撞模型扎入 Workpiece。已按以下关系修正：

```text
P_seam = P_TCP + standoff * Z_TCP
P_TCP  = P_seam - standoff * Z_TCP
```

已备份原文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/_backup_before_reorg/build_weld_seam.before-standoff-20260915.m
```

已修改：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/build_weld_seam.m
```

主要变化：

- 新增可选参数 `standoff_m`，默认 `0.018` m；
- 新增 `seam.positionsSeamWP`：真实焊缝点；
- 新增 `seam.positionsTCPWP`：退让后的 TCP 目标点；
- 新增 `seam.positionsSeamMat`：真实焊缝在 MATLAB base 中的坐标；
- `seam.positionsMat` 保持表示 TCP 在 MATLAB base 中的轨迹；
- 新增 `seam.standoff_m`、`seam.zTCP_WP`、`pStartTCPWP`、`pEndTCPWP`；
- `seam.positionsWP` 保留为历史兼容字段，指向 TCP 目标点；
- `seam.tcpPoses` 改为使用 `positionsTCPWP` 生成，IK 不再把几何焊缝当作 TCP 位置。

同时更新并备份了以下显示脚本，使红线表示真实焊缝、蓝色虚线表示 TCP 退让轨迹：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/test_weld_seam_geometry.m
/home/ttbstt/jixiebi/UR5/matlab仿真/test_weld_ik_path.m
/home/ttbstt/jixiebi/UR5/matlab仿真/main_welding_rrt.m
```

备份文件分别带有 `before-standoff-20260915` 后缀。初版实现的静态检查和最小数值测试均通过；后续默认值调整见第 39 节。`T_mount_tcp`、ROS2 控制代码和 CoppeliaSim 场景未修改。

## 39. standoff 默认值与 IK 测试调用方式调整（2026-09-15）

根据后续要求，为保持 `build_weld_seam` 的历史调用兼容性，默认 `standoff_m` 已从 `0.018` 调整为 `0`。因此未显式传入 standoff 的旧脚本不会改变原有轨迹。

实际焊接 IK 测试已显式设置：

```matlab
tcpStandoff = 0.018;   % 18 mm
seam = build_weld_seam( ...
    scene, pStartWP, pEndWP, RtcpWP, numPoints, tcpStandoff);
```

已应用到：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/test_weld_ik_path.m
/home/ttbstt/jixiebi/UR5/matlab仿真/test_weld_seam_geometry.m
```

`build_weld_seam` 还新增 `seam.positionsTCPMat`，并保留 `seam.positionsMat` 作为同一 TCP 轨迹的兼容别名；新增 `seam.standoff` 作为 `standoff_m` 的兼容别名。对 `RtcpWP(:,3)` 现在会自动归一化，仅拒绝零向量。

本次 follow-up 的备份文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/_backup_before_reorg/build_weld_seam.before-default-standoff-adjustment-20260915.m
/home/ttbstt/jixiebi/UR5/matlab仿真/_backup_before_reorg/test_weld_ik_path.before-default-standoff-adjustment-20260915.m
/home/ttbstt/jixiebi/UR5/matlab仿真/_backup_before_reorg/test_weld_seam_geometry.before-default-standoff-adjustment-20260915.m
```

## 40. WeldTCP 重定义待场景手动确认（2026-09-15）

用户计划将 CoppeliaSim 中的 `WeldTCP` 从当前喷嘴参考位置移动到真正的焊接作用点，并新增 `NozzleTip` 作为喷嘴端部参考。该场景改动尚未执行；当前远程读取仍为：

```text
TCP relative mount
position = [-0.0603, 0.0310, 0.3701] m
quaternion = [0, 0, 0, 1]
```

因此目前仍保留旧的 `ur5_tool_calibration.mat` 中的 `T_mount_tcp`，也没有把 MATLAB 测试脚本的显式 `tcpStandoff = 0.018` 提前改成 0。待用户沿焊枪自身轴完成 `NozzleTip`/`WeldTCP` 调整并保存场景后，应重新运行 `read_tool_frames.py`，只更新 `T_mount_tcp`，再进行 2～3 个非零姿态的 TCP 数值验收；验收通过后才将焊缝测试的 standoff 设为 0。

## 41. 将 WeldTCP 调整为实际焊接作用点（2026-09-15）

检查已保存的原始场景发现，用户手动调整后的 TCP 相对旧参考点位移为 `0.025019992` m（约 25.02 mm），不是目标 18 mm。根据该实际手动移动方向，将 TCP 缩放到精确 18 mm 后生成临时场景并验证：

```text
TCP before = [-0.069300000, 0.015000000, 0.387100000]
TCP after  = [-0.066774822, 0.019489205, 0.382330220]
displacement = 0.018000000 m
quaternion relative WeldingGunMount = [0, 0, 0, 1]
```

已备份并替换正式 CoppeliaSim 场景：

```text
/home/ttbstt/jixiebi/UR5/ur5.ttt
/home/ttbstt/jixiebi/UR5/ur5.ttt.before-tcp-18mm-20260915.ttt
```

已重新导出当前场景几何 JSON（通过临时 Remote API 端口 23002）：

```text
/home/ttbstt/jixiebi/UR5/scene_geometry.json
/home/ttbstt/jixiebi/UR5/scene_geometry.json.before-tcp-18mm-20260915.json
```

JSON 中 `WeldTCP pose_mount` 与新标定一致：`[-0.066774822, 0.019489205, 0.382330220]`。

已更新并备份 MATLAB 标定：

```text
/home/ttbstt/jixiebi/UR5/ur5_tool_calibration.mat
/home/ttbstt/jixiebi/UR5/ur5_tool_calibration.mat.before-tcp-18mm-20260915.mat
```

仅更新了 `T_mount_tcp` 的平移，`qOffset`、`T_CopBase_MatBase`、`T_tool0_mount` 未改。由于原 GUI CoppeliaSim 进程在文件替换前已加载旧场景，必须关闭该旧实例时选择“不保存”，再重新打开 `/home/ttbstt/jixiebi/UR5/ur5.ttt`，避免旧内存场景覆盖新文件。

WeldTCP 已按当前定义作为实际焊接作用点，因此以下测试脚本的 `tcpStandoff` 已设为 `0`：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/test_weld_ik_path.m
/home/ttbstt/jixiebi/UR5/matlab仿真/test_weld_seam_geometry.m
```

## 43. 新增整圈工具轴 roll 搜索脚本（2026-09-15）

新增：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/search_weld_roll.m
```

脚本固定当前 205 mm 实际焊缝、45°工作角、TCP 位置和 `tcpStandoff = 0`，只测试 TCP 局部 Z 轴的 `0:15:345` 度，共 24 个候选姿态。每个候选均重新调用 `solve_weld_ik` 和 `validate_weld_path`，不直接修改 `q(6)`。

输出包括：

- 每个 roll 的 IK 失败数；
- 80 个焊缝节点中的碰撞点数；
- 最大关节跳变；
- 最大 TCP 位置误差；
- 最优 roll 及 q6 起点/中点/终点；
- 最优候选的 `diagnose_weld_collisions` 详细诊断。

结果变量和文件：

```text
MATLAB 工作区：results 不适用；主要结果变量为 resultTable、bestRollDeg、bestQWeld、bestReport
保存文件：/home/ttbstt/jixiebi/UR5/matlab仿真/weld_roll_search_result.mat
```

静态检查通过：

```text
search_weld_roll messages=0
```

运行：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/search_weld_roll.m')
```

运行前需关闭旧的 CoppeliaSim 场景实例（不保存），重新打开已更新的 `ur5.ttt`。该脚本不修改 ROS2 控制链路、RRT 或 `optimize_weld_orientation.m`。

## 44. 完成整圈 roll 搜索（2026-09-16）

已在最新 CoppeliaSim 场景运行：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/search_weld_roll.m')
```

运行前已确认：

- CoppeliaSim 使用更新后的 `ur5.ttt`；
- `WeldTCP relative mount = [-0.066774822, 0.019489205, 0.382330220]`；
- 仿真正在运行并持续发布 `/joint_states`；
- ROS2 daemon 已重启。

搜索结果：

```text
候选数：24（0:15:345 deg）
IK 失败：0 / 24
TCP 最大位置误差：约 0.00001～0.00003 mm
```

碰撞点数最少的候选为：

```text
roll = 120 deg : collision = 59 / 80
roll = 300 deg : collision = 59 / 80
```

按最大关节跳变作为第二优先级，脚本选择：

```text
Best roll = 300 deg
Collision points = 59 / 80
Maximum joint jump = 0.4649 deg
q6: start = 21.691 deg, middle = 8.567 deg, end = -3.860 deg
```

最佳候选的详细诊断为：

```text
WeldingGunMount <--> WorkpieceCollision_wall_x1 : 59 / 80
起点无碰撞；中点和终点碰撞 wall_x1
```

结论：单纯固定 roll 尚未实现整条焊缝无碰撞；180°并非当前最佳。结果文件已保存：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_roll_search_result.mat
```

该结果支持下一步只在 `120°` 和 `300°` 附近做局部细搜索，或检查 `wall_x1` 代理/枪体包络；暂不进入动态 roll、RRT 或多 IK 分支。

## 45. 逐焊点 roll 可行性分析（2026-09-16）

为判断固定 roll 造成的碰撞是否可以通过沿焊缝逐点改变 roll 消除，新增：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/analyze_weld_roll_feasibility.m
```

脚本复用 `sync_ur5_joint_states.m` 建立的唯一规划场景，固定当前焊缝和 TCP 姿态，测试 `0:15:345 deg` 共 24 个 roll。每个候选均执行连续 IK 和完整碰撞检查，并生成 `feasibleMap`（80 个焊点 × 24 个 roll），同时保存 `qBank`、`seamBank`、`ikBank`、`reportBank`、`ikValidMap`。

静态检查：

```text
analyze_weld_roll_feasibility messages=0
```

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/analyze_weld_roll_feasibility.m')
```

本次运行结果：

```text
Weld points                 = 80
Roll candidates per point   = 24
Points with >=1 free roll   = 42 / 80
Points with NO free roll    = 38 / 80
Minimum free rolls at any point = 0
Maximum free rolls at any point = 1
```

各固定 roll 的自由点数中，只有 `120 deg` 和 `300 deg` 各有 `21/80` 个无碰撞点，其余候选均为 `0/80`。起点只有 `300 deg` 可行，中点（第 40 点）没有可行 roll，终点只有 `120 deg` 可行。无可行 roll 的点为第 `22:59` 点，共 38 点。

结果文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_roll_feasibility.mat
```

结论：当前 V-HACD/凸分解焊枪代理与 `wall_x1` 的几何关系导致中段 38 个焊点在当前 15° roll 采样下没有任何无碰撞姿态。因此不能直接声称“动态 roll”即可解决；下一步应优先检查 `wall_x1` 代理尺寸/位置与焊枪包络，或扩大姿态搜索（更细 roll、工作角/工具位姿），仍暂不进入 RRT。

## 46. roll 5° 加密复验（2026-09-16）

按上一阶段建议，先对 `analyze_weld_roll_feasibility.m` 做 5° roll 加密，不增加 travel angle、工作角或 RRT。修改前已备份：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/analyze_weld_roll_feasibility.m.before-roll5deg-20260916.m.bak
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_roll_feasibility.mat.before-roll5deg-20260916.bak
```

当前脚本候选改为：

```matlab
rollCandidatesDeg = 0:5:355;
```

静态检查：

```text
analyze_weld_roll_feasibility messages=0
```

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/analyze_weld_roll_feasibility.m')
```

运行结果：

```text
Testing 72 roll orientations
Weld points                 = 80
Roll candidates per point   = 72
Points with >=1 free roll   = 42 / 80
Points with NO free roll    = 38 / 80
Minimum free rolls at any point = 0
Maximum free rolls at any point = 4
```

起点可行 roll 为 `295, 300, 305, 310 deg`；第 40 个焊点仍无可行 roll；终点可行 roll 为 `115, 120, 125, 130 deg`。无解区仍是第 `22:59` 点，共 38 点。细化后可行 roll 窗口从约 15°采样下的单点扩大为约 20°窗口，但没有增加可行焊点数量。

结果已覆盖保存至：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_roll_feasibility.mat
```

结论：5° roll 加密没有改变全局结论，中段仍存在连续无解区。因此不再继续单独细化 roll；下一步可进入 `roll + travel angle` 二维可行性分析，或先检查 `wall_x1` / V-HACD 凸分解代理是否过于保守。ROS2 控制链路未修改，仍不进入 RRT。

## 47. roll + travel angle 二维可行性分析（2026-09-16）

根据 5° roll 复验结果，新增并运行：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/analyze_weld_orientation_feasibility.m
```

该脚本固定当前 45° 工作角、当前焊缝、`tcpStandoff = 0`，搜索：

```matlab
travelCandidatesDeg = -20:5:20;   % 9 个
rollCandidatesDeg = 0:15:345;     % 24 个
```

共 `9 × 24 = 216` 个固定姿态组合。每个组合使用同一 `qSeed = scene.qMat0`，进行连续 IK、焊缝节点碰撞检测，并保存三维可行矩阵：

```text
feasibleMap(point, travel, roll)
```

静态检查结果：

```text
analyze_weld_orientation_feasibility messages=0
```

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/analyze_weld_orientation_feasibility.m')
```

本次结果：

```text
Orientations tested                  = 216
IK failures                          = 0 / 216
Points with >=1 feasible orientation = 80 / 80
Points with NO feasible orientation  = 0 / 80
Minimum feasible orientations/point  = 10
Maximum feasible orientations/point  = 28
```

关键焊点可行姿态数量：

- 第 1 点：28 个；
- 第 40 点：13 个；
- 第 80 点：28 个。

说明 `roll + travel` 二维姿态空间已覆盖每个焊点至少一个无碰撞姿态。但这只是逐点可行性，不代表直接逐点选最优姿态就能形成连续轨迹；姿态切换仍需后续连续性/动态规划筛选。

固定二维姿态中碰撞最少的组合：

```text
Travel angle      = -20 deg
Roll angle        = 150 deg
Collision points  = 31 / 80
Max joint jump    = 0.6167 deg
```

该固定姿态的碰撞来源：

```text
WeldingGunMount <--> WorkpieceCollision_wall_x0 : 24 / 80
WeldingGunMount <--> WorkpieceCollision_wall_x1 :  7 / 80
```

结果文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_orientation_feasibility.mat
```

脚本生成两幅诊断图：每个焊点可行姿态数量曲线，以及第 40 点的 roll-travel 可行域图。ROS2 控制链路、CoppeliaSim 场景和现有 RRT 代码均未修改。下一步可基于该可行矩阵做连续姿态动态规划；在此之前仍应注意当前 `validate_weld_path.m` 的碰撞统计是焊缝节点级检查，尚未加入相邻关节状态插值碰撞检查。

## 48. 动态 roll + travel 轨迹规划首次运行（2026-09-16）

新增：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/plan_dynamic_weld_orientation.m
```

脚本直接读取 `weld_orientation_feasibility.mat`，为每个焊点建立可行状态集合，并按三组连续性约束进行动态规划：

```text
strict : |Δroll|<=15°, |Δtravel|<=5°,  |Δq|max<=8°
medium : |Δroll|<=30°, |Δtravel|<=10°, |Δq|max<=15°
loose  : |Δroll|<=45°, |Δtravel|<=15°, |Δq|max<=25°
```

脚本还包含：多圈关节角展开、节点碰撞复验、相邻节点 1° 关节步长插值碰撞检查、TCP 位置/姿态误差计算，以及主场景轨迹叠加代码。

静态检查：

```text
plan_dynamic_weld_orientation messages=0
```

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/plan_dynamic_weld_orientation.m')
```

运行时 80 个焊点均成功建立了可行状态集合（每点 10～28 个状态），但三组 profile 均无法把第 1 点连接到第 80 点：

```text
Trying profile: strict  -> No complete path
Trying profile: medium  -> No complete path
Trying profile: loose   -> No complete path
```

因此本次没有生成 `dynamic_weld_orientation_plan.mat`，也没有进入节点/边最终验证和轨迹执行。

基于已保存的候选库做的独立连通性检查显示：即使放宽 roll/travel 步长限制，保留 `|Δq|max<=25°` 时仍在约第 57 个焊点处断开；完全取消关节步长限制后图才连通。说明当前问题主要不是姿态可行域不足，而是每个固定姿态候选只有单一 IK 分支，跨候选状态在中后段出现关节分支跳变。下一步应优先增加同一 TCP 姿态的多 IK 候选并纳入状态图，或对候选姿态网格/IK seed 做扩展；不能直接把大关节跳变轨迹送入 ROS2 执行。ROS2 控制链路和 CoppeliaSim 场景未修改。

## 49. 第 55～60 点多初值 IK 分支测试（2026-09-16）

新增多初值 IK 候选生成器：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/generate_ik_candidates.m
```

新增局部测试脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/test_multi_ik_near_point57.m
```

`generate_ik_candidates` 对同一 TCP 位姿使用：

```text
当前已有解 + scene.qMat0 + 32 个固定随机初值 = 34 个 seeds
```

并按 TCP 位置误差 `<=0.1 mm`、姿态误差 `<=0.1°` 筛选，按 2π 等价关节角去重，再进行环境碰撞过滤。两个新文件的 Code Analyzer 结果均为 0 条消息。

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/test_multi_ik_near_point57.m')
```

测试结果：

```text
Point 55 total free IK states = 70   (11 poses, max 7 branches/pose)
Point 56 total free IK states = 70   (11 poses, max 7 branches/pose)
Point 57 total free IK states = 68   (10 poses, max 7 branches/pose)
Point 58 total free IK states = 68   (10 poses, max 7 branches/pose)
Point 59 total free IK states = 68   (10 poses, max 7 branches/pose)
Point 60 total free IK states = 73   (11 poses, max 7 branches/pose)
```

每个测试姿态均得到：

```text
raw = 34
unique = 8
free = 4～7（随姿态变化）
```

结论：第 57 点附近同一个 TCP 姿态确实存在多组不同的无碰撞 IK 构型，验证了“单一 IK 分支覆盖不足”的判断。下一步应把多 IK 候选扩展到整条 80 点焊缝，建立状态 `S_i^k=(travel, roll, q_i^k)`，并在建边时直接加入关节插值碰撞检查，再做动态规划。当前没有修改 `plan_dynamic_weld_orientation.m`、ROS2 控制链路或 CoppeliaSim 场景。

## 50. 完整 80 点多 IK 状态库（2026-09-16）

新增并运行：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/build_weld_multi_ik_state_graph.m
```

脚本只生成状态库，不连接相邻点、不运行动态规划、不运行 RRT。它读取 `weld_orientation_feasibility.mat` 中 `feasibleMap=true` 的姿态，对每个姿态使用 34 个 IK 初值（已有解、`scene.qMat0`、32 个固定随机种子），保留满足 TCP 误差阈值且无碰撞的多分支结果。

脚本支持逐点 checkpoint。当前生成的 checkpoint：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_multi_ik_state_graph_checkpoint.mat
```

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/build_weld_multi_ik_state_graph.m')
```

本次 80 点完整运行成功：

```text
Weld points              = 80
Minimum states/point     = 36
Maximum states/point     = 175
Mean states/point        = 69.49
Total states             = 5559
```

断点附近状态数量与局部测试一致：

```text
Point 55 : 70 states
Point 56 : 70 states
Point 57 : 68 states
Point 58 : 68 states
Point 59 : 68 states
Point 60 : 73 states
```

最终状态库：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_multi_ik_state_graph.mat
```

每个 `stateBank{i}` 包含：`q`、`travelIdx`、`rollIdx`、`travelDeg`、`rollDeg`、`localBranch` 和 `Ttarget`。下一步可基于该状态库重写动态规划，将节点无碰撞、roll/travel 连续性、IK 分支连续性以及相邻状态的插值边碰撞统一放入建边过程。本次未修改原 `plan_dynamic_weld_orientation.m`、ROS2 控制链路或 CoppeliaSim 场景。

## 51. 多 IK 状态图动态规划首次运行（2026-09-16）

新增并运行：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/plan_dynamic_weld_multi_ik.m
```

脚本读取完整的 5,559 状态库，按照三组连续性 profile 建边；只有通过 roll/travel/关节步长约束且相邻状态之间 1° 插值碰撞检查通过的边才进入动态规划。

静态检查：

```text
plan_dynamic_weld_multi_ik messages=0
```

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/plan_dynamic_weld_multi_ik.m')
```

结果：三组 profile 均未连通到第 80 点：

```text
strict : 在 weld point 54 断开
medium : 在 weld point 57 断开
loose  : 在 weld point 57 断开
```

候选边和碰撞统计：

```text
strict : tested 1518, free 1518
medium : tested 1975, free 1975
loose  : tested 1983, free 1979
```

说明断开主要由关节连续性/IK 分支连接造成，而不是插值碰撞；loose profile 仅发现少量边碰撞。此次未生成 `dynamic_weld_multi_ik_plan.mat`，因为没有完整状态路径。下一步需要在断点附近扩充姿态候选（包括原 `feasibleMap=false` 但可能有其它无碰撞 IK 分支的 216 个姿态），或增加更密的 travel/roll 网格；不能取消关节连续性约束后直接执行。ROS2 控制链路、原动态规划脚本和 CoppeliaSim 场景均未修改。

## 52. 第 52～60 点局部姿态重新判定与多 IK 扩展（2026-09-16）

新增并运行：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/expand_weld_multi_ik_states_local.m
```

脚本只重新测试局部点 `52:60` 中原 `feasibleMap=false` 的姿态，使用 32 个随机 IK seed，并将找到的新无碰撞分支与原状态合并。原始状态库未覆盖，扩展结果另存为：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_multi_ik_state_graph_expanded.mat
```

静态检查：

```text
expand_weld_multi_ik_states_local messages=0
```

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/expand_weld_multi_ik_states_local.m')
```

结果：

```text
Original total states = 5559
Expanded total states = 5663
New states added      = 104
```

局部状态数量变化：

```text
Point 52 :  76 ->  83 (+ 7), rescued poses = 1
Point 53 :  73 ->  85 (+12), rescued poses = 2
Point 54 :  70 ->  82 (+12), rescued poses = 2
Point 55 :  70 ->  82 (+12), rescued poses = 2
Point 56 :  70 ->  82 (+12), rescued poses = 2
Point 57 :  68 ->  80 (+12), rescued poses = 2
Point 58 :  68 ->  80 (+12), rescued poses = 2
Point 59 :  68 ->  80 (+12), rescued poses = 2
Point 60 :  73 ->  86 (+13), rescued poses = 2
```

主要被救回的姿态是 `travel=-20°, roll=105°` 和 `travel=-5°, roll=120°` 附近；第 57 点由 68 个状态增至 80 个。下一步应复制动态规划脚本为 expanded 版本，仅替换输入状态文件，保持算法和 profile 不变，以判断这 104 个补充状态能否恢复全程连通。原 `weld_multi_ik_state_graph.mat`、原动态规划脚本、ROS2 控制链路和 CoppeliaSim 场景均未修改。

## 53. Expanded 多 IK 状态库动态规划复验（2026-09-16）

按要求复制：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/plan_dynamic_weld_multi_ik.m
→ /home/ttbstt/jixiebi/UR5/matlab仿真/plan_dynamic_weld_multi_ik_expanded.m
```

expanded 脚本只修改了状态库输入和结果输出文件名：

```matlab
weld_multi_ik_state_graph_expanded.mat
dynamic_weld_multi_ik_plan_expanded.mat
```

`profiles`、权重、`edgeResolutionDeg`、`maxRollStepDeg`、`maxTravelStepDeg`、`maxJointStepDeg`、`localPlanMultiIK` 和边碰撞检测逻辑均未修改。

静态检查：

```text
plan_dynamic_weld_multi_ik_expanded messages=0
```

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/plan_dynamic_weld_multi_ik_expanded.m')
```

结果：扩展状态库共有 `5663` 个状态（原来 `5559` 个，新增 `104` 个），但仍未找到完整路径：

```text
strict : disconnect at weld point 54
medium : disconnect at weld point 57
loose  : disconnect at weld point 57
```

断点附近 reachable states：

```text
strict : point 52 = 4/83,  point 53 = 2/85,  point 54 无完整路径
medium : point 52 = 11/83, point 53 = 7/85, point 54 = 4/82,
         point 55 = 4/82, point 56 = 4/82, point 57 无完整路径
loose  : point 52 = 11/83, point 53 = 7/85, point 54 = 4/82,
         point 55 = 4/82, point 56 = 4/82, point 57 无完整路径
```

边统计：

```text
strict : tested 1518, free 1518
medium : tested 1975, free 1975
loose  : tested 1983, free 1979
```

没有生成 `dynamic_weld_multi_ik_plan_expanded.mat`，原因是三个 profile 都没有完整状态路径。结论是：新增 104 个局部状态虽增加了候选数量，但不足以恢复第 54～57 点的图连通性；严格和中等 profile 的断开不是边碰撞导致，主要仍是相邻层之间的 IK 分支/关节连续性连接不足。下一步应做第 `53↔54`、`56↔57` 的跨层最小关节差分析，或在断点局部扩充全部姿态候选/插入更密焊缝点。未修改 ROS2 控制链路、CoppeliaSim 场景和原始规划结果。

## 54. 断开层跨状态最小关节差诊断（2026-09-16）

新增并运行：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_weld_state_disconnect.m
```

该脚本加载 `weld_multi_ik_state_graph_expanded.mat`，使用与原多 IK 规划器完全一致的 strict/medium/loose 姿态步长和关节步长约束，只传播状态连续性，不重复执行边碰撞检测、不发送 ROS2 命令。静态检查通过，运行命令为：

```matlab
checkcode('/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_weld_state_disconnect.m')
run('/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_weld_state_disconnect.m')
```

输出诊断文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_state_disconnect_diagnosis.mat
```

结果：

```text
strict : 实际可达状态在 53 -> 54 断开
medium : 实际可达状态在 56 -> 57 断开
loose  : 实际可达状态在 56 -> 57 断开
```

关键 A/B/C 结果如下（单位为 deg）：

```text
53 -> 54, strict/medium/loose：
A reachable prev + pose limits : No candidate pair exists
B all prev + pose limits       : min max|dq| = 0.327525
                                  dq = [-0.2178  0.0204  0.2674 -0.0432 -0.0056 -0.3275]
C reachable prev + no pose     : min max|dq| = 39.117338
                                  dq = [-5.4931 4.5126 -4.3834 3.4932 2.9457 39.1173]

56 -> 57, medium/loose：
A reachable prev + pose limits : No candidate pair exists
B all prev + pose limits       : min max|dq| = 0.315480
                                  dq = [-0.2097  0.0622  0.2663 -0.0928 0.0028 -0.3155]
C reachable prev + no pose     : min max|dq| = 75.067764
                                  dq = [-47.7084 2.4940 3.1368 41.3826 57.8498 75.0678]
```

两处最小差值的主导关节均为 `J6`。但 B 结果表明：在上一层全部状态中确实存在几乎连续的对应分支（同为 `travel=-20°, roll=105°`，关节最大差仅约 `0.32°`），这些状态已经在更早层的可达传播中丢失。对实际可达状态取消 roll/travel 限制后，53→54 仍需约 `39.1°` 的最大关节跳变，56→57 仍需约 `75.1°`，均主要由 J6 造成；因此问题不是单纯的姿态网格步长，也不是边碰撞，而是早期状态图选择/可达分支被截断，叠加了腕部 J6 分支跳变。

本次未修改任何规划参数、状态库、ROS2 控制链路或 CoppeliaSim 场景。下一步应围绕 `45:60` 局部重新建立多 IK 状态并保留可连续的 J6 分支，再重新做状态图连接；不要直接放宽关节连续性或进入 RRT。

## 55. 40～60 点 continuation 状态扩展与 DP 复验（2026-09-16）

新增脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/expand_weld_states_by_continuation.m
```

脚本从 `weld_multi_ik_state_graph_expanded.mat` 开始，在点 `40→60` 逐层使用上一层状态作为 IK seed，并对 `J6` 使用 `[0,-5,5,-10,10]°` 微扰 seed。候选姿态窗口为 roll ±30°、travel ±10°，最大相邻关节差为 15°；通过 TCP 误差、碰撞和连续性检查后才加入新状态。原始和 expanded 状态库均未覆盖。

静态检查通过：

```matlab
checkcode('/home/ttbstt/jixiebi/UR5/matlab仿真/expand_weld_states_by_continuation.m')
```

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/expand_weld_states_by_continuation.m')
```

输出：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_multi_ik_state_graph_continuation.mat
Original total states : 5663
Continuation total    : 5755
New states added      : 92
```

新增状态主要位于点 41～52：

```text
41:+9, 42:+9, 43:+9, 44:+9, 45:+9, 46:+9, 47:+9,
48:+7, 49:+7, 50:+7, 51:+7, 52:+1
```

点 53～60 均未新增 continuation 状态：

```text
53:+0, 54:+0, 55:+0, 56:+0, 57:+0, 58:+0, 59:+0, 60:+0
```

随后复制得到：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/plan_dynamic_weld_multi_ik_continuation.m
```

该脚本只把输入状态文件改为 `weld_multi_ik_state_graph_continuation.mat`，输出文件改为 `dynamic_weld_multi_ik_plan_continuation.mat`；weights、strict/medium/loose profile、边碰撞检查和 DP 逻辑均保持不变。静态检查通过。

DP 结果仍未找到完整 80 点路径：

```text
strict : 第54点断开，tested 1518，free 1518
medium : 第57点断开，tested 1991，free 1991
loose  : 第57点断开，tested 1999，free 1995
```

因此没有生成 `dynamic_weld_multi_ik_plan_continuation.mat`。continuation 在 53～60 点没有找到可加入的新状态，说明仅从 expanded 图中所有前驱状态做 J6 微扰仍不足以恢复目标分支；问题需要改为“前向可达集驱动的 continuation”，从起点开始只保留实际可达分支，并在 `40:60` 过程中为每个可达状态主动跟踪所有局部姿态候选，而不是先独立建立状态库再连接。ROS2 控制链路、CoppeliaSim 场景和原状态库均未修改。

## 56. 前向可达集驱动 continuation 复验（2026-09-16）

新增并运行：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/plan_weld_forward_reachable_continuation.m
```

该脚本以 `weld_multi_ik_state_graph_expanded.mat` 的第 1 点作为初始可达状态，从第 1 点开始逐层传播。每一层先连接已有状态，再只从上一层真实可达状态做 J4/J6 continuation；新增状态必须通过 TCP 误差、节点碰撞、15°关节连续性和 1°分辨率整条边碰撞检查。使用 medium 约束：roll 30°、travel 10°、joint 15°。运行不发送 ROS2 命令，不修改 CoppeliaSim。

静态检查通过：

```matlab
checkcode('/home/ttbstt/jixiebi/UR5/matlab仿真/plan_weld_forward_reachable_continuation.m')
```

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/plan_weld_forward_reachable_continuation.m')
```

前向传播结果：

```text
Point 1 : 173 reachable states
Point 2 : 81
Point 3 : 41
Point 4 : 40
Point 5 : 40
Point 6 : 40
Point 7 : 40
Point 8 : 52
Point 9 : 56
Point 10～14 : 56
Point 15 : 52
Point 16～18 : 52
Point 19～21 : 52
Point 22～24 : 48
Point 25～28 : 48
Point 29～30 : 44
Point 31～32 : 40
Point 33～37 : 40
Point 38 : 36
Point 39 : 32
Point 40～45 : 28
Point 46～47 : 24
Point 48～51 : 16
Point 52 : 12
Point 53 : 8
Point 54 : 4
Point 55 : 4
Point 56 : 4
Point 57 : 0  [断开]
```

第 54～56 点仍有 4 个真实可达状态，但第 57 点结果为：

```text
Existing accepted       = 0
Continuation new states = 0
IK calls                = 540
Edge tests              = 0
FINAL reachable states  = 0
```

因此脚本在第 57 点按设计报错并停止：

```text
Forward reachable continuation disconnected at point 57.
```

已保存断点 checkpoint：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_forward_continuation_checkpoint.mat
```

该实验说明：从起点实际可达的分支传播到第 56 点后，仍无法进入第 57 点；问题已不再是“状态库里有没有某个孤立连续状态”，而是当前 medium 姿态窗口、J4/J6 continuation seed 和可达分支共同作用下，第 57 点没有可接受子状态。由于第 57 层没有任何父状态通过连续性筛选，边碰撞不是本次断开原因。尚未生成 `dynamic_weld_forward_continuation_plan.mat`，未修改 ROS2 控制链路、CoppeliaSim 场景和原始状态库。

## 57. 前向—后向可达分支 gap 诊断（2026-09-16）

新增并运行：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_weld_forward_backward_gap.m
```

该脚本不重新求 IK、不做碰撞检测，读取：

```text
weld_forward_continuation_checkpoint.mat
weld_multi_ik_state_graph_expanded.mat
```

并按 medium 约束（roll 30°、travel 10°、joint 15°）从第 80 点全部状态反向传播，再与第 1～56 点的前向 reachableBank 比较。静态检查通过。运行命令：

```matlab
checkcode('/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_weld_forward_backward_gap.m')
run('/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_weld_forward_backward_gap.m')
```

输出文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_forward_backward_gap_diagnosis.mat
```

反向可达状态数量：

```text
Point 80 : 175
Point 57 : 80
Point 56 : 78
Point 55 : 78
Point 54 : 78
Point 53 : 77
Point 52 : 72
Point 51 : 65
Point 50 : 61
Point 49 : 61
Point 48 : 61
Point 47～45 : 59
Point 44 : 59
Point 43 : 53
Point 42 : 48
Point 41～40 : 41
Point 39 : 40
...
Point 25 : 5
Point 24及之前 : 0
```

前向/后向 gap（`min max|Δq|`）在 40～56 点均为大间隙：

```text
Point 40 : 84.2727°
Point 41 : 84.1822°
Point 42 : 84.0956°
Point 43 : 84.0128°
Point 44 : 83.9337°
Point 45 : 83.8585°
Point 46 : 83.7869°
Point 47 : 83.7190°
Point 48 : 83.6548°
Point 49 : 83.5941°
Point 50 : 83.5370°
Point 51 : 83.4834°
Point 52 : 75.3954°
Point 53 : 75.3878°
Point 54 : 75.3833°
Point 55 : 75.3818°
Point 56 : 75.3832°
Point 57 : 前向集合为空
```

40～51 点的最佳 forward/backward 状态对主要由 J6 约 `84°` 的差异主导；52～56 点仍由 J6 约 `75°` 的差异主导。最佳状态对的 J5 大致落在 forward 的 `±100°` 与 backward 的 `±42～45°` 两个分支，距离 0°/±180° 腕部奇异位形仍有明显余量。因此当前证据更支持“两个普通离散 IK 分支从更早点开始分离”，而不是 J5 奇异导致的 J4/J6 连续交换。

重要结论：未来分支并非只在 56→57 突然出现；backward 可达集在第 25 点前已完全消失，40～56 点前后向分支之间长期存在约 75～84° 的 J6 间隙。下一步应在 25～45 点寻找可桥接的共同姿态/同一 roll-travel 状态，做双向 continuation 或 branch bridge；不要继续随机 IK、盲目扩大 J6 扫描或放宽 15°关节阈值。ROS2 控制链路、CoppeliaSim 场景和已有状态库均未修改。

## 58. 同一 TCP 姿态约束下的前后向 branch gap（2026-09-16）

新增并运行：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_weld_same_pose_branch_gap.m
```

该脚本读取 forward continuation checkpoint 和 expanded 状态库，重新计算 medium backward mask，并强制 forward/backward 状态具有相同的 `travelIdx` 与 `rollIdx`，只在同一个 TCP 姿态候选内部比较 IK 分支。脚本不重新求 IK、不做碰撞检测、不发送 ROS2 命令。

静态检查通过：

```matlab
checkcode('/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_weld_same_pose_branch_gap.m')
```

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_weld_same_pose_branch_gap.m')
```

输出：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_same_pose_branch_gap_diagnosis.mat
```

结果：第 25～56 点每一层均为：

```text
common poses = 0
No same-pose forward/backward overlap found.
```

代表性姿态索引：

```text
Point 25 forward : (travelIdx,rollIdx) ≈ (6～9, 20～23)
Point 25 backward: (travelIdx,rollIdx) = (1, 11)

Point 40 forward : (8～9, 20～23)
Point 40 backward: (1～2, 8～11)

Point 52 forward : (9, 20/22/23)
Point 52 backward: (1～4, 8～11)

Point 56 forward : (9, 23)
Point 56 backward: (1～4, 8～11)
```

因此在当前 medium 姿态网格和状态库中，前向分支与后向分支不是“同一 TCP 姿态下的两个 IK 解族”，而是连姿态候选都没有交集。此前 75～84° 的 gap 还混入了不同 roll/travel 姿态；本次结果进一步表明，若要连接两端，必须先构造一条连续的 `travel/roll` 姿态 homotopy，再沿该姿态路径做 IK continuation，不能只在同一姿态内切换 J6 分支。当前没有发现可直接用于 bridge 的共同姿态，尚不能判定是否存在经过近奇异位形的连续桥接。ROS2 控制链路、CoppeliaSim 场景和所有已有状态库均未修改。

## 59. 双向 continuation 与自动 branch bridge 搜索（2026-09-16）

新增并运行：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_weld_bidirectional_branch_bridge.m
```

脚本以 backward 诊断得到的 B57（80 个状态）为起点，向前反向做 IK continuation；seed 使用局部 `ΔJ4/ΔJ5/ΔJ6` 微扰 `[0, ±2.5, ±5]°` 组合，并保持 medium 约束：roll 30°、travel 10°、joint 15°。每个候选状态均检查节点碰撞和 1°分辨率插值边碰撞；同时逐层测试 `F_i → B_{i+1}` 自动桥接。未修改 ROS2、CoppeliaSim 或已有状态库。

静态检查通过。运行命令：

```matlab
checkcode('/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_weld_bidirectional_branch_bridge.m')
run('/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_weld_bidirectional_branch_bridge.m')
```

反向 continuation 可从 B57 一直生成到 B25，代表性状态数：

```text
B57 : 80
B56 : 78
B55 : 78
B54 : 78
B53 : 77
B52 : 72
B51 : 72
B50 : 68
B49 : 68
B48 : 68
B43 : 60
B42 : 55
B41 : 48
B40 : 48
B35 : 39
B34 : 39
B33 : 27
B32 : 27
B31 : 26
B30 : 26
B29 : 19
B28 : 13
B27 : 5
B26 : 5
B25 : 5
```

但所有桥接过渡 `F_i → B_{i+1}`（`i=25:56`）均为：

```text
pose-compatible pairs = 0
min max|dq| = Inf
full compatible pairs = 0
edge-free bridge = 0
```

因此本次没有任何候选进入关节差或边碰撞筛选，也没有找到 bridge。结论比之前更明确：forward 分支与 backward 分支之间的差异首先发生在 `travel/roll` 姿态通道，当前 ±10° travel、±30° roll 的局部窗口无法把两者接起来；不是单纯的 J6 小幅调整或边碰撞问题。输出诊断文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_bidirectional_branch_bridge_diagnosis.mat
```

下一步如果继续，应在约 25～45 点做更宽的姿态 homotopy/局部姿态加密（先允许更大的 travel/roll 变化但仍保留节点和边碰撞检查），或引入小范围 work-angle 自由度；不能把 15°关节阈值直接放宽到几十度，也不能继续盲目增加随机 IK。所有结果已同步到本交接文档。

## 60. 25→45 连续 pose homotopy 双端 shooting（2026-09-16）

新增并运行：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/search_weld_pose_homotopy_bridge.m
```

脚本使用 F25 的 48 个前向状态和 B45 的 66 个反向 continuation 状态，按端点姿态距离选择前 30 组 short-roll 路径及前 30 组 alternate-roll 路径。每组在 25→45 之间对 travel/roll 做连续插值，并分别从两端执行局部 J4/J5/J6 continuation；每个 IK 解都检查 TCP 误差、15°关节连续性、节点碰撞和 1°分辨率边碰撞，然后测试 `F_i→B_{i+1}` 桥。

静态检查通过：

```matlab
checkcode('/home/ttbstt/jixiebi/UR5/matlab仿真/search_weld_pose_homotopy_bridge.m')
```

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/search_weld_pose_homotopy_bridge.m')
```

结果：

```text
Endpoint pairs : 30 short-roll + 30 alternate-roll
short-roll     : 30/30 在正向 Point 26 失败
alternate-roll : 多数在 Point 28 失败，其余在 Point 26 失败
Bridge found   : 0
```

所有试验均未获得同时成功的 forward/backward homotopy 链，因此没有任何 `F_i→B_{i+1}` 进入桥接判定，也没有生成最终轨迹。输出诊断文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_pose_homotopy_bridge_diagnosis.mat
```

对第一组代表性 short-roll 端点（F25 travel/roll=`5°/315°`，B45=`-10°/105°`）单独复核：Point 26 的 IK 解 TCP 位置误差约 `1.2e-9 m`、最大关节变化约 `5.18°`，但节点碰撞检查为 true；说明至少该 homotopy 路径是在进入边桥前被焊枪/工件几何碰撞淘汰，而非单纯 IK 不收敛。当前结果不支持仅靠 25→45 的线性 2D travel/roll homotopy 连接两条分支。ROS2 控制链路、CoppeliaSim 场景和已有状态库均未修改。

## 61. 局部细化二维 travel-roll 可行性地图（2026-09-16）

新增并运行：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/map_local_weld_pose_feasibility_fine.m
```

脚本固定当前工作角 `RtcpWP0`，只对焊点 `26/30/35/40/45` 做局部二维节点地图：

```text
travel = -20:2.5:20 deg   (17 个值)
roll   = 0:5:355 deg       (72 个值)
每个姿态单元使用 17 个确定性 IK seed
```

对每个候选姿态记录 IK 可行性、节点碰撞和碰撞主类。分类约定为：`-1=IK invalid`、`0=free`、`1=gun-bottom`、`2=gun-wall_y1`、`3=gun-wall_x0`、`4=gun-wall_x1`、`5=robot-world`、`6=other`。脚本只做节点可行性诊断，不做路径规划、不发布 ROS2、不修改 CoppeliaSim 或既有状态库。

静态检查通过（仅有预分配、`caxis` 兼容性提示）：

```matlab
checkcode('/home/ttbstt/jixiebi/UR5/matlab仿真/map_local_weld_pose_feasibility_fine.m')
```

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/map_local_weld_pose_feasibility_fine.m')
```

实测结果（每个点共 `17×72=1224` 个姿态单元）：

```text
Point 26 : IK-feasible 1205/1224, collision-free 291/1224
           type1 bottom=396, type2 wall_y1=282, type3 wall_x0=236,
           type4 wall_x1=0, type5 robot-world=0, type6 other=0
Point 30 : IK-feasible 1208/1224, collision-free 292/1224
           type1=396, type2=284, type3=236, type4=0, type5=0, type6=0
Point 35 : IK-feasible 1213/1224, collision-free 293/1224
           type1=396, type2=290, type3=232, type4=0, type5=2, type6=0
Point 40 : IK-feasible 1219/1224, collision-free 293/1224
           type1=396, type2=296, type3=229, type4=0, type5=5, type6=0
Point 45 : IK-feasible 1224/1224, collision-free 314/1224
           type1=396, type2=302, type3=207, type4=0, type5=5, type6=0
```

8 邻域连通性检查：Point 26/30/35/40 的全部 free 单元各自构成一个连通分量（分别 291/292/293/293 个）；Point 45 有两个分量，主分量 294 个、次分量 20 个。主 free 区域的包围范围大致为 `travel=-15..20°`、`roll=55..185°`。因此细网格证明固定工作角下存在宽的局部无碰撞姿态岛，失败主因仍是枪体与底板、`wall_y1`、`wall_x0` 的几何碰撞，而不是 `wall_x1`。

输出文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_local_pose_feasibility_fine.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_local_pose_feasibility_fine.png
```

下一步建议：先基于该 free 区域做局部 reachable continuation/corridor 搜索，并加入相邻点关节步长与插值边碰撞检查；不要据此直接放宽 `maxJointStepDeg`，也暂不修改工作角或 ROS2 控制链路。

## 62. Fine travel-roll 跨焊点 reachable corridor（2026-09-16）

新增并运行：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/search_local_weld_pose_corridor.m
```

脚本从 Point 26 的 fine-map 全部无碰撞状态（291 个）出发，向 Point 45 做跨点 continuation。为避免重复 IK，改为逐目标姿态单元选择最多 6 个低代价兼容父状态，每个父状态使用当前 seed 加小范围 J5/J6 扰动；每个候选都检查：

```text
travel 步长 ≤ 5°
roll 步长 ≤ 10°
max|Δq| ≤ 15°
节点无碰撞
1° 分辨率关节插值边无碰撞
```

静态检查通过（仅有脚本动态数组增长提示）：

```matlab
checkcode('/home/ttbstt/jixiebi/UR5/matlab仿真/search_local_weld_pose_corridor.m')
```

运行命令：

```matlab
run('/home/ttbstt/jixiebi/UR5/matlab仿真/search_local_weld_pose_corridor.m')
```

结果：

```text
Point 26 initial reachable states = 291
Point 27..45 全部保持可达，无断点
Point 45 reachable states = 588
Path states = 20 (Point 26..45)
Complete corridor = 1
Total edge tests = 153386
Total edge-free = 153386
Max joint jump = 0.3395 deg
```

回溯得到的最低代价路径在整段中保持同一个 fine 姿态单元：

```text
travel = 7.5°
roll   = 65.0°
```

因此，在当前固定工作角和当前碰撞代理下，Point 26→45 存在一条非常稳定的连续无碰撞局部姿态通道；不需要放宽关节阈值，也不需要马上引入 work-angle 自由度。该结果也说明此前 forward 分支在 Point 53→57 的断路是状态库/姿态分支选择问题，而非这段几何焊缝在固定工作角下不可通行。

输出文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_local_pose_corridor.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_local_pose_corridor.png
```

本步骤未修改 ROS2 控制代码、CoppeliaSim 场景、TCP 标定、原有状态库或 RRT。下一步应将该 corridor 方法扩展到 Point 1→80（必要时用 Point 26 corridor 状态作为分支种子），并在完整焊缝上再次执行全路径节点/插值边验证。

## 63. MATLAB 焊丝与喷嘴端可视化（2026-09-16）

用户要求先确认喷嘴、焊丝、WeldTCP 的相对位置，再继续姿态/路径搜索。已在：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m
```

中加入焊丝可视化，未修改 ROS2 控制链路、CoppeliaSim 场景、TCP 标定或碰撞代理。

实现方式：

```text
WeldTCP body
├─ visual Cylinder：焊丝
└─ visual Sphere：NozzleTip 端点标记
```

当前几何约定为：

```text
+Z_TCP：喷嘴 → 工件
WeldTCP 原点：焊接作用点
NozzleTip：TCP - 18 mm·Z_TCP
焊丝：沿 TCP 局部 z=-18 mm 到 z=0
```

焊丝长度沿用此前已确认的 18 mm；焊丝直径目前为 `1.2 mm` 临时显示值，因为标定文件中没有实际焊丝直径字段。脚本会打印警告并标记：

```text
temporary_visual_default_NEEDS_CONFIRMATION
```

后续拿到实际送丝规格后，可在 `ur5_tool_calibration.mat` 中加入 `wireDiameterM`（单位 m），脚本会自动优先读取该值。焊丝当前是 `addVisual` visual-only 几何，不进入 `checkCollision`，因此没有擅自改变避障结果；若以后需要把焊丝纳入碰撞，需要单独定义允许的焊接接触规则。

运行验证：

```matlab
checkcode('/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m')
run('/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m')
```

MATLAB 输出的当前零位端点为：

```text
WeldTCP    = [-0.0195 -0.5674 0.9344] m
NozzleTip  = [-0.0195 -0.5494 0.9344] m
```

已生成用于近距离核对的预览图：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/scene_with_wire_preview.png
/home/ttbstt/jixiebi/UR5/matlab仿真/welding_tip_detail_preview.png
```

近距离图中可见金色细圆柱连接 WeldTCP 与黑色 NozzleTip 标记，且该 visual 会作为 `WeldTCP` 的固定几何随任意 `show(robot,q)` 刚性运动。按用户要求，本阶段暂停重新优化路径；下一步先根据实际焊丝直径/喷嘴端位置确认该可视化，再恢复完整焊缝 corridor 搜索。

## 64. 修正焊丝可视化定义：不新增 NozzleTip（2026-09-16）

用户指出不应新建 `NozzleTip`。已修正 `sync_ur5_joint_states.m`：

```text
删除：Sphere NozzleTip visual、NozzleTip 字段和标记
保留：WeldTCP body 上的 visual-only 焊丝 Cylinder
```

现在焊丝只表示“现有焊枪头端部”和 `WeldTCP` 之间的实体间隙；焊枪头继续由已有 `WeldingGun_visual/GunCollisionHull` 表示，不再创建新的端点对象。当前 +Z_TCP 约定不变，焊丝覆盖 TCP 局部 `z=-18 mm..0`，并随 WeldTCP 刚性运动。MATLAB 场景重新运行验证通过，`WeldTCP` 只含 1 个 visual、0 个 collision。

当前零位下用于显示核对的焊枪头端位置为：

```text
Gun head end = TCP - 18 mm·Z_TCP
             = [-0.0195 -0.5494 0.9344] m (MATLAB base)
```

`welding_tip_detail_preview.png` 已重新生成，当前只显示现有焊枪头、连接焊丝和 WeldTCP，不再包含端点球标记。

## 62. 独立 WeldTCP 可达性总复核（2026-09-16）

为把“工作空间 → 完整姿态 IK → 多分支 → 连续性 → 插值碰撞”统一成一次可重复验收，新增并运行：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/analyze_weld_reachability.m
```

该脚本只读当前场景和既有状态库，不运行 RRT、不发送 ROS2、不修改 CoppeliaSim。它完成：

- 验证 `scene.tcpBodyName == "WeldTCP"`，并检查当前 TCP 轨迹与保存状态库的目标位置一致性；
- 用固定随机种子采样 30,000 个关节构型，计算 `WeldTCP` 三维位置工作空间并叠加 80 点焊缝；
- 重新对扩展状态库的 5,663 个状态做 FK 误差和 Robot/Gun–Workpiece 碰撞复核；
- 按 strict/medium/loose profile 前向传播可达集，并对候选边做 1° 关节插值碰撞检查；
- 按 medium profile 从终点反向传播状态集，给出前后向分支重叠和断点信息；
- 输出 MAT、CSV、TXT 和两张 PNG 诊断图。

输出文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_reachability_analysis.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_reachability_summary.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_reachability_analysis_report.txt
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_tcp_reachability_workspace.png
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_reachability_summary.png
```

复核结果：

```text
TCP body                                  = WeldTCP
当前 TCP 与状态库最大位置差               = 0 mm
Monte-Carlo workspace samples             = 30,000
所有 80 个 TCP 焊缝点均落在采样边界内       = true
逐点有可行 travel/roll 姿态                = 80 / 80
逐点有无碰撞完整姿态多 IK 状态              = 80 / 80
多 IK 状态总数                             = 5,663
每点状态数 min / max / mean                = 36 / 175 / 70.787
状态 FK 最大位置误差                       = 0.000246 mm
状态 FK 最大姿态误差                       = 0.000008 deg
```

连续可达性结果（含节点和 1° 插值边检查）：

```text
strict : 53 -> 54 断开，53 为最后可达点；断开层 pose-compatible/full pairs = 0 / 0
medium : 56 -> 57 断开，56 为最后可达点；断开层 pose-compatible/full pairs = 0 / 0
loose  : 56 -> 57 断开，56 为最后可达点；断开层 pose-compatible/full pairs = 0 / 0
```

在断开之前，strict/medium/loose 的插值边拒绝数分别为 `0 / 8 / 84`；因此本次断开主因是前向状态图的姿态/关节连续性分支耗尽，而不是插值碰撞。medium 反向可达后缀最早从第 25 点开始，25～56 点与前向可达集没有同一 `travelIdx + rollIdx` 的状态重叠。

结论：当前 TCP 的“单点完整姿态可达性”已通过，但“从焊缝起点连续焊到终点”仍未通过。该结论支持继续做局部姿态 homotopy/工作角或碰撞代理审计；在出现完整连续状态路径之前，不应把问题交给 RRT 参数调节或 ROS2 执行。当前场景、标定、ROS2 控制链路和既有状态库均未修改。本次新增脚本和报告均可直接删除回滚。

## 63. 完整 Point 1–80 局部 corridor 断裂诊断（2026-09-16）

按上一节建议扩展并运行：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/search_local_weld_pose_corridor.m
```

脚本现在从 Point 1 开始，不再依赖只覆盖 `26/30/35/40/45` 的 fine-map，使用：

```text
travel = -20:2.5:20 deg（17 个）
roll   = 0:5:355 deg（72 个）
Point 1 初始层：17 个确定性 IK seed，每个姿态最多 4 个状态
连续层快速代表分支：每个姿态保留 1 个最低代价状态，只尝试 1 个最低代价父状态
max|Δq| ≤ 15°，1° 分辨率关节插值边检查
```

节点和边碰撞使用普通快速检查；只有发现碰撞的样本才调用 exhaustive 分离矩阵，
并把对象名累计到统计中。静态检查通过：

```text
CORRIDOR_CHECKCODE_PASS
```

最终运行结果：

```text
Point 1 reachable                         = 2028
Point 25 -> 26 reachable                  = 247 -> 250
Point 53 -> 54 reachable                  = 273 -> 274
Point 80 reachable                        = 279
Last reachable point                      = 80
Complete corridor                         = 1
First break point                         = none
First break reason                        = 无断裂，Point 1-80 全部连通
Path states                               = 80
Max joint jump                            = 9.444 deg
Total edge tests / edge-free / rejected   = 20959 / 20933 / 26
```

因此，在当前 WeldTCP、当前工件碰撞代理和细 travel-roll 网格下，Point 1–80
存在一条连续状态走廊；旧粗状态图的 `53 -> 54` 断裂没有在该细化搜索中复现。
26 条边拒绝全部出现在 Point 4，碰撞对象为：

```text
base_link <-> WorkpieceCollision_wall_y1
```

节点拒绝记录中也只出现该对象，没有发现 WeldTCP、焊枪凸包、bottom、wall_x0
或 wall_x1 导致的本次 corridor 断裂。首断裂字段因此为 `firstBreakPoint=0`，
而不是把某个局部拒绝误判为断点。

输出文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_local_pose_corridor.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_local_pose_corridor.png
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_local_pose_corridor_summary.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_local_pose_corridor_collisions.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_local_pose_corridor_report.txt
```

文件级断言通过：MAT 中 80 个 layer 全部非空，summary CSV 为 80 行，collision
CSV 为 81 条对象统计记录。一次更高分支数的完整试跑（每姿态最多 4 状态、每目标
最多 6 父状态、5 个 wrist seed 扰动）也已在 Point 80 完成核心计算，但首次落盘
时因旧版 MATLAB `table` 参数写法报错；本节最终文件采用上面的快速代表分支模式，
并已修复兼容性后重新运行成功。整个过程未修改 ROS2 控制、CoppeliaSim 场景、TCP
标定、原有状态库或 RRT。

## 64. Point 1–80 corridor 回放可视化（2026-09-16）

第 63 节只给出了数值上的连续走廊，没有把 80 个关节状态实际驱动起来看过。
本节新增一个**纯可视化回放**脚本，并在过程中确认了一个**会影响后续所有
MATLAB 可视化工作的环境结论**（见 64.4），务必先读那一节。

### 64.1 新增脚本

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/play_weld_corridor_motion.m
```

签名与参数：

```matlab
out = play_weld_corridor_motion(opts)
%   StepDeg         每帧允许的最大关节变化，默认 0.5 deg
%   Speed           回放速度倍率，默认 1.0
%   ShowCollisions  是否显示碰撞模型，默认 true
%   ShowTrail       是否画 TCP 轨迹拖尾，默认 true
%   Verify          是否复验碰撞，默认 true
%   SaveVideo       是否录制，默认 false
%   VideoFile       录像文件，默认 weld_corridor_motion.avi
%   View            相机视角，默认 default（见下表）
%   ViewSpan        特写视角的视野边长[m]，默认 0.35
%   FollowTcp       特写视角是否跟随 TCP，默认 true
%   MaxFrames       最多渲染前 N 帧，默认 Inf（试角度时用，省时间）
```

用法：

```matlab
play_weld_corridor_motion                                   % 默认回放
play_weld_corridor_motion(StepDeg=0.2, Speed=0.5)           % 更细更慢
play_weld_corridor_motion(SaveVideo=true, Verify=false)     % 录制
play_weld_corridor_motion(View="allTop", SaveVideo=true, ...% 全场景俯拍
    VideoFile="weld_all_top.avi")
play_weld_corridor_motion(View="gunTopTilt", ViewSpan=0.35, ... % 焊枪特写
    MaxFrames=36, SaveVideo=true, VideoFile="test.avi")
```

#### 64.1.1 `View` 可取值

两类视角的取景机制**完全不同**，这是本次踩过的坑：

| 类别 | 取值 | 取景方式 |
| --- | --- | --- |
| 全景 | `default` | 交给 `show()` 自己算，**只按机器人本体**算轴范围 |
| 全景 | `all` / `allTop` | 脚本自动求整个场景包围盒（见下） |
| 特写 | `gun` `gunSide` `gunSideNeg` `gunTop` `gunTopTilt` `gunBack` `gunFront` | 轴范围收到 TCP 附近 `ViewSpan` 见方的小块 |

`all` = 3/4 视角（`view(-37.5,25)`），`allTop` = 俯视（`view(0,90)`）。

**为什么必须自己算包围盒**：`show()` 设置的轴范围只覆盖机器人本体，位于
`x≈-0.605` 的工件会被裁在画面外，焊枪跟着一起出画 —— 这正是默认视角
"看不到焊枪"的真正原因，问题在**轴范围**而不在相机方向。全景视角改为采样
15 帧，对每帧所有 patch 顶点求并集，再加上焊缝点和 TCP 轨迹，得到覆盖整个
运动过程的包围盒，外扩 8% 余量后 `daspect([1 1 1])`。实测包围盒：

```text
X [-0.727 0.107]   Y [-0.479 0.207]   Z [-0.057 0.570]   m
```

**已知取舍**：`all`（3/4）能看到工件、机械臂、焊枪，但焊缝贴在工件朝向
`-X` 的立面上，从该方位被工件壁挡住；`allTop`（俯视）四个对象加焊缝全部
可见，是"全都要看见"时应当选的视角。

脚本职责边界（严格遵守既有约束）：

- 只读 `weld_local_pose_corridor.mat` 的 `qPath` / `posePath` / `RtcpWP0`，
  **不重新规划、不重新求 IK**；
- 通过 `sync_ur5_joint_states.m` 复用**唯一主 Figure**（第 36 节约定）。
  该脚本是 script 且开头 `clear`，因此回放在隔离的子函数工作区里 `run` 它，
  只把 `scene` 带出来；
- 插值方式为**关节空间线性插值**，与 `controller.py` 处理 `JointTrajectory`
  的方式一致，保证回放看到的就是控制器实际会走的路；
- TCP 拖尾用 `getTransform` 的**运动学真值**，不是直接画目标点；
- **不发 ROS2、不修改 CoppeliaSim 场景、不修改 `functional.lua`、
  不修改 `controller.py`、不改动任何规划结果文件或 TCP 标定。**

### 64.2 本次回放当场验证的数字

以下数字由独立脚本重新读取 `weld_local_pose_corridor.mat` 并重新做 FK /
碰撞检查得到，非沿用第 63 节的记录值：

| 项目 | 值 |
| --- | --- |
| corridor 结果 | `complete=1`，`lastPoint=80`，80 个焊点 |
| 焊缝长度 | 0.205000 m |
| 焊缝起点（MATLAB base） | `[-0.605, -0.0599999, 0.00500016]` |
| 焊缝终点（MATLAB base） | `[-0.605, 0.145, 0.00500016]` |
| 80 个规划点 TCP 位置误差 | `1.599315e-05` mm |
| 最大相邻关节跳变（raw） | 9.443961 deg |
| 最大相邻关节跳变（wrapped） | 9.443961 deg |
| 与 MAT 中 `maxJointJumpDeg` 一致性 | 9.443961，一致 |
| travel 范围 | -12.500 .. 0.000 deg |
| roll 范围 | 40.000 .. 75.000 deg |
| 80 个焊点环境碰撞数 | **0 / 80** |

`raw` 与 `wrapped` 的 `max |dq|` **完全相同**，说明这条路径上不存在 2π
关节回绕歧义；回放不会因为选错分支而走出完全不同的姿态。

回放帧本身（`StepDeg=0.5` → 143 帧）：

| 项目 | 值 |
| --- | --- |
| 回放帧数 | 143 |
| 回放帧 TCP 相对理想直线焊缝的最大偏离 | **0.4620 mm** |

该 0.4620 mm **不是规划误差**：80 个规划点自身精度是 1.6e-5 mm。它是关节
空间线性插值在两点之间走弦而非走弧造成的弦误差。若后续对焊缝跟踪精度提出
亚毫米要求，需要把插值改为任务空间插值或在关节空间加密轨迹点。

### 64.3 录制与视频产物

录制在 `-batch`（无窗口）下完成，一次跑通，143 帧、1394×840、
Motion JPEG AVI、8.6 MB、耗时 26.5 s，**未出现帧尺寸变化警告**。

注意：`play_weld_corridor_motion.m` 中 `fig.Resize = "off"` **仅在
`SaveVideo=true` 时生效**。这条是为了防止窗口管理器在录制中途最大化窗口、
改变 `getframe` 捕获尺寸导致 `VideoWriter` 拒绝后续帧；手动查看时保持可缩放。

画面内容已做非空白校验（直接读回 AVI 逐帧统计）：

| 指标 | 值 |
| --- | --- |
| 平均亮度 | ~239 |
| 标准差 | ~52 |
| 独立颜色数 | 17k – 22k |
| 非黑像素占比 | 1.000 |
| 相邻帧平均差异 | 0.938（证明机器人在运动） |

产物文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_corridor_motion.avi   Motion JPEG, 8.6 MB   默认视角
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_corridor_motion.mp4   H.264, 342 KB
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_corridor_motion.gif   800x482, 9.3 MB, 143 帧

/home/ttbstt/jixiebi/UR5/matlab仿真/weld_all.avi       Motion JPEG, 9.3 MB  View=all（全场景 3/4）
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_all.mp4       H.264, 225 KB
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_all.gif       800x482, 10.1 MB, 143 帧
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_all_top.avi   Motion JPEG, 8.7 MB  View=allTop（全场景俯视）
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_all_top.mp4   H.264, 262 KB
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_all_top.gif   800x469, 6.8 MB, 143 帧
```

抽帧目视核对（`cv2` 读第 0 / 71 / 142 帧）：`weld_all` 与 `weld_all_top` 中
工件、机械臂、焊枪均在画面内且**无裁切**，各留有余量；`weld_all_top` 中焊缝
红线从 WeldStart 到 WeldEnd 全程可见，焊枪横向摆角清楚，是"四个对象全都要
看见"时的推荐视角。`weld_all` 的四对象也都在，但焊缝贴在工件朝 `-X` 的
立面上，从该方位被工件壁挡住，只剩轮廓边缘一条。

**播放器兼容性（本机实测，容易踩坑）**：

- `weld_corridor_motion.avi` → **`totem` 可播**。gst-discoverer 确认
  gstreamer 能解其中的 Motion JPEG。
- `weld_corridor_motion.mp4`（H.264）→ **`totem` 放不了**。本机 gstreamer
  **缺 H.264 解码器**（`Missing plugins: H.264 (High Profile) decoder`），
  也缺 MPEG-4 Part 2 解码器。该 MP4 只能用浏览器（Chrome 自带解码器）看。
- `weld_corridor_motion.gif` → VSCode 里可直接点开预览。

转码脚本（新增，放在 `UR5/` 下，与既有 `export_scene_geometry.py`、
`read_tool_frames.py` 同级）：

```text
/home/ttbstt/jixiebi/UR5/convert_playback_video.py
python3 convert_playback_video.py weld_corridor_motion.avi --fps 30
```

它用 `cv2`（OpenCV 自带 FFMPEG）转 MP4、用 `PIL` 写 GIF，MP4 的 fourcc
按 `avc1 -> H264 -> mp4v` 依次尝试。**必须优先 `avc1`**：本机 gstreamer
没有 MPEG-4 Part 2 解码器，`mp4v` 出来的文件谁也播不了。

系统里没有可用的 `ffmpeg` 可执行文件：`~/.local/bin/ffmpeg` 是一个指向
已不存在 conda 环境的损坏 wrapper。OpenCV 自带的那份 FFMPEG 是唯一能用的。

### 64.4 重要环境结论：本机 MATLAB GUI 无法显示窗口

**这是本节最重要的内容。** 本机上 MATLAB R2025b 的**图形窗口无法显示**，
导致 `play_weld_corridor_motion` 在 MATLAB 里"看不见"。已排除的假设：

| 启动方式 | 结果 |
| --- | --- |
| `matlab -desktop` | 所有窗口 `IsUnMapped`，桌面主窗口也不出现 |
| `matlab -nodesktop` | 连 X 窗口都不创建 |
| `matlab -softwareopengl` | 仍然 `IsUnMapped` |
| 非沙箱启动（`dangerouslyDisableSandbox`） | 仍然 `IsUnMapped` |

**根因**：R2025b 的图形栈已换成 **Chromium/CEF + ANGLE 走 WebGL**，
不再走传统 OpenGL 直接绘制。`rendererinfo` 返回：

```text
GraphicsRenderer: 'WebGL'
          Vendor: 'Google Inc. (NVIDIA Corporation)'
         Version: 'WebGL 2.0 (OpenGL ES 3.0 Chromium)'
  RendererDevice: 'ANGLE (NVIDIA Corporation, NVIDIA GeForce RTX 5060/PCIe/SSE2, OpenGL 4.5.0)'
```

该 CEF 渲染进程在本机上起不来，窗口创建后立即被 `Withdrawn`。

**已排除的非原因**：

- 不是显卡/驱动问题：系统 OpenGL 正常（RTX 5060，驱动 580.178.04，
  OpenGL 4.6，`direct rendering: Yes`，X11 会话）。
- 不是权限或沙箱问题：环境变量齐全（`DISPLAY=:0`、`XAUTHORITY`、
  `DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus`）。
- 不是 MATLAB 本身不能跑：同进程树里 `-batch` 计算和**离屏渲染完全正常**
  （第 64.3 节的视频就是这么录出来的）。
- **不是 GUI 栈坏了**：`totem` 在本机上窗口一次就正常 `IsViewable`。

**结论与后续做法**：

1. 在本机**不要指望用 MATLAB 窗口看任何 3D 场景**；`show()`、
   `plot3`、`figure` 的结果都看不到。
2. 需要看效果时，走**离屏录制**路线：
   `matlab -batch "play_weld_corridor_motion(SaveVideo=true)"`，
   再按 64.3 转码。
3. 该限制只影响**可视化**，不影响规划、IK、碰撞检查等任何计算 ——
   第 63 节和本节的数值结果都建立在这些计算上，全部有效。
4. 若后续确实需要交互式 MATLAB 窗口，需要单独排查 CEF/ANGLE
   （方向：CEF 渲染进程的 GPU 初始化、`--no-sandbox`、
   或改用更早的 MATLAB 版本）。

**快速复现/确认该故障**（约 1 分钟，不必再走一遍上表的四种启动方式）：

```bash
# 1) 起一个 MATLAB 桌面，等它初始化
pgrep -x MATLAB >/dev/null || (nohup matlab -desktop >/dev/null 2>&1 &)
sleep 60

# 2) 看 Matlab 的 X 窗口是否真的显示出来
for id in $(xwininfo -root -children | awk '/Matlab-GLEE/{print $1}'); do
  printf '%-12s %s\n' "$id" "$(xwininfo -id $id | awk -F': ' '/Map State/{print $2}')"
done
```

判读：全部输出 `IsUnMapped`、或**根本没有 `Matlab-GLEE` 条目** ⇒ 故障仍在，
不要浪费时间等它"再启动一会儿"。对照组：`totem` 的窗口应当立刻
`IsViewable`（GUI 栈本身是好的）。用完 `pkill -x MATLAB` 收掉。

另一个旁证（以下两行是在 `-batch` 模式下实测的输出）：

```text
OPENGLL_FAILED: 调用 "opengl" 时，未对一个或多个输出参数赋值。
RENDERERINFO_OK renderer=WebGL
RENDERERINFO_OK device=ANGLE (NVIDIA Corporation, NVIDIA GeForce RTX 5060/PCIe/SSE2, OpenGL 4.5.0)
```

即渲染器信息拿得到、但窗口呈现这一步断了。

**注意**：`matlab -batch` **不接受多行命令**，传多行会报
`没有为 -batch 命令行参数指定 MATLAB 命令` 并静默退出（退出码仍是 0，
很容易误判为"跑过了没输出"）。多行逻辑必须先写成 `.m` 文件再
`matlab -batch "run('/tmp/xxx.m')"`。

### 64.5 约束遵守情况

本节新增内容未触碰任何既有约束：未恢复 CoppeliaSim Python 的 `simROS2`
订阅者，未修改 `functional.lua`，未修改 `controller.py` 或 ROS2 topic，
未改动冻结的 TCP 标定（`T_mount_tcp`），未用 STL 拟合覆盖 WeldTCP，
未添加焊丝伸出量，未在运行中的机器人上施加任何关节跳变。
`sync_ur5_joint_states.m` 只被 `run` 调用、未被修改。回放全程只读
`weld_local_pose_corridor.mat`。

## 65. 焊丝方向修正：沿现有焊枪头到 WeldTCP 的实际工具方向（2026-09-16）

用户指出焊丝不能按任意的 TCP 轴或新增 `NozzleTip` 放置，而应沿现有焊枪的真实方向，位于“焊枪头 ↔ WeldTCP”之间。已据此修正
`/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m`：

- 没有创建或加入 `NozzleTip` 对象；`WeldTCP` 标定和 `T_mount_tcp` 未修改。
- 在当前 `GunCollisionHull` 中寻找距离标定 TCP 最近的枪头侧顶点，作为现有焊枪头端的几何估计。
- 将该点转换到 `WeldTCP` 局部坐标，使用“枪头 → TCP”的单位向量作为焊丝圆柱轴；焊丝另一端位于 WeldTCP，故焊丝不再强制沿 TCP `+Z`。
- 焊丝仅作为 `WeldTCP` 下的 visual-only 圆柱，不参与碰撞检测；当前长度仍为 18 mm，直径 1.2 mm 为临时显示默认值，实际直径待用户确认。

当前运行验收：

```text
WeldTCP visuals=1 collisions=0
Wire visual length  = 18.0 mm
Wire axis in TCP    = [-0.4047 -0.5816 0.7057]
Nearest hull head gap = 20.8 mm (geometry estimate)
```

已更新预览：
`/home/ttbstt/jixiebi/UR5/matlab仿真/welding_tip_detail_preview.png`。
预览中金色焊丝连接绿色 GunCollisionHull 的枪头侧与红色 WeldTCP，未增加端点球或 NozzleTip。

静态检查：
`checkcode('/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m')`
仅报告原有的第 913 行已被抑制消息不再生成，没有新增语法或方向计算错误。

若用户所说的“焊枪方向1”特指 CoppeliaSim 中某一条明确局部轴（例如 `+X/+Y/+Z` 或其反向），需要以该轴约定覆盖当前几何推断；在未给出明确轴定义前，当前实现以现有 GunCollisionHull 的枪头侧几何方向为准。

## 66. 焊丝改为枪头端固定、与 WeldTCP 保留间隙（2026-09-16）

用户进一步明确：焊丝应连接在焊枪头，而不是把一端锚定在 `WeldTCP`；焊丝末端与 TCP 可以保留少量距离。已在
`/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m` 调整 `T_tcp_wire`：

- Cylinder 的 `-Z` 端固定在当前 GunCollisionHull 推算的枪头端点 `pGunHeadTcp`。
- Cylinder 沿“枪头 → TCP”方向伸出 `wireLengthM`（当前 18 mm）。
- 不再让焊丝的末端强制落在 TCP；若枪头几何估计距离大于 18 mm，则在焊丝末端与 TCP 之间保留真实计算出的间隙。
- 主场景中金色实线只表示实际焊丝，焊丝末端到红色 WeldTCP 之间用灰色虚线表示保留的间隙。
- 未新增 `NozzleTip`，未修改 `T_mount_tcp`、ROS2 控制链路或 CoppeliaSim 场景。

对应新增场景字段：

```matlab
scene.pWireEndMat0
scene.wireTcpGapM
```

运行时会打印 `Wire-to-TCP remaining gap`，便于确认实际间隔；当前 GunCollisionHull 几何估计为约 2.8 mm（20.8 mm 枪头距离减去 18 mm 焊丝长度）。

已按该端点定义重新生成当前枪头细节预览：
`/home/ttbstt/jixiebi/UR5/matlab仿真/welding_tip_detail_preview_current.png`。
图中金色实线是从枪头开始的 18 mm 焊丝，灰色虚线是焊丝末端到 WeldTCP 的约 2.8 mm 保留间隙。

## 67. 当前 Workpiece 板厚核对（2026-09-16）

从当前 `/home/ttbstt/jixiebi/UR5/scene_geometry.json` 读取到：

```text
plate_thickness_m = 0.005
```

因此仿真工件底板和三块侧板的碰撞代理板厚均为 **5 mm**。当前 U 型工件外包络尺寸为 `235 mm × 235 mm × 235 mm`，开口方向为 `y=0`。MATLAB 中对应的四个 `collisionBox` 代理也使用 5 mm 厚度：底板尺寸 `[235,235,5] mm`，两块 x 侧板尺寸 `[5,235,235] mm`，y 侧板尺寸 `[235,5,235] mm`。

## 68. 焊丝加入碰撞模型（2026-09-17）

用户要求焊丝参与避障。已在
`/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m` 中保留原金色 visual 圆柱，并使用同一半径、长度和 `T_tcp_wire` 姿态创建：

```matlab
wireCollision = collisionCylinder(wireDiameterM/2, wireLengthM);
addCollision(tcpBody, wireCollision, T_tcp_wire);
```

该碰撞圆柱挂在 `WeldTCP` 刚体下，因此随 `WeldingGunMount` 刚性运动，并参与 `checkCollision` 的环境碰撞和自碰撞检查。焊丝当前仍为 18 mm 长、1.2 mm 直径；直径是临时默认值，后续应按真实焊丝规格更新。

启动场景脚本的实测输出：

```text
WeldingGunMount collision count = 4
WeldTCP collision count (wire) = 1
Wire collision          : enabled (collisionCylinder)
[FREE] 当前零位 Robot/Gun 与 Workpiece 无碰撞。
Wire-to-TCP remaining gap: 2.8 mm
```

未修改 ROS2 控制链路、CoppeliaSim 场景或 TCP 标定。`checkcode` 仅报告原有第 952 行的历史 analyzer 提示。

## 69. CoppeliaSim 焊丝碰撞体请求的处理（2026-09-17）

用户随后明确：焊丝碰撞体只需要加入 MATLAB 规划模型，不在 CoppeliaSim 中继续创建。已停止 CoppeliaSim 场景写入，确认 `/UR5/connection/WeldingGunMount/WireCollision` 不存在，未修改 `ur5.ttt`。

为清理一次临时 Remote API 测试连接，CoppeliaSim 进程曾被重启；场景文件重启前已确认保存，重启后远程 API 正常。原 ROS2 控制节点已按
`ros2 run ur5_controller controller --ros-args -p send_demo:=false` 恢复，并实测重新连接 CoppeliaSim、识别 6 个关节。当前焊丝碰撞体仍只存在于 MATLAB 的 `sync_ur5_joint_states.m`。

## 70. 新路径重规划前的当前工具/焊缝几何审计（2026-09-17）

新增并运行：
`/home/ttbstt/jixiebi/UR5/matlab仿真/audit_current_weld_geometry.m`

该脚本复用 `sync_ur5_joint_states.m`，只读检查当前模型，不求 IK、不读取旧路径、不发布 ROS2。结果保存为：
`/home/ttbstt/jixiebi/UR5/matlab仿真/current_weld_geometry_audit.mat`

关键结果：

```text
WeldingGunMount collision objects = 4
WeldTCP collision objects         = 1
Wire collision                    = MATLAB collisionCylinder
Gun head -> TCP distance          = 20.786 mm (GunCollisionHull nearest-point estimate)
Wire length                       = 18.000 mm
Wire end -> TCP gap               = 2.786 mm
Wire axis in TCP                  = [-0.404661 -0.581560 0.705718]
Workpiece plate thickness         = 5.000 mm
Workpiece proxies                 = 4
```

当前 U 型工件中，`bottom × wall_y1` 焊缝的几何信息可明确写成：焊缝切线为 Workpiece `+X`；底板自由空间法向为 `+Z`；`wall_y1` 内表面法向为 `-Y`；U 型内部自由空间位于 `y < 0.230 m, z > 0.005 m`。两法向的自由空间角平分线为 `[0,-0.7071,0.7071]`，现有 `+Z_TCP=[0,0.7071,-0.7071]` 正好反向指向内角，数学上对应 45° 工作角。这里的 `+Z_TCP` 只是焊接姿态的接近/工作轴，不能直接当作焊丝轴；焊丝轴独立使用 GunCollisionHull 枪头端点到 TCP 的延伸方向 `wireAxisTcp=[-0.404661,-0.581560,0.705718]`。

仍需用户确认工艺语义：实际焊接是否确实在 U 型内部（`y<0.230,z>0.005`）进行，且焊枪 `+Z` 是否允许从自由空间指向 `bottom × wall_y1` 内角。该确认完成前不生成新的姿态候选和 80 层 IK 图，避免把错误焊接侧作为硬约束写死。当前 GunCollisionHull 没有独立的喷嘴中心/轴参考，枪头点仍是最近顶点估计；2.8 mm 不是重新标定的工艺参数。

## 71. 区分实际焊枪轴与 TCP +Z（2026-09-17）

用户强调：工作角必须基于实际焊枪工作轴线，而不是简单使用 TCP `+Z`。已更新并重新运行 `audit_current_weld_geometry.m`：

```text
wire axis in WeldTCP = [-0.404661 -0.581560 0.705718]
wire segment endpoint error = 0.000 deg
```

这证明当前 visual/collision 焊丝中心线确实从 GunCollisionHull 枪头端沿枪头延伸方向伸出，且与 `WeldTCP` 之间保留 2.786 mm 间隙；但由于场景没有独立的喷嘴/送丝轴 Dummy，当前轴仍只能作为“枪头延伸轴代理”，不能直接宣称是已标定的喷嘴工作轴。

将该代理轴带入当前参考焊缝姿态 `RtcpWP` 后：

```text
wire/gun axis in Workpiece = [-0.4047 0.9102 -0.0878]
wire axis vs TCP approach axis = 45.11 deg
wire axis vs bottom-inward     = 84.96 deg
wire axis vs wall-inward       = 24.46 deg
```

因此现有“45°工作角”只证明 `+Z_TCP=[0,0.7071,-0.7071]` 平分底板/侧板参考方向，不能证明实际焊丝/喷嘴轴也为 45°。在用户确认“焊丝中心线就是实际喷嘴工作轴”或提供独立喷嘴轴/设计夹角前，不得把现有 45°姿态直接用于新的 Point 1–80 搜索。

## 72. 用户确认以当前焊丝轴作为真实焊枪轴（2026-09-17）

用户明确授权：当前 MATLAB 中的焊丝中心线作为实际焊枪工作轴，重新定义焊接侧、工作角和完整碰撞约束。由此冻结以下规划语义：

```text
实际枪轴（TCP 局部） = scene.wireAxisTcp
焊缝                 = bottom × wall_y1 交线
焊接侧               = U 型工件内部：y < wall_y1 内表面，z > bottom 上表面
目标枪轴（Workpiece）= [0, 1, -1] / sqrt(2)
工作角               = 相对底板法向 45°（由目标枪轴定义）
```

枪轴硬约束采用实际焊丝轴变换后的方向与目标枪轴夹角不超过 5°；枪头估计位置还必须保留在 U 型内部。焊丝不在 CoppeliaSim 中新增碰撞体，仍只作为 MATLAB `WeldTCP` 刚体下的 `collisionCylinder` 参与规划碰撞检测。未修改 TCP 标定、CoppeliaSim 场景或稳定 ROS2 控制器。

## 73. 实际枪轴重规划代码（2026-09-17）

新增文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/make_actual_gun_axis_pose.m
/home/ttbstt/jixiebi/UR5/matlab仿真/generate_actual_axis_ik_candidates.m
/home/ttbstt/jixiebi/UR5/matlab仿真/replan_weld_actual_axis.m
/home/ttbstt/jixiebi/UR5/matlab仿真/animate_weld_actual_axis_replan.m
/home/ttbstt/jixiebi/UR5/matlab仿真/publish_weld_trajectory.m
```

用途：

- `make_actual_gun_axis_pose.m`：以 `scene.wireAxisTcp` 为物理枪轴，施加 travel/roll，构造 TCP 目标姿态。
- `generate_actual_axis_ik_candidates.m`：每个焊点保留多个 IK 候选，检查 TCP 误差、关节限位、U 内部焊接侧、UR5/焊枪/焊丝与工件碰撞，以及自碰撞。
- `replan_weld_actual_axis.m`：不读取旧焊缝路径或旧状态图，自动由四个 Workpiece box 推导 `bottom × wall_y1` 焊缝，粗采样后局部细化并建立 80 层相邻状态图。
- `animate_weld_actual_axis_replan.m`：只播放已保存结果，不重新求 IK、不发布 ROS2；显示工件、焊缝、UR5、焊枪、焊丝 TCP 和实际枪轴。
- `publish_weld_trajectory.m`：可选地把完整 `qPath` 转发到现有 `/joint_trajectory` 控制接口；未修改控制器。

自碰撞采用显式结构连接排除（相邻安装体及工具附着体的必然重叠），除此以外的自碰撞仍作为硬约束；焊丝与环境接触没有焊缝豁免。

## 74. 80 点完整重规划验收结果（2026-09-17）

运行命令：

```bash
matlab -batch "addpath('/home/ttbstt/jixiebi/UR5/matlab仿真'); opts=struct('animate',false,'publish',false,'outputFile','weld_actual_axis_replan.mat'); result=replan_weld_actual_axis(opts);"
```

结果文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan.mat
```

输出：

```text
[PASS] Complete measured-wire-axis 80-point path found.
Weld points             : 80
Layers with IK states   : 80 / 80
Reachable final states  : 2
Node collision free     : 1
Edge collision free     : 1
Max joint jump          : 9.954 deg
Wire collision model    : enabled
Approach path planned   : 0
Exit path planned       : 0
```

独立验收（重新用保存的 `qPath` 检查环境碰撞、枪轴侧约束和节点/边统计）：

```text
nodes = 80
environment collision nodes = 0
minimum actual-wire-axis side dot = 0.996195
maximum side angle error = 5.0000 deg
graph collision-free edges = 372
nodeCollisionFree = 1
edgeCollisionFree = 1
maxJointJumpDeg = 9.953837
```

另外逐层候选状态中最大 TCP 位置误差为 `0.000039 mm`，最大姿态误差为 `0.000003 deg`；保存路径为 `80 × 6`，单位 rad。已用 `animate_weld_actual_axis_replan.m` 播放保存路径，动画执行完成。

## 75. 当前边界与后续注意事项（2026-09-17）

1. 焊缝端点默认各裁掉 25 mm（`endMarginM=0.025`）。这是为了让保守的 V-HACD 凸包避开 x=0/x=L 端壁，并不等同于工艺端部余量；若要焊满全长，需单独重新验证更小端部余量。
2. `qMat0 → Point 1` 的自由空间接近路径尚未规划，`Point 80 → 收焊退出` 也尚未规划。结果明确标记 `approachPathPlanned=false`、`exitPathPlanned=false`；当前 PASS 仅表示 80 个焊点和 79 条焊缝内相邻边通过。
3. `work angle = 45°` 的定义现在基于用户确认的实际焊丝轴目标方向，不再把 TCP `+Z` 当作枪轴。若实际工艺轴与当前焊丝模型不同，必须先重新标定工具几何，再重规划。
4. `publish=false` 是默认值；验证通过后才可显式调用发布脚本发送到现有 `/joint_trajectory`。不要修改稳定的 ROS2 controller。
5. 主规划脚本默认 `animate=true`，若只需快速重算可传入 `struct('animate',false,'publish',false)`；动画也可单独运行：

```bash
matlab -batch "addpath('/home/ttbstt/jixiebi/UR5/matlab仿真'); animate_weld_actual_axis_replan('/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan.mat');"
```

## 76. 轨迹发布接口修正与运行时检查（2026-09-17）

首次调用 `publish_weld_trajectory` 时，MATLAB ROS 2 消息校验指出 `JointTrajectoryPoint.time_from_start.sec` 必须为 `int32`、`nanosec` 必须为 `uint32`。已修正
`/home/ttbstt/jixiebi/UR5/matlab仿真/publish_weld_trajectory.m`：

```matlab
point.time_from_start.sec = int32(...);
point.time_from_start.nanosec = uint32(...);
```

修正后实际发布成功：

```text
Published 80 trajectory points on /joint_trajectory (11.85 s).
```

但随后检查发现当前 CoppeliaSim 进程的运行环境没有加载 `simROS2`：

```text
simROS2 loaded = False
/joint_states Publisher count = 0
```

原因是当前 CoppeliaSim 进程未继承 `/opt/ros/humble/setup.bash` 的 ROS 动态库环境；`ldd /opt/coppeliaSim/libsimROS2.so` 可见多个 ROS 库为 `not found`。尝试通过 Remote API 在当前进程加载插件返回 `-3`，未修改场景或插件文件。当前规划结果仍有效，ROS2 controller 也未修改；要在 CoppeliaSim 中看到实际执行和 `/joint_states` 反馈，应在保存场景后从已 `source /opt/ros/humble/setup.bash` 的终端重新启动 CoppeliaSim，再启动/确认现有 controller，然后重新发布保存的 `qPath`。

## 77. 按最新交接方案制作焊缝视频（2026-09-17）

用户要求依据当前交接文档制作新动画。已在
`/home/ttbstt/jixiebi/UR5/matlab仿真/animate_weld_actual_axis_replan.m` 中保留/完善视频输出功能：

- 读取已验收的 `weld_actual_axis_replan.mat`，不重新求 IK、不修改场景。
- 播放前对 `qPath` 做 `2*pi` 分支 unwrap，避免视频出现整圈跳转；这不改变姿态。
- 显示 UR5、焊枪碰撞模型、四块工件碰撞体、理论焊缝、TCP 拖尾、TCP 点、实际焊丝轴箭头。
- 为避免 1.2 mm 焊丝在全场景中不可见，新增显式枪头到 TCP 的焊丝线段叠加；碰撞圆柱仍由 MATLAB 模型负责，未改变碰撞计算。

新增视频：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_wire_axis_v2.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_wire_axis_top_v2.avi
```

两段视频均为 Motion JPEG AVI，`1394×840`、`30 fps`、约 `4.10 s`，分别为三维视角和俯视角；每段包含 123 个插值回放帧。视频生成日志：

```text
TCP 回放轨迹最大偏离焊缝直线：2.1823 mm
抽查插值帧碰撞：0 / 20
```

运行命令：

```bash
matlab -batch "addpath('/home/ttbstt/jixiebi/UR5/matlab仿真'); animate_weld_actual_axis_replan('/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan.mat',StepDeg=1,Speed=100,SaveVideo=true,VideoFile='weld_actual_axis_replan_wire_axis_v2.avi',View='all');"
matlab -batch "addpath('/home/ttbstt/jixiebi/UR5/matlab仿真'); animate_weld_actual_axis_replan('/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan.mat',StepDeg=1,Speed=100,SaveVideo=true,VideoFile='weld_actual_axis_replan_wire_axis_top_v2.avi',View='allTop');"
```

本视频只验证 MATLAB 规划模型的回放；由于当前 CoppeliaSim 进程仍未加载 `simROS2`，不能把视频视为 CoppeliaSim 实时执行录像。

## 78. 视频颜色恢复为主场景原始配色（2026-09-17）

用户反馈前一版视频颜色不清楚。已对照 `sync_ur5_joint_states.m` 的主场景显示方式，调整
`animate_weld_actual_axis_replan.m` 的全场景视角从 `(-37.5°,25°)` 恢复为主场景使用的 `(135°,25°)`；不再额外覆盖机器人/工件的颜色。焊丝显式叠加线段仍保留，方便观察。

重新生成的视频为：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_original_colors.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_original_colors_top.avi
```

其中第一份是原主场景三维视角，第二份是俯视角；两者均为 `1394×840`、`30 fps`、约 `4.10 s` 的 Motion JPEG AVI。原先的 `*_wire_axis_v2.avi` 文件保留，不作为最新版推荐文件。

## 79. 视频配色问题修正（2026-09-17）

用户指出 `original_colors` 初版仍呈现亮绿色/亮橙色。原因不是场景颜色被修改，而是动画脚本新建了空 Figure，未继承 `sync_ur5_joint_states.m` 创建的光照对象；在无光照的 Figure 中，原始 FaceColor 被直接显示。

已修正 `animate_weld_actual_axis_replan.m`：现在动画先运行 `sync_ur5_joint_states.m`，直接复用其唯一主 Figure/axes、light、材质和工件对象，再叠加焊缝/TCP/焊丝轨迹，不再新建第二套场景。检查结果：动画 Figure 保留 `lights=1`，颜色与同步主场景一致（深绿色受光机器人、棕/橙受光工件）。

最新版视频已重新覆盖生成：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_original_colors.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_original_colors_top.avi
```

两份文件均完成 `VideoReader` 验证：`1394×840`、`30 fps`、`4.10 s`。修复前的亮色视频已保留为：

```text
weld_actual_axis_replan_original_colors.before-light-fix.avi
weld_actual_axis_replan_original_colors_top.before-light-fix.avi
```

俯视视频随后已重新生成并用 `VideoReader` 验证；同时复制了便于点击的 ASCII 文件名：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_top_latest.avi
```

## 80. 增加多视角焊缝动画（2026-09-17）

用户要求增加多个视角，并保证焊缝与焊枪都能在动画中观察。未修改 CoppeliaSim 场景、ROS2 控制链路、标定参数或已通过验证的规划结果。

已备份动画脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/animate_weld_actual_axis_replan.before-seam-visibility-20260917.m.bak
/home/ttbstt/jixiebi/UR5/matlab仿真/animate_weld_actual_axis_replan.before-weld-views-20260917.m.bak
```

`animate_weld_actual_axis_replan.m` 当前仍直接复用 `sync_ur5_joint_states.m` 建立的主 Figure、灯光、材质和原始配色。为使内角焊缝在视频叠加层中尽量清楚，焊缝/TCP/焊丝标注每帧置于标注层顶端；这只影响可视化叠加，不改变碰撞模型。

新增视角参数：

```text
allWeldSide     高位侧视，view(90,55)
allWeld         焊缝内侧高位三维，view(120,55)
allWeldReverse  反向高位三维，view(140,55)
```

本次生成并验证的视频：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_main_multiview.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_top_multiview.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_weldside.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_weld_inside.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_weld_reverse.avi
```

每个文件均为 Motion JPEG AVI，`1394×840`、`30 fps`、`4.10 s`；均来自同一个已保存的 `weld_actual_axis_replan.mat`，播放前对 qPath 做连续分支 unwrap。视频生成时保持 `StepDeg=1`、`Speed=100`，抽查插值帧碰撞仍为 `0/20`，未重新规划。

为方便继续沿用旧入口名，俯视版本已同步到 `weld_actual_axis_replan_top_latest.avi` 和 `weld_actual_axis_replan_original_colors_top.avi`；主三维版本已同步到 `weld_actual_axis_replan_original_colors.avi`。其中俯视角最适合检查整条焊缝，三个高位视角用于从不同方向观察焊枪和焊缝相对位置。

运行示例：

```bash
matlab -batch "addpath('/home/ttbstt/jixiebi/UR5/matlab仿真'); animate_weld_actual_axis_replan('/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan.mat',StepDeg=1,Speed=100,SaveVideo=true,VideoFile='weld_actual_axis_replan_weld_inside.avi',View='allWeld');"
```

## 81. 焊接段 TCP 固定速度回放（2026-09-17）

用户指出原动画在焊枪姿态调整时有短暂停顿，要求焊接过程中 TCP 移动速度固定。未修改 CoppeliaSim 场景、ROS2 控制链路、标定参数或 `weld_actual_axis_replan.mat`。

已备份动画脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/animate_weld_actual_axis_replan.before-fixed-tcp-speed-20260917.m.bak
/home/ttbstt/jixiebi/UR5/matlab仿真/animate_weld_actual_axis_replan.before-speed-quantization-20260917.m.bak
```

`animate_weld_actual_axis_replan.m` 当前采用以下回放时间参数化：

1. 将已验收的连续 `qPath` 做较细的关节空间插值；
2. 通过 FK 计算每个插值姿态的 WeldTCP 位置；
3. 对 TCP 轨迹累积弧长按固定速度重新采样；
4. 迭代 10 次弧长重参数化，消除关节线性插值造成的局部 TCP 速度凹陷；
5. 所有视频帧等时间间隔输出，姿态变化与 TCP 前进同步，不再在焊点处等待较大的关节调整。

默认参数为 `TcpSpeedMps=0.043`（43 mm/s），可按实际焊接工艺覆盖，例如：

```matlab
animate_weld_actual_axis_replan("", ...
    TcpSpeedMps=0.020, ...
    Speed=1, ...
    View="allWeld")
```

固定速度版数值验收（30 fps）：

```text
目标 TCP 速度      : 0.0430 m/s
TCP 轨迹弧长       : 0.177 m
时间参数化时长     : 4.13 s
回放帧数           : 125
TCP 帧速度范围     : 0.042852 .. 0.042965 m/s
TCP 最大焊缝偏离   : 2.3398 mm
抽查插值帧碰撞     : 0 / 20
```

重新生成并通过 `VideoReader` 验证的固定速度视频（均为 Motion JPEG AVI、`1394×840`、`30 fps`、`4.167 s`）：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_fixed_speed_main.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_fixed_speed_top.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_fixed_speed_weldside.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_fixed_speed_weld_inside.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_fixed_speed_weld_reverse.avi
```

为便于从文件名直接识别速度，又生成了带速度标记的同内容副本（`tcp43mmps` = 43 mm/s）：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_tcp43mmps_main.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_tcp43mmps_top.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_tcp43mmps_weldside.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_tcp43mmps_weld_inside.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_tcp43mmps_weld_reverse.avi
```

这些带速度标记的视频均为 `1394×840`、`30 fps`、`4.167 s`（`VideoReader` 报告时长）；固定速度重参数化后的实测 TCP 帧速度为 `0.042852–0.042965 m/s`（目标 `0.043 m/s`）。

常用旧入口已同步为固定速度版本：

```text
weld_actual_axis_replan_original_colors.avi
weld_actual_axis_replan_original_colors_top.avi
weld_actual_axis_replan_top_latest.avi
weld_actual_axis_replan_main_multiview.avi
weld_actual_axis_replan_top_multiview.avi
weld_actual_axis_replan_weldside.avi
weld_actual_axis_replan_weld_inside.avi
weld_actual_axis_replan_weld_reverse.avi
```

覆盖前的旧视频均保存在同名的
`.before-fixed-tcp-speed-20260917.avi` 备份中。

恒速弧长量化改进后，常用入口又同步到最新版本；改进前入口保存在同名的
`.before-speed-quantization-20260917.avi` 备份中。

重要边界：本次只修正 MATLAB 动画的时间参数化。`publish_weld_trajectory.m` 和 ROS2 控制器尚未改为按 TCP 弧长重新分配 `time_from_start`；因此若要求 CoppeliaSim 实际执行时也保持固定 TCP 速度，下一步需要在发布器中复用同一套场景 FK/弧长时间参数化，再发送带时间戳的 `JointTrajectory`，不能仅依赖当前视频时序。

## 82. 端部余量为 0 的可达性检查（2026-09-17）

用户询问当前焊缝两端的 `endMarginM` 是否可以改为 0。当前已验收规划使用 `endMarginM = 0.025 m`，即从两端内壁各缩进 25 mm；这不是 TCP standoff，也没有修改 TCP 标定。

新增只读诊断脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/check_end_margin_zero_reachability.m
```

运行：

```matlab
addpath('/home/ttbstt/jixiebi/UR5/matlab仿真');
report = check_end_margin_zero_reachability();
```

诊断将当前保存路径的两端分别延伸 25 mm 到工件底板/侧板的内边界，并保持原规划端点 TCP 姿态，使用现有 UR5、GunCollisionHull、焊丝碰撞圆柱和四块 Workpiece 碰撞体检查。

结果：

```text
零余量起点 TCP : [-0.60500 -0.07000 0.00500] m
零余量终点 TCP : [-0.60500  0.15500 0.00500] m

起点：98 次 IK，67 个位姿误差合格；67 个全部被环境碰撞拒绝；可行状态 0
终点：98 次 IK，59 个位姿误差合格；59 个全部被环境碰撞拒绝；可行状态 0

ZERO-MARGIN ENDPOINT TEST: FAIL
```

因此结论是：**机械臂的 TCP 位姿从纯 IK 角度可以求到，但在完整焊枪凸包/焊丝与工件硬碰撞约束下，端部余量为 0 的当前姿态不可行。** 当前 `replan_weld_actual_axis.m` 的注释也明确指出，导入的保守 V-HACD 凸包需要约 25 mm 端部裁剪来避开 `wall_x0/wall_x1` 端壁。

## 83. 焊缝路径时间说明（2026-09-17）

需要区分两个时间：

1. **焊接运动执行时间**：当前 TCP 轨迹弧长约 `0.177 m`，动画目标 TCP 速度为 `0.043 m/s`，时间参数化结果为约 `4.13 s`；30 fps 视频由 125 帧组成，`VideoReader` 报告时长约 `4.167 s`。
2. **规划算法计算耗时**：旧版 `weld_actual_axis_replan.mat` 未保存 elapsed time，不能用 4.13 s 视频时长代替规划计算耗时。后续已加入算法区间计时，并在第 87 节给出实际测量值。

零余量测试没有覆盖所有可能的工作角/行进角/roll 姿态，也没有覆盖完整 80 点重新规划；但现有无余量全规划的第 1 层粗姿态和细姿态搜索均为 0 状态，已足以说明不能直接把当前验收路径的 `endMarginM` 改成 0。若以后要减小余量，应从 25 mm 向下做独立参数扫描，并同时报告端点状态、端点邻接边和碰撞来源；不要覆盖当前 `weld_actual_axis_replan.mat`。

## 84. RRT-Connect 接近/退出路径（2026-09-17）

在不修改稳定 ROS2 控制链路、CoppeliaSim 场景、TCP 标定和已验收 80 点焊缝的前提下，新增独立的 MATLAB RRT-Connect 阶段。现有 `weld_actual_axis_replan.mat` 仅作为只读输入；80 点 `qWeld` 没有重新规划。

新增脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/plan_weld_approach_exit_rrt_connect.m
/home/ttbstt/jixiebi/UR5/matlab仿真/animate_weld_rrt_connect_sequence.m
```

规划脚本首先读取当前规划场景和 `weld_actual_axis_replan.mat`，使用 Point 1/80 的 TCP 位姿及当前真实焊丝轴生成预焊/退出候选。沿工作轴反方向尝试 30、50、75、100、125 mm 候选；两端均选中 30 mm 候选。预焊和退出候选均经过多 IK、关节限位、完整节点碰撞和端点插值边碰撞检查：

```text
PreWeld  30.0 mm: pose-valid=8, state-free=2, edge-free=1
PostWeld 30.0 mm: pose-valid=8, state-free=2, edge-free=1
```

RRT-Connect 在 6D MATLAB 关节空间中使用双树交替扩展、目标偏置、固定随机种子 `20260917`、步长 `12 deg`、最大迭代次数 `8000`，每个节点和插值边均检查 UR5/工具/焊丝/工件及自碰撞。默认结果：

```text
Path A: qHome -> qPreWeld
  success=1
  planning time       = 11.898240 s
  iterations          = 31
  samples             = 31
  tree nodes          = 32
  collision checks    = 317
  edge checks         = 60
  path length         = 19.842372 rad
  max joint jump      = 12.000000 deg
  dense validation    = 21 nodes / 20 edges, node-free=1, edge-free=1

Path B: qPostWeld -> qHome
  success=1
  planning time       = 6.752958 s
  iterations          = 11
  samples             = 11
  tree nodes          = 22
  collision checks    = 181
  edge checks         = 30
  path length         = 17.255930 rad
  max joint jump      = 12.000000 deg
  dense validation    = 19 nodes / 18 edges, node-free=1, edge-free=1
```

五段端点连续性也已检查：`Home -> PreWeld`、`PreWeld -> Point 1`、已有 `Point 1 -> Point 80`、`Point 80 -> PostWeld`、`PostWeld -> Home`。PreWeld→Point 1 最大关节跳变为 `3.279 deg`，Point 80→PostWeld 为 `3.170 deg`。最终结果为：

```text
Approach RRT PASS      = 1
PreWeld -> Point 1     = 1
Existing weld 1 -> 80  = 1 (read-only, not replanned)
Point 80 -> PostWeld   = 1
Exit RRT PASS          = 1
Overall PASS           = 1
```

保存结果和报告：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_approach_exit.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_approach_exit_report.txt
```

MAT 文件包含 `qHome`、`preWeld.q`、`postWeld.q`、两棵 RRT 的路径和统计、端点连接结果、原始 `qWeld` 及完整验证结果。文本报告记录两条路径的时间、迭代、采样、节点、碰撞检查、边检查、路径长度和高密度复核结果。

新增 MATLAB-only 五段动画（不发送 ROS2）：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_tcp43mmps_main.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_tcp43mmps_top.avi
```

动画顺序为 `Home → PreWeld → Point 1 → Point 80 → PostWeld → Home`，显示原规划场景颜色、UR5、GunCollisionHull、MATLAB 焊丝、工件、理论焊缝和 TCP 轨迹。自由空间段按关节插值；焊接段沿 TCP 弧长重参数化，目标 TCP 速度 `0.043 m/s`（文件名 `tcp43mmps`）。两视频均已用 `VideoReader` 验证为 `1394×840`、`30 fps`、`20.200 s`，共 `606` 帧；焊接段实测帧速度 `0.0429–0.0430 m/s`。

运行示例：

```matlab
addpath('/home/ttbstt/jixiebi/UR5/matlab仿真');
result = plan_weld_approach_exit_rrt_connect();
out = animate_weld_rrt_connect_sequence( ...
    '/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_approach_exit.mat', ...
    struct('saveVideo',true,'view','all'));
```

俯视动画可将 `view` 改为 `allTop`。本阶段只做 MATLAB 可视化和结果保存，未发送 `/joint_trajectory` 或其它 ROS2 指令。

## 85. RRT-Connect 动画隐藏 TCP 方向箭头（2026-09-17）

用户指出动画中的橙色 TCP 方向箭头容易被误认为焊丝。已在 `animate_weld_rrt_connect_sequence.m` 中新增选项：

```matlab
showTcpAxis = false   % 默认隐藏
```

现在动画仅显示真实的 MATLAB 焊丝线段/焊丝端点，不再显示 TCP 方向箭头；焊枪、GunCollisionHull、工件、理论焊缝和 TCP 轨迹保持不变。若诊断时确实需要辅助箭头，可显式传入：

```matlab
struct('showTcpAxis',true)
```

已重新生成并用 `VideoReader` 验证以下同名视频：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_tcp43mmps_main.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_tcp43mmps_top.avi
```

两者均为 `1394×840`、`30 fps`、`20.200 s`、606 帧，焊接段 TCP 帧速度仍为 `0.0429–0.0430 m/s`。RRT 结果 MAT、统计报告和原 80 点焊缝路径未改变。

## 86. RRT-Connect 新增两种焊枪/工件视角（2026-09-17）

用户要求增加能够同时看到完整焊枪和工件的视角。基于同一份只读 RRT-Connect 结果重新生成了两种高位斜视动画；保持原场景颜色、焊丝显示、焊缝、TCP 轨迹和 `showTcpAxis=false` 设置不变：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_tcp43mmps_weldside.avi
    view = allWeldSide, view(90,55)

/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_tcp43mmps_weldreverse.avi
    view = allWeldReverse, view(140,55)
```

两份视频均已用 `VideoReader` 验证为 `1394×840`、`30 fps`、`20.200 s`、606 帧；焊接段 TCP 帧速度保持 `0.0429–0.0430 m/s`。两种视角都包含 UR5、焊枪、焊丝、工件和理论焊缝。未修改规划结果、ROS2 链路、CoppeliaSim 场景或 TCP 标定。

## 87. 80 点焊缝纯算法计算时间（2026-09-17）

用户要求只统计规划算法计算时间，不包含视频播放时间。已新增：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/time_weld_actual_axis_replan.m
```

同时在 `replan_weld_actual_axis.m` 中加入 `result.algorithmElapsedSec`。计时起点位于规划场景建立完成、开始焊缝生成之后；计时终点位于粗/细姿态候选 IK、碰撞检测、层图构建和最终路径搜索结束之后。因此不包含：MATLAB 启动、场景/机器人/碰撞模型加载、动画、ROS2 发布和结果写盘。

使用固定随机种子 `20260917`、当前默认规划参数、80 个焊点，关闭动画和 ROS2 发布后，独立重跑得到：

```text
纯规划算法时间 = 454.006036 s
                 ≈ 7 min 34.006 s

焊点数量       = 80
IK 状态层      = 80 / 80
最终可达状态   = 2
节点无碰撞     = 1
边无碰撞       = 1
最大关节跳变   = 9.953837 deg
```

本次重跑结果为独立文件，未覆盖原始验收结果：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_timed_20260917.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_timing.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_timing_report.txt
```

计时复现实例：

```matlab
addpath('/home/ttbstt/jixiebi/UR5/matlab仿真');
timing = time_weld_actual_axis_replan();
```

注意：这次统计的是“重新规划一条 80 点焊缝”的算法耗时，不是 RRT-Connect 接近/退出路径的耗时，也不是 AVI 播放时长。

## 88. RRT-Connect 完整路径发送到 CoppeliaSim（2026-09-17）

用户要求将完整路径导入 CoppeliaSim。已新增独立发布脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/publish_rrt_connect_full_path.m
```

脚本读取只读结果 `weld_rrt_connect_approach_exit.mat`，按以下五段拼接并去除重复端点：

```text
Home -> PreWeld
PreWeld -> Point 1
Point 1 -> Point 80       （原 80 点焊缝，未重新规划）
Point 80 -> PostWeld
PostWeld -> Home
```

关键执行处理：

1. 规划结果是 MATLAB 关节坐标，发送前使用冻结的 `qOffset` 转换为 CoppeliaSim 关节坐标：`qCop = qMat - qOffset`。
2. 对每个关节做连续 `2π` 分支展开，避免控制器对 RRT 路径中的等价角度产生几百度的错误直线插值。
3. 自由空间 RRT 段使用每边 `0.15 s`；PreWeld/Point 1 和 Point 80/PostWeld 按 TCP 速度参数计算；焊接段目标 TCP 速度为 `0.043 m/s`。
4. 发送前检查首尾 Home、轨迹点时间严格递增和六个标准 UR5 关节名。

发送前干运行验证：

```text
轨迹点数                  = 120
总执行时间                = 16.141291 s
qCop 起点                 = [0 0 0 0 0 0] deg
qCop 终点                 = [360 0 0 0 0 0] deg
终点按 2π 等价 Home       = [0 0 0 0 0 0] deg
最大相邻 qCop 跳变        = 12 deg
```

已在当前 ROS2 控制器运行、`send_demo:=false` 的条件下实际发送：

```text
Published complete path: 120 points, duration 16.141 s.
Sent = 1
```

命令归档：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_full_path_command.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_full_path_command_report.txt
```

复现命令：

```matlab
addpath('/home/ttbstt/jixiebi/UR5/matlab仿真');
out = publish_rrt_connect_full_path('',struct('send',true));
```

本次未修改 `controller.py`、CoppeliaSim 场景、TCP 标定或原始焊缝结果。当前检查到 `/joint_trajectory` 的控制器发布/订阅端均存在；但当前 ROS 图中 `/joint_states` 的 CoppeliaSim 发布器计数为 0，因此 ROS 反馈暂时不能作为运动完成确认。若 CoppeliaSim 画面没有运动，应先确认仿真处于运行状态，并从已加载 ROS2 环境的终端启动 CoppeliaSim 的 `simROS2` 发布器后再发送同一命令。

## 89. simROS2 依赖库启动修复（2026-09-17）

当前报错：

```text
plugin 'simROS2': load failed (could not load)
The plugin probably couldn't load dependency libraries.
```

根因已确认：当前 CoppeliaSim 进程不是从 `source /opt/ros/humble/setup.bash` 的环境启动的。未加载 ROS 环境时，`ldd /opt/coppeliaSim/libsimROS2.so` 会将若干 ROS 库列为 `not found`；加载 ROS2 环境并补充 CoppeliaSim、ROS2 library paths 后，依赖检查为：

```text
simROS2 dependency check: OK (no not found entries)
```

新增启动器：

```text
/home/ttbstt/jixiebi/UR5/start_coppeliasim_ros2.sh
```

使用前先在当前 CoppeliaSim 中保存场景，然后关闭当前 CoppeliaSim 和旧 controller，避免未保存改动丢失。重新启动：

```bash
cd /home/ttbstt/jixiebi/UR5
./start_coppeliasim_ros2.sh
```

启动器自动执行：

```text
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=0（若未设置）
ROS_LOCALHOST_ONLY=0（若未设置）
RMW_IMPLEMENTATION=rmw_fastrtps_cpp（若未设置）
设置 simROS2 所需的 LD_LIBRARY_PATH
打开 /home/ttbstt/jixiebi/UR5/ur5.ttt
```

然后在另一个终端启动 controller：

```bash
source /opt/ros/humble/setup.bash
cd /home/ttbstt/jixiebi/ros2_ws
source install/setup.bash
ros2 run ur5_controller controller --ros-args -p send_demo:=false
```

启动仿真后检查：

```bash
source /opt/ros/humble/setup.bash
ros2 topic info /joint_states -v
```

正常时 `/joint_states` 应为 `Publisher count: 1`。确认后再用第 88 节的 `publish_rrt_connect_full_path(...,struct('send',true))` 发送完整路径。该修复只改变 CoppeliaSim 启动环境，没有修改 `libsimROS2.so`、CoppeliaSim 场景、TCP 标定、controller 源码或 MATLAB 规划结果。

## 90. 启动器兼容 ROS2 setup.bash（2026-09-17）

首次执行 `start_coppeliasim_ros2.sh` 时，ROS2 setup 报：

```text
/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
```

原因是启动器在 `source /opt/ros/humble/setup.bash` 之前启用了 `set -u`，而 ROS2 setup 脚本会读取尚未定义的变量。已将启动器改为 `set -eo pipefail`，保留错误检查但关闭未定义变量即报错的模式。

验证结果：

```text
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
simROS2 dependencies: OK
```

现在可重新执行：

```bash
cd /home/ttbstt/jixiebi/UR5
./start_coppeliasim_ros2.sh

## 91. 当前 CoppeliaSim 执行验证与脚本清理边界（2026-09-17）

用户自行启动的 CoppeliaSim 进程为：

```text
/opt/coppeliaSim/coppeliaSim -f /home/ttbstt/jixiebi/UR5/ur5.ttt
```

`simROS2` 已正常加载，ROS 图检查为：

```text
/sim_ros2_interface
/ur5_trajectory_controller
/joint_states: Publisher count = 1
```

发现两个同名 controller 后，已停止在当前 CoppeliaSim 启动前运行的旧 controller，只保留当前 controller，避免两套控制器同时接收轨迹。

重新发送已验证的完整 120 点轨迹：

```matlab
addpath('/home/ttbstt/jixiebi/UR5/matlab仿真');
out = publish_rrt_connect_full_path('', struct('send', true));
```

发布结果：

```text
Published complete path: 120 points, duration 16.141 s.
Sent = 1
```

实时 `/joint_states` 监测显示六轴均发生实际运动，之后继续等待稳态，最终回到 Home：

```text
POST q modulo home deg = [0 0 0 0 0 0]
```

因此当前结论是：完整路径已被 CoppeliaSim 接收并执行，且最终回 Home；未修改原始 80 点焊缝结果、TCP 标定、CoppeliaSim 场景或稳定 ROS2 controller 源码。

关于“删除没用脚本”：目前没有执行永久删除，因为“没用”没有给出明确文件清单。当前完整路径执行至少依赖：

```text
matlab仿真/sync_ur5_joint_states.m
matlab仿真/publish_rrt_connect_full_path.m
matlab仿真/weld_rrt_connect_approach_exit.mat
ros2_ws/src/ur5_controller/ur5_controller/controller.py（稳定控制链路）
ur5.ttt、export_scene_geometry.py、start_coppeliasim_ros2.sh（场景/启动辅助）
```

旧诊断、重新规划和测试脚本应先明确列出后再归档或删除，不能凭文件名直接删除规划结果或控制依赖。

## 92. CoppeliaSim 碰撞检测现状（2026-09-17）

CoppeliaSim 本身支持通过 Remote API 显式调用 `sim.checkCollision(entityA, entityB)`。当前运行场景中已实测：

```text
sim.checkCollision(/UR5/connection/WeldingGunMount/GunCollisionHull, /Workpiece)
    -> 0 (当前姿态无碰撞)
sim.checkCollision(/UR5, /Workpiece)
    -> 0 (当前姿态无碰撞)
```

当前场景树中 `Workpiece` 是一个单独的 CoppeliaSim shape；MATLAB 使用的四个 `WorkpieceCollision_*` box 是规划模型中的环境对象，并不在 CoppeliaSim 场景树内。`GunCollisionHull` 在场景树中存在，并为 compound hull。

对象属性检查：

```text
GunCollisionHull: respondable = 0, static = 1
Workpiece:       respondable = 0, static = 1
```

因此当前不是“物理引擎自动碰撞停止”模式：CoppeliaSim 有碰撞检测能力，但场景没有配置自动碰撞监视/停止逻辑，且这些对象不参与动态接触响应。现阶段完整路径的碰撞安全性来自 MATLAB 的 `checkCollision` 预验证；ROS2 controller 只执行轨迹，不做碰撞判定。

如需在 CoppeliaSim 播放时实时报警或停机，应另加独立的只读碰撞监视脚本，显式调用 `sim.checkCollision`，不必修改现有 ROS2 控制链路。

## 93. CoppeliaSim 轨迹播放与实时碰撞监视（2026-09-17）

新增独立监视脚本：

```text
/home/ttbstt/jixiebi/UR5/monitor_coppelia_collision.py
```

它通过 ZMQ Remote API 检查：

```text
/UR5 ↔ /Workpiece
/UR5/connection/WeldingGunMount/GunCollisionHull ↔ /Workpiece
```

当前姿态单次检查结果为无碰撞。焊丝仍是 MATLAB-only `collisionCylinder`，因此该脚本不会检查焊丝；焊丝碰撞仍由 MATLAB 规划验证负责。

单次检查：

```bash
cd /home/ttbstt/jixiebi/UR5
PYTHONPATH=/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src \
python3 monitor_coppelia_collision.py --once
```

持续监视：

```bash
PYTHONPATH=/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src \
python3 monitor_coppelia_collision.py
```

检测到 UR5 或焊枪凸包与工件碰撞时自动暂停仿真：

```bash
PYTHONPATH=/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src \
python3 monitor_coppelia_collision.py --stop-on-collision
```

注意上面路径的实际目录名为 `zmqRemoteApi`（大小写为 `Api`）。

重新执行已保存的完整 120 点轨迹，不需要重新规划：

1. CoppeliaSim 打开 `/home/ttbstt/jixiebi/UR5/ur5.ttt`，点击运行并确认 `simROS2` 发布 `/joint_states`。
2. 新终端启动一个且仅一个 controller：

```bash
source /opt/ros/humble/setup.bash
cd /home/ttbstt/jixiebi/ros2_ws
source install/setup.bash
ros2 run ur5_controller controller --ros-args -p send_demo:=false
```

3. MATLAB 执行：

```matlab
addpath('/home/ttbstt/jixiebi/UR5/matlab仿真');
out = publish_rrt_connect_full_path('', struct('send', true));
```

该命令读取 `weld_rrt_connect_approach_exit.mat`，拼接 `Home → PreWeld → Point 1–80 → PostWeld → Home`，并发布到 `/joint_trajectory`。CoppeliaSim 内部没有直接运行轨迹的 Lua 命令；轨迹由 ROS2 controller 接收后通过 ZMQ Remote API 驱动关节。

## 94. CoppeliaSim 播放速度核对（2026-09-17）

已核对当前发布器和命令 MAT：没有修改原始关节路径 `qPath`、`controller.py`、CoppeliaSim 场景或 MATLAB 动画的 `speed=1.0` 参数。当前发布器的时间参数为：

```text
weldSpeedMps       = 0.043 m/s（43 mm/s）
transitionSpeedMps = 0.043 m/s
freeSegmentDuration = 0.15 s
```

当前 120 点命令的总时间为 `16.141291 s`。其中焊接段相邻点时间约 `0.051516 s`，焊接段累计约 `4.716 s`；自由空间段使用 `0.15 s/段`，较长的 PostWeld 段约 `5.126 s`。

旧版 `publish_weld_trajectory.m` 的默认规则是所有点统一 `0.15 s/段`，若用于当前 120 点则总时长为 `17.850 s`。因此当前版本相对“所有点 0.15 s”整体略快，并且焊接段明显更快；这是时间参数化规则不同造成的，不是修改了保存的关节路径。

还需注意：ROS2 controller 只在 50 Hz 插值 `JointTrajectory` 并调用 `setJointTargetPosition`。CoppeliaSim 的力控关节会受到自身速度/加速度响应影响，所以实际画面可能出现加速、减速或短暂跟随滞后；发布器中的 `43 mm/s` 是目标时间参数，不等于动力学执行后的严格实测 TCP 速度。

如果需要恢复旧版速度，应明确选择：

```text
A. 所有轨迹点固定 0.15 s/段（旧版时序）；
B. 保持焊接 TCP 43 mm/s，仅调整自由空间段速度；
C. 重新按 TCP 弧长密集采样并限制关节速度/加速度，使 CoppeliaSim 实测速度更接近恒定。
```

## 95. 恢复完整轨迹慢速播放（2026-09-17）

用户选择恢复旧版慢速效果。已修改：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/publish_rrt_connect_full_path.m
```

当前默认：

```text
timingMode = "fixed"
fixedSegmentDuration = 0.15 s
```

即完整 120 点轨迹的每个相邻点固定间隔 `0.15 s`，总时长已验证为：

```text
17.850000 s
```

MATLAB 干运行验证通过，所有 `segmentDurationSec` 均为 `0.15`。原有 `qPath`、规划 MAT、ROS2 controller 和 CoppeliaSim 场景未修改。

重新发送慢速轨迹：

```matlab
addpath('/home/ttbstt/jixiebi/UR5/matlab仿真');
out = publish_rrt_connect_full_path('', struct('send', true));
```

若以后需要恢复 43 mm/s 时间参数，可显式使用：

```matlab
out = publish_rrt_connect_full_path('', ...
    struct('send', true, 'timingMode', 'tcp_speed'));
```

## 96. 真实 UR5 CB3 External Control URCap（2026-09-17）

用户确认第一步 `ros-humble-ur` 已完成，进入真实机器人第二步。当前 Ubuntu 本机已提供兼容 CB3/PolyScope 3.15.4 的文件：

```text
/opt/ros/humble/share/ur_robot_driver/resources/externalcontrol-1.0.5.urcap
```

ROS 2 Humble 官方 CB3 安装说明要求使用 `externalcontrol-1.0.5.urcap`，最低 PolyScope 版本为 3.7；当前 PolyScope 3.15.4 满足该要求。安装只能在真实 UR5 示教器完成，CoppeliaSim 仿真不需要 URCap。

示教器操作：

```text
Setup Robot → URCaps → + → 选择 externalcontrol-1.0.5.urcap → Open → 重启
```

重启后：

```text
Program Robot → Installation → External Control
```

配置运行 ROS 驱动的电脑 IP；自定义端口先保持默认。随后在程序树中插入 `External Control` 节点，确认设置后保存程序。当前尚未连接真实机器人、尚未发送轨迹。

## 97. External Control URCap 已复制到 U 盘（2026-09-17）

已识别 U 盘：

```text
/media/ttbstt/8 GB
```

已将兼容文件复制到 U 盘的 UR5 文件夹：

```text
/media/ttbstt/8 GB/UR5/externalcontrol-1.0.5.urcap
```

源文件与 U 盘文件 SHA-256 一致：

```text
21ee6c9b4d8a64cf47ce2446233627deec0d2999a0440f4261155d79223b4901
```

文件大小为 `35287` bytes。下一步需在真实 UR5 示教器上按第 96 节安装，安装后重启示教器并配置 External Control 的电脑 IP。

## 98. External Control 网络字段填写说明（2026-09-17）

本机当前检测到的 IPv4 地址：

```text
wlp9s0: 10.1.233.33/21
Meta:   198.18.0.1/30（虚拟接口，不使用）
```

External Control 的 `Host IP/电脑 IP` 必须填写“与机器人实际连接的那块电脑网卡”的地址。如果机器人位于 `10.1.232.x/21` 网络，则填 `10.1.233.33`；如果机器人 IP 仍为 `192.168.0.10`，则当前 `10.1.233.33` 不能直接使用，电脑还需要在连接机器人网口上配置同网段地址（例如 `192.168.0.20/24`），并以 `ping 192.168.0.10` 验证。

External Control `Custom Port/自定义端口` 保持默认值；当前 ROS 2 驱动的 External Control URCap 默认对应 `50002`。不要把机器人 IP 填到 Host IP，也不要选择 `Meta` 虚拟接口地址。网络配置完成后再在示教器保存 Installation 设置。

## 99. 真机上电后只读状态验证（2026-09-18）

用户确认真机已上电。网络检查：

```text
ping 192.168.0.10：3/3 成功，0% packet loss
```

已启动官方 ROS 2 驱动进行只读状态连接，使用：

```text
ur_type:=ur5
robot_ip:=192.168.0.10
kinematics_params_file:=/home/ttbstt/jixiebi/UR5_real/ur5_calibration.yaml
launch_rviz:=false
launch_dashboard_client:=false
activate_joint_controller:=false
```

驱动日志确认：

```text
URControl version 3.15.4.0
RTDE protocol version 2
Calibration checksum checked successfully
System successfully started
```

`/joint_states` 已由真机 `joint_state_broadcaster` 发布，当前一次采样（按关节名称重排）为：

```text
q = [-4.087664, -1.010111, -2.155438, -0.488511, 1.523536, -3.159206] rad
q = [-234.206, -57.875, -123.497, -27.990, 87.292, -181.009] deg
```

原始消息顺序不是标准六轴顺序，名称映射必须保留。当前只激活状态/诊断类 broadcaster，轨迹控制器保持 inactive；未启动 External Control 程序，未发送任何运动指令。下一步应在示教器对照这组关节角，确认显示值和驱动反馈一致后再做真机控制准备。

## 100. 仿真中 Workpiece 与 UR5_base 的相对位置（2026-09-18）

当前 `scene_geometry.json` 记录的 CoppeliaSim/MATLAB 仿真几何关系为：

```text
UR5_base 原点（world）： [ 0.550000, -0.875000,  0.014650 ] m
Workpiece 原点（world）： [ 0.475000, -0.500000,  0.000000 ] m
```

因此 Workpiece 原点相对 `UR5_base` 为：

```text
[-0.075000, 0.375000, -0.014650] m
[-75.0, 375.0, -14.65] mm
```

Workpiece 相对 UR5_base 的姿态为单位旋转，即两者坐标轴平行。当前 U 型工件的整体尺寸约为 `235 × 235 × 235 mm`，开口方向为工件局部 `y=0` 一侧。

由于 Workpiece 原点位于几何包络的角点附近，按当前四个 MATLAB 碰撞盒计算，工件包络在 `UR5_base` 中约为：

```text
x: -75.0 ... 160.0 mm
y: 375.0 ... 610.0 mm
z: -14.65 ... 220.35 mm
```

这些数值只适用于当前 CoppeliaSim/MATLAB 仿真场景，不代表真实 UR5 焊接现场的工件位置；真机现场需要用实际工装坐标或测量重新标定。

## 101. 真机 FK、TCP 与现有 RRT 起点诊断（2026-09-18）

新增只读诊断脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_real_ur5_fk_and_rrt_start.m
```

脚本读取真机 `/joint_states`、`/tcp_pose_broadcaster/pose`，不发送轨迹指令。
真机关节按名称映射为：

```text
[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
```

本次读取到的真机 q（ROS/URDF 坐标，单位 deg）约为：

```text
[-234.207, -57.875, -123.497, -27.990, 87.290, -181.003]
```

结论 1：真机 q 直接输入 MATLAB `universalUR5`，`base_link -> tool0` FK 与 ROS TF 一致：

```text
MATLAB tool0 position ≈ [0.0745, -0.2966, 0.4819] m
```

真机 q 不应加 CoppeliaSim 专用的 `qOffset`。

结论 2：驱动 TCP 话题使用 `base`，MATLAB 根坐标使用 `base_link`；已按 ROS TF 中的 `base -> base_link = Rz(pi)` 转换后比较。当前驱动 TCP 与项目 `T_tool0_mount*T_mount_tcp` 预测的 WeldTCP 相差约：

```text
位置误差       ≈ 54.8 mm
姿态误差       ≈ 178 deg
```

因此当前不能认为真机示教器/驱动 TCP 与项目 WeldTCP 是同一个工具坐标系。必须在示教器确认当前激活 TCP 的名称、位置和姿态，并与 `T_tool0_mount*T_mount_tcp` 对齐后，才能用于真机焊接轨迹。

结论 3：现有 RRT 文件 `weld_rrt_connect_approach_exit.mat` 的起点为：

```text
qPathStart = [180, -90, 0, -90, 0, 0] deg
```

与当前真机姿态的最短关节差约为：

```text
[-54.207, 32.125, -123.497, 62.010, 87.290, 178.997] deg
max jump ≈ 178.997 deg
2-norm ≈ 250.453 deg
```

所以现有 RRT 路径不能直接从当前真机姿态开始执行；需要从真实当前 q 重新规划接近路径，或先在安全条件下将真机移动到规划起点。当前未发送任何运动指令。

## 102. 焊枪头到 WeldTCP 的当前模型距离（2026-09-18）

运行 `sync_ur5_joint_states.m` 的当前输出为：

```text
Nearest hull head gap   : 20.8 mm (geometry estimate)
Wire visual length      : 18.0 mm
Wire-to-TCP remaining gap: 2.8 mm
```

因此当前 MATLAB 工具模型定义为：

```text
焊枪凸包头端 → 焊丝末端 = 18.0 mm
焊丝末端 → WeldTCP       = 2.8 mm
焊枪凸包头端 → WeldTCP   ≈ 20.8 mm
```

所以不能说当前模型严格等于 `22 mm`；按当前 V-HACD/凸包几何估计约为 `20.8 mm`，通常可近似说约 `21 mm`。这个 20.8 mm 是“凸包中最接近 TCP 的顶点”到 TCP 的估计，不一定等于真实喷嘴金属端面的精确机械尺寸。若要确认真实值，需要在 CoppeliaSim 中读取实际喷嘴端点或用实物测量后更新工具标定。

## 103. 真机激活 TCP 只读诊断（2026-09-18）

新增只读工具：

```text
/home/ttbstt/jixiebi/UR5/read_robot_description_readonly.py
/home/ttbstt/jixiebi/UR5/real_robot_description_calibrated.urdf
/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_real_ur5_fk_and_rrt_start.m
/home/ttbstt/jixiebi/UR5/matlab仿真/real_ur5_tcp_diagnostic_report.txt
```

`real_robot_description_calibrated.urdf` 是从运行中的 ROS2 `/robot_description` 只读导出的；该 URDF 由驱动启动参数 `/home/ttbstt/ur5_calibration.yaml` 生成。没有调用运动、TCP 设置、负载或轨迹服务。

最终读取到的真机关节（ROS/URDF 顺序，deg）约为：

```text
[-223.390, -65.872, -127.124, -14.613, 85.182, -157.972]
```

使用真实标定 URDF 在 MATLAB 计算得到 `base_link -> tool0`：

```text
position  = [0.1559, -0.3087, 0.4354] m
quaternion [w x y z] = [0.4481, 0.8395, 0.1603, 0.2620]
```

ROS URDF 中 `flange -> tool0` 是固定变换（零平移，固定旋转）；`tool0` 是 ROS 的 all-zeros tool frame，不等于示教器当前激活 TCP。

示教器照片给出的 TCP 偏移（相对 tool0，位置 mm、旋转向量 rad）为：

```text
[13.53, 17.13, 379.83, -0.9562, 2.4566, 1.2418]
```

旋转向量模长约 `166.959 deg`，不是 XYZ 欧拉角。由实时 `/tcp_pose_broadcaster/pose` 和 `tool0` 反推出的实际 TCP 偏移为：

```text
position ≈ [13.5, 17.0, 379.9] mm
quaternion [w x y z] ≈ [0.1136, -0.3260, 0.8376, 0.4234]
```

与照片 TCP 参数的误差为：

```text
position ≈ 0.10 mm
orientation ≈ 0.009 deg
```

因此照片参数就是当前真机激活 TCP，且当前 `base`/`base_link` 参考系转换正确。

当前 MATLAB 旧工具链 `T_tool0_mount*T_mount_tcp` 为：

```text
position  = [19.5, 66.7, 375.9] mm
quaternion [w x y z] = [0.7071, 0, 0, -0.7071]
```

它与真机照片 TCP 相差约：

```text
50.06 mm
154.687 deg
```

所以此前约 `54.8 mm / 178 deg` 的问题主要不是 ROS `base`/`base_link` 参考系，而是 MATLAB 旧工具安装组合变换与真实 TCP 不一致。仅凭当前数据不能唯一判断错误全部来自 `T_tool0_mount` 还是 `T_mount_tcp`；能确定的是两者的组合必须重建。

真实 TCP 导入 MATLAB 的安全方案：

```matlab
T_tool0_tcp_real = poseVectorToTform([ ...
    13.53e-3, 17.13e-3, 379.83e-3, ...
    -0.9562, 2.4566, 1.2418]);
```

先单独保存为真实 TCP 标定变量，不覆盖现有 `ur5_tool_calibration.mat`。只有在实物安装坐标 `T_tool0_mount_real` 被测量或 CAD 验证后，才可计算：

```matlab
T_mount_tcp_real = mldivide(T_tool0_mount_real,T_tool0_tcp_real);
```

当前软件数据不能唯一确定真实焊丝轴线。还需要测量或提供：喷嘴出口点、焊丝末端点在 tool0/TCP 坐标中的位置，焊丝伸出长度和直径，以及 TCP 哪一根轴与焊丝轴重合（含正负方向）。

## 104. 80点焊缝规划性能剖析与稳定优化（2026-09-23）

本阶段目标是定位约 450 秒规划耗时，并在不接触真机、不发布 ROS2 指令的前提下，对现有 80 点 `replan_weld_actual_axis` 做可复现性能对比。

### 性能剖析结论

新鲜 MATLAB profiler 基线：

```text
algorithm time = 672.732 s
wall time      = 676.762 s
```

历史已验收结果（作为实际 450 秒基线）：

```text
weld_actual_axis_replan_timed_20260917.mat
algorithm time = 454.006 s
```

主要耗时来自重复 IK，而不是分层图边检测：

```text
localGenerateLayer                         658.869 s
generate_actual_axis_ik_candidates         658.441 s
inverseKinematics.stepImpl                 292.792 s
inverseKinematics.solve                    288.084 s
rigidBody.rigidBody construction           194.318 s
IKHelpers.computeCost                      188.465 s
localJointLimits                           168.784 s
inverseKinematics constructor              135.267 s
```

### 稳定优化实现

修改但保持默认行为不变：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/replan_weld_actual_axis.m
/home/ttbstt/jixiebi/UR5/matlab仿真/generate_actual_axis_ik_candidates.m
```

增加了三个可选优化：

1. `reuseIKSolver=true`：每次运行只构造一个 `inverseKinematics` 对象，并复用一次计算的关节限位。
2. `reuseCandidateCache=true`：对细化阶段重复出现的完全相同目标姿态缓存已接受候选 IK 状态。
3. `lazyCollision=true`：节点碰撞仍立即检查；只有分层图边的内部插值样本延后到边测试时检查，避免重复做节点碰撞。

默认配置仍是 `false`，因此既有调用不变。

### 优化基准结果

优化配置脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/profile_weld_actual_axis_fast.m
```

优化结果：

```text
weld_actual_axis_replan_fast.mat
algorithm time = 346.143 s
wall time      = 350.343 s
complete       = 1
states         = 910
graph edges    = 435
collision-free edges = 343
max joint jump = 9.270 deg
node collision free = 1
edge collision free = 1
max TCP position error = 0.000047 mm
max TCP orientation error = 0.000005 deg
```

与历史 454.006 秒基线相比约减少 `23.76%`；与同机同次 profiler 基线 672.732 秒相比约减少 `48.55%`。两者都保持 80 点完整 PASS，TCP 误差与碰撞结果一致。

只读对比脚本和报告：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/compare_weld_planner_optimization.m
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_planner_optimization_report.txt
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_baseline_profile.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_fast_profile.mat
```

### 候选种子复用实验结论

曾测试 `reusePoseCandidates=true`，让上一姿态候选直接成为下一姿态 IK 种子。该实验在细化阶段保留了过多状态：

```text
states = 1756
algorithm time = 997.286 s
complete = 0
```

因此它目前关闭，不作为稳定优化配置。原因不是 IK 无解，而是候选状态数量膨胀后分层图无法在硬阈值下连通。后续若要做 continuation，应加入状态上限、分支聚类或按图可达性筛选，而不是直接保留全部上一层状态。

### 复现命令

```bash
matlab -batch "run('/home/ttbstt/jixiebi/UR5/matlab仿真/profile_weld_actual_axis_fast.m');"
matlab -batch "addpath('/home/ttbstt/jixiebi/UR5/matlab仿真'); compare_weld_planner_optimization();"
```

本阶段没有修改 ROS2 控制链路、CoppeliaSim 场景或真机参数；默认规划脚本仍可按原方式运行。

## 105. 冷启动局部 IK continuation 优化（2026-09-23）

目标是把 80 点焊缝规划从稳定优化版 `346.143 s` 继续降低，同时保持 80 个焊点、完整节点/段间碰撞检测和 TCP 约束。新实验不读取已有 `qPath` 或状态图，默认不发布 ROS2、不调用真机。

### 新增文件

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/replan_weld_actual_axis_continuation.m
/home/ttbstt/jixiebi/UR5/matlab仿真/profile_weld_actual_axis_continuation.m
/home/ttbstt/jixiebi/UR5/matlab仿真/compare_weld_planner_continuation.m
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_planner_continuation_report.txt
```

实验算法：

1. Point 1 只用少量冷启动种子生成可行 IK 分支。
2. 后续点使用上一关节解和局部速度预测作为 IK seed。
3. 只在上一姿态附近展开最多 9 个 roll/travel 姿态。
4. J4/J5/J6 只做有限局部扰动。
5. 每层严格 beam 限制，避免候选状态膨胀。
6. 每个接受节点都做完整环境碰撞和自碰撞检查，焊丝碰撞体仍启用。
7. 每条保留边都做密集关节插值碰撞检查。
8. 独立计算相邻点之间 TCP 位置和姿态偏差。
9. 必要时可调用原有 `replan_weld_actual_axis` 作为全新场景 fallback；报告的最快基准关闭 fallback，确保计时公平。

### 冷启动结果

稳定版本对比：

```text
                         stable 346 s   beam=3       beam=2       beam=1
Algorithm time (s)          346.143     58.232       31.825       16.836
Complete path                     1          1           1           1
IK calls                       60936      13116        5988        2424
IK time (s)                       n/a        n/a          n/a       9.546
Max joint jump (deg)          9.270      9.172        9.048       12.580
Max node TCP error (mm)    0.000047   0.000044     0.000048    0.000048
Max segment TCP error (mm) 2.127387   2.127386     2.084652    1.955098
Max segment TCP orient (deg)
                            0.108361   0.102252     0.103100    0.141530
Node collision free               1          1           1           1
Edge collision free               1          1           1           1
```

推荐当前实验结果：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_continuation_beam1.mat
```

该结果 16.836 秒完成 80 点路径，IK 耗时 9.686 秒，2424 次 IK，IK 位姿通过率 `95.75%`，最大关节跳变 `12.580°`，所有节点和 79 条焊缝段边均通过碰撞检测。

注意：节点 TCP 误差只有约 `0.000048 mm`，但相邻关节插值的 TCP 最大位置偏差约 `1.955 mm`、姿态偏差约 `0.142°`。这说明当前路径节点准确，但如果执行器使用关节线性插值，段间 TCP 不会严格沿直线；后续若要提高焊接精度，应增加笛卡尔段间约束或使用笛卡尔轨迹参数化，不能只看节点误差。

### 解析 IK 检查

MATLAB R2025b 的 `analyticalInverseKinematics` 已对当前 `universalUR5` 做实测：

```text
available = 1
IsValidGroupForIK = 0
KinematicGroupType = RRRRRR
```

因此当前加载的 UR5 模型不能直接生成校准一致的闭式 IK。实验继续使用经过验证的数值 `inverseKinematics`；不能为了追求 8 秒而未经验证地替换解析 IK。

### 8 秒目标判断

beam=1 已从 346.143 秒降到 16.836 秒（约 `20.56×` 加速），但尚未达到 8 秒。当前单分支 IK 已占 `9.686 s`，剩余约 7 秒来自完整碰撞检测、图边复核、TCP 段误差检查和 MATLAB 对象开销。继续降低到 8 秒需要针对碰撞对象/插值检查做安全优化，或建立与当前标定、TCP 完全一致的闭式 UR5 IK；不能跳过节点/边碰撞，也不能减少 80 个焊点。

### 复现命令

```bash
matlab -batch "addpath('/home/ttbstt/jixiebi/UR5/matlab仿真'); profile_weld_actual_axis_continuation();"
matlab -batch "addpath('/home/ttbstt/jixiebi/UR5/matlab仿真'); compare_weld_planner_continuation();"

## 106. 自适应段间 TCP 修正与分段计时（2026-09-23）

针对 beam=1 路径节点准确但关节插值段间 TCP 偏差较大的问题，新增：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/replan_weld_actual_axis_adaptive.m
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_planner_adaptive_tcp_report.txt
```

该版本先冷启动执行 beam=1 continuation，再对段间 TCP 位置误差超过 `0.5 mm` 的区段插入笛卡尔中间目标。中间目标采用位置线性插值和姿态四元数插值，每个中间点重新求 IK，并检查关节限位、完整节点碰撞、自碰撞和新增段边碰撞。原始 80 个焊缝节点不会被删除。

### 实测结果

```text
Original weld nodes       : 80
Execution points          : 85
Segments over 0.5 mm      : 2
Adaptive points inserted  : 5
Adaptive IK calls         : 5
Adaptive IK time          : 0.011 s
Base continuation algorithm time : 17.528 s
Base continuation wall time      : 22.164 s
Adaptive phase time              : 2.301 s
Total algorithm time             : 19.829 s
Total wall time                  : 24.465 s
PASS                            : 1
```

段间 TCP 结果：

```text
Initial max segment TCP position error : 1.955098 mm
Final max segment TCP position error   : 0.265895 mm
Final max segment TCP orientation error: 0.038748 deg
Node collision free                   : 1
Edge collision free                   : 1
```

发生修正的区段：

```text
Segment 64: 1.955098 mm -> 插入 3 点
Segment 74: 1.350423 mm -> 插入 2 点
```

最终 0.265895 mm 已低于 0.5 mm 容差；节点 TCP 误差仍约为 `0.000048 mm`。执行轨迹使用 `result.qPathExecution`，规划层的 80 点原始路径仍保存在 `result.qPath`。

结果文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_adaptive.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_adaptive_base.mat
```

当前 `8 s` 目标尚未达到。自适应修正只增加约 `2.301 s`；主要剩余耗时仍是基础 continuation 的数值 IK 和完整碰撞/边验证。不能通过跳过边碰撞或减少焊点达到 8 秒。
```

## 107. Reduced continuation 与自适应 TCP 的公平冷启动结果（2026-09-23）

修正 `replan_weld_actual_axis_adaptive.m` 的参数透传：自适应包装器现在会把 continuation 的采样、首点随机种子、beam 和边检查参数完整传给底层规划器，避免包装器默认值与独立实验配置不一致。

新增独立冷启动剖析脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/profile_weld_actual_axis_continuation_reduced.m
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_continuation_reduced_profile.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_continuation_reduced_profile.txt
```

配置：

```text
beamWidth             = 1
firstPointMaxStates   = 2
maxPoseSeeds          = 3
numRandomSeedsFirstPoint = 3
coarseTravelDeg       = 0
coarseRollDeg         = 0:45:315
enableFallback        = false
```

该测试从当前场景冷启动，不加载旧 `qPath` 或状态图。结果：

```text
Algorithm time        : 15.272679 s
Wall time              : 19.422270 s
IK time                : 8.348373 s
IK calls               : 2192
80-point complete      : 1
Node collision free    : 1
Edge collision free    : 1 (79/79)
Max joint jump         : 12.580372 deg
Max node TCP error     : 0.000048 mm
Max segment TCP error  : 1.955098 mm
Max segment TCP orient : 0.141530 deg
```

同一基础配置叠加自适应 TCP 段间修正后：

```text
Original weld nodes    : 80
Execution points       : 85
Inserted points        : 5
Adaptive IK calls      : 5
Total algorithm time   : 17.787 s
Total wall time        : 22.046 s
Final segment TCP      : 0.265895 mm
Final segment orient   : 0.038748 deg
Node/edge collision    : true / true
PASS                   : true
```

修正区段仍为 segment 64（插入 3 点）和 segment 74（插入 2 点）。`result.qPath` 保持原始 80 个焊点，`result.qPathExecution` 为实际执行用的 85 个路点。

分层耗时显示：首点 32 次 IK、0.402390 s IK；第 2 点 54 次 IK、0.199159 s IK；后续典型层约 27 次 IK。解析 IK 对当前 `universalUR5` 仍不可用（`IsValidGroupForIK=0`），所以不能未经验证地替换数值 IK。

复现：

```bash
matlab -batch "addpath('/home/ttbstt/jixiebi/UR5/matlab仿真'); profile_weld_actual_axis_continuation_reduced();"
matlab -batch "addpath('/home/ttbstt/jixiebi/UR5/matlab仿真'); opts=struct('animate',false,'publish',false,'enableFallback',false,'beamWidth',1,'firstPointMaxStates',2,'maxPoseSeeds',3,'maxPoseNeighbors',9,'numRandomSeedsFirstPoint',3,'coarseTravelDeg',0,'coarseRollDeg',0:45:315,'maxSegmentTcpErrorMm',0.5,'maxIntermediatePoints',4,'outputFile','weld_actual_axis_replan_adaptive_reduced.mat','baseOutputFile','weld_actual_axis_replan_adaptive_reduced_base.mat'); replan_weld_actual_axis_adaptive(opts);"
```

相对于 346.143 s 稳定版本，基础 continuation 约 22.68 倍加速；相对于 16.836 s 的旧 beam=1 配置，本次进一步降至 15.273 s。8 s 目标尚未达到，且不能通过删除焊点、跳过段碰撞或复用旧路径冒充冷启动来达成。

同一配置再次冷启动复测（仅 MATLAB 运行时波动）：

```text
Algorithm time        : 15.496429 s
Wall time              : 19.803686 s
IK time                : 8.512210 s
IK calls               : 2192
PASS                   : 1
```

因此当前应按约 `15.3--15.5 s` 算法时间记录，而不是把单次运行的最后小数位当成固定常数。

## 108. 基于论文的渐进式局部姿态扩展优化（2026-09-23）

参考了 Moscatelli 等关于增量式焊接轨迹生成、Xiao 等关于多 IK 候选与连续性代价图、以及许家伟关于船舶狭小空间保守碰撞模型和分阶段规划的思路，新增了一个不改变工艺硬约束的局部姿态扩展机制。

实现方式：

```text
普通焊点层：只搜索 5 个局部 travel/roll 邻域
当前层无可行状态：仅该层扩展到 9 个邻域
下一层继续沿已通过的状态传播
```

所有 IK、关节限位、焊接侧、焊枪/焊丝/工件、自碰撞和插值边检查保持不变；不读取旧 `qPath`，不使用未经验证的解析 IK，也不删除 80 个焊点。

新增剖析脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/profile_weld_actual_axis_continuation_progressive.m
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_planner_paper_optimization_report.txt
```

5→9 渐进扩展冷启动结果：

```text
Algorithm time        : 9.971550 s
Wall time              : 14.189 s
IK time               : 5.275876 s
IK calls              : 1286
Expanded layers       : 65, 75（第2层包含两个初始分支）
80-point path         : PASS
Node/edge collision   : PASS / PASS (79/79)
Max joint jump        : 12.580 deg
Max node TCP error    : 0.000048 mm
Max segment TCP error : 1.955098 mm
```

叠加原有自适应段间 TCP 修正：

```text
Execution points       : 85
Inserted points        : 5
Corrected segments     : 64（3点）, 74（2点）
Total algorithm time   : 12.340 s
Total wall time        : 16.619 s
Final segment TCP      : 0.265895 mm
Final segment orient   : 0.038748 deg
Node/edge collision    : PASS / PASS
```

3→9 实验也通过，但由于触发更多局部扩展，算法时间为 `10.447 s`，因此不作为基线。当前推荐基线是 5→9 渐进扩展；相对于 346.143 s 稳定版本约提升 34 倍。8 s 目标尚未达到，剩余主要耗时仍是数值 IK 和完整碰撞验证。

另外测试了 Gao 论文中的相邻焊点角度冗余代价：将姿态邻域排序从全局零角参考改为相对上一焊点的 `travel/roll` 变化最小。该版本 1259 次 IK、完整 PASS，但一次冷启动耗时 `11.264 s`，没有超过 5→9 版本，因此只保留为质量优先选项 `poseNeighborhoodCost="previous"`，不替换当前速度基线。

根据 Gao 等人的“角度冗余按需触发”思想，`replan_weld_actual_axis_continuation.m` 新增可选参数：

```text
angularRedundancyFallback = true
angularFallbackRollDeg    = [10 20]
angularFallbackTravelDeg  = [0 5 10]
```

只有某个焊点层没有可行状态时，才围绕当前实际焊丝轴尝试相邻冗余姿态；正常层不会增加 IK。当前场景的 5→9 路径没有触发该回退，独立启用版本仍为 `PASS`，耗时 `9.773 s`、1286 次 IK。该机制用于提高不同工件位姿/狭窄障碍环境下的成功率，不改变当前速度基线。

结果文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_continuation_progressive.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_continuation_progressive_profile.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_continuation_progressive_profile.txt
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_adaptive_progressive.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_adaptive_progressive_base.mat
```

## 109. 实验 C：基于 9.773 s 基线的自适应 TCP 段间修正（2026-09-23）

按 Gao 论文启发的 5→9 渐进 continuation 重新冷启动，并启用角度冗余回退后，对原始 80 点的全部 79 条段边进行 TCP 插值偏差检查。仅对超过配置阈值的段插入 Cartesian 中间目标，再重新 IK、节点碰撞和段边碰撞验证。

```text
Base algorithm time       : 9.823827 s
Base IK calls             : 1286
Original weld nodes       : 80
Execution points          : 85
Inserted points           : 5
Corrected segments        : 64（3点）, 74（2点）
Adaptive IK calls         : 5
Total algorithm time      : 12.013 s
Total wall time           : 16.174 s
Initial max segment TCP   : 1.955098 mm
Final max segment TCP     : 0.265895 mm
Final segment orientation : 0.038748 deg
Node/edge collision       : PASS / PASS
Complete                  : PASS
```

结果文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_angular_adaptive.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan_angular_adaptive_base.mat
```

## 110. Gao-AR-RRT* 同场景对比实验（2026-09-24）

根据 Gao 等（2020）的角度冗余、区域采样、任务空间 steering 和中点/边约束思想，新建了独立的约束焊缝 RRT* 对比实现：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/replan_weld_actual_axis_gao_ar_rrtstar.m
/home/ttbstt/jixiebi/UR5/matlab仿真/profile_weld_actual_axis_gao_ar_rrtstar.m
```

该实现的层定义为 `Point 1..80`，travel/roll 是焊枪角度冗余状态；每个候选状态都复用 `generate_actual_axis_ik_candidates.m`，因此使用与现有算法完全相同的 UR5、GunCollisionHull、wireCollision、4 个 Workpiece collisionBox、焊接侧硬约束、关节限位、节点碰撞、自碰撞和相邻段完整插值碰撞。

实现不加载旧 `qPath`、状态图或 checkpoint，也不修改稳定 continuation、ROS2、CoppeliaSim 或真机。局部姿态采样失败时只对失败层启动有限的角度冗余区域恢复，仍执行完整边检查。

固定随机种子 `20260924` 的冷启动结果（当前场景）为：

```text
Algorithm time        : 21.750 s
Wall time              : 26.210 s
IK calls / pose OK     : 4592 / 3977
Accepted states        : 107
Edge tests / free      : 56 / 55
Complete 80-point path : FAIL
Failure region         : approximately Point 25 onward in this seed/run
```

这不是把失败路径当作 PASS；它说明直接移植 Gao 风格的随机/区域采样，在当前保守 V-HACD 焊枪模型和硬焊接侧约束下，单次随机运行仍存在分支断开。当前稳定 continuation + 条件角度冗余版本仍是可执行基线：约 `9.773 s`，80 点、节点和段边均 PASS。

因此论文对比必须同时报告：首条可行路径时间、成功率、均值/标准差、IK 调用数、边检查数、TCP 段间误差和最大关节跳变；不能直接拿 Gao 论文的 `7.85 s/13.16 s` 与本机单次 MATLAB 时间作结论。当前 Gao 实验结果文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_gao_ar_rrtstar.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_gao_ar_rrtstar_report.txt
```

后续若要形成正式统计，应在相同场景、相同时间预算下用多个固定随机种子重复运行，并增加窄通道/局部障碍场景；在 Gao 版本未完成 80 点之前，不应进行路径质量横向排名。

## 111. 公平多随机种子统计脚本（2026-09-24）

新增：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/benchmark_weld_planners_fair.m
```

默认实验配置为：

```text
Continuation       30 个冷启动随机种子
Gao-AR-RRT*        30 个冷启动随机种子
时间预算          10 / 20 / 30 / 60 s
```

每次运行均重新建立当前 UR5、GunCollisionHull、wireCollision、Workpiece 和 80 点焊缝场景；不会读取旧 `qPath`、状态图或 checkpoint。输出包括成功率、首条可行时间、IK 调用数/成功率、节点/边检查、失败焊点、最大关节跳变、节点和段间 TCP 误差。

运行完整统计：

```matlab
addpath('/home/ttbstt/jixiebi/UR5/matlab仿真');
summary = benchmark_weld_planners_fair();
```

完整批处理可能耗时较长；建议先用小规模冒烟：

```matlab
summary = benchmark_weld_planners_fair(struct( ...
    'runCount',1, ...
    'timeBudgetsSec',10, ...
    'outputDir','fair_benchmark_smoke2', ...
    'summaryFile','fair_benchmark_smoke2.mat', ...
    'csvFile','fair_benchmark_smoke2.csv'));
```

已验证的冒烟结果：

```text
Continuation, 10 s: 1/1 PASS
Gao-AR-RRT*,  10 s: 0/1 PASS
```

冒烟文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_benchmark_smoke2.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_benchmark_smoke2.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_benchmark_smoke2/
```

该结果仅验证批处理流程，不用于宣称算法优劣；正式结论必须基于 30 个随机种子和统一时间预算的成功率曲线。

## 112. 5 种子、单次 60 s 预实验（2026-09-24）

按“每个种子只运行一次、最多 60 s，再由 `firstFeasibleTimeSec` 推导 10/20/30/60 s 成功率”的方式完成了预实验。算法本体未修改。

结果文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_benchmark_pre5.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_benchmark_pre5.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_benchmark_pre5/
```

汇总：

```text
Continuation : 4/5 PASS
Gao-AR-RRT*  : 2/5 PASS
```

由首次可行时间推导的成功率：

```text
                 10 s    20 s    30 s    60 s
Continuation      80%     80%     80%     80%
Gao-AR-RRT*        0%      0%      0%     40%
```

Gao 成功样本首次可行时间约为 `40.14 s` 和 `39.50 s`；失败样本分别保留了 `failurePoint/furthestPoint`、IK 调用数、IK 成功率和边检查数。一个 Continuation 种子在第 1 点即失败，说明该基线也存在初始姿态随机性，不能把 4/5 结果描述为确定性 100% 成功。

本预实验只用于检查统计流程和数据完整性，不作为最终论文结论。正式实验仍应运行 30 个种子，并在报告中区分成功样本首次可行时间与 60 s 截尾失败。

当前 `0.5 mm` 只是算法验收阈值，不是已确认的焊接工艺容差；真实执行前必须由工艺要求替换，并确认控制器实际采用的是关节线性插值还是样条插值。本结果只验证了 MATLAB 中设定的关节线性插值段。

## 113. 30 种子公平冷启动对比完成（2026-09-24）

按预实验确定的公平口径完成了正式批次：同一 UR5、同一 80 点焊缝、同一 GunCollisionHull/wireCollision/Workpiece 模型；每个算法使用 30 个固定种子，每个种子只运行一次、最长 60 s。算法本体、ROS2、CoppeliaSim 和真机均未修改。

结果文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_benchmark_30.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_benchmark_30.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_benchmark_30/
```

分析脚本和产物：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/summarize_fair_weld_benchmark.m
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_benchmark_30/fair_benchmark_30_summary.txt
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_benchmark_30/plots/success_rate_curve.png
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_benchmark_30/plots/first_solution_time_boxplot.png
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_benchmark_30/plots/ik_calls_boxplot.png
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_benchmark_30/plots/failure_point_distribution.png
```

正式批次汇总：

```text
                                10 s    20 s    30 s    60 s
Continuation                    43.3%   43.3%   43.3%   43.3%   (13/30)
Gao-AR-RRT*-inspired baseline     0%      0%      0%    40.0%   (12/30)
```

成功样本首次完整路径时间（只对 PASS 样本统计）：

```text
Continuation       median 8.7049 s, mean 8.5066 s, sample std 0.8327 s, P90 9.3248 s
Gao-AR-RRT*        median 42.9763 s, mean 43.5351 s, sample std 3.8927 s, P90 49.4563 s
```

全部 30 个运行的资源开销：

```text
                         IK calls mean/median       edge tests mean/median
Continuation             799.5 / 966.5              104.1 / 100.5
Gao-AR-RRT*              5752.0 / 7776.0            889.1 / 942.5
```

通过样本质量统计：

```text
                         max jump median/max        segment TCP error median/max
Continuation             12.5804 / 12.5804 deg      1.9551 / 2.2007 mm
Gao-AR-RRT*              8.2314 / 11.2635 deg       1.3177 / 1.9728 mm
```

失败点分布（失败样本保留在 CSV，不丢失）：Continuation 主要在 Point 1 冷启动失败，另有 Point 34、43、47、64 附近断开；Gao-AR-RRT* 失败包含 Point 1，以及 Point 10、19、22、24、57、65、66、68、72、73 等位置。

注意：Gao 版本是“基于 Gao 等（2020）思想的 AR-RRT* 复现”，不是作者官方源码。当前基准说明的是在本项目、当前保守碰撞模型和当前硬约束下的统计差异；不能直接宣称优于论文原算法。CSV 中失败样本的 `firstFeasibleTimeSec` 保持 NaN，实际提前死路退出时间仍保留在 `algorithmTimeSec`，没有把失败时间伪装成成功的 60 s。

## 114. 稀疏焊缝离散实验（2026-09-24）

新增独立实验脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/benchmark_weld_seam_density.m
/home/ttbstt/jixiebi/UR5/matlab仿真/summarize_adaptive_density_benchmark.m
```

实验只改变现有 continuation 的 `options.numPoints`，并将其它参数对齐到公平基线（`beamWidth=1`、progressive pose neighbors、有限初始 seeds）；没有修改算法函数本体。每个点数使用同一组 5 个固定种子进行配对冷启动：

```text
8 / 12 / 16 / 20 / 40 / 80 点
```

结果文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_density_benchmark.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_density_benchmark.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_density_summary.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_density_runs/
```

当前配对实验结果：

```text
点数   PASS率    成功样本规划时间中位数   IK调用中位数   最大段间TCP偏差
  8     0/5        --                    251            --
 12     0/5        --                    338            --
 16     3/5        2.1936 s             326            2.2061 mm
 20     0/5        --                    443            --
 40     3/5        4.2578 s             695            2.2046 mm
 80     4/5        7.5374 s            1325            2.2007 mm
```

分析图：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_density_runs/plots/density_time.png
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_density_runs/plots/density_ik_calls.png
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_density_runs/plots/density_success_rate.png
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_density_runs/plots/density_segment_tcp_error.png
```

解释：减少焊缝点数在成功样本上确实降低了 IK/规划时间，但当前 continuation 对相邻焊点的局部姿态变化和随机初始分支很敏感；8、12、20 点在这 5 个配对种子中全部断开，不能直接选“点数越少越好”。16 点和 40 点各为 60% PASS，80 点为 80% PASS。最大段间 TCP 偏差仍约 2.2 mm，说明稀疏规划点不能替代最终密集笛卡尔验证。

因此下一步应实现“稀疏候选规划 + 超限段自适应插点 + 密集最终验证”，而不是直接把 8/12 点路径送入执行控制器。当前报告仅验证点数、候选分支和连续 IK 之间的关系，不代表已满足实际焊接 TCP 工艺精度。

## 115. ACCP 自适应插点原型（2026-09-24）

利用已有的独立 `replan_weld_actual_axis_adaptive.m` 做了一个 16 点起始的自适应 TCP 修正验证；稳定 80 点 continuation 文件没有修改。

运行配置：

```text
initial points       : 16
seed                 : 20268002
segment TCP threshold: 0.5 mm（仅为算法实验阈值）
max intermediate     : 4 per segment
```

结果：

```text
Initial weld nodes       : 16
Segments over threshold  : 5
Inserted execution points: 15
Execution points         : 31
Initial max TCP segment  : 2.1999 mm
Final max TCP segment    : 0.2801 mm
Final max orientation    : 0.0295 deg
Adaptive IK calls        : 15
Total algorithm time     : 3.344 s
Node collision free      : 1
Edge collision free      : 1
PASS                     : 1
```

结果文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/replan_weld_actual_axis_adaptive.m
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_base_seed20268002.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_seed20268002.mat
```

当前独立脚本已将 TCP 段误差、段关节跳变和段碰撞纳入同一触发器；本次种子实际触发的是 TCP 误差（5 段），没有触发关节跳变或碰撞。它已经证明“16 个规划节点 + 局部插入 15 个执行/验证点”可以把 TCP 段间误差从约 2.20 mm 降到 0.28 mm，同时保持节点和边碰撞 PASS。正式 ACCP 仍需在多个固定种子上统计，并补充 IK 失败时的局部重新求解/细分策略。

## 116. ACCP 可行性+精度双自适应（2026-09-24）

新增独立文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/replan_weld_actual_axis_adaptive_core.m
/home/ttbstt/jixiebi/UR5/matlab仿真/replan_weld_actual_axis_adaptive_feasibility.m
/home/ttbstt/jixiebi/UR5/matlab仿真/benchmark_adaptive_arc_ik.m
/home/ttbstt/jixiebi/UR5/matlab仿真/summarize_adaptive_arc_ik_benchmark.m
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_benchmark_summary.txt
```

与稳定的 `replan_weld_actual_axis_continuation.m` 不同，新的 feasibility wrapper 采用：

```text
16 个初始焊缝点
    ↓
bounded continuation
    ↓
若第 k 层失败：只把第 k-1 到 k 段二分
    ↓
重新冷启动求解
    ↓
起点层失败时仅做一次更强 first-point seed recovery
    ↓
可行链形成后，再对超限 TCP 段插入执行/验证点
```

硬约束没有放宽；每次重试都继续检查 IK、关节限位、UR5/焊枪/焊丝/工件碰撞和插值边碰撞。失败段细分不是简单在失败点后追加点，而是对失败区间做几何中点二分。

单个已知失败种子验证：

```text
seed 20268004
Fixed-16       第 4 层失败
ACCP       二分为 17 个规划点后可行
执行/验证点    34
最终 TCP 段误差 0.4400 mm
节点/边碰撞    PASS/PASS
```

正式配对冷启动实验：30 个相同种子（20269001..20269030），每个种子分别运行 Fixed-80、Fixed-16、ACCP，共 90 次。结果文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_benchmark.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_benchmark.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_benchmark_runs/
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_summary.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_benchmark_plots/
```

汇总：

```text
方法       PASS率       成功样本规划时间中位数   成功样本 IK中位数   最终规划点中位数  最终TCP段误差中位数
Fixed-80   12/30=40.0%  9.3506 s                 1286                80              1.9551 mm
Fixed-16   13/30=43.3%  2.3323 s                 326                 16              2.1999 mm
ACCP   24/30=80.0%  3.1515 s                 364.5               16              0.3316 mm
```

ACCP 成功样本最终点数范围为 16--17 个规划点，执行/验证点中位数为 22 个；成功样本最大段间 TCP 误差最大为 0.4400 mm，成功样本最大执行关节跳变中位数约 8.70°、最大约 8.79°。相对 Fixed-16，ACCP 在 30 个配对种子中额外救活了 11 个样本；ACCP 仍有 6 个失败种子：20269012、20269018、20269019、20269022、20269024、20269027，失败时通常已细分到约 20--21 个点，说明这些样本需要后续有限 beam/角度冗余或更深局部细分，不能宣称已经完全鲁棒。

注意：Fixed-80/Fixed-16 的段间 TCP 误差仍是关节线性插值误差；ACCP 的最终误差包含新增执行/验证点的精度修正，因此比较时应同时报告 PASS 率和最终 TCP 误差，不能只比较规划时间。当前实验没有修改 ROS2、CoppeliaSim、真机、TCP 标定或稳定 continuation 文件。

## 117. ACCP 条件恢复实验（2026-09-24）

在 ACCP 自适应细分仍失败后，新增了一个只对困难样本触发的恢复阶段：

```text
beam = 2 → 3
局部姿态邻域：以上一姿态为中心
travel/roll 邻域：±10°/±20°级别
roll fallback：±10°、±20°
```

重要实现细节：恢复阶段使用 `poseNeighborhoodCost="previous"`，避免扩大姿态邻域后又被零中心排序裁掉。正常成功样本不进入该阶段。

恢复测试文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/benchmark_adaptive_arc_ik_recovery.m
/home/ttbstt/jixiebi/UR5/matlab仿真/summarize_adaptive_arc_ik_recovery.m
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_recovery.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_recovery_combined.csv
```

仅对原 ACCP 的 6 个失败种子进行恢复：

```text
恢复成功：20269012、20269024、20269027
恢复仍失败：20269018、20269019、20269022
```

将恢复成功样本合并回原 30 组后：

```text
方法       PASS率       成功时间中位数   IK中位数   最终TCP段误差中位数
Fixed-80   12/30=40.0%  9.3506 s        1286       1.9551 mm
Fixed-16   13/30=43.3%  2.3323 s        326        2.1999 mm
ACCP 27/30=90.0%  3.8821 s        618        0.3316 mm
```

其中 `ACCP` 表示“正常 ACCP + 失败后条件恢复”，不能把恢复阶段的 20--32 秒困难样本耗时隐藏在正常样本统计中。恢复仍失败的 3 个种子表明，beam 和 ±10°/±20° 局部冗余不是所有断点的充分条件；下一步应诊断这 3 个样本的姿态/碰撞断点，必要时再局部引入 work-angle 或更深的姿态通道搜索，不应继续盲目扩大全局状态库。

## 118. 三个剩余失败种子的局部可行域与连通性诊断（2026-09-24）

按要求没有修改 ACCP 主算法，也没有扩大正常流程。新增独立诊断脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_adaptive_arc_ik_failures.m
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_failure_diagnostics.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_failure_diagnostics.csv
```

诊断了 `20269018`、`20269019`、`20269022` 三个仍失败的恢复种子。每个样本在第一空层前后五层窗口中使用局部 `travel/roll` 偏移网格：

```text
[-30 -25 -20 -15 -10 -5 0 5 10 15 20 25 30] deg
```

每个姿态继续检查 IK 位姿误差、焊接侧、关节限位、UR5/工具/焊丝与工件碰撞、自碰撞；相邻局部层使用当前恢复规则检查姿态步长、最大关节跳变和插值边碰撞。

结果：

```text
seed       fail layer   local window   states/layer       reachable   local result
20269018   15            13:17          7,8,6,7,6          1,1,0,0,0   no local chain
20269019   11             9:13          12,12,8,10,6       3,3,1,1,1   local chain exists
20269022   17            15:19          14,5,3,2,0         2,1,0,0,0   no local chain
```

关键断点：

- `20269018`：`Point 14 -> 15` 的最小候选最大关节变化约 `38.961°`，超过当前 `15°` 阈值；后续层虽有少量候选，但没有前向可达父节点。
- `20269019`：局部扫描找到从左边界到 `Point 13` 的完整无碰撞链，代表链约为 `travel=0°→+5°`、`roll=90°`；关键过渡的最小最大关节变化约 `14.742°`，属于当前搜索遗漏。
- `20269022`：`Point 16 -> 17` 的最小候选最大关节变化约 `24.303°`，超过 `15°` 阈值；`Point 19` 在当前局部网格和硬约束下没有可接受状态，因此没有完整局部链。

三个样本的局部边主要在关节步长检查处被拒绝，没有进入插值碰撞拒绝阶段。详细的每层 IK/姿态/焊接侧/环境/自碰撞计数保存在 MAT 的 `layer` 和 `edgeDiag` 字段中。

当前判断：

```text
20269018  姿态/IK 分支之间发生关节连续性断裂
20269019  明确属于搜索遗漏，可做局部双向桥接
20269022  存在关节连续性断裂且末层无可行状态，暂不能判定为单纯搜索遗漏
```

本诊断仅针对五层局部窗口、当前姿态网格和当前硬阈值，不等于证明全局不可达。下一步若继续开发，应只对 `20269019` 做局部双向桥接验证；对 `20269018/22` 先根据断点数据决定是否局部细化姿态或引入有限 work-angle，不应全局增大 beam。

## 119. 局部双向桥接实验（2026-09-24）

新增独立实验脚本，没有改变 ACCP 默认恢复流程：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/run_adaptive_arc_ik_local_bridge_experiment.m
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_local_bridge_experiment.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_local_bridge_experiment.csv
```

该实验复用局部诊断生成的候选状态，从左边界做前向可达标记，从局部最右非空层做反向可达标记，再搜索 `F_i -> B_(i+1)` 的完整节点/边连接。边仍使用 `15°` 最大关节跳变、姿态步长限制和 3° 插值碰撞检查。

结果：

```text
seed       backward reachable   bridge result
20269018   [2 2 2 3 6]          no complete local bridge
20269019   [4 3 4 5 6]          bridge found: Point 10 -> 11
20269022   [1 1 1 2 0]          no complete local bridge
```

`20269019` 的桥接边为：

```text
travel: +5° -> +5°
roll  : 90° -> 90°
max joint delta: 5.113°
edge collision: PASS
```

因此 `20269019` 确认是搜索遗漏型失败，适合把这个局部双向桥接器作为条件恢复模块；`20269018` 与 `20269022` 虽有部分局部自由边，但没有覆盖完整窗口的双向桥接，不能靠简单增加 beam 解决。

本次实验只证明了局部桥接结构和候选边的可行性；要把 `20269019` 正式计入 30 次成功率，还需要将桥接状态拼回其全局前缀/后续连续求解并进行完整 80 点（或最终执行点）验证，不能直接把局部 PASS 当作全局 PASS。

## 120. 局部桥接全局拼接验证（2026-09-24）

新增全局拼接实验：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/run_adaptive_arc_ik_global_bridge.m
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_global_bridge_seed20269019.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_global_bridge_core_seed20269019.mat
```

实验将 `20269019` 的原始全局前缀、Point 9--13 局部桥接链拼接后，交给未改动的 Continuation 核心继续求解 Point 14--21，并重新验证完整路径。

结果：

```text
prefix -> bridge entry : PASS
bridge internal edges  : PASS
Point 13 -> Point 14   : FAIL
global complete        : FAIL
```

因此不能把成功率更新为 `28/30=93.3%`。局部桥接边本身可行，但它没有找到能继续进入 Point 14 的全局后续构型。

为确认不是桥接终点太早，又对 `20269019` 扩展到 Point 6--16 的局部窗口重新扫描。结果在 Point 14 已有候选状态，但从前向可达链进入 Point 14 的可达状态数仍为 0；扩展局部双向桥接也没有覆盖完整窗口的桥。

最终判断：

```text
20269019：局部 Point 10 -> 11 确实是搜索遗漏，但当前桥接链无法与 Point 14 之后的全局路径拼接。
```

主算法和 90% 统计保持不变，不能把局部 PASS 当作全局 PASS。后续若继续处理，应在同一局部窗口同时生成“桥接终点到后续焊点”的候选链；这属于局部桥接的完整化，不应放宽 `15°` 关节连续性阈值，也不应全局增大 beam。

## 121. 多套 16 点锚点重采样实验（2026-09-24）

为验证失败是否由稀疏规划锚点位置造成，新增独立实验：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/benchmark_adaptive_arc_ik_multigrid.m
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_multigrid_corrected.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_multigrid_corrected.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_multigrid_corrected_runs/
```

实验使用三个剩余失败种子，每个种子测试五套确定性 16 点网格：

```text
grid0_uniform       均匀采样
grid1_shift_plus    中间锚点向后错开 1/4 间距
grid2_shift_minus   中间锚点向前错开 1/4 间距
grid3_alternating   交替错位
grid4_center_dense  中部余弦加密
```

首轮实现曾把自定义网格误传给 wrapper，导致五套网格实际使用同一默认焊缝；该轮结果不作结论。随后已增加 `initialSeamPositionsWP` 接口并使用独立 `corrected` 输出目录重新完成 15 次冷启动。

校正后的结果为：

```text
seed       grid PASS 数量   失败层范围       算法时间范围
20269018   0/5              14--15          22.58--26.55 s
20269019   0/5              11--16          19.46--26.04 s
20269022   0/5              16--17          16.23--19.13 s
```

所有 15 次均在自适应细分后达到 21 个点左右并进入 bounded recovery，但没有形成完整可行路径；因此多套 16 点锚点重采样在当前姿态网格、15°关节连续性和完整碰撞硬约束下没有救活三个失败种子。它改变了失败层位置和 IK 调用次数，但没有改变总体 `0/15 PASS` 结论。

本实验没有修改稳定主算法，也没有加入 Global Layered Graph。当前应保留的判断是：单纯改变稀疏锚点布局不足以解决这三个失败样本，下一步若继续处理，应回到失败窗口的局部姿态/分支连通性，而不是继续增加全局网格数量。

## 122. 局部密集连通性诊断（2026-09-24）

新增独立诊断脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/analyze_local_dense_connectivity.m
/home/ttbstt/jixiebi/UR5/matlab仿真/local_dense_connectivity.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/local_dense_connectivity.csv
/home/ttbstt/jixiebi/local_dense_connectivity.log
```

诊断只使用三个失败样本的局部窗口，窗口前后各 2 个原始点；每个原始区间细分为 4 段，局部 `travel/roll` 使用 `-30:5:30` 度网格，保留所有通过 IK、焊接侧、关节限位、焊枪/焊丝/机器人碰撞和自碰撞检查的状态。相邻密集层仍使用原来的姿态步长和 `15°` 最大关节变化硬约束，并做插值碰撞检查。

结果：

```text
seed       dense layers   states   forward final   backward first   local chain   diagnosis
20269018   17             116      1               5                 YES          DENSE_LOCAL_CHAIN_EXISTS
20269019   17             400      14              19                YES          DENSE_LOCAL_CHAIN_EXISTS
20269022   17              66      0               0                 NO           NODES_EXIST_BUT_INTERPOLATED_COLLISION_BLOCKS_EDGES
```

更具体地说：

- `20269018`：原来约 `38.96°` 的断裂在局部细分后出现完整无碰撞链，说明该失败主要是离散跨度/分支搜索问题，不是局部几何必然不可行。
- `20269019`：局部密集图也存在完整链，进一步确认存在可行的局部姿态与 IK 通道；但这仍不是全局 80 点成功证明，必须后续拼接前缀和后缀再验证。
- `20269022`：每层仍有节点，最小姿态兼容关节变化保持在 `15°` 内，但前向可达性在约 `base index 16.75` 处归零，局部图存在插值碰撞拒绝边；当前结论是局部连通性被碰撞/分支组合阻断，而不是某一层完全没有 IK 节点。

局部转移统计中，`20269018` 和 `20269019` 均存在大量 collision-free edges；`20269022` 最后阶段虽有若干关节兼容边，但无法从前向可达状态传播到末层，不能把它判定为简单 beam 不足。

本诊断没有修改 ACCP 主体、没有放宽 `15°` 阈值、没有运行 Global Layered Graph，也没有修改 ROS2/CoppeliaSim。下一步最有价值的是：针对 `20269018/19` 提取密集链并做全局前缀/后缀拼接；针对 `20269022` 在 `16.5→16.75` 附近进一步分类可达源边的碰撞对象，而不是全局扩大搜索。

从局部前向父指针回溯得到的代表链最大关节跳变为：

```text
20269018 : 2.569°
20269019 : 14.898°
20269022 : 无完整链
```

因此 `20269018/19` 的局部密集链都满足当前 `15°` 阈值；其中 `20269019` 已非常接近阈值，后续全局拼接时仍需重新检查边和碰撞，不能只依据局部图计入总成功率。

## 123. 局部密集图全局拼接验证（2026-09-24）

新增独立拼接脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/run_local_dense_graph_recovery.m
/home/ttbstt/jixiebi/UR5/matlab仿真/local_dense_graph_recovery.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/local_dense_graph_recovery.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/local_dense_recovery_runs/
```

该实验把局部密集图中所有“从全局前缀可达”的末端状态一起交给原 Continuation 核心，而不是只选一条代表链；局部内部和全局后缀仍重新执行节点/边约束。

结果：

```text
20269018 : 局部链接入后缀失败，combined 33 points，首次失败约第 30 层
20269019 : 保留全部局部末端分支后仍接入后缀失败，首次失败约第 29 层
```

因此目前还不能把任一 seed 计入 30 次成功率。局部图本身存在完整链，但它们尚未找到能继续通过整个焊缝后缀的全局构型；这说明下一步若继续，必须在局部窗口同时生成“通向后缀”的反向可达状态，而不是只从局部末端单向 continuation。

对 `20269022` 的首个前向断点（dense layer 7→8，对应 base index `16.50→16.75`）做了可达源边分类：

```text
pose-rejected edges   : 1
joint-rejected edges  : 2
pose/joint-compatible : 0
collision-rejected    : 0
```

所以更精确的结论是：虽然整个局部图中存在碰撞拒绝边，但从当前前向可达状态出发，第一断点首先是姿态/关节连续性组合阻断，并非已确认的某个具体工件碰撞对象。当前没有可靠的碰撞对象标签，不能把 `20269022` 简化写成“纯碰撞阻断”。

## 124. 双向局部密集恢复（BLDR）首次实现（2026-09-24）

新增独立实验脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/run_bidirectional_local_dense_recovery.m
/home/ttbstt/jixiebi/UR5/matlab仿真/bidirectional_local_dense_recovery.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/bidirectional_local_dense_recovery.csv
/home/ttbstt/jixiebi/bidirectional_local_dense_recovery.log
/home/ttbstt/jixiebi/bidirectional_local_dense_recovery_strict2.log
```

该脚本只在三个剩余失败 seed 的局部窗口运行，不修改 ACCP 主流程。它执行：

```text
全局前缀严格前向可达集
        +
局部 dense layers
        +
从焊缝后缀终点反向生成的可达集
        ↓
Forward ∩ Backward 会合
        ↓
完整路径节点/插值边/TCP 段间误差复核
```

重要修正：初版诊断曾复用局部报告中未包含边界 `15°` 约束的 `forwardMask`，并且验证只检查了碰撞。当前脚本已重新计算严格 forward reachability，并在 `localEdgeFree` 中统一强制：

```text
|Δtravel| ≤ 5°
|Δroll|   ≤ 15°（圆周角距离）
max|Δq|   ≤ 15°
插值碰撞检查通过
```

同时以 `0.5 mm` 作为本实验的 TCP 段间误差验收阈值；该阈值只用于本次诊断，不修改主算法的既有参数。

严格结果：

```text
seed       meet   complete   forwardFinal   backwardFirst   maxJump       maxTCPseg       diagnosis
20269018   no     no         0              5                --            --              NO_FORWARD_BACKWARD_MEET
20269019   yes    no         1              19               14.304°       1.5310 mm       TCP_SEGMENT_ERROR
20269022   no     no         0              0                --            --              NO_FORWARD_BACKWARD_MEET
```

解释：

- `20269018`：局部 dense 图虽然存在链，但在把真实全局前缀纳入严格 `15°` 边界约束后没有找到前后会合，不能计入成功率。
- `20269019`：找到严格双向会合，最大关节跳变 `14.304°`，节点和插值碰撞可通过；但完整拼接路径的最大 TCP 段间偏差 `1.5310 mm`，超过本实验 `0.5 mm` 阈值，因此仍不能计入 PASS。它是“存在会合但需要 TCP 局部精度修正”的候选，而不是完整成功。
- `20269022`：严格 forward 集合无法与后向集合会合；当前仍不能判断为全局不可达，只能说在当前局部姿态网格、15°连续性和完整碰撞约束下未找到会合。

因此本轮没有把成功率从 `27/30=90%` 更新到更高值，也没有修改原 benchmark 结果。BLDR 已证明“后向集合参与分支选择”可以找到 `20269019` 的候选会合，但要形成真正 PASS，还需要在会合路径上做局部 TCP 自适应插点/重新 IK，并再次执行整条焊缝验证。当前没有运行 Global Layered Graph、没有发送 ROS2、没有修改 CoppeliaSim 或真机。

## 125. 全新 100 seed 冷启动验证（2026-09-26）

为避免继续针对历史失败样本过拟合，冻结 ACCP 的现有实现和参数，使用与开发集 `20269001:20269030` 完全不重叠的新 seed：

```text
20270001:20270100
```

新增独立 benchmark wrapper：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/benchmark_adaptive_arc_ik_new100.m
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_new100.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_new100.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/adaptive_arc_ik_new100_runs/
/home/ttbstt/jixiebi/adaptive_arc_ik_new100.log
```

每个 seed 都重新调用 `replan_weld_actual_axis_adaptive_feasibility`，重新加载当前 MATLAB 场景，使用 16 点稀疏起始焊缝、最多 4 轮局部二分、最多 40 个规划点，并保留原有 beam/angular post-recovery 参数。没有加载历史 `qPath`、状态图或上一 seed 的 MAT 结果。失败样本的 round/recovery 文件也全部保留。

本轮统计：

```text
样本数                         100
PASS                           89 / 100 = 89.0%
FAIL                           11 / 100
成功率 Wilson 95% 区间         81.4% ~ 93.8%
首次完整解算法时间（成功样本）   median 2.820 s, P90 6.826 s
总算法时间（成功样本）           median 3.867 s, P90 7.793 s
墙钟时间（成功样本）             median 5.578 s, P90 10.446 s
IK 调用（成功样本）              median 618, P90 1551
最终规划点（成功样本）            median 16, P90 17
执行点（成功样本）               median 22, P90 34
最大关节跳变（成功样本）          median 8.703°
最大 TCP 段间误差（成功样本）     median 0.332 mm, P90 0.440 mm
```

快速通道与恢复通道：

```text
未触发 post-recovery       84 次，其中 84 PASS
触发 post-recovery         16 次，其中 5 PASS、11 FAIL
```

失败 seed 为：

```text
20270003, 20270004, 20270007, 20270054, 20270057,
20270062, 20270071, 20270078, 20270085, 20270092, 20270095
```

11 个失败均是在有限 `beam=2→3 + angular recovery` 用尽后结束，没有运行时异常；10 个进入 beam 3，1 个在 beam 2 阶段结束。失败样本墙钟时间约 `14.8~39.3 s`，说明尾部困难样本会显著拉高 P90，但没有改变快速样本的几秒级性能。

本轮结论：新 100 seed 的 `89%` 与历史开发集 `27/30=90%` 一致，暂不能声称成功率已达到 95% 或 100%。结果支持“多数任务走快速 ACCP 通道、少数任务进入昂贵恢复并仍可能失败”的分层性能描述。BLDR 仍保持为独立诊断模块，未强行并入本轮 benchmark，因为此前尚未产生完整 PASS 的全局拼接路径；因此本轮结果不会把 BLDR 的候选会合误计入成功率。

## 126. 新 100 seed 的 11 个失败样本：局部密集连通性与双向会合诊断（2026-09-26）

针对新 100 seed 中的 11 个失败样本，冻结主算法和全部参数，仅运行独立诊断：

```text
20270003, 20270004, 20270007, 20270054, 20270057,
20270062, 20270071, 20270078, 20270085, 20270092, 20270095
```

新增批量诊断入口及结果：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/benchmark_local_dense_diagnostics_new11.m
/home/ttbstt/jixiebi/UR5/matlab仿真/local_dense_connectivity_new11.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/local_dense_connectivity_new11.mat
/home/ttbstt/jixiebi/local_dense_diagnostics_new11.log
```

局部密集图分类：

```text
局部存在完整 dense chain（4 个）:
20270054, 20270062, 20270078, 20270095

存在节点，但插值边被碰撞阻断（5 个）:
20270004, 20270007, 20270057, 20270085, 20270092

至少一层没有可行节点（2 个）:
20270003, 20270071
```

说明：`DENSE_LOCAL_CHAIN_EXISTS` 只表示局部密集状态图中存在候选链，不代表它已经与全局前缀、后缀完整拼接，也不代表最终焊缝路径 PASS。

随后运行严格的 Bidirectional Local Dense Recovery（BLDR）诊断，保持：

```text
|Δtravel| ≤ 5°
|Δroll|   ≤ 15°（圆周角距离）
max|Δq|   ≤ 15°
节点/插值边碰撞检查
TCP 段间误差阈值 0.5 mm
```

新增入口、结果和日志：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/run_bidirectional_local_dense_recovery_new11.m
/home/ttbstt/jixiebi/UR5/matlab仿真/bidirectional_local_dense_recovery_new11.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/bidirectional_local_dense_recovery_new11.mat
/home/ttbstt/jixiebi/bidirectional_recovery_new11.log
```

严格 BLDR 结果：

```text
seed       meet  complete  结论
20270003   no    no        NO_FORWARD_BACKWARD_MEET
20270004   no    no        NO_FORWARD_BACKWARD_MEET
20270007   no    no        NO_FORWARD_BACKWARD_MEET
20270054   no    no        NO_FORWARD_BACKWARD_MEET
20270057   no    no        NO_FORWARD_BACKWARD_MEET
20270062   yes   no        TCP_SEGMENT_ERROR（max TCP segment = 2.2111 mm）
20270071   no    no        NO_FORWARD_BACKWARD_MEET
20270078   no    no        NO_FORWARD_BACKWARD_MEET
20270085   no    no        NO_FORWARD_BACKWARD_MEET
20270092   no    no        NO_FORWARD_BACKWARD_MEET
20270095   no    no        NO_FORWARD_BACKWARD_MEET
```

只有 `20270062` 找到严格前后向会合；其最大关节跳变约 `9.947°`，但完整拼接候选的 TCP 段间误差为 `2.2111 mm`，所以没有形成完整 PASS。其余 10 个样本在当前局部窗口、姿态网格、15°关节连续性和碰撞规则下没有找到前后向会合。

本轮没有修改 ACCP、没有把 BLDR 并入主恢复流程、没有更新 89/100 成功率，也没有运行 Global Layered Graph、ROS2 或真机。当前结论是：局部密集图可作为失败原因诊断工具，但现阶段还不能直接作为可靠 fallback。下一步若继续研究，应优先记录阻断边的具体碰撞对象，并分别判断是姿态/IK 分支问题、插值碰撞问题，还是当前工艺姿态和几何约束下的局部不可行；不要继续针对这 11 个 seed 盲目扩大全局搜索。

## 127. 对称焊缝端部余量独立实验（2026-09-27）

本轮冻结 ACCP 主算法，仅改变有效焊缝参数区间：

```text
s ∈ [m, L-m]
```

其中 `m=endMarginMm`，完整机器人、焊枪、焊丝、工件和自碰撞检查始终保留；没有忽略任何端部碰撞。新增实验入口：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/benchmark_end_margin.m
/home/ttbstt/jixiebi/UR5/matlab仿真/summarize_end_margin_benchmark.m
/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_end_margin_collisions.m
```

汇总结果：

```text
端部余量       PASS/100       成功率
0 mm           0/100          0%
5 mm           0/100          0%
10 mm          0/100          0%
15 mm          0/100          0%
20 mm          0/100          0%
21 mm          0/100          0%
22 mm          59/100        59%
23 mm          72/100        72%
24 mm          69/100        69%
25 mm          89/100        89%
30 mm          69/100        69%
```

因此在当前 100 个 benchmark seed 中：

- 余量低于 `22 mm` 时，首层/首个有效焊接姿态整体不可行，全部失败；
- `22–24 mm` 已出现部分可行分支，但成功率低于当前 `25 mm` 基线；
- `25 mm` 达到本实验最高成功率 `89/100`；
- 增大到 `30 mm` 后成功率反而降至 `69/100`，说明端部余量不是单调越大越好，而会改变首个 TCP 姿态和 IK 分支结构；
- 本组测试没有任何余量达到 `100/100`，因此不能定义“100% benchmark 最小余量”。

对应文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/end_margin_benchmark.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/end_margin_benchmark.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/end_margin_summary.txt
/home/ttbstt/jixiebi/UR5/matlab仿真/end_margin_plots/end_margin_success_rate.png
/home/ttbstt/jixiebi/UR5/matlab仿真/end_margin_plots/end_margin_time.png
/home/ttbstt/jixiebi/UR5/matlab仿真/end_margin_plots/end_margin_failure_type.png
```

端部碰撞位置诊断：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/end_collision_diagnostics.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/end_collision_diagnostics.mat
```

物理焊缝长度为 `225 mm`。局部密集图中碰撞拒绝边的最近物理端部距离范围为 `26.458–105.937 mm`；按 seed 汇总，很多碰撞发生在距物理端部 `50–100 mm` 的内部区域，不能简单归因于端部。因此端部余量确实决定了首层可行性，但不能解释全部后续边失败。

当前 MATLAB `rigidBodyTree/checkCollision` 聚合接口没有返回具体碰撞 body 对，因此诊断表中的 `collisionPair` 明确标记为 `UNAVAILABLE_AGGREGATE_CHECKCOLLISION`，没有伪造 GunHull、焊丝或 link 名称。

本轮没有修改稳定 ACCP、ROS2、CoppeliaSim 或真机。当前建议冻结 `25 mm` 作为本场景的实验基线，而不要把它表述为理论最小值或所有工业场景通用值。若要继续优化，应先解决内部碰撞/IK 连通性，并单独增加可返回碰撞对象的几何级诊断。

## 128. 正向/反向焊缝遍历方向敏感性实验（2026-09-27）

本轮没有修改 ACCP 主算法，也没有重新运行正向基线。固定当前 `25 mm` 端部余量，正向结果直接读取已有的 100 个冷启动结果；反向实验将同一组 16 个初始焊缝锚点按 `flipud` 反转，并使用相同的 ACCP 参数、随机 seed、完整焊枪/焊丝/工件/自碰撞约束。自定义 seam builder 根据反转后的首尾点重新计算焊缝切向，因此 travel 方向随焊接遍历方向改变；不是任意打乱点序。

入口与结果文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/benchmark_direction_sensitivity.m
/home/ttbstt/jixiebi/UR5/matlab仿真/direction_sensitivity.log
/home/ttbstt/jixiebi/UR5/matlab仿真/direction_sensitivity.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/direction_sensitivity.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/direction_reverse_runs/reverse_seed*.mat
```

100 个新 seed（`20270001:20270100`）的结果：

```text
Forward                 89/100 = 89%
Reverse                 74/100 = 74%
Forward OR Reverse      97/100 = 97%
```

反向成功但正向失败、因此被方向切换救回的 8 个 seed：

```text
20270003, 20270004, 20270054, 20270062,
20270078, 20270085, 20270092, 20270095
```

两个方向都失败的 3 个 seed：

```text
20270007, 20270057, 20270071
```

成功样本的算法时间（不含场景加载）统计：

```text
Forward 成功样本中位数：5.803 s
Reverse 成功样本中位数：8.595 s
```

因此方向反转确实改变了 IK continuation 分支，并将成功率从单向 `89%` 提升到方向并集 `97%`。但 `97/100` 是“两个独立方向均允许尝试”的实验并集，不等于已经实现的在线级联时间；若以后采用 `Forward -> FAIL -> Reverse`，应另测级联墙钟时间，并保持正向 PASS 样本不再执行反向规划。

当前结论：正/反向双启动是比整条 Global Layered Graph 更便宜、且有效的一级恢复机制。下一步可以在不改变主算法的前提下，把它作为 `ACCP` 失败后的方向级 fallback；但对三个双向失败 seed 不能宣称全局不可达，仍需后续局部姿态/碰撞诊断。ROS2、CoppeliaSim、真机和 TCP 标定均未修改。

## 129. 冻结 Forward→Reverse 级联的独立 held-out 验证（2026-09-27）

为避免用发现正/反向策略的 `20270001:20270100` 数据自证，本轮使用完全未参与开发的新 seed：

```text
20270101:20270200
```

级联逻辑严格固定为：

```text
Forward ACCP
    ├─ PASS：直接输出，不运行 Reverse
    └─ FAIL：反转同一组 16 个焊缝锚点，重算 seam tangent/travel，运行 Reverse ACCP
```

没有修改 Fast Path、beam、姿态范围、碰撞规则、TCP 标定、ROS2、CoppeliaSim 或真机。

新增入口与结果：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/benchmark_direction_cascade.m
/home/ttbstt/jixiebi/UR5/matlab仿真/direction_cascade_heldout.log
/home/ttbstt/jixiebi/UR5/matlab仿真/direction_cascade_heldout.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/direction_cascade_heldout.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/direction_cascade_forward_runs/
/home/ttbstt/jixiebi/UR5/matlab仿真/direction_cascade_reverse_runs/
```

最终结果：

```text
Forward 直接成功          87/100 = 87%
Reverse 触发              13/100 = 13%
Reverse 成功               9/13  = 69.2%（仅在 Forward 失败样本中）
Forward→Reverse 级联成功  96/100 = 96%
```

反向成功并救回的 9 个 seed：

```text
20270108, 20270117, 20270139, 20270146, 20270166,
20270169, 20270178, 20270179, 20270185
```

级联仍失败的 4 个 seed：

```text
20270113, 20270125, 20270159, 20270187
```

本批次成功样本的时间统计（算法计时不含场景加载）：

```text
Forward 直接成功样本中位数：3.110 s
完整级联成功样本中位数：    3.148 s
完整级联成功样本 P90：       25.061 s
实际级联墙钟时间中位数：     4.031 s
```

其中 P90 明显高于中位数，说明少数需要 Reverse 的困难样本会支付额外恢复代价；这正是“快速路径 + 按需恢复”的预期特征。该批次的 `96/100` 与发现集方向并集 `97/100` 接近，支持方向级联具有一定泛化性，但还不能声称工业级 100% 可靠。

### 三个历史双向失败样本的局部诊断

复用已有的局部密集连接报告，而没有修改主算法：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/local_dense_connectivity_new11.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/bidirectional_local_dense_recovery_new11.csv
```

```text
Seed       诊断结论                                      关键统计
20270007   有节点，但插值边被碰撞阻断                    3 条碰撞拒绝边，105 条自由边
20270057   有节点，但插值边被碰撞阻断                    20 条碰撞拒绝边，59 条自由边
20270071   局部某一层没有可行节点                        diagnosis=NO_NODE_AT_ONE_OR_MORE_DENSE_LAYERS
```

三个样本的局部双向图均未找到完整会合，因此当前不能把它们称为全局不可达。前两个应继续做具体碰撞对象级诊断；`20270071` 应先区分 IK 无解、姿态约束过滤、节点碰撞或 welding-side 过滤，再决定是否调整工艺姿态窗口。

当前推荐的工业级规划顺序冻结为：

```text
Forward ACCP
    ↓ fail
Reverse ACCP
    ↓ fail
Local Dense / Deterministic Recovery
    ↓ fail
报告局部不可行原因或进入更重的全局方法
```

## 130. 同一 U 形工件的 Seam B/C 泛化 smoke test（2026-09-27）

在不修改 Seam A、稳定 ACCP、ROS2、CoppeliaSim、真机和 TCP 标定的前提下，新增了 seam-specific 实验副本，使焊接侧和实际焊枪轴可以随交线改变。原稳定文件保持不变。

新增实验文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/replan_weld_seam_generalization_core.m
/home/ttbstt/jixiebi/UR5/matlab仿真/replan_weld_seam_generalization_feasibility.m
/home/ttbstt/jixiebi/UR5/matlab仿真/generate_seam_axis_ik_candidates.m
/home/ttbstt/jixiebi/UR5/matlab仿真/benchmark_weld_seam_generalization_smoke.m
/home/ttbstt/jixiebi/UR5/matlab仿真/seam_generalization_smoke.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/seam_generalization_smoke.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/seam_generalization_smoke.log
```

实验使用新 seed：`20270301:20270305`，每条 seam 5 个，固定：

```text
端部余量：25 mm
初始规划点：16
Forward → 失败才 Reverse
最大关节步长：15 deg
完整机器人/焊枪/焊丝/工件/自碰撞检查
```

### 几何定义

实际 Workpiece proxy 名称已检查为：

```text
WorkpieceCollision_bottom
WorkpieceCollision_wall_x0
WorkpieceCollision_wall_x1
WorkpieceCollision_wall_y1
```

Seam B：`bottom × wall_x0`

```text
有效端点 WP：约 [0.005, 0.025, 0.005] 到 [0.005, 0.205, 0.005] m
有效长度：180 mm
tangent：+Y
实际焊枪轴：[-0.7071, 0, -0.7071]
焊接侧：x > wall_x0 内侧，z > bottom 顶面
```

Seam C：`bottom × wall_x1`

```text
有效端点 WP：约 [0.230, 0.025, 0.005] 到 [0.230, 0.205, 0.005] m
有效长度：180 mm
tangent：+Y
实际焊枪轴：[+0.7071, 0, -0.7071]
焊接侧：x < wall_x1 内侧，z > bottom 顶面
```

### Smoke 结果

```text
Seam B：Forward 5/5，Forward→Reverse 5/5
Seam C：Forward 4/5，Forward→Reverse 4/5
```

Seam C 的失败 seed：

```text
20270301：Forward FAIL，Reverse FAIL
```

Seam B 五个样本都通过完整节点和边验证；Seam C 的四个 PASS 样本也通过完整验证。当前 smoke 结果不应外推为正式成功率，尤其 Seam C 仍有一个双向失败样本；按计划暂不启动 Seam B/C 的 100-seed 长实验。

当前下一步应先对 Seam C 的 `20270301` 做局部失败分类，确认是首层姿态、IK 分支、焊接侧过滤还是插值碰撞，再决定是否进入 100-seed 泛化实验。不能因为 Seam B 的 5/5 就跳过该诊断。

## 131. 多启动 First-Feasible 泛化实验已启动（2026-09-27）

根据最新实验目标，停止针对单个历史失败 seed 的定向调试，改用工业式“首条可行轨迹”评价。新增脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/benchmark_multistart_first_feasible.m
```

固定设置：

```text
Seam A/B/C：各 100 个全新 task seed（20270601:20270700；与 smoke/开发 seed 分离）
端部余量：25 mm
初始规划点：16
每个 trial：最多 3 次独立初始化
每次初始化：Forward ACCP，失败后才 Reverse ACCP
总时间预算：30 s / trial
严格保留 TCP、15° 关节连续性、机器人/焊枪/焊丝/工件/自碰撞和边插值检查
```

输出文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/multistart_first_feasible_fresh.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/multistart_first_feasible_fresh_attempts.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/multistart_first_feasible_fresh.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/multistart_first_feasible_fresh.log
/home/ttbstt/jixiebi/UR5/matlab仿真/multistart_first_feasible_runs_fresh/
```

脚本将“完整路径但超过 30 s”记录为 `PASS-LATE`，不计入预算内 first-feasible PASS；所有尝试仍保留在 attempt CSV 中。正式结果待运行完成后补充，不能用 smoke test 或历史 seed 代替。

## 132. 多启动 First-Feasible 正式结果（2026-09-27）

正式实验已完成。样本为与开发集和 smoke test 分离的全新 seed：`20270601:20270700`；每条 Seam 100 个 trial，共 300 个 trial。每个 trial 的时间预算为 30 s，规划器配置为最多 3 次初始化，但在本次严格 30 s 预算下实际每个 trial 都在第一次 Forward 或其 Reverse 结束后达到预算，因此 `attemptsUsed` 全部为 1；这不是脚本漏记，而是单次恢复规划已消耗预算。实际尝试数为 300 个 Forward + 25 个 Reverse。

结果文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/multistart_first_feasible_fresh.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/multistart_first_feasible_fresh_attempts.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/multistart_first_feasible_fresh.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/multistart_first_feasible_fresh.log
/home/ttbstt/jixiebi/UR5/matlab仿真/multistart_first_feasible_runs_fresh/
```

### 30 s 预算内首条可行轨迹

```text
Seam A bottom_x_wall_y1 : 86/100 = 86%
Seam B bottom_x_wall_x0 : 100/100 = 100%
Seam C bottom_x_wall_x1 : 69/100 = 69%
总体                      : 255/300 = 85%
```

预算内成功样本的首次可行时间：

```text
                 median     P90       P95       max
Seam A           3.874 s    7.527 s  11.786 s  29.781 s
Seam B           3.752 s    3.867 s   3.885 s   3.921 s
Seam C          21.396 s   29.450 s  29.764 s  29.888 s
总体             3.830 s   22.329 s  22.614 s  29.888 s
```

方向贡献（预算内成功）：

```text
Seam A：Forward 84，Reverse 2
Seam B：Forward 100，Reverse 0
Seam C：Forward 66，Reverse 3
```

额外完成但超过 30 s 的 `PASS-LATE`：Seam A 5 个、Seam B 0 个、Seam C 27 个，共 32 个。若忽略 30 s 截止时间，完整路径实际完成数为 `287/300`，但该数不能作为预算内工业成功率。

预算内成功轨迹的几何/连续性上界：

```text
Seam A：最大段间 TCP 误差 0.440 mm，最大关节跳变 8.786 deg
Seam B：最大段间 TCP 误差 0.491 mm，最大关节跳变 13.048 deg
Seam C：最大段间 TCP 误差 0.481 mm，最大关节跳变 13.831 deg
```

所有计入 PASS 的结果都经过完整节点和插值边碰撞检查，焊丝碰撞模型启用。该实验说明：Seam B 最稳定；Seam C 的主要问题是恢复代价高，导致大量 `PASS-LATE` 或预算内失败；当前 30 s 预算下多启动尚未真正进入第二次初始化，后续若研究“多启动本身”的收益，应另设更宽时间预算或单次尝试上限，不得把本次 30 s 结果误称为 3 次重启实验。

## 133. 任务级 First-Feasible 评价实验（2026-09-27，进行中）

根据任务级成功定义，新增包装脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/run_task_level_multistart_benchmark.m
```

它不改变 ACCP 或 Forward/Reverse 级联，只将 seed 视为初始化，并对每条 Seam 任务执行：

```text
最多 5 次初始化
总任务预算 60 s
任意一次完整节点/边/TCP/碰撞 PASS 即停止
```

正式使用全新 task seed `20270801:20270810`，每条 Seam 10 个任务，共 30 个任务。输出：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/task_level_multistart.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/task_level_multistart_attempts.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/task_level_multistart.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/task_level_multistart.log
```

结果已完成。该实验的 PASS 含义是任务级首条可行轨迹，不是单个随机 seed 的成功率。

### 任务级结果（60 s 总预算）

```text
Seam A：10/10 任务找到完整 PASS，median 6.284 s，P90 11.603 s
Seam B：10/10 任务找到完整 PASS，median 3.766 s，P90 3.840 s
Seam C：10/10 任务找到完整 PASS，median 22.191 s，P90 30.177 s
总体：30/30 任务 PASS
```

首次可行轨迹的方向贡献：

```text
Seam A：Forward 9，Reverse 1
Seam B：Forward 10，Reverse 0
Seam C：Forward 8，Reverse 2
```

本批 30 个任务中，所有任务都在第一次初始化内部的 `Forward → Reverse` 级联中完成，因此 `attemptsUsed=1`；实际执行 30 个 Forward 和 3 个 Reverse，没有触发第二个 random initialization。这个结果证明当前三条焊缝在任务级上均可行，并给出了 60 s 预算下的首条可行时间；若要测量第二/第三次 restart 的边际收益，需要专门构造或抽取首轮 F/R 都失败的任务样本，不能从本批 30/30 反推出 restart 收益。

## 134. ACCP 与 Gao-AR-RRT* 任务级公平对比基线（2026-09-27）

为进入论文实验阶段，新增独立对比脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/benchmark_task_level_arc_vs_gao.m
```

该脚本冻结现有 ACCP，不读取历史 `qPath`、checkpoint 或状态图；Gao 侧使用独立的 `replan_weld_actual_axis_gao_ar_rrtstar.m`。两者均以同一 MATLAB 场景、GunCollisionHull、wireCollision、4 个 Workpiece collision proxy、同一 TCP/焊接侧/关节连续性/节点与插值边碰撞约束为输入。

当前对 Gao 基线仅补齐了实验所需的 `seamSpec`：

```text
Seam A = bottom × wall_y1，gun axis [0, 1, -1]/sqrt(2)
Seam B = bottom × wall_x0，gun axis [-1, 0, -1]/sqrt(2)
Seam C = bottom × wall_x1，gun axis [ 1, 0, -1]/sqrt(2)
```

Gao 候选状态改用 seam-aware 的 `generate_seam_axis_ik_candidates`，因此 B/C 使用各自墙面内侧约束；这不改变 ACCP 稳定代码。

评价口径固定为：每个任务只运行一次最大预算，得到首条完整可行轨迹的 wall-clock TFF；`PASS@10/20/30/60 s` 由 TFF 阈值派生，不将 Gao 后续优化时间计入首条可行解时间。正式默认设置为 30 个全新 task seed（每条 Seam 30 个），最大预算 60 s：

```matlab
addpath('/home/ttbstt/jixiebi/UR5/matlab仿真');
benchmark_task_level_arc_vs_gao();
```

输出：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/task_level_arc_vs_gao.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/task_level_arc_vs_gao.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/task_level_arc_vs_gao_runs/
```

已完成单 seed、20 s 的接口 smoke（seed `20270901`），结果不是正式统计：

```text
             ACCP                         Gao-AR-RRT*
Seam A       PASS，约15.13 s                   FAIL，约20.0 s内未完成
Seam B       PASS，约7.42 s                    FAIL，约20.0 s内未完成
Seam C       本次未在20 s内完成                 FAIL，约20.0 s内未完成
```

Smoke 期间已确认：Seam A/B/C 的焊缝端点、切向、真实焊丝轴和焊接侧均被正确传入；Gao 的首条可行解计时和超时返回正常。该 smoke 不能替代 30×3×2 正式任务实验。由于 MATLAB 规划器目前不支持外部硬中断，单个调用可能在内部计算结束后略超出预算；包装器会把超预算完整解标为 `PASS-LATE`/不计入对应 PASS@预算，并保留 wall time。

## 135. Gao 公平对比 5-seed 预实验（2026-09-27）

使用全新 seeds `20271001:20271005`、每条 Seam 5 个任务、最大 wall budget 60 s，运行：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/benchmark_task_level_arc_vs_gao.m
```

输出：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_task_pre5.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_task_pre5.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/fair_task_pre5_runs/
```

预实验结果（仅接口/计时预实验，不作为最终论文统计）：

```text
Seam A | ACCP      PASS@10 3/5，PASS@20 5/5，PASS@30 5/5，PASS@60 5/5；median TFF 7.713 s，P90 11.658 s
Seam A | Gao-AR-RRT*   PASS@10 0/5，PASS@20 0/5，PASS@30 0/5，PASS@60 0/5

Seam B | ACCP      PASS@10/20/30/60 = 5/5；median TFF 3.869 s，P90 4.013 s
Seam B | Gao-AR-RRT*   PASS@10/20/30 = 0/5，PASS@60 = 3/5；成功 median TFF 47.658 s，P90 51.428 s

Seam C | ACCP      PASS@10 1/5，PASS@20 1/5，PASS@30 4/5，PASS@60 5/5；median TFF 22.520 s，P90 58.953 s
Seam C | Gao-AR-RRT*   PASS@10/20/30/60 = 0/5
```

预实验只说明当前 Gao 复现基线在这些参数和 60 s 预算下明显更难首次完成 80 点受约束焊缝；不能据此声称 Gao 论文方法在一般问题上失败，也不能与论文报告时间直接横向比较。正式实验仍需至少每 Seam 30 个全新任务，并报告 PASS@预算、成功样本 TFF 分布、IK 调用和完整碰撞/TCP验收。

预实验汇总脚本的 percentile 索引与缓存刷新问题已修正；重新汇总不会重跑规划。ACCP 的 TCP 指标现在读取自最终自适应执行轨迹（例如 Seam A 约 0.236--0.332 mm、Seam B 最大约 0.491 mm、Seam C 最大约 0.481 mm），不再误报未修正的初始段间误差。

## 136. 三条焊缝路径动画（2026-09-28）

新增只读动画脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/record_three_weld_videos.m
```

脚本直接读取已通过完整节点/边碰撞与 TCP 验证的：

```text
fair_task_pre5_runs/Seam_A_arc_001_forward.mat
fair_task_pre5_runs/Seam_B_arc_001_forward.mat
fair_task_pre5_runs/Seam_C_arc_001_forward.mat
```

不重新规划、不修改 ROS2/CoppeliaSim/TCP 标定；按 43 mm/s TCP 速度回放 `qPathExecution`，保留原场景颜色与碰撞显示，显示实际焊丝和 TCP 轨迹，不显示 TCP 坐标箭头。每条焊缝输出透视和俯视两个 AVI：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_seam_videos/Seam_A_AARC_IK_43mmps_perspective.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_seam_videos/Seam_A_AARC_IK_43mmps_top.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_seam_videos/Seam_B_AARC_IK_43mmps_perspective.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_seam_videos/Seam_B_AARC_IK_43mmps_top.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_seam_videos/Seam_C_AARC_IK_43mmps_perspective.avi
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_seam_videos/Seam_C_AARC_IK_43mmps_top.avi
```

视频清单保存在 `weld_seam_videos/three_weld_video_manifest.mat`。重新录制：

```matlab
addpath('/home/ttbstt/jixiebi/UR5/matlab仿真');
record_three_weld_videos;
```

随后按用户要求又录制了 100 cm/min 版本（`0.016667 m/s = 16.667 mm/s`），输出目录为：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_seam_videos_100cmmin/
```

其中 6 个 AVI 文件均已用 MATLAB `VideoReader` 验证可读取，时长约 10.5--10.8 s，分辨率 1394×840。调用示例：

```matlab
o = struct('tcpSpeedMps',100/6000, ...
    'outputDir','/home/ttbstt/jixiebi/UR5/matlab仿真/weld_seam_videos_100cmmin');
record_three_weld_videos(o);
```

## 137. 三焊缝正式任务级对比实验（2026-09-28）

在不修改 ACCP、Gao-AR-RRT*、碰撞规则、端部余量、TCP 定义或实验口径的前提下，完成了：

```text
3 seams × 30 seeds × 2 algorithms = 180 tasks
max wall budget = 60 s
PASS@10/20/30/60 s derived from one 60 s run
```

正式输出：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/task_level_arc_vs_gao.csv
/home/ttbstt/jixiebi/UR5/matlab仿真/task_level_arc_vs_gao.mat
/home/ttbstt/jixiebi/UR5/matlab仿真/task_level_arc_vs_gao_runs/
```

正式汇总如下（TFF 统计只对 60 s 内成功样本；失败任务按右删失处理）：

```text
Seam A / ACCP      PASS@10 18/30  PASS@20 23/30  PASS@30 25/30  PASS@60 28/30
                       TFF median 7.251 s, P90 23.651 s

Seam A / Gao-AR-RRT*   PASS@10 0/30   PASS@20 0/30   PASS@30 0/30   PASS@60 8/30
                       TFF median 45.338 s, P90 47.666 s

Seam B / ACCP      PASS@10 30/30  PASS@20 30/30  PASS@30 30/30  PASS@60 30/30
                       TFF median 3.176 s, P90 3.269 s

Seam B / Gao-AR-RRT*   PASS@10 0/30   PASS@20 0/30   PASS@30 0/30   PASS@60 18/30
                       TFF median 48.925 s, P90 51.429 s

Seam C / ACCP      PASS@10 8/30   PASS@20 17/30  PASS@30 21/30  PASS@60 29/30
                       TFF median 17.936 s, P90 52.441 s

Seam C / Gao-AR-RRT*   PASS@10 0/30   PASS@20 0/30   PASS@30 1/30   PASS@60 5/30
                       TFF median 34.890 s, P90 45.076 s
```

总体：

```text
ACCP      PASS@10 56/90  PASS@20 70/90  PASS@30 76/90  PASS@60 87/90
               TFF median 4.048 s, P90 34.830 s

Gao-AR-RRT*   PASS@10 0/90   PASS@20 0/90   PASS@30 1/90   PASS@60 31/90
               TFF median 47.497 s, P90 51.260 s
```

成功样本的总体轨迹指标：

```text
ACCP      median IK calls 380,  P90 2688
               core continuation edge tests median 44, P90 51
               median max joint jump 7.460°, P90 10.026°
               median max segment TCP error 0.332 mm, P90 0.450 mm

Gao-AR-RRT*   median IK calls 7776, P90 7848
               median max joint jump 10.381°, P90 14.248°
               median max segment TCP error 1.010 mm, P90 1.759 mm
```

正式 benchmark 运行期间 MATLAB Figure 已设为隐藏，并在每个任务结束后关闭隐藏窗口；已完成任务由 `resume=true` 复用，没有重复覆盖或手动剔除失败任务。上述结果是当前冻结实现下的正式数据，不应再与开发集混用。

## 138. Zotero 只读连接与文献改进方案（2026-09-28）

Zotero Desktop Local API 已启用并复核正常：

```text
profile: /home/ttbstt/.zotero/zotero/gm1m91kv.default
Zotero: 10.0.3
Local API: http://127.0.0.1:23119
api_status: 200
connector_status: 200
```

本次只执行了状态、目录、集合和全文读取，没有新增、删除或修改任何 Zotero 条目。

库统计：

```text
顶层记录总数：99
可引用书目记录：33
集合/分类数量：5
```

随机抽取的 5 篇标题：

```text
Research on Motion Trajectory Correction Method for Wall-Climbing Robots Based on External Visual Localization System
A review of applications of collaborative robot in welding and additive manufacturing
Intelligent path planning for robotic welding of precast beam reinforcement frameworks using an improved ant colony optimization
Capacitive bimodal flexible sensor for enhanced proximity and pressure sensing enabled by co-planar interdigitated sensing electrode patterns
焊接机器人视觉感知及智能化焊接关键技术研究进展
```

已精读并用于算法分析的相关文献：

```text
YS8CJXS4  Handling Transitions Across Singularities for UR-Like Serial Robots
4Q4SHSAX  IKLink: End-Effector Trajectory Tracking with Minimal Reconfigurations
3ABDWD5D  Predictive inverse kinematics with trajectory scaling for redundant manipulators based on quadratic optimization
J5XQXFXT  Spatial Obstacle Avoidance for Dual-Arm Welding Robots in Ship Hull Subassemblies and Intermediate Assemblies
H37G9RPF  Innovation in robot welding trajectory control: Online synchronous planning of welding paths and torch postures for structures with unknown spatial geometries
```

新建独立文献改进方案：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/literature_guided_arc_ik_improvement_plan.md
```

当前不修改正式 ACCP/Gao benchmark。推荐的新实验顺序为：

```text
奇异位形指标与 branchSignature 诊断
→ 奇异触发的局部分支扩展
→ 局部 IKLink 风格候选 DP
→ 2~3 点前瞻式触发
→ 新 seeds 独立回归测试
```

不建议直接移植完整 Gao-RRT*、全局增大 beam 或完整 PQPIK-S；这些会破坏当前速度基线或改变 TCP/时间参数化口径。

## 139. IKLink 官方源码 sanity check 与 UR5 Seam A 适配（2026-09-28）

官方仓库已独立拉取到：

```text
/home/ttbstt/jixiebi/UR5/third_party/IKLink
```

仓库：`uwgraphics/IKLink`，commit：`d2edc00`。Rust 工具链已安装到用户目录，官方源码编译成功：

```text
cargo build  -> success
cargo run --bin traj_tracing -> success
Panda demo: Min Num of Reconfig = 2
```

新增适配脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/export_iklink_seam_input.m
/home/ttbstt/jixiebi/UR5/matlab仿真/validate_iklink_ur5_seam.m
```

生成的 UR5 Seam A 输入和官方输出：

```text
/home/ttbstt/jixiebi/UR5/third_party/IKLink/input_trajectories/ur5_seamA_arc001.csv
/home/ttbstt/jixiebi/UR5/third_party/IKLink/output_motions/ur5_seamA_arc001.csv
```

输入使用 MATLAB 已验收的 `tool0→WeldTCP` 标定变换转换成 IKLink `tool0` 目标，17 个焊缝点。官方 UR5 IKLink 输出：

```text
Min Num of Reconfig = 0
最大关节跳变 = 9.876°
```

统一 MATLAB 严格验证报告：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/iklink_ur5_seamA_validation.txt
```

结果：

```text
姿态误差最大约 0.163°
TCP位置误差最大约 2.113 mm
节点碰撞：17/17
插值边碰撞：16/16
连续焊接 PASS：否
```

结论：官方 IKLink demo 和 UR5 配置正常，但其当前目标函数/配置没有接入本项目的焊枪、焊丝、工件和严格边碰撞规则。因此不能把官方输出直接当作焊接成功；后续必须保持 `IKLink candidate/DP → MATLAB统一validator` 的公平接口，并单独实现 `reconfiguration=0` 的连续焊接适配版本。

## 140. IKLink MATLAB 同平台 baseline（2026-09-28）

为避免 Rust IKLink 的 URDF/tool/collision 环境与本项目不一致，新建了同一 MATLAB 场景中的 IKLink 风格 Planner：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/replan_weld_iklink_baseline.m
```

核心流程：

```text
每个焊点生成多 IK candidate
→ 节点姿态/焊接侧/关节限位/机器人+焊枪+焊丝+工件+自碰撞过滤
→ 相邻层两两尝试连接
→ 15°关节连续性约束
→ 完整插值边碰撞检测
→ 分层动态规划回溯
```

接口支持：

```matlab
r = replan_weld_iklink_baseline(options);
r = replan_weld_iklink_baseline(scene,seamSpec,options);
```

默认 `poseMode="fixed"`，对应固定焊枪姿态的 IKLink baseline；`poseMode="ar"` 可在相同合法 travel/roll 姿态空间内做 layered candidate DP。输出统一包含 `qPath`、`success`、`tff`、`ikCalls`、`nodeChecks`、`edgeChecks`、`maxJointJumpDeg`、`maxTcpErrorMm`、`failureReason` 等字段。

独立 80 点 Seam A fixed-pose smoke test：

```text
time              = 25.336 s
IK calls          = 5200
first empty layer = 26
edge tests/free   = 27 / 27
complete          = 0
```

结果文件：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/iklink_fixed_seamA_smoke.mat
```

这只是 baseline smoke test，不覆盖正式 ACCP/Gao benchmark。固定姿态在第 26 层断开，说明后续必须分别报告 `IKLink-FixedPose` 与同平台 `IKLink-AR`，不能把固定姿态失败误写成整个 IKLink 方法失败。

## 141. Gao-AR-RRT* 三条焊缝视频（2026-09-28）

新增只读播放录制脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/record_three_weld_gao_videos.m
```

## 143. 二维 RRT 算法 GIF 动画（2026-09-29）

用户要求将 RRT 项目目录中的二维算法动态图录制为 GIF。未修改任何二维规划、采样、碰撞
检测或路径算法逻辑；仅在共享二维绘图接口增加了可选 GIF 录制钩子，并使用
`RRT_HEADLESS=1` 运行，因此录制过程中不会弹出 MATLAB 窗口。

录制统一使用 `defaultRRTConfig` 的二维地图、起点 `[10,10]`、目标 `[90,90]` 和 `seed=1`。
画面配色保持项目原有约定：障碍物灰色、起点绿色、目标红色、搜索树使用各算法原有颜色、
最终路径蓝色；成功路径末尾追加紫色运动轨迹回放。

生成脚本与结果：

```text
/home/ttbstt/jixiebi/RRT/benchmark/record_all_2d_rrt_gifs.m
/home/ttbstt/jixiebi/RRT/results/2d_rrt_gifs/
```

共生成 10 个二维 GIF：Basic RRT、RRT-Connect、RRT*、Bi-RRT、Bi-RRT*、Informed RRT*、
Q-RRT*、GD-RRT*、Classified RRT*、APF-Bi-RRT*。每个文件为 `900×700`，并通过 MATLAB
`imfinfo` 检查可读取、帧数正常；汇总数据保存在：

```text
/home/ttbstt/jixiebi/RRT/results/2d_rrt_gifs/summary.csv
/home/ttbstt/jixiebi/RRT/results/2d_rrt_gifs/summary.mat
```

本次仅为二维动态图展示，不代表 UR5 六维规划 benchmark，也不改变原始 RRT-Connect
ground-truth 或后续 `30 s best-so-far` 实验定义。

使用正式 task-level Gao 结果中首个完整 PASS 轨迹：

```text
Seam A -> Seam_A_gao_008_result.mat
Seam B -> Seam_B_gao_002_result.mat
Seam C -> Seam_C_gao_001_result.mat
```

输出目录：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_seam_videos_gao_100cmmin/
```

有效视频（原规划场景颜色，含焊枪、焊丝、工件、理论焊缝和 TCP 轨迹，无 TCP 坐标箭头）：

```text
Seam_A_Gao_AR_RRTstar_100cmmin_perspective.avi
Seam_B_Gao_AR_RRTstar_100cmmin_perspective.avi
Seam_C_Gao_AR_RRTstar_100cmmin_perspective.avi
```

速度标签为 `100 cm/min`。A 视频为 30 fps，B/C 为 15 fps；帧率差异不改变轨迹播放速度。三段视频均通过 ffprobe 完整读取，时长约 109.4 s、108.9 s、112.9 s。录制过程中产生的未完成 A 俯视文件已改名为：

```text
Seam_A_Gao_AR_RRTstar_100cmmin_top.incomplete.avi
```

该文件不可作为结果使用。

## 142. Gao 视频完整取景修正（2026-09-29）

用户反馈旧 Gao 视频只显示枪头。原因是录制脚本相机边界只使用 TCP/焊缝点，未把 UR5 连杆和工件代理纳入边界。已修改：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/record_three_weld_gao_videos.m
```

新版本相机边界包含：

```text
全部 UR5 body frame
WeldingGunMount / WeldTCP / wire
四个 Workpiece collision proxy 的完整包围盒
```

完整视频输出到：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_seam_videos_gao_100cmmin_full/
```

有效视频：

```text
Seam_A_Gao_AR_RRTstar_100cmmin_perspective.avi
Seam_B_Gao_AR_RRTstar_100cmmin_perspective.avi
Seam_C_Gao_AR_RRTstar_100cmmin_perspective.avi
```

三个视频均为完整 UR5 + 焊枪 + 焊丝 + U 型工件取景，速度标签为 100 cm/min，录制帧率 10 fps；帧率降低不改变轨迹速度。录制后已用系统 ffprobe 验证三个 AVI 可读取。

## 140. Phase 12A-0R：原始 RRT-Connect 与恢复版 wrapper 审计（2026-09-28）

针对近期把 `0.5°`、后验验证、动画插值和正式成功判据混在一起的问题，已读取本交接文档及项目内 RRT 文件，并暂停新的 benchmark。详细审计记录：

```text
/home/ttbstt/jixiebi/RRT/docs/phase12a0r_original_vs_restored_audit.md
```

审计结论：原始规划脚本
`/home/ttbstt/jixiebi/UR5/matlab仿真/plan_weld_approach_exit_rrt_connect.m`
明确包含：

```text
RRT 搜索边分辨率       = 2 deg
独立路径后验验证分辨率 = 0.5 deg
动画自由空间帧步长     = 独立的 stepDeg，当前动画默认 1 deg
```

因此 `0.5 deg` 不是只来自动画；旧交接文档只是没有把原始脚本中的 `validationResolutionDeg=0.5`
记录清楚。原始代码没有后来增加的 `2 s + 0.125 deg strict certification` 门槛。

同时确认当前恢复版 wrapper 并非原始 RRT-Connect 的逐行复现：它使用共享 `jointPlannerCore`，
增加了 state/edge cache、内部候选路径复核和 30 s wall-clock guard，且当前 `runConnect` 的树
选择/交换写法与原始本地 `localRrtConnect` 不同。故 `phase12a_seam_approach_exit_pilot_40`
只能归档为：

```text
diagnostic / restored-pilot attempt under current shared core
```

不能直接作为原始 RRT-Connect ground-truth benchmark。已复现的原始基线仍为：

```text
qHome -> qPreWeld : 12.219553 s，本次重跑 PASS
qPostWeld -> qHome:  6.786776 s，本次重跑 PASS
```

对应历史记录 `11.898240 s / 6.752958 s`，路径长度、迭代数、节点数、碰撞检查数和边检查数一致。
后续若继续，必须先冻结原始 RRT-Connect 作为 ground-truth implementation，再让其他算法仅替换
搜索器并共用相同的场景、边插值、碰撞接口和成功判据。当前审计阶段未启动新的 40-run、160-run
或 100-seed 实验。

## 141. Phase 12A anytime 首次方法验证（2026-09-28）

研究目标已固定为：在原始 Seam A 场景和碰撞条件不变的情况下，同时记录首解时间/首解路径长度，
以及统一 30 s 预算内的 best-so-far 路径长度。新增入口：

```text
/home/ttbstt/jixiebi/RRT/benchmark/run_phase12a_anytime_home_preweld.m
```

固定任务为 `qHome -> qPreWeld`；搜索边 `2 deg`；独立后验验证 `0.5 deg`；关节步长 `12 deg`；
目标偏置 `0.20`；总预算 `30 s`。RRT-Connect 首个认证解后停止，三个 RRT* 变体继续搜索，
只有通过同一后验验证的候选才更新 best path。入口设置 `RRT_HEADLESS=1`，不会弹出 MATLAB 场景窗口。

`seed=1` 单次方法验证结果：

```text
RRT-Connect       T_first=3.058 s, L_first=L_best@30=5.7018 rad
Bi-RRT*           T_first=8.524 s, L_first=6.3495 rad, L_best@30=5.1845 rad
ClassGoal-Lite    30 s 内未找到认证首解
Full Classified   30 s 内未找到认证首解
```

Bi-RRT* checkpoint：`L@10=6.3495`、`L@15=6.3495`、`L@20=5.2487`、`L@30=5.1845 rad`；
RRT-Connect 首解后保持 `5.7018 rad`。这只是方法验证，不是最终算法排名。

结果文件：

```text
/home/ttbstt/jixiebi/RRT/results/phase12a_anytime_home_preweld_seed1.mat
/home/ttbstt/jixiebi/RRT/results/phase12a_anytime_home_preweld_seed1.csv
```

时间—路径质量图：

```text
/home/ttbstt/jixiebi/RRT/results/phase12a_anytime_home_preweld_seed1_time_quality.png
```

本次回归已运行 joint-space 基础测试和 `test_phase12a_joint_planners`；默认 planner 仍保持首解
模式，RRT-Connect/Bi-RRT* smoke 路径有效。

后续如继续，只增加固定 seed 和 `qPostWeld -> qHome` 任务，不改变原始碰撞分辨率，也不恢复
0.25/0.125 deg 认证支线。

## 142. 三条焊缝完整演示视频恢复原始演示速度（2026-09-29）

用户要求把 ACCP 与 Gao-AR-RRT* 视频从焊接工艺演示速度恢复到原来的快速演示速度。
本次只改变视频重采样/播放速度，不修改任何 `qPath`、规划参数、碰撞模型、ROS2、CoppeliaSim
或 TCP 标定。

固定视频速度：

```text
tcpSpeedMps = 0.043 m/s
             = 43.0 mm/s
             = 258.0 cm/min
```

两套算法均重新录制了三条焊缝的完整机器人、焊枪、焊丝、工件、理论焊缝与 TCP 轨迹，保留
透视和俯视两个视角；视频帧率为 30 fps，视频尺寸为 1394×840。

ACCP 视频目录：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_seam_videos_43mmps_full/
```

包含：

```text
Seam_A_AARC_IK_43mmps_perspective.avi
Seam_A_AARC_IK_43mmps_top.avi
Seam_B_AARC_IK_43mmps_perspective.avi
Seam_B_AARC_IK_43mmps_top.avi
Seam_C_AARC_IK_43mmps_perspective.avi
Seam_C_AARC_IK_43mmps_top.avi
three_weld_video_manifest.mat
```

Gao-AR-RRT* 视频目录：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_seam_videos_gao_43mmps_full/
```

包含：

```text
Seam_A_Gao_AR_RRTstar_43mmps_perspective.avi
Seam_A_Gao_AR_RRTstar_43mmps_top.avi
Seam_B_Gao_AR_RRTstar_43mmps_perspective.avi
Seam_B_Gao_AR_RRTstar_43mmps_top.avi
Seam_C_Gao_AR_RRTstar_43mmps_perspective.avi
Seam_C_Gao_AR_RRTstar_43mmps_top.avi
gao_three_weld_video_manifest.mat
```

视频录制脚本：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/record_three_weld_videos.m
/home/ttbstt/jixiebi/UR5/matlab仿真/record_three_weld_gao_videos.m
```

此外，ACCP 录制器已改为按文件名排序自动选择每条焊缝的第一个完整 PASS 结果；因此
当 `arc_001_forward.mat` 失败但其他方向/恢复结果通过时，不会错误地拒绝录制（本次 Seam C
使用了已验证的 PASS 结果）。原有 `100cmmin` 视频目录保持不变。
## 106. UR5 CB3 真机反馈链只读验收（2026-09-29）

已连接真实 UR5（IP `192.168.0.10`），使用 ROS 2 Humble 官方 `ur_robot_driver`，并加载
`/home/ttbstt/ur5_calibration.yaml`。RTDE/URControl 连接成功，标定 checksum 验证通过。
示教器 External Control 已运行；本次只读诊断，没有发送轨迹、没有激活运动控制器、没有改变示教器参数。

当前控制器状态：

```text
joint_state_broadcaster          active
tcp_pose_broadcaster             active
speed_scaling_state_broadcaster  active
io_and_status_controller         active
scaled_joint_trajectory_controller inactive
joint_trajectory_controller       inactive
```

六个关节的 position/velocity/effort command interfaces 均为 available/unclaimed，说明硬件接口
可用，但运动控制器尚未占用。

真机 `/joint_states` 按 UR 关节顺序映射后的当前角度（rad）：

```text
[-3.9057189, -1.1526178, -2.2383769, -0.2658656, 1.5029256, -2.7651277]
```

对应角度（deg）：

```text
[-223.781, -66.040, -128.250, -15.233, 86.111, -158.430]
```

真机 TCP broadcaster（frame `base`）反馈位置约为：

```text
[-0.387343, 0.559111, 0.254759] m
```

根据现有 `base -> base_link` 固定坐标变换换算后，与 MATLAB calibrated URDF 的 `tool0` 和旧的
MATLAB `WeldTCP` 进行只读比较，结果为：

```text
旧 MATLAB WeldTCP 位置误差：约 50.130 mm
旧 MATLAB WeldTCP 姿态误差：约 154.684 deg
```

由真机 TCP 反馈反推的 `tool0 -> TCP` 与示教器照片中的 TCP 参数
`[13.53, 17.13, 379.83 mm]`、旋转向量 `[-0.9562, 2.4566, 1.2418] rad` 一致（约 0.079 mm、
0.007 deg 差异）。因此真机当前激活 TCP 正常，问题在于旧 MATLAB WeldingGun/WeldTCP 标定
不能直接用于真机。

现有 RRT 结果的 `qHome = [180,-90,0,-90,0,0] deg` 与真机当前构型不一致；最短角差最大约
158.430 deg。因此禁止直接把现有焊缝轨迹发送给真机。下一步应建立独立的“真机 TCP 标定/模型”
（不要覆盖 CoppeliaSim 标定），再重新做 MATLAB FK 与真机 TCP 的毫米级验收；验收通过后才可
激活 `scaled_joint_trajectory_controller` 并进行极小幅、低速的单关节测试。

## 107. 真机姿态再次调整后的只读核对（2026-09-29）

用户将真机姿态调整到接近仿真姿态后重新读取。当前真机 URDF/ROS 顺序关节角为：

```text
qReal [rad] = [-1.0577, -1.5380, -0.1180, -1.4410, -0.1476, -1.6950]
qReal [deg] = [-60.600, -88.122, -6.762, -82.566, -8.455, -97.116]
```

相对于当前仿真规划参考 `qHome = [180,-90,0,-90,0,0] deg`，最短连续关节差为：

```text
[119.400, 1.878, -6.762, 7.434, -8.455, -97.116] deg
```

因此第 2～5 轴已接近参考姿态，但第 1 轴仍差约 119.4°、第 6 轴仍差约 97.1°；视觉上可能相似，
但不能作为轨迹起点一致的数值验收。

当前真机 TCP（driver frame `base`）为：

```text
position = [-0.5271, -0.2175, 1.0109] m
```

换算到 MATLAB `base_link` 后，旧 MATLAB WeldTCP 仍与真机存在约 50.165 mm、154.689°差异。
本次仍未激活运动控制器、未发送任何运动命令。下一步应先决定是继续把真机调整到准确的规划起点，
还是以当前真机姿态为新起点重新规划自由空间接近路径；不能直接执行旧 `qHome` 轨迹。

## 108. 关于将真机调整到仿真姿态的安全结论（2026-09-29）

用户询问是否可以将真机调整到仿真姿态。当前不能直接把 MATLAB 仿真中的
`[180,-90,0,-90,0,0] deg` 作为真机 ROS 目标发送，因为该数组属于旧 MATLAB
`universalUR5` 坐标约定，真机 `/joint_states` 使用 ROS/URDF 关节坐标；两者之间还涉及
旧 CoppeliaSim `qOffset` 与 base/tool 坐标定义。当前运动控制器保持 inactive，未发送运动。

在得到 CoppeliaSim 目标的六个原始关节角并完成 MATLAB/ROS 坐标映射验收前，禁止执行大幅
姿态调整。必要时应先用示教器低速手动对照，或先完成一个只读的仿真关节值导出和坐标映射验证。

## 109. CoppeliaSim 界面卡顿诊断（2026-09-29）

CoppeliaSim 进程仍在运行，ZMQ Remote API `23000` 正常，远程读取场景对象和关节值正常，
不是进程崩溃。诊断发现：

```text
simulation state = simulation_advancing_running
boolparam_realtime_simulation = false
boolparam_dynamics_handling_enabled = true
```

仿真时间约 1 秒内推进了 16 秒，主计算线程约 94% CPU，因此 GUI 表现为卡死。UR5 六个关节
在短时间内基本不变，未发现新的轨迹运动指令。

处理方式：先在 CoppeliaSim 点击暂停/停止，再打开 Real-time simulation（实时仿真）选项；如果
只是读取目标姿态，可以保持仿真停止后直接运行 `read_tool_frames.py`，不需要启动仿真。

## 110. 读取 CoppeliaSim 当前目标姿态（2026-09-29）

CoppeliaSim 当前已停止，读取成功。场景中的六个 UR5 关节均为零位：

```text
qCop [rad] = [0, 0, 0, 0, 0, 0]
qCop [deg] = [0, 0, 0, 0, 0, 0]
```

按项目旧映射，对应 MATLAB `universalUR5` 参考关节为：

```text
qMat = [180, -90, 0, -90, 0, 0] deg
```

本次只读取 CoppeliaSim，没有向真机发送任何目标。由于 MATLAB/Coppelia 的 `qOffset` 与真机
ROS/URDF 坐标定义尚未完成独立验收，`qMat` 不能直接作为真机命令；下一步必须先建立并验证
`qCop -> qReal` 的坐标映射。

## 111. CoppeliaSim -> 真机 ROS/URDF 关节映射验收（2026-09-29）

使用 3 组不同关节姿态进行只读 FK 比较：

```text
qMat = qCop + qOffset
qReal(ROS/URDF) = qMat
```

该映射下，`universalUR5` 与 `/home/ttbstt/UR5/real_robot_description_calibrated.urdf` 的末端变换
在 3 组测试姿态中保持近似固定差异：位置约 1.4–2.1 mm，姿态约 0.15–0.17°。而直接使用
`qReal = qCop` 时，末端相对变换随姿态变化，不是固定误差，因此该方式错误。

所以当前 CoppeliaSim 目标：

```text
qCop = [0,0,0,0,0,0] deg
```

对应真机 ROS/URDF 目标应为：

```text
qReal = [180,-90,0,-90,0,0] deg
```

为从当前真机姿态选择最短连续分支，可表示为：

```text
qReal_continuous = [-180,-90,0,-90,0,0] deg
```

相对当前真机角度，目标差异最大约 119.4°（J1）和 97.1°（J6）。本次仍未激活运动控制器，
未发送任何运动指令；该目标只完成坐标映射验收，不能视为已批准执行。

## 112. 最小真机动作测试前的最新姿态确认（2026-09-29）

重新只读读取真机后，当前 ROS/URDF 顺序关节角为：

```text
qReal [rad] = [-1.0576833, -1.5380123, -0.1180356, -1.4410718, -0.1476110, -1.6949895]
qReal [deg] = [-60.600, -88.122, -6.763, -82.566, -8.455, -97.115]
```

此前记录的 `[-223.781,-66.040,-128.250,-15.233,86.111,-158.430] deg` 已过期，不能作为本次
真机动作测试的起点。若只测试 J5（`wrist_2_joint`）+1°，只读计算出的目标应为：

```text
qTest [deg] = [-60.600, -88.122, -6.763, -82.566, -7.455, -97.115]
```

该目标尚未发送；运动控制器仍保持 inactive，等待现场安全确认。

## 113. J5 最小动作测试前置检查（2026-09-29）

收到现场安全确认后，曾只读/可逆地激活 `scaled_joint_trajectory_controller` 进行前置检查，未发布
任何 `JointTrajectory`。检查结果：

```text
speed_scaling = 0.0
robot_program_running 未读到 true
```

## 144. RRT 项目 V1 Fast-Connect Anytime RRT*（2026-09-30）

在 `/home/ttbstt/jixiebi/RRT` 新增了独立 V1 planner：先用现有双树 RRT-Connect
快速建立首条路径，再保留两棵树执行 Bi-RRT* 风格 ChooseParent/Rewire 优化。未修改原始
RRT-Connect、Bi-RRT*、UR5 场景、碰撞模型、ROS2、CoppeliaSim 或焊缝路径。

Task A `qHome -> qPreWeld`、seed `20260917`、30 s、搜索 `2°`、最终 dense validation `0.5°`：

```text
RRT-Connect  Tfirst=0.900 s  Lfirst=L30=5.701772 rad
Bi-RRT*       Tfirst=6.558 s  Lfirst=6.349535 rad  L30=5.184512 rad
V1            Tfirst=0.766 s  Lfirst=5.701772 rad  L30=5.173362 rad
```

三条最终路径均通过 `0.5°` dense validation。V1 Stage 1 与同参数 RRT-Connect 均为 `35`
次迭代、首解路径相同；V1 improvement events 已保存为独立 CSV。结果和图位于：

```text
/home/ttbstt/jixiebi/RRT/results/v1_fast_connect_anytime_taskA_seed20260917.mat
/home/ttbstt/jixiebi/RRT/results/v1_fast_connect_anytime_taskA_seed20260917.csv
/home/ttbstt/jixiebi/RRT/results/v1_fast_connect_anytime_taskA_seed20260917_improvement_events.csv
/home/ttbstt/jixiebi/RRT/results/v1_fast_connect_anytime_taskA_time_quality.png
```

说明示教器 External Control 程序当前没有真正运行，尚不具备正常运动条件。随后已将
`scaled_joint_trajectory_controller` 恢复为 `inactive`；`joint_state_broadcaster` 仍为 `active`。
没有发生真机运动。

## 145. V1 Seam A 双向任务验证（2026-09-30）

正式 V1 任务现已覆盖两个自由空间方向：

```text
Task A: qHome -> qPreWeld + PreWeld -> Point1 局部接近
Task B: Point80 -> qPostWeld 局部退出 + qPostWeld -> qHome
```

RRT 算法比较只对 `qHome -> qPreWeld` 和 `qPostWeld -> qHome` 两个自由空间段进行；局部
连接以及 Point1 -> Point80 的既有焊缝路径保持不变。Task B 新增入口为：

```text
/home/ttbstt/jixiebi/RRT/benchmark/run_v1_fast_connect_anytime_taskB.m
```

在 seed `20260917`、30 s anytime、2 deg 搜索边、0.5 deg 后验 dense validation 下，Task B：

```text
RRT-Connect                    Tfirst=1.896 s  Lfirst=L30=6.688920 rad
Bi-RRT*                        Tfirst=8.170 s  Lfirst=5.140525 rad  L30=5.095463 rad
Constraint-aware Bidirectional RRT* (CBi-RRT*)   Tfirst=1.784 s  Lfirst=6.688920 rad  L30=4.729628 rad
```

三种算法 Task B 最终路径均通过 0.5 deg 后验验证。Task A + Task B 联合汇总文件为：

```text
/home/ttbstt/jixiebi/RRT/results/v1_seamA_taskA_taskB_comparison.csv
/home/ttbstt/jixiebi/RRT/results/v1_seamA_taskA_taskB_combined.csv
/home/ttbstt/jixiebi/RRT/results/v1_seamA_taskA_taskB_combined.mat
```

固定 seed 的两段 best-so-far 路径长度合计为：RRT-Connect `12.390692 rad`、Bi-RRT*
`10.279975 rad`、V1 `9.902990 rad`；两段首解时间合计分别为 `2.795 s`、`14.728 s`、
`2.551 s`。这只是 Task A/Task B 联合功能验证，不是多 seed 正式排名；V2、Phase 12B 和真机
执行时间参数化尚未开始。运行采用 `RRT_HEADLESS=1`，不会弹出 MATLAB 窗口。

## 146. V1 Seam A 完整路径规划视频（2026-09-30）

已使用 V1 Task A/Task B 的已保存 qPath 录制完整 Seam A 离线演示，未重新规划、未发送
ROS2/CoppeliaSim 命令。视频包含：

```text
Home -> PreWeld -> Point1 -> Point80 -> PostWeld -> Home
```

其中 `PreWeld -> Point1` 和 `Point80 -> PostWeld` 是原有确定性局部连接，焊缝段是原有
Point1 -> Point80 路径，RRT 自由空间段为 Home -> PreWeld 与 PostWeld -> Home。

输出视频：

```text
/home/ttbstt/jixiebi/RRT/results/v1_fast_connect_anytime_videos/v1_seamA_full_sequence_V1FastConnectAnytimeRRTStar_perspective.avi
/home/ttbstt/jixiebi/RRT/results/v1_fast_connect_anytime_videos/v1_seamA_full_sequence_V1FastConnectAnytimeRRTStar_top.avi
```

两个视频均为 1394×840、30 fps、527 帧、约 17.57 s，并通过 VideoReader/ffprobe 验证可读。
录制保持主场景光源和原颜色，且确认包含 `PostWeld -> Home` 退出段。可复现脚本为：

```text
/home/ttbstt/jixiebi/RRT/benchmark/make_v1_seamA_video_report.m
/home/ttbstt/jixiebi/RRT/benchmark/animate_phase12a_anytime_sequence.m
```

## 114. ROS2 驱动重新连接后的状态（2026-09-30）

官方 UR ROS2 驱动已重新启动并成功连接网络/RTDE，机器人 IP 为 `192.168.0.10`，但当前真机状态为：

```text
robot_mode = 3 (POWER_OFF)
safety_mode = 1
speed_scaling = 0.0
robot_program_running 未读到 true
```

因此 `/joint_states` 当前全为零，不能当作真实关节角使用。`scaled_joint_trajectory_controller` 和
`joint_trajectory_controller` 均保持 `inactive`，没有发送任何轨迹或运动指令。需要先在示教器上
上电、释放抱闸、运行已保存的 External Control 程序，再重新读取状态。

## 115. 真机反馈恢复（2026-09-30）

重新启动 ROS2 UR 驱动并在示教器上完成上电后，真机状态恢复为非零反馈：

```text
robot_mode = 7
safety_mode = 1
speed_scaling = 0.0
```

当前关节反馈（ROS/URDF canonical order，rad）约为：

```text
[-1.057707, -1.538060, -0.118120, -1.441084, -0.148354, -1.694582]
```

TCP broadcaster 也有有效非零位姿反馈。由于 speed scaling 仍为 `0.0`，当前只确认了 RTDE 和
状态反馈链，尚未确认 External Control 已允许运动；轨迹控制器保持 inactive，未发送动作。

## 116. External Control 运行后复核（2026-09-30）

用户在示教器按下 Play 后复核，结果仍为：

```text
robot_program_running：未收到 true
speed_scaling：0.0
trajectory controllers：inactive
```

随后 ROS2 驱动日志出现 RTDE stream lost/reconnect 循环以及 `Background reading is not running`，
驱动已停止。此次没有发送轨迹，也没有发生由 ROS2 发起的机器人运动。下一次应先在示教器确认
程序树的执行光标确实停留在 External Control 节点、无保护停止，并避免在 External Control
运行期间点击其他运动相关按钮；然后再重新启动驱动进行状态验证。

## 117. 重新按正确顺序启动驱动后的状态（2026-09-30）

本次使用 `headless_mode:=false` 启动 ROS2 Driver，RTDE 和真实标定校验成功。驱动启动后读取到：

```text
robot_mode = 5 (IDLE)
speed_scaling = 0.0
robot_program_running 未收到 true
```

`/joint_states` 已恢复有效非零姿态，但 External Control 尚未进入运行态。应在 ROS2 Driver
启动完成后，再在示教器上重新按 Play 运行包含 External Control 节点的程序；在确认运行标志和
速度倍率之前，不激活轨迹控制器、不发送运动。

## 118. External Control 端口与 Wrist 2 小动作验证（2026-09-30）

本次按“先启动 Driver，再运行示教器 External Control”的顺序操作。ROS2 Driver 使用：

```text
robot_ip=192.168.0.10
reverse_ip=192.168.0.20
reverse_port=50001
script_sender_port=50002
headless_mode=false
activate_joint_controller=false
kinematics_params_file=/home/ttbstt/ur5_calibration.yaml
```

确认 PC 端脚本发送端口已监听：

```text
0.0.0.0:50002 LISTEN (ur_ros2_control)
```

示教器 External Control 运行后，读取到：

```text
robot_mode = 7
speed_scaling = 100.0
scaled_joint_trajectory_controller = active
joint_state_broadcaster = active
io_and_status_controller = active
```

`robot_program_running` 话题本次没有返回样本，但 robot_mode=7 且 speed_scaling=100% 与控制器
已能激活；因此仅以这两个状态作为本次小动作前的运行态判断。未发送焊缝轨迹。

完成一次低速 Wrist 2（`wrist_2_joint`）局部测试：从实时反馈姿态出发，约 3 s 移动 `+1°`，
再约 3 s 返回。测试发布的关节顺序为标准 UR 顺序：

```text
[shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
 wrist_1_joint, wrist_2_joint, wrist_3_joint]
```

测试起点约为（rad）：

```text
[ 0.00570412, -1.57032854, -0.00241739,
 -1.56669218,  0.00918741, -0.00776941 ]
```

目标仅将 `wrist_2_joint` 增加 `0.0174533 rad`（1°），随后回到起点。测试结束后反馈约为：

```text
[ 0.00572809, -1.57030422, -0.00238115,
 -1.56669218,  0.00921138, -0.00778133 ]
```

结论：ROS2 Driver → scaled trajectory controller → 真机的最小运动闭环验证通过，机器人已回到
原位附近；本次没有发送完整焊缝或 RRT 轨迹。控制器当前仍为 active，后续发送任何轨迹前仍需
重新确认现场安全、External Control 运行态、speed_scaling 和轨迹起点与实时关节角一致。

## 119. 独立 PC 端 UR5 Safe HMI（2026-09-30）

新增目录：

```text
/home/ttbstt/jixiebi/UR5/ur5_hmi/
```

该模块使用 PyQt5 + ROS2 `rclpy`，不修改稳定 ROS2 Driver、CoppeliaSim、MATLAB 或真机标定。
启动时只监控，不自动激活控制器、不自动发送轨迹。

主要文件：

```text
main.py                         入口
gui/main_window.py              主窗口、状态区、Jog、加载/验证/确认执行
gui/widgets.py                  状态标签和 Jog 按钮
ros/ros_node.py                 ROS2 订阅、控制器状态和 TCP 状态
ros/trajectory_client.py        FollowJointTrajectory 非阻塞 action 客户端
ros/state_monitor.py            实时状态数据结构
safety/safety_checks.py         运动安全门
safety/joint_limits.py          软件关节限位
trajectory/trajectory_loader.py MAT/YAML/JSON 读取
trajectory/trajectory_validator.py 结构、限位、相邻跳变验证
trajectory/trajectory_utils.py  Jog 轨迹生成和连续关节值处理
config/hmi_config.yaml          ROS topic/action 与默认 Jog 参数
README.md                       使用说明
```

启动命令：

```bash
source /opt/ros/humble/setup.bash
cd /home/ttbstt/jixiebi/UR5/ur5_hmi
python3 main.py
```

HMI 默认监控：

```text
/joint_states
/io_and_status_controller/robot_program_running
/io_and_status_controller/robot_mode
/io_and_status_controller/safety_mode
/speed_scaling_state_broadcaster/speed_scaling
/tcp_pose_broadcaster/pose
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

Jog/执行按钮只有在以下条件同时满足时才启用：关节反馈新鲜、External Control running、
`robot_mode=7`、`safety_mode=1`、speed scaling 大于 0、scaled controller active、action server
就绪，且轨迹起点与最新真实关节角一致。Jog 每次重新读取实时关节角，只改变一个关节，默认
0.5°/5%，不做 `wrapToPi`。`STOP MOTION` 仅取消 FollowJointTrajectory goal，不能替代示教器
急停。HMI 不声称完成碰撞检测，完整 RRT/焊缝轨迹仍需独立验证。

静态验证已完成：`python3 -m compileall` 通过；PyQt5 离屏启动 smoke test 无 Python traceback。

## 120. HMI 实时 UR5 视图与 RViz 入口（2026-09-30）

在 `UR5/ur5_hmi` 中新增只读图形显示，不改变 ROS2 控制链路：

```text
gui/robot_view.py          基于 Matplotlib Qt 后端的轻量 UR5 骨架视图
config/ur5_hmi.rviz       RViz2 RobotModel + TF + Grid 配置
run_hmi.sh                 source ROS2 环境后启动 HMI
```

轻量视图直接解析：

```text
/home/ttbstt/jixiebi/UR5/real_robot_description_calibrated.urdf
```

使用 URDF 中的 joint origin、axis、fixed/revolute 关系计算显示用 FK；实时关节角只来自
`/joint_states`，TCP marker 只来自 `/tcp_pose_broadcaster/pose`。图形 FK 不用于控制、IK 或
碰撞检测。若 ROS2 发布 `/robot_description`，HMI 会优先加载该实时 URDF 字符串，失败时回退
到本地标定 URDF。

主界面左侧新增 `UR5 real-time skeleton (base frame)`，显示 base 坐标轴、J1–J6 关节、tool0
骨架和红色 TCP 点，100 ms 刷新。右侧新增“打开 RViz 3D 模型”按钮，启动：

```text
rviz2 -d /home/ttbstt/jixiebi/UR5/ur5_hmi/config/ur5_hmi.rviz
```

RViz 配置使用 Fixed Frame=`base`，显示 `/robot_description`、TF 和 Grid；RViz 异常不影响
HMI 的 ROS 控制线程。启动方式：

```bash
source /opt/ros/humble/setup.bash
/home/ttbstt/jixiebi/UR5/ur5_hmi/run_hmi.sh
```

本轮已完成 Python 编译、URDF FK 解析和 PyQt5 离屏视图 smoke test；没有发送任何真机运动指令。

## 122. HMI 页面内嵌 UR5 实体网格模型（2026-09-30）

用户要求在 HMI 页面内直接显示 3D 模型。系统已确认有 VTK 9.1 和 Qt5 的
`QVTKRenderWindowInteractor`，因此新增：

```text
UR5/ur5_hmi/gui/vtk_robot_view.py
```

该视图：

- 解析 `real_robot_description_calibrated.urdf` 的 link mesh、joint origin 和 joint axis；
- 使用 `/opt/ros/humble/share/ur_description/meshes/ur5/collision/*.stl` 的 7 个 UR5 网格作为
  页面内实体显示模型，单位为米；
- 按 `/joint_states` 的连续关节角实时更新每个 mesh actor 的刚体变换；
- 用 `/tcp_pose_broadcaster/pose` 更新红色 TCP 球；
- 显示 base 坐标轴；
- 只读，不发布 topic、不调用 action、不参与控制、IK 或碰撞检测。

`main_window.py` 已优先使用 VTK 视图；若 VTK Qt 集成不可用，自动回退到原 Matplotlib 骨架视图。
同时支持从 `/robot_description` 重新解析当前 ROS2 URDF。当前页面内已验证加载 7 个网格：

```text
base_link_inertia, shoulder_link, upper_arm_link, forearm_link,
wrist_1_link, wrist_2_link, wrist_3_link
```

VTK 页面视图在 X11 环境中启动测试通过；离屏模式不适用于 QVTK，因此离屏 smoke test 仅验证
Python 编译，不能代表无显示环境可用。RViz 按钮和原控制安全门保持不变，未发送任何真机运动。

## 123. 修复 HMI 页面 3D 模型与布局显示（2026-09-30）

根据页面截图完成以下修复：

1. VTK 3D 模型此前在加载 actor 后立即重置相机，收到实际关节变换后容易只看到局部重叠模型。
   现在改为第一次应用真实 `/joint_states` 变换后再初始化固定相机位置和观察中心。
2. 修复 `/robot_description` 重新加载时 `RemoveActor` 传入 tuple 的问题，并清理旧 TCP actor，
   避免重复模型叠加。
3. 关节表限制为约 235–250 px 高度，日志区限制为 100–150 px，避免页面出现大面积空白。
4. 扩大关节表列宽，完整显示关节名称和数值。

验证结果：7 个 UR5 网格 actor 均能加载，实时姿态变换和相机初始化测试通过；本次没有发送
任何真机运动指令。

## 124. HMI 3D 视角优化（2026-09-30）

页面内嵌 VTK 视图新增标准视角按钮：

```text
等轴 / 俯视 / 正视 / 侧视 / 重置
```

视角操作只改变 VTK camera，不改变机器人状态和任何 ROS 控制接口。VTK 鼠标交互为：

```text
左键拖动       旋转
中键拖动       平移
右键拖动/滚轮  缩放
```

如果视角丢失，点击“重置”；实时关节更新不会覆盖用户手动调整的相机。视角方法已完成
Python 编译和 X11 窗口测试，未发送真机运动指令。

## 121. 修复 HMI 启动脚本的 ROS2 环境问题（2026-09-30）

用户执行 `run_hmi.sh` 时遇到：

```text
/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
```

原因是 HMI 启动脚本原先使用 `set -u`，而 ROS2 Humble 的 `setup.bash` 在某些环境下会读取尚未
定义的 `AMENT_TRACE_SETUP_FILES`。已将：

```bash
set -euo pipefail
```

改为：

```bash
set -eo pipefail
```

保留错误传播和管道错误检查，但不再对 ROS2 环境变量启用 `nounset`。`bash -n`、ROS2 环境
启动检查和 Python 编译均通过。未修改 Driver、控制器或机器人状态。

## 125. HMI UR5 网格配色（2026-09-30，已被第 126 节替代）

这一版曾在页面内嵌 VTK 网格中手工设置蓝白 link 颜色：

```text
base_link_inertia  深灰
shoulder_link     蓝色
upper_arm_link    浅灰白
forearm_link      浅灰白
wrist_1/2/3_link  蓝色
```

该手工配色不再作为当前实现使用；详见第 126 节。它没有修改 URDF、ROS2、碰撞检测或轨迹执行。

## 126. HMI 颜色改为读取原始 UR5 材质（2026-09-30）

用户指出第 125 节中的蓝/白配色是手工指定的，不应擅自添加。已移除
`ur5_hmi/gui/vtk_robot_view.py` 中的 `LINK_COLORS` 固定调色板。

当前 VTK 仍使用轻量的 UR5 collision STL 作为几何，但颜色不再由 HMI 自行猜测；
程序会读取同名原始 URDF visual COLLADA 文件中的 `<diffuse>` 材质颜色，并按三角面数量
加权得到该 link 的显示颜色。这样颜色来源于 `/opt/ros/humble/share/ur_description/`
原始 UR5 文件，而不是新添加的自定义颜色。它只影响 HMI 显示，不影响 ROS2、碰撞和控制。

已验证 `base/shoulder/upperarm/forearm/wrist1/wrist2/wrist3` 的 DAE 材质均可读取，
`python3 -m compileall -q ur5_hmi/gui` 通过。若要看到更新后的颜色，重启 HMI：

```bash
cd /home/ttbstt/jixiebi/UR5/ur5_hmi
./run_hmi.sh
```

## 138. HMI TCP frame 与右侧信息架构修复（2026-09-30）

本轮未发送任何真机运动指令，也未修改 ROS2 控制、安全门、FollowJointTrajectory、
joint mapping、continuous joint、RealHome 或轨迹校验逻辑。

- 通过只读检查确认 `/tcp_pose_broadcaster/pose` 当前 `header.frame_id = base`；VTK URDF viewer 的 link transforms 也统一到 URDF `base` frame，因此 TCP topic 与模型使用同一参考系。
- `RobotState` 现在保留 `tcp_frame_id`，可视化只接受 `base/base_link` frame；未知 frame 时隐藏红色 TCP marker，避免显示错误坐标。
- VTK fit 只使用 7 个 UR5 link mesh actor 的包围盒，排除 TCP marker、TCP 连线、文字和 XYZ 轴；新增 flange/tool0 到 TCP 的红色辅助连线，帮助识别真实工具偏置。
- 3D 关节标注简化为 `J1`–`J6`，减小重叠；不改变关节数据或运动学。
- 右侧安全控制保持最上方；STOP、刷新、RViz 和 UI Scale 在紧凑横向安全卡中排列，状态/TCP 卡片并排，减少右侧滚动。
- 关节点动区获得更多空间，表格使用四列且六行完整可见；默认 UI scale 改为 100%，125%/150% 仍可选。
- HMI Command Speed 从配置读取 `default_speed_percent` 和 `max_hmi_speed_percent`，当前默认 5%、上限 20%；机器人自身 `speed_scaling` 仍是只读状态，不受此 UI 限制影响。

验证：

```text
python3 -m compileall -q .       PASS
bash -n run_hmi.sh               PASS
ROS 只读 TCP frame 查询         base
VTK smoke                       7 个 mesh actor，TCP base-frame marker/line 正常
100%/125%/150%                  六行关节表与 STOP 控件可见
真实机器人运动                  NO
```

## 139. HMI 右侧紧凑化与 TCP 可视化收敛（2026-10-08）

- `/tcp_pose_broadcaster/pose` 只读确认 `frame_id=base`；模型 FK 和 marker 均使用 base frame。
- VTK marker 不再参与 fit 包围盒；只用 UR5 mesh 计算相机，并显示 tool0 到真实 TCP 的辅助连线。
- 右侧安全卡改为紧凑横向结构：STOP、警告、刷新、RViz、缩放集中在一张卡内；状态与 TCP 卡片并排显示。
- 状态行压缩为 30/32/34 px（100/125/150%），仍保持独立 chip 和垂直居中；125% 下右侧纵向滚动条 maximum=0。
- 关节点动区域保持四列和六行；125% 下表格滚动 maximum=0，J1–J6 全部可见。
- 关节标注简化为 J1–J6，避免中文长标签在腕部重叠。
- HMI Command Speed 使用 `default_speed_percent: 5`、`max_hmi_speed_percent: 20`；UR 的 Robot Speed Scaling 仍只读。
- 默认 UI scale 改为 100%，125%/150% 保留可选。

验证：

```text
python3 -m compileall -q .       PASS
X11 MainWindow smoke              PASS（7 mesh actor，base-frame TCP）
100%/125%/150%                   右侧 rightmax=0，关节 tablemax=0
真实机器人运动                    NO
```

## 140. 掉电状态不再显示缓存姿态（2026-10-08）

现场只读检查确认：机器人断电时仍可收到 `robot_mode=3`、`speed_scaling=0.0`，而 ROS2
进程、controller manager 和部分 latched 状态仍在线；因此“ROS2 Driver Connected”不等于机器人上电。

- `RobotState`/HMI 增加 live feedback 门控：必须有新鲜关节反馈且 `robot_mode` 为 5（IDLE）或 7（RUNNING）。
- mode 3 显示为 `3 (POWER_OFF)`；Robot Program 不再沿用掉电前的 Running，Joint State 显示 stale，Safety 显示 not available。
- 掉电时清空 TCP mm/deg 显示并隐藏实时 3D 姿态，避免把最后一帧缓存数据误认为当前机器人位置。
- 机器人重新上电并进入 IDLE/RUNNING 后，关节、TCP 和 3D 姿态自动恢复。
- ROS2 Driver 的 Connected 保留为通信进程状态，不代表动力电源或抱闸状态。

验证：当前现场 mode=3/speed=0.0 时，HMI 显示 `program=Stopped`、`joint=stale`、`tcp=—`；本轮未发送运动指令。

## 127. 恢复 HMI 初始 UR5 显示配色（2026-09-30）

用户反馈按 DAE 材质加权后的颜色显示效果不如最初版本。因此已撤销第 126 节的
材质解析逻辑，也撤销第 125 节的手工蓝白调色板。

当前 `ur5_hmi/gui/vtk_robot_view.py` 恢复为最初的统一中性网格颜色：

```python
actor.GetProperty().SetColor(0.75, 0.78, 0.82)
```

不再读取 DAE 材质、不再按连杆自行配色，也没有改变模型、关节状态、ROS2 或运动控制。
重启 HMI 后即可恢复最初显示效果。

## 128. 放大 HMI 关节控制页面（2026-09-30）

根据页面截图，已放大 C 区实时关节状态与单关节点动区域：

- 关节表改为可扩展布局，显示完整 6 行；
- 表格最小高度调整为 350 px，行高调整为 48 px；
- 数值列和 Jog 列加宽，字体和表头放大；
- Jog `+/-` 按钮调整为 48×38 px，字体加大；
- 步长、速度下拉框和 RealHome 按钮加高；
- 关节控制区域获得可用的垂直伸展空间。

修改文件：`UR5/ur5_hmi/gui/main_window.py`、`UR5/ur5_hmi/gui/widgets.py`。

## 132. HMI HiDPI 与大尺寸控制优化（2026-09-30）

在现有 splitter/tab 架构上做局部优化，没有修改 ROS2、FollowJointTrajectory、关节映射、
安全门或真实机器人运动逻辑：

- `main.py` 在 QApplication 创建前启用 Qt High-DPI scaling 和高 DPI 图标；
- 默认 `ui.scale: 1.25`，右侧可选择 100%/125%/150%，配置写入 `config/hmi_config.yaml`，
  重启后完整生效；
- Jog 区增加右侧状态卡片，显示 Motion/Action、反馈、目标、指令速度和 Robot Speed Scaling；
- 关节表不再无限垂直拉伸，保留 60 px 行高和角度优先显示；
- 速度滑块高度增大，保留 5/10/20% 快捷按钮；
- 状态 chip 最小高度 30 px，TCP 数值使用较大等宽字体；
- VTK 默认相机距离缩短，坐标轴缩小，TCP 增加文字标注，模型标签仍为只读显示。

验证：`python3 -m compileall -q .`、`bash -n run_hmi.sh` 通过；X11 VTK smoke test 加载 7 个
网格、6 个关节标签和 TCP 标注通过。本轮没有发送任何机器人运动指令。

## 133. 125% UI Scale 下的可操作性修复（2026-09-30）

本轮未修改 ROS2 Driver、FollowJointTrajectory、`can_move()`、joint mapping、RealHome、
trajectory validation 或任何机器人运动逻辑。

已完成：

- `set_status_label(..., scale)` 统一按 UI scale 设置状态 chip 高度、宽度、字体和内边距；
- 右上状态行增加动态垂直间距，125%/150% 下避免文字上下裁切；
- STOP MOTION 说明增加上边距，按钮和警告文字不重叠；
- 关节表启动时强制滚动到顶部，确认 J1–J6 均可见；
- Jog 按钮支持统一缩放，基础尺寸 80×48 px，带 hover/pressed/disabled 状态；
- 关节表使用角度为主、rad 为参考，行高和字体由 `apply_ui_scale(scale)` 同步调整；
- 步长按钮、速度滑块和快捷按钮统一水平对齐并随 scale 调整；
- Jog 状态卡增加 `Start Check` 和 `Control Source: PC / ROS2`；
- 保留实时 TCP 红色 marker，并增加 TCP 文字标识；
- UI 缩放 100%/125%/150% 会同步更新字体、行高、按钮、Tab、间距和状态 chip，配置写入
  `config/hmi_config.yaml`。

验证结果：

```text
100%  row=48 px, Jog=80×48 px, scroll=0
125%  row=60 px, Jog=100×62 px, scroll=0
150%  row=72 px, Jog=120×72 px, scroll=0
```

`python3 -m compileall -q .`、`bash -n run_hmi.sh`、X11 smoke test 和 offscreen layout
smoke test 均通过。本轮真实机器人运动：NO。

## 131. HMI 工业化布局重构（2026-09-30）

按用户提供的布局建议，完成界面外观和信息层级调整，未修改 ROS2 控制链路或安全门：

- 主窗口最小尺寸调整为 `1440×850`，启动时最大化；
- 上半区改为水平 `QSplitter`，左侧大尺寸 UR5 3D 视图，右侧机器人状态、TCP 和安全控制；
- 3D 视角按钮移到模型下方，按钮高度增加；
- 下半区改为 `QTabWidget`：关节点动、轨迹执行、RealHome、运行日志；
- 关节点动改为角度优先显示（deg），rad 作为参考列；
- 步长改为大尺寸单选按钮，速度改为 `1–100%` 滑块并保留 5/10/20% 快捷按钮；
- Jog 行、STOP、轨迹按钮和 TCP 数值字体放大；
- RealHome 单独显示保存结果，日志页使用等宽字体。

修改文件：`UR5/ur5_hmi/gui/main_window.py`、`UR5/ur5_hmi/gui/widgets.py`。
启动方式不变：

```bash
source /opt/ros/humble/setup.bash
cd /home/ttbstt/jixiebi/UR5/ur5_hmi
./run_hmi.sh
```

## 136. HMI 四区域布局统一（2026-09-30）

本轮继续只修改 UI 展示和尺寸约束，未修改 ROS2、FollowJointTrajectory、`can_move()`、
关节映射、连续关节处理、RealHome、安全门、轨迹校验或真机动作。

完成内容：

- 新增 `UR5/ur5_hmi/gui/ui_metrics.py`，统一管理 scale、状态行、TCP 行、关节行、按钮和间距尺寸。
- 机器人状态区改用 `StatusRow`：名称、伸缩空白和独立状态 chip 分离；chip 最小宽度 170 px，行高按 100%/125%/150% 同步放大，保持居中。
- TCP 状态拆成 `TCP POSITION` 与 `TCP ORIENTATION` 两张卡片。主显示为 mm/deg；m/rad 通过 tooltip 提供，不再把两套单位挤在同一文本行。
- 安全区加入水平分隔线，STOP、警告、刷新、RViz、缩放按纵向顺序排列；警告支持换行、独立高度和上下边距。
- 关节点动表从 5 列减少为 4 列：`关节 / 当前角度 / 连续值 / Jog`；rad 仅放在角度单元格 tooltip。Jog 两个按钮保持独立可读尺寸，J1–J6 初始化滚动到顶部。
- Jog 状态区复用 `StatusRow`，保留 Motion/Action、Current Joint、Target、Command Speed、Robot Scaling、Start Check、Control Source 等原有只读信息。
- 增加 `UI_DEBUG_LAYOUT = False`，需要排查 Qt 压缩时可临时打开彩色边框，默认关闭。

验证：

```text
python3 -m compileall -q .       PASS
bash -n run_hmi.sh               PASS
offscreen MainWindow smoke       PASS
X11 VTK MainWindow smoke         PASS（7 个 UR5 mesh actor）
100% / 125% / 150%              状态行、TCP 卡片、STOP 警告和 J1–J6 均有独立尺寸
真实机器人运动                   NO
```

## 137. HMI 上下空间与安全控制可见性修复（2026-09-30）

针对截图中 J1–J6 只显示到 J4、右侧 STOP MOTION 需要向下滚动的问题，未修改任何真实控制逻辑。

- 顶部 splitter 最小高度由 400 调整为 360，3D 视图最低高度由 480 调整为 360。
- 主窗口上/下区域 stretch 调整为 `4:6`，为关节点动表提供更多垂直空间；在当前 1600×1000 Qt 窗口和 125% scale 下，表格 viewport 可完整容纳 6 行。
- 右侧安全控制卡移动到右侧滚动面板的第一个位置，STOP MOTION、警告、刷新、RViz 和缩放在打开面板时优先可见。
- 右侧其余状态/TCP 内容仍保留滚动，不再通过压缩布局造成覆盖。

验证：

```text
100% / 125% / 150% layout smoke       PASS
125% table viewport                   483 px，6 行可见，scroll maximum=0
STOP MOTION                           位于右侧面板顶部且可见
真实机器人运动                        NO
```

## 130. 关节中文名称与 3D 对应标注（2026-09-30）

关节控制表第一列已改为中文：

```text
J1 肩部旋转
J2 肩部俯仰
J3 肘部
J4 腕部1
J5 腕部2
J6 腕部3
```

VTK 3D 模型中新增随关节运动更新的 `J1–J6` 中文标注，标注对应 URDF 的实际关节坐标。
它们只用于显示，不参与控制、IK 或轨迹计算。已完成 Python 编译和 X11 窗口 smoke test，
确认加载 7 个网格和 6 个关节标签。
仅调整界面布局和可读性，没有修改 Jog 逻辑、安全门或 ROS2 控制接口。

## 129. 放大关节控制区文字（2026-09-30）

用户反馈关节页面文字仍然过小，已进一步调整：

- C 区标题、标签、下拉框和 RealHome 按钮字体放大；
- 关节表内容和表头统一放大到 18 px；
- 表格行高调整为 60 px；
- Jog 按钮调整为 56×46 px，字体调整为 22 px。

修改文件：`UR5/ur5_hmi/gui/main_window.py`、`UR5/ur5_hmi/gui/widgets.py`。

## 134. HMI 125% 高 DPI 裁切修复（2026-09-30）

本轮只修复布局和只读显示，没有修改 ROS2 Driver、FollowJointTrajectory、
`can_move()`、joint mapping、RealHome、trajectory validation 或任何真机运动逻辑。

修改内容：

- 右侧机器人状态改为 7 个独立的 HBox 行；状态名称、chip 高度和间距均由
  `apply_ui_scale(scale)` 管理，125%/150% 时不会上下挤压或裁切。
- TCP 六行使用独立的标签和值布局，并随 scale 调整行高；右侧整体放入滚动容器，窗口高度不足时不会覆盖其它内容。
- STOP MOTION 警告改为 `⚠ 软件停止不能替代示教器急停`，固定最小高度并保留上边距，禁止与按钮重叠。
- 关节表启动和缩放后都强制滚动到顶部，确保 J1–J6 可访问；Jog 子按钮基础尺寸为 72×46 px，并随 scale 放大。
- VTK 新增 `fit_robot_view()`：只使用 UR5 网格包围盒计算相机，排除坐标轴、文字和 TCP 标记；默认保留约 12%–15% 留边，模型目标占视窗高度约 65%–72%。等轴/俯视/正视/侧视/自动适配共用该逻辑。
- 保留原始统一中性 UR5 网格颜色和实时 TCP 红色 marker/标签，不改变数据来源或控制接口。

验证：

```text
python3 -m compileall -q .       PASS
bash -n run_hmi.sh               PASS
QT_QPA_PLATFORM=offscreen layout smoke: PASS
100% / 125% / 150%: J1–J6 scroll=0，状态行和 STOP 警告均有独立高度
真实机器人运动：NO
```

启动：

```bash
source /opt/ros/humble/setup.bash
cd /home/ttbstt/jixiebi/UR5/ur5_hmi
./run_hmi.sh
```


## 139. CR7-7 独立 MATLAB / CoppeliaSim 仿真环境（2026-10-04）

依据本文搭建 `/home/ttbstt/jixiebi/cr7` 独立工程，当前场景 `cr7/simulation/cr7.ttt`。
以已有 `CR7 2.ttt` 的 xMateCR7 六轴模型替换 UR5，迁移 UR5 的工件、焊枪、四组件 GunCollisionHull 和冻结 WeldTCP，CR7 base 采用原 UR5 base world pose。
原 UR5 场景、UR5 MATLAB 文件、ROS2 控制包及四个原 CR7 场景均未覆盖；未修改全局 functional.lua。

- 独立 topic `/cr7/joint_trajectory`、`/cr7/joint_states`，ZMQ 23010；外部 rclpy 驱动，Lua simROS2 仅发布。
- CR7 六轴显式路径 `/CR7/joint1`–`joint6`，MATLAB 由场景零位变换建立运动学，qOffset=0，不沿用 UR5 标定。
- Mount 直接接 joint6，采用单位安装变换；真实 CR7 装枪尚需物理标定。
- WeldTCP 局部位置保留 `[-0.06677482221230063,0.019489204955399853,0.38233021973475156] m`、单位四元数，不再加焊丝长度。
- 新场景为 kinematic 规划回放，网格 static、respondable=false；不代表接触动力学验证。
- MATLAB 工件沿用 4 块 5 mm 板、焊枪 4 凸组件；18 mm 焊丝与 1.2 mm 临时默认直径沿用原 MATLAB 几何定义。

验证：4 组 FK，最大位置差 4.34e-16 m、旋转矩阵差 1.08e-15；MATLAB 完整闭环 1498 条反馈，J1 2° 往返、结束误差 0°；含焊丝初始自碰撞/环境碰撞 `[0 0]`；小幅 RRT-Connect 插值路径 15×6；80 点焊缝生成和 MATLAB 实体预览通过。
完整 CR7 连续焊缝 IK、轨迹规划和焊缝执行尚未验收，不能使用 UR5 关节解。

详细文件、启动方法、关键数值和回滚见 `cr7/README.md`、`cr7/AGENT_HANDOFF.md`。回滚副本 `cr7/variants/cr7_original/` 保存 CR7 原夹具闭环版本；原自动焊接脚本已保存为文本并在新场景停用。

## 141. HMI 显式显示机器人电源状态（2026-10-08）

在 `/home/ttbstt/jixiebi/UR5/ur5_hmi/gui/main_window.py` 的现有“安全控制”卡片中新增独立电源状态芯片，复用 ROS2 `robot_mode` 反馈，不修改任何 ROS2 控制、安全门、Jog、轨迹执行或真机运动逻辑：

- `robot_mode == 4/5/6/7`（POWER_ON/IDLE/BACKDRIVE/RUNNING）：显示 `电源：已上电`，浅绿色；
- `robot_mode == 3`：显示 `电源：已断电`，浅红色；
- 其它/未知模式：显示 `电源：未知`，浅黄色。

该标识与 `ROS2 Driver: Connected` 分开：Connected 仅代表 PC 端 ROS2 进程/通信在线，不代表机器人已经上电。掉电时原有 stale 反馈保护仍然生效，关节、TCP 和 3D 实时姿态不会继续冒充实时数据。

验证：

```text
python3 -m compileall -q .                         PASS
source /opt/ros/humble/setup.bash                  PASS
QT_QPA_PLATFORM=offscreen MainWindow smoke test   PASS
mode 3 -> 电源：已断电
mode 7 -> 电源：已上电
真实机器人运动                                      NO
```

启动：

```bash
source /opt/ros/humble/setup.bash
cd /home/ttbstt/jixiebi/UR5/ur5_hmi
./run_hmi.sh
```

## 142. HMI 掉电时保持完整 UR5 参考显示（2026-10-08）

修复 `UR5/ur5_hmi/gui/vtk_robot_view.py` 和 `gui/robot_view.py` 的掉电显示问题。
此前主窗口在 `robot_mode=POWER_OFF` 时传入空关节状态，VTK 没有应用任何关节变换，导致各连杆停在局部原点并重叠，看起来只剩一个圆柱。

现在的行为：

- 首次没有实时关节反馈时显示完整零位参考构型；
- 运行中断反馈时冻结最后一个已知构型；
- HMI 状态区仍显示 `Joint State stale`、TCP 为 `—`，运动按钮仍受安全门禁用；
- TCP 实时红点/连线在反馈无效时继续隐藏，避免把冻结模型或旧 TCP 冒充实时数据；
- 没有修改 ROS2 控制、Jog、FollowJointTrajectory 或安全逻辑。

说明：CoppeliaSim 的 `.ttt` 场景不能直接作为 Qt/VTK 模型嵌入。当前 HMI 使用同一 URDF 的 UR5 网格和运动学；若要把 CoppeliaSim 中的焊枪、焊丝、工件也搬到 HMI，需要先从场景导出视觉网格（STL/OBJ/JSON）并增加只读渲染层，不能直接加载 `.ttt` 文件。

验证：

```text
python3 -m compileall -q .                 PASS
offscreen MainWindow / 3D fallback smoke    PASS
电源掉电时不发送运动                         PASS
```

## 143. HMI 关节表保证 J1–J6 同时可见（2026-10-08）

用户反馈窗口底部只能看到 J1–J5，J6 被裁切。原因是 `joint_table` 的最大高度被设置为 620 px，表格在有剩余空间时继续扩张，挤压并裁切底部内容。

修复：

- `UiMetrics.joint_row` 调整为 100%/125%/150% 约 56/58/60 px；
- 表格最大高度改为“表头 + 六行 + 内边距”的精确内容高度；
- 水平/垂直滚动条关闭，避免滚动条占用最后一行的可视高度；
- 保留启动和缩放后的滚动条回到 0；
- 未修改 Jog 命令、ROS2 控制、安全门或真机运动。

验证：

```text
六行内容高度计算通过
vertical scrollbar maximum = 0
J6 行位于 viewport 内
python3 -m compileall -q . = PASS
```

## 144. HMI MATLAB 轨迹导入、预演与最终执行门（2026-10-08）

本轮继续在 `/home/ttbstt/jixiebi/UR5/ur5_hmi` 开发，没有修改 CoppeliaSim、MATLAB 规划结果、ROS2 driver、External Control 或 FollowJointTrajectory 控制链路，也没有发送真机轨迹。

新增/修改：

- `trajectory/trajectory_utils.py`：新增 `TrajectoryData` 标准结构，严格使用六个 ROS/URDF 关节名；保留连续关节值，不执行 `wrapToPi`；新增 timed trajectory 到 `JointTrajectory` 的转换。
- `trajectory/trajectory_loader.py`：MAT/YAML/JSON 读取 `qPath/q_path/qTraj/positions` 和 `timeFromStart/time_from_start/time/t`；没有时间不猜执行时长。
- `trajectory/trajectory_validator.py`：检查 N×6、NaN/Inf、关节顺序、软件限位、时间严格递增、相邻跳变、估计速度/加速度和实时起点误差。
- `gui/main_window.py`：轨迹页显示轨迹名称、来源、点数、时长、起终点、最大关节行程、最大跳变、估计速度/加速度、六关节起点误差；加入 3D 预演、暂停/停止、0.5x/1x/2x 和点位 slider。
- `gui/main_window.py`：加入 `FINAL EXECUTION CHECK`。默认起点容差为 1°；必须通过 ROS/程序/mode/safety/speed/controller/joint freshness/validation/start/action gate；执行前弹出四项人工确认。
- `gui/main_window.py`：执行期间禁用 Jog、RealHome、加载/验证新轨迹，仅保留 STOP MOTION；复用现有 scaled controller action。
- `gui/main_window.py`：执行期间按时间插值记录 q_command、q_actual、joint_error、TCP、speed scaling 和 safety mode；结束后保存 CSV + JSON 到 `logs/trajectory_runs/`。
- `gui/vtk_robot_view.py`、`gui/robot_view.py`：增加只读 preview pose，预演不显示为实时 TCP。
- `tools/create_test_trajectory.py`：生成 5 点、10 秒、最大 0.5° 偏移的安全软件测试 MAT，不连接 ROS2。
- `config/hmi_config.yaml`：新增 `trajectory.start_tolerance_deg: 1.0`。

测试：

```text
python3 -m compileall -q .                         PASS
MAT qPath + timeFromStart load/validate            PASS
MAT 无时间 -> TIME PARAMETERIZATION REQUIRED      PASS
3D preview slider / 5-point test                   PASS
真实机器人运动                                      NO
完整旧焊缝轨迹                                      未加载、未执行
```

软件测试命令：

```bash
source /opt/ros/humble/setup.bash
cd /home/ttbstt/jixiebi/UR5/ur5_hmi
python3 tools/create_test_trajectory.py --output /tmp/hmi_test_trajectory_5pt.mat
./run_hmi.sh
```

将该 MAT 加载后先点击“验证轨迹”和“3D轨迹预演”。默认零位首点通常与真机不一致，
因此 Final Gate 会阻止执行；只有用实时当前六关节角生成首点，并完成人工确认，才会出现可执行状态。
HMI 仍不做碰撞检测，轨迹验证 PASS 不等于碰撞验证 PASS。

## 145. UR5 真机重启后连接修复（2026-10-08）

诊断发现：电脑端 `eno1` 有时在真机重启/网线重新协商时处于 `NO-CARRIER`，但旧
ROS2 Driver 仍会继续发布 ros2_control 的缓存/周期状态，导致 HMI 错误显示
`ROS2 Driver Connected`，而机器人实际已经不可达。另一个问题是未显式指定
`reverse_ip` 时可能选到 Wi-Fi 地址。

已新增：

```text
/home/ttbstt/jixiebi/UR5/start_ur5_real.sh
```

`ur5_hmi/run_hmi.sh` 已整合该启动流程，现在推荐只运行：

```bash
cd /home/ttbstt/jixiebi/UR5/ur5_hmi
./run_hmi.sh
```

如果检测到已有 `ur_ros2_control_node` 或 50002 监听，HMI 会复用现有 Driver；
否则等待网络并启动一个独立 Driver 进程组，关闭 HMI 时只清理本次由 HMI 启动的 Driver。

脚本行为：

- 激活 NetworkManager 连接 `ur5-static`；
- 等待 `eno1=192.168.0.20/24` 和 `192.168.0.10` ping 可达；
- 显式设置 `reverse_ip=192.168.0.20`、`reverse_port=50001`、`script_sender_port=50002`；
- 使用 `/home/ttbstt/jixiebi/UR5_real/ur5_calibration.yaml`；
- 启用 Dashboard client，但保持 `activate_joint_controller:=false`；
- 等待失败时退出，不留下看似在线的缓存 Driver。

HMI 状态模型新增对真机 29999/30001 的只读 TCP 探测。断网时即使
`/joint_states` 仍在发布，也显示 Driver disconnected、Joint State stale、
`电源：未知（通信中断）`，并禁止运动门。配置新增：

```yaml
ros:
  robot_ip: 192.168.0.10
```

安全模式显示使用完整 `SafetyMode` 枚举；例如 `safety_mode=9` 显示为
`FAULT`，不再显示含义不清的 `not available`。

## 147. MATLAB RealHome → PreWeld 轨迹导出与 HMI 碰撞门（2026-10-08）

本轮只做离线轨迹封装和 HMI 门禁增强，未重新规划 80 点焊缝，未修改 ROS2/CoppeliaSim/TCP
标定，也没有发送真机运动。

### 已确认的真实 RRT 数据

读取：

```text
/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_approach_exit.mat
```

`result.approach.qPath` 是现有 `qHome → qPreWeld` 的 21×6 路径，且：

```text
result.approach.success = 1
result.approach.pass = 1
result.approach.validation.pass = 1
nodeCount = 21
edgeCount = 20
nodeCollisionFree = 1
edgeCollisionFree = 1
```

RRT 文件中的 q 值在 MATLAB/ROS 真实关节空间（由 `scene.qMat0` 规划），不是 CoppeliaSim
q 空间。CoppeliaSim 若需要转换，仍使用 `qReal = qCop + qOffset`，其中
`qOffset = [180 -90 0 -90 0 0] deg`；本次导出没有使用该转换。

### 新增 MATLAB 文件

```text
matlab仿真/parameterize_joint_path.m
matlab仿真/lift_continuous_joint_path.m
matlab仿真/export_hmi_trajectory.m
matlab仿真/export_realhome_to_preweld_hmi.m
```

- `parameterize_joint_path`：按显式速度/加速度上限生成严格递增时间，不改变 qPath。
- `lift_continuous_joint_path`：针对 RRT 文件中跨 ±2π 边界的等价数值表示，只加减整圈恢复连续
  分支；不执行 `wrapToPi`，不覆盖输入。
- `export_hmi_trajectory`：要求明确 `CoordinateSpace`，保存标准 `qPath/joint_names/timeFromStart`
  和 metadata；正式轨迹保存碰撞验证字段。
- `export_realhome_to_preweld_hmi`：读取现有 `result.approach.qPath` 并调用以上工具，不重跑 RRT。

已生成：

```text
轨迹文件/realhome_to_preweld.mat
轨迹文件/realhome_to_preweld_report.json
```

离线验收：21 点、47.588 s，最大相邻步 12°；原始数值表示最大跳变 369.938°，连续分支提升后
最大跳变 12°。HMI loader/validator 读取成功，格式/时间/离线起点均 PASS；原始 RRT MAT 未修改。

### HMI 修改

`ur5_hmi/gui/main_window.py` 轨迹摘要增加 `Coordinate Space` 和 `Collision Check`。来源为
MATLAB/RRT 或带 `collision_verified`/`collision_check_pass` 元数据的正式轨迹，Final Execution
Check 必须同时满足：

```text
collision_verified == true
collision_check_pass == true
```

缺少或失败时只能加载/预演，禁止执行。来源含 `test`/`commissioning` 的调试轨迹不冒充碰撞 PASS，
但仍不能绕过实时反馈、起点误差、External Control、控制器和人工二次确认门。

验证：

```text
python3 -m compileall -q /home/ttbstt/jixiebi/UR5/ur5_hmi   PASS
source /opt/ros/humble/setup.bash
HMI trajectory loader + validator on realhome_to_preweld.mat   PASS
真实机器人运动   NO
```

使用 HMI 时先点击“加载轨迹”选择上述 MAT，再点击“验证轨迹”和“3D轨迹预演”。本轮禁止点击
“低速执行（5%）”，也不把离线 PASS 解释为已完成真机执行验收。

## 148. 最新 V1 RealHome → PreWeld 导出（2026-10-08）

此前生成的 `轨迹文件/realhome_to_preweld.mat` 来自旧的
`plan_weld_approach_exit_rrt_connect.m` 本地双树 RRT-Connect，不是最新 V1。

已核对最新 V1 文件：

```text
/home/ttbstt/jixiebi/RRT/results/v1_fast_connect_anytime_taskA_seed20260917.mat
```

其中 `report.paths{3}` / `report.rows(3)` 对应：

```text
algorithm = Constraint-aware Bidirectional RRT* (CBi-RRT*)
task = T01_Safe_to_PreWeld
qHome(qSafe) -> qPreWeld
path size = 10×6
first solution ≈ 0.766 s
final best path length = 5.173362 rad
final dense validation = PASS
```

新增导出器：

```text
matlab仿真/export_v1_realhome_to_preweld_hmi.m
```

已生成独立文件（未覆盖旧 RRT-Connect 文件）：

```text
轨迹文件/realhome_to_preweld_v1.mat
轨迹文件/realhome_to_preweld_v1_report.json
```

V1 导出同样保持 `ros_real` 连续关节分支，处理跨 ±2π 的等价数值表示但不使用
`wrapToPi`。离线 HMI loader/validator 结果：10 点、40.516 s、最大导出相邻步约 39.977°、
结构/时间/碰撞元数据 PASS。这个 39.977° 是稀疏 V1 路径点之间的表示，不是 dense edge 验证的
分辨率；正式执行前仍应在 MATLAB/HMI 中确认时间参数与真实起点，并可另行导出插值后的标准轨迹。

使用 V1 版本预演：

```text
HMI → 加载轨迹 → 轨迹文件/realhome_to_preweld_v1.mat
→ 验证轨迹 → 3D轨迹预演
```

本轮没有执行 V1 轨迹，没有启动真机运动，也没有替换稳定的旧导出文件。

验证：`python3 -m compileall -q ur5_hmi`、`bash -n start_ur5_real.sh`、
`bash -n ur5_hmi/run_hmi.sh` 均通过。当前没有发送真机运动指令。

## 149. HMI 在线规划 Tab 与统一 MATLAB 多算法接口（2026-10-08）

在不改 ROS2 控制、安全门、FollowJointTrajectory 和真机执行逻辑的前提下，HMI 增加“在线规划”
Tab。起点可选最新有效 `/joint_states` snapshot 或 `ur5_hmi/config/hmi_config.yaml` 中的
`real_home`；目标 `PreWeld` 来自 `轨迹文件/realhome_to_preweld_v1_report.json` 的 `task.goal`，
不在 Python 中硬编码。

算法下拉框为 `RRT`、`RRT-Connect`、`Bi-RRT*` 和默认的
`Constraint-aware Bidirectional RRT* (CBi-RRT*)`。统一 MATLAB 入口：

```text
/home/ttbstt/jixiebi/RRT/benchmark/plan_hmi_request.m
```

其直接调用形式为 `plan_hmi_request(qStart,qGoal,algorithm,seed)`；HMI 的 QProcess 传入同字段
的 request struct，并额外给出输出目录。

映射为 `RRTJoint`、`RRTConnectJoint`、`BiRRTStarJoint` 和
`FastConnectAnytimeRRTStarJoint`。新增的 `RRT/jointspace/RRTJoint.m` 是 MATLAB 六维单树
实现，不是 Python 重写。四种算法共享场景、关节限制、碰撞接口，并统一做 dense 节点/边碰撞
验证和 `parameterize_joint_path` 时间参数化；不使用 `wrapToPi`，保留连续关节分支。

HMI 通过 QProcess 异步调用 MATLAB `-batch`，`planning_busy` 与 motion busy 分开。每次请求保存：

```text
/home/ttbstt/jixiebi/UR5/planning_runs/<request_id>/request.json
/home/ttbstt/jixiebi/UR5/planning_runs/<request_id>/trajectory.mat
/home/ttbstt/jixiebi/UR5/planning_runs/<request_id>/report.json
```

MAT 含 `qPath`、`timeFromStart`、标准六关节 `joint_names` 和 metadata（algorithm、seed、碰撞
标志、request_id）；JSON 含 algorithm、seed、success、first_solution_time、planning_time、
path_length_rad、raw/execution point count、collision_check_pass、duration。成功后自动加载轨迹
页，可验证和3D预演，但不会自动发送真机轨迹；Collision Check 非 PASS 时 Final Execution Check
保持 BLOCKED。

同一 Saved RealHome → PreWeld、seed `20260917`、30 s 上限的离线对比：

| Algorithm | Success | First solution | Planning | Path length (rad) | Points | Collision |
|---|---:|---:|---:|---:|---:|---:|
| RRT | 0 | — | 30.075 s | — | — | FAIL/not generated |
| RRT-Connect | 1 | 0.973 s | 1.034 s | 5.035870 | 26 | PASS |
| Bi-RRT* | 1 | 6.760 s | 6.807 s | 4.362400 | 8 | PASS |
| Constraint-aware Bidirectional RRT* (CBi-RRT*) | 1 | 0.928 s | 30.059 s | 4.364048 | 11 | PASS |

结果目录：`/home/ttbstt/jixiebi/UR5/planning_runs/comparison_saved_realhome_*`。RRT 失败是相同场景/种子/30 s
预算下的真实结果；HMI 会显示失败并不会加载或执行。验证通过：
`python3 -m compileall -q /home/ttbstt/jixiebi/UR5/ur5_hmi`。本轮没有发送 ROS2 或真机运动指令。

### 在线规划结果归档（2026-10-08）

HMI 在线规划每次仍先写入审计目录：

```text
/home/ttbstt/jixiebi/UR5/planning_runs/<request_id>/
```

规划结果随后同步复制到用户指定的集中目录：

```text
/home/ttbstt/jixiebi/UR5/规划文件/
  online_<algorithm>_<request_id>.mat
  online_<algorithm>_<request_id>_report.json
  online_<algorithm>_<request_id>_request.json
```

MAT 文件仍由同一套 MATLAB planner 生成，归档只是复制，不改变轨迹数据，也不绕过
trajectory validation、start check、collision check 或 Final Execution Check。规划完成后仍只
自动加载/预演，不自动发送真机运动。目录由 `ur5_hmi/config/hmi_config.yaml` 的
`online_planning.planning_file_dir` 配置，默认值为上述 `规划文件` 路径。

## 150. HMI 焊缝任务类型 + 工件 3D 显示：调研结论与阻塞点（2026-10-08，未改任何代码）

本节记录一次**只读调研**的结果。本轮没有修改任何 MATLAB / Python 文件，没有运行真机，
没有发送 ROS2 或 FollowJointTrajectory。目的是接续 Codex 会话
`01a085eb-f065-7741-80ee-6075429183b3` 在 2026-10-08 14:59 被中断的第 224 条要求：

> 继续完善 UR5 HMI，重点增加“焊缝规划”和工件3D显示。
> 1. 在线规划增加任务类型：RealHome → PreWeld / Weld Seam / PostWeld → RealHome / Full Welding Task
> 2. 自由空间段默认使用 Constraint-aware Bidirectional RRT* (CBi-RRT*)，保留现有算法下拉框用于实验对比
> 3. Weld Seam 不要使用旧的逐点独立IK方法，调用当前 MATLAB 工程中「最新焊缝规划算法」

### 150.1 中断时的真实进度

- **MATLAB 侧已写完但从未运行过。** `RRT/benchmark/plan_hmi_weld_request.m`（14044 字节，
  309 行，mtime 2026-10-08 14:59，是全仓库最后被修改的文件）已实现全部 4 种任务，焊缝段调用
  `UR5/matlab仿真/replan_weld_actual_axis_adaptive.m`。符合要求 3 的字面意图。
- `UR5/规划文件/weld_anchor_points.json`（14:58）保存 `qPreWeld/qPostWeld/qWeldStart/qWeldEnd/source`。
- **HMI 的 Python 侧一行都没写。** `gui/main_window.py` 的在线规划页目标写死 `PreWeld`
  （`main_window.py:857`），没有任务类型概念，只调用 `plan_hmi_request.m`。
- `planning_runs/` 与 `规划文件/` 里全部是自由空间多算法对比记录，**没有任何焊缝规划运行记录**。
  也就是说 `plan_hmi_weld_request.m` 的成功路径从未被执行验证过。

### 150.2 plan_hmi_weld_request.m 的缺陷清单（已审计，均未修复）

该文件从未执行，审计确认它在**每种任务上都会崩溃**，且崩溃点被前一个崩溃掩盖。

| 编号 | 位置 | 缺陷 | 影响 |
|---|---|---|---|
| D1 | `plan_hmi_weld_request.m:103` | `metadata = exportReport.metadata;` | `export_hmi_trajectory.m` 返回的 report（`:129-147`、`:164`）**没有** `metadata` 字段——`metadata` 只是被 `save` 进 `.mat` 的局部变量。抛 `Unrecognized field name 'metadata'`。**所有任务必崩。** 正确写法见 `plan_hmi_request.m:104-105`：`saved=load(options.outputFile,'metadata'); metadata=saved.metadata;` |
| D2 | `plan_hmi_weld_request.m:148-151` | 用绝对路径传 `outputFile`/`baseOutputFile` | 规划器内部 `if ~isfile(f), f=fullfile(here,f); end`（`replan_weld_actual_axis_continuation.m:258`、`replan_weld_actual_axis_adaptive.m:328`）会把绝对路径再次拼到 `matlab仿真/` 后面 → 父目录不存在 → `MATLAB:save:noParentDir`。**约 20 s 规划跑完后才崩。** 防护写法见 `benchmark_adaptive_arc_ik.m:79` 的 `~startsWith(string(p),filesep)` |
| D3 | `plan_hmi_weld_request.m:73` | 读 `weldReport.collision_check_pass` | `localWeldStage` 的第 4 个返回值是 `:169-170` 构造的 `stageReports`，字段只有 `approach/weld_seam/exit`。**Full Welding Task 必崩。** 应改用 `weldReport.weld_seam` |
| D4 | `plan_hmi_weld_request.m:274` | `localGet(anchors,'qHome',zeros(1,6))` | 锚点文件没有 `qHome` → 静默返回全零，`:31` 把 Home 当成 `[0 0 0 0 0 0]`。不报错，但结果是错的 |
| D0 | `plan_hmi_weld_request.m:75-79`、`:88-92` | `qPath` 与 `tcpPath` 两套去重规则 | `localConcatPaths` 只在拼接边界去重，而 `tcpPath` 之后又跑 `localUniqueRows(tcpPath)`（按 1e-10 距离去重）。行数几乎必然不等 → HMI 侧 `trajectory/trajectory_utils.py:110` 抛 `ValueError: tcpPath must be N x 3` |
| D5 | `plan_hmi_weld_request.m:83-87` | 校验失败只设 `errorMessage` | 不 `error()`、不跳过导出，仍照常 `parameterize_joint_path` + `export_hmi_trajectory`。**校验失败的无效轨迹会写进 HMI 的 `.mat`** |
| D6 | `plan_hmi_weld_request.m` 自由空间段 | 无 try/catch | `plan_hmi_request` 在无路径（`plan_hmi_request.m:70`）和 dense 校验失败（`:90`）时都 `error()`。任何自由空间失败会中止整个任务，wrapper 自己的 `report.json` 永远写不出 |
| D7 | `localWeldStage` | 不透传时间预算 | `maxPlanningTimeSec` 只对自由空间段生效；`replan_weld_actual_axis_adaptive` 没有该选项。这正是需要补的“时间预算” |

次要已知不一致（记录，不阻塞）：`localDenseValidate` 用 `wrapJoints=false`
（`plan_hmi_weld_request.m:256-257`），而 `plan_hmi_request.m:201` 用 `true`。

### 150.3 阻塞点：焊缝段与锚点不在同一 IK 分支（关键，未解决）

**这是本轮最重要的发现，也是 HMI 焊缝任务的真正阻塞点。**

`plan_hmi_weld_request.m` 把焊缝段拼成 `qPath = [qPre; qSeam; qPost]`（`:75-79`、`:88-92`）。
这要求 `qSeam(1,:)` 与 `qPreWeld` 在同一条 IK 分支上。**实测不是。**

| 量 | j1 | j2 | j3 | j4 | j5 | j6 |
|---|---:|---:|---:|---:|---:|---:|
| `qPreWeld`（锚点，2026-09-17 已验证分支） | −0.9° | −97.4° | −90.7° | −80.7° | 103.2° | 137.5° |
| `_adaptive` 新规划 `qPath(1,:)` | 157.4° | −79.5° | 88.9° | −95.5° | −77.3° | 114.8° |

展开的原始值：

```text
qPreWeld              = [-0.014894647819507709, 4.5836787985654084, -1.5827839940260957, 4.8750956074309979, -4.4811791362770172, -3.8834036633603373]
qPostWeld             = [ 0.10103311567507861,  4.5548166837048392, -1.4847377912913284, 4.802397855108552,  -5.0336332283726337, -4.3317029796689042]
qWeldStart（锚点内）  = [-0.014329134332356164, 4.5264531427900421, -1.5705598632654856, 4.9202301990107706, -4.4811916127687788, -3.8828228197622088]
qWeldEnd（锚点内）    = [ 0.097304845667246551, 4.4994900201289347, -1.4729445207528835, 4.8471715179409767, -5.0335619702468941, -4.3356314885783931]
_adaptive qPath(1,:)  = [ 2.741396, -1.387900,  1.552137, 4.616149, -1.348566,  2.004334]
_adaptive qPath(end,:)= [ 2.734767, -1.311210,  1.407242, 4.554738, -1.694091,  1.630353]
```

这不是 2π 环绕差异，是**真实的 IK 分支差**（肩/肘翻转）。因此拼接处有约 158° 的基座摆动。
旧代码会静默拼出无效轨迹；加上 D5 修复后会直接硬失败。**两种结果都意味着 wrapper 无法按现状运行。**

#### 已排除的原因：场景没有变

| 来源 | seam startMat | seam endMat |
|---|---|---|
| `weld_rrt_connect_approach_exit.mat`（2026-09-17 15:00:09） | `[-0.605000, -0.045000, 0.005000]` | `[-0.605000, 0.130000, 0.005000]` |
| `weld_actual_axis_replan_adaptive_progressive.mat`（2026-09-23） | `[-0.605000, -0.045000, 0.005000]` | `[-0.605000, 0.130000, 0.005000]` |

两者逐位相同 → 工件位姿与焊缝几何没有变。分支差异纯粹来自规划器的**首点种子/beam 选择**：
`replan_weld_actual_axis_continuation.m:314` 用 `qSeeds = scene.qMat0` 起手，`:142` 用
`localKeepBeam(states, beamWidth, scene.qMat0)` 按“离 `scene.qMat0` 最近”挑 beam，
而 `localGenerateFirstBeam` 里 `continuationCost = sum(localWrapRad(q - scene.qMat0).^2)`
（`:331`）同样以 `qMat0` 为基准。所以分支由 `qMat0` 与随机种子共同决定，与锚点无关。

#### 锚点自身是自洽且已验证的

`weld_rrt_connect_approach_exit.mat`（`result` 结构）：

```text
created              = 2026-09-17 15:00:09
sourceWeldResultFile = /home/ttbstt/jixiebi/UR5/matlab仿真/weld_actual_axis_replan.mat
fullPass             = 1
note                 = 80-point weld path was read from the validated result and was not replanned.
weldValidation       : pass=1, nodeCount=80, edgeCount=79, nodeCollisionFree=1,
                       edgeCollisionFree=1, maxJointJumpDeg=9.9538,
                       collisionChecks=327, edgeChecks=79
preWeldValidation    : pass=1, nodeCount=2, edgeCount=1, maxJointJumpDeg=3.2788
qHome                = [3.141593, -1.570796, 0, -1.570796, 0, 0]
max|qWeld(1,:) - preWeld.q| = 3.279 deg
字段                 : created, sourceWeldResultFile, scene, qHome, qWeld, weldResult,
                       preWeld, postWeld, approach, exit, weldValidation,
                       preWeldValidation, postWeldValidation, fullPass, options, note
```

也就是说：`qPreWeld` 是从焊缝起点沿焊丝轴退刀约 30 mm 的接近位姿，与 `qWeld(1,:)` 只差 3.28°，
**锚点分支 A 对全部 80 点都是可行且已通过验证的**（`maxJointJumpDeg=9.95°`）。
所以“把新规划的 80 点拉回分支 A”在可行性上没有风险。

验证过的几何关系（`chk_seam.m` 实测）：

```text
FK(_adaptive qPath(1,:)) 位置 = [-0.605000, -0.045000, 0.005000]   ← 正好是焊缝起点
FK(qPreWeld)         位置 = [-0.583787, -0.045000, 0.026213]
偏移 = [+0.021213, 0, +0.021213]，模长 0.030000 m，姿态差 0.0000°
```

即 `qPreWeld` 与焊缝起点**姿态完全相同，位置沿 [+1,0,+1]/√2 退刀 30 mm**。30 mm 的位移
在约 0.5 m 臂展上对应约 3.4°，与实测 3.279° 吻合 → 锚点语义正确，只是分支不同。

#### 三个候选处理方案（需用户决定，尚无结论）

1. **给规划器加可选首点种子（推荐）。** 给 `replan_weld_actual_axis_continuation.m` 增加两个
   默认关闭、行为不变的可选参数：`initialSeeds`（用锚点里已验证的 `qWeldStart`）与
   `branchReferenceQ`（同值），让第 1 点 beam 回到分支 A；之后整条路径靠 ≤15° 局部传播保持
   在该分支。改动约 4 处，全部由新选项守护。PreWeld/PostWeld 仍是固定任务交接位姿，
   HMI 三个任务可组合。
2. **不动规划器，让锚点跟随。** 每次规划后从 `qSeam(1)`/`qSeam(end)` 按固定 30 mm 沿焊丝轴
   退刀反算 PreWeld/PostWeld 并写回锚点文件。零改动已验证规划器，但 PreWeld 会变成
   “规划器这次挑到的分支”，换种子或换算法即漂移，HMI 的 RealHome→PreWeld 目标不再稳定。
3. **直接用锚点里已验证的 80 点焊缝路径。** 即 `weld_rrt_connect_approach_exit.mat` 的 `qWeld`
   （来源 `weld_actual_axis_replan.mat`，weldValidation PASS），与锚点天然同分支。代价：该路径
   来自较旧的 actual-axis 规划器（约 454 s 版），可能不符合要求 3 的“最新算法”。

### 150.4 待确认：哪个才是「最新焊缝规划算法」

要求 3 让我“优先搜索并确认当前最新版本”。按 mtime 排序，`matlab仿真/` 下比
`replan_weld_actual_axis_adaptive.m`（2026-09-24 12:19）更新的焊缝规划器有：

| 文件 | mtime | 判断 |
|---|---|---|
| `replan_weld_actual_axis_gao_ar_rrtstar.m` | 2026-09-27 15:34 | 文件头自述为 *"Fair, independent Gao-style angular-redundancy RRT\* **baseline**"* → 像是给论文做对比的基线，不是上 HMI 的生产算法 |
| `replan_weld_seam_generalization_core.m` | 2026-09-27 12:49 | 43 KB，名字是“焊缝泛化”。**接口与是否产出完整轨迹尚未核实** |
| `replan_weld_iklink_baseline.m` | 2026-09-28 16:34 | 名字含 baseline |

**倾向于仍用 `replan_weld_actual_axis_adaptive.m`**：手工记录的 §115–117 显示
ACCP → ACCP（30 种子 90% PASS），而 `_adaptive` 的
`maxSegmentTcpErrorMm` / `maxIntermediatePoints` / `maxAdaptiveRounds` 正是 ACCP 的触发器集合，
可视为 ACCP 的生产化版本，符合要求 3 的“ACCP 或其最新优化版本”。
但**这一点没有得到用户确认**，也未核实 `replan_weld_seam_generalization_core.m`。

注意：`plan_hmi_weld_request.m` 现调用 `_adaptive`，若最终选定另一个规划器，`:152` 一行即需替换。

### 150.5 两个「Home」，不要混用（易错点，已核实）

| 名称 | 值 | 出处 | 含义 |
|---|---|---|---|
| MATLAB `qHome` / `scene.qMat0` | `[3.141593, -1.570796, 0, -1.570796, 0, 0]` = `qOffset` | `sync_ur5_joint_states.m:800`（`qMat0 = qCop0 + qOffset`）、`plan_weld_approach_exit_rrt_connect.m:46` | CoppeliaSim 零位映射到 ROS 后的位姿 |
| HMI `real_home` | `[-1.0147631804095667, -1.5450175444232386, 0.03494739532470703, -1.5666325728045862, 0.05197520926594734, -0.007649723683492482]` | `ur5_hmi/config/hmi_config.yaml:26-32` | 真机实测 Home |

**这是两个不同的物理位姿**（j1 相差约 238°），不是同一姿态的两种表示。

- `weld_rrt_connect_approach_exit.mat` 的 `qHome` 是前者（`qOffset`）。
- `轨迹文件/realhome_to_preweld_v1_report.json` 的 `q_start_rad` 也是前者（`qOffset`），
  尽管文件名写着 “realhome”。
- `规划文件/comparison_saved_realhome_v1_request.json` 的 `qStart` 才是配置里的真机 `real_home`
  （差 1.2e-5 rad 以内），且该次规划 `success=true`、`collision_check_pass=true`。

所以 **D4 补 `qHome` 时不能取 `realhome_to_preweld_v1_report.json` 的 `q_start_rad`**
（那是 `qOffset`，会让 PostWeld→RealHome / Full 把机器人规划回 CoppeliaSim 零位而不是真机 Home）。
应以配置 `real_home` 为准，或直接取 `comparison_saved_realhome_v1_request.json` 的 `qStart`。

另附：`轨迹文件/realhome_to_preweld_v1_report.json` 的 `q_end_rad` 与锚点 `qPreWeld` 是同一姿态的
两个 2π 分支（j2/j4/j5/j6 各差正好 6.2832 rad）。HMI 的起点容差检查若不做周期处理，
在这两个表示之间会看到约 360° 误差。

### 150.6 工件 3D 显示：坐标系约定（已核实，含一次自我纠正）

**HMI 的 3D 视图工作帧是 URDF `base`，不是 `base_link`。**
`gui/robot_view.py:133-141` 的 `link_transforms_in_base()` 末尾做了
`T_base_world = inv(frames["base"])`，把所有连杆变换重新基准到 `base`；
`gui/vtk_robot_view.py:297` 直接消费这些矩阵。`AGENT_HANDOFF.md:7125` 也记录
“VTK URDF viewer 的 link transforms 也统一到 URDF `base` frame”。

三层基准（`base_link → base = Rz(π)`，见 URDF:621-632）：

| 基准 | 用途 |
|---|---|
| CoppeliaSim `UR5_base` | `scene_geometry.json` 的工件与焊枪位姿 |
| MATLAB `base_link` | MATLAB 规划场景；`plan_hmi_weld_request.m` 输出的 `tcpPath` |
| URDF `base` | HMI 的 VTK 视图（= 驱动 TCP 话题的 `header.frame_id`） |

冻结标定 `ur5_tool_calibration.mat` 的 `T_CopBase_MatBase`（方向 **Mat→Cop**，由
`sync_ur5_joint_states.m:838-840`、`show_robot_workpiece_frames.m:35` 证实）：

```text
T_CopBase_MatBase = Rz(-90°), 平移 [0, 0, -0.014649842865765095]
```

两个方向的换算式：

```text
p_base_link = Rz(+90°) · p_UR5base + (0, 0, +0.014649842865765095)   % = inv(T_CopBase_MatBase)
p_base      = Rz(-90°) · p_UR5base + (0, 0, +0.014649842865765095)   % = Rz(180°) · inv(T_CopBase_MatBase)
```

工件原点换算：`UR5_base` 的 `[-0.075, 0.375, -0.01465]` → `base_link` 的 `[-0.375, -0.075, 0]`
→ **VTK `base` 帧的 `[+0.375, +0.075, 0]`，姿态 `Rz(-90°)`**。

**两个来源的帧不同，不能共用同一个矩阵**：工件来自 `UR5_base`（用 `Rz(-90°)`），
而 `tcpPath` 来自 MATLAB `base_link`（用 `Rz(180°)`）。
**不要用 `Rz(180°)` 直接当工件换算，也不要漏掉 `+0.0146498 m` 的 Z 平移。**

TCP 标记本身不受影响：驱动话题 frame 就是 `base`，视图工作帧也是 `base`，现在直接使用是正确的
（`vtk_robot_view.py:311-322`）。但它同时接受 `base_link` 而不做 `Rz(180°)` 转换，
是一处既有的小不一致，**记录但不在本次范围内**。

### 150.7 耗时：先前记录不可信，需重测

会话早期提到的 346 s / 454 s 是旧的非 adaptive 版本
（`weld_actual_axis_replan.m` 80 点 = 454.006 s，见 `weld_actual_axis_replan_timing_report.txt`）。

**注意：原先引用的 `AGENT_HANDOFF.md` §106/§107 记录“adaptive ≈ 19.8 s 算法 / 24.5 s 墙钟”是错的**
——本文件存在重复编号，当前 §106/§107 是 2026-09-29 的“真机反馈链只读验收”与“真机姿态再次调整后的
只读核对”，与焊缝计时无关。**adaptive 的真实耗时本轮没有测量**，需按 150.9 的步骤实测。

顺带修正一处本轮自己引入的读数错误：`weld_actual_axis_replan_adaptive_progressive.mat` 的
`finalTcpMetrics.segmentPositionErrorMm` **已经是 mm**（`localMeasureExecution` 里乘过 1000），
最大段间 TCP 误差是 0.266 mm，与 `complete=1` 一致。若再乘 1000 会误读成 265.9 mm。

### 150.8 HMI 侧待做工作的文件位置（已核实，未改动）

- 在线规划页构造：`gui/main_window.py:840-926`；目标标签写死 `"PreWeld"` 在 `:857`。
- `_start_online_planning` `:1700`；`_online_process_finished` `:1779`；
  `_archive_online_result` `:1641`；`_online_goal_report_path` `:1526`；
  `_refresh_online_goal_display` `:1532`；`_update_online_start_display` `:1563`；
  `_adopt_trajectory_file` `:1824`；`_set_preview_pose` `:1947`；`_stop_preview` `:1985`；
  `_matlab_quote` `:1581`；`_update_buttons` `:1304`。
- 3D 视图 `gui/vtk_robot_view.py` **没有任何显示非机器人几何的机制**：`_load_meshes`
  （`:107-194`）只建机器人连杆 actor、J1–J6 标签、TCP 球/连线/标签。
- matplotlib 回退版 `gui/robot_view.py:205` 每 100 ms `ax.clear()`，外部叠加的图元会被擦掉，
  因此叠加必须在 `_refresh` 内部重建。
- `TrajectoryData.tcp_path` 字段已存在（`trajectory/trajectory_utils.py:26`），
  加载器已支持 `tcpPath` 键（`:160`），并已强制校验 `N×3`（`:110`）。
- `UR5/scene_geometry.json` 里工件有 4 个 box 碰撞代理（`WorkpieceCollision_bottom`、
  `wall_x0`、`wall_x1`、`wall_y1`），`geometry_description` 表明 box 即精确几何
  （底板 + 三面 5 mm 侧板，与 MATLAB 碰撞模型同源）。本地没有工件网格文件。
- 环境：PyQt5 / VTK 9.1 / numpy / scipy / yaml 可用；**trimesh 不可用**，**h5py 不可用**
  （因此用 box，不引入新依赖；读 `-v7.3` 的 `.mat` 需走 MATLAB）。

### 150.9 复现本轮结论的方法

本轮全部为只读。已把诊断固化为仓库内的只读脚本（不规划、不写文件、不启动 ROS2/CoppeliaSim、
不下发运动指令）：

```bash
matlab -batch "diagnose_weld_anchor_branch"
```

源码：`/home/ttbstt/jixiebi/UR5/matlab仿真/diagnose_weld_anchor_branch.m`。
实测输出（关键部分）：

```text
anchor fields : qPreWeld, qPostWeld, qWeldStart, qWeldEnd, source
qHome MISSING -> plan_hmi_weld_request.m:274 silently falls back to zeros(1,6)
scene.qMat0   : [ 3.141593 -1.570796 0.000000 -1.570796 0.000000 0.000000]
seam startMat : [ -0.605000 -0.045000 0.005000]
seam endMat   : [ -0.605000 0.130000 0.005000]
qPath(1,:)    : [ 2.741396 -1.387900 1.552137 4.616149 -1.348566 2.004334]
qPath(1)-qPreWeld   : [   157.92    17.85   179.62   -14.84   179.49   -22.66]
qPath(end)-qPostWeld: [   150.90    23.90   165.70   -14.19  -168.66   -18.40]
max |delta|         : 179.62 deg (start) / 168.66 deg (end)
raw (unwrapped) jump to splice [qPre;qSeam;qPost]: 342.15 deg
FK(qPath(1,:)) position : [ -0.605000 -0.045000 0.005000]
FK(qPreWeld)   position : [ -0.583787 -0.045000 0.026213]
offset                  : [ 0.021213 0.000000 0.021213]  |d| = 0.030000 m
orientation difference  : 0.0000 deg
weldValidation       : pass=1 nodeCount=80 edgeCount=79 maxJointJumpDeg=9.9538
preWeldValidation    : pass=1 maxJointJumpDeg=3.2788
max|qWeld(1)-preWeld.q| = 3.279 deg
```

注意拼接到 `[qPre;qSeam;qPost]` 的**未环绕**跳变是 **342.15°**（不是 158°）——因为
`qPreWeld` 与 `qPath(1,:)` 本身还差一个 2π 表示。`localContinuity` 的
`maxJointJumpDeg<=45` 判定必然失败。

下一步若要推进，建议顺序：

1. 先定 150.3 的处理方案与 150.4 的规划器（这两个是阻塞点，其余都是机械工作）。
2. 修 D0–D7，其中 D2 的守卫**必须同时改** `replan_weld_actual_axis_continuation.m:258` 与
   `replan_weld_actual_axis_adaptive.m:328`。**本仓库不是 git 仓库，改动前先手工备份。**
3. 单独跑通焊缝入口（不经 GUI）：
   `matlab -batch` 调 `plan_hmi_weld_request`，期望
   `HMI_WELD_PLAN_DONE ... seam=80 collisionPASS=1 continuityPASS=1`，
   且 `.mat` 里 `tcpPath` 行数 == `qPath` 行数（D0 回归点）。
4. 再改 HMI Python 侧（150.8 的接线点）与 3D 显示（150.6 的换算式）。
5. 全程不点执行、不发送 FollowJointTrajectory；真机不接收运动指令。

上一轮生成的完整方案文件在 `/home/ttbstt/.claude/plans/vast-munching-bear.md`，
其中 **D4 的取值来源与 150.5 冲突，以本节为准**（不要取
`realhome_to_preweld_v1_report.json` 的 `q_start_rad`）。

## 151. ACCP 冻结、方案①分支控制与 HMI 焊缝任务修复（2026-10-08）

本节记录第 150 节列出的两个阻塞点的处理结果，并修正第 150 节若干错误结论。
本轮**全程 NO ROBOT MOTION**：未发送 ROS2 指令、未启动/修改 CoppeliaSim、未触碰真机，
未修改 `controller.py`、`functional.lua`、TCP 标定，也未覆盖任何已验收 MAT。

### 151.1 算法冻结

正式焊缝算法冻结为 **ACCP**：

```text
入口：UR5/matlab仿真/replan_weld_actual_axis_adaptive_feasibility.m
配置：initialPoints=16, maxRefinementRounds=4, maxPoints=40,
      maxSegmentTcpErrorMm=0.5, maxIntermediatePoints=4,
      endMarginM=0.025, enablePostRecovery=true

流程：16 点稀疏起始 → 失败区间局部二分 → 条件 beam/angular 恢复
      → 选择性 TCP 执行点精度修正 → Forward 失败后才 Reverse
```

明确不再使用：

- `replan_weld_actual_axis_continuation.m`（旧 80 点 continuation）
- `replan_weld_actual_axis_adaptive.m`（原型）

`replan_weld_seam_generalization_core.m` 保留为 Seam B/C 泛化实验版本，本轮未修改。

### 151.2 备份（先备份后修改）

```text
/home/ttbstt/jixiebi/backup_before_hmi_weld_20261008/
  plan_hmi_weld_request.m
  replan_weld_actual_axis_adaptive_core.m
  replan_weld_actual_axis_adaptive_feasibility.m
  replan_weld_actual_axis_adaptive.m
  weld_anchor_points.json
```

本仓库不是 git 仓库，所有修改前均手工备份。

### 151.3 方案①：分支参考（branchReferenceQ）

在 `replan_weld_actual_axis_adaptive_core.m` 中新增**默认关闭**的 `branchReferenceQ`，
默认取 `scene.qMat0`，因此**所有既有实验的行为完全不变**。

只改 5 处「分支选择」逻辑，不动任何硬约束：

```text
localGenerateFirstBeam : qSeeds 前导行 + parentQ + continuationCost
localKeepBeam          : 首层 beam 与逐层 beam 的参考构型
localSeedBank          : 后续层种子前导行
```

**刻意不改** `localBuildGraph` 中 `scene.qMat0` 的接近连接性统计——那只是信息性报告，
不是硬约束。`replan_weld_actual_axis_adaptive_feasibility.m` 负责透传该参数。

锚点来源：`UR5/规划文件/weld_anchor_points.json` 的 `qWeldStart`（新增
`branchReferenceQ` 与 `branchReferenceNote` 字段）。

**验收结果**：`qSeam(1,:)` 与已验证 `qWeldStart` **逐位相同**；四个锚点周期性误差全部
为 `0.000e+00`（见 151.7）。

### 151.4 修正第 150 节的错误结论

第 150.3 节报告的「锚点拼接 342.15° 跳变」是**错误的**，它把新规划的 branch-B
`qPath(1,:)` 与锚点比较，而不是把锚点自身内部比较。实测：

```text
qPreWeld  -> qWeldStart : 3.279 deg
qWeldEnd  -> qPostWeld : 3.170 deg
qWeldStart -> qWeldEnd  : 31.648 deg
```

锚点自身完全自洽。真正的分叉只存在于「锚点」与「新规划」之间，因此方案①是正确选择。
第 150.5 节的取值结论（`qHome` 必须取 HMI 真机 `real_home`，不能取
`realhome_to_preweld_v1_report.json` 的 `q_start_rad`）**仍然有效并已采纳**。

新增只读诊断：

```text
UR5/matlab仿真/check_branch_anchor_alignment.m
matlab -batch "info=check_branch_anchor_alignment();"
```

输出确认 `qWeldStart` 的 FK = `[-0.605000 -0.045000 0.005000] m`，与 25 mm 余量的
Seam A 起点误差 `0.000189 mm`；`qWeldEnd` 误差 `0.000168 mm`；PreWeld/PostWeld 退刀
各 `30.0000 mm`；四个锚点全部无碰撞。

### 151.5 D0–D7 修复清单

| 编号 | 修复方式 |
|---|---|
| D0 | `tcpPath` 改为**由最终 `qPath` 推导**，删除 `localUniqueRows`；新增 `qPath`/`tcpPath` 行数一致性断言（>1e-6 差异直接失败） |
| D1 | 改为 `saved=load(outputFile,'metadata'); metadata=saved.metadata;`（`export_hmi_trajectory` 的 report 结构本来就没有 `metadata` 字段） |
| D2 | ACCP 的三个 writer 本已带 `~startsWith(filesep)` 守卫；额外给 `_adaptive` 原型补上同款守卫并补 `mkdir` |
| D3 | 改用 `weldInfo.collisionCheckPass`（原代码错把第 3 个输出 `stageReports` 当成含该字段的结构） |
| D4 | anchors JSON 补 `qHome`（真机 `real_home`，并另存 `qOffsetHomeMatlab` 仅作文档）；请求缺 `qStart` 且锚点无 `qHome` 时**报错**，不再静默 `zeros(1,6)` |
| D5 | 公共校验失败直接 `error()`，不再把无效轨迹写进 HMI 的 `.mat` |
| D6 | 自由空间两段各自 try/catch，失败写入 `report.json` 的 `free_space_info` |
| D7 | 新增**协作式**时间预算 `maxPlanningTimeSec`，在轮与恢复阶段之间检查；MATLAB 无法硬中断正在运行的 IK 调用，单轮仍可能超时，实际耗时始终如实报告 |

### 151.6 额外发现并修复的缺陷（原 wrapper 与本轮自身）

1. **`struct('f', <13×2 cell>)` 生成非标量 struct 数组** → `collisionFreeJointEdge`
   抛 `options must be a scalar structure`。原 wrapper 第 256–257 行即有此 bug，是它
   从未跑通的原因之一。改为逐字段赋值。
2. **2π 拼接断裂**：各段单独做过 continuous-branch lift，但彼此未对齐。Full Task
   实测 `maxJointJump = 360.000°`。已在 `localConcatPaths` 中对每个新段做整圈平移
   对齐（只加减 2π 整数倍，不 wrap 到 [-π,π]，不改变物理构型），并加 ≥180° 自检。
3. **`qGoal` 缺失导致 Home→Home 假成功**：原代码 `localGet(request,'qGoal',qHome)`
   会规划零长度运动并报告 PASS（`RealHome_to_PreWeld` 曾返回 3 点退化路径）。
   已改为按任务解析端点，并新增**端点守卫**：起点/终点周期性误差 >1e-6 rad 即失败。
4. **多行字符串字面量在本机 MATLAB 解析失败**（同第 37 节记录的问题，表现为
   「无效表达式」）。已全部改为单行 `sprintf` 或 `fmt` 变量。
5. **两个 Home 的隔离已在代码中显式化**：`weld_anchor_points.json` 同时保存 `qHome`
   （真机 `real_home`）与 `qOffsetHomeMatlab`（CoppeliaSim 零位映射），并注明两者
   j1 相差约 238°、不可互换。

### 151.7 四任务 matlab -batch 验收（绕开 HMI GUI）

新增 headless 测试入口：

```text
UR5/matlab仿真/run_hmi_weld_seam_batch.m
matlab -batch "run_hmi_weld_seam_batch('<Task>',20260917,'<dir>')"
```

结果（seed `20260917`，全部 PASS）：

```text
Task                    rows  collision  continuity  rowsMatch  TCP err    maxJump   wall
Weld_Seam                 26  PASS       PASS        26/26      0.000000mm  13.518°   6.4 s
RealHome_to_PreWeld       11  PASS       PASS        11/11      0.000000mm  28.354°  34.5 s
PostWeld_to_RealHome      11  PASS       PASS        11/11      0.000000mm  20.524°  34.0 s
Full_Welding_Task         51  PASS       PASS        51/51      0.000000mm  28.354°  72.2 s
```

独立复核端点（周期性差异，全部为 0）：

```text
hmi_weld_RealHome_to_PreWeld     rows=11  start-qHome=0.00e+00  end-qPreWeld=0.00e+00
hmi_weld_PostWeld_to_RealHome    rows=11  start-qPostWeld=0.00e+00  end-qHome=0.00e+00
hmi_weld_Weld_Seam               rows=26  start-qPreWeld=0.00e+00  end-qPostWeld=0.00e+00
hmi_weld_Full_Welding_Task       rows=51  start-qHome=0.00e+00  end-qHome=0.00e+00
```

Full Welding Task 分段结构（从各 stage MAT 直接核对，非启发式）：

```text
stage rows: approach=11  qSeam(exec)=24  exit=16   -> total 51
qPreWeld row=11   qWeldStart row=12   qPostWeld row=36
splice qPreWeld  = 8.241 deg
splice qPostWeld = 13.518 deg
max jump inside approach = 28.354 deg
max jump inside exit     = 19.575 deg
max joint jump in weld seam block = 13.518 deg
dense collision checks  : Weld_Seam 182 / Full 910
```

Weld_Seam 的 Forward 首轮即 PASS，Reverse 未触发（`seam_direction=forward`，
`cascade=forward_first_pass`）。Seam 规划点 16、执行点 24。

结果目录：`UR5/planning_runs/hmi_weld_<Task>/`。

### 151.8 ACCP 真实墙钟时间（实测，非旧原型）

新增：

```text
UR5/matlab仿真/time_aarc_ik_plus_coldstart.m
matlab -batch "time_aarc_ik_plus_coldstart(struct('seeds',20260917:20260922))"
```

6 seed × 2 配置，全部冷启动，不读取任何历史 `qPath`/状态图/checkpoint：

```text
branchReferenceOn    runs=6 pass=6/6
  cold-start wall (含场景加载) : median 3.639  P90 5.599  max 5.599 s
  planner algorithm time       : median 3.203  P90 3.782  max 3.782 s
  IK calls                     : median 334
  final planning points        : median 16  max 16
  execution points             : median 24
  max segment TCP error        : max 0.4267 mm
  max joint jump               : max 8.786 deg

branchReferenceOff   runs=6 pass=6/6
  cold-start wall (含场景加载) : median 4.801  P90 22.562  max 22.562 s
  planner algorithm time       : median 4.255  P90 20.156  max 20.156 s
  IK calls                     : median 740
  final planning points        : median 16  max 21
  execution points             : median 23
  max segment TCP error        : max 0.3316 mm
  max joint jump               : max 8.703 deg
```

结论：**方案①不是代价，而是更快更稳**——算法时间中位数 3.203 s（对比 4.255 s），
尾延迟 P90 3.782 s（对比 20.156 s），IK 调用 334（对比 740）。
`branchReferenceOff` 即历史默认配置，其 6/6 通过与约 4 s 量级与第 125 节
（new100 成功样本总算法时间 median 3.867 s）一致，可视为**默认行为未变**的旁证。

输出：

```text
UR5/matlab仿真/aarc_ik_plus_coldstart_timing.mat
UR5/matlab仿真/aarc_ik_plus_coldstart_timing.csv
UR5/matlab仿真/aarc_ik_plus_timing_runs/
```

第 150.7 节标记为「需实测」的 adaptive 真实耗时，现已由本节给出（ACCP 版本）。
旧的 `346 s / 454 s` 仍只属于未自适应、非 continuation 的历史版本。

### 151.9 本轮约束遵守情况

- 未恢复 CoppeliaSim Python `simROS2` subscriber；未修改 `functional.lua`
- 未修改 `controller.py`、ROS2 topic 或任何 ROS2 控制链路
- 未修改冻结的 TCP 标定（`T_mount_tcp`、`qOffset`、`T_CopBase_MatBase`、`T_tool0_mount`）
- 未覆盖 `weld_rrt_connect_approach_exit.mat`、`weld_actual_axis_replan*.mat` 等已验收结果
- 未发送任何 `/joint_trajectory` 或 FollowJointTrajectory；真机未收到运动指令
- HMI 的 GUI 侧（第 150.8 节列出的接线点）本轮**尚未修改**，HMI 仍调用旧的
  `plan_hmi_request.m`；本轮只完成了 MATLAB 侧与 headless 验证

## 152. HMI 在线规划正式接线：ACCP 焊缝任务 + 3D 工件/焊缝显示（2026-10-08）

第 151 节完成 MATLAB 侧验收后，本节把 HMI 的在线规划页接到
`plan_hmi_weld_request.m`，并为 VTK 视图增加只读的工件/焊缝/规划 TCP 路径图层。

本轮**NO ROBOT MOTION**：未发送 ROS2 消息、未调用 FollowJointTrajectory、未使用
STOP MOTION、未启动真机运动、未修改 `controller.py` / `functional.lua` / TCP 标定，
**也未修改 ACCP 核心**（`replan_weld_actual_axis_adaptive_core.m` 与
`replan_weld_actual_axis_adaptive_feasibility.m` 自第 151 节起未再改动）。

### 152.1 任务分派

在线规划页新增任务下拉框，四种任务全部走 `plan_hmi_weld_request.m`：

```text
RealHome → PreWeld      自由空间，算法可选
Weld Seam               ACCP 焊缝
PostWeld → RealHome     自由空间，算法可选
Full Welding Task       APPROACH → WELD → EXIT
```

- 自由空间段算法下拉框保留 `RRT` / `RRT-Connect` / `Bi-RRT*` /
  `Constraint-aware Bidirectional RRT* (CBi-RRT*)`，用于论文对比；默认仍为 V1。
- 焊缝段算法**固定显示为 `ACCP（固定）`**（绿色加粗），不受下拉框影响。
  论文对比算法不进入正式焊缝执行入口。
- 端点不再依赖 `realhome_to_preweld_v1_report.json`，改为读取
  `规划文件/weld_anchor_points.json` 的 `qPreWeld` / `qPostWeld` / `qHome`，
  其中 `qHome` 是真机实测 `real_home`，**不是** `qOffset`。

### 152.2 阶段状态与取消

`plan_hmi_weld_request.m` 新增阶段标记，HMI 解析后显示：

```text
HMI_WELD_STAGE stage=PREPARING / APPROACH / WELD / EXIT / VALIDATING / EXPORTING
```

Full Welding Task 的阶段顺序已改为 APPROACH → WELD → EXIT（此前是 WELD → APPROACH →
EXIT），使显示顺序与物理任务顺序一致。

GUI 显示：`STARTING MATLAB` → `PREPARING` → `PLANNING APPROACH` → `PLANNING WELD`
→ `PLANNING EXIT` → `VALIDATING` → `EXPORTING` → `SUCCESS` / `FAILED`。
Forward 焊缝失败时额外显示 `WELD FORWARD FAIL → REVERSE`。

「取消规划」只 `terminate()` MATLAB 的 `QProcess`，不调用 STOP MOTION、不取消任何
FollowJointTrajectory goal、不发布任何 ROS2 消息。Full Task 约 70 s 属正常，GUI 通过
阶段字段区分「正在工作」与「卡死」。

### 152.3 结果自动加载

规划成功后自动读取 `trajectory.mat` + `report.json`，送入现有
trajectory loader → validator → 3D preview → Final Execution Check。
**不自动执行**，必须人工点击验证与预演。

结果字段（全部已实现并验证）：Task / Algorithm / Planning Time / Path Points /
Duration / Collision / Continuity / TCP Error / Max Joint Jump；Weld Seam 额外显示
Initial Planning Points（固定 16）、Final Planning Points、Execution Points、
Adaptive Inserted Points、Forward/Reverse、ACCP result；Full Task 显示
Approach / Weld / Exit / Total points。

### 152.4 3D 只读场景图层

`gui/vtk_robot_view.py` 与 matplotlib 回退 `gui/robot_view.py` 增加 6 个显示开关：

```text
☑ Robot  ☑ Workpiece  ☑ Seam  ☐ Planned TCP Path  ☑ Current TCP  ☑ Collision Geometry
```

新增只读模块 `gui/scene_geometry.py`：直接读取
`UR5/scene_geometry.json` 的 4 个真实碰撞 box 尺寸与位姿，**不重新创建尺寸**，
半透明灰色显示；工件、焊缝、规划 TCP 路径在 3D 中各自使用不同线型/颜色
（工件半透明灰、焊缝黄色实线、规划路径绿色虚线、当前 TCP 红色球）。
相机「自动适配」在工件可见时同时取景机器人与工件。

### 152.5 坐标变换（严格区分，已数值验收）

```text
工件   ：CoppeliaSim UR5_base -> VTK base
          p_base = Rz(-90°) * p_UR5base + [0, 0, 0.014649842865765095]
tcpPath ：MATLAB base_link    -> VTK base
          p_base = Rz(180°)  * p_base_link
Current TCP ：已经是 URDF base，直接显示（不做任何变换）
```

两个来源**不使用同一个矩阵**。独立验证脚本：

```text
UR5/ur5_hmi/tools/verify_scene_frames.py
python3 tools/verify_scene_frames.py
```

实测：规划焊缝 TCP 与由工件 box 推导的 Seam A 法向偏差 **max 0.0002 mm**，
沿焊缝跨度 175.000 mm，证明两个独立变换落在同一物理几何上。

**本轮修正的一个真实几何缺陷**：最初只把 box 的*中心*做了变换，忘了变换 box 的*轴向*，
导致 Workpiece 的 x 向尺寸被画到 base 的 −y 方向，工件包围盒变成 350×465×235 mm。
现改为有向 box（`Box.axes_base`，8 个角点全部经 `R_base` 变换），
实测包围盒恢复为 235×235×235 mm（base 帧下为 235 mm 的一个置换）。

### 152.6 焊缝定义

`gui/scene_geometry.py` 提供 Seam A/B/C 中心线（均为只读推导）：

```text
Seam A = bottom x wall_y1,  gun axis [0, +1, -1]/sqrt(2),  长度 175 mm
Seam B = bottom x wall_x0,  gun axis [-1, 0, -1]/sqrt(2),  长度 180 mm
Seam C = bottom x wall_x1,  gun axis [+1, 0, -1]/sqrt(2),  长度 180 mm
```

与第 130 节的 Workpiece 坐标参考值一致：

```text
Seam B : [0.005, 0.025, 0.005] -> [0.005, 0.205, 0.005]  (180 mm)
Seam C : [0.230, 0.025, 0.005] -> [0.230, 0.205, 0.005]  (180 mm)
```

首版 B/C 实现错误地沿 +X 取了 50 mm 短段，已修正为沿 +Y 从开口侧到 `wall_y1` 内表面。
工件与焊缝**没有任何人工平移**。

### 152.7 修正：2π 分支表示导致软件限位失败

第 151 节的 `localConcatPaths` 为保证拼接连续会按整圈平移某段，导致
`PostWeld_to_RealHome` 的终点 `qHome` 的 J6 落在 **−360.44°**，刚好越过 HMI 的
±360° 软件限位，被 trajectory validator 判为
`point 11: software joint limit`——尽管物理位姿与配置里的 `qHome` 完全相同。

修法：`localCanonicalizeBranch` 对每个关节施加**一个常量整圈**，使整列落入
[−π, π]。因为是常量平移，所有相邻差值（以及插值路径与连续性指标）完全不变，
物理构型也不变。这**不是 wrapToPi**：不会逐行独立重表示，因此不会制造整圈跳变。
跨距超过 2π 的关节会被跳过并打印 WARN，绝不折叠。报告新增
`branch_canonicalized` 与 `branch_turns` 字段以供审计。

> 已知残留项（未修改 Final Gate，按要求保留）：HMI validator 的起点误差仍用
> `q[0] - q_actual` 的普通差值，未做周期处理。若实时 `/joint_states` 恰好以
> ±π 附近的另一种 2π 表示回报，可能出现约 360° 的假误差。规范化到 [−π, π]
> 后与驱动通常的表示一致，可降低但不能消除该风险。此项与第 150.5 节同源，
> 后续如需修复应单独评估。

### 152.8 验证

三个只读/无运动测试脚本，全部通过：

```text
UR5/ur5_hmi/tools/verify_scene_frames.py        坐标变换数值验收
UR5/ur5_hmi/tools/smoke_hmi_weld_planning.py    offscreen MainWindow，55/55
UR5/ur5_hmi/tools/smoke_vtk_scene.py            真实 VTK 视图，15/15
UR5/ur5_hmi/tools/e2e_hmi_weld_planning.py      真实 QProcess 端到端，4 任务
```

四种任务的端到端结果（seed `20260917`，起点 Saved RealHome）：

```text
Task                  点数  collision continuity 端点误差      墙钟    结果
RealHome_to_PreWeld    11   PASS      PASS       0.00e+00   37.5 s   ALL PASSED
Weld_Seam              26   PASS      PASS       0.00e+00   10.2 s   ALL PASSED
PostWeld_to_RealHome   11   PASS      PASS       0.00e+00   37.0 s   ALL PASSED
Full_Welding_Task      51   PASS      PASS       0.00e+00   76.5 s   ALL PASSED
```

Full Welding Task 分段与焊缝一致性：

```text
stage_point_counts : approach=11, weld=25, exit=15, total=51
seam               : 16 planning points -> 24 execution points, 8 inserted
direction          : forward_first_pass（Forward 即通过，Reverse 未触发）
weld block         : 25 rows, max deviation from Seam A 30.00 mm（含 30 mm 退刀）
                     沿焊缝跨度 175.00 mm
preview            : 抽样每个 index 的机器人位姿与 traj.positions 完全一致（0 bad）
Final Gate         : execute 按钮保持 disabled；motion_busy=False；goal_active=False
```

关键不变式核对：

```text
ros/  safety/  trajectory/  与本轮开始前逐字节一致（未改动）
_motion_busy / _stop_motion / _execute_path / _confirm_execute
_update_trajectory_gate / _set_trajectory_summary / _validate_path / _save_home
                     未被本轮修改
```

### 152.9 尚未完成 / 后续

- HMI 页面仍是只读显示：工件与焊缝来自 CoppeliaSim 导出几何，真机现场工装坐标
  需要重新标定，不能直接复用。
- 第 152.7 节的起点误差周期化问题未处理（按要求未改 Final Gate）。
- 未做真机执行验收；本轮所有 Execute 路径保持 blocked。

## 153. 中断后复核与真机验收冻结条件（2026-10-08）

本次从交接点继续做了只读复核，确认第 152 节的离线链路仍与当前文件一致：

- 当前 MATLAB HMI 入口仍为 `RRT/benchmark/plan_hmi_weld_request.m`；焊缝段固定调用
  ACCP（`replan_weld_actual_axis_adaptive_feasibility.m`），不是旧的逐点独立 IK。
- HMI 仍提供 `RealHome → PreWeld`、`Weld Seam`、`PostWeld → RealHome`、
  `Full Welding Task` 四种任务；自由空间算法下拉框不改变焊缝段算法。
- 工件与焊缝显示继续直接使用 `UR5/scene_geometry.json` 和现有坐标变换；没有新增手工偏移。
- `TrajectoryData` 已保留可选 `tcp_path`，用于 HMI 的 Planned TCP Path 显示，不会送入 ROS 控制器。

只读验证结果（已 source `/opt/ros/humble/setup.bash`）：

```text
verify_scene_frames.py       PASS（焊缝/工件坐标、175 mm 跨度、0.0002 mm 最大偏差）
smoke_hmi_weld_planning.py   55/55 PASS（无真机运动）
smoke_vtk_scene.py           15/15 PASS（工件/焊缝/TCP 路径图层）
python3 -m compileall -q .   PASS
```

已验收的 Full Welding Task 样例仍为：51 点、collision PASS、continuity PASS、
planning time 72.490 s、duration 81.368 s；Final Execution Check 保持人工门控，未发送
任何 FollowJointTrajectory goal。

### 153.1 真机前唯一未决软件项：起点角度的 2π 表示

当前 `trajectory_validator.py` 与 `safety_checks.py` 仍采用普通差值
`q_path(1,:) - q_actual`。因此同一物理姿态若由驱动以相差整圈的连续表示返回，可能被
误报为约 360° 起点误差。这一点**尚未修复，也不能在本阶段用逐点 `wrapToPi` 修复**。

若后续处理，只允许对某一关节的整条轨迹施加同一个整数圈 `2*k*pi`，并重新通过：

```text
joint limits
continuity / adjacent jump
start error
node + dense edge collision validation
```

找不到合法统一分支时，Final Execution Check 必须继续保持 `BLOCKED`。当前不要修改
ACCP、HMI 主界面或 ROS2 控制链路。

### 153.2 建议的真机分阶段验收顺序

```text
备份冻结版本
→ 启动 UR5 + ROS2 Driver + External Control，只读确认 Final Gate
→ RealHome → PreWeld（5%，焊机关闭，单独 dry-run）
→ Weld Seam（空走，不起弧）
→ PostWeld → RealHome
→ Full Welding Task（51 点，仍先空走）
→ 全部验收后再考虑真实焊接
```

首次真机实验必须记录 `q_command`、`q_actual`、joint tracking error、planned/actual
TCP、timestamp、`robot_mode`、`safety_mode`、`speed_scaling`。在起点分支问题未完成验证前，
不得直接执行完整 Full Welding Task。

## 154. 在线规划页布局整理（2026-10-09）

本轮只调整 `UR5/ur5_hmi/gui/main_window.py` 的在线规划 Tab 布局，不修改 MATLAB
规划器、ROS2 控制、安全门、轨迹执行或真机逻辑。

页面现在按固定信息层级显示：

```text
1. 规划设置
   任务类型 / 起点 / 目标 / 自由空间算法 / ACCP / Seed / 时间预算
   目标来源 / 起点快照

2. 规划控制
   开始规划 / 取消规划 / 当前阶段 / 状态说明

3. 结果摘要
   基础指标卡片 + 焊缝/分段指标卡片

4. MATLAB 输出（只读）
```

追加整理：在线规划页现在使用内部 `QScrollArea`，规划设置区最低高度 210 px，
各设置行最低高度 36 px，起点快照单独占一行并显示 `Source` / `Freshness`。规划控制、
结果摘要和 MATLAB 输出在 100%/125%/150% 下按比例放大，不再用压缩行高强行塞入页面。

任务字段会按任务类型显示：自由空间任务显示自由空间算法，焊缝任务显示固定的
ACCP，Full Task 显示 Approach/Weld/Exit 组合信息。`qPreWeld` 目标显示已与
实时 `/joint_states` 解耦：即使当前机器人反馈不新鲜，目标仍会显示为 `PreWeld`，而
“起点快照”明确显示为 `Freshness: WAITING`，规划按钮仍由原有实时反馈门控。

保留原有控件对象名和信号连接，因此规划流程、自动加载、取消规划和 Final Gate
行为不变。结果字段没有删除，仅将原来的长单列表拆成两张指标卡片；MATLAB 输出
限制为紧凑的只读区域，避免挤占设置和结果区域。

验证：

```text
source /opt/ros/humble/setup.bash
python3 -m compileall -q UR5/ur5_hmi       PASS
python3 UR5/ur5_hmi/tools/smoke_hmi_weld_planning.py  55/55 PASS
```

三档 UI 尺寸检查：

```text
100%：设置区 210 px，输出区 105 px，PreWeld 显示正常
125%：设置区 262 px，输出区 131 px，PreWeld 显示正常
150%：设置区 315 px，输出区 158 px，PreWeld 显示正常
```

本轮仍为 NO ROBOT MOTION。

## 155. 点动不可用诊断（2026-10-09）

本次只读检查发现，点动按钮变灰不是 GUI 布局造成的，而是 HMI 安全门的预期行为：

```text
/joint_states                                  有效
robot_mode                                     7 (RUNNING)
safety_mode                                    1 (NORMAL)
speed_scaling                                  100%
robot_program_running                          true
scaled_joint_trajectory_controller             inactive
```

当前 ROS2 Driver 启动命令包含：

```text
activate_joint_controller:=false
```

因此 `check_motion_ready()` 会拒绝点动。FollowJointTrajectory action server 虽然存在，
但对应控制器未 active，不能作为可运动条件。

另外只读进程检查发现当前有两个 `ur_control.launch.py` 和两个 `python3 main.py` 实例。
真机验收前必须只保留一个 ROS2 Driver、一个 HMI，避免两个控制管理器/界面实例互相干扰。

恢复顺序：确认 External Control 正在运行后，再由操作者手动激活
`scaled_joint_trajectory_controller`，确认 `ros2 control list_controllers` 显示 active，
然后刷新 HMI。未执行控制器切换，也未发送点动或轨迹指令。

## 156. 点动状态恢复与 program_running 兼容修复（2026-10-09）

后续复核发现，CB3/External Control 在 ROS2 Driver 重启后可能暂时不向
`/io_and_status_controller/robot_program_running` 发布首帧；但 Dashboard 只读服务
`/dashboard_client/program_running` 已能返回真实状态。这会让 HMI 的状态保持为未知，
从而按安全门规则禁用 Jog，即使机器人实际上已经在运行。

修复文件：

```text
UR5/ur5_hmi/ros/ros_node.py
```

保留原话题订阅，并增加只读服务补偿查询。服务成功时只更新
`RobotState.program_running`，不改变 `check_motion_ready()` 的任何安全条件，
不自动上电、不释放抱闸、不激活控制器、不发送运动。

本次现场状态验证：

```text
program_running       = True
robot_mode            = 7
safety_mode           = 1
speed_scaling         = 100.0
scaled controller     = active
FollowJointTrajectory = ready
joint feedback        = 6 joints, fresh
```

同时清理了重复 HMI，仅保留一个 `python3 main.py`；ROS2 Driver 保持单实例。
当前 Driver 仍使用 `activate_joint_controller:=false`，这是有意保留的安全策略。
每次 Driver 重启后，如控制器再次为 inactive，必须由操作者确认 External Control 和
现场安全状态后手动激活 `scaled_joint_trajectory_controller`，不能由 HMI 自动激活。

## 158. 3D 模型区显示控件重叠修复（2026-10-09）

修复了 HMI 顶部 3D 模型区中“等轴/俯视/正视/侧视/自动适配”按钮与
Robot、Workpiece、Seam、Planned TCP Path、Current TCP、Collision Geometry
显示开关互相覆盖的问题。

修复文件：

```text
UR5/ur5_hmi/gui/main_window.py
```

本次仅调整 GUI：

- 增大顶部模型区和模型卡片的最小高度；
- 为 3D 视图设置可缩放的最小高度；
- 视图按钮单独占一行；
- 显示开关改为 3 列 × 2 行网格；
- 所有间距、按钮高度和字号继续由 UI Scale（100%/125%/150%）统一计算。

验证结果：

```text
100% / 125% / 150%：视图按钮与显示开关无重叠
python3 -m compileall -q .                         PASS
tools/smoke_hmi_weld_planning.py                   55/55 PASS
```

本轮没有修改 MATLAB 规划、ROS2 控制、安全门、轨迹执行逻辑，也没有发送机器人运动指令。

## 157. 点动重复触发事件（2026-10-09）

现场点击 J1 后发现多个约 2° 的点动 goal 连续进入
`scaled_joint_trajectory_controller`。驱动日志连续出现：

```text
Received new action goal
Accepted new action goal
Goal reached, success!
```

实时只读监视确认 J1 持续变化。为安全起见，已调用 Dashboard `stop` 停止
`ExternalControl.urp`，并关闭 HMI；没有自动回零或发送任何回退轨迹。当前机器人停在
操作者最后形成的姿态，J1 约 `-12.3 deg`（仅作记录，不能当作 RealHome）。

GUI 端已增加以下防护：

- `JogButtons` 显式关闭 Qt auto-repeat；
- GUI 端 `_jog_pending` 互斥锁，一次点击最多产生一个未完成 goal；
- goal 完成/拒绝后才重新启用点动按钮；
- worker 在发送被 busy 拒绝时回传状态，避免界面锁死。

修复文件：

```text
UR5/ur5_hmi/gui/widgets.py
UR5/ur5_hmi/gui/main_window.py
```

补丁通过 `compileall` 和 HMI 55/55 无机器人运动 smoke test。重新启动 HMI 前，必须由
现场人员确认 J1 当前姿态和周围安全；重新运行 External Control 后再手动激活运动控制器。

## 159. 主界面 3D 区放大与布局优化（2026-10-09）

本轮只调整 HMI GUI 和 3D 相机显示，未修改机器人运动学、坐标系、工件几何、ROS2、
安全门或轨迹执行逻辑。

修复文件：

```text
UR5/ur5_hmi/gui/main_window.py
UR5/ur5_hmi/gui/vtk_robot_view.py
UR5/ur5_hmi/gui/robot_view.py
```

改动摘要：

- 上部 3D/状态区与下部 Tab 改为可拖动的上下 `QSplitter`，默认约 58% / 42%；
- 上部左右比例调整为约 70% / 30%；
- 3D 控制条压缩为两行：视图按钮一行、六个显示开关一行；
- 右侧安全控制改为窄屏纵向布局，状态/TCP 卡片不再被横向挤压；
- 轨迹执行页面改为内部 `QScrollArea`，避免内容不足时强制拉伸；
- 增加 `机器人` 与 `机器人+工件` 两种适配模式；
- VTK fit bounds 只使用 UR5 mesh 和可选工件，不包含 TCP、文字、坐标轴、焊缝标记、
  TCP 辅助线或 Planned TCP Path；
- 自动适配后使用约 3.5x camera zoom；不改变任何几何或坐标变换；
- Matplotlib fallback 同步使用机器人/工件 bounds 适配。

布局测试结果：

```text
100%：上部/下部约 58% / 42%，无重叠
125%：上部/下部约 58% / 42%，无重叠
150%：上部/下部约 56% / 44%，无重叠
```

截图：

```text
UR5/ur5_hmi/screenshots/main_layout_100.png
UR5/ur5_hmi/screenshots/main_layout_125.png
UR5/ur5_hmi/screenshots/main_layout_150.png
UR5/ur5_hmi/screenshots/main_layout_vtk_scene_100.png
```

验证：`python3 -m compileall -q .` 通过，`smoke_hmi_weld_planning.py` 为 `55/55 PASS`；
本轮没有发送真机运动指令。

## 163. 3D XYZ 标签缩小（2026-10-09）

用户反馈放大后的 UR5 3D 视图中 XYZ 字母过大。本轮只改显示：

- `vtk_robot_view.py` 关闭 VTK 默认会随相机放大的轴标题，改用紧凑 billboard 标签；
- `robot_view.py` 的 Matplotlib fallback 同步缩小 X/Y/Z 文本；
- 坐标轴原点、方向、长度、模型坐标、工件坐标和 TCP 均未修改。

验证：`python3 -m compileall -q .` 通过，`smoke_vtk_scene.py` 为 `15/15 PASS`，
`smoke_hmi_weld_planning.py` 保持 `55/55 PASS`；没有发送真机运动指令。

补充验证：VTK 原生渲染窗口截图确认机器人与工件均可正常显示，模型占视窗高度约
70%--80%。Qt `QWidget.grab()` 在当前 X11/VTK 嵌入窗口上可能得到黑色视口，这是截图抓取
路径的限制，不影响实际 QVTK 渲染；`smoke_vtk_scene.py` 仍为 `15/15 PASS`。

## 160. 关节点动页可见性与上电诊断（2026-10-09）

关节点动页现在在选中时自动使用约 46% / 54% 的上下分割比例，J1--J6 行高按 UI
Scale 设置为约 50/56/62 px，确保六行及 Jog 按钮同时可见；其它页面恢复约 58% / 42%。
这只改变 GUI 布局，不改变点动、控制器、安全门或轨迹逻辑。

现场只读诊断显示 ROS2 话题当前为 `robot_mode=3 (POWER_OFF)`、`speed_scaling=0.0`，
但 `/joint_states` 仍以约 125 Hz 发布。因此 HMI 的“ROS2 Driver Connected”表示网络/驱动
链路可达，而 `Joint State stale`、点动禁用和 `FINAL CHECK BLOCKED` 是安全门的预期结果，
不是 HMI 断线。示教器上还需执行机器人 Power On、释放抱闸并运行 External Control，直到
`robot_mode=5/7`、`speed_scaling>0`、`robot_program_running=true`；HMI 不会自动替这些步骤。

现场复核还发现机器人重启后旧 RTDE 会话可能卡在 POWER_OFF，而 Dashboard 已恢复 RUNNING。
已在 `UR5/ur5_hmi/run_hmi.sh` 增加只读 RTDE/Dashboard 不一致检测；检测到该特定失配时，
脚本仅重启旧 ROS2 Driver 进程并重新等待端口，不执行任何机器人运动。

随后已在现场仅重启一次 ROS2 Driver（未发送轨迹），当前只读状态恢复为：
`robot_mode=7 (RUNNING)`、`robot_program_running=true`、`speed_scaling=100%`。
`scaled_joint_trajectory_controller` 当前仍为 `inactive`，因此点动/轨迹执行继续保持安全阻止，
需按既有流程由操作者手动激活控制器。

随后按用户确认的现场步骤执行了只切换控制器的命令（未发送 FollowJointTrajectory）：
`ros2 control switch_controllers --activate scaled_joint_trajectory_controller --strict`。
结果为 `scaled_joint_trajectory_controller=active`、`joint_state_broadcaster=active`，
`joint_trajectory_controller` 仍为 `inactive`。启动文件中的
`activate_joint_controller:=false` 保持不变，避免 HMI 自动取得运动控制权。

## 162. HMI 主界面左右分栏（2026-10-09）

本轮只调整 GUI 布局，没有修改 ROS2、运动控制、安全门、MATLAB/ACCP、机器人或
工件几何、TCP 和坐标变换。

- `UR5/ur5_hmi/gui/main_window.py` 的主窗口改为单个水平 `QSplitter`，默认左右 50:50，
  左侧完整占用 UR5 3D 视图，右侧为安全区和功能 Tab；用户可拖动分隔线。
- 右侧长期显示内容精简为 STOP MOTION、电源、机器人状态、安全状态；原 ROS2 详细状态、
  TCP Position/Orientation、Jog 状态移到“运行日志 → 详细状态”。
- Jog 页不再显示右侧 Jog 状态卡，J1--J6 和 +/- 控件使用整个右侧 Tab 区。
- 新增“工件标定（只读）”Tab，仅显示现有 `scene_geometry.json` 来源及坐标关系，不提供
  几何编辑。
- 左侧视图按钮、六个图层开关和 camera fit 逻辑保持不变；未移动或缩放任何模型几何。

验证：`python3 -m compileall -q .` 通过，`tools/smoke_hmi_weld_planning.py` 为 `55/55 PASS`。
布局检查确认主 splitter 为 Horizontal、默认尺寸约 50:50、Tab 共 6 个、Jog 表含 J1--J6；
本轮没有发送真机运动指令。

## 161. 重启后自动恢复控制器接口（2026-10-09）

为避免每次重启后手动切换 `scaled_joint_trajectory_controller`，已在
`UR5/ur5_hmi/run_hmi.sh` 增加后台 controller watchdog：

- 不自动上电、释放抱闸、运行 External Control 或发送轨迹；
- 只有只读状态同时满足 `robot_mode=7`、`robot_program_running=true`、
  `safety_mode=1`、`speed_scaling>0` 时，才重试激活
  `scaled_joint_trajectory_controller`；
- 控制器切换只取得 command interface，不产生运动目标；
- 条件不满足时保持 inactive，HMI 点动/轨迹 Final Gate 继续 BLOCKED；
- 设置 `UR5_AUTO_ACTIVATE_CONTROLLER=0` 可关闭自动重试，恢复只读手动模式。

`start_ur5_real.sh` 仍保持 `activate_joint_controller:=false`，避免驱动启动阶段抢占
控制权。以后统一运行 `UR5/ur5_hmi/run_hmi.sh`，它负责旧 RTDE 会话检查、驱动重连，
以及满足安全条件后的控制器激活。已通过 `bash -n run_hmi.sh`、控制器状态解析和只读
readiness 检查；没有发送 FollowJointTrajectory。

## 163. 工件三点示教标定页（2026-10-09）

`UR5/ur5_hmi/gui/main_window.py` 已将“工件标定”占位页替换为只读 TCP 三点示教流程：
读取 P0/PX/PY 与重复测量点 P_CHECK，计算 `T_base_workpiece_real`，检查右手系和 2 mm
残差阈值，只有 `VALID` 才可保存。结果保存到
`/home/ttbstt/jixiebi/UR5/workpiece_pose_real.json`，不会发送运动、修改 TCP 标定或改动
ROS2 控制器。

`gui/vtk_robot_view.py` 和 `gui/robot_view.py` 增加了显示层工件校准变换：真实标定成功后，
工件、焊缝和已加载的规划 TCP 路径一起切换到实测位姿；“恢复仿真位姿”仅清除显示变换。
机器人 mesh、TCP、坐标系和 `scene_geometry.json` 均保持不变。MATLAB 规划器若要使用实测
位姿，需显式读取该 JSON；本轮未自动改写 MATLAB 场景。

验证：`python3 -m compileall -q UR5/ur5_hmi`、`smoke_hmi_weld_planning.py` 为 `55/55`
通过，`smoke_vtk_scene.py` 为 `15/15` 通过；三点计算/保存/恢复离线 smoke 通过。本轮
没有发送真机运动指令。

## 164. 运行日志页精简（2026-10-09）

运行日志页改为紧凑的“关键诊断状态 + 事件日志”布局，仅长期显示 ROS2 Driver、程序、
robot_mode、safety_mode、speed_scaling、scaled controller 和 Joint State。TCP 数值卡片与
Jog 当前指令卡片不再嵌套显示，避免窄右栏中的纵向挤压和文字裁切；底层状态变量、点动
逻辑和安全门均未改变。保留完整 TCP 诊断对象供状态更新和后续诊断使用。

## 165. TCP 定点旋转标定页（2026-10-09）

`UR5/ur5_hmi` 新增“TCP标定”Tab，与“工件标定”并列。标定页支持手动采集 4–6 组姿态，
每次只读取最新 `/joint_states`，使用 `real_robot_description_calibrated.urdf` 的
`base -> flange` FK，严禁使用已包含控制器 TCP 偏移的 `/tcp_pose_broadcaster/pose` 求解。

求解实现位于 `UR5/ur5_hmi/calibration/tcp_pivot.py`，模型为
`R_i*p_flange_tcp+t_i=p_pivot_base` 的最小二乘定点旋转法。结果检查姿态跨度、矩阵秩、
条件数、RMS/最大残差；至少 4 组、rank=6、RMS 不超过 2 mm 才可保存。可采集新姿态做
独立验证，验证最大误差不超过 2 mm 才允许应用。

独立结果保存到 `UR5/real_tcp_calibration.json`，应用记录保存到
`UR5/real_tcp_calibration_applied.json`。MATLAB 显式读取入口为
`UR5/matlab仿真/load_real_tcp_calibration.m`；没有自动修改冻结的
`UR5/ur5_tool_calibration.mat`、示教器 TCP、ROS2 控制或 ACCP。

应用需要人工确认。应用后 HMI 可显示独立的 `TCP calibrated (FK)` 青色标记；已加载的
旧轨迹清除并要求重新规划、重新碰撞验证。全程无自动运动。

验证：`tools/test_tcp_pivot.py` PASS，`python3 -m compileall -q UR5/ur5_hmi` PASS，
`tools/smoke_hmi_weld_planning.py` 55/55 PASS，`tools/smoke_vtk_scene.py` 15/15 PASS。

## 166. HMI 读取与人工确认写入 UR5 当前 TCP（2026-10-10）

用户明确要求 TCP 标定页能够直接读取 UR5 并修改机器人当前 TCP，而不只是在 HMI
中显示软件 marker。本轮新增隔离的官方 URCL helper：

- C++：`UR5/ur5_hmi/native/robot_tcp_helper.cpp`
- 构建：`UR5/ur5_hmi/native/build_robot_tcp_helper.sh`
- 可执行文件：`UR5/ur5_hmi/bin/robot_tcp_helper`
- HMI 集成：`UR5/ur5_hmi/gui/tcp_robot_panel.py`

读取使用官方 RTDE output-only recipe，单个数据包同时取得 `actual_q`、`actual_qd`、
`actual_TCP_pose`、`tcp_offset`、`robot_mode`、`safety_mode`、`runtime_state`、
`payload` 和 `payload_cog`。GUI 显示坐标单位和 URDF/controller 对齐误差；读取不会写
任何参数。

真机写入不是自动动作。必须先满足：新鲜真机快照、URDF/tool0 对齐、标定写入质量门、
没有运动/规划任务，并且机器人 RUNNING/NORMAL、程序已由现场人员手动
停止、六轴静止。再次弹出人工确认后，helper 只发送一段由有限数值生成的受限
`sec hmi_set_tcp(): set_tcp(p[...]) end`，不发送关节/笛卡尔运动，不上电，不停止程序，
不重试。发送后至少三个递增 RTDE `tcp_offset` 样本（跨度至少 0.1 s）必须匹配目标，同时
复核关节漂移不超过 0.1°、负载/CoG 不变、RX/RY/RZ 原样保留；否则结果是未确认，不能
宣称成功，且必须人工重新读取。

写入结果保存到 `UR5/ur5_hmi/logs/tcp_changes/<request_id>/`，包含 `before.json`、
`request.json`、`result.json`。该接口设置的是 UR 当前激活 TCP 的运行时值，返回明确的
`persistence=runtime_only`；它不修改示教器安装配置、`ur5_tool_calibration.mat`、
ROS2 控制链路或 CoppeliaSim。要在重启后保留，需在示教器的工具/安装配置中人工保存。

当前 `/home/ttbstt/jixiebi/UR5/real_tcp_calibration.json` 的四组样本可复现且 rank=6、
condition≈56.8、最大拟合残差≈0.681 mm，但估计 XYZ 标准差约 `[4.810, 2.970,
6.038] mm`，因此真机写入按钮会 BLOCKED；第 5/6 组只是改善覆盖的可选采集，不是普通
四点保存的强制步骤。开发期间只执行了 helper self-test、纯数学测试和 HMI smoke test，
未连接或写入真机。

历史文件重新导入：TCP 页的“加载已保存标定”会优先做完整 URDF 指纹、样本复算和验证
记录检查。若旧文件仅因当前质量门（例如最大残差或缺少可复算的独立姿态）不通过，仍会
恢复 `pose_samples` 到表格供继续采集/重算，但状态明确为仅恢复、禁止真机写入；若 URDF
指纹或样本结构不匹配则拒绝导入。

## 168. 工件三点标定改为非阻断质量提示（2026-10-10）

工件标定继续使用 `P0 / PX / PY` 三点计算右手 `T_base_workpiece_real`，共线、零长度、
非有限数据和无效旋转矩阵仍禁止计算。`P_CHECK` 现在是建议验证点：没有 P_CHECK 或误差
较大时，三点几何有效的结果仍可保存为 `DRAFT`，不再使用固定 2 mm 作为保存门槛。

HMI 按 P_CHECK 残差给出质量提示：`≤2 mm` 绿色 GOOD、`2–5 mm` 黄色 WARNING、`>5 mm`
红色 POOR；其中 5–6 mm 通过人工确认后可申请应用，6–10 mm 仍可保存为 DRAFT 并做
3D 诊断预览但禁止真实规划，超过 10 mm 阻止预览；
没有 P_CHECK 显示 UNVERIFIED。保存 JSON 同时保留 P0/PX/PY/P_CHECK 原始位置、
坐标帧、时间戳、变换矩阵、残差、`quality_band` 和 `review_status`，旧文件先归档到
`UR5/workpiece_calibration_history/`。

保存不等于应用。选择 `REAL CALIBRATION` 时仍要求 P_CHECK 独立验证、应用精度（当前为
6 mm）、统一 base 坐标帧、有效右手旋转矩阵和人工确认；不满足条件就回到 SIMULATION，
保持规划/碰撞/ROS2 安全逻辑不变。没有自动移动机器人，也没有修改 MATLAB/CoppeliaSim
工件几何或 ACCP。

## 169. 工件标定数据恢复与真实位姿 3D 显示（2026-10-10）

工件标定页新增“加载已保存标定”和“显示到3D（预览）”两个独立操作。加载操作只恢复
`workpiece_pose_real.json` 中的 `T_base_workpiece_real`、P0/PX/PY/P_CHECK、质量等级、
残差、坐标帧和时间戳，不自动切换场景。显示操作复用既有真实工件应用门，要求几何有效、
P_CHECK/应用精度、坐标帧和旋转矩阵检查通过，并经人工确认后，使用同一变换更新 3D 工件、
焊缝、工件原点和已加载 TCP 路径；通过应用门的结果才切换 REAL CALIBRATION。不通过时，
经人工确认可以切换到 `REAL CALIBRATION PREVIEW` 仅作诊断显示，规划/执行仍记录 BLOCKED。

VTK 通过 nominal workpiece pose 的相对变换显示实测工件，Matplotlib fallback 使用同一
`T_base_workpiece_real`。不修改机器人/工件原始坐标、碰撞几何、ROS2 控制、规划算法或
安全门；没有发送机器人运动指令。

## 167. TCP 标定简化为普通四点流程（2026-10-10）

TCP 标定页不再把“RMS≤2 mm”当成保存硬门。普通四点流程采集 4 组不同姿态即可计算，
可选第 5/6 组增加冗余；求解仍是 `base→flange` FK 下的最小二乘位置标定，工具姿态
保持现有设置。

HMI 质量提示按 RMS 分级：`≤2 mm` 绿色 GOOD、`2–5 mm` 黄色 WARNING、`>5 mm`
红色 POOR。这只是初步质量提示，不是 UR 官方精度标准。数值求解有效（姿态跨度、
rank=6、有限条件数/位置）即可保存；JSON 增加 `review_status`（未独立验证时为
`DRAFT`）、`quality_band`、`quality_note` 和 `independent_verification_optional`。
独立验证按钮仍保留，但没有额外验证姿态时显示“未验证（可选）”，不再禁用保存或软件
应用。应用软件规划模型前仍弹出二次确认，明确提示未验证/黄色/红色结果不应直接用于
正式焊接；应用会使旧轨迹失效，要求重新规划与碰撞验证。
矩阵秩不足、condition>100 或 URDF 坐标/关节模型不匹配时，即使草稿可以保存，软件应用按钮也保持 BLOCKED。

保存新结果前，现有 `UR5/real_tcp_calibration.json` 会先复制到带时间戳的
`UR5/tcp_calibration_history/`，因此不会覆盖历史标定记录。

真机运行时 TCP 写入保持独立的严格门：`calibration_write_quality` 仍检查姿态覆盖、
条件数、最大残差和估计不确定度，另需新鲜 RTDE 快照、URDF/controller 对齐、机器人
静止且 RUNNING/NORMAL，并经过人工确认。放宽保存条件不会修改 ROS2、轨迹规划、安全门、
示教器参数，也不会自动发送机器人运动。

离线验证：`PYTHONPATH=UR5/ur5_hmi python3 -m unittest UR5/ur5_hmi/tools/test_tcp_frames.py UR5/ur5_hmi/tools/test_tcp_pivot.py`（9/9）；
`python3 -m compileall -q UR5/ur5_hmi`；HMI weld-planning smoke 55/55。未连接或运动真机。

## 170. HMI 在线规划空路径诊断与保护（2026-10-10）

修复 `RRT/benchmark/plan_hmi_request.m` 与
`RRT/benchmark/plan_hmi_weld_request.m` 的失败传播。此前自由空间子规划捕获
RRT 异常后返回 `zeros(0,6)`，外层仍继续计算 `localEndpointErrors`，导致真正根因被
`cannot check endpoints of an empty path` 覆盖。现在：

- RRT 前置检查记录 `qStart/qGoal` 连续关节值、关节限位、节点碰撞和自碰撞；
- planner 异常、无路径、`success=true` 但空路径、输出 MAT 缺失/空路径均生成明确的
  `PLAN_FAILED` report.json，并包含算法、终止原因、迭代/采样数和碰撞检查计数；
- 空路径在进入 endpoint validation 前立即终止，`localEndpointErrors` 只接受非空路径；
- weld wrapper 将 approach/exit 子报告、锚点检查和场景标定来源保存在最终报告中。

当前离线场景诊断结果：

- `weld_anchor_points.json:qHome/qPreWeld/qPostWeld` 在冻结 MATLAB 场景中均通过关节限位
  和节点碰撞检查，连续分支原样保留（未使用 `wrapToPi`）；
- MATLAB `sync_ur5_joint_states.m` 仍使用冻结的 `ur5_tool_calibration.mat` 和
  `scene_geometry.json`。`real_tcp_calibration.json`、
  `real_tcp_calibration_applied.json`、`workpiece_pose_real.json` 当前没有自动注入该场景；
- 冻结工件原点约 `[-375.000,-75.000,0.000] mm`，实测工件原点约
  `[-302.836,363.234,3.080] mm`，差约 `444.146 mm`，因此 qPreWeld 是
  `STALE_FOR_REAL_WORKPIECE_CALIBRATION`，不能作为真实标定工件的有效目标；
- 冻结 `tool0→WeldTCP` 仍为旧模型，真实 TCP JSON 是独立 flange→TCP 数据，二者尚未
  建立并验收统一坐标变换链。报告会明确标记 `real_*_loaded_into_scene=false`，不会假装
  已应用。

回归：

- 冻结仿真场景 `qHome→qPreWeld`、V1、seed `20260917`：PASS，规划约 30.09 s，
  首解约 1.01 s，11 个执行点，dense collision PASS；
- 当前真实标定文件场景：安全诊断发现工件位姿未注入且锚点过期，报告为 BLOCKED/不具备
  真实场景规划条件；未伪造路径、未放宽碰撞/关节限制、未发送 ROS2 或真机运动。

`plan_hmi_weld_request` 现在支持 `planning_scene=REAL_CALIBRATION` 的显式安全门；在
真实 TCP/工件变换尚未注入并验收前会直接输出 `PLAN_FAILED`，不会把冻结仿真场景冒充真实
标定场景。默认 HMI 请求仍使用 `FROZEN_SIMULATION`，报告会同时标出真实标定文件存在但
未加载的状态。
本轮持久化回归产物：

- 冻结场景日志/报告：`UR5/planning_runs/diagnostic_regression_simulation/matlab.log`、
  `report.json`、`trajectory.mat`。V1 规划器记录 `terminationReason=timeout_with_solution`，
  约 30.09 s、首解约 1.02 s、采样 132、节点 105、配置碰撞检查 7812、边检查 723；
  dense 检查 369 个配置和 10 条边，全部通过。
- 真实标定场景诊断日志/报告：
  `UR5/planning_runs/diagnostic_regression_real_calibration/matlab.log`、`report.json`、
  `request.json`。在规划器启动前因真实 TCP/工件变换尚未注入且 qPreWeld 与实测工件原点相差
  444.146 mm 被安全阻断，未执行 RRT、未生成轨迹，也未进入端点检查。
- 低时间预算故障回归已确认：planner 返回无路径时报告 `PLAN_FAILED`，包含
  `terminationReason`、迭代/采样数、配置和边碰撞检查计数；不再以空路径端点异常覆盖根因。

补充端点诊断：HMI 配置中的 Saved RealHome（`UR5/ur5_hmi/config/hmi_config.yaml`）为
`[283.431,-103.527,61.184,81.348,81.010,-4.295] deg`，在冻结场景内虽未越过关节限位，
但检测到 `selfCollision=true`，因此是无效起点；锚点文件中的 `qHome` 为
`[-58.142,-88.523,2.002,-89.761,2.978,-0.438] deg`，节点环境/自碰撞均通过。两者最大
周期关节差约 171.109 deg，不能互相替代。当前 HMI 仍会把 Saved RealHome 的无效性作为
规划前置失败返回，不会自动改写起点或跳过碰撞检查。
端点复核完整 MATLAB 日志保存在 `UR5/planning_runs/diagnostic_endpoint_states/matlab.log`。
Saved RealHome 组合的完整回归日志/报告保存在
`UR5/planning_runs/diagnostic_regression_saved_realhome/matlab.log` 和 `report.json`；
该测试在 RRT 入口前以 `PLAN_FAILED preflight: qStart is in collision` 阻断。

## 171. MATLAB/HMI TCP 一致性审计与标定版本门（2026-10-10）

新增只读审计脚本：

- MATLAB：`UR5/matlab仿真/check_hmi_tcp_pose_consistency.m`
- Python：`UR5/ur5_hmi/tools/check_tcp_consistency.py`
- 报告：`UR5/planning_runs/tcp_consistency_report.json`

审计使用同一份 `hmi_20261010_105205_168803/trajectory.mat` 的 23 个 qPath 节点，
并固定选取 10 个节点 `[1,3,6,8,11,13,16,18,21,23]`。MATLAB 场景使用
`sync_ur5_joint_states.m` 的冻结模型；HMI 使用 `real_robot_description_calibrated.urdf`
的只读 FK。MATLAB base 到 HMI URDF base 仅使用已记录的 Rz(180°) 转换。

结果：

- HMI tool0 与 MATLAB tool0（MATLAB 的 flange-equivalent）10 点位置误差 RMS
  `1.688 mm`、最大 `2.199 mm`；姿态最大 `0.216°`。
- WeldingGunMount 位置 RMS `1.678 mm`、最大 `2.185 mm`；姿态最大 `0.216°`。
- 冻结仿真 WeldTCP 节点与保存 `tcpPath` 的 10 点位置 RMS `2.458 mm`、最大
  `3.297 mm`；全 23 点 RMS `2.515 mm`、最大 `3.321 mm`（节点 14）。这不是
  TCP 插值误差，而是 HMI URDF 与 MATLAB universalUR5/标定 FK 的节点级模型差异。
- 线性关节插值的 FK 与 TCP 端点直线之间另有 RMS `10.946 mm`、最大 `38.245 mm`
  （第 3 条边中点）；因此必须把“节点模型差异”和“插值曲率”分开报告，不能把
  `3.321 mm` 归因于插值，也不能用直线 TCP 代替 FK 预演。
- 冻结 flange→TCP 偏移 `[375.912,19.489,66.681] mm`；当前已应用真实标定
  `[377.684,-31.712,7.112] mm`，切换位移 `78.569 mm`。真实标定文件声明的是
  URDF flange 坐标；MATLAB universalUR5 的 `tool0` 是 flange-equivalent，不能
  直接把这三个数当作 MATLAB tool0 偏移。通过 HMI flange→tool0 固定旋转后，
  仅按 URDF 的 flange→tool0 固定旋转转换后，等效 MATLAB tool0 偏移约为
  `[-31.712,7.112,377.684] mm`（跨 HMI URDF 与 MATLAB universalUR5 的基座 FK
  对齐审计值为 `[-31.320,7.312,377.372] mm`）；在该转换被正式接入、
  场景重新碰撞验证前，真实 TCP 规划保持 BLOCKED。
- `flange→WeldingGunMount` 与 `WeldingGunMount→WeldTCP` 相对变换全程不变，
  最大 Frobenius 漂移分别约 `8.1e-16`、`4.1e-16`；GunCollisionHull 的 4 个
  组件及 `wireCollision` 不随 TCP 标记切换而重建或移动。MATLAB 当前仍为 4 个
  枪体碰撞组件 + 1 个 18 mm × 1.2 mm 焊丝碰撞圆柱。

HMI 变更：

- `main_window.py` 为加载轨迹增加 TCP/工件/URDF 版本绑定门。缺少绑定的旧 MAT
  轨迹直接标记 `STALE`，仍可离线预演，但 FINAL EXECUTION CHECK 永远 BLOCKED；
  当前已应用 TCP、工件来源或 URDF 改变时会动态重新检查并失效旧路径。
- 在线规划请求携带只读 `calibration_binding`，MATLAB wrapper 将其写回 trajectory
  metadata；规划算法、碰撞门和控制链未改写。
- `run_hmi.sh` 在加载 ROS Humble 后显式使用 `/usr/bin/python3` 并检查 `import rclpy`，
  避免未 source 环境时的假性“缺少 rclpy”。当前验证：source 后 rclpy 导入 PASS；
  未发送任何机器人指令。

验证命令：

`python3 UR5/ur5_hmi/tools/check_tcp_consistency.py`

`python3 -m compileall -q UR5/ur5_hmi && bash -n UR5/ur5_hmi/run_hmi.sh`

MATLAB 审计通过 `matlab -batch` 运行，未修改 `ur5_tool_calibration.mat`、
`scene_geometry.json`、URDF、ROS2 控制器或真机状态。

## 172. 真实焊枪 J6 轴 ±90° 安装姿态预览（2026-10-10）

现场反馈为真实焊枪相对当前橙色 `GunCollisionHull` 绕 UR5 J6/法兰旋转轴逆时针
90°。本轮只做离线预览，没有写入 `scene_geometry.json`、
`ur5_tool_calibration.mat`、URDF 或 MATLAB 规划场景。

预览脚本与产物：

- `UR5/ur5_hmi/tools/preview_real_gun_mount_roll.py`
- `UR5/real_gun_mount_roll_preview.json`
- `UR5/real_gun_mount_roll_preview.png`

URDF 解析得到的 J6 轴在 `flange` 坐标系为：

```text
[1.0, 0.0, 0.0]
```

因此旋转中心是 URDF `flange` 原点，旋转轴不是焊枪凸包中心。MATLAB 刚体链的
候选实现为：

```text
T_tool0_mount' = inv(T_flange_tool0) * R_J6(±90°) *
                  T_flange_tool0 * T_tool0_mount
T_mount_tcp'   = inv(T_tool0_mount') * T_tool0_mount * T_mount_tcp
```

第二式用于保持已标定 TCP 在 flange 坐标系下不变，不重复施加 90° 到真实 TCP。
GunCollisionHull 的 4 个组件和焊丝作为同一父变换的子几何整体旋转，组件间相对
距离不改变。

数值预览（当前已应用真实 TCP `[377.684,-31.712,7.112] mm`）：

| 候选 | 固定 TCP 误差 | 当前喷嘴端→真实 TCP | 焊丝端点到真实 TCP最近距离 |
|---|---:|---:|---:|
| J6 +90° 右手旋转 | < `1e-11 mm` | `39.674 mm` | `36.812 mm` |
| J6 -90° 右手旋转 | < `1e-11 mm` | `99.315 mm` | `99.315 mm` |

两种候选的 4 个凸组件相对中心距离误差均低于 `6e-14 mm`；焊丝使用同一个父级
刚体旋转，未发生独立缩放或重新定位。

`+90°` 在符号上明显更接近现场描述；并且修正后的 `T_tool0_mount` 接近单位旋转，
而 `-90°` 候选使安装姿态接近 180° 反向。因此当前符号候选推荐为 `+90°`。

但是，`+90°` 后焊丝端点仍与真实 TCP 相差约 `36.8 mm`，并未满足“焊丝尖端与
真实 TCP 重合/保留既定工艺间隙”的几何条件。冻结 TCP 经 +90° 旋转后与当前
真实 TCP 仍有约 `37.136 mm` 差异。这说明除安装旋转符号外，当前真实 TCP 标定值、
焊枪头定义或实际焊丝伸出方向至少还有一项需要现场核对。

在该差异未解释前，禁止把任一候选写入真实 MATLAB 碰撞场景或 HMI 的规划/碰撞
显示链，也不允许据此解除真实 TCP/工件/碰撞质量门。HMI 后续允许有明确标注的
只读姿态预览，但不能把预览当成已验收规划几何。旧轨迹不做修改；正式写入后仍
必须使旧轨迹标记 `STALE` 并重新做碰撞与规划验证。

## 173. HMI 焊枪 +90° 显示修正（2026-10-10）

根据现场确认，HMI 的只读 3D 显示已将 `GunCollisionHull` 和 `wireCollision` 作为
同一安装刚体，绕 URDF `wrist_3_joint`（J6）在 `flange` 坐标系中的右手旋转轴应用
`+90°`。当前 URDF 解析的轴约为 `[1,0,0]`，旋转中心为 `flange` 原点；4 个凸
组件之间的相对几何关系保持不变。

本次修改只在 `UR5/ur5_hmi/gui/vtk_robot_view.py` 的显示变换中生效：

- 红色真实 TCP、蓝色预演 TCP 以及固定 flange→TCP 标定偏移未旋转、未改写；
- `scene_geometry.json`、`ur5_tool_calibration.mat`、真实 TCP 标定文件、URDF 和
  MATLAB 规划场景均未修改；
- MATLAB 规划场景仍保留之前约 `36.8 mm` 的焊丝端点/TCP 几何待核对项，因此该
  显示修正不能解除真实规划质量门，也不会自动使旧轨迹失效或发送运动指令；
- HMI 状态文字会标明“`HMI显示 J6 +90°；MATLAB场景未改写`”。

静态检查：`python3 -m compileall -q UR5/ur5_hmi`、`bash -n UR5/ur5_hmi/run_hmi.sh`
均通过。

## 174. 示教器当前 TCP 只读同步（2026-10-10）

HMI 现在默认开启“自动同步示教器 TCP（只读）”。`TcpRobotPanel` 每 5 秒调用现有
`UR5/ur5_hmi/native/robot_tcp_helper --read`，该 helper 使用独立输出 RTDE 会话读取
同一数据包中的 `tcp_offset`、`actual_TCP_pose`、`actual_q`、运行/安全状态；此模式
不会调用 `set_tcp`，不会发送运动，也不会修改 MATLAB 参数。读取失败显示为
`UNKNOWN`，不会用缓存数据冒充实时结果。检测到控制器 `tcp_offset` 变化时，沿用已有
轨迹失效门并要求重新读取、碰撞验证和规划。

红色 `TCP LIVE` 显示链已更新为优先使用 ROS2
`/tcp_pose_broadcaster/pose`。`RobotState.tcp_stamp` 记录话题接收时间；在 URDF
`base`（或明确的 `base_link→base`）坐标系且数据不超过 1.5 s 时，红色标记直接跟随
控制器当前生效 TCP。话题缺失、过期或坐标系不受支持时才回退到 HMI 已应用的
flange FK 偏移，并记录回退来源。蓝色预演 TCP 和红色实时 TCP 仍保持独立。

TCP 页新增对比信息：控制器当前 TCP、HMI 已应用 TCP（转换到控制器 tool0 表示）、
位置差、姿态差和更新时间。HMI 的标定记录是位置校准且姿态沿用控制器当前值，因而
姿态差显示为“0°（沿用控制器当前姿态）”，不伪造独立姿态标定结果。新增/修改文件：
`UR5/ur5_hmi/ros/state_monitor.py`、`ros/ros_node.py`、`gui/robot_view.py`、
`gui/vtk_robot_view.py`、`gui/tcp_robot_panel.py`、`gui/main_window.py` 和 README。

验证：`python3 -m unittest tools.test_tcp_frames`（9/9 PASS）、HMI 相关模块
`py_compile`/`compileall` 及 `run_hmi.sh` `bash -n` 均通过。未连接或修改真机，未
发送任何运动指令。

## 175. HMI 焊枪安装姿态离线调整（2026-10-10）

HMI 新增“焊枪安装调整”页。角度和 XYZ 平移均以 URDF `flange` 局部坐标表达，
旋转轴由 `wrist_3_joint` 的 J6 轴解析得到（当前为 `[1,0,0]`），旋转中心是
实际 `flange` 原点。默认显示现场确认的 `+90°`，支持 `±1°`、`±0.1°`、`±90°`
按钮和 `±50 mm` 范围内的局部平移微调。

`vtk_robot_view.py` 将同一个左乘刚体变换应用到 GunCollisionHull（4 个凸组件的
组合 actor）、焊枪显示几何和 wireCollision；红色控制器 LIVE TCP、关节角和固定
TCP 偏移不参与也不受该调整影响。页面显示焊丝物理末端到 LIVE TCP 的 ΔX/ΔY/ΔZ
及欧氏距离；如果没有新鲜控制器 TCP，则明确显示不可用。可隐藏机器人并用独立
焊枪视角检查重合。

“保存草稿”写入独立的 `UR5/real_gun_mount_calibration.json`，记录 flange 坐标系、
J6 轴、角度、平移、wire/TCP 诊断、时间和版本；“确认安装标定”需要人工确认，
仅保存 `CONFIRMED` 版本并使当前已加载轨迹标记 `STALE`，不会自动写 MATLAB、
示教器 TCP、ROS2 控制器或发送运动。预览和草稿不改变冻结的
`scene_geometry.json`、`ur5_tool_calibration.mat`。

新增 `UR5/matlab仿真/load_real_gun_mount_calibration.m`，只读校验并返回
flange-local 4×4 调整矩阵，后续真实规划脚本必须显式加载并重新完成碰撞/TCP
一致性验证；当前不会自动应用到规划场景。编译检查通过，未连接或移动真机。
轨迹版本绑定仅纳入 `CONFIRMED` 安装标定文件的 `gun_mount_sha256`；DRAFT 不会改变
规划依赖版本。确认后的新安装版本会使旧绑定轨迹 STALE。

## 176. 焊丝末端与控制器 TCP 离线诊断（2026-10-10）

本轮没有修改已应用焊枪安装标定、示教器 TCP、ROS2 控制链路或真机状态。
`UR5/ur5_hmi/tools/diagnose_gun_tcp_alignment.py` 使用已标定 URDF 计算 J6
`wrist_3_joint` 在 URDF flange 中的轴（当前 `[1,0,0]`），并按
`T_base_flange · T_adjust(flange) · T_flange_tcp · p_tip_tcp` 输出 0°、+90°、-90°
的焊丝端点位置和相对实时 TCP 的 XYZ/距离。报告在
`UR5/diagnostics/gun_tcp_alignment_latest.json`。

本次只读 RTDE helper 快照：q（ROS 顺序，rad）=`[4.905406952,-2.121999089,
1.836483479,1.041663289,1.340594410,-0.179558102]`；`actual_TCP_pose` base XYZ
=`[-0.285722171,0.365155362,0.208954990] m`；raw `tcp_offset`=`[-0.002745350,
0.016193651,0.359952098,-0.629535210,2.614397603,1.155686755]`（控制器
tool-output/flange 表示，不能直接当作 URDF flange XYZ）。

用 URDF `T_flange_tool0` 将 raw `tcp_offset` 平移转换到 flange 得到
`[359.952098,-2.745350,16.193651] mm`，与由 `actual_TCP_pose` 反算的点只差约
`0.043 mm`；这验证了控制器帧转换本身，而不是偏差来源。

按同一 q 将 `actual_TCP_pose` 反算到 URDF flange，控制器当前有效 TCP 点为
`[359.927843,-2.719047,16.218292] mm`。HMI 已应用记录仍是
`[377.683916,-31.712136,7.111947] mm`，两者差约 `35.197 mm`；因此当前红色
TCP若话题新鲜应显示控制器点，不应误认为本地应用标定点。该差异提示 TCP 参考
坐标/旧位置标定版本不一致，未获确认前不改写任何一方。

使用当前控制器 TCP 的焊丝端点诊断：

| J6 flange 右手旋转 | 焊丝端点 base (m) | ΔX/ΔY/ΔZ (mm) | 距离 (mm) |
|---|---|---:|---:|
| 0° | `[-0.326434, 0.392861, 0.236761]` | `[-40.712,+27.705,+27.806]` | `56.553` |
| +90° | `[-0.236541, 0.406368, 0.201800]` | `[+49.182,+41.213,-7.155]` | `64.564` |
| -90° | `[-0.340167, 0.320200, 0.173377]` | `[-54.444,-44.955,-35.578]` | `79.063` |

这些数值不是把 TCP 调到重合的建议；它们暴露了当前几何定义仍不一致。主要风险是：
焊丝头由 GunCollisionHull 最近顶点估计、未提供实测导电嘴中心；18 mm/1.2 mm来自
审计 MAT 的记录值而非本次现场实测；安装旋转与固定 TCP采用不同记录版本。不能使用
最近顶点直接代表导电嘴中心，也不能为视觉重合而平移/扭曲碰撞体。

VTK 视图新增黄色 `WIRE TIP` 标记和洋红色 `ΔTCP` 偏差线/数值。偏差线仅在新鲜
控制器 TCP 话题存在时显示；HMI FK 回退时明确隐藏实测偏差线。GunCollisionHull、
焊丝和焊枪仍由同一 flange 局部刚体变换驱动，预演 q 与真实 q/TCP状态隔离。

验证：`python3 -m compileall -q UR5/ur5_hmi`、`PYTHONPATH=UR5/ur5_hmi python3 -m unittest tools.test_tcp_frames`（9/9 PASS）。
后续需现场测量导电嘴中心/焊丝轴线并确认控制器 TCP与HMI记录的参考系，再由人工
确认是否写入独立真实安装版本；任何安装或 TCP 版本改变都必须使旧轨迹 `STALE` 并
重新进行碰撞和路径验证。无自动运动。

## 177. 示教器当前 TCP 采用流程与焊枪几何审计（2026-10-10）

TCP 页新增三个独立按钮：

1. “采用示教器当前 TCP（生成候选）”：只读使用同一 RTDE 数据包中的
   `actual_q`、`actual_TCP_pose`、`tcp_offset`。`calibration/tcp_frames.py` 的
   `controller_snapshot_to_flange_tcp()` 先用校准 URDF 求 `base→flange`，再计算
   `inv(T_base_flange)*T_base_controller_tcp`，得到完整的 URDF `T_flange_tcp`，
   避免 raw `tcp_offset` 的 tool-output 表示被误当成 flange XYZ，也避免重复施加
   tool0/flange 变换。
2. “确认并保存新 TCP 版本”：要求人工确认示教器 TCP 的参考点确实是焊丝/导电嘴尖端。
   版本保存到 `UR5/real_tcp_calibration_versions/tcp_from_pendant_*.json`，并写入
   独立的 `real_tcp_calibration_from_pendant_latest.json`。不覆盖
   `real_tcp_calibration_applied.json`、`real_tcp_calibration.json` 或冻结
   `ur5_tool_calibration.mat`。
3. “应用到真实规划模型”：第二次人工确认后写入独立
   `real_tcp_calibration_active.json`，HMI 和 MATLAB 的真实规划输入指向同一份
   完整 6D 变换；旧轨迹标记 `STALE`，要求重新碰撞/连续性/TCP 验证。无任何示教器
   写入和运动指令。冻结 SIMULATION 规划仍绑定冻结 MAT。

`load_real_tcp_calibration.m` 默认在存在且人工确认的 active 文件时读取它，并验证
`T_flange_tcp` 为刚体变换；没有 active 文件时保持旧 position-only 文件行为。
`check_hmi_tcp_pose_consistency.m` 同样优先 active 文件，但不改变冻结场景。

`tools/audit_gun_visual_geometry.py` 和
`diagnostics/gun_visual_geometry_audit.json` 的只读审计结果：

- `scene_geometry.json` 的 `WeldingGun_visual` 是 CoppeliaSim 对象路径，不是本地 STL/CAD；
  工程内没有找到可信可视 CAD 文件。
- `GunCollisionHull` 为 4 个凸组件、104 顶点、192 三角面，仅是碰撞代理；不能据此
  认定导电嘴中心。
- `WeldTCP` 有参考位姿，但尚未用实物确认等于导电嘴尖端。
- 18.0 mm 伸出量、1.2 mm 直径来自 `current_weld_geometry_audit.mat` 的历史记录，
  不是本次实测；真实送丝轴线、导电嘴中心、伸出长度均保持
  `PENDING_PHYSICAL_MEASUREMENT`，禁止自动猜测。

本轮验证：`python3 -m compileall -q UR5/ur5_hmi`、
`PYTHONPATH=UR5/ur5_hmi python3 -m unittest tools.test_tcp_frames`（9/9 PASS），
`audit_gun_visual_geometry.py` 正常生成报告。未发送 ROS2/RTDE 运动指令。

## 178. HMI 示教器 TCP 实时同步回退（2026-10-10）

现有 CB3 环境中 `/tcp_pose_broadcaster/pose` 可能存在订阅但没有发布者。TCP 页的
5 秒只读 RTDE 自动同步现在把同一数据包的 `actual_TCP_pose`、`actual_q` 单独缓存到
3D视图；ROS TCP 话题新鲜时仍优先，话题不可用时红色标记显示为
`TCP LIVE [RTDE]`，不会回退到旧的 HMI 规划 TCP。预演状态不会覆盖该缓存。

“UR5 当前偏移”表示控制器当前 active `tcp_offset`；“HMI 已应用 TCP（规划模型）”
表示上一次人工确认的规划版本，二者故意不自动覆盖。只有人工确认“采用示教器当前
TCP”并应用后，才更新真实规划模型并使旧轨迹失效。此修复只读，不写 TCP、不发送运动。

## 179. 机械臂控制算法与架构说明归档（2026-10-10）

根据本交接文档和当前工作区代码，在以下目录新增了面向后续开发、答辩和交接的说明文件：

```text
/home/ttbstt/jixiebi/UR5/机械臂控制算法/
```

文件：

```text
README.md
当前控制算法与系统架构.md
算法思路说明.md
代码索引与运行说明.md
安全边界与未完成事项.md
```

文档覆盖 ACCP 连续焊缝规划、RRT 自由空间规划、MATLAB/ROS2/CoppeliaSim 控制链、
HMI 状态与执行门、关键代码索引、运行命令，以及真实 TCP/工件/焊枪几何和真机执行限制。

本次只新增说明文档，没有复制或修改算法源代码，没有修改 ROS2、CoppeliaSim、HMI 控制逻辑、
TCP 标定和已有规划结果。

按用户后续要求，已在同一目录新增 `源码副本/`，复制 MATLAB、RRT、HMI、ROS2 控制节点和
场景几何参考文件。副本仅用于集中查看和归档，原始工程路径仍是唯一运行源；未复制 MAT/CSV
实验结果、视频、截图和日志，也未修改原始源文件。

## 180. 补充自由空间 RRT 算法思路（2026-10-10）

用户指出此前说明对焊缝 ACCP 描述较完整，但没有清楚解释自有自由空间 RRT。已在：

```text
UR5/机械臂控制算法/算法思路说明.md
UR5/机械臂控制算法/当前控制算法与系统架构.md
```

补充自由空间算法说明：

1. 状态为 UR5 六维关节角 `q∈R^6`，单位 rad；自由空间只规划
   `qHome→qPreWeld` 和 `qPostWeld→qHome`，不替换已经确定的焊缝段。
2. 基础 `RRTConnectJoint` 使用两棵树交替扩展，关节限位内随机采样、目标偏置 `0.20`、
   最近邻、最短角差 steer、约 `12°` 最大扩展步长和 CONNECT 连续追赶。
3. 节点及关节插值边均检查 UR5、焊枪、MATLAB 焊丝、工件和自碰撞；搜索边约 `2°`
   分辨率，首解后另用约 `0.5°` 高密度后验复核。
4. 自研 V1 `FastConnectAnytimeRRTStarJoint` 采用两阶段结构：Stage 1 先用 RRT-Connect
   找首条路径，Stage 2 保留双树执行 Bi-RRT* 风格 ChooseParent/Rewire，并使用缓存和
   best-so-far/时间检查点；可首解停止，也可在离线预算内继续改善路径长度。
5. `BiRRTStarJoint`、`ClassGoalLiteJoint`、`ClassifiedRRTStarJoint` 是统一接口下的
   对比实现；二维 `RRT_basic/RRT_star/...` 是演示/基准，不等于当前 UR5 六维控制器；
   Gao-AR-RRT* 是独立对比基线，不能与自研自由空间 V1 混称。

本次只更新说明文档和交接记录，没有修改任何 RRT 源代码、ACCP、ROS2、CoppeliaSim、
TCP 标定或规划结果。

## 181. 新焊枪 STL 替换（2026-10-10，暂存未激活）

**本节为第一轮历史状态；V-HACD 可用性、显示方式和焊丝前置条件已由第182节更正。**

用户提供的新模型位于 `焊枪模型/焊枪.STL`，已原样复制到：

```text
UR5/real_gun_geometry/welding_gun_source.STL
```

替换前备份位于：

```text
UR5/backups/gun_geometry_20261010_184312/
```

备份包含冻结 `scene_geometry.json`、`ur5_tool_calibration.mat`、当前真实 TCP/焊枪
安装记录和诊断报告；工程中没有独立旧焊枪 STL，旧可视/碰撞几何嵌在
`scene_geometry.json` 中，因此该文件已备份作为回滚源。

新 STL 只读审计结果保存于 `UR5/real_gun_geometry/stl_audit.json`：二进制 STL，
1,251,936 三角面，原始范围 `[38.13095,0.36275,7.91431]` 到
`[390.53653,196.85893,73.91431]`，尺寸约 `352.406×196.496×66.000`（按尺寸推断
单位为 mm；源文件未被缩放、居中或旋转），无 NaN/Inf 或退化三角面。网格预览显示
STL 已包含 L 形连接支架。

独立几何清单为 `UR5/real_gun_geometry/real_gun_geometry.json`。当前状态是
`STAGED_ALIGNMENT_REQUIRED`，碰撞层为 16 个确定性分段凸包审查代理，状态
`PENDING_VHACD_AND_ALIGNMENT`。本机没有可调用的 V-HACD 可执行文件/库，因此没有把
该代理冒充生产 V-HACD 结果。清单没有猜测法兰安装变换，也没有生成焊丝：新 STL
未携带法兰原点、导电嘴中心或送丝轴元数据。

HMI `vtk_robot_view.py` 和 MATLAB `sync_ur5_joint_states.m` 已增加独立清单读取门：
只有人工补齐并确认 `T_flange_source`、碰撞分解及焊丝实测数据后将清单明确改为
`ACTIVE`，新 STL 和独立半透明碰撞层才会替换旧显示/进入 REAL MATLAB 场景；当前
运行仍安全地显示冻结旧 GunCollisionHull，并标注“新焊枪 STL 已暂存”。UR5 本体、
J6/法兰坐标、真实 TCP、冻结 `scene_geometry.json`、`ur5_tool_calibration.mat` 和
原始 CoppeliaSim 场景均未修改，旧真实轨迹尚未被伪造为可执行。

要完成实际替换，还需要人工确认：源 STL 到 URDF flange 的刚体安装变换（含已确认的
J6 旋转符号）、生产级 V-HACD/等价碰撞模型、导电嘴中心/焊丝轴/伸出长度。确认前
不得把 `real_gun_geometry.json` 改成 ACTIVE，也不得运行旧真实轨迹。

## 182. 新焊枪实际 V-HACD、独立场景与离线一致性（2026-10-10）

本轮按用户要求复用 CoppeliaSim → compound hull → JSON → MATLAB collisionMesh，
没有修改 UR5/TCP/工件标定、ROS2 控制链或规划算法，没有发送真机运动。

### 对第181节的更正

- 本机 CoppeliaSim 4.10 有 `libsimConvex.so` / `simConvex.vhacd`；此前“不可用”
  判断不完整。旧16分段审查代理已归档为 `archive/slab_review_proxy_not_vhacd.json`。
- 导电嘴/焊丝精密测量不是新焊枪+支架外观和凸分解的前置条件。
  不可靠旧焊丝在 REAL 中明确 `DISABLED_UNVERIFIED`，不算作已验证碰撞几何。
- 未确认安装允许带明确标签的 HMI 离线预览，不允许进入正式真实规划或执行。

### 最终分解及文件

`UR5/real_gun_geometry/welding_gun_source.STL` 是用户原 STL 原样副本（含 L 支架），
1,251,936 三角面，SHA256 `437e28161ee9f50ad688f6f4239f432775dc6a9594f331a13110a89847442699`。
尺寸约352.406×196.496×66 mm；单位由尺寸推断，仍需人工确认。无自动居中或修改源顶点。

实际最终 V-HACD **直接输入完整源 STL**：26组件、853顶点、1602三角面，12.308987539 s。
参数 resolution=1000000, concavity=0.0025, plane/hull_downsampling=4,
alpha/beta=0.05, max_vertices=64, min_volume=0.00001, pca=false, voxels=true。
此前74,556面简化副本仅为实验中间数据，不是最终输入。

`vhacd_collision_export.json` 保存实际参数/源哈希及逐组件几何；组件组合前后坐标差
最大3.18e-9 m。复用 `UR5/export_scene_geometry.py:mesh_record`，不合并成一个总凸包。
初始耗时长主要是完整视觉网格经 Lua/远程接口大量传输；已改包围盒信息检查，
只传小型凸组件。重定向包围盒只改包围盒元数据，不改形状坐标/顶点。

独立场景 `new_gun_vhacd_source_preview.ttt`、`new_gun_source_unconfirmed.ttm`、
`ur5_real_gun_alignment_preview.ttt` 已保存。最后一份由原 ur5.ttt 只读加载后另存，
移除2个旧焊枪形状及1个场景脚本，停止状态；不得作为已验收执行场景。

### HMI / MATLAB

- `gui/vtk_robot_view.py` 载完整新 STL，半透明橙色碰撞层独立，26组件共享
  `T_base_flange*T_flange_source`。未确认默认 identity source-at-flange 仅为预览。
- `gui/main_window.py` 增加源 CAD 基准 R/P/Y；现有 J6 ±90°/1°/0.1° 和
  flange局部XYZ仅调整模型。草稿包含源哈希、总/基准变换；旧源记录拒绝套用。
  保存旧安装记录前留 `mount_versions/` 备份，不自动激活 MATLAB。
- `matlab仿真/apply_real_gun_geometry.m` 为显式 REAL 加载器：使用有明确 flange 的
  已标定 URDF，转换 `T_mount_source=inv(T_flange_mount)*T_flange_source`，
  每组件独立 `collisionMesh(vertices)`。视觉使用源网格的单纯单位转换副本。
  原 TCP 固定变换不改；不可靠旧焊丝碰撞/显示移除。
- `sync_ur5_joint_states.m` 默认保持冻结 SIMULATION，不能因真实 manifest 激活而
  隐式更换仿真基准。当前 manifest仍 `STAGED_ALIGNMENT_REQUIRED`，不能正式真实规划。

### 实际测试与未通过项

- HMI/MATLAB同10组关节角、26组件中心最大差2.83e-13 mm，RMS1.22e-13 mm；
  flange矩阵最大元素差4.44e-16，固定安装链 Frobenius误差7.18e-16。
- 0/+90/-90° **未确认预览变换**：Saved RealHome 自碰撞/工件碰撞均未检出。
  不能由此宣布真实安装 PASS 或认定旧碰撞根因已解决；真实工件记录仍 DRAFT。
- 2万源顶点抽样覆盖99.91%，最大归一化半空间偏差0.885761 mm。仍有局部欠覆盖，
  不是完整曲面距离/保守包络保证，必须审查后才能接受碰撞模型。
  源网格合计12条边界/非流形边；未自动修补。
- 3种角度HMI渲染、逐组件刚体一致性、LIVE状态不被预览覆盖、reload无重复演员通过。
- 100/125/150%安装控件 smoke通过（ROS worker启动mock，不是现场ROS连接测试）。
- Python compileall通过；TCP frame离线单元10/10通过。
- 7项文件与备份SHA256一致：scene_geometry.json、ur5_tool_calibration.mat、
  real_tcp_calibration.json、real_tcp_calibration_applied.json、workpiece_pose_real.json、
  real_robot_description_calibrated.urdf、ur5.ttt。

证据：`replacement_report.json`、`geometry_review.json`、`matlab_review_+0/+90/-90.json`、
`matlab_geometry_review.log`、`hmi_geometry_smoke.json`、`hmi_panel_smoke.json`、
`hmi_new_gun_preview_+0/+90/-90.png`。工具脚本位于 `UR5/ur5_hmi/tools/`。
备份 `UR5/backups/gun_geometry_20261010_184312/` 保留原场景/配置以及修改前代码。

下一步：用户在HMI依据真实法兰接口确定安装刚体变换与单位；审查局部凸包欠覆盖；
随后以已确认变换重做碰撞/TCP一致性及路径规划。旧真实轨迹全部保持STALE。
不要自动猜枪嘴、恢复旧焊丝或按TCP最短距离选安装变换；本轮NO ROBOT MOTION。

## 183. 新 STL 法兰盘圆面候选对齐（2026-10-12）

用户要求继续调整焊枪法兰与机械臂末端的虚拟对齐。没有连接真机、没有发送运动，
也没有修改 TCP/ROS2/冻结仿真场景。

从 `welding_gun_source.STL` 的最小源 X 平面提取了 248 个面点，得到候选安装圆面：

```text
plane x = 38.130954743 mm
center = [38.130954743, 166.858932495, 40.914321899] mm
normal = [-1, 0, 0]  (source frame)
```

候选变换将圆心映射到 URDF `flange` 原点，并保留现场已经确认的 J6 右手
`+90°` 旋转：

```text
T_flange_source_nominal = I with translation
  [-38.130955, -166.858932, -40.914322] mm

T_flange_source = Rx(+90°) * T_flange_source_nominal
```

数值复核：圆心残差 `5.67e-7 mm`，旋转正交误差 0，行列式 1。该结果只是依据
STL 几何的候选，不等于实物法兰确认；还需人工核对接触面、螺栓孔和安装厚度。

`UR5/real_gun_mount_calibration.json` 已更新为 `DRAFT`，HMI 重启后会显示该草稿；
旧记录已备份到 `UR5/real_gun_geometry/mount_versions/`
（`before_face_center_alignment_20261012_093442.json`）。MATLAB REAL manifest
仍为 `STAGED_ALIGNMENT_REQUIRED`，不会自动接收此草稿；旧轨迹继续 `STALE`。

证据文件：`UR5/real_gun_geometry/flange_face_alignment_review.json`。

## 184. 修正误选圆面：按实际安装盘四孔对齐（2026-10-12）

用户截图确认第183节候选明显悬空。根因是把源 STL 最小X处的焊枪本体圆面
误认作法兰安装面；原中心Y=166.859 mm，真正安装盘中心Y=31.859 mm，相差135 mm。
第183节的极小圆心残差仅验证所选点映射，不构成安装正确性证据，现已作废。

新安装面由网格平面边界识别：源X=49.23116 mm，中心
[49.23116,31.858934585,40.914309240] mm，盘外径63 mm，中心孔径15 mm。
四个直径6 mm螺栓孔组成PCD=50 mm方形；另两个轴向孔未误作为四个安装孔。
从当前HMI实际使用的URDF wrist3碰撞STL提取对应四孔，将顶点按
inv(T_base_flange)*T_base_wrist3*T_wrist3_mesh转换到flange进行匹配。
末端网格接触面X=0.025055182 mm；刚体配准接触面间隙为0，法向相反。
四孔中心RMS=0.031352333 mm，最大0.040254500 mm。机器人低面数网格有离散误差，
这些数字不表示实物装配精度。四孔存在90度对称，选取用户此前确认+90方向对应分支，
实际拟合角89.979187934度；没有用TCP距离优化安装。

当前T_flange_source为：
```text
[1,0,0,-0.04920610481801987;
 0,0.0003632390676729519,-0.9999999340286881,0.040933945631130085;
 0,0.999999934028688,0.00036323906767285474,-0.031885669187674034;
 0,0,0,1]
```
`real_gun_mount_calibration.json`更新为FOUR-HOLES-DRAFT-2，仍DRAFT、未应用MATLAB。
保留roll=90和零微调平移，微小角度修正在source nominal中；HMI重载读取完整矩阵。
上轮错误草稿备份：`real_gun_geometry/mount_versions/before_hole_alignment_20261012.json`。
源STL、机器人几何、TCP、工件、ROS2控制链均未改变，旧真实轨迹保持STALE。

新增工具：`ur5_hmi/tools/align_gun_mount_holes.py`、`preview_hole_aligned_gun.py`。
旧review_flange_face_alignment.py已改为四孔核对，禁止再次自动选最小X面。
证据：mount_hole_alignment.json、mount_hole_inspection.png、mount_holes_aligned.png、
hmi_hole_aligned_gun.png、hmi_hole_aligned_flange_closeup.png。
渲染核对26个凸组件共享视觉安装矩阵，MainWindow离线100/125/150%控件核对，
ROS worker未启动、LIVE状态未覆盖。本轮NO ROBOT MOTION。
## 185. REAL 场景同步与真实焊缝锚点（2026-10-12）

新增 `UR5/matlab仿真/build_real_planning_scene.m`，严格模式使用已应用真实 TCP、
HMI 保存的 `real_home`、真实工件位姿和 REAL 焊枪几何版本绑定；不改变 UR5、RRT
或冻结仿真文件。HMI 规划请求会核对场景实际绑定，防止请求继续携带冻结工件版本。

真实焊缝锚点入口为 `generate_real_weld_anchors.m`，调用现有 ACCP，不新增
规划算法。焊枪安装记录已有人工 `CONFIRMED` 标记，本轮已通过独立激活器将
`real_gun_geometry.json` 升级为 `ACTIVE`（26 个 CoppeliaSim V-HACD 组件，保留版本备份）。
真实工件仍为 `DRAFT`，且新 STL 没有可靠的真实焊丝轴线；
因此严格模式不发布锚点。诊断模式使用冻结轴线仅用于定位可达性，明确标记
`UNVERIFIED_FROZEN_DIAGNOSTIC_ONLY`，不能执行。

2026-10-12 离线诊断：RealHome 来自 `ur5_hmi/config/hmi_config.yaml`，而不是冻结
`weld_anchor_points.json`；qHome 自碰撞检查 PASS、关节范围 PASS。ACCP 实际
运行于 REAL 场景，在第 2 个焊点无碰撞自由状态（初始16点，IK调用296；细化后
21点、506调用、7轮，complete=0），所以旧 qPreWeld 未被复用，也没有生成新的可执行
路径。ACTIVE 焊枪复核输出在 `UR5/规划文件/real_scene_active_diagnostic_20261012/`，状态
`PLAN_FAILED`，旧结果全部 STALE。

此前一次相对路径错误导致的 `real_scene_sync_20261012` 结果仅是无效日志，不能作为
规划成功证据。所有变更均未发送机器人运动指令。

## 186. 算法名称统一：CBi-RRT* 与 ACCP（2026-10-12）

本轮仅统一算法名称，不改变算法逻辑、数学模型、函数接口、变量名、参数配置、碰撞
检测、实验数据或运行结果：

| 原名称 | 统一名称 | 作用 |
|---|---|---|
| `V1` / `V1 FastConnectAnytimeRRTStar` | `CBi-RRT*` | 约束感知双向 RRT*，用于自由空间段 |
| `A-ARC-IK+` | `ACCP` | 自适应构型连续规划，用于连续焊缝段 |

HMI 下拉框、在线规划状态、MATLAB 规划报告、运行输出、测试提示和相关说明已使用
`CBi-RRT*`、`ACCP`。完整英文释义为 `Constraint-aware Bidirectional RRT*` 与
`Adaptive Configuration Continuation Planning`。

为保持现有入口兼容，以下内容有意保留旧内部标识：

- MATLAB `switch` 分发键 `V1 FastConnectAnytimeRRTStar`；
- HMI 配置和请求中的兼容 `data` 值；
- 既有函数名、文件名（例如 `FastConnectAnytimeRRTStarJoint.m`、
  `online_v1_fast_connect_anytime_preweld.m`）；
- 历史实验报告、归档日志中的原始名称。

这些旧字符串只是兼容 ID 或历史证据，不代表存在第二套算法。新生成的用户可见报告
和界面显示统一使用新名称。Python 编译检查通过，HMI 离线 smoke test 的命名检查
通过；其余未通过项为既有的预演滑块对应性检查，与本次重命名无关。本轮未发送任何
真机运动指令。

## 187. 真实场景规划链路切换与安全阻塞（2026-10-12）

本轮新增 `UR5/matlab仿真/run_real_welding_pipeline.m`，作为真实焊接离线总入口：
它先建立严格 `REAL_CALIBRATION` 场景，再调用现有 ACCP 生成
`weld_anchor_points_real.json`，最后调用现有 CBi-RRT* 自由空间段和公共验证，成功时
在每个独立运行目录写入 `request.json`、`trajectory_real.mat`、`report.json`。
该入口不连接 ROS、不发送动作，也不会复制/覆盖冻结 `weld_anchor_points.json`、旧
`trajectory.mat` 或冻结 qPreWeld。

`build_real_planning_scene.m` 与 HMI 绑定现在同时记录四个可读版本：
`tcp_calibration_version`、`gun_geometry_version`、`workpiece_pose_version`、
`urdf_version`，并保留原始 SHA-256。真实 anchor 只有在这些版本完全匹配时才能被
HMI 加载；旧 anchor、缺少版本字段的诊断 anchor 和 `preview_only=true` 文件继续
保持 STALE/不可执行。

HMI 在线规划已取消对冻结仿真 anchor 的隐式回退：未显式选择
`REAL CALIBRATION` 时显示 `BLOCKED`，不会读取旧 qPreWeld。启动规划前增加只读真实
场景预检，要求真实 TCP 已应用、真实工件几何有效且质量不为 `INVALID`、真实焊枪
ACTIVE 且 V-HACD 已确认、真实焊丝轴线已验证。工件 `ACTIVE_WARNING` 允许规划并提示
精度一般，不再因为旧 `DRAFT` 字符串单独阻塞。

当前实际状态：

- `real_gun_geometry.json`：ACTIVE，CoppeliaSim V-HACD 26 组件；版本已绑定。
- `real_tcp_calibration_applied.json`：已应用，版本 `UR5-HMI-TCP-PIVOT-1`。
- `workpiece_pose_real.json`：当前 P_CHECK 残差 3.388 mm，按新规则为
  `quality_level=WARNING`、`ACTIVE_WARNING`；仍需真实焊丝轴线等其它场景门通过。
- 新 STL 的焊丝尖端/轴线仍为 `DISABLED_UNVERIFIED`，不能为 ACCP 猜测轴线。

因此本轮严格真实入口生成的证据目录为
`UR5/规划文件/real_pipeline_current_20261012/`，其 `report.json` 状态为
`PLAN_BLOCKED`，根因是 `Workpiece is DRAFT; independent review is required`；未生成
`trajectory_real.mat`，未提升任何旧诊断文件，未发送真机运动。待人工确认真实工件位姿、
补充可信焊丝几何后，重新运行该入口即可生成真实 anchor 和版本绑定的真实轨迹。

同日只读诊断目录 `UR5/规划文件/real_pipeline_diagnostic_20261012/` 使用了
`preview_only=true` 和冻结焊丝轴，仅用于观察真实场景可达性，不能提升为正式数据：
初始 16 点，细化到 21 点，ACCP 6.754 s，`complete=0`，第 4 点后无连续可行状态，
最终 `PLAN_FAILED`。该结果不是成功路径，也没有写入正式
`UR5/规划文件/weld_anchor_points_real.json`。

## 188. 工件/TCP 标定质量门分级（2026-10-12）

HMI 与 MATLAB 真实场景的质量判断已统一为数值分级，不再把工件 3.388 mm
残差自动标为 DRAFT：

| 对象 | 质量范围 | quality_level | planner-facing 状态 | 规划门 |
|---|---:|---|---|---|
| 工件 | `<5 mm` | `GOOD` | `ACTIVE` | 允许 |
| 工件 | `5–10 mm` | `WARNING` | `ACTIVE_WARNING` | 允许，提示精度一般 |
| 工件 | `>10 mm` | `INVALID` | `INVALID` | 禁止 |
| TCP | `<2 mm RMS` | `GOOD` | `ACTIVE` | 允许 |
| TCP | `2–5 mm RMS` | `WARNING` | `ACTIVE_WARNING` | 允许，提示精度一般 |
| TCP | `>5 mm RMS` | `INVALID` | `INVALID` | 禁止应用到规划模型 |

`quality_level` 已写入新的 HMI 保存 JSON；旧文件在加载时按残差兼容推导。
规划器只以 `INVALID`（以及缺少有效残差/几何无效等数据完整性问题）作为质量阻塞，
`ACTIVE_WARNING` 可加载、预演并规划。质量提示、残差计算、P_CHECK 和人工确认流程
仍保留；本轮没有修改 ROS2、运动控制、碰撞算法或发送真机运动指令。

## 189. 真实焊枪焊丝改为模型定义认证（2026-10-12，历史中间版本）

真实规划不再因为缺少物理导电嘴测量而阻塞。当前
`UR5/real_gun_geometry/real_gun_geometry.json` 的 `wire` 记录使用：

```text
status                    = MODEL_DEFINED_ACTIVE
definition                = MODEL_DEFINED
coordinate_frame          = STL_SOURCE
tip_source                = [0.39053653, 0.05583741, 0.04091431] m
axis_source               = [1, 0, 0]
length_m / diameter_m     = 0.018 / 0.0012
physical_measurement      = false
contact_tip_calibration   = false
```

该段记录使用的是早期冻结 TCP 端点和 18 mm 历史圆柱，仅保留为审计轨迹；随后
第 190/191 节已改为“CAD 枪尖到当前已应用真实 TCP”的 51.126 mm 参数化圆柱。

HMI 的 REAL preflight 接受 `MODEL_DEFINED_ACTIVE`（同时兼容历史
`CONFIRMED`/`VERIFIED`），并强制检查尖端/轴线为有限非零向量、长度和直径为
正数，以及焊枪碰撞分解和法兰安装变换状态为 `CONFIRMED`。MATLAB
`load_real_gun_geometry.m` 执行同样的完整性检查；
`apply_real_gun_geometry.m` 现在将同一固定点、方向、51.126 mm × 1.2 mm 圆柱加入
真实 `WeldingGunWire` visual/collision 子对象，不使用最近凸包顶点推断。

真实场景最终状态字段：

```text
scene.realWireAxisStatus       = MODEL_DEFINED_ACTIVE
scene.realWireDefinition       = MODEL_DEFINED
scene.wireCollisionEnabled     = true
scene.realWirePhysicalMeasurementRequired = false
```

已验证 `build_real_planning_scene(false)` 和离线几何检查可加载该模型；没有启动
ROS2、没有激活控制器、没有发送真机动作。
冻结仿真 `scene_geometry.json`、`ur5_tool_calibration.mat` 和原 CoppeliaSim 场景未覆盖。

## 190. 参数化 CAD 焊丝模型（2026-10-12）

真实焊丝不再依赖 physical measurement 或 contact-tip calibration。`wire` 使用
`MODEL_DEFINED_ACTIVE`，保存 CAD `tip_source`、同一安装链中的 `tcp_source`，并将
`axis_source = normalize(tcp_source - tip_source)`、`length_m = norm(tcp_source - tip_source)`
作为唯一轴线和长度定义；直径固定为 1.2 mm。按当前已应用真实 TCP，长度为约 51.126 mm。

MATLAB REAL 场景中的固定层级为：

```text
flange -> WeldingGunMount -> WeldingGunWire
```

`WeldingGunWire` 同时挂载圆柱 visual 和 collisionCylinder；`WeldTCP` 只保留 TCP
坐标系，不再承载焊丝碰撞体。HMI 使用相同 manifest 的 tip/axis/length/diameter
显示焊丝。ACCP 与 CBi-RRT* 只在 manifest、变换和碰撞组件检查通过后使用该模型；
TCP 标定、UR5 FK、机器人模型和 ROS2 控制链路未修改。

## 191. 焊丝固定子对象碰撞处理（2026-10-12）

模型定义焊丝现在使用 CAD 枪尖到当前已应用真实 TCP 的参数化圆柱（51.126 mm × 1.2 mm），并以
`flange → WeldingGunMount → WeldingGunWire` 作为真实 MATLAB 场景固定层级。由于
圆柱从枪尖安装面开始，焊丝与固定父对象的接触是装配界面而非障碍物碰撞；
`RRT/jointspace/deriveStructuralSelfCollisionSkips.m` 仅跳过
`WeldingGunMount ↔ WeldingGunWire` 这一对固定父子接口，焊丝与 UR5 其它连杆、
真实工件的碰撞检查仍启用。RealHome 离线检查结果为自碰撞 0、环境碰撞 0；未启动
ROS2、未激活控制器、未发送真机指令。

## 192. 取消错误的人工焊丝显示（2026-10-12）

后续覆盖本节：用户明确改为枪尖到红色 TCP 的最短直线，直径 1.2 mm，参与碰撞。
HMI REAL 使用直线 TubeFilter（封口，半径 0.6 mm），LIVE 端点只取新鲜控制器 TCP
球心；无控制器反馈不伪造端点。预演连接自身预演 TCP，不连接 LIVE 世界点。
恢复 manifest MODEL_DEFINED_ACTIVE，definition=OPERATOR_REQUESTED_STRAIGHT_CONNECTOR。
MATLAB 固定圆柱恢复参与碰撞，并新增终点与规划 TCP 1e-6 m 一致性检查；旧 TCP
不同则报告 WireTcpStale，不改变标定。当前已应用 TCP 对应长度 51.125812 mm。
不恢复黄色 WIRE TIP 球、ΔTCP 标签或偏差卡。未修改控制器、TCP、URDF 和冻结场景。
manifest 哈希变化使旧真实轨迹 STALE。NO ROBOT MOTION。

第 190/191 节枪尖到 TCP 的 51.126 mm 斜向圆柱已被用户否定，不再视为有效焊丝。
HMI 移除两点圆柱生成、最近凸包顶点估算、黄色 WIRE TIP 球、偏差线、ΔTCP 标签及偏差卡片。
保留原始焊枪 STL、现有法兰局部安装调整和独立红色 TCP LIVE；不改安装标定、TCP、FK 或 ROS2。
当前未找到独立已确认原始 wire 网格，新 STL 审计也未提供可靠枪嘴轴定义，不能声称已恢复真实焊丝。
manifest wire 改为 DISABLED_NO_ORIGINAL_WIRE；历史参数保留供追溯但不可用于规划。
这使 MATLAB 原有校验拒绝错误模型，并通过几何版本哈希使旧真实轨迹保持 STALE。
后续应接入原始 CAD 焊丝/枪嘴轴，再由人工调整安装刚体变换；不得重新生成 TCP 斜线。
NO ROBOT MOTION。

## 2026-10-12 焊丝缩短至 TCP 标记球面

按用户要求，简化直线焊丝不再延伸到红色 TCP 球心，只需接触 TCP 标记球面。
`real_gun_geometry.json` 的 `tcp_source` 是球心，新增 `tcp_marker_radius_m=0.025`
和 `wire_end_source`（枪尖→球心方向上距球心 25 mm 的接触点），`length_m` 更新为
0.026125812375786386 m，直径保持 0.0012 m。HMI 动态 LIVE/预演焊丝及 MATLAB
`collisionCylinder` 使用相同接触点规则；MATLAB 终点校验改为接触球面，不再要求
与 TCP 中心重合。TCP、UR5 FK、ROS2 控制、冻结仿真场景均未修改；manifest 版本变化
使旧真实轨迹保持 STALE。已通过 JSON、Python compileall 和 MATLAB
`build_real_planning_scene(false)` 离线检查，未发送机器人运动指令。

## 2026-10-12 ACCP 候选排序诊断与综合评分

新增 `UR5/matlab仿真/accp_score_candidates.m` 和离线对照入口
`test_accp_candidate_selection.m`。修改 adaptive core/feasibility、
`RRT/benchmark/plan_hmi_weld_request.m`、`generate_real_weld_anchors.m`、
HMI main_window/config/README。没有修改轴约束、关节限位、节点/边碰撞门、
机器人几何、标定文件或 ROS2。NO ROBOT MOTION。

原实现不是“第一个合格候选”，而是累计关节距离排序后裁剪 beam。
新增 candidate_scoring_mode 和四权重，J=关节距离rad + 轴误差rad +
限位成本 + 碰撞成本的加权和。默认 HMI/真实anchor开启，实验core默认关闭；
HMI YAML可关闭或调整权重，真实anchor第三参数selectionOptions可配置。
此功能改变 beam 保留排序，不改最终连接图硬门和既有图路径代价。

所有节点合格候选在裁剪前审计，输出q_deg、距上一点/保存RealHome的原始
关节距离（不wrap）、轴误差、六关节余量、真实checkCollision最小有效间隙、
评分分项、连接池成员/beam保留标记、有限下一点预测。去重保留seed_hit_count。
下一点预测独立求解器，RNG恢复；失败只表示有限搜索未找到，不能判不可达。
未知间隙为UNKNOWN并使用保守排序成本，不能伪造 clearance。每round独立JSON。

重要纠正：此前“Point1有5个候选，关轴后16点PASS”的日志实际上使用冻结
`sync_ur5_joint_states.m`，不是当前REAL，不足以证明真实TCP/几何场景排除问题。
本轮显式使用 `sync_ur5_real_scene.m`：seed20260917、16点、travel[-15,0,15]、
roll0:45:315、10随机种子、firstPointMaxStates2、beamWidth1、fallback关闭，
评分开/关真实场景均失败：264次数值IK、154姿态合格、方向拒绝103、
限位拒绝110、焊枪相关自碰撞拒绝18、工件碰撞拒绝33、机器人本体自碰撞拒绝0、最终合格0。
这些是不同阶段/类别统计，不可全部相加；continuity154为诊断而非Point1硬门。
没有有效首点，不能归因于候选裁剪造成后续断裂。

冻结场景同配置评分关/开均16点完成，节点和边碰撞PASS，最大关节跳变
约12.670°；插值TCP最大误差分别2.200/1.869mm，尚非执行精度验收通过。
完整报告在 `UR5/planning_runs/accp_candidate_selection_20261012/`，
日志 `accp_candidate_selection_20261012_verified.log`；未生成真实anchor或可执行
trajectory。旧轨迹依旧STALE。下一步应分析真实首点方向合格后的碰撞候选，
不可关闭正式轴约束或降低碰撞要求来宣称评分修复成功。

## 2026-10-12 REAL ACCP 轴约束灵敏度测试

新增 `axis_angle_tolerance_deg=[]` 可选参数，未指定仍保持原5°默认。
修改generate_actual_axis_ik_candidates、adaptive core和feasibility进行参数
接入；缓存key加入实际容差。新增 `test_accp_axis_sensitivity.m`，只生成离线
诊断，不调用HMI执行、ROS2或真实anchor生成。HMI参数未调整。

显式 `sync_ur5_real_scene.m`，真实TCP/工件/gun/wire均加载并断言，五档
calibration_binding完全一致。固定seed20260917、16点、travel[-15,0,15]、
roll0:45:315、10随机种子、beam1、firstPointMaxStates2、综合评分ON、fallback
OFF，除轴容差外设置一致。工件/焊枪/焊丝碰撞始终ON，正式轴约束没有关闭。

|容差°|IK姿态/限位成功|轴通过|碰撞通过(去重前)|最终候选(beam前去重)|16点|
|---|---|---|---|---|---|
|5|154|51|0|0|FAIL|
|10|154|51|0|0|FAIL|
|15|154|103|0|0|FAIL|
|20|154|154|0|0|FAIL|
|30|154|154|0|0|FAIL|

数值IK返回264，110被关节限位拒绝，表中154是通过限位和TCP姿态精度的解，
含重复seed命中。轴通过只是方向门，不含碰撞。焊接侧位置拒绝均0。
5/10°工件33、焊枪相关自碰撞18；15°工件71、焊枪相关自碰撞32；
20/30°工件118、焊枪相关自碰撞36；机器人本体自碰撞分类均0。全停Point1。
结论：当前搜索候选集内并非仅轴门过严；20°已让全部154候选过轴门，碰撞仍
全部拒绝。不得把30°设为正式默认，也不能降低碰撞门宣称成功。

完整JSON/CSV及每档候选：`UR5/planning_runs/accp_axis_sensitivity_20261012/`，
MATLAB日志 `UR5/planning_runs/accp_axis_sensitivity_20261012.log`。
未生成weld_anchor_points_real.json或执行轨迹，没有更改TCP/工件/gun标定，
旧轨迹保持STALE。NO ROBOT MOTION。

## 2026-10-12 Point1 全154候选碰撞细分

新增 `UR5/matlab仿真/diagnose_accp_point1_collision_pairs.m`。输入20°灵敏度
保存的core结果，重新build真实场景确认calibration_binding相同；不重新抽样IK。
所有154候选逐对诊断：原机器人矩阵保留本体和wire；26个机器人副本分别只保留
mount的一个真实凸组件，安装矩阵和原selfCollisionSkipPairs不变。环境和自碰撞
的逐对归因分别断言与完整原模型一致。没改变算法、安全门、碰撞阈值/排除表。

全部154候选有 `GunCollisionHull/component_04 ↔ WireCollision` 自碰撞。
36候选没有环境碰撞，118还有工件碰撞。该工具内部固定重叠不是轴约束问题，
也不是UR5本体撞枪。所有工件碰撞对均涉及焊枪凸组件。各工件面涉及候选数：
bottom15、wall_x0 84、wall_x1 22、wall_y1 6。统计是每对象每候选至多一次，
允许同一候选涉及多个对象，不可相加等同拒绝总数。

各GunCollisionHull组件候选数：01=15,02=0,03=5,04=154,05=30,06=10,
07=6,08=0,09=0,10=0,11=5,12=7,13=36,14=5,15=5,16=0,17=7,
18=12,19=5,20=0,21=12,22=12,23=0,24=0,25=12,26=12；WireCollision154。
源坐标凸半空间检查：wire.tip_source 位于component04内，最大外向半空间距离
-0.3589331238mm，解释其姿态无关固定重叠。该距离不是圆柱穿透深度。
本轮不自行移起点/改几何或增加碰撞skip；应先确认CAD枪嘴点与凸包关系。

记录每候选q/TCP、全环境与自碰撞pair、最近距离、原第一拒绝原因。
MATLAB checkCollision相交时distance=NaN，不提供penetration depth；保存
penetration_depth_mm=null和UNAVAILABLE_CHECKCOLLISION_API，不能声称已测得
穿透深度。最近10按碰撞pair数升序、非碰撞最小clearance降序、RealHome距离
升序筛选，再按原始连续q每关节0.5°去重；这是诊断启发式，不是最小穿透排序，
不代表可行。额外保留未去重raw top10，均不可执行。

输出 `UR5/planning_runs/accp_point1_collision_pairs_20261012/`：
collision_pairs.json、object_counts.csv、closest_10.json、fixed_attachment_overlap.json；
完整日志 accp_point1_collision_pairs_20261012_evidence.log，保留首次失败和前两次
诊断日志。未生成anchor/轨迹，不修改TCP/gun/workpiece/URDF，NO ROBOT MOTION。

## 2026-10-12 固定焊枪—焊丝内部装配碰撞排除

用户授权只排除工具内部结构碰撞。新增
`ignore_fixed_gun_wire_self_collision.m`，幂等地只加入精确body对
WeldingGunMount/WeldingGunWire。新STL的26个凸组件都挂在mount上，
无需删除任何凸组件或扩大为整个工具的全局碰撞忽略。
`build_real_planning_scene.m`与ACCP adaptive core调用同一helper，后者
原先重建skip列表会覆盖builder规则，所以必须同步接入。
冻结场景没有WeldingGunWire，此helper不产生变更。
原skip条目保持不变，wire与工件、wire与其他UR5 link、gun与工件、
gun与其他UR5 link均继续检查。无IK/ACCP约束/TCP/碰撞阈值变更。

新增 `test_fixed_gun_wire_collision_filter.m`，对保存的20°全部154候选，
过滤前后仅一个对改变，并逐候选assert除该对外的自碰撞距离矩阵完全一致。
工具内部碰撞154→0，工件碰撞118→118，gun对工件118→118，wire对工件
0→0；原环境free后的gun self拒绝36→0。通过全部碰撞门的raw候选0→36。
保持原20°实验选项重新冷运行，Point1去重连接池22，beam保留2；继续到
Point4中断，不能宣称16点PASS。20°仅诊断，正式默认仍5°。
结果目录 `UR5/planning_runs/accp_fixed_wire_filter_20261012/`，
filter_comparison.json、filtered_core.mat及逐点候选JSON；日志
accp_fixed_wire_filter_20261012.log。旧真实轨迹继续STALE，不生成anchor或
可执行轨迹，不发送机器人运动。NO ROBOT MOTION。

补充正式默认5°复测（axis_angle_tolerance_deg=[]，sideToleranceDeg=5）：
Point1去重候选11，16点core节点和边碰撞PASS，耗时14.514535s，插值TCP最大
误差1.482825mm（尚未满足原0.5mm执行验收）。未规划approach/exit或时间参数化。
20°扩大搜索池在beam1下停Point4，不代表5°也失败；两者仅诊断，不修改默认。
首次5°到达最终图后暴露qMat0的6×1输入格式错误，core加载后reshape为1×6，
validate六关节有限数；复测assert与原六个关节值逐值相同，没有IK/数学/约束变化。
default5_core.mat及accp_fixed_wire_filter_default5_20261012_verified.log留存。

## 2026-10-12 ACCP焊缝Cartesian精度细化

用户要求不改TCP、gun、碰撞模型、ACCP硬约束，解决16点joint-space插值误差。
新增 `UR5/matlab仿真/refine_accp_cartesian_path.m` 和
`interpolate_tcp_pose_slerp.m`。feasibility生产入口精度阶段改调用共享helper，
不改稀疏ACCP候选搜索/评分/约束/RRT。读取stateBank中原选中TCP目标：位置
linear、四元数shortest-arc SLERP，每个新增点调用原候选生成器，保持原
position/orientation/axis/side/limit/collision gate，环境与self collision ON。
原节点和连续joint branch严格保留，raw关节步长检查，禁止per-point wrapToPi。
每原段至少1新增点，失败可试到已有maxIntermediatePoints=4；插点失败或最终
误差>=0.5mm均complete=false。不改TCP、gun安装、26凸组件、wire或工件位姿。

修正原precision阶段的三个问题：NLERP不是真SLERP；3°误差采样可能只有
端点；插点只要IK/碰撞通过就accepted，complete没强制最终误差。现在每边
至少21采样含中点，大角度使用更密0.25°级别网格，复查所有碰撞、限位、
物理轴/焊接侧位置和TCP误差；插点预算/硬约束数值不变。移除被替换的私有
旧精度实现，避免两套接口。feasibility最大jump报告用raw diff，不隐去2pi跳变。

测试 `test_accp_cartesian_refinement.m` 从已PASS真实5°default5_core.mat加载，
与build_real_planning_scene当前calibrationBinding全字段一致。16→31点，插15点，
最大TCP0.423960675mm；独立每边101样本对原全段TCP目标复核一致；节点
0.000027737mm，姿态误差0.01776355°，raw最大joint step6.98628857°，原节点
逐值保留。节点/密集边碰撞与硬门PASS。法兰→TCP固定变换最大变化1.8082e-15。
密集原始误差1.667204379mm，旧core3°采样为1.482825mm，差别来自峰值采样。
精度阶段离线139.01s，不应宣称该阶段已符合30s规划预算。

输出 `UR5/planning_runs/accp_cartesian_refinement_20261012/`：
original_qPath.mat、refined_qPath.mat、report.json；均未时间参数化，不生成或
覆盖weld_anchor_points_real.json/正式trajectory，不发送ROS。完整日志
accp_cartesian_refinement_20261012.log。生产feasibility冷运行结果另存
production_round_00.mat、production_feasibility.mat和独立production日志。
HMI wrapper继续复用qPathExecution，并新增seam_cartesian_refinement报告。
max_tcp_error_mm改显示实际细化焊缝边误差；旧节点FK/tcpPath自一致指标（近0）
另存max_tcp_node_consistency_error_mm。该变更仅报告，不改变既有安全门。

额外审计 `audit_accp_export_interpolation.m`：现有export_hmi_trajectory自动生成
velocities=[0;diff(q)/diff(t)]，本机ROS humble trajectory.hpp明确positions+
velocities采用cubic。离线101样本/边Hermite审计几何chord偏离约0.3849mm，
但同时间相位位置差约2.3314mm；两者不是同一指标。保留现有时间参数化/
velocity/ROS执行链路，不声称达到0.5mm时间跟踪或真机精度。
export_interpolation_audit.json保存独立报告。旧轨迹继续STALE，NO ROBOT MOTION。

生产feasibility冷运行实测：同一seed/正式5°约束，ACCP 16点再次PASS，
精度输出31点、插15点、最大0.423961mm、Precision/Overall PASS；没有靠
加载旧qPath冒充冷规划。三次导出插值额外630个碰撞配置采样全部PASS，
101样本/边joint limit PASS，几何0.384872270mm，同时间相位2.331393534mm。
其独立日志accp_cartesian_export_audit_20261012.log。MATLAB checkcode新精度
helper/SLERP无告警，其余测试/既有入口仅非致命代码分析提示。

## 2026-10-12 Cartesian Refinement正式并入ACCP core

按用户要求，算法名称保持ACCP（Adaptive Configuration Continuation Planning）
＋CBi-RRT*。Cartesian refinement是ACCP内部精度增强阶段，不新增第三个算法。
流程：姿态硬约束生成→自适应多分支构型搜索/连续性评价→位置linear与姿态
SLERP细化/增量IK→连续性和密集碰撞验证。先粗搜索后细化，保持上一轮精度策略，
不改TCP/gun/碰撞模型/轴与焊接侧约束/安全门/ROS2/CBi-RRT*。

core默认内部执行精度阶段，qPathInitial保存稀疏链，qPath输出验证后的密集链；
tcpPath为N×3米的目标位置（robot root frame），tcpPosePath为4×4位姿cell。
qPathExecution与tcpTargetExecution保留兼容别名。stateBank/graph/seam仍是
稀疏构型搜索诊断，不能再按密集qPath的行号直接索引stateBank。旧的桥接实验
独立验证器已适配tcpTargetExecution，避免16层bank与31点路径越界。
configurationTcpMetrics/配置collision与jump字段保存粗搜索指标，tcpMetrics
改为对应密集轨迹；configurationComplete与整体complete分开。
精度失败不输出正式qPath/qPathExecution，仅cartesianRefinement保留失败候选。

feasibility协调器不再调用外部精度helper，二分/beam恢复依据configurationComplete，
继承core的全部最终结果。precisionStageInvocationCount记录单次内部调用，
耗时分别统计configurationSearchTimeSec与precisionAlgorithmTimeSec；协调器
求和避免重复计入精度时间。HMI wrapper与generate_real_weld_anchors改读主qPath，
报告点数以qPathInitial和主qPath区分，不生成或推广本次真实anchor。

历史Point1/轴敏感性/候选选择测试显式configurationOnlyDiagnostic=true，仅
测粗搜索，不额外改变既有诊断问题/约束。此模式禁止publish/animate，整体
complete=false、高精度输出为空；粗搜索的PASS只由configurationComplete表示。
轴关闭诊断同样隔离，不允许进入正式ACCP高精度结果。旧诊断/实验数据不改写。

新增 test_accp_embedded_precision.m 冷运行生产协调器，并检查core与外层同一
qPath、精度只调用一次、与上一轮16/31点结果一致、场景版本绑定一致、计时不
重复，以及仅配置诊断不产生正式PASS。输出目录
UR5/planning_runs/accp_embedded_precision_20261012/，日志同名.log。
实现变更文件：core、feasibility、refine_accp_cartesian_path、HMI MATLAB wrapper、
generate_real_weld_anchors、bridge验证器、相关诊断测试和README/本交接文档。
本机受限沙箱启动MATLAB立即退出，经批准仅用沙箱外MATLAB运行离线测试，
publish/animate始终false。NO ROBOT MOTION。

本次冷运行回归PASS：主qPath 31点，qPathInitial 16点，细化仅调用1次，最大
TCP路径误差0.423960675mm；与上一轮refined_qPath逐值差0，原16点粗搜索
逐值差0。节点/密集边碰撞PASS，场景版本绑定完全一致。搜索14.931544s，
内部精度129.862017s，算法合计144.793561s，无重复计时。仅配置诊断另冷运行
PASS，但整体complete=false、qPathExecution为空，正确禁止冒充正式结果。
报告report.json，粗/密原始数组original_qPath.mat与refined_qPath.mat、
embedded_round_00.mat、embedded_accp.mat和configuration_diagnostic.mat留存。
时间相位误差问题仍是上一节的独立限制，本轮未改时间参数化或控制器插值。

## 2026-10-12 时间参数化 TCP 误差诊断（只读）

用户要求只建立诊断，不修改 ACCP、TCP、碰撞模型、时间参数化或 ROS2 控制链路。
新增 `UR5/matlab仿真/verify_time_parameterized_trajectory.m`，以及回归脚本
`test_time_parameterized_trajectory_diagnostic.m`。诊断对最终轨迹逐时间采样点执行：

- 读取 `qPath`、`timeFromStart` 和可选 `velocities`；
- 用当前规划场景和既有 `getTransform(..., tcpBodyName)` 计算 TCP FK；
- expected Cartesian 路径必须由同长度 `tcpPosePath`/`tcpTargetExecution`（4×4
  位姿）或 `tcpPath`（N×3 米）明确提供；缺失时直接报错，不猜测；
- A：refined qPath 航点直接 FK；
- B：按 `joint_trajectory_controller` positions+velocities 规则重建 cubic Hermite
  时间轨迹（没有 velocities 时为线性关节插值），期望位置线性插值、姿态四元数
  SLERP；
- 输出最大/RMS TCP 位置误差、最大姿态误差、各自对应的采样时间，并保存完整
  CSV/MAT 样本和 JSON 摘要；始终标记 `robot_motion=NO`。

离线回归输入：`UR5/planning_runs/accp_embedded_precision_20261012/embedded_accp.mat`
的同一 31 点 ACCP refined 路径，速度字段按现有导出规则
`[zeros(1,6); diff(qPath)./diff(timeFromStart)]` 构造。结果：

| 对比 | 最大 TCP 误差 | RMS TCP 误差 | 最大姿态误差 | 对应时间 |
|---|---:|---:|---:|---:|
| A 直接 refined qPath FK | 0.000027737 mm | 0.000010216 mm | 0.000001207° | 位置：8.888006 s |
| B 时间参数化轨迹 FK | 2.331393534 mm | 0.376481260 mm | 2.518817449° | 位置：10.696707 s；姿态：8.394341 s |

诊断因此把误差来源定位为时间参数化后控制器等价的 joint-space cubic 插值，而非
ACCP refined 航点的 FK 误差。报告和逐采样文件保存在：

```text
UR5/planning_runs/accp_embedded_precision_20261012/time_parameterized_tcp_diagnostic.json
UR5/planning_runs/accp_embedded_precision_20261012/time_parameterized_tcp_diagnostic_samples.csv
UR5/planning_runs/accp_embedded_precision_20261012/time_parameterized_tcp_diagnostic_samples.mat
```

本轮没有优化插值、没有改变任何轨迹数据、没有发送 `FollowJointTrajectory`，旧轨迹
状态和所有安全门保持不变。NO ROBOT MOTION。

## 2026-10-12 ACCP 时间参数化方案离线比较（不接入真机）

新增 `UR5/matlab仿真/compare_accp_time_parameterizations.m`，在同一份31点
ACCP refined qPath、真实TCP、焊枪/工件场景上比较四种时间参数化。每种方案均按
至少3001个采样点评估轮廓误差（TCP到焊缝折线最短距离）、时间同步误差（同一时刻
期望位姿与实际FK）、姿态误差、关节速度/加速度、关节限位和密集碰撞。控制器依据
`ur5_driver_reconnect.log`及Humble `trajectory.hpp`核对：仅position为线性，
position+velocity为cubic/splines；因此当前导出速度字段对应基线 cubic。

最新报告：
`UR5/planning_runs/accp_embedded_precision_20261012/time_parameterization_comparison.json`

| 方案 | 轮廓最大 mm | 时间同步最大 mm | 姿态最大 deg | 碰撞/限位 |
|---|---:|---:|---:|---|
| baseline_cubic | 0.384872 | 2.331394 | 2.518817 | PASS/PASS |
| positions_only_linear | 0.361549 | 0.423961 | 0.017764 | PASS/PASS |
| shape_preserving_cubic | 1.979591 | 2.633922 | 1.245269 | PASS/PASS |
| retimed_cubic (alpha=0.60) | 0.408472 | 0.863335 | 2.425349 | PASS/PASS |

`positions_only_linear` 是本次离线指标最优候选，但其数值加速度峰值约
23.476 rad/s²，反映分段线性关节速度不连续，不能未经设备限值与控制器实测确认就
替换执行接口。四种方案均保持原31点、总时长14.235234 s和碰撞/限位安全检查；
时间同步指标与0.5 mm几何轮廓验收分开记录。

新增 `validate_accp_timed_output.m` 作为显式输出门：只有调用者配置有限的
`timeSyncAcceptanceMm`，且轮廓、时间同步、碰撞和限位均通过时才返回
`execution_ready=true`；未配置时间门或任何失败均返回明确
`TIMED_OUTPUT_GATE_FAILED`，不会标记可执行。`test_validate_accp_timed_output.m`
离线回归 PASS。比较器和门控均为 NO ROBOT MOTION，未修改真机执行链路。

## 2026-10-13 Adaptive Time Scaling（仅离线）

在不改变 ACCP 生成的31个 q 节点、不改变 TCP/IK/焊枪/碰撞模型的前提下，
比较器新增 `adaptive_time_scaled_positions_only`。它根据每段最大 Δq、TCP步长、
姿态变化和TCP方向变化（曲率特征）生成初始时长，再用独立段时长优化平滑相邻
关节速度。只改变节点时间，不改变任何采样关节配置。

最新结果（3001采样点）：

| 方案 | 总时间 s | 最大速度 rad/s | 最大加速度（密集节点估计） rad/s² | 轮廓最大 mm | 同步最大 mm | 姿态最大 deg | 碰撞 |
|---|---:|---:|---:|---:|---:|---:|---|
| positions-only 原始 | 14.235234 | 0.087266 | 23.475957 | 0.361549 | 0.423961 | 0.017764 | PASS |
| adaptive positions-only | 21.246303 | 0.070576 | 5.358140 | 0.361549 | 0.423961 | 0.017764 | PASS |
| 当前 Hermite | 14.235234 | 0.140154 | 0.994265 | 0.384872 | 2.331394 | 2.518817 | PASS |

自适应方案的段间加速度估计为 `0.053581 rad/s²`，低于默认15 deg/s²
限制；密集节点的 `5.358140 rad/s²` 是 positions-only 分段线性插值在节点速度
不连续处的过渡估计，单独记录，不能与连续段物理加速度混淆。总时间21.246303 s，
低于50 s；TCP轮廓和时间同步误差均保持原值。碰撞验证复用完全相同的3001个关节
配置，不是跳过检查。尚未自动替换真机轨迹。
## 2026-10-13 ACCP 内置 Adaptive Time Scaling（离线集成完成）

`replan_weld_actual_axis_adaptive_core.m` 现在把 ACCP 固定为三段内部流程：

```text
Continuous IK -> Cartesian Refinement -> Adaptive Time Scaling
```

Cartesian refinement 生成的 31 个 `qPath` 节点保持不变；第三段只优化节点间隔，
最终结果同时输出 `qPath`、`tcpPath/tcpPosePath`、`timeFromStart`、`velocities` 和
`accelerations`，并通过 `result.timeParameterization` 暴露阶段结果。硬门要求 TCP
轮廓 `<0.5 mm`、同时间 TCP 同步误差 `<0.5 mm`、碰撞/限位通过、分段速度/加速度
不超过配置上限且总时长不超过 50 s。碰撞结果由同一份 ACCP refinement dense
node/edge validation 传入，未跳过碰撞检查。

时间输出仍采用 positions-only 语义，现有 ROS2 trajectory 导出和控制器插值接口
不变；返回速度和加速度用于 ACCP 验证与记录，不会自动发送运动命令。分段线性节点
折点的密集有限差分峰值另存为 `finiteDifferenceMaxAccelerationRadPerSec2`，
不与连续段物理加速度混淆，也不用于通过/拒绝门。

`test_accp_adaptive_time_scaling_integration.m` 使用已冻结的 31 点 refined 结果回归
通过：总时长 `21.246303 s`，最大分段速度 `0.070576 rad/s`，最大分段加速度
`0.053581 rad/s²`，TCP 轮廓最大 `0.361549 mm`，时间同步最大 `0.423961 mm`，
姿态最大 `0.017764°`，碰撞 PASS、关节限位 PASS，`executionReady=1`。本轮未发送
任何机器人运动指令；旧轨迹和 ROS2 执行链路未修改。

同轮修复了 `replan_weld_actual_axis_adaptive_core.m` 的冷启动诊断子函数作用域：
使用已保存状态图进行离线回归时，Point-1 诊断结构现在能正确初始化，不会在进入
Cartesian Refinement 前因 MATLAB 子函数不可见而中止；这不改变正常冷启动搜索或任何
规划约束。

## 2026-10-13 模型枪口 → 已应用真实 TCP 焊丝（当前有效定义）

本节取代历史 190–192 节及“焊丝缩短至 TCP 标记球面”的 REAL 焊丝定义。
用户当前授权采用 CAD 模型参考点与已应用 TCP 构造直径 1.2 mm 的直线圆柱，
不要求物理导电嘴测量、独立 endpoint 文件或黄色 WIRE TIP 球。

- manifest：`UR5/real_gun_geometry/real_gun_geometry.json`，顶层 `wire_axis_status`
  与 `wire.status` 均为 `MODEL_AXIS_TO_TCP_ACTIVE`；版本
  `real-gun-1791783961397392571-wire-axis-to-tcp-20261013`，`requires_replan=true`。
- 模型枪口参考点沿用现有 CAD 记录 `[0.39053653, 0.05583741, 0.04091431] m`
  （STL_SOURCE），显式保存为 `gun_tip_model`；不是本轮重新测量或最近凸包顶点推断。
- 从 `real_tcp_calibration_applied.json` 读取 URDF flange 坐标中的已应用 TCP，
  经 `inv(T_flange_source)` 转到 STL_SOURCE。wire_start=gun_tip_model，wire_end=TCP，
  axis=normalize(end-start)，length=norm(end-start)，diameter=0.0012 m。
- Python `calibration/model_axis_wire.py` 与 MATLAB `model_axis_wire_to_tcp.m`
  运行时只读重建几何，忽略过期球面端点/半径字段；保留有限值、安装刚体变换、
  TCP坐标系及应用状态检查。焊丝自身检查正长度、正直径、单位轴。
- HMI 黄色焊丝与 MATLAB collisionCylinder 是同一固定安装圆柱，父对象仍是
  WeldingGunMount。无黄色端点球；红色 LIVE TCP 继续独立取控制器数据，不被模型
  定义/预演改写。控制器 TCP 与 applied 记录不一致时不自动伸缩焊丝伪装重合。
- 修复真实场景装配适配器的一处旧副本使用：apply_real_gun_geometry 返回加入焊丝的
  scene.robot，但 build_real_planning_scene 原来对未加入焊丝的局部 robot 建过滤列表。
  现改为对 scene.robot 调用原 deriveStructuralSelfCollisionSkips 和
  ignore_fixed_gun_wire_self_collision；规则本身未改，仅沿用枪体—焊丝固定装配接口
  过滤，保留 wire/gun 对工件、wire 对其他 UR5 link 的碰撞检查。
- 更新 HMI preflight、REAL anchor/pipeline 与规划 wrapper 接受新的状态名，
  不改 ACCP/CBi-RRT* 的算法或硬约束，不生成 anchor 或规划轨迹。

离线验收：Python 3 项正常/无参考点/零长度/错误坐标系回归通过，compileall 通过；
MATLAB REAL 场景加载 26 个 V-HACD 组件与固定焊丝，长度 `51.125812375786 mm`。
10 姿态 cylinder 端点与规划 TCP 最大偏差 `1.1444e-13 mm`，HMI/MATLAB 端点最大差
`3.3307e-13 mm`。独立 VTK smoke 确认黄色端点未生成、LIVE 与 preview 隔离、
控制器参考偏移不会拉伸焊丝。Saved RealHome 自碰撞/环境碰撞均为 false。
这不是新轨迹的密集碰撞验收；旧 anchor/trajectory 仍因 manifest hash/version 变化 STALE。

报告/截图/日志：`UR5/diagnostics/model_axis_wire_to_tcp_20261013.json`、
`model_axis_wire_hmi_offline_20261013.png`、`model_axis_wire_matlab_20261013.log`、
`model_axis_wire_hmi_20261013.log`。截图使用合成控制器测试数据，明确标注 TCP TEST，
不是已连接真机现场截图。旧 manifest 回滚备份：
`UR5/real_gun_geometry/wire_versions/pre_model_axis_to_tcp_20261013.json`。
TCP applied JSON、URDF、scene_geometry.json、ur5_tool_calibration.mat 哈希前后相同。
没有修改TCP标定/控制器参数/算法；NO ROBOT MOTION。

补充 GUI 纯方法回归（未创建 ROS 节点）：当前 REAL 标定预检 reasons=[]，
焊丝定义门已通过；人工构造旧 gun hash/version 的轨迹仍被版本检查拒绝。
这不表示 Final Execution Check 已就绪，也不表示 ACCP 全路径本轮重新规划通过。

## 2026-10-13 缩小红色 LIVE TCP 标记（仅显示）

按用户要求，VTK 红色 LIVE TCP 球半径由 25 mm 缩为 3 mm（直径 6 mm），
球心仍是原实时 TCP 坐标。仅修改 `gui/vtk_robot_view.py` 的显示常量，
不改变 TCP 标定、焊丝端点/长度、碰撞几何或相机/坐标变换。NO ROBOT MOTION。

## 2026-10-13 LIVE TCP / 规划 TCP 来源纠正

用户明确控制器当前 RTDE TCP 就是规划 TCP，不能拿旧四点标定作为另一个规划真值。
截图中黄色线断开原因：旧代码由 10 月 9 日 `real_tcp_calibration_applied.json`
生成固定焊丝，红球来自当前 `actual_TCP_pose`。此前离线两端重合测试只证明了与旧文件
一致，并不证明与现场当前控制器一致；本节纠正该限制。

HMI/ MATLAB 焊丝解析现与 build_real_planning_scene 同步：优先读取已经人工确认的
`real_tcp_calibration_active.json`（控制器当前 TCP 的 URDF flange 6D 版本镜像），
不再在 active 存在时使用旧 applied 文件。active 无效不得回退旧文件。
在线 REAL 预检要求新鲜 RTDE 快照、控制器版本 active 文件、保存的
controller_tcp_offset 与当前 tcp_offset 一致；没有镜像或值变化时 BLOCKED/STALE，
不得把旧四点标定当作当前真机 TCP。确认入口仍是“采用示教器当前 TCP（生成候选）”
→“确认并保存新 TCP 版本”→“应用到真实规划模型”，不需要重复做四点标定。
未替用户点人工确认，未写示教器参数，未伪造质量残差、未放宽质量/碰撞/运动门。
当前磁盘尚无 active 控制器版本；实际采用/精度验收仍须人工完成，不能声称规划已恢复。

黄色显示焊丝现直接连接当前红色 LIVE TCP；预演时只连接预演 TCP，LIVE 快照不变。
单独橙色半透明层保留已确认的固定规划碰撞圆柱，由 Collision Geometry 开关控制；
显示线不回写 MATLAB 碰撞数据。Wire 开关和 tooltip 明确两层含义。
实时 TCP 快照 setter 同时刷新 marker/焊丝，避免异步更新断开。
LIVE/规划端点不同时，状态行及 tooltip 报差异，不能把“显示连上”当作规划已同步。

新增离线35.197 mm合成偏差回归：黄色显示端点与红色TCP完全重合，固定碰撞圆柱不变；
10姿态固定碰撞体与MATLAB最大差3.3307e-13 mm，预演隔离/无黄色端点球PASS。
截图/报告：`UR5/diagnostics/live_wire_tcp_display_offline_20261013.png` 与同名 JSON，
明确是合成测试不是现场RTDE值。Python四项焊丝测试、compileall通过（含active优先与
active无效禁止回退测试）。没有重新运行ACCP或发送运动。NO ROBOT MOTION。

## 2026-10-13 控制器 TCP 统一为 REAL 规划唯一来源

用户确认：以后 TCP 一律以示教器/控制器当前生效值为准，并自动同步到规划版本。
`UR5/ur5_hmi/calibration/controller_tcp_sync.py` 实现只读 RTDE 镜像：使用同一数据包
的 `tcp_offset`、`actual_TCP_pose`、`actual_q`，核对 URDF 坐标后按固定
`flange <- tool0` 变换生成完整 6D `T_flange_tcp`。它只写本地
`UR5/real_tcp_calibration_active.json` 及版本归档，不调用 `--request`，不写示教器，
不发送运动；重复读取相同 TCP 不会因时间戳或关节变化而生成新版本。

当前现场只读核对结果：控制器 `tcp_offset` 为
`[0.0035786184, 0.0124803772, 0.3710356277, -0.5084952856, 2.7116703942, 1.1228175432]`
（m/rad）；URDF/控制器工具基准误差 `0.038003 mm / 0.006708°`，通过既有
`3 mm / 0.5°` 坐标门。当前镜像版本为
`CONTROLLER-TCP-f2d4b6d9a94d20e89a5001a6d0ad435bf4e61fc4ec243f2457e5eb7186ac24b0`，
状态 `ACTIVE / CONTROLLER_DEFINED`。`CONTROLLER_DEFINED` 只代表参数来源和坐标链
一致，不伪造四点标定 RMS，也不代表导电嘴实物精度。

HMI 规划/显示在运动或规划期间延迟切换；控制器 TCP 变化后旧轨迹自动 `STALE`，
必须重新生成焊点并完成焊枪、焊丝、工件、自碰撞和轨迹验证。MATLAB REAL 场景的
`build_real_planning_scene.m`、`load_real_tcp_calibration.m`、
`model_axis_wire_to_tcp.m` 均只接受 ACTIVE 控制器镜像；历史
`real_tcp_calibration_applied.json`、冻结 `ur5_tool_calibration.mat` 不被覆盖，
仅可显式用于离线诊断。真实焊丝继续由 ACTIVE 焊枪模型枪口参考点到控制器 TCP
生成，直径 1.2 mm，并与 MATLAB 使用同一端点定义。

离线验证：`tools/test_controller_tcp_sync.py` 11 项通过（含重启恢复完整变换）；
`tools/test_model_axis_wire.py` 5 项通过；Python `compileall` 通过。
MATLAB `matlab仿真/test_controller_tcp_mirror.m` 的已有离线报告为 PASS：10 姿态，
固定变换最大差 6.9422e-16，焊丝端点最大差 7.7579e-15 mm，9 类坏输入被阻止，
缺少 ACTIVE 不回退历史标定；报告 `UR5/diagnostics/controller_tcp_mirror_matlab_offline_20261013.json`。
这是合成数据一致性验证，不是现场焊接精度验收。后续 MATLAB 重跑因审批额度耗尽未执行。
一次未隔离完整 GUI 构建尝试触发 ROS 线程初始化，但日志目录权限错误导致初始化失败；
上述正式单元测试使用替身，不连接 ROS 或机器人。未运行 ACCP/CBi-RRT*，未发送运动。
真实焊点文件 `UR5/规划文件/weld_anchor_points_real.json` 仍缺失；TCP 来源已统一不等于
真实规划链路全部恢复，需要另行重新生成版本绑定的真实焊点，禁止替换为冻结仿真目标。

## 2026-10-13 新 A焊枪real.STL 视觉替换及真实 V-HACD 重建

用户源：`焊枪模型/A焊枪real.STL`，SHA256
`06f38186eddd712eb239f069878da1d9d307171dc693df3a83c8b086b8d2e21c`。
397,910 三角面，原始尺寸 66.000×191.493×366.135（尺寸判断为 mm，待人工核对单位）。
非有限/退化三角面均为0；VTK拓扑检查边界/非流形边共20，不声称网格完全封闭。
原始坐标、原点、源文件不改写；仅另生成 mm→m 的视觉副本。

备份：`UR5/backups/before_Areal_gun_20261013T065647Z/`，旧模型目录完整副本，
另含旧安装、ACTIVE TCP、历史TCP及工件记录。新独立构建包：
`UR5/real_gun_geometry_candidates/Areal_06f38186/`；已安装到 HMI 固定读取的
`UR5/real_gun_geometry/`，状态 **STAGED_ALIGNMENT_REQUIRED**，不是自动 ACTIVE。
旧真实轨迹版本绑定因源/manifest/安装改变失效，保持 STALE，不改写旧轨迹。

复用 `decompose_real_gun_coppelia.py`（新增 --folder，不变分解参数），独立停止的
CoppeliaSim实例23060/23061，仅源STL导入，无ROS/仿真运行/机器人连接。
实际 simConvex.vhacd：10组件、429顶点、818三角面，耗时4.373589502秒；
分组前后组件坐标差3.6623e-9 m。参数同第182节，直接输入完整新源STL。
保存独立 `Areal_flange_aligned_preview.ttt`，未覆盖原ur5.ttt，处理后实例已关闭。
20,000顶点抽样在凸包并集内比例0.99725，最大半空间覆盖偏差0.750874mm；
这是近似包络审查指标，不是精确距离/覆盖验收，不擅自增大碰撞阈值或宣称PASS。

新CAD安装面为source Z=0，中心[33.330000913,31.814999795,0]mm，外径63mm，
四个6mm孔的节圆约49.599mm，与URDF网格孔位并非完全相同。
按圆边界+四孔SVD刚体拟合，新source X/Y/Z对应旧-Z/Y/X，延续先前确认的+90
物理方向分支，不按TCP最短距离选安装。孔心RMS0.194098mm，最大0.220724mm，
面间隙0、法向相反；仅为网格拟合证据，必须人工复核。
`real_gun_mount_calibration.json`更新为新source绑定DRAFT，增量roll=0（总安装
姿态已包含方向变换，不能额外再加90）。视觉与每个凸组件使用同一个T_flange_source。

新源没有复用旧gun_tip_model/wire datum，焊丝 **DISABLED_UNVERIFIED**。
因此REAL正式规划仍BLOCKED；不能因枪体重建成功而假定旧焊丝位置准确。
示教器ACTIVE TCP、历史TCP、URDF、workpiece_pose_real、冻结scene_geometry及
ur5_tool_calibration.mat均未修改，相关哈希/与备份比较已核对。

新增工具：`prepare_replacement_gun.py`、`install_replacement_gun_preview.py`、
`verify_replacement_gun.py`；`review_real_gun_geometry.py`新增 --folder。
MATLAB新增只读审查 `test_replacement_gun_geometry.m`，不改规划/控制算法。
10姿态HMI/MATLAB凸组件中心最大差3.3422e-13mm，变换最大差4.4409e-16。
Saved RealHome枪体自碰撞=false，枪体/机器人与真实工件碰撞=false；明确未包括焊丝，
不构成完整焊接轨迹验收。报告/日志在当前几何包：`Areal_hmi_report.json`、
`Areal_matlab_report.json`、`Areal_matlab.log`；截图为离线Saved RealHome姿态，
`Areal_visual_preview.png`/`Areal_collision_preview.png`，无实时数据伪装。
未运行ACCP/CBi-RRT*，不写示教器、不发送运动。NO ROBOT MOTION。

## 2026-10-13 A焊枪新CAD添加模型定义焊丝（接续上一节）

用户要求按最新枪嘴→示教器同步TCP最短直线方式添加焊丝。已恢复模型定义，
上一节“新焊丝DISABLED_UNVERIFIED”被本节预览定义取代，枪体安装仍待人工确认。
`tools/add_Areal_model_wire.py`只接受新源SHA256，通过实际STL枪嘴终端环面的
平面边界圆拟合得到source中心[33.330000677,59.362931823,357.000221002]mm。
外圈半径10.291886mm、内圈8.612502mm，圆拟合RMS<0.000006mm；
不是凸包最近顶点，不套用旧gun_tip_model。证据 `Areal_nozzle_reference.json`。
CAD枪嘴轴为[0,-1,1]/sqrt(2)，自带12.5mm×0.8mm细丝仍在原始STL中，未悄悄
删除/扭曲源模型或重建枪体凸包。用户新增圆柱采用gun_tip_model→TCP，
不把CAD轴拟合当作实物送丝轴验收，不以TCP距离优化安装。

参数化焊丝直径1.2mm、当前长度59.703019356mm，终点是ACTIVE控制器镜像TCP球心
位置（标记大小不参与计算）。当前TCP版本
`CONTROLLER-TCP-fa3f8dfe17fa33101b874414944bf8ce89428e47879c6c4a7cf3993634046c95`。
定义记录 `Areal_wire_definition.json`，manifest已保存source/tcp版本绑定并增加版本号，
旧manifest备份在wire_versions/before_Areal_wire_*.json，旧轨迹继续STALE。
TCP标定、枪体安装矩阵、URDF、工件、V-HACD凸组件及冻结仿真文件均未修改。

`calibration/model_axis_wire.py`和MATLAB `model_axis_wire_to_tcp.m`新增显式preview
参数；默认仍禁止STAGED枪体用于规划，只有显式离线预览允许STAGED。
焊丝状态 `MODEL_AXIS_TO_TCP_PREVIEW`，顶层枪体仍 `STAGED_ALIGNMENT_REQUIRED`。
VTK提供黄色圆柱显示和独立橙色碰撞层，无黄色WIRE TIP球；只读同步TCP。
MATLAB离线 `test_Areal_model_wire.m`构建WeldingGunMount固定子对象WeldingGunWire，
碰撞圆柱与HMI完全相同；正式激活仍需原人工确认流程，不自动放行规划或执行。

验证：Python test_model_axis_wire.py 6项PASS（含默认拒绝STAGED/显式preview测试）；
10姿态HMI视觉端点误差0，碰撞层端点最大数值误差<1e-12mm；MATLAB端点最大
误差1.6653e-13mm、collisionCylinder创建成功。此为定义/变换一致性测试，不是密集
轨迹碰撞PASS。报告 `Areal_wire_hmi_report.json`/`Areal_wire_matlab_report.json`，
截图 `Areal_wire_preview.png`蓝色标记明确为离线PREVIEW，不伪装实时红色TCP。
Python compileall通过；未运行ACCP/CBi-RRT*、未修改ROS控制/安全门、无机器人运动。

## 2026-10-13 A焊枪安装/包络经用户确认，正式激活REAL几何

用户针对安装位置、方向和新10个凸组件覆盖确认回复“确认”。使用原
`activate_real_gun`流程，非绕过状态检查：源哈希、刚体安装变换、各凸组件有限性/
三维秩、V-HACD方法均检查；激活时增加同一TCP版本的模型焊丝构建验证，失败则
不写入ACTIVE清单。manifest为ACTIVE，visual/mounting/collision为CONFIRMED，
wire和wire_axis_status为MODEL_AXIS_TO_TCP_ACTIVE，当前长度59.703019356mm。
安装记录为CONFIRMED/applied_to_planning_model=true，confirmation_source记录
2026-10-13用户聊天确认。变换不改，安装记录旧版本备份为
`mount_versions/before_operator_confirmation_Areal_20261013.json`，manifest由原
激活函数自动归档。所有旧trajectory保持STALE/requires_replan，不覆盖历史数据。

确认弹窗改为动态读取组件数及当前枪嘴→同步TCP的长度/直径，去掉错误18mm/26
组件文案；tooltip说明激活REAL后MATLAB用同一版本，不发送运动。
保护文件（ACTIVE/HISTORY TCP、URDF、workpiece、冻结JSON/MAT）激活前后SHA一致。

新增MATLAB只读 `test_confirmed_Areal_scene.m`：正式
`build_real_planning_scene(false)` PASS，real TCP/workpiece均加载，
10个枪体collisionMesh和固定WeldingGunWire collisionCylinder加载成功。
Saved RealHome含焊丝的自碰撞=false、环境碰撞=false，未改变原跳过列表/碰撞阈值。
结果：`real_gun_geometry/Areal_activation_scene_report.json`及同名前缀log。
Python焊丝6项测试和compileall PASS。未运行ACCP/CBi-RRT*，未执行路径或发送运动。
当前 `规划文件/weld_anchor_points_real.json`仍缺失；几何激活解除对应4项阻塞，
不声称在线规划已完全恢复，须重新生成当前版本绑定的真实焊点和轨迹。

## 2026-10-13 激活新A焊枪后的真实焊点生成尝试

针对HMI缺少weld_anchor_points_real.json报错，执行已有
`generate_real_weld_anchors(outputDir,false)`，保持ACCP参数、硬碰撞约束和安全门不变。
输出目录：`UR5/规划文件/real_anchor_runs/Areal_confirmed_20261013/`；完整日志：
`UR5/规划文件/real_anchor_runs_Areal_confirmed_20261013.log`。
TCP/workpiece实际加载成功，10个新枪体组件和MODEL_AXIS_TO_TCP_ACTIVE焊丝参与检查。
结果PLAN_FAILED，ACCP算法累计时间9.155048秒，总调用时间17.524532秒。
最终Point1诊断：264次IK调用，141个pose成功候选，其中86个被方向约束拒绝，
55个被workpiece_collision拒绝，最终接受0。连续性计数为诊断项，不是硬拒绝原因；
限位/位置等计数与候选记录口径不同，不应加总为互斥分类。当前诊断未细分工件碰撞
具体link/component pair，不能据此断言是焊丝或枪体与哪个面相撞。
未生成、复制或伪造正式真实anchor，不运行CBi-RRT*，旧数据继续STALE。
生成失败原因和当前四项版本绑定保存在report.json，候选保存在
accp_point1_diagnostics.json/real_seam.mat。下一步应审查这些候选的工件碰撞pair，
不能通过关闭碰撞、复制历史anchor或放宽约束解除缺文件问题。
HMI `_load_weld_anchors` 仅增加缺少REAL文件的中文说明，解释须先通过ACCP与
进退枪验证，不再直接展示Errno 2。py_compile及抽取方法的缺文件分支测试PASS。
本轮仅离线生成和显示文案调整，无ROS运动、示教器写入、TCP/安装参数修改。

## 2026-10-13 ACCP Point1的55个工件碰撞候选逐对审计

用户要求审计55个候选并输出Top10，不修改过滤规则。复用
diagnose_accp_point1_collision_pairs.m，增加显式workpiece_collision选择模式，
支持MATLAB string字段；保留原历史154候选模式及其20deg/版本检查。
指定55模式只用序列化scene，版本与当前不一致时在报告中标为
scene_is_current=false及execution_authorized=false，不绕过规划/执行版本门。
实际运行发现控制器镜像TCP已更新，当前wire约55.9mm；被审计快照wire为
59.703019mm，故本结果不能代替最新TCP场景验收。

55个均与工件碰撞，0个自碰撞。WireCollision↔bottom=55、↔wall_y1=55；
gun component03/04↔wall_x0各20、component09/10↔wall_x0各12。
有枪体碰撞的独立候选27个，其余28个仅焊丝碰撞；wall_x1=0。
原完整模型与逐组件归因逐候选assert一致。Top10原索引为
51/55/54/52/53/62/63/61/60/64，均仅两对焊丝—工件碰撞，不可执行。
排序为碰撞对数量→非碰撞间隙→RealHome距离，连续q每关节0.5deg去重，
不合并2pi分支；不能解释为最小穿透，MATLAB API不提供penetration depth。
报告、完整55候选逐对JSON、Top10 JSON/CSV及说明保存于
`UR5/规划文件/real_anchor_runs/Areal_confirmed_20261013/collision_audit_55/`，
成功日志collision_audit_55_final.log；两次前置诊断失败日志保留。
不修改TCP、安装、工件、机器人、规划算法或任何碰撞skip/阈值；无运动。

## 2026-10-13 HMI只读真实焊丝几何诊断与绿色起点

用户要求不改规划逻辑，仅报告真实枪口/TCP/焊丝端点及轴线并增加绿色起点。
仅修改 `UR5/ur5_hmi/gui/vtk_robot_view.py` 的显示/只读诊断：底部新增可折叠
“焊丝几何诊断（world=URDF base）”，显示nozzle_tip_world、tcp_live_world、
wire_start、wire_end、wire_length及单位轴线，并报告CAD枪嘴轴夹角、起点与
独立CAD枪嘴基准偏差、黄色显示线端点与LIVE TCP偏差。JSON accessor
`real_wire_geometry_diagnostics()`长度/坐标单位m，面板统一转换为mm。
CAD独立基准读取manifest wire.evidence_file并核对源STL哈希，不推测枪嘴或
把CAD基准宣称为物理测量。绿色球半径2mm仅用于标记wire_start，不进入fit bounds。
红色LIVE TCP和黄色焊丝端点/碰撞定义保留现有逻辑，诊断不纠正任何不一致。
预演中绿色点随原预演焊丝显示，但诊断仍读取LIVE q/TCP并明确PREVIEW显示模式，
不把蓝色预演TCP当作红色真实TCP；显示端点比较在预演时为UNKNOWN/null。
缺少控制器TCP、FK回退或控制器时间戳过期时，真实诊断显示UNKNOWN。

新增 `tools/test_wire_geometry_diagnostics.py` 使用本地合成反馈进行VTK smoke：
6项请求字段、端点误差0、独立枪嘴基准偏移1mm仅报告、单位轴、绿色标记、
展开面板、LIVE/preview隔离、过期及FK回退UNKNOWN、四个保护文件哈希均PASS。
Saved RealHome测试夹具下wire长55.941479mm、CAD轴夹角10.013151deg；夹具坐标
不作为当前真机world测量。既有test_model_axis_wire.py 6项PASS，py_compile PASS。
未修改MATLAB、ACCP、RRT、碰撞过滤、工具/TCP/工件标定；无机器人连接或运动。
当前轴线约10deg偏差只报告，不自动旋转焊枪、缩短焊丝或强制对齐。

## 2026-10-13 当前Point1方向约束A/B与全IK成功候选碰撞审计

用户中断CAD轴替换请求，改为不修改参数/模型/过滤的诊断。上一请求未发生任何
Python/MATLAB轴逻辑或manifest修改，当前仍为枪口→TCP连线轴。
新增diagnose_current_point1_axis_ab.m，读取原real_seam_round_01真实options，
在当前真实场景冷生成，保持seed/beam/网格/限位/方向与碰撞参数；仅输出目录及
configurationOnlyDiagnostic隔离诊断。既有pair审计新增all_ik模式，逐对审计全部
144个限位/位姿有效IK候选，保存方向误差、orientation_error_deg、TCP位姿、q、
collision pairs与距离；264次求解中其余120次不属于前置检查有效候选。
审计场景版本与当前一致(scene_is_current=true)，wire55.941479mm。

A/B采用同一IK池两组过滤评估，B只跳过方向比较，其余侧位置/碰撞/限位和
pose精度不变；不是两次整条16点路径规划，也不宣称关闭方向后整条路径可行。
264次IK调用，144pose成功；A方向通过55、碰撞通过0、最终0；
B方向允许144、碰撞通过0、最终0。144全部WireCollision↔bottom和↔wall_y1，
还有若干枪体凸组件与x0/x1面附加碰撞。关闭方向不能解除本候选池的碰撞阻塞。
保存于 `UR5/规划文件/real_anchor_runs/point1_axis_ab_current_20261013/`，
README、comparison.csv、collision_pairs.json、Top10 CSV/JSON、current_core.mat；
完整日志为同名目录外的.log。逐组件归因与完整checkCollision逐候选assert一致。
API不提供penetration depth，未推测穿透量。未生成真实anchor，无运动。

## 2026-10-13 Point1焊丝碰撞圆柱缩短敏感性诊断

新增 `wire_collision_shorten_mm`，仅在离线场景副本中重建
`WeldingGunWire` 的 collisionCylinder，保持原有局部变换/中心，只改变长度；
视觉焊丝、TCP、IK、ACCP方向约束、枪体V-HACD、工件和碰撞过滤均不变，默认0。
core及feasibility已透传该参数。使用同一真实场景设置测试0/3/5/10mm，结果目录：
`UR5/规划文件/real_anchor_runs/wire_collision_shorten_20261013b/`。

| shorten(mm) | IK | WireCollision候选 | Workpiece碰撞候选 | Point1最终可行 |
|---:|---:|---:|---:|---:|
| 0 | 144 | 144 | 144 | 0 |
| 3 | 144 | 0 | 85 | 16 |
| 5 | 144 | 0 | 85 | 16 |
| 10 | 144 | 0 | 85 | 16 |

视觉焊丝四组均55.941479mm；临时碰撞圆柱长度为55.941479、52.941479、
50.941479、45.941479mm。3mm起WireCollision不再是Point1阻塞，但枪体凸组件
与工件仍使85个候选拒绝，16只是Point1数量，不代表16点焊缝PASS。结果仅为
诊断，不能把缩短模型直接作为真实安全模型。comparison.csv/json、各组summary、
完整逐对collision_pairs和Top10均已保存；真实manifest/TCP/场景未写回，
不生成anchor、不授权执行，NO ROBOT MOTION。

## 2026-10-13 正式 ACCP 固定 3 mm 后的完整离线运行

用户要求将 `wire_collision_shorten_mm` 固定为 3 mm 并重新运行完整 ACCP。正式入口
`generate_real_weld_anchors.m` 已显式设置 `opts.wire_collision_shorten_mm=3`；
`replan_weld_actual_axis_adaptive_core.m` 与 `replan_weld_actual_axis_adaptive_feasibility.m`
默认值也为3，诊断脚本仍可显式传入其它值做敏感性实验。该参数只改变碰撞圆柱长度，
不改变视觉焊丝、TCP、IK、ACCP约束、焊枪几何或碰撞过滤。

完整结果保存于
`UR5/规划文件/real_anchor_runs/full_accp_wire3_20261013/`。16点初始焊缝经自适应
细分到21点并进行了有限beam/angular recovery；Point1的WireCollision阻塞已解除，
但第20层没有连续可行候选，最终 `success=false`、`complete=false`、
`collision_check_pass=false`。累计ACCP算法时间约52.284s，总墙钟时间约60.690s。
未生成或复制 `weld_anchor_points_real.json`，没有新可执行轨迹，也没有任何机器人运动。

## 2026-10-13 ACCP Point20 候选过滤审计

针对固定 `wire_collision_shorten_mm=3` 的真实场景完整运行，新增
`UR5/matlab仿真/diagnose_accp_point20_candidates.m`。脚本只读重放正常邻域、
角度恢复 10/20° 以及 no-retry recovery beam-3 的 Point20 候选，未修改 ACCP、
TCP、碰撞规则或安全门，未发送运动。

结果目录：
`UR5/规划文件/real_anchor_runs/full_accp_wire3_20261013/point20_audit_20261013/`。

审计统计：258 次 IK、258 次数值成功、230 个位姿通过；方向约束拒绝 102，关节
限位拒绝 0，工件节点碰撞拒绝 84，焊枪/焊丝节点碰撞拒绝 0，自碰撞拒绝 0；
节点通过 6（去重后的主要候选 5），Point19→Point20 连续性拒绝 6，其中最大
关节步长拒绝 2，边碰撞拒绝 4。四条边碰撞均为
`WeldingGunWire <-> WorkpieceCollision_bottom`；节点工件碰撞主要为
`WeldingGunMount <-> WorkpieceCollision_wall_x1`。最终 connected candidate 为 0，
状态仍为 `NO_CONNECTED_CANDIDATE`。Top 候选、q、TCP pose、与 Point19 关节差、
边碰撞 pair 和距离已写入该目录 JSON/CSV。`checkCollision` 不提供有符号穿透深度，
报告明确标记为不可用。

## 2026-10-13 ACCP Point19→Point20 密集边审计与局部笛卡尔恢复（诊断，不接入正式规划）

用户要求针对固定 `wire_collision_shorten_mm=3` 的 Point20
`NO_CONNECTED_CANDIDATE` 做原因审计和独立恢复实验。新增入口：
`UR5/matlab仿真/diagnose_accp_point19_point20_local_recovery.m`；新增离线查看器：
`UR5/matlab仿真/visualize_accp_point20_edge_audit.m`。二者均不修改正式 ACCP、
CBi-RRT*、TCP、焊枪/焊丝/工件几何、碰撞过滤、轨迹或 `weld_anchor_points_real.json`，
没有 ROS2 连接和机器人运动。

脚本先核对失败 MAT 中的 TCP、real gun、gun mount、real workpiece、calibrated URDF
五项 SHA-256 及 wire shorten；当前全部一致，状态为 `CURRENT`，但
`execution_authorized=false`。若将来任一绑定不一致，脚本直接返回 `STALE`，不做恢复。

原始连接仍为 `NO_CONNECTED_CANDIDATE`。对既有3个边候选各取101个关节空间样本，首次
碰撞比例分别为0.05、0.24、0.06，具体均为
`WeldingGunWire <-> WorkpieceCollision_bottom`；每条记录包含首次碰撞的6关节角、TCP
位姿、碰撞前最后安全姿态和pair。MATLAB `checkCollision` 不提供有符号穿透深度，报告
明确为不可用。两个步长候选最大变化分别为31.5835°（J1/J5超限）和17.6808°
（J3/J5超限）。

独立局部恢复采用 Point19→Point20 的笛卡尔位置线性插值与姿态四元数SLERP，在25%、
50%、75%目标重新调用现有数值IK，并对每个新增节点和相邻边进行101点完整碰撞检查。
结果为 `DIAGNOSTIC_PASS`：第一条成功分支增加3个IK节点，20次IK调用、4条边检查，
最大关节变化3.5049°，最大TCP节点误差0.000027mm，最小可用正碰撞间隙0.4034mm；
两个原步长候选也分别找到合法的局部笛卡尔分支。该结果只保存在：
`UR5/规划文件/real_anchor_runs/full_accp_wire3_20261013/point20_local_recovery_20261013/`，
只标记 `DIAGNOSTIC_PASS`，绝不自动回写正式ACCP或授权执行。

## 2026-10-13 ACCP 局部笛卡尔桥接回退（离线集成与真实场景回归）

按 Point19→Point20 的独立 `DIAGNOSTIC_PASS` 结果，局部 Cartesian Bridge 已作为
`replan_weld_actual_axis_adaptive_core.m` 内部的可配置、有限预算回退策略接入。正常
连续构型搜索保持不变；只有相邻层返回 `NO_CONNECTED_CANDIDATE` 时才触发回退。回退在
失败段插入 25%、50%、75% 的 TCP 中间目标，位置线性插值、姿态四元数 SLERP；每个
新增目标重新调用现有 IK，保留合法分支，并对每个新增节点及每条边做完整碰撞/关节步长
检查。每条桥接边至少 101 个关节空间样本。成功的中间状态会插回 `stateBank` 和
`seam`，随后继续搜索全部后续焊点，不能只修复 Point20 就返回成功。

回退默认预算：`cartesianBridgeMaxDepth=2`、`cartesianBridgeMaxIKCalls=3000`、
`cartesianBridgeMaxTimeSec=60`、`cartesianBridgeMaxEdgeSamples=101`，并保留有限确定性
分支种子。报告区分“选中的成功桥接路径”与“所有尝试中的最近失败候选”，避免用失败
近邻间隙冒充安全余量。`wire_collision_shorten_mm=3` 继续保留，并明确为需要物理安全
验证的碰撞近似参数。所有输出仍 `executionReady=false`、`offlineOnly=true`。

离线同场景对比结果：

* 原始 ACCP：21 个焊点，第 20 点 `NO_CONNECTED_CANDIDATE`，配置成功率 0/1，
  3026 次 IK，29.283615 s。
* 桥接 ACCP：原始 21 点扩展为 24 层，在第 20 点恢复一次；选中的桥接路径插入
  3 个节点（25/50/75%），13 次桥接尝试、总桥接尝试插入统计 12 个节点，配置链
  24/24、最终可达状态 3；连续配置成功率 1/1，配置 IK 3314 次、桥接 IK 584 次，
  总 IK 3898 次，配置搜索 60.273676 s。
* 选中桥接路径：最大关节步长 1.742416°、节点 TCP 误差约 0.0000024 mm、
  101 点/边、最小正间隙 0.579816 mm。所有尝试中的最近失败近邻为 0.007115 mm，
  不作为成功路径安全余量。
* 桥接后完整几何精度阶段：Cartesian Refinement 47 个输出节点，TCP 轮廓最大
  0.076950 mm，节点/边碰撞 PASS；但控制器等价的 positions+velocities cubic
  Hermite 复核（4601 个采样点）发现时间同步最大 2.885364 mm、姿态最大 0.694330°、
  峰值关节加速度 0.323359 rad/s²（限值 0.261799），因此 Adaptive Time Scaling
  阶段为 FAIL，完整最终轨迹不通过，仍不可执行。

控制器复核不再用分段平均加速度替代节点过渡峰值；新增
`UR5/matlab仿真/validate_accp_controller_interpolation.m`，按 ROS2
`joint_trajectory_controller` 的 positions+velocities cubic Hermite 规则密集采样，
同时检查 TCP 轮廓/同时间误差、姿态、关节速度/加速度、限位和碰撞。新增汇总脚本
`UR5/matlab仿真/compare_accp_cartesian_bridge_fallback.m`，对比报告：

`UR5/规划文件/real_anchor_runs/accp_bridge_comparison_20261013/comparison_report.json`

完整离线结果 MAT：

`UR5/规划文件/real_anchor_runs/accp_bridge_comparison_20261013_improved_full_cold_v2.mat`

本轮未生成真实可执行轨迹、未设置 `executionReady=true`，未修改 CBi-RRT*、TCP、
焊枪/焊丝/工件几何、碰撞过滤、ROS2 控制链路或发送任何机器人运动。

## 2026-10-13 Controller-aware 时间优化（离线，不可执行）

新增 `UR5/matlab仿真/optimize_accp_controller_timing.m` 和
`test_accp_controller_timing_optimization.m`。仅优化段时长/局部时间比例，沿用
现有 `[zeros(1,6);diff(qPath)./diff(timeFromStart)]` 速度生成规则。
输入使用此前完整桥接/refinement通过的47点真实场景快照，未重新运行或修改ACCP。
qPath和tcpPosePath逐元素不变，场景及标定绑定随报告保存；不覆盖anchor或真实轨迹。

桥接邻域和高姿态变化段加权搜索；最终精确三次Hermite验证4601个样本，完整碰撞/
限位PASS，并解析检查段内速度驻点及所有节点左右加速度峰值。对比：

| 指标 | 优化前 | 优化后（第二轮） |
|---|---:|---:|
| 时长 s | 18.606468 | 17.099156 |
| TCP轮廓最大 mm | 0.187600 | 0.073211 |
| 同时间TCP误差最大 mm | 2.885364 | 0.863661 |
| 姿态误差最大 ° | 0.694330 | 0.453679 |
| 速度峰值 rad/s | 0.074037 | 0.087119 |
| 解析加速度峰值 rad/s² | 0.328283 | 0.261798 |
| 碰撞/限位 | PASS | PASS |

原采样基线加速度0.323359未完全覆盖节点左右极限；解析峰值为0.328283。
优化后限值0.261799满足，最小采样正间隙0.583667 mm，不代表独立物理安全余量。
wire_collision_shorten_mm=3 保持不变且仍需物理验证。

严格失败原因：首点速度为零，第一段末速度为Δq/h，Hermite
`q(u)=q0+(u-h10(u))*Δq` 对任何正段时长h不变，101个固定归一化采样点已发现
同步误差0.863661 mm；纯段时长优化不可能达到0.5 mm同步指标。
返回 `TIME_ONLY_INFEASIBLE_WITH_EXISTING_VELOCITY_RULE`，
`numericalAcceptancePass=false`、`executionReady=false`。
未擅自优化节点速度、增加节点、修改TCP路径或改安全门。

第二轮报告/完整诊断MAT/日志：
`UR5/规划文件/real_anchor_runs/controller_aware_timing_20261013_v2/`。
第一轮候选也保留在 `controller_aware_timing_20261013/`（12.713352 s，姿态0.522923°，
未通过姿态门）；第二轮优先选作离线候选，仍不满足全部验收项。

本机ROS2控制器 positions+velocities 使用三次Hermite，另携带 accelerations 则为
五次插值。本模块候选明确不导出accelerations，不修改现有消息/执行代码。
无机器人运动，不生成可执行真实轨迹。后续要突破同步误差下界，需用户另行授权
调整速度生成规则，再重新执行控制器等价全程验证。

## 2026-10-13 Adaptive Velocity Profile Generation（离线速度策略对比）

用户授权只优化节点速度后，新增独立模块
`UR5/matlab仿真/generate_accp_adaptive_velocity_profile.m` 和回归入口
`test_accp_adaptive_velocity_profile.m`。输入取已保存 Controller-aware 第二轮时间候选，
47个q节点、tcpPosePath和全部时间戳保持逐元素不变。未改ACCP、Cartesian Bridge、
CBi-RRT*、TCP、焊枪、碰撞过滤或ROS2。

A为非均匀网格加权中心差分；B为端点静止的C2最小积分jerk五次曲线导出的节点速度；
C允许非零入口/出口，优化节点速度使固定TCP路径的时间同步误差降低。B的五次曲线
只用于求速度，实际验收全部使用positions+velocities三次Hermite，明确不导出acceleration。
C使用线性FK/Jacobian目标与精确加速度端点约束，速度先密集约束再追加解析驻点约束。
没有改节点时间，不使用全局速度倍率来掩盖误差。

| 方案 | 同步最大 mm | 轮廓最大 mm | 姿态最大 ° | 速度峰值 rad/s | 加速度峰值 rad/s² | 数值结果 |
|---|---:|---:|---:|---:|---:|---|
| 原前段平均速度 | 0.863661 | 0.073211 | 0.453679 | 0.087119263 | 0.261798476 | FAIL |
| A中心差分 | 0.543578 | 0.045823 | 0.302694 | 0.087112452 | 0.177026986 | FAIL |
| B五次导出速度 | 1.229321 | 0.033386 | 0.306707 | 0.086337104 | 0.175337720 | FAIL |
| C非零入口优化 | 0.238287 | 0.013041 | 0.212377 | 0.087265027 | 0.261796382 | PASS |

全部时长17.099156 s，均4601点精确FK/碰撞复核、限位PASS。C同步RMS0.078519 mm、
轮廓RMS0.002150 mm、采样最小正间隙0.628018 mm。速度限值deg2rad(5)，加速度
限值0.261799 rad/s²。第一轮C候选解析速度略超限（0.087266609）被正确判FAIL；
后续加1条解析速度驻点约束，未放宽限值，达到上述PASS。失败第一轮保留在
`adaptive_velocity_profile_20261013/`，第二轮在`adaptive_velocity_profile_20261013_v2/`。

最终完整结果目录：
`UR5/规划文件/real_anchor_runs/adaptive_velocity_profile_20261013_final/`，含
`report.json`、`comparison.csv`、四种六关节速度曲线CSV、`velocity_curves.png`、
`velocity_profiles_diagnostic.mat`及`matlab.log`。

所有结果 `offlineOnly=true`、`executionReady=false`。C存在非零入口及出口速度，必须
独立验证与接近/离开段及实际控制器首尾状态的衔接，不能从静止直接执行本段。
小间隙、wire_collision_shorten_mm=3仍需物理安全验证；采样PASS不是连续安全证明。
没有生成正式真实轨迹、没有覆盖anchor或标定、没有发送任何机器人运动。

## 2026-10-14 Hybrid Adaptive Velocity Optimization（离线研究）

在 `generate_accp_adaptive_velocity_profile.m` A/B/C 比较基础上新增独立
`optimize_accp_hybrid_velocity.m`、精确导数指标 `accp_cubic_derivative_metrics.m`、
回归入口 `test_accp_hybrid_velocity.m`。只优化47个固定节点速度，qPath、tcpPosePath、
17.099156 s时间戳、TCP、场景和几何不动。目标包含对中心差分A/五次参考B的速度偏离、
解析分段jerk、节点加速度跳变、积分加速度能量和节点速度曲率。C用作可行初始点；
约束是局部线性化凸二次问题，最终必须通过控制器等价三次Hermite精确FK和完整碰撞审计。
默认尝试90%加速度软上限，未通过才回到原硬上限；本次90%软目标可行。所有候选的
positions+velocities都明确按三次插值评估；无acceleration字段。

| 策略 | TCP轮廓最大 mm | 同步最大 mm | 姿态最大 ° | 速度最大 rad/s | 加速度最大/RMS rad/s² | jerk最大/RMS rad/s³ | 节点加速度跳变最大 rad/s² |
|---|---:|---:|---:|---:|---:|---:|---:|
| A中心差分 | 0.045823 | 0.543578 | 0.302694 | 0.087112 | 0.177027/0.026212 | 2.208399/0.295479 | 0.094015 |
| B五次参考 | 0.033386 | 1.229321 | 0.306707 | 0.086337 | 0.175338/0.027088 | 1.164653/0.189855 | 0.143568 |
| C精度优先 | 0.013041 | 0.238287 | 0.212377 | 0.087265 | 0.261796/0.044657 | 3.517053/0.612044 | 0.418127 |
| D Hybrid | 0.109436 | 0.355067 | 0.420968 | 0.087265 | 0.148927/0.029294 | 0.717414/0.119425 | 0.180123 |

各候选时长17.099156 s，4601个样本碰撞/限位均PASS。D满足所有数值硬门及90%加速度
软目标；相对C，最大加速度、最大jerk、RMS jerk、节点加速度跳变分别下降43.1%、79.6%、
80.5%、56.9%。D的TCP误差高于C但仍合格。加速度并非节点连续；三次段内jerk常量，节点
跳变量独立列出。最小采样正间隙0.569161 mm，不代表安全余量认证。wire_collision_shorten_mm=3
仍需物理验证。

入口/出口衔接状态 `BLOCKED_NO_CURRENT_BOUND_APPROACH_EXIT`：冻结旧接近规划没有当前场景
绑定，最近真实接近规划状态为 `PLAN_FAILED_REAL_SCENE_OR_STALE_GOAL`，没有当前版本退出轨迹。
D首末速度非零，未证明与CBi-RRT*段位置/速度/加速度连续。控制器本机JTC源码默认splines，
scaled controller配置position命令接口；位置+速度字段按三次样条。没有读取运行时活动参数、
实际UR内部伺服插值或speed scaling时间映射。因此只证明ROS2层离线模型，整体仍
`executionReady=false`，没有生成正式轨迹。

最终输出目录：
`UR5/规划文件/real_anchor_runs/hybrid_velocity_20261014_final/`，含MAT/JSON、对比表、
速度/加速度/分段jerk曲线、节点加速度跳变表、J3/J6 12–15 s专图及MATLAB日志。未修改
ACCP、Cartesian Bridge、CBi-RRT*、TCP/焊枪/碰撞规则或ROS2执行链路；无机器人运动。

## 2026-10-14 完整焊接任务拼接验证（离线）

新增 `UR5/matlab仿真/validate_full_welding_execution.m`，用于严格校验当前版本的
CBi-RRT*接近段 + ACCP-D焊接段 + CBi-RRT*退出段。它要求TCP/URDF/焊枪/焊丝/工件/
碰撞场景六项版本绑定，核对两处关节、TCP位置/姿态、速度及由三次Hermite计算的
加速度连续性；拼接合格后，使用positions+velocities cubic Hermite对完整路径至少
5000点做FK、关节限位和含机器人/焊枪/焊丝/工件的碰撞采样，并报告jerk与节点加速度跳变。
验证器只读路径，不规划、不修复、不导出真机轨迹、不发ROS指令。

当前证据入口 `UR5/matlab仿真/run_full_welding_execution_validation_current.m` 和报告：
`UR5/规划文件/real_anchor_runs/full_welding_execution_validation_20261014/full_welding_execution_validation.json`。
当前验收状态仍为 `BLOCKED`：ACCP-D单段47点、17.099156s，已有4601点验证，轮廓误差
0.109436mm、同步误差0.355067mm、姿态误差0.420968°、最大速度0.087264717rad/s、
最大加速度0.148926629rad/s²、最大jerk0.717414rad/s³、碰撞/限位PASS；但该段记录未单独
给出wire_model_version及collision_scene_version。10月10日成功接近段未绑定当前标定，
10月12日最近真实接近规划失败，当前无版本绑定退出段；10月8日旧退出/完整轨迹均STALE。
故两处边界速度/加速度与TCP差异不可验证，未执行整程5000点复核，`executionReady=false`。

当前 JSON 已通过 JSON 解析检查；两个 MATLAB 文件与缺段回归入口通过 `mlint` 静态解析。
本环境执行 `matlab -batch` 返回状态码1且无启动诊断，因此未声称 MATLAB 运行时回归通过；
缺段回归入口需在 MATLAB 可正常启动后执行。

下一步必须在相同REAL_CALIBRATION快照下生成接近段和退出段，并为三段补齐上述六项版本；
再运行 `run_full_welding_execution_validation_current`（或向验证器传入新三段）完成全程离线验收。
wire_collision_shorten_mm仍为3mm近似值，碰撞采样PASS不构成连续路径证明或物理安全余量认证。
本次未改变ACCP/CBi-RRT*/场景和控制链路，无机器人运动。

## 2026-10-14 HMI规划预览与执行准备解耦（当前入口）

在线规划新增两个独立按钮：“规划轨迹”→`PlanningWorkflowPanel._start_preview_planning`
→`plan_hmi_preview_request.m`；“轨迹连续规划”→`_start_continuous_planning`
→`prepare_hmi_continuous_request.m`。`gui/planning_workflow.py`维护独立状态：
IDLE、PLANNING PREVIEW、Preview Ready（绿色）、PREPARING EXECUTION、Execution Ready（蓝色）、
PREVIEW FAILED、PREPARATION FAILED、CANCELED。准备失败不丢弃原始预览；没有任何运动信号发出。

第一阶段不调用`_real_planning_readiness_reasons`或正式real anchor，不读取
`weld_anchor_points_real.json`。REAL预览明确复用当前已完成的47节点ACCP-D候选，然后在
同一个真实场景快照内重新调用CBi-RRT*生成接近/退出。缺该候选时调用现有ACCP；
SIMULATION使用显式冻结场景。新冷启动ACCP测试在Point20暴露现有核心`selected_fractions`
字段重复错误；本轮按要求未修改ACCP核心，原失败日志保留。复用D仅用于预览和速度参考，
保存其原始版本绑定，第二阶段仍逐段核验版本，不会把旧候选自动认证为当前执行结果。

原始MAT是`full_raw_trajectory.mat`，包含qPath、TCP 4×4 pose、weldingPosePath、robotState、
segmentLabel及approach/weld/exit各段、来源版本、raw_scene.mat及其hash。保留重复/断裂边界；
拼接时钟DISPLAY_ONLY，始终preview_only=true/executionReady=false。后验碰撞FAIL可加载和
播放；实际算法找不到路径/模型数据无效仍明确失败。HMI增加在线播放/暂停/停止/slider，
显示预览TCP XYZ/RPY、关节角与段名；真实q/LIVE TCP不会被覆盖。预览MAT在Final Gate直接
显示Preview Only。真实规划TCP path只作已有base_link→base转换，不重复施加工件显示变换。

第二阶段先诊断边界、需要时有限次数Cartesian线性位置/SLERP+连续IK桥接（maxDepth=2、
maxIK=100、maxTime=30s；各边101点检查；关节步长15°和0.5mm/0.5°门保持），再复用既有
Hybrid D，以其为参考优化自由空间速度及焊接段两侧2或4个接口节点速度；所有原始q/TCP节点
及各段时间保持，内部D速度固定。保持速度deg2rad(5)、加速度0.261799，任务接口C2约束。
全程三次Hermite采样至少5000，补充解析速度/加速度/关节位置极值、jerk与节点加速度跳变；
最后检查TCP/URDF/枪/焊丝/工件/碰撞场景版本和真实场景准入。wire_model_version与
collision_scene_version是依赖文件hash+现有3mm焊丝参数的内容指纹，未改几何。
全部通过才写trajectory_execute_ready.json/.mat；MAT只有positions/velocities，不带
accelerations以免控制器改用五次插值。实际Execute仍使用原ROS2动作链和Final Check。

真实完整预览回归：`UR5/规划文件/workflow_runs/real_preview_final_20261014/`。
Preview Ready PASS，共171节点（approach63/weld47/exit61），播放时间92.914463s，规划71.994237s，
3570个预览诊断采样碰撞PASS。两个接口关节差严格0，TCP位置差5.080e-6/4.201e-6mm，
姿态差约2.415e-6/1.708e-6°；该采样不是执行认证。

完整执行准备：`UR5/规划文件/workflow_runs/real_prepare_final_20261014/report.json`。
FAIL=`Boundary velocity/acceleration continuity infeasible under fixed weld nodes/timestamps`；
允许两侧2节点及4节点速度调整的两个QP尝试都exitflag=-2。接近→焊接最大速度跳变
0.071398rad/s，焊接→退出最大加速度跳变0.546370rad/s²。完整任务未进入5000点验收，
未生成任何execute_ready文件，executionReady=false。后续应单独评估接口局部时间调整，
不能把Preview Ready当作Execution Ready。

GUI离线测试`tools/test_two_stage_planning_workflow.py`显式禁用ROS线程，验证缺anchor预览、
碰撞FAIL仍预演、失败保留预览、真实171节点自动加载及Live q不被覆盖。MATLAB
`test_hmi_two_stage_workflow.m`通过版本/STALE门、真实已有边的3IK中间节点+404边样本、
QP可行分支及真实D单段5015点审计（轮廓0.109442mm、同步0.355085mm、姿态0.420974°，
碰撞PASS）。单段结果不替代整程PASS。日志、JSON、GUI截图在
`UR5/规划文件/workflow_runs/gui_regression_20261014/`。MATLAB已在获准本机环境正常运行。
未修改CBi-RRT*/ACCP-D核心、TCP/枪/焊丝/工件文件、ROS2或原执行安全门；NO ROBOT MOTION。

## 2026-10-14：预览穿模审计；此前碰撞PASS被新证据纠正

用户限制：只检查/修复preview显示链；不修改CBi-RRT*、ACCP-D、IK、TCP标定或碰撞模型。

证据来源：`UR5/规划文件/workflow_runs/preview_20261014_121436_257478/`的原始171点轨迹
和不可变raw_scene.mat。截图第117帧q_deg=[285.4890,-20.4954,-27.9821,-9.5079,-104.8243,-3.3611]。
实际保存的MATLAB模型中：base_link_inertia、shoulder_link、upper_arm_link、forearm_link、
wrist_1_link、wrist_2_link、wrist_3_link的Collisions均为0。WeldingGunMount有10个凸组件，
WeldingGunWire有1个圆柱。原self check返回0碰撞、所有gun↔UR5距离Inf，原因是本体没有
几何，不是证明路径安全。请撤回此前该场景预览“碰撞PASS”的安全含义；完整真实轨迹
不能据此验收。未重新认证或更改任何旧正式轨迹。

HMI/MATLAB171帧FK位置差：flange最大3.3766e-13mm、gun最大3.3766e-13mm、wire最大
4.5776e-13mm。VTK第117帧STL表面交叉查到forearm_link↔WeldingGun_visual；另外腕3的
法兰安装接触须按原结构skip pairs区别解释，不得顺带忽略前臂。显示穿模并非单纯FK
更新不同步。MATLAB真实场景构建`build_real_planning_scene.m`第8行importrobot后本体
碰撞网格未加载，需要下一轮明确授权处理加载链；本轮没有修改该文件或碰撞模型。

实现仅在preview数据/显示/审计入口：
- 新`matlab仿真/audit_hmi_preview_display.m`读取原场景，导出每帧法兰/枪体/焊丝pose和
  collision_status/pairs，并21点采样每条播放边。保留原skip pairs和3mm碰撞缩短参数。
  3741原生样本零命中，但coverage=false，因此previewReady=false、PREVIEW WARNING。
  输出独立raw副本，不覆盖用户原轨迹/场景/标定。未知/不兼容场景为UNKNOWN，仍可查看。
- `plan_hmi_preview_request.m`后验检查改调该审计，不改任何planner输入、算法参数或路径。
- 新`gui/preview_display.py`读取带哈希的不可变工具快照；实时控制器TCP/安装微调不能
  覆盖旧轨迹预览的固定变换。LIVE q/TCP始终独立。VTK每帧同步更新本体与工具actor，
  使用独立snapshot工具actor，隐藏重复live工具层；停止预览恢复原显示。
- 原始MAT新增flangePosePath、gunPosePath、wirePosePath、collisionResult及snapshot引用。
- `gui/planning_workflow.py`新增PREVIEW WARNING；缺审计、发现交叉或缺几何时不能显示
  Preview Ready，仍可播放失败结果。每帧显示base下TCP/gun tip/wire tip/flange XYZ/RPY，
  tooltip含完整flange pose和T_flange_to_gun；日志保留安装矩阵与发现的表面交叉对象对。
- VTK只读表面交叉诊断与MATLAB实体collision result分别标识。该显示诊断不增加/替换
  规划碰撞几何，不是安全证明。原有ROS控制/运动执行/标定/几何文件保持不变。

诊断与截图目录：`UR5/规划文件/workflow_runs/preview_display_audit_20261014/`。
`preview_display_audit.json`记录缺失7个本体几何及全部171帧；`display_chain_test.json`
记录FK、实际VTK actor矩阵、live q保留、控制器TCP变化不影响预览、停止恢复与GUI WARNING
回归结果。`tools/test_preview_display_chain.py --vtk`显式禁用HMI ROS线程并生成第117帧
等轴/正视/侧视和HMI WARNING截图。旧两阶段GUI测试也通过，旧无审计轨迹现在显示WARNING。
未发送任何机器人运动；未生成execute_ready；仅显示链修复不能消除实际路径碰撞。

## 2026-10-14：MATLAB UR5 本体碰撞几何加载修复与预览复审

用户已确认两阶段改造后的预览穿模来自MATLAB碰撞场景没有加载UR5本体碰撞网格。本次只修复URDF网格导入和独立预览审计，没有修改CBi-RRT*、ACCP-D、IK、TCP、碰撞过滤/阈值、轨迹节点/时间、ROS2或执行链。

根因：`build_real_planning_scene.m` 原来把`MeshPath`指向`/opt/ros/humble/share`。MATLAB R2025b解析`package://ur_description/...`时会剥掉包名，导致`meshes/ur5/collision/*.stl`路径解析失败。改为单一包根目录`/opt/ros/humble/share/ur_description`后，MATLAB直接导入七份URDF碰撞网格。R2025b对该参数传cell路径列表会报`convertStringsToChars`输出数量错误，故使用单个包根目录。

新增`matlab仿真/load_ur5_body_collision_geometry.m`，供已有场景副本补入URDF定义的STL顶点、scale及原点变换；`base_link`本身是frame-only，实际底座碰撞网格归属固定子link`base_link_inertia`。映射后的七个物理link均各有1个collision mesh：`base_link_inertia`、`shoulder_link`、`upper_arm_link`、`forearm_link`、`wrist_1_link`、`wrist_2_link`、`wrist_3_link`。不居中、不重缩放、不改坐标。新建`matlab仿真/repair_and_audit_preview_robot_collision.m`，只在独立输出目录复制旧预览场景再修复碰撞几何。

对`workflow_runs/preview_20261014_121436_257478/`原171点完整预览轨迹复审。输出在`UR5/规划文件/workflow_runs/robot_collision_repair_preview_20261014/`。新场景覆盖检查PASS；qPath、timeFromStart、TCP path逐项保持一致；抽查10个姿态所有robot body FK Frobenius差为0；Gun/TCP/Wire碰撞组件数、安装变换、workpiece碰撞pose和skip pairs均未改变。原raw scene/trajectory文件未修改。

每个节点及每条播放边21个关节插值采样，共3741次碰撞查询。发现146个节点命中、3059个边采样命中，因此预览`WARNING`、`previewReady=false`、`executionReady=false`。最早节点碰撞是帧1`forearm_link ↔ wrist_2_link`（RobotSelfCollision）。最早RobotGunCollision在帧111→112边的35%位置：`forearm_link ↔ WeldingGunMount`。另有RobotWorkpieceCollision：`forearm_link ↔ WorkpieceCollision_wall_x1`及`upper_arm_link ↔ WorkpieceCollision_wall_x0/wall_x1`。全pair频次和逐帧/边采样对在`preview_display_audit.json`。最小有限正间隙0.008794 mm（`forearm_link ↔ wrist_3_link`，非碰撞对）；发生碰撞的原生距离返回NaN，没有虚构penetration depth。

单独复核了碰撞几何缺失分支：旧场景coverage=false时MATLAB状态为`PREVIEW WARNING`、collision pass=false；Python HMI状态机同样落在`PREVIEW WARNING`。修复几何后的帧111 HMI诊断显示`forearm_link ↔ WeldingGunMount`并仍为WARNING。完整MAT报告/轨迹副本和缺失几何回归分别在`robot_collision_repair_preview_20261014/`与`robot_collision_missing_coverage_regression_20261014/`。这些是诊断副本，不可执行；碰撞结果现在暴露出真实路径上的自碰撞、焊枪碰撞和工件碰撞，不能按旧Preview PASS解读。

## 2026-10-14：finishResult失败返回修复与端点/qOffset诊断

按用户授权，只修复`FastConnectAnytimeRRTStarJoint.m`的端点失败返回参数数量错误：主`RRT/jointspace/`及UR5源码副本同时将7参数调用改为6参数，并处理未创建搜索树时的空treePair。保留start/goal诊断，返回invalid_start_collision、invalid_goal_collision、invalid_start_and_goal_collision；不修改CBi-RRT*搜索、ACCP-D、TCP或碰撞阈值。两份实现各运行三种端点失败回归，6项PASS，零搜索迭代、正常返回空路径结构体。

新增离线`matlab仿真/diagnose_hmi_preview_endpoints.m`读取14:37失败run的raw_scene/request与既有47点候选，不写回输入。指定q0=[180,-90,0,-90,0,0]及全零q均无自/环境碰撞；forearm_link↔wrist_2_link间距分别14.431392/15.218746mm。实际请求起点q_deg=[305.286944,-99.668542,64.495017,83.969068,104.327246,-9.883301]检测该pair自碰撞。缓存qPreWeld_deg=[278.862708,-4.717265,-56.812594,-18.026429,-71.924571,8.263081]检测forearm_link↔WorkpieceCollision_wall_x1环境碰撞，非Gun/Wire碰撞；TCP(base)=[-356.494887,592.054404,7.315590]mm。

qOffset只用于冻结Coppelia适配器qMat=qCop+qOffset。REAL builder覆盖qMat0为SavedRealHome，preview输入真实关节值直接进入cfg.start/goal；HMI FK也不叠加qOffset，base转换只改变刚体坐标表示。四个诊断姿态HMI/MATLAB flange FK最大矩阵元素差≤2.220446e-16。未调整qOffset/Home/PreWeld或碰撞规则。完整MD/JSON/MAT及两份planner失败返回结果位于`UR5/规划文件/workflow_runs/endpoint_diagnostic_20261014_finishResult/`。未生成正式anchor/真实执行轨迹；executionReady=false；NO ROBOT MOTION。

## 2026-10-14：RealHome保存后端点验证、冷启动PreWeld与版本隔离

用户要求新RealHome立即进入规划、不再加载旧hybrid_velocity_diagnostic.mat首节点。本轮新增`gui/realhome_workflow.py`：REAL CALIBRATION+新鲜真实反馈保存六关节rad，原子写yaml兼容real_home并新增real_home_metadata（timestamp、realhome_version、URDF/robot、TCP/tool/gun/wire/collision版本与验证记录）。保存后选择SavedRealHome，重新读磁盘，不用旧内存q，清空raw/ready引用并标记旧加载结果STALE，独立QProcess运行只读MATLAB Endpoint Validation，PASS/FAIL显示具体pair。过期Home或scene回包拒收。SIMULATION下提示选择REAL后保存；没有自动修改关节角或移动机器人。UI scale保存也读取最新配置，避免旧配置覆盖新Home。

`gui/planning_workflow.py`规划请求从一次最新Home读取取q+version，带realhome_version、collision_scene_version、preweld_version=null/regenerate_preweld=true；删除旧candidate发现和seamCandidateFile请求。在线页新增Start State/Goal State，日志及结果报告同步。完成回收时再次比对Home/scene，第二阶段拒绝旧Home/PreWeld/scene版本。`plan_hmi_preview_request.m`先对显式qStart做Endpoint检查，scene.qMat0取本次request，ACCP在当前完整场景冷启动，历史仅取参数配方；任何旧candidate路径不加载。新PreWeld仅完整生成且goal endpoint PASS后保存独立run/preweld_current.json，版本绑定Home和collision scene；START/GOAL失败均不会调用RRT。新增`validate_hmi_endpoint.m`、`validate_hmi_realhome_request.m`，原skip规则和阈值不变；Robot/Gun/Wire/Workpiece分类报告准确，Wire名字中的Gun不再混淆分类。

版本元数据现在包括七个UR5实际碰撞mesh指纹与加载coverage，防旧缺本体collision的场景沿用相同版本。HMI与MATLAB当前mesh SHA、wire/scene版本一致。新增hmi_planning_dependency_snapshot/changes，在场景加载前后比较17项实际源依赖，任何内容/引用变化返回STALE+changed_files。未改变模型/安装变换/标定。主CBi-RRT*、ACCP core、TCP、gun manifest、workpiece、URDF和生产Home配置哈希前后相同。

现场当前SavedRealHome已更新（本轮未改值）：q_rad=[-1.276243034993307,-2.0179079214679163,1.5659332275390625,-1.3189204374896448,-1.3801425139056605,3.3301687240600586]，q_deg=[-73.123340,-115.617607,89.721365,-75.568575,-79.076341,190.804613]。当前场景Endpoint PASS，START COLLISION CHECK: FREE；旧14:37起点准确自碰撞拒绝，未进ACCP/RRT。GUI保存回归用临时配置+mock反馈/QProcess，生产Home字节未变；新版Home实时请求、状态显示、过期报告、SIM拒绝、清缓存、UI写配置保护与既有两阶段GUI回归PASS。MATLAB版本语义8项、源快照5类变化回归PASS。

实际HMI请求已离线送到MATLAB：新Home START PASS、不使用旧47点缓存；冷启动21层ACCP在Point14出现NO_CONNECTED_CANDIDATE，进入localCartesianBridgeFallback时第1642行bridge struct重复selected_fractions（同时重复selected_level）抛MATLAB:DuplicateFieldName。按用户“不要修改ACCP-D”限制未修核心、未关闭Bridge或放宽约束；尚未判断Bridge能否恢复该点。新PreWeld未生成，Goal State NOT_GENERATED、rrt_entered=false、executionReady=false；不能宣称完整规划PASS。下一步若处理该阻塞，应只修重复诊断字段并重新验证原Bridge，不能退回旧路径。

完整MD/JSON、原生Endpoint、实际HMI冷启动日志与GUI状态测试位于`UR5/规划文件/workflow_runs/realhome_update_regression_20261014_v2/`，入口报告`report.md`与`integration_summary.json`。实际请求/日志在hmi_current_home_preview/；新Home快照端点报告在hmi_current_home_validation/report.json。现有HMI需重新打开才能加载新代码。未生成真实执行轨迹、未修改ROS2执行链；NO ROBOT MOTION。

## 2026-10-14：Cartesian Bridge重复结构字段修复与Point14返回测试

用户明确授权只修Bridge fallback数据结构、不改ACCP搜索/IK/collision/axis/threshold或参数。生产`replan_weld_actual_axis_adaptive_core.m`仅删除bridge struct中第二次重复定义selected_level/selected_fractions的一行；原定义保留。逐字与输出目录core_before_fix.m备份比较确认只有此一行差异。原成功分支合法重复赋值未动。修改前SHA256 c672f52cf459cae1466dda5e289cc9abeddfbca76114b04c7f78cbf8621a6a11；修改后ffae45d1f8415fc8a2bb6fbcf125504a06fb31d8bbbdb4802c14362a43aa09bd。

新增`matlab仿真/test_cartesian_bridge_struct_return.m`：独立test-only源副本公开原private fallback；wrapper后核心文本与生产逐字一致，调用前observer只捕获失败输入并停止前缀，生产不加hooks。使用上一轮实际HMI只读场景及同一配方，在14.750s准确复现Point14 node6/connected0、2090 IK调用，保存scene/seam/prev/candidatePools/options和rng_state。随后恢复RNG重放原Bridge，全部已传参数逐项相等，没有stub IK/碰撞或放宽约束。

Point14 Bridge正常返回success=true、failure_reason空，2条可行分支各3个intermediate TCP poses/q节点（合计6个候选；选中路径新增3节点），fractions=[.25,.5,.75]。Bridge IK580、边检查26、各边101采样，运行15.926s。选中路径最大关节变化3.515457°、最大节点TCP误差0.000009862mm、最小正间隙0.506720mm；全attempt最小值包含失败尝试，不可混作选中路径间隙。空上一层失败contract也PASS：success=false、NO_PREVIOUS_LAYER、空paths。正常返回数据仍在bridge.paths(k).intermediate_states(j).Ttarget/Tactual/q；测试JSON另导出inserted_poses/inserted_q_nodes，生产接口不变。

证据在`UR5/规划文件/workflow_runs/bridge_struct_fix_20261014/`：report.md、bridge_return_report.json、point14_bridge_return.mat、point14_input_fixture.mat、point14_capture_and_replay.log及before备份。保护源依赖/Home/标定/模型哈希检查通过。此次只验证Point14回退返回，未继续完整焊缝、时间参数化或RRT任务，因此不宣称完整ACCP/Full Task PASS，executionReady=false。未写正式anchor/真实执行轨迹，NO ROBOT MOTION。

## 2026-10-14：REAL 地面环境导出、MATLAB加载与HMI一致显示

用户发现在线规划环境遗漏地面/安全栅栏，本轮授权只改planning scene环境层、不改CBi-RRT*/ACCP-D/IK/TCP/轨迹优化。只读盘点后用户明确回复“栅栏等我之后再加，先把地面加进去”，所以最终范围仅地面；未编造围栏/工作台/夹具模型。

新增`UR5/export_environment_geometry.py`，读取独立、停止的CoppeliaSim实例。`UR5/ur5.ttt`共19个shape：17个robot/tool/workpiece排除，环境只有`/Floor`与`/Floor/box`。主地面原生cuboid为5×5×0.20000018179416656m；子box是完全包含在主地面中的重复slab/棋盘格顶面，不创建第二个重复collision，也没有把零厚度平面伪造成凸体。源collidable位均未启用，按用户要求只在独立规划清单中启用主物理地面，原ttt未改。导出environment_geometry_real.json、environment_inventory_raw.json及全shape inventory。环境版本environment_dd72e9f8d5fbd1db，最终SHA02f81207182e3275f7ce69a3aac7e06f2cd9c26fcddefeeef3ec7c00611dbc55。

新增MATLAB load_real_environment_geometry/hmi_environment_names：严格校验米单位、rigid transform、URDF/冻结标定注册SHA与源ttt SHA；primitive按实际形状创建，非凸mesh必须可信逐组件分解，拒绝整体大凸包。build_real_planning_scene在原4workpiece完成变换后追加1个EnvironmentCollision_Floor，scene.environment共5个；原4工件索引、工具、本体、wire和skip表不变。endpoint与preview audit使用全environment名称映射，新增Robot/Gun/WireEnvironmentCollision分类。Gun-Wire原固定内部结构排除保持并报告SKIPPED_FIXED_ATTACHMENT，不冒充该pair PASS。环境JSON SHA追加collision_scene_version指纹，源JSON/ttt加入加载前后依赖快照，旧无地面轨迹自动STALE。

HMI在scene_geometry/preview_display/VTK/fallback中增加Environment显示层和紧凑开关；使用导出的精确T_base_UR5base，不复用显示近似offset。live/preview环境隔离，旧预览不借用当前环境冒充原快照；相机robot fit排除5m地面。最终当前绑定MATLAB快照与HMI的环境名称/shape/尺寸完全一致，最大matrix差1.110223e-16、位置差1.570092e-13mm；开关、FK后环境固定、工件标定隔离、live q不被预览覆盖PASS。缺fence只作为源覆盖说明，不因用户本轮暂缓fence新增gate。

实际最新HMI规划按钮请求已禁用ROS线程后捕获并送本机MATLAB离线入口：request/current binding一致、dependency加载前后PASS，但START COLLISION CHECK COLLISION/INVALID，pair为base_link_inertia↔EnvironmentCollision_Floor；Goal NOT_GENERATED，rrt_entered=false，executionReady=false。HMI完成回调准确显示PREVIEW FAILED+pair。基座STL最低z约-0.002399320mm，导入floor顶面+0.000246541mm，垂向bounds重叠约0.002646mm，是固定安装接触/网格精度候选问题；非原生penetration depth。没有下移floor、放宽threshold或skip base来取得PASS。

对用户新369帧`preview_20261014_190649_439402/full_raw_trajectory.mat`复制诊断，8097次节点/21点边采样：首帧固定base↔Floor，另外WeldingGunMount↔Floor1870样本（首frame141出边u=.9,t80.653043s），WeldingGunWire↔Floor2样本（首frame150出边u=.55,t85.499510s），wrist2↔Floor2031、wrist3↔Floor1930，另有其它link穿地/自碰撞/工具工件边碰撞。原轨迹文件不修改，正式仍STALE、preview WARNING。最小有限正距离0.109465mm仅未相交pair，不是碰撞路径的clearance；NaN相交距离不虚构signed penetration。

完整8097sample审计当时绑定旧JSON SHA04f45e…，之后补齐源inventory元数据导致最终SHA02f812…；几何版本不变。旧audit/snapshot保持原绑定不静默改SHA，另current_binding_audit逐项证明全部5native environment collider几何/pose完全相同、所有原robot/tool/workpiece/skips一致，并重构当前绑定1帧快照。HMI最后回归使用此02f812快照，environment_binding_pass=true。Synthetic loader回归检查Robot/Gun/Wire对Environment及1mm错frame STALE分支PASS；既有两阶段GUI回归PASS；Python编译/MATLAB代码分析通过。原model comparison所有body FK差0，原碰撞组件/安装变换/关节限位/qOffset/skip/4workpiece pose均一致。

报告与回滚数据在`UR5/规划文件/workflow_runs/environment_extension_20261014/report.md`，实际按钮请求/日志/端点在current_online_replan/，完整逐帧pair在real_scene_audit/preview_display_audit.json，当前绑定证明在current_binding_audit/environment_binding_refresh.json，最终截图+VTK核对在final_current_matlab_hmi_display/。代码仅新增环境export/loader/显示/审计与版本元数据，不修改算法或控制链。原scene_geometry.json、ur5_tool_calibration.mat、URDF、枪/TCP/工件、ttt不覆盖；source ttt SHA仍1e2b295f479d96ea51489e7bea7425dc832748695b2c0941ce69315afd642644，ACCP core SHA仍ffae45d1f8415fc8a2bb6fbcf125504a06fb31d8bbbdb4802c14362a43aa09bd。重新打开HMI加载本轮代码。未生成真实执行轨迹，NO ROBOT MOTION。

## 2026-10-15：核实固定底座—地面安装接触、仅单对例外与真实冷规划

用户授权：先核实base_link_inertia↔EnvironmentCollision_Floor是否正常固定安装接触，确认后仅该对建立例外，再从零跑CBi-RRT*/ACCP；运动部件/枪/丝的地面碰撞保持硬检查。没有授权全局忽略地面或移除底座碰撞网格。

新增verify_fixed_base_floor_contact读取原URDF collision STL及origin：base_link_inertia经两个fixed joints到world，无运动祖先；10组关节角底座FK差0；完整处于水平5m地面范围。垂向bounds重叠约0.002646mm，符合模型固定支撑接触。0.01mm是固定安装分类核对带，不是规划collision阈值；未移动floor/机器人/工具或修改任何实物参数。证据明确physical_installation_independently_measured=false，不宣称实物测量；仅按当前真实场景模型/固定安装假设认证，现场仍需确认安装与注册一致。

独立UR5/fixed_environment_contacts.json版本fixed-base-floor-20261015-v1（SHA a787e02ffe3bd27e5ef49eb5d3f46240eb3c8125e4b4dc3e406d1d1841f0adfb），严格绑定URDF/environment/base.STL。load_fixed_environment_contacts每次REAL scene构建重新核实模型支撑。新增RRT/checkEnvironmentCollision对原生exhaustive结果只排除base_link_inertia行×该Floor列，保留raw_contact并记录EXEMPT_FIXED_INSTALLATION_CONTACT；其余distance matrix单元完全不动。允许pair设Inf而不是0，避免污染最小间隙。保留原self skip、Gun-Wire固定内部排除；base对其他障碍物及所有运动link/Gun/Wire对floor均检查。取消policy文件则恢复原生严格检查，不清除任何body collision geometry。

生产接入仅碰撞后端及元数据：公共collisionFreeJointState/Edge增加可选policy，CBi-RRT* config default/normalize及调用透传；ACCP core两处environment query、actual-axis候选、Cartesian Refinement和现有controller/time-compare碰撞查询接同一后端。没有改变搜索/IK/轴/step/error/limits/采样/优化数学。core已用verify_source_scope.py反向恢复两处调用后逐字SHA精确等于先前ffae45d1…；当前core SHA8560bb219174115f52b42f439f78bdc38559728a72a29547d7ce770b738429a5。RRT六份对应源码副本同步后端，历史MATLAB算法档案保留未覆盖。TCP/URDF/枪/丝/工件/地面/frozen ttt/scene_geometry源SHA均未改变。

policy SHA纳入REAL collision_scene_version、HMI/MATLAB绑定及源依赖快照；REAL新scene version SCENE-20241f44dfca1a55fad4a387ca3637ce2793c8ccf450706f65a6cd95714ea3ef。冻结仿真digest保留原语义，旧真实轨迹继续STALE。Endpoint保存raw base-floor接触证据，audit snapshot也保存固定接触policy。没有改ROS2、STOP或执行链。

最终负向回归PASS：Home严格模式FAIL/单对例外PASS；10组状态只有指定矩阵单元改变；全零姿态wrist2/wrist3/Gun/Wire↔floor仍拒绝；base↔其他fixture障碍仍拒绝；floor移动1mm、改名、base移动1mm均拒绝例外，任意扩展shoulder-floor例外拒绝；robot colliders/FK/environment pose未改。MATLAB静态检查无语法错误，Python编译和实际HMI完成回收/STALE/执行门关闭测试PASS。生产Home YAML字节未变。

实际HMI请求cold_full_preview_v2从当前真实完整scene开始：Start/Goal/Exit Start/Exit Goal全部PASS，ACCP从零（不加载旧qPath/statebank）；原21点第17层断裂由既有Cartesian Bridge插3点恢复至24粗点，后续全部焊点完成，细化47点，最大TCP几何误差0.244253mm，节点/边collision PASS。配置IK2426，配置时间94.711291s，总ACCP302.859404s。CBi-RRT*接近30.003295s（15原节点、21+62迭代）、退出30.001662s（13原节点、2+59迭代）均timeout_with_solution，表示anytime预算耗尽但有解。

新完整预览173节点=66approach+47weld+60exit；3785原生节点/边采样、173节点本身无碰撞，所有运动link/Gun/Wire的Floor碰撞数量0。因此本轮固定支撑与非预期地面碰撞区分已生效，未沿用旧穿地路径。但完整预览仍WARNING：18个工件碰撞边采样，全部在66→67帧approach→weld接口。J6从371.227115°到11.227115°，raw Δ=-360°，wrapped Δ≈0，TCP XYZ/R差0；lift_continuous_joint_path后的圈数表示与焊接段未统一，插值出现整圈旋转。首次u=.05/t39.833782s Wire↔WorkpieceCollision_wall_y1，后续Gun↔bottom/wall_x0/wall_x1/wall_y1、Wire↔bottom。预览lift后的63–66帧J6分别362.693/365.538/368.382/371.227°越上限，原RRT节点限位则PASS。没有擅自改变CBi-RRT*或拼接/圈数规则来消除该剩余问题；应下一轮在轨迹表示/边界层诊断修复，不可把周期等价当实际运动安全。

ACCP本次仅几何阶段PASS，时间参数化依旧FAIL：cubic Hermite轮廓0.406375mm、同步2.055985mm、姿态0.738561°、速度0.091117459rad/s、加速度0.835528804rad/s²；其中碰撞/限位PASS不等于整体验收。没有重用旧47点D速度或自动调整算法阈值。完整previewReady=false、executionReady=false，未运行连续执行准备、未生成execute-ready或正式anchor、未发送任何机器人运动。3mm焊丝碰撞缩短保留原近似、独立安全余量未认证，采样检查不是连续安全证明。

证据目录UR5/规划文件/workflow_runs/fixed_base_floor_contact_20261015/：report.md、fixed_support_verification.json、fixed_contact_regression.json、source_scope_verification.json、cold_run_summary.json及hmi_completion_test.json。实际MAT请求/日志/ACCP/RRT/完整pair在cold_full_preview_v2/。初次cold_full_preview因新诊断器把YAML列向量用于vertcat而失败，已只归一化诊断输入row、在v2新目录完成冷规划；初次日志保留不覆盖。HMI需重启加载版本/后端；不允许把当前WARNING轨迹用于真机。

## 2026-10-15：接近/焊接数值圈数接口修复，时间失败独立诊断

范围仅接口包装与离线验证；CBi-RRT*/ACCP-D/IK/TCP/URDF/枪丝工件地面/skip/阈值/ROS2执行链不改。旧预览approach末端J6=371.227115°，weld首点11.227115°，数值差-360°；其他5轴一致。根因为preview freePath无条件lift后未检查目标圈数与限位。退出端也被lift成-169.198133°，而Home是+190.801867°。

plan_hmi_preview_request改用hmi_plan_interface_free_path，有限重试原规划器并记录seed；hmi_checked_free_path_branch把周期lift仅作候选，严格钉定start/goal数值、真实限位、按实际数值边验碰。若原始数值边含大旋转，也完整采样，而不是取模。新hmi_audit_joint_interface_path每边≥101点、角采样≤0.5°，使用同一scene固定接触policy和self skip。不能整体平移焊接段±360°（上限371.227°、下限-366.802°），也不能平移approach改变Home数值。

最终几何预览359点=165approach+47weld+147exit，7877节点/边采样0碰撞、所有节点限位PASS；接口6轴差0、返回Home数值一致，47个ACCP q/TCP/time保持不变。接口相邻3边303点PASS，最小正pair距离0.628870mm；全预览最小正距离0.231724mm，不是独立物理安全余量证明。原3mm焊丝碰撞缩短不变。接近实际原始J6长边329.109496°经过1973状态验碰PASS，累计旋转478.644240°；退出累计425.188596°。当前结果仍有大绕转，不是最短规划；不能把它说成取模后无实际运动。新seed18/19/20的部分候选实际碰工件被拒绝。正式新HMI接口入口重新调用原CBi-RRT*，所得165点approach与修复预览逐值差0。

接口通过后才运行prepare_hmi_continuous_request。当前47点25.135842s的weld，不同于旧D快照，禁止复用历史速度。原速度规则解析sync2.055985mm/ori0.738561°/v0.091121156/a0.848252593。现有C速度改善至contour0.182934mm、sync0.568271mm、ori0.548976°、v0.084158503、a0.261796382，collision PASS，但同步/姿态仍超0.5mm/0.5°。Hybrid要求可行C初始解，因此本轮停在VELOCITY_OPTIMIZATION/PREPARE:TimingInput，未进入完整≥5000点cubic验收，不生成execute-ready，executionReady=false。新增包装诊断保存base/audits和精确失败原因，未修改速度/时间优化算法。

prepare增加旧越限/整圈接口前置拒绝，不把周期表示错误送Cartesian Bridge。回归包括实际360°边721点拒绝、错误等价起点拒绝、合法小步通过、旧preview准备拒绝。HMI MAT loader/节点验证与状态机PASS：Preview Ready→Preparation Failed后359点预览保留、无ready path；无ROS节点/action/GUI机器人worker。16份受保护源/模型/配置SHA均不变。

全部证据UR5/规划文件/workflow_runs/interface_branch_repair_20261015/：report.md、summary.json、protected_before.sha256、replanned/full_raw_trajectory.mat及逐帧pair、workflow_entry_replan/正式入口复验、timing/诊断和日志、regression/及hmi_state_regression.json。旧源目录/正式anchor/冻结文件未覆盖。后续应单独优化当前q/time下速度精度及自由空间大绕转，不得把本轮几何PASS当成执行认证。NO ROBOT MOTION。

## 2026-10-15：完整轨迹控制器插值误差诊断与离线优化

完整任务的原始控制器等价验证发现同步误差2.816926 mm、姿态误差0.760377°；焊接段47个离散目标本身未变，主要误差来自节点速度/时间分配下的positions+velocities cubic Hermite插值，另有接近段起步的短时间段影响。当前只读控制器快照确认 `scaled_joint_trajectory_controller` 使用 position command、`interpolation_method=splines`、125 Hz，JTC安装头文件说明positions+velocities为cubic spline；HMI发送层保留time_from_start和velocities。没有假设控制器自动消除误差。

新增 `UR5/matlab仿真/hmi_cubic_tracking_diagnostics.m`、`optimize_hmi_cubic_node_velocities.m`、`optimize_hmi_controller_continuity.m` 及诊断入口。固定原47个ACCP焊接q/TCP目标，只在接近/退出增加57个执行插值节点；自适应局部时间分配和节点速度优化后，414节点候选在41301个控制器等价采样点上达到同步0.461499 mm、姿态0.459695°、轮廓0.224258 mm、最大速度0.0819511 rad/s、最大加速度0.259982 rad/s²；两个段接口执行C2约束，节点限位与UR5/焊枪/焊丝/工件/地面碰撞均PASS。总255.040840 s，最小有限正模型间隙0.193050 mm，仅为采样几何距离，不是物理安全余量证明。

J6没有取模：接近段约329.109496°实际长旋转仍保留并按真实数值采样验证；最终完整路径J6累计绝对运动约921.862300°，较长时间是关节约束结果。

证据目录：`UR5/规划文件/workflow_runs/continuity_optimization_20261015/`，主报告 `report.md`，最终离线审计 `full_c2_dense_audit.json/.mat`，诊断候选 `full_candidate_c2.mat`。所有结果 `executionReady=false`，没有生成trajectory_execute_ready文件，也没有发送ROS运动。wire_collision_shorten_mm=3仍为近似参数；真实控制器伺服响应和物理安全余量仍需人工/真机验证后才可考虑升级。

## 2026-10-16：HMI预览专项性能剖析与优化（相同REAL场景/种子）

用户提供 `preview_20261016_124729_001415`：595.276691s，ACCP 307.005957s（构型91.824448s、Cartesian Refinement 187.138561s），359节点、7877审计采样。目标仅降低“规划轨迹”预览耗时，保留全部几何/碰撞/限位/精度要求，执行准备延后；本轮全程离线、executionReady=false。

先给生产入口 `plan_hmi_preview_request.m` 补分段计时，并运行完整MATLAB profiler；剖析886.201862s有明显探针开销，不参与性能均值。确认：3094次refinement hardState反复读取robot.Bodies，剖析266.159s/precision303.567s约88%；整个请求50550次Bodies getter和约81万次刚体对象构造主要由此产生。构型搜索的dense graph/Bridge重复状态碰撞是第二类成本；原主表IK2426没有覆盖所有评分/恢复/细化，完整实际IK solve为3384。原CBi-RRT*接近/退出各3次，每次30s，纯搜索180.017s；首解只需0.64–1.40s。原报告仅显示选中的30s，漏看其余尝试。预览还提前运行了约28s时间缩放/控制器审计，其中fminsearch本身很小，主要成本为4601控制器采样碰撞。

生产变更：refinement把不变限位读到循环外；新增HmiExactSceneQueries对同一请求/不可变场景的原生环境、自碰撞矩阵和FK作完整num2hex关节键缓存，不取模/近似，不跨请求复用，绑定Home/qStart/qHome、全部模型/标定/环境版本、scene SHA、skip/limits和碰撞规则源SHA。新增缓存不改变原CBi-RRT*/IK内部既有缓存语义。原生collisionFreeJointState/Edge、checkEnvironmentCollision、check_scene_environment_collision、REAL scene loader、Adaptive Time Scaling数学、controller interpolation validator源码SHA未变。core/generate候选仅接性能查询封装，搜索/IK/轴/阈值不动；缓存handle从结果scene中移除，不写入场景/轨迹。

CBi-RRT*新增明确opt-in `stopAfterFirstSolution=false`默认，历史anytime实验仍按原逻辑运行；HMI优化预览打开此选项，首解仍必须通过原数值起止钉定、真实关节限位和≥101点/≤0.5°物理边审计。失败分支拒绝；必要时有限尝试后保留原anytime恢复。不能把periodic planner首解直接当作合法真实运动。完整preview每边21点不减少。审计最近距离改为保持column-major tie顺序的向量化最小值；缓存版和关闭缓存版逐帧验证一致。

预览只设置core `deferTimeParameterization=true`，保留Continuous IK/Cartesian Bridge/Cartesian Refinement完整几何验收；qPath/TCP不变，animation clock标DISPLAY_ONLY，焊段标time_parameterization_deferred，timing complete=false明确为DEFERRED_TO_PREPARATION。其它core调用默认false，仍执行原Stage3。prepare入口通过新增hmi_prepare_deferred_weld_timing恢复原时间算法，再走原边界/Hybrid/控制器/版本门。回归证明重新计算q/TCP不变、timestamps/velocities逐值等于原算法，二次处理同一内存段不重复计算；原时间初始方案仍FAIL（sync1.486543mm、orientation0.738518°、v0.08804385、a0.84365559），完整失败原因保留，未冒充执行认证。

未剖析旧流程重新运行597.130360s，与用户595.276691s一起均值596.203526±1.310742s（样本std，n=2）。新跑旧流程ACCP构型95.266411s、precision187.933020s、timing28.298471s；算法外285.632458s分布为RRT纯搜索180.026s、物理分支审计50.765s、完整审计46.486s、scene/version4.727s、ACCP载入/装配/保存剩余2.708s、其它0.922s。

优化两次独立新缓存冷ACCP为191.180807s、185.378401s；另外通过真实MainWindow的“规划轨迹”QPushButton→QProcess→MATLAB→完成回调/预览跑第三次192.419892s，点击到显示197.537485s，文件采用0.056280s。GUI测试仅禁用ROS工作线程，未模拟MATLAB/规划/报告。三次规划均值189.659700±3.759119s，降低68.1888%、加速3.1435倍；成功率基准2/2、优化3/3，只是固定场景/同seed20260917重复性，不外推多seed总体成功率。优化构型均值81.313s、precision13.751s、preview timing0（延迟）、approach含物理审计17.549s、exit15.476s、full audit53.527s。

三次焊段原47个q/TCP目标逐值等于原结果，最大差0，TCP几何最大0.244252934mm、node0.000142558mm、几何姿态0.018085902°，最大相邻数值关节步长7.118994°、所有node limits PASS。首解策略自由空间可能较长，新188approach+47weld+181exit=416点，原165+47+147=359点；不宣称路径更短/执行更快。完整9131原采样0碰撞，机器人本体/枪/丝/工件/地面coverage PASS；独立关闭缓存复审9131同点，逐帧状态/碰撞pairs/最近对象与距离完全一致（max gap delta0）。最小有限正模型间隙0.402180626mm不是物理安全余量/连续安全数学证明；wire shorten3mm近似保留。原生矩阵/FK缓存一致性、1e-12rad不近似命中、新版本实例冷缓存、首解模式碰撞起点拒绝均PASS。HMI实时关节未被预览覆盖，原YAML未写入。

证据根 `UR5/规划文件/workflow_runs/preview_performance_20261016/`：report.md/performance_result.json总报告；comparison.json三次逐值/版本/约束比较；baseline_profile/含完整profiler；baseline_unprofiled/未剖析旧流程；optimized/两次优化；native_reaudit/和native_reaudit_regression.json；deferred_timing_regression.json；exact_cache_regression.json；hmi_button_e2e.json及真实HMI第三次目录 `preview_20261016_142704_413362/`。首次native复审末尾比较脚本因JSON空数组double与MATLAB空cell的isequal类型差异报断言，实际全9131采样PASS；已用verify_hmi_preview_native_reaudit逐帧规范比较并保存PASS，不改审计数据/碰撞规则。baseline_sources/与baseline_source_manifest.json保留优化前MATLAB/RRT源码。README同步；每次report记录performance、accp_performance、audit_performance、全部interfaceTrials；HMI另存hmi_transfer_performance.json。请求performance_mode='baseline'可离线恢复旧工作量作对照。未生成正式anchor或execute-ready轨迹，未改ROS2执行链路，NO ROBOT MOTION。

## 2026-10-16：连续规划离线PASS与物理/伺服待验收分离

用户看到PREPARATION FAILED但controller validation PASS。准确根因：原prepare_hmi_continuous_request.m第149行在所有离线/版本门通过后主动抛PREPARE:OfflineOnly，catch把success=false；原planning_workflow.py第272行及状态类又把executionReady=false一概当作连续规划失败。修改前3个源文件保留在 `UR5/规划文件/workflow_runs/continuous_state_20261016/before/`，行号可核对。

只改返回契约、HMI状态及诊断导出：CONTINUOUS_PLANNING→PHYSICAL_VALIDATION_PENDING（“连续规划完成（离线验证通过）”；“真机执行准备：待验证”）。真正验证失败显示CONTINUOUS_PLANNING_FAILED及具体failed_stage/reason。EXECUTION_READY独立保留，本离线入口没有授予路径；旧execution-ready导出分支移除，所有结果preview_only/diagnostic_only=true、executionReady=false。新增hmi_export_offline_continuous_result汇总原审计并导出offline_optimized_trajectory.mat/json、FK误差CSV；GUI新增本地报告/误差图查看及白名单离线ZIP保存。优化节点的显示FK数组按已保存scene重算，逐帧碰撞状态源自完整controller采样，不复用旧帧数组；TCP路径已变换则不重复跟随工件变换。绘图失败不会覆盖数值成功；早期未运行碰撞检查显示NOT RUN而非凭空FAIL。

实际按钮→QProcess MATLAB→完整优化/验证→HMI回调通过，目录 `prepare_20261016_145333_894143/`。414优化节点、254.954031717s；41301控制器等价cubic采样，轮廓max0.223461146mm/RMS0.023218212、同步max0.462406673mm/RMS0.053454208、姿态max0.273888491°、解析vmax0.0834013475rad/s、amax0.261687136rad/s²，碰撞/限位/版本均PASS。两处任务接口C²通过、最大接口加速度跳变5.56e-11；**全局不保证所有节点C²**，最大全局加速度跳变0.405732520rad/s²明确报告。段内jerk max4.312066613/RMS0.181580035，与节点跳变区分。最小正模型间隙0.195796703mm不是独立物理安全余量；wire shorten3mm近似不变，实际伺服与物理验收待做。

回归根 `continuous_state_20261016/`：report.md含根因/源行号/变更/指标；hmi_actual_pass.json实际按钮成功及另存ZIP；hmi_actual_fail.json对独立TEST COPY故意越限，MATLAB真实返回PREPARE:InterfaceLimits、GUI失败且预览保留；contract_tests.json共16项假PASS/超限/NaN/采样/版本/插值/执行许可反例全拒绝；fresh_hmi_replay.json全新HMI加载真实结果、首中末帧显示FK/碰撞一致、报告按钮和5项坏报告反例通过。所有执行按钮禁用、ready_path=None，未生成trajectory_execute_ready文件。截图为offscreen骨架，不是真实VTK桌面图。测试ROS线程关闭，未模拟MATLAB优化/PASS。

注意用户在原报告之后14:42又保存了新RealHome（REALHOME-1758…→REALHOME-00fce…），旧预览在当前配置中正确STALE。回归成功用例因此在隔离测试YAML中重放原报告的真实历史Home及readiness快照，不改当前生产配置，不把历史PASS当作当前新Home或真机认证。生产YAML逐字节未改；8项受保护算法/REAL场景loader/优化器/controller validator/TCP/枪/工件/ROS发送层SHA不变。CBi-RRT*/ACCP/IK/碰撞几何/阈值/过滤/ROS2链路不改。需重启HMI加载新状态机，旧错误报告不改写。NO ROBOT MOTION。

## 2026-10-17：ACCP加速后预览失败调查——Bridge重新引入硬方向约束拒绝端点

失败请求preview_20261016_144235_155027与原性能成功请求并非同输入：seed20260919（原20260917），RealHome版本00fce…（原1758…），J2差-0.426149°；TCP、枪/安装、工件、URDF、地面、固定接触、焊丝及collision scene绑定全部相同。未改用户Home或模型，10月17复核当前配置仍与失败请求一致。

准确根因早于性能优化：原始21层在Point17 NO_CONNECTED_CANDIDATE，Bridge插3层恢复24层，graph119测试/83通过、configurationComplete=true。选中的原Point17 Bridge终点（扩展稀疏节点20、失败细化诊断节点38）实际轴误差5.0000000335531976°，严格上限5°；原数值/位姿诊断池159已经标rejection_reason=orientation_direction。但core中Bridge恢复diagnosticCandidatePools端点只调用localFullStateFree（碰撞），没有重新检查方向、焊接侧、位姿等节点硬约束，故重新引入了已拒绝记录。Refinement严格门正确拒绝，稀疏边19/20、输出诊断边37/38失败，最大TCP几何误差0.656519mm。45诊断点不等于合法qPath，公开qPath为空、adaptiveCorrectionComplete/complete=false。两条失败边额外101/113点原生审计零物理碰撞，各只在共享端点方向拒绝1次，不能将segmentCollisionFree（混合hardState）false误报成物理碰撞。PreWeld候选单点FREE，但原请求没有合法完整焊缝、未导出正式PreWeld，Goal=NOT_GENERATED。

先不改生产代码，在冻结失败scene/Home/seed/options下缓存开启/关闭各5次：全部PRECISION_FAILURE，10次稀疏q、45点诊断q和TCP目标逐值完全一致；缓存开均值105.093s，缓存关135.055s（只ACCP；关缓存诊断类Enabled=false、保留独立限位读取上提）。原性能优化前归档core+refinement同输入反事实重跑316.811s，同样失败边19/20、0.656519mm，q数组逐值一致。故不是精确缓存随机错误/近似键/跨请求合法性污染，而是不同输入触发原Bridge漏检。原缓存类/IK候选生成器/Cartesian Refinement生产SHA均未改。

独立源码副本验证通过后，最小生产修复：replan_weld_actual_axis_adaptive_core.m:1771加入端点池硬门复核，新localBridgeEndpointHardValid(:1966)用与原生成器/细化相同的FK、关节边界、位姿、物理枪轴、焊接侧不等式，不增加5°容差、不重新IK或改q。14个去重Bridge端点拒绝9个，保留其他合法分支继续全部焊点。未改变主连续搜索策略、RRT、IK数学或参数。新增hmi_accp_preview_status.m和preview入口:92分类configuration/geometry/precision/schema，日志中已恢复的Point17事件不再被当成最终失败点；原部分成功仍不允许Preview Ready。6种状态/假成功反例分类测试PASS。

正式plan_hmi_preview_request相同失败请求连续5次完整预览PASS：耗时212.906634、212.381742、203.566355、197.335677、196.789895s，均值204.596061±7.816523s（样本std，n=5）；ACCP构型+细化均值60.173428s。每次189接近+47焊接+178退出=414点，START/GOAL PASS、关节限位PASS、9087采样零碰撞，最大TCP几何0.244252901mm，严格轴门通过；五次47个q/TCP目标逐值相同。修复后缓存关闭再冷ACCP87.126s PASS，q/TCP与完整预览逐值一致。旧性能成功请求重新完整运行173.498851s、416点/9131采样PASS，旧47个焊接q及TCP目标逐值未变。

10月17又通过真实MainWindow“规划轨迹”QPushButton→QProcess MATLAB→回调/自动加载/预演验证：MATLAB195.674063s、按钮到Preview Ready200.527280s、414点/9087采样PASS；同seed20260919和RealHome00fce…，只禁用ROS工作线程，未模拟MATLAB或PASS。生产YAML逐字节不变、实时关节状态未被覆盖、执行门BLOCKED。原189.660s基准是旧Home/seed17，不能把新输入204.596s当作严格同条件比较，也不能拿原102.352s提前失败时间与完整成功流程计算提速。旧成功请求单次173.499s表明未退回596s原流程，但不外推广泛seed总体成功率。

全部证据根 `UR5/规划文件/workflow_runs/accp_stability_20261016/`：report.md/result_summary.json；saved_failure_audit/saved_result_audit.json实际字段/候选拒绝/失败边FK；offending_candidate.log原被拒绝端点；saved_graph_replay.json逐层图边回放119/83完全匹配；failed_layer_statistics.csv；cache_on/cache_off各5次MAT日志及frozen_cache_comparison.json；preperformance_case/；diagnostic_patch_run/；fixed_full_preview/五次正式入口；previous_success_regression/；fixed_cache_off/；final_numeric_regression.json；stage_status_regression.json；hmi_button_e2e.json及真实GUI目录preview_20261017_220611_847885/。before/保留原两个生产源；生成的独立诊断源码副本不参与生产路径。候选池计数含重复种子/缓存记录；Bridge共享586次IK在原MAT无逐层细分，报告明确不捏造；保存池真实碰撞对来自距离矩阵，不将聚合布尔flag误当对象索引。

正式生产改动仅上述core节点复核、preview分类入口及新增状态解释helper；测试工具e2e_preview_performance.py只新增可选seed参数。未改碰撞模型/过滤/阈值、TCP、几何、ROS2执行链；未恢复旧非法PreWeld、未覆盖正式anchor/冻结基准。time parameterization依旧DEFERRED_TO_PREPARATION，所有executionReady=false，无trajectory_execute_ready许可文件。模型小间隙与wire shorten3mm仍需物理安全验证；采样通过不是连续安全数学证明。NO ROBOT MOTION。

## 2026-10-19：日常连续规划与四方案速度实验分离

用户要求将四方案实验移出日常按钮，并询问真机执行方式。只改连续规划包装和对比审计开关，不改优化数学或执行门。`generate_accp_adaptive_velocity_profile`新增`compareStrategies=true`默认保留独立实验，HMI明确false，只审计C候选；基线/A/B明确NOT_RUN_COMPARISON_ONLY，A/B仍为Hybrid参考。`optimize_accp_hybrid_velocity`新增`compareReferences=true`默认，HMI false时只移出A/B对比审计，C初始解/D候选仍验证。原恢复/Hybrid选择不变。新增显式离线入口`diagnose_hmi_velocity_comparison`，读取某次prepare记录的精确时间输入/场景，检查SHA与绑定，另建诊断目录，不覆盖历史或发布ROS指令。

同一20261019_090735预览输入，实际HMI按钮回归`prepare_20261019_093204_279082` PASS：MATLAB469.661168s；原`prepare_20261019_091210_620739`为546.832251s，单次减少77.171083s/14.1124%，不外推多次均值。最终469节点、47焊接节点、46801完整碰撞样本不变；全流程控制器碰撞审计从69806减为56003，少13803个非最终基线/A/B审计。严格isequal确认全部参考速度、C审计、最终q/TCP/时间/速度和controller_validation逐值一致。轮廓0.292624132mm，同步0.461103141mm，姿态0.346849579°，vmax0.086485055rad/s，amax0.260089124rad/s²；所有离线门PASS，executionReady=false。

证据根`UR5/规划文件/workflow_runs/continuous_daily_20261019/`：report.md、before/三个旧生产源、numerical_regression.json、hmi_actual_pass.json（实际按钮/进程/自动载入/预演/另存，ROS线程禁用、记录场景回放，非物理认证）、hmi_restart.json、16项offline_contract_after.json、合成保持轨迹comparison_modes_unit.json（默认完整实验及C-PASS Hybrid分支，非真实UR5验收）。合成夹具初版缺显式parent过滤导致重复MATLAB警告，已仅修测试夹具并重跑，真实过滤未改。生产YAML/实时关节状态不变，所有执行按钮禁用，无许可文件。

新增`ur5_hmi/docs/continuous_planning_and_execution.md`并链接README：明确当前PHYSICAL_VALIDATION_PENDING产物不能执行，现有软件尚缺针对这类离线产物的现场物理/伺服验收→正式执行许可入口；不能手改preview_only/executionReady代替验收。现有轨迹页可发送已批准正式轨迹，但“低速执行（5%）”文本不是任意轨迹自动5%重定时；trajectory_to_ros_message直接传文件时间/速度，本轮未改该链路。CBi-RRT*/ACCP/IK、TCP、几何、碰撞密度/阈值/过滤、Home、ROS2执行链不变。NO ROBOT MOTION。

## 2026-10-19：用户要求简化并补齐轨迹执行入口

用户先要求加入真机执行准备区域，开发过程中又明确取消多项验收/许可设置，希望规划后人工执行，说明现场具备物理急停。最终采用“检查执行条件→人工二次确认→现场调试执行”，不再要求外部物理/伺服报告、草稿或独立授权文件。中途多表单方案已撤出运行路径，源码/截图仅归档在`execution_preparation_20261019/advanced_ui_superseded/`，不是最终界面。

`gui/execution_preparation_panel.py`自动关联同目录report.json（通常无需操作），提供简洁状态及必要时选择报告；`safety/execution_preparation.py`验证完整离线报告、原始MAT/JSON参考、内存q/速度/时间、REAL场景、RealHome及全部标定/几何/环境版本、ROS关节顺序/弧度端点及已验证P+V插值字段。源文件哈希按精确stat identity缓存，不跨变化沿用；内存摘要基于全部float64字节。日常点击不重复8分钟优化，复用绑定同一不可变结果的完整离线证据。

`main_window.py`轨迹页按钮改成“加载轨迹 / 检查执行条件 / 3D轨迹预演 / 确认并执行（按轨迹速度）”，不再以“5%”误导实际重定时。FINAL标记明确为软件PASS。原实时软件门（反馈/真实链路、程序、模式、安全状态、正倍率、controller active、action ready、1°起点、并发及版本/碰撞/限位）保留。每次发送弹出原四项现场确认，并在确认结束后再次检查；任何失败阻止发送。旧preview-only一律阻塞分支现在只对完整软件证据通过的PHYSICAL_VALIDATION_PENDING输出开放人工现场调试，不将实际物理/伺服验收标成PASS，原metadata executionReady=false/preview_only不改，不导出正式验收许可。这一策略变化来自用户本轮明确要求，不等于安全等级认证或实际跟踪精度已验收。物理急停不替代必要的软件门。

证据根`UR5/规划文件/workflow_runs/execution_preparation_20261019/`：report.md、simple_execution_tests.json（真实469点结果+SYNTHETIC READY状态，仅验证按钮软件逻辑）、25项失败反例拒绝、实际确认对话框四项缺省未勾选时不能确认、取消及确认期间掉线两条拒绝路径无发送、原MAT/生产YAML逐字节不变。100/125/150%离屏GUI布局通过，新区域宽不超过右半屏viewport；截图simple_execution_*.png为无真实反馈pending状态，左侧骨架回退不是VTK真机截图。原16项离线反例simple_offline_contract.json与旧两阶段重载回归仍通过。未调用成功发送分支，motion signal=0；未启动ROS线程或发送真机运动。before/main_window.py保留改动前版本。

CBi-RRT*/ACCP/IK、TCP、机器人/枪/丝/工件/地面模型、碰撞阈值/过滤和ROS2发送器/控制器未修改。操作与限制更新在`ur5_hmi/docs/continuous_planning_and_execution.md`。重新打开HMI后生效；NO ROBOT MOTION。

## 2026-10-19：连续规划点击后HMI无响应——主线程状态刷新积压

已证实是新增执行准备区域的刷新回归，不是MATLAB未启动。原始414点preview在planning_busy时，30次_on_state性能分析共3.562s、平均118.731ms；RealHome YAML读取120次（原本1次/帧变成4次）、execution_context90次、binding150次，YAML解析占84.5%。ROS worker按20ms回调发状态，GUI处理不过来，QProcess进度/完成也积压。真实prepare_20261019_103553_826984的MATLAB于10:43:44已写PASS报告，HMI完成通知10:55:58才处理，延迟约734s；不能把1204.222s进程/传输计时当作算法耗时。离线46801采样，轮廓0.266538mm、同步0.478806mm、姿态0.299963°、vmax0.083001283、amax0.259063731，碰撞/限位PASS，executionReady=false。

最小修复仅gui/main_window.py、gui/execution_preparation_panel.py：_on_state只存最新快照/记录执行采样，100ms显示定时器合并重绘，失去就绪立即禁用执行；规划中不重复执行证据/整路径软件验收，门明确BLOCKED而非假PASS；raw preview在读配置前拒绝进入执行上下文；去重软件原因查询，删除额外1秒全门轮询；已验证上下文按配置内容SHA等身份复用，Home文件变更会失效，保存读取格式未改。发送前最新状态/完整软件门及人工确认仍在，不改ROS2收发或任何模型/算法/阈值。

证据根`UR5/规划文件/workflow_runs/hmi_responsiveness_20261019/`，report.md记录源文件与限制。修复后槽平均0.00314ms，完整显示同样cProfile平均27.230ms且100ms一次，30帧只读Home30次。Qt实际按钮+真实QProcess回调+已保存真实MATLAB结果重放：骨架及生产VTK/X11均PASS；VTK按钮44ms返回，计算中最大心跳间隔61.3ms，取消263ms，完成通知正常；完成回调含图表渲染仍有2.622s短停顿，未宣称完全异步。合成未就绪50Hz状态、ROS/RTDE关闭，仅UI回放非新规划/物理认证。26项软件门反例拒绝，包括缓存后Home文件变更；忙时执行禁用，100/125/150%布局通过，发送信号0。原MAT/report/生产YAML未由测试修改。用户在调查期间另外保存了新Home，旧报告不能作为当前版本执行许可。初版VTK测试曾因临时Python方法被Qt定时器弱引用后释放导致夹具崩溃，持久全局测试回调修复后VTK通过，生产VTK未改。

旧HMI仍需退出重开；不要同时保留多个实例。未主动强制退出用户HMI、MATLAB、驱动，也未重跑或发送真机轨迹。ROS trajectory_client.py、trajectory_utils.py及MATLAB controller validator SHA与改前一致。NO ROBOT MOTION。

## 2026-10-19：规划结果自动导入、易读命名、78份旧HMI轨迹进回收站

用户要求规划完成自动导入执行页、明确文件名并删除旧轨迹。原先回调已共用loaded_trajectory但没有明确提示/导航。本轮新增trajectory/result_store.py：发布字节一致MAT/报告副本到`规划文件/当前轨迹/`，名称`预览轨迹_仅预演_日期_时间_序号.mat`与`连续规划完成轨迹_日期_时间_序号.mat`，配套`同名_report.json`和`最新轨迹索引.json`。源请求仍是scene/snapshot/JSON参考依赖，不是自包含包；索引记录source_run。新预览取消旧连续规划的“当前结果”选择。

gui/planning_workflow.py成功回调自动发布/导入，连续规划完成自动切到轨迹执行页；main_window.py显示实际易读文件名，“导入轨迹…”默认选中最新结果，带报告的预览导入可恢复连续规划入口；execution_preparation_panel.py优先关联同名报告、兼容report.json。未改q/时间/速度/MAT metadata，未自动执行或制造executionReady=true，原软件/版本门保留，发布复制问题只回退原始有效结果并记录。

现有最新结果`preview_20261019_110448_976674`416点匹配当前Home REALHOME-5b20…与scene20241…，已发布为`规划文件/当前轨迹/预览轨迹_仅预演_2026-10-19_11-04-48_976674.mat`。它仅为预览，当前Home尚无已完成连续规划结果；旧prepare103553等绑定旧Home，不能沿用其PASS。未自动重新规划。

证据根`trajectory_autoload_cleanup_20261019/`：report.md、result_store_tests.json（原MAT/report字节一致、完整软件上下文、阶段/碰撞错误拒绝、新预览替换旧选择、篡改拒绝）；vtk_autoload/responsiveness_tests.json实际VTK按钮/QProcess重放真实历史优化报告，自动导入中文副本/报告关联/切执行Tab/取消PASS，合成未就绪状态和历史测试YAML，ROS/RTDE关闭，发送0次；execution_regression软件门26反例拒绝及100/125/150布局PASS；current_import_after_cleanup.json在清理后全新HMI导入实际当前416点预览、最新定位/连续规划恢复/不执行全部PASS。不是新MATLAB优化或物理安全验收。

按用户删除授权，先清单与SHA核对，无HMI/MATLAB活动时通过gio trash移入系统回收站78份旧HMI轨迹（8,953,839字节）：planning_runs/hmi_*36、规划文件旧发布/日常preview/prepare40、轨迹文件2。仅MAT及轨迹JSON负载，不删完整请求目录/旧报告日志，不动算法实验/comparison/冻结anchors/模型/标定/备份。cleanup_inventory.json列出精确路径与SHA，cleanup_inventory_trashed.json逐项成功，cleanup_verification.json确认78原路径不存在且78份Trash原路径恢复信息都在，当前请求/副本/场景依赖完好。若历史诊断脚本引用这些已删文件，需要先从回收站恢复，不能再声称原路径存在。

算法、几何、安全门和ROS2执行链不变，操作文档已更新。需要重新打开HMI加载新界面。NO ROBOT MOTION。

## 2026-10-19：上位机手动倍率、真机反馈动画、2倍速度离线FAIL

用户要求实机更快、上位机调速及执行同步动画；只读取样时实际倍率已经100%。用户明确“希望达到当前约2倍，先做离线验证”。新增轨迹页1%～100%目标/“应用真机倍率”，复用现有UR 2.14驱动`/io_and_status_controller/set_speed_slider`，不启动/拖动即发送，运行中二次确认。`ros/speed_slider.py`非阻塞，检查真实反馈/安全/程序/scaled控制器及服务，超时UNKNOWN保留未决请求，拒绝重叠/自动重试，ACK不伪造实际反馈。真实调速本轮没有调用。100%是轨迹文件原速，不是额定性能，接口本身不是200%功能；动态调速实际加减速/伺服未验收。

新增`gui/execution_live_panel.py`、`trajectory/execution_feedback.py`，修改`main_window.py`、`ros_node.py`、`state_monitor.py`、`trajectory_client.py`。发送前/运动忙时退出预演，禁止预演覆盖真实关节驱动的UR5/焊枪/焊丝，10Hz显示。新增只读action反馈钩子，不改消息位置/时间/速度/加速度。执行日志比较同一控制器反馈desired/actual，不再用未计scaling的墙钟线性播放计算误差。核对安装JTC2.54源码发现desired.time_from_start为下一节点时间，故进度只显示当前段下界/估计剩余时间，过期UNKNOWN；取消请求/未知提交不是终止结果，保持反馈跟随直到真实结果。顺便修复原动作接受时文本中`0%`被百分号格式化解释导致异常的问题。原实时门/人工执行确认不取消。

独立`matlab仿真/diagnose_hmi_uniform_speed_scaling.m`使用当前471节点/47焊接节点结果`规划文件/当前轨迹/连续规划完成轨迹_2026-10-19_12-14-01_560570.mat`，核对绑定/Home/场景与源SHA，不改q/TCP，仅t/2、V*2。完整101点/段合计47001样本，耗时276.394510s：294.880033→147.440017s；vmax0.084673742→0.169347484rad/s（限0.087266463），amax0.261718677→1.046874709rad/s²（限0.261799）；轮廓0.291503mm、同步0.462008mm、姿态0.414448°、碰撞/限位PASS但速度/加速度FAIL，executionReady=false。最小正间隙0.459441mm未作物理余量认证。段内jerk3.583708→28.669662rad/s³，节点加速度跳变单独统计。固定数值q节点/5°/s限速的纯速度必要时长下界225.857235s，故147.44s不能仅靠重新分配时间达成；该下界不是已找到的可执行结果。J6累计行程约1076.31°，未取模或改变路径。本轮无2倍MAT/执行许可导出，不得把候选当可执行轨迹。

证据根`UR5/规划文件/workflow_runs/speed_live_sync_20261019/`：report.md、speed_2x_offline_rerun/speed_2x_validation.json、before/旧生产文件。初版离线wrapper比较raw附加request-only键导致断言，改为逐个优化依赖键+独立Home核对后完成重跑，未放宽正式版本门。`vtk_tests/speed_live_tests.json`真实Qt/VTK+合成反馈及假服务/action全部PASS：1/100边界、无启动/拖动指令、故障/过期/非法/重叠拒绝、超时迟到、字段保持、5帧gun固定法兰变换/wire末端、预演隔离、取消待返回；无ROS节点、真实调速/action。26项执行门拒绝+100/125/150布局PASS；gui_responsiveness/真实VTK50Hz合成状态/QProcess历史回放PASS，按钮0.043s、忙时最大心跳49ms、取消0.262s，完成图表仍约2.597s短停顿。语法检查PASS。原MAT/生产YAML不变，未调用成功真机发送分支。

操作说明已更新`ur5_hmi/docs/continuous_planning_and_execution.md`。CBi-RRT*/ACCP/IK、TCP/几何/碰撞规则、控制器YAML、trajectory_utils发送数据构建不改，仅新增现有调速接口客户端和只读反馈。用户原HMI/驱动未关闭；需机器人停止且无活动任务后重开HMI生效。NO ROBOT MOTION。

## 2026-10-19后续：本体来源限值、用户安全页照片、2倍配置审计PASS但提速执行仍BLOCKED

用户追问并确认前次5°/s、15°/s²是项目设置，要求放宽软件限值、以UR5为主、超限不允许运行。新增`UR5/robot_motion_limits.json`（SHA8eaa422c…）和`ur5_hmi/safety/robot_motion_limits.py`：通过192.168.0.10只读Primary30011 recv-only取得445字节ConfigurationData，6轴maxspeed=3.341592653589793rad/s、maxacceleration=40rad/s²，原始报文SHA16f37086…保留；与校准URDFπrad/s取交集，不修改URDF、J3位置限制或碰撞规则。官方ur_description说明加速度限值未公开，40只是本机配置报文上界，不是当前负载动态能力或推荐运行值。

用户随后提供两张示教器照片（原件暂存remote-attachments，已副本保存`UR5/robot_motion_limit_evidence/`并绑定SHA）：Normal六轴显示191°/s（旁列-11°/s），一般限制TCP1500mm/s、Force150N、Power300W、Momentum25kgm/s。最终离线速度上界仍为更严180°/s。收到的是安全页而非工具负载/重心页。另只读运行既有robot_tcp_helper，无--request/空RTDE input recipe，登记payload0.5kg、CoG[-6.412,-16.572,390.823]mm，保存payload_readonly.json；没有将登记值当实物称量证明。适用负载/加速度及物理伺服仍未验收，未放行提速执行。

新HMI连续规划请求增加motion_limits_file/SHA；MATLAB独立读/校验原始报文、URDF、照片和配置版本，设置当前请求scene.hmiMotionLimits，统一传给deferred timing、C/Hybrid速度、controller continuity及最终控制器等价验证。只替换阈值输入，不改搜索/IK/Bridge/优化目标与算法结构。原实验/旧请求未携带新profile时保留原阈值（adaptive保留精确deg2rad(15)）。新报告/轨迹metadata记录同一契约，限值变化须重新验收。Python离线状态机识别新契约，不套旧5°/s；旧报告不能自行冒充新门PASS。

新增HMI与ROS worker双重极值拦截：P+V三次曲线段内速度驻点、加速度端点峰值、关节位置驻点，逐轴检查配置/URDF交集，不只看离散节点。缺数据、未知/变化版本、非有限值/错顺序/不同插值字段/超限拒绝。GUI数值检查按轨迹、配置、URDF、activeTCP SHA缓存，不重现50Hz文件读取积压。TCP速度以全姿态三角不等式杠杆长度界检查，Normal1.5m/s；超过保守界会拒绝，非稀疏差分“最大速度”。新增hmi_tcp_speed_upper_bound.m；两个后端链分解不同，数值界不同并非FK位置误差。执行增速仍显示MOTION_LIMITS_PAYLOAD_ACCELERATION_PENDING，旧低速兼容范围不是硬件上限，改JSON verified布尔值不能伪造许可。没有改变UR示教器速度滑块1～100%接口或自动使用40rad/s²运动。

证据根`UR5/规划文件/workflow_runs/robot_motion_limits_20261019/`：report.md、before/、matlab_2x.log、speed_2x_offline/speed_2x_validation.json、tcp_speed_bound.json、payload_readonly.json。对原12:14的471节点/47焊接节点做相同q/P、t/2、V*2新配置审计：147.440017s、vmax0.169347484rad/s（9.702896°/s）、amax1.046874709rad/s²，轮廓0.291503mm、同步0.462008mm、姿态0.414448°，47001点碰撞/限位PASS，minGap0.459441mm未认证物理余量，offline_pass=true但executionReady=false；TCP速度保守界Python0.741335m/s、MATLAB0.752234m/s均<1.5，辅助审计与既有同一候选的碰撞审计分开保存，不谎称又做一次完整碰撞。无2倍MAT/执行许可导出，原源SHA7bab695b…不变。先前225.857s下界只适用于旧5°/s，不能再当硬件下界。

专项17负例全部拒绝，含段内峰值/过冲/TCP界、源与契约错误、伪造PASS、旧报告套新限值；fake worker实际send=0，GUI缓存随数值变化失效；新门离线PASS状态为合成报告协议测试不是新正式轨迹。原26执行门负例及100/125/150布局、VTK反馈回归PASS；GUI按钮/QProcess历史回放PASS（不是新默认阈值完整冷连续优化），完成图表仍约2.6s短停顿。Python语法、MATLAB加载/新旧参数分支、跨语言契约相等、50配置Jacobian杠杆界检查PASS。未修改CBi-RRT*/ACCP构型/IK/Bridge、任何模型/TCP、碰撞阈值/skip、控制器配置或ROS2驱动，只新增发送前检查，轨迹消息数值不改。

重要并发状态：用户在调查期间自己保存新Home并运行`preview_20261019_151534_200122`，15:18当前索引切到新预览，prepare=null，Home为REALHOME-a3b74aa159f6c5b30239fc95a32d10aaec3f58fe00efafde8b0b9d7934bf84d7。没有恢复/覆盖用户索引或旧Home。12:14轨迹的2倍数学测试不能用于新Home任务执行，当前预览需重新连续规划。新专项测试用显式不可变历史文件，避免把用户latest永远当prepare。用户现有HMI进程未由本轮重启/终止。

剩余：确认控制器登记0.5kg/重心与实际焊枪及支架一致，确定适用负载下的运行加速度并完成必要伺服验收；两张安全页不能替代这项。当前只完成离线范围放宽和安全拒绝实现，没有宣称放宽后可直接运行。NO ROBOT MOTION。

## 2026-10-19：Hybrid-D迭代耗尽保留C候选；独立内点法对比；全局连续化仍FAIL

用户授权修复可选Hybrid-D失败后的候选交接，而非降低门限或授予执行。先前只读诊断根 `workflow_runs/diagnose_hybrid_20261019_152229/`：新Home a3b74…有效，C焊缝同步0.428674657mm/姿态0.280061645° PASS；六次D均exitflag0/1000迭代，却被旧调用者当成无解立即终止。首轮原矩阵LP可行，换可行初值仍耗尽，不能称约束无解。

生产最小修改：`optimize_accp_hybrid_velocity.m`保存真实退出状态/迭代/耗时，未收敛值不采用，显式返回C非最优候选；新`hmi_quadprog_exit_status.m`区分0的ITERATION_LIMIT与-2的QP_INFEASIBLE；新`hmi_select_hybrid_velocity_candidate.m`核对q/TCP/time/绑定、完整焊缝审计及解析极值，D有效则用D，否则只接受原样已审计C。`prepare_hmi_continuous_request.m`去掉强制新D成功门，保存Hybrid诊断并让候选进入原有全任务连续化/控制器/碰撞/版本门；历史复用要求明确D_found，避免把第4槽中的C误称D；增加GLOBAL_CONTINUITY阶段命名。输出元数据及离线HTML保留种子来源；源报告不会把C回退冒称D优化成功。原D目标、矩阵、约束、active-set及1000预算未改，全局优化器未改。

用原 `prepare_20261019_152229_336637` 请求和原SHA bb3473…预览完整重跑，根 `UR5/规划文件/workflow_runs/hybrid_fallback_recovery_20261019/`。实际C交接PASS，进入后续4次细化+4次时间/速度恢复，随后全局QP均-2未找到可行解。末次FK诊断同步0.637526939mm（approach t=4.361637546s，诊断边135）、轮廓0.614167135mm、姿态0.280061645°；原节点保持，新增717个仅诊断执行节点。返回CONTROLLER_FK_VELOCITY_TIMING_NOT_FEASIBLE，耗时962.479387s（与独立对比共享资源，非标准性能基准）。最终接口C²/独立密集碰撞/版本收尾未进入，collision=NOT_CHECKED，不能沿用C或预览的PASS；没有优化轨迹/执行许可导出，executionReady=false。完整运行开始时已解析入口，报告仍保留原宽阶段名VELOCITY_OPTIMIZATION；新增GLOBAL_CONTINUITY仅影响下一次请求的定位，不重写真实日志或改计算。

独立 `solver_copy/hybrid_solver_comparison_copy.m`仅参数化算法，完全相同47节点/TCP/时间/目标/约束/1000预算：active-set六次各0/1000、总47.907538s，选C；interior-point-convex首次exit1/17迭代、QP444.111493s/含审计总504.030547s，得到有效D，焊缝同步0.488369722mm/轮廓0.464747480mm/姿态0.422572829°，vmax0.218274090/amax0.658433750、jerk max1.803635432/RMS0.288672821，碰撞/限位PASS。内点法支持“并非无解”但本次明显慢且精度余量变小，没有改成日常默认或自动用于全任务。对比不是相同尝试次数/隔离性能基准，不能推广成功率。

19项候选反例全部拒绝；历史真正D成功输入重算及当前选择器PASS（不是当前Home任务PASS）。第一次历史测试缺scene.jointLimits，仅测试内从同一旧rigidBodyTree恢复原限位后重跑，不改生产。HMI状态机和真实Qt回调重放本次全局失败：保留预览、显示新具体原因、执行禁用，无ROS/RTDE/发送，YAML/当前索引逐字节不变。证据 `candidate_contract_tests.json`、`hmi_contract_tests.json`、`hmi_actual_failure.json`、`historical_D_positive_regression.json`、完整 `full_prepare/report.json` / `full_recovery.log`、`solver_comparison.json`，总报告report.md；before/保留4个旧生产源。Python语法PASS。操作文档已补充。

**交接重点：候选契约修复完成不等于完整任务恢复成功。** 当前下一个问题是approach全局FK约束/时间输入与有界恢复未找到解；不要重做ACCP几何或切旧PreWeld，不要放宽0.5mm/碰撞门。用户Home、输入预览、限值profile SHA与此前一致，实时HMI/驱动未重启/停止。NO ROBOT MOTION。

## 2026-10-19：固定正式门的QP数值对比与隔离TCP灵敏度——定位时间参考/插入节点建模冲突

用户要求固定全部正式阈值优化数值条件，只在隔离副本改变TCP内部约束，比较后再决定重构。实验根 `UR5/规划文件/workflow_runs/qp_numerics_sensitivity_20261019/`，report.md完整报告、comparison_summary.json统一验证。正式优化器/验收/参数/Home/TCP/模型/ROS链路本轮完全未改，仅新增隔离脚本与交接记录，未发布HMI轨迹/索引，executionReady=false、NO ROBOT MOTION。

冻结同一新Home原预览及C候选，初始全任务360节点/2160速度变量/236272不等式/24等式；末次准确重建1077节点。`copies/freeze_hmi_continuity_qp_copy.m`机械复用正式矩阵组装，只加标注与捕获返回。注意这是全任务QP，不是前轮47节点282变量的Hybrid-D，前轮D的LP可行不能推及此矩阵。

独立副本采用正行/变量缩放、统一目标缩放、Hessian对称表示、精确等式消元和单变量约束转上下界，不放宽任意正式约束。condest从815260.663552降至127002.255289，2160→2136变量，72常量冗余行、17496单变量行精确处理。等价变换小QP目标一致、真实Eq映射残差2.22e-16。冻结初始QP重复3次：原始内点法均-2，11.884651±0.820162s；等价改良均-2，9.963730±0.478230s。计时不含矩阵组装/变换/FK，非HMI整体提速。原始LP也-2；active-set前置LP无可行初值，未执行其目标优化。约束子集LP仅归因，未当合法候选：运动学/加姿态可行，加入位置同步不可行。

隔离灵敏度：初始QP单独改变位置（姿态固定0.46°）或姿态（位置固定0.46mm）至0.40/0.46/0.49/0.50/0.55/0.65，均无LP可行解。共同数值扫描至2仍失败，5可行；进一步单因素最小位置内部界3.972748994mm（姿态0.46°不变，原矩阵残差5.15e-12），主要活跃行在exit原边359、u=5/6。该段TCP43.284563mm/0.609492s，目标按每段线性时间插值约71mm/s至终点，而实际终点六轴速度必须零；原种子该段同步误差6.431394mm。此最小界仅描述当前线性化，不是新的正式阈值或所有非线性路径无解证明。首个辅助上界2mm的LP失败不代表任意阈值均无解；末次最小界辅助LP90秒预算退出也保留UNKNOWN。

确认第二个硬问题：`optimize_hmi_controller_continuity/subdivide`用关节线性插新q，却用Cartesian+SLERP插新目标，FK二者不等。原360节点max位置0.000140118mm；新增717节点中178个固定节点>0.5mm，最大0.637526782mm（approach节点135，t=4.361438866s），所有超差都是插入点。速度不能改变Hermite在节点处的FK，故继续仅调速度不可能通过原门。

末次QP诊断仅位置内部界改0.65mm、姿态仍0.46°，实际QP含重复3/3收敛，但正式精度0/3通过：107601 FK样本同步0.638394979mm/轮廓0.615247129mm/姿态0.282327447°，对原正式QP位置行仍违反0.178394173。未做最终碰撞，不假报PASS，不导出正式轨迹；`*_candidate.mat`仅诊断速度和报告。LP0.46/0.55失败、0.65/0.8成功仅证明内部约束能影响求解，不支持放宽正式门。

**后续决策建议：先修复局部插入q与Cartesian目标一致性，以及首尾减速s(t)与期望TCP时间参考，再复用数值等价改良/phase-I。** 不要事后把期望目标改成错误q的FK，不要将自由空间免于0.5mm门作为未授权捷径，不要重做算法框架/改碰撞。新增Cartesian/IK点仍须原硬约束和完整节点/边/插值碰撞重验。当前没有证据支持直接替换正式默认或单纯调大门限。

证据：initial/last_qp.mat、initial/last_input.mat、fixed_node_audit.json逐节点、initial_attribution.json、minimum_position_attribution.json主导行与最小界、initial/last_sensitivity.json、各数值/重复QP JSON与MAT/日志、transformation_tests.json、verify_and_summarize.py。脚本初版struct/cell及sparse JSON编码失败已仅在隔离脚本修正/重跑，原日志保留。正式SHA检查前后相同：cubic optimizer850a632c…、global continuitybb676dc5…、controller validator486d15bf…、限值profile8eaa422c…、HMI配置b4b77f1c…。固定输入重复成功次数不是跨场景概率。

## 2026-10-19：连续规划三阶段重构——独立对比后最小生产接入，离线PASS不等于可执行

用户要求先梳理、独立验证，再简化日常调用链；保留47个焊接q/TCP目标与全部原安全门。本轮根`UR5/规划文件/workflow_runs/continuous_refactor_20261019/`，完整report.md、architecture.md、production_changes.patch及before/可回滚。旧入口依次deferred timing、C、可选D、回退C、全局QP多轮细分、时间QP、接口QP；此前已确认新增q/Cartesian目标不一致及Home静止与线性时间参考冲突。本轮不通过放宽0.5mm/0.5°门或证明旧QP无解处理。

正式`prepare_hmi_continuous_request.m`现64行，依次调用新`hmi_continuous_geometry`、`hmi_continuous_parameterize`、`hmi_continuous_acceptance`。几何唯一所有者用原Cartesian/SLERP参考、C²几何样条与有界局部IK细化（6轮/1800节点/150秒），保留原三段全部q及47个原焊接TCP目标逐值不变；共享边界微小独立FK矩阵差以原ACCP目标为准。新节点的FK不能超过原门，不能事后把目标改成候选实际FK。统一参数化唯一所有者冻结几何后分配单调局部时钟、首尾静止五次进度及解析速度，12轮/2400节点预算；不再多模块反复改同一轨迹。C²承诺仅三段接口与静止首尾，其他节点加速度跳变和段内jerk分开报告，不称整体C²/jerk最优。

两个新纯参考助手`hmi_continuous_reference/pose_at_s`；原controller validator仅增加可选独立时钟，缺省旧语义不变；FK曲线helper与离线export增加同一参考及metadata保存，不把诊断加速度作为命令字段导出，继续P+V三次。验收用原生`check_scene_environment_collision`（保留原唯一固定安装接触）与原self skip，未新增过滤。验收只读，完整碰撞/解析极值/版本/新鲜Home检查失败保留原因，不回改轨迹。Home版本前置快速拒绝并在末尾重新读取，不能把历史Home测试当当前准入。

先独立副本、再同名staged入口完整验证后才生产安装；9个生产MATLAB文件与staged源码逐字节相同。旧C、Hybrid-D、全局/局部QP、ACCP、CBi-RRT*源码均未删除或修改，只不再日常串联。17项受保护SHA含这些算法、TCP/URDF/枪/安装/工件/地面/限值/Home/ROS发送层及最新轨迹索引完全不变，见protected_verification.json。生产安装后另做8项快速契约（不是又一次400秒完整审计）。未关闭用户HMI/驱动、未修改当前索引、未生成执行许可、NO ROBOT MOTION。

当前原请求prepare_20261019_152229_336637/预览SHA bb3473…/Home a3b74…：独立新局部时钟完整401.169s PASS；同名staged全入口含新鲜场景/Home版本及导出402.180953s PASS，638节点/47原焊接节点、63701密集采样零碰撞、限位/解析极值/接口C² PASS。轮廓0.251530mm、同步0.287995mm、姿态0.262226°；接口加速度跳变7.11e-15，全部节点max跳变0.005476。几何11.341s、统一参数化1.041s、最终FK/控制器/全碰撞382.598s、版本3.738s、导出1.943s，瓶颈仍严格最终验收，未减密度。旧全局恢复962.479s最终FAIL只含FK诊断，无最终碰撞；单次/共享资源计时不包装成统计提速率。

**当前新结果的重要局限：** 使用请求既有新限值profile（40rad/s²只是配置上界），最大v0.230830rad/s、a14.465545rad/s²，段内jerk max1600.279845/RMS15.681107rad/s³，并未优化到旧D平滑水平。运动时长207.842390s，非计算耗时402秒。最小正间隙0.319238mm、wire shorten=3mm仍未认证物理余量，焊接进给非恒定；这些不应隐去。实际HMI软件执行检查对该真实MAT仍返回MOTION_LIMITS_PAYLOAD_ACCELERATION_PENDING，execute禁用，executionReady=false，不能用数学PASS放行高加速度或当物理伺服验收。

历史prepare_20261019_121401_560570冻结旧Home/旧5°/s与0.261799门：独立副本298.796919s PASS，478节点/47原焊接、47701采样零碰撞，轮廓0.271158mm/同步0.303025mm/姿态0.274799°、v0.082903/a0.247550；运动325.086889s，比原294.880s慢约10.2%；jerk max4.478462比原3.583708高约25%，RMS0.161065比原0.187197低，跳变0.200930比原0.439865低，不宣称所有指标更优。原历史计算478.566s；新几何7.226/参数化1.087/验收290.376s。历史只是冻结离线回归，正式新Home门应拒绝它。统一倍率原型使历史运动1749s，已拒绝并保留copy_uniform与*_uniform，不接生产。

测试证据：contract_tests.json13项原q/TCP保留、独立参考、静止首尾、接口及坏几何拒绝；production_contracts.json8项端点/Home过期/坏时钟/原始SHA/失败不导出及旧validator缺省行为；new_workflow_contract.json真实MAT载入/软件上下文与17项假PASS拒绝，确认高加速度执行仍BLOCKED；legacy_workflow_contract.json旧16反例；new_hmi_replay.json生产MainWindow真实回调/载入/3帧显示/错误碰撞拒绝、ROS线程禁用与隔离结果目录，不是现场VTK真机测试。installed_controller_equivalence.json直接调用本机JTC动态库15150个关节样本，P/V/A与Hermite差<=1.2e-14，无ROS init，不能等同UR内部伺服已验证。

下次点击HMI“轨迹连续规划”启动的新MATLAB进程即走新三阶段入口，不必重算原始预览；本轮诊断结果仅在独立目录，没有替换用户当前加载结果。操作文档已更新，旧四方案实验只适用于有velocity_profile_input_diagnostic.mat的旧记录，不伪造新C输入。所有完整/局部测试均executionReady=false，无真机运动。

## 2026-10-20：三阶段内平稳时间/速度优化与等覆盖验收加速——生产全入口143秒离线PASS

用户要求先诊断/独立副本，再必要生产修改，保留简洁三阶段、不放宽正式门。根`UR5/规划文件/workflow_runs/continuous_smoothing_20261020/`，完整`report.md`、`comparison_summary.json`、`production_changes.patch`与`before/`。未改入口/几何/验收门所有者，仍几何一致性→统一参数化→只读全程验收；没有重新串联C、Hybrid-D、QP。生产只更新`hmi_continuous_parameterize.m`与`validate_accp_controller_interpolation.m`，新增内部`hmi_audit_collision_state.m`/`hmi_audit_collision_batch.m`。26项SHA保护检查全部不变，包括ACCP/CBi-RRT*/旧QP、TCP/URDF/枪/安装/工件/地面/限值、Home/YAML、ROS发送层、当前轨迹索引及其余三阶段源码。没有发布或自动加载本轮优化轨迹，没有重启用户HMI/驱动，无机器人运动。

**准确jerk根因：** 原峰值在焊接→退出接口前边299/J1，88.813563044～88.824374982s仅10.811938ms，J1速度−0.114512589→−0.051646776rad/s，加速度−2.836581563→14.465544664rad/s²，解析jerk1600.279845；下一退出边约1018.778。原接口C²修正并未限制短段内加速度变化，40rad/s²配置上界也没有jerk优化目标。不是FK/IK噪声或UR底层会自行消除的问题。`current/diagnosis.json`有前10峰值、六轴状态/TCP，`derivative_analysis.json`/四份`*_derivatives_offline.csv`保留前后解析曲线。仅按局部jerk缩放原链式速度的原型运动接近992秒，明确拒绝，未接生产。

**唯一时间/速度主方法：** 全部q/TCP节点、独立Cartesian/SLERP参考和几何样条冻结，内部夹持三次样条一次求速度，再按实际Hermite解析极值有界局部时间分配；静止首尾速度/加速度为零，三段接口C²。软优化偏好a1rad/s²、段内jerk5rad/s³、节点a跳变0.005rad/s²，30轮预算，取与原硬限值更严交集；这些不是硬件额定值/物理许可，不改限值文件。两处静止过渡内部仍有微小a跳变并报告，不能宣称全局C²或把段内jerk当包括脉冲的数学全局jerk。未改ROS P+V消息/插值，导出仍不把诊断加速度作为命令字段。

**验收瓶颈有实测证据：** 原383.418916s中碰撞369.860156s（环境63701+self63701）、TCP FK7.974367s、全段轮廓3.181099s、参考2.285751s。63701状态只有1个重复，不引入取整/跨请求缓存。现在每状态一次原生exhaustive同时取得完整self/world矩阵，原skip与唯一底座↔地面固定接触单元严格保持；轮廓对所有原段做有界内存向量化，临近阈值恢复原标量计算保留原`>=0.5`判定；最多四个本地独占进程分摊相同采样。无依赖/启动失败/存在外部池则完整串行，不接管外部池，工作进程开始后的错误必须失败关闭，完成/异常清理本轮池。单纯串行改法374.797s收益很小，主要加速是等覆盖四进程，不能称API合并把碰撞工作量减半。

**同一新Home当前请求完整结果：** 先同名staged全入口140.104329s PASS，再实际生产入口`production_current/report.json`独立重跑143.205980s PASS，q/TCP/time/v与staged逐值一致。生产输入1.367625/几何11.357875/参数化1.508182/全程验收123.442417/版本3.498565/导出1.938601s；全碰撞113.096179s含四进程启动6.809304s。原全入口402.180953s。638节点及47原焊接目标逐值不变，63701采样零碰撞，解析关节限位、速度/加速度、接口/静止端、版本/Home全部PASS。最大v0.169564946rad/s、a0.652434285（原14.465545），段内jerk4.934481758/RMS0.480050564（原1600.279845/15.681107），RMSa0.033565596、最大节点a跳变0.000542982（原0.005476303）。TCP轮廓0.248502409mm、同步0.286324597mm、姿态0.261429241°；原0.251530/0.287995/0.262226。运动时长227.023342s，比207.842390增加9.23%，计算更快不等于真机更快。最小正间隙0.319629919mm。

**旧Home冻结回归：** 仍478节点/47原焊接、47701全样本PASS，全部q/TCP及路径参考逐值不变；a0.247550→0.227390，段内jerk4.478462→3.918559/RMS0.161065→0.130630，跳变0.200930→0.000388。TCP轮廓0.263850mm/同步0.282548mm/姿态0.283045°（姿态比原0.274799略增但原门内），v0.085301仍低于原5°/s。运动325.086889→325.871446s，验收290.376→93.388s。旧Home只是冻结离线对照，正式当前Home门不能放行它，不能把历史PASS当当前执行许可。

**验收与集成反例：** 原同一轨迹63701全状态前后q/v/a/t/FK/目标/碰撞flags/间隙逐值一致，轮廓数值差<=7.46e-14mm；105状态包含实际自/环境碰撞，逐对象矩阵和NaN/Inf掩码精确一致。副本和生产10项backend契约PASS，含精度阈值±1e-12mm判定、空环境、非法状态、工作进程失败关闭、外部池不变及5000点串行全覆盖。校正的取消测试仅针对本轮测试MATLAB及四子进程，4.47s内全退出；初版PID浮点路径测试无效，`cancel_test/INVALID_README.md`明确作废，不能引用该结果。生产真实MAT载入/17项假PASS拒绝及MainWindow离屏真实回调/638点载入/3帧显示/真实状态不被覆盖/碰撞失败拒绝均PASS，执行按钮禁用；禁用ROS线程且仅隔离结果目录。见`production_backend_contracts.json`、`production_workflow_contract.json`、`production_hmi_replay.json`。单场景几次计时不是跨场景成功率或性能保证。

**仍非执行许可：** 本轮所有成功保持PHYSICAL_VALIDATION_PENDING/`executionReady=false`，实际软件发送检查仍返回MOTION_LIMITS_PAYLOAD_ACCELERATION_PENDING；0.652rad/s²不是负载动态能力已认证。wire shorten=3mm及约0.32mm间隙仍需物理安全验证，密集采样PASS不是连续路径安全数学证明；没有新增恒定焊接进给保证。下次点击连续规划的新进程会使用本轮4文件，不必重算原始预览；用户当前轨迹、索引和Home原样保留。NO ROBOT MOTION。

## 2026-10-20：预演动画加速——默认5x、两页可选0.5～20x，真机速度不变

用户仅要求播放预演更快。原`gui/main_window.py`预演默认1x、最高2x，调速只在轨迹执行页，直接改倍率还会把已累计播放时间按新倍率重新计算而跳帧。本次新增共享`PREVIEW_SPEED_OPTIONS`（0.5x/1x/2x/5x/10x/20x）、默认5x，在在线规划播放按钮旁也加预演倍速下拉；两页同步。`_preview_speed_changed`只重定位本地monotonic播放时钟，保持切换前后的轨迹进度连续；暂停时调速不自动续播。新下拉纳入原motion busy禁用，不更改执行链路。未修改轨迹文件、时间戳、q/v、真机速度倍率或校验规则，没有机器人指令，未重启用户HMI。

新`tools/test_preview_playback_speed.py`使用实际MainWindow及动画回调、200秒合成轨迹和受控时钟，ROS线程/TCP自动读取禁用；覆盖默认5x、5→10→2切换无跳变、暂停改20x续播、拖动、停止/完成重播、运动期间预演禁用。测试PASS，ROS轨迹消息与q/v/time逐值不变、真机倍率与实时状态不变、运动/调速信号均0、YAML逐字节不变。证据`UR5/规划文件/workflow_runs/preview_speed_20261020/test_results.json`；不是新规划或真实机器人执行测试。操作文档已补充，新UI在用户安全重开HMI后加载，默认约227秒轨迹约45秒播放完。NO ROBOT MOTION。

## 2026-10-20：修复HMI连续规划冷启动缺少checkEnvironmentCollision路径，实际按钮命令完整回归PASS

用户`prepare_20261020_094039_057772`在几何阶段2.963秒报rigidBodyTree对应checkEnvironmentCollision未定义。准确根因：适配器在RRT根目录，HMI `planning_workflow.py:270`的新MATLAB只加matlab仿真及RRT/benchmark；连续规划load场景不执行build_real_planning_scene，第60行几何细化已经调用碰撞，而build中的genpath到最终版本验证才运行。此前完整独立测试预先genpath(RRT)，掩盖了生产冷启动缺陷，不是实际算法/碰撞失败。本轮真实MainWindow点击连续规划按钮并捕获原QProcess表达式，再原样冷启动运行，修复前同栈UndefinedFunction可重复（2.950489秒）。

只改生产`prepare_hmi_continuous_request.m`：从mfilename定位自身，在任何IK/碰撞前显式加载项目RRT根目录，核对which确为本项目checkEnvironmentCollision.m，缺失/同名遮蔽返回CONTINUOUS:Dependency，报告实际依赖路径及耗时。没有改GUI启动命令、三模块算法、碰撞实现/过滤/阈值、模型、TCP或ROS链路。新测试`ur5_hmi/tools/test_continuous_launch_paths.py`实际点击Qt按钮、无ROS线程/TCP读，原始表达式没有额外genpath，所有结果仅隔离目录，不覆盖原用户失败请求或当前索引。

修复后同一09:34预览SHA2b1f49…及新Home b8b8b4…，实际按钮冷启动命令完整PASS：`prepare_20261020_094748_672689/report.json`，总145.022300秒，几何11.304037/时间速度1.484127/控制器验收125.269439/版本3.552100/导出1.973954秒。638节点/47原焊接目标/63701采样，完整碰撞、限位、解析极值、接口及版本全部PASS；TCP轮廓0.248496522mm/同步0.286312952mm/姿态0.261428939°，段内jerk4.938160944。PHYSICAL_VALIDATION_PENDING、executionReady=false，未发送机器人运动。

证据根`UR5/规划文件/workflow_runs/continuous_path_fix_20261020/`：完整report.md、before/、production_changes.patch、before/after_launch.json与日志、dependency_guard_tests.json、regression_results.json。实际MainWindow完成回调重放原失败、新成功并载入638点/报告可查看/执行禁用；初次默认SIMULATION窗口正确拒绝REAL结果，测试明确切至用户相同REAL CALIBRATION才通过，版本门未放宽。同名遮蔽与原始SHA过期反例拒绝且无优化文件导出。以本轮开始为准28项已有文件SHA仅预期入口改变，Home/YAML/索引、算法、模型、碰撞与ROS保持不变。用户当前HMI不用重启、不用重算预览或修改Home；再点连续规划将读取新MATLAB入口。NO ROBOT MOTION。

## 2026-10-20：真机执行BLOCKED只读诊断——旧兼容范围、inactive及用户关机状态分开处理

用户10:07已成功导入638点/227.017345秒轨迹，截图报MOTION_LIMITS_PAYLOAD_ACCELERATION_PENDING和scaled controller未active。本轮只读调查，不修门、不切控制器、不改倍率/轨迹。真实MAT解析v0.169566642rad/s=9.715453°/s、a0.652382403rad/s²=37.378758°/s²，超过`robot_motion_limits.py:205–207`旧兼容5°/s、0.261799rad/s²；该阻塞是现有软件策略，不是UR5硬件额定上限或当前轨迹超出配置/URDF上限。load_limits第105–114行未接入可用的独立负载/加速度验收记录，且check_motion直接比较旧范围，所以重复验证、选离线报告/翻JSON布尔值不能解除；不能声称用户已有按钮可完成这项验收。

只在内存测试同q、time×2、v÷2：454.034690秒、最大4.857726°/s与9.344690°/s²，此软件范围阻塞消失；不是完整验收，没有导出/发布/加载该候选。后续需用户选择保守兼容版全量离线验证，或保留227秒速度并正当核实负载/重心/加速度与实现验收接入，不能自动放宽原门。

只读ListControllers确认scaled_joint_trajectory_controller确为inactive，不是UI解析误报；start_ur5_real.sh:70有意activate_joint_controller=false，controller_stopper参数joint_controller_active/headless_mode均false。现有run_hmi.sh另有条件watchdog，源码依赖多次3秒topic echo，可能受锁存/发现影响，但截图时到底因未就绪、watchdog缺失还是采样超时，证据不足不能归因。没有调用switch_controllers、run_hmi、Dashboard播放/重连或驱动重启。

调查期间状态变化：此前一次Dashboard只读查询成功program=true，13:35后9秒采样却为Dashboard未连接、驱动mode3 POWER_OFF/倍率0、程序话题无样本、RTDE持续reconnecting，joint_states仍重复发布。用户明确回复“已经关机／正在重启”，故与截图100%/程序运行属于不同时间，不能将先前状态当当前许可；也不能把关节话题新鲜当真机新鲜。当前应待用户恢复后重新只读核实，不能自动上电/激活。截图目标1%明确尚未应用，原实际100%；本轮未替用户调速。

证据`UR5/规划文件/workflow_runs/execution_blockers_20261020/report.md`、readonly_runtime.json、trajectory_diagnosis.json；新增`ur5_hmi/tools/diagnose_execution_readonly.py`仅限定9秒读topic及ListControllers/GetParameters/IsProgramRunning，无写/动作接口。只增诊断工具/报告/交接，生产执行链、模型/碰撞/标定/Home/轨迹/配置及执行门未改，executionReady=false，NO ROBOT MOTION。

## 2026-10-20：倍率调到最低仍BLOCKED——目标倍率不等于轨迹重验收，也不恢复控制链

用户先选保持227秒核实负载接入，随后询问调速界面/最低倍率为何仍不能执行。本轮14:08:35只读9秒ListControllers确认scaled控制器inactive；驱动锁存program_running=false，Dashboard查询连接失败；1124个新倍率样本为0%，mode7/safety1不能替代控制链就绪。不能与此前截图100%混用。源码execution_live_panel仅点击应用才请求；ros/speed_slider要求程序运行及控制器active，拖动不发送、不激活。robot_motion_limits.check_motion仍验算文件原始q/time/v，不接收滑块值，故即使倍率已降低也不解除旧兼容范围之外的MOTION_LIMITS_PAYLOAD_ACCELERATION_PENDING；不能解释成低速仍超过UR5本体能力。实际227.017345秒/638点MAT SHA94ef7306…不变，v0.169566642/a0.652382403，executionReady=0。

负载/加速度验收接入尚未完成，不是已生效交付。本轮只读登记0.5kg/CoG[-6.412,-16.572,390.823]mm与TCP平移相同，需核实实物，不能把控制器登记或e-Series性能说明当CB3物理验收。为避免未完成UI回调进入生产，接入中间代码隔离到`tool_dynamics_acceptance_20261020/isolated_draft/`；本轮四个曾编辑的生产文件已恢复至before/逐字节一致，语法PASS，未创建已审核记录/执行许可或放宽门。详细`tool_dynamics_acceptance_20261020/speed_question_report.md`及speed_question_runtime.json。无调速服务、控制器切换或机器人动作。后续分别处理连接/控制器、实物配置与正当验收入口，不能再告知用户现有按钮能完成尚未实现的验收。NO ROBOT MOTION。

## 2026-10-20：用户选择454秒保守调试版，并要求今后新连续规划默认保守输出

完成HMI离线文件调速接入，不是运行时滑块乘倍率后跳过验收。新`hmi_scale_continuous_candidate.m`严格1%～100%，冻结全部q/TCP及Hermite h*v系数，仅t÷factor、V×factor，独立Cartesian参考duration/clockTimes同步膨胀；不改目标路径或碰撞。`prepare_hmi_continuous_request.m`及HMI新连续规划启动默认50%，只在唯一全程验收前缩放一次，参数化诊断同步保存最终候选。新`retime_hmi_continuous_request.m`用于已有完整离线结果，严格核对源MAT/报告/JSON/请求SHA、原预览/47焊接目标/Home与场景，复用既有hmi_continuous_acceptance及导出，不重新运行ACCP或CBi-RRT*，不构造许可。

轨迹执行页新增“文件调速：生成新轨迹并完整离线验收”，1%～100%相对当前文件，预计时间/v/a可见，原227秒文件默认50%。已载入降速版时此文件选项默认100%保持，避免454再误减半变908；100%不生成重复文件。“在线规划→新连续规划输出速度（默认50%）”独立，不受已载入文件100%影响。原真机倍率区域明确改为运行时示教器倍率/不生成验收文件，预演5x等与之独立。结果通过后按`保守调试轨迹_50pct_日期时间.mat`另存并自动导入，不自动执行；原HMI未强制重启，安全无任务时重开显示新控件。

实际638点/227.017345秒结果另生成454.034690秒，所有638个q/TCP节点及原47焊接目标逐值不变，时间精确×2、V精确÷2。峰值4.857726°/s、9.344690°/s²；段内Jerk max0.617270/RMS0.060377rad/s³，节点a跳变0.000135748rad/s²。轮廓0.248497mm、同步0.286313mm、姿态0.261429°；63701完整采样零碰撞，解析限位/极值、接口C²、版本全部PASS。最小正间隙0.300716mm与原样本相同，非物理余量认证。原47目标未改，非全局C²或全局数学Jerk有界证明。

真实Qt按钮捕获冷启动：retime `prepare_20261020_143926_708830`134.103259秒PASS；普通连续规划默认50% `prepare_20261020_144848_669941`145.975289秒PASS；最终生产默认入口再次`prepare_20261020_145827_003182`145.832296秒PASS，最终参数化诊断q/time/v/duration读回一致。三个完整运行q/TCP/time/v逐值一致、同采样/门均通过。不是跨场景成功率或不同计算量的性能对比。

当前最新轨迹已通过原publish_result发布为`规划文件/当前轨迹/保守调试轨迹_50pct_2026-10-20_14-58-27_003182.mat`（SHA60288ec1…）及同名报告，原227秒MAT SHA94ef7306…保留；发布后offline_context重新验证通过。旧索引备份在本轮before/latest_trajectory_index_before_publication.json。当前HMI如需手动导入默认定位新文件；外部发布不会强行让正在运行的用户HMI切换载入。

证据根`UR5/规划文件/workflow_runs/conservative_speed_20261020/`：report.md、numeric_comparison.json、publication.json、scaling_contracts.json、input_guards/guard_results.json（5种SHA/源关联/部分PASS/非法比例全部失败关闭且无导出）、hmi_release_replay.json/png（持久Qt回调+只读RTDE、真实完成回调/自动导入/默认50/防重复减半/碰撞失败拒绝，ROS线程禁用且运动/调速/停止信号0）、motion_limits_regression/（17反例与worker独立拒绝，内存恢复2倍仍被原负载门拒绝）、preview_regression.json（5x等及ROS消息不变）。早期临时lambda Qt夹具因晚到事件引用释放回调导致假jog/TypeError，已明确作废并修为模块级持久回调；另测试命令误写请求后缀264382未进入入口，实际003182完整通过。附加MAT读取首次授权审查超时未执行，重试才DIAGNOSTIC_MATCH_PASS，不伪称超时成功。

生产仅修改MAT连续入口/导出、HMI流程/控件及结果命名，新增统一缩放与复用验收入口/测试；6份改前源码在before/。未改主几何/速度算法、ACCP/CBi-RRT*/IK、TCP/几何/碰撞/skip/阈值、限值配置/原motion gate或ROS发送/控制器配置。50%是相对本次输出的保守比例，其他场景仍须重新验收，不当成额定性能或物理许可。454版仅消除该文件原兼容速度/加速度范围阻塞，不绕过连接/External Control/scaled controller/起点/版本/人工确认，所有executionReady=false，无真机指令。NO ROBOT MOTION。
