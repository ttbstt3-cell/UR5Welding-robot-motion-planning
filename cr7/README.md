# CR7-7 MATLAB / CoppeliaSim 仿真环境

依据 `/home/ttbstt/jixiebi/AGENT_HANDOFF.md` 迁移 UR5 的焊接环境，以本目录已有的 `xMateCR7` 六轴模型替换 UR5。
当前场景是 `simulation/cr7.ttt`；原来的四个 CR7 场景和 UR5 工程未被覆盖。

## 启动

终端一：

```bash
cd /home/ttbstt/jixiebi/cr7
./start_coppeliasim_ros2.sh
```

在 CoppeliaSim 中点击运行，再在终端二启动：

```bash
cd /home/ttbstt/jixiebi/cr7
./start_controller.sh
```

MATLAB：

```matlab
addpath('/home/ttbstt/jixiebi/cr7/matlab仿真')
sync_cr7_joint_states
```

关闭 MATLAB 图窗即可退出同步。先关闭同步脚本，才能在同一 MATLAB 会话执行其它函数。
仅查看静态场景：

```matlab
scene = load_cr7_scene();
display = show_cr7_scene(scene);
```

## 通信和模型

- MATLAB → `/cr7/joint_trajectory` → 外部 rclpy → ZMQ 23010 → CoppeliaSim。
- CoppeliaSim Lua `simROS2` 只发布 `/cr7/joint_states`；不使用内嵌订阅控制。
- 六轴名称固定为 `joint1`–`joint6`，通过对象路径显式映射，单位 rad。
- 仿真器、控制器、MATLAB 使用相同 ROS_DOMAIN_ID；默认沿用当前环境。
- 独立命名空间避免与正在运行的 UR5 topic 混用。
- 场景为运动学轨迹回放：六轴 kinematic，形状 static，不进行接触动力学/力矩仿真。
- MATLAB 使用当前 CoppeliaSim 零位变换构建 rigidBodyTree；每轴 local Z 旋转，零偏置为 0。没有复制 UR5 的 qOffset、tool0 或 base 标定。
- `CR7_linux.urdf` 已修复 Linux 网格路径，仅用于原始模型导入参考；主 MATLAB 模型以场景导出几何为准。

## 焊接几何

CR7 基座位于原 UR5 基座的 world pose。保留原 UR5 的 Workpiece 和焊枪材质、视觉网格、4 个 GunCollisionHull 凸组件、冻结 TCP。
`WeldingGunMount` 直接安装到 CR7 joint6 的坐标系，局部位姿为单位变换；这是一致的仿真安装约定，实际 CR7 法兰装枪还需物理安装标定。

Mount → WeldTCP：

```text
position = [-0.06677482221230063, 0.019489204955399853, 0.38233021973475156] m
quaternion [x,y,z,w] = [0,0,0,1]
```

没有再叠加焊丝伸出量。MATLAB 保留 UR5 当前算法中的 18 mm、1.2 mm 焊丝碰撞圆柱和真实枪轴构造；1.2 mm 是 UR5 脚本的临时默认直径。CoppeliaSim 保留源场景已有的焊枪碰撞体，未额外新增焊丝物体。
Workpiece 采用源工程的四板代理，厚度 5 mm，开口方向 local y=0；没有将整个凹形工件填成一个凸包。高面数视觉焊枪不参与 MATLAB 碰撞检测。

## 验证和规划入口

```matlab
validate_cr7_fk                         % 对比 4 组 CoppeliaSim 真值
run('/home/ttbstt/jixiebi/cr7/matlab仿真/test_cr7_ros2_control.m') % J1 2° 往返，检查反馈
seam = build_cr7_weld_seam();            % 原 UR5 底板/后壁焊缝，80 点、25 mm 端部余量
[q,info] = solve_cr7_ik(T_target);       % T_target 在 CR7_base 下
path = plan_cr7_rrt(qStart,qGoal);       % 碰撞检测 RRT-Connect，不自动发布
publish_cr7_trajectory(path,t);         % 显式发布 N×6 和 N 个时间戳
```

已验证：4 组 TCP FK 一致；MATLAB/ROS2/ZMQ/CR7/反馈完整往返；初始姿态自碰撞/环境碰撞均为 false；小幅 RRT-Connect 路径生成并插值成功。
证据：`simulation/fk_validation.txt`、`ros2_validation.txt`、`planning_smoke.mat` 和 `matlab_scene.png`。

完整焊缝的 CR7 多分支 IK、连续路径和焊接段执行仍需重新规划；UR5 的 MAT 轨迹和关节解不能直接用于 CR7。

## 回滚与再导出

`variants/cr7_original/` 保留第一次搭建的 CR7 原夹具/原焊枪场景及导出数据。最初四个 `CR7*.ttt` 是完整源备份。
`simulation/original_script_15.txt` 保存停用的 CR7 自动焊接演示源码；`ur5_source_script_*.txt` 保存迁移时停用的源脚本。

场景几何变更后，在停止仿真的状态下运行 `python3 scripts/export_current_scene.py`，再运行 `python3 scripts/validate_fk.py` 和 MATLAB `validate_cr7_fk`。
`build_scene.lua` / `migrate_ur5_environment.py` 是一次性搭建记录，会修改场景，不应作为日常启动器。
