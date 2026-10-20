# CR7-7 仿真环境交接

更新时间：2026-10-04（Asia/Shanghai）。启动方法与完整边界见 `README.md`。

当前主场景：`simulation/cr7.ttt`。采用本目录原 `CR7 2.ttt` 的 xMateCR7 六轴网格和零位坐标，迁移 `../UR5/ur5.ttt` 的焊枪、4 组件 GunCollisionHull、冻结 WeldTCP、Workpiece 和场景布置。
CR7 根路径 `/CR7`，六轴 `joint1`–`joint6`，TCP `/CR7/WeldingGunMount/WeldTCP`。
MATLAB 不使用 UR5 关节标定，使用导出零位链和 local Z，qOffset=0。

- ZMQ 23010；`/cr7/joint_trajectory` → 外部 rclpy → setJointPosition。
- `/cr7/joint_states` 由场景 Lua simROS2 只发布，不恢复内嵌 subscriber。
- 新场景所有网格 static、respondable=false、关节 kinematic；用于规划和轨迹回放，不是力矩/接触动力学验证。
- Mount 相对 joint6 为单位变换，这是仿真安装约定，尚无真实 CR7 装枪标定。
- Mount→TCP 位置 `[-0.06677482221230063, 0.019489204955399853, 0.38233021973475156] m`，xyzw `[0,0,0,1]`，不得自动重新拟合或再加焊丝长度。
- MATLAB 的 GunCollisionHull 保持四个凸组件；Workpiece 为原 UR5 四块 5 mm 板；保留原脚本 18 mm 焊丝圆柱和临时默认直径 1.2 mm，方向按原算法从现有 hull 头端到 TCP 构造。
- CoppeliaSim 没有额外加焊丝形状，沿用源 UR5 场景碰撞对象。

2026-10-04 验证：

1. MATLAB/CoppeliaSim 四组 FK，包括 q=0、两组非零 q 和初始 q：最大位置差 `4.34e-16 m`，旋转矩阵 Frobenius 差 `1.08e-15`。
2. 完整 MATLAB→ROS2→ZMQ→CR7→ROS2→MATLAB：1498 条反馈，J1 2° 往返，最大幅度 2.000000°，结束误差 0.000000°。只操作 `/cr7/*`，未向 UR5 控制 topic 发布。
3. 初始 q 的自碰撞/环境碰撞 `[0 0]`；含焊丝的模型检查也通过。
4. `manipulatorRRT` 小幅无碰撞路径验证：15×6 插值状态；80 点原焊缝生成通过。
5. 实体网格和工具预览 `simulation/matlab_scene.png` 已渲染检查；Python 编译、shell 语法检查通过。

回滚：原四个 CR7 场景和原 UR5 场景均未覆盖。`variants/cr7_original/` 保存第一次 CR7 原夹具闭环版本（该夹具整体凸包在初始姿态有环境碰撞，因此没有宣称其规划有效）。原自动焊接脚本已存为文本，新主场景停用；没有修改 CoppeliaSim 全局安装文件。

主入口：`matlab仿真/load_cr7_scene.m`、`sync_cr7_joint_states.m`、`publish_cr7_trajectory.m`、`solve_cr7_ik.m`、`plan_cr7_rrt.m`、`build_cr7_weld_seam.m`。
`rokae_description/CR7_linux.urdf` 是修复 Windows 网格路径后的参考导入文件，实际主模型是导出链。

尚未验收：完整 CR7 焊缝连续 IK、多分支搜索、接近/退出完整路径及焊缝运动执行。不要直接发布 UR5 MAT 文件里的 q 或 UR5 原轨迹。
