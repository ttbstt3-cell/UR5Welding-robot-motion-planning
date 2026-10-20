# HMI连续规划冷启动缺少碰撞适配器：复现与修复

## 根因（源码与运行证据确认）

用户失败请求：`prepare_20261020_094039_057772`，阶段`GEOMETRIC_CONSISTENCY`，2.963145秒失败。
原栈：`hmi_continuous_geometry.m:60` → `check_scene_environment_collision.m:10` → `checkEnvironmentCollision`未定义。
不是rigidBodyTree不支持碰撞，也不是IK/路径不可行；适配器实际存在于`/home/ttbstt/jixiebi/RRT/checkEnvironmentCollision.m`。

HMI `gui/planning_workflow.py:270`生成的新MATLAB命令只添加`UR5/matlab仿真`和`RRT/benchmark`。
连续规划直接load已保存场景，直到最终版本复核才调用`build_real_planning_scene`；其第5行的`addpath(genpath(RRT))`来不及在几何细化使用碰撞检查前运行。
此前独立回归脚本先添加了完整RRT路径，掩盖了这个冷启动缺陷；不能以那些脚本PASS声称HMI实际启动路径已测试。
本次使用真实MainWindow“轨迹连续规划”按钮，捕获原生产QProcess的启动命令（不启动ROS线程），再原样在全新MATLAB运行。
修复前重复得到同一个`MATLAB:UndefinedFunction`和几何阶段栈，2.950489秒失败。

## 最小修复

仅修改生产`UR5/matlab仿真/prepare_hmi_continuous_request.m`：

- 根据入口自身`mfilename('fullpath')`定位项目，而不是借用其他函数的which结果。
- 在读取轨迹及任何IK/碰撞检查之前，显式添加本项目RRT根目录。
- 检查实际解析的`checkEnvironmentCollision`正是本项目文件，路径缺失或同名遮蔽返回`CONTINUOUS:Dependency`，不能继续或绕过碰撞。
- 报告记录依赖实际路径和依赖检查耗时。

没有修改碰撞实现、skip、阈值、几何、IK、速度算法、ACCP、CBi-RRT*、ROS执行代码或HMI启动表达式。
仍然只有原三个算法模块，依赖检查属于输入准备，不是新增规划算法。
`before/`保留原入口，`production_changes.patch`为完整生产差异。

## 实际失败输入重跑与HMI接收结果

同一预览SHA `2b1f49af7a4dae01cc9f13f30744036d41566f149bddcd01cdc632c84f600595`，
同一Home `REALHOME-b8b8b4ede731db946b3da967c8a07ca07fb8b2fa47b19fcbcfd1d193a673c63f`，所有版本/阈值不变。
第二次真实按钮捕获的命令仍只添加matlab仿真和benchmark，没有在测试前偷偷加genpath。

修复后完整输出：`../prepare_20261020_094748_672689/report.json`。

| 项目 | 结果 |
| --- | --- |
| 完整离线连续规划 | PASS，145.022300秒 |
| 几何一致性 | PASS，11.304037秒 |
| 时间/速度参数化 | PASS，1.484127秒 |
| 全程控制器等价验收 | PASS，125.269439秒 |
| 版本及当前Home | PASS，3.552100秒 |
| q节点／原焊接目标 | 638／47 |
| 密集采样／碰撞／关节限位 | 63701／PASS／PASS |
| 最大TCP轮廓误差 | 0.248496522mm |
| 最大TCP时间同步误差 | 0.286312952mm |
| 最大姿态误差 | 0.261428939° |
| 最大段内jerk | 4.938160944rad/s³ |
| 状态 | PHYSICAL_VALIDATION_PENDING |
| executionReady | false |

`tools/test_continuous_launch_paths.py`确实点击真实Qt按钮并记录完整生产启动表达式：`before_launch.json`/`after_launch.json`及两个日志。
未从测试脚本预加载RRT根目录，未调用机器人接口。
`verify_result_and_hmi.py`用真实完成回调分别回放原失败及新成功：原错误保留为失败，新结果在REAL CALIBRATION下正常加载638点、报告/另存可用、执行仍禁用。
初次测试窗口默认SIMULATION，因此正确触发Version mismatch；随后仅在测试窗口切换到与用户请求相同的REAL CALIBRATION后通过。保留为额外反例，不能取消版本检查以让测试通过。

`dependency_guard_tests.json`：同名适配器被遮蔽时前置拒绝；原始轨迹SHA错误仍拒绝；两种失败均不导出优化轨迹，保持executionReady=false。
`protected_verification.json`以本轮开始时的实际数据为基准，28个已有文件仅计划内入口变化，配置/Home/当前轨迹索引/模型/碰撞规则和ROS代码不变。
新增测试请求是隔离离线输出，结果没有发布到用户当前轨迹索引。没有重启用户HMI、修改RealHome、写TCP或发送运动。

## 使用

不必重新标定、修改RealHome或重新计算预览。当前无规划进程时，在现有HMI再次点击“轨迹连续规划”即可；它会启动新MATLAB并读取修复后的入口。
离线PASS仍不等于真机许可，物理/伺服及原有发送前检查继续有效。
