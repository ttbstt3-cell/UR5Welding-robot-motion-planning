# 真机执行阻塞：只读诊断（2026-10-20）

## 结论

当前已选638节点连续轨迹的离线规划/控制器插值审计已经通过，不是再次发生ACCP或连续规划失败。
截图有两个独立阻塞：运行控制器inactive、以及软件旧调试范围以上的负载/加速度待验收。
调查期间用户确认机器人已经关机/正在重启；最新断连不能当成截图时一直存在的第三个故障。
本轮没有激活控制器、重启Driver、运行示教器程序、调整倍率、改写TCP、修改执行门或发送轨迹。

## 1. 软件执行范围阻塞（项目源码与实际轨迹确认）

文件`UR5/ur5_hmi/safety/robot_motion_limits.py:205–207`：只要`execution=True`且解析峰值
速度超过LEGACY_V=5°/s，或加速度超过LEGACY_A=0.261799rad/s²（约15°/s²），
就添加`MOTION_LIMITS_PAYLOAD_ACCELERATION_PENDING`。
这是软件保留的旧调试兼容范围，不是UR5本体额定上限。

当前选中：`连续规划完成轨迹_2026-10-20_10-05-24_168245.mat`，638节点，227.017345秒。

| 指标 | 当前轨迹解析值 | 旧软件兼容范围 |
| --- | ---: | ---: |
| 最大关节速度 | 0.169566642rad/s = 9.715453°/s | 0.087266463rad/s = 5°/s |
| 最大关节加速度 | 0.652382403rad/s² = 37.378758°/s² | 0.261799rad/s² ≈15°/s² |

原始配置/URDF交集检查没有报超限，但它们不代表当前负载动态能力已验收。
`load_limits`第105–114行明确尚未接入独立、经验证的安装/负载加速度记录，返回verification=false；
本执行阻塞直接比较旧范围，不会因重复选择离线报告、点击验证或翻转JSON布尔值而解除。
界面缺少可完成此项验收的流程，是当前功能边界，不能把它描述为“用户再点一个按钮即可解决”。

已进行**仅内存数值诊断**：同一q不变、时间×2、速度÷2，得到454.034690秒、
最大4.857726°/s、9.344690°/s²，`check_motion(execution=True)`不再返回此范围阻塞。
没有生成、发布或加载这份候选，没有完成它的全量离线/物理验收，不能称其可以直接执行。
是否采用此兼容候选，或继续保留227秒并实现正当负载/加速度验收，应由用户明确选择。
详见`trajectory_diagnosis.json`，不是执行许可。

## 2. 轨迹控制器确实inactive（ROS只读服务确认）

`/controller_manager/list_controllers`返回`scaled_joint_trajectory_controller=inactive`，
普通joint_trajectory_controller也inactive，joint_state_broadcaster等监视控制器active。
因此不是`ros_node.py:216`的名称解析误报。

`UR5/start_ur5_real.sh:70`明确`activate_joint_controller:=false`；
只读`/controller_stopper/get_parameters`也返回`joint_controller_active=false`、`headless_mode=false`。
启动本身不会授予运动控制权。

当前`run_hmi.sh:138–172`另有原有watchdog，只有mode7、program=true、NORMAL、正倍率才尝试激活。
它依赖多次3秒的独立topic echo，可能受状态只在变化时发布/订阅QoS和发现延迟影响；
截图时是否因该逻辑未取到状态，还是尚未满足条件/未运行watchdog，本轮不能仅凭源码定论。
没有运行该脚本或调用switch_controllers，也没有擅自修复自动激活行为。

## 3. 当前状态已不同于截图（13:35只读采样；用户确认关机/重启）

`readonly_runtime.json`记录9秒内：

- 机器人模式仅TRANSIENT_LOCAL取得mode3（本机RobotMode.msg定义POWER_OFF），VOLATILE无样本。
- 安全模式TRANSIENT_LOCAL为1；机器人程序话题两种持久性均无样本，不能假设为true。
- Dashboard读取返回success=false、未连接；此前一次只读服务曾返回program_running=true，属于不同时间，不能覆盖最新失败。
- 速度倍率当前0，轨迹控制器inactive。
- /joint_states仍持续发布约1124个样本，但这不能证明有新的真机数据。
- Driver日志持续`Currently reconnecting to the RTDE interface, unable to get data package`。

用户随后明确确认“已经关机／正在重启”。当前应先等待操作者恢复机器人及External Control，再重新只读核实链路和控制器；
不能把历史100%倍率/程序true当成当前就绪，更不能因关节话题持续发布而绕过断连。
没有主动重连、上电、释放抱闸、播放程序、切控制器或驱动重启。

## 4. 截图1%不是生效倍率

截图已显示“目标尚未应用”，实际反馈100%；滑块只填写目标，未发送调速。
即使操作者应用较低倍率，当前软件门仍按原文件的q/time/velocity解析峰值检查，
不会把示教器倍率自动当作重新时间参数化并验收后的轨迹。
本轮没有应用1%或100%，也没有为显示可执行而修改原时间戳。

## 证据与限制

- `UR5/ur5_hmi/tools/diagnose_execution_readonly.py`只创建只读订阅和ListControllers/GetParameters/IsProgramRunning客户端，限定9秒，无运动/写接口。
- `readonly_runtime.json`：运行时只读原始结果。
- `inspect_current_trajectory.py`、`trajectory_diagnosis.json`：真实当前MAT解析值与只在内存的2倍时长诊断。
- 机器人/轨迹/标定/碰撞/控制器配置/ROS发送/执行门代码本轮均未改；仅新增诊断工具、输出和交接记录。
- 仍`executionReady=false`，不生成execute-ready文件。本报告不把软件兼容范围等同于物理安全认证。
