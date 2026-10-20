# UR5来源限值统一与2倍候选复核

2026-10-19。当前完成：源限值读取/绑定、离线放宽、超限拦截、2倍离线审计。未完成：提速真机执行放行。没有运动、变速、TCP/负载写入或控制器配置修改。

## 结论与剩余条件

原5°/s、0.261799rad/s²为项目阈值，已不作为新HMI离线请求的本体上限。新的上界来自实际配置与URDF、用户照片的交集。**ConfigurationData不是安装安全配置或随负载变化的动态能力曲线**；不能把40rad/s²当成当前焊枪可直接执行的加速度。

两张照片已确认收到并记录，不再要求重复提供关节速度页。尚缺工具负载/重心与实物一致性的确认及适用加速度/伺服验证。只读RTDE当前登记0.5kg，并不证明实物重量/重心正确。因此新增限值当前仅用于离线规划和审计，提速轨迹执行保持阻止。不是通过修改executionReady或安全门声称物理PASS。

## 证据与生效值

| 来源 | 读取/观察 | 处理 |
| --- | --- | --- |
| 本机192.168.0.10:30011，445字节ConfigurationData | 每轴jointMaxSpeed=3.341592653589793rad/s，jointMaxAcceleration=40rad/s² | recv-only；保留原始报文/解析值/SHA，不发送URScript |
| 当前校准URDF | 每轴velocity=π rad/s；原位置范围含J3±π | 取更严模型交集，不修改URDF或限位 |
| 用户照片2：Normal关节限制 | 六轴显示191°/s，旁列-11°/s | 记录照片数值；最终仍受更严180°/s模型值限制 |
| 用户照片1：Normal一般限制 | TCP1500mm/s，力150N，功率300W，动量25kg·m/s | TCP速度增加保守上界检查；力/功率/动量仅记录，没有动力学验收结果，不能宣称物理PASS |
| 当前只读RTDE | payload=0.5kg，payload_cog=[-0.006411568,-0.016572140,0.390822876]m | 仅控制器登记值，不是实物称量/重心证明 |

统一离线关节速度上界πrad/s（180°/s），加速度配置上界40rad/s²。按每轴检查；既有只接受标量的优化器用六轴最小值，最终独立极值检查仍逐轴执行。Normal之外原实时门仍阻止执行。

`UR5/robot_motion_limits.json`保存限值与原始数据，SHA为`8eaa422c8c8856ecc5f7745a8add8716de6cbe6983813a4d6d1afa5651dd03a3`。照片副本在`UR5/robot_motion_limit_evidence/`，原始配置报文SHA为`16f37086fb9d6bb38b68bf8c5d4d8d0d09e1c4c789250dfda3b6f5de61e65f7e`。[只读负载快照](payload_readonly.json)保留空RTDE input recipe与output_only标志。

快照不是示教器设置的自动持续同步；设置、模型、工具改变后须重新采集/核对及重新验证。没有解析或猜测UR未公开的SafetyData子包，也没有修改示教器安全设置。

## 统一链路与拦截

1. HMI新连续规划请求加入`motion_limits_file`及SHA。
2. MATLAB独立核对文件、URDF、原始网络报文、解析值与照片哈希，设置当前请求scene的`hmiMotionLimits`。只更新时间/速度阶段的阈值输入，不修改构型搜索、IK、Bridge、RRT或几何/碰撞规则。
3. 新报告与轨迹metadata绑定同一motion_limits；生成中限值变更会失败。Python与MATLAB派生契约逐字段一致。
4. HMI离线状态机用已核对的新契约判断数值，不再错误套用旧5°/s。没有契约的旧结果仍按旧阈值解释，不得冒充新验收。
5. 执行页与worker独立重算P+V三次曲线：端点及段内速度极值、线性加速度端点峰值、位置驻点；不只看节点或平均速度。NaN、缺时间/速度、不同关节顺序、不同插值字段、模型/配置失效及超限均拒绝。worker拒绝在真正action发送前发生。
6. TCP线速度采用所有姿态有效的三角不等式杠杆长度上界，不把采样差分峰值当数学证明；超过保守界时拒绝并提示需要更精细验证。图中的力、功率、动量不能由这项运动学检查替代。
7. 超过旧低速兼容范围而负载/可用加速度尚未核实时，HMI及worker均显示`MOTION_LIMITS_PAYLOAD_ACCELERATION_PENDING`，不会因离线配置范围PASS就放行。JSON布尔字段不能解除此项。

原轨迹位置、时间、速度没有自动修改，本轮未替换原294.880s文件或当前索引。不更改UR速度滑块1%～100%的接口含义，不把100%改成200%。已启动旧HMI需在停止/无任务后重开；没有自动关闭旧实例或驱动。

调查期间用户另外保存了新RealHome并完成`preview_20261019_151534_200122`；15:18后的当前索引为这份预览、prepare=null，RealHome为`REALHOME-a3b74aa159f6c5b30239fc95a32d10aaec3f58fe00efafde8b0b9d7934bf84d7`。没有恢复或覆盖用户新索引。下述2倍候选属于原12:14已记录场景/Home版本；它不能作为新Home任务的执行依据。专项测试改为显式指定不可变历史输入，不能假定用户最新选择永远是prepare。

## 当前2倍候选验证

保持471个q节点（47焊接节点）、TCP目标、几何、碰撞规则不变，仅`t/2,V*2`。来源是`当前轨迹/连续规划完成轨迹_2026-10-19_12-14-01_560570.mat`，SHA`7bab695b30f6069dbc0408a37423c3e1eb01f57fbc6271f6bea3bb41c488ce0c`，与前次诊断相同。

| 指标 | 2倍候选 | 当前离线配置范围 |
| --- | ---: | --- |
| 时长 | 147.440017s（原294.880033s） | 不主动要求满额速度 |
| 最大关节速度 | 0.169347484rad/s，9.702896°/s | ≤πrad/s，PASS |
| 最大关节加速度 | 1.046874709rad/s²，约59.982°/s² | ≤40rad/s²配置上界，数值PASS，不是负载验收 |
| TCP轮廓最大误差 | 0.291503mm | <0.5mm，PASS |
| TCP时间同步最大误差 | 0.462008mm | <0.5mm，PASS |
| 姿态最大误差 | 0.414448° | ≤0.5°，PASS |
| 碰撞/关节限位 | 47,001点PASS | 原规则不变 |
| 最小正间隙 | 0.459441mm | 未认证物理安全余量，不能忽略标定误差/模型近似 |
| TCP线速度全程保守界 | Python0.741335m/s；MATLAB0.752234m/s | 均<1.5m/s，PASS |
| 离线配置审计 | PASS | 不是完整实际执行能力证明 |
| 真机执行 | BLOCKED | 负载/适用加速度/实际伺服待验证，executionReady=false |

[完整密集审计](speed_2x_offline/speed_2x_validation.json)及`matlab_2x.log`；随后新增的[TCP线速度证明](tcp_speed_bound.json)是同一不可变候选的辅助审计，没有伪装成又跑了一次碰撞。MATLAB杠杆长度来自rigidBodyTree固定链，Python来自URDF+固定flange/TCP长度；两种保守界因固定链分解不同而不同，不是TCP实际位置偏差。MATLAB另以50个配置检查Jacobian平移列范数不超这些杠杆界。

等比例时间缩放保持理想Hermite空间曲线不变，不代表真实伺服误差相同；段内jerk及节点加速度跳变仍存在。没有修改阈值来隐藏TCP/碰撞失败，也没有导出2倍轨迹或执行许可。

## 测试与限制

- [专项测试](python_tests/motion_limits_tests.json)：17个负例含段内速度峰值、加速度峰值、位置过冲、TCP速度界、非有限值、零dt、缺速度、URDF/报文/照片/关节顺序/契约错误、伪造summary PASS及旧报告套用新限值，全部拒绝。假worker中2倍候选在发送前阻止，send_calls=0；GUI检查缓存随轨迹数值改变失效。新契约离线PASS仍为PHYSICAL_VALIDATION_PENDING的状态测试为合成报告，不是新正式轨迹。
- [最终执行门回归](execution_regression_final/simple_execution_tests.json)：原26项反例拒绝、100/125/150%布局通过，旧有效低速结果未被错误拒绝。所有robot-ready状态为合成，无真机动作。
- [真机反馈显示回归](live_regression/speed_live_tests.json)：真实VTK+合成反馈、假service/action，PASS，无真实调速或运动。
- [Qt按钮/进程回放](gui_regression/responsiveness_tests.json)：既有MATLAB结果重放，不是新默认阈值的完整冷规划；用于确认新请求参数、异步完成和界面响应。未宣称完成新阈值下的整套连续优化冷启动回归。
- MATLAB/Python限值契约一致；旧实验未携带新请求字段时仍用原5°/s、15°/s²（adaptive原精确deg2rad(15)）以保留历史实验。没有批量改写历史结果或算法参数文件。
- Python语法、MATLAB限值加载/配置分支及辅助速度证明通过。无物理验收或执行许可文件。

## 修改文件

新增：`UR5/robot_motion_limits.json`、`robot_motion_limit_evidence/`两张照片副本、`ur5_hmi/safety/robot_motion_limits.py`、`ur5_hmi/tools/test_robot_motion_limits.py`、`matlab仿真/hmi_load_robot_motion_limits.m`、`hmi_scene_motion_limits.m`、`hmi_tcp_speed_upper_bound.m`。

修改：`ur5_hmi/gui/planning_workflow.py`、`gui/main_window.py`、`gui/execution_preparation_panel.py`、`safety/execution_preparation.py`；`matlab仿真/prepare_hmi_continuous_request.m`、`hmi_prepare_deferred_weld_timing.m`、`generate_accp_adaptive_velocity_profile.m`、`optimize_accp_hybrid_velocity.m`、`optimize_hmi_cubic_node_velocities.m`、`optimize_hmi_controller_continuity.m`、`adaptive_time_scale_accp_output.m`、`hmi_export_offline_continuous_result.m`、`diagnose_hmi_uniform_speed_scaling.m`；操作文档与交接文档。旧生产源位于`before/`。

未修改CBi-RRT*、ACCP构型搜索/IK/Bridge、机器人URDF、TCP、焊枪/焊丝/工件/地面、碰撞阈值/过滤、控制器YAML或ROS2驱动；发送接口及轨迹数据不变，只增加发送前校验。

## 官方依据

[UR Primary/Secondary协议](https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/primary-secondary-guide.html)说明30011为只读端口及ConfigurationData字段；[UR官方客户端解析实现](https://raw.githubusercontent.com/UniversalRobots/Universal_Robots_Client_Library/master/src/primary/robot_state/configuration_data.cpp)与本机安装头文件一致。只使用已定义的配置字段，没有反解内部SafetyData。

已安装`/opt/ros/humble/share/ur_description/config/ur5/joint_limits.yaml`明确标注加速度限值未公开；因此不把官方URDF未给出的加速度或程序默认值虚构为已验收硬件能力。配置报文中的40仅是本机返回的配置值。NO ROBOT MOTION。
