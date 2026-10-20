# 固定底座—地面接触核实、单对例外及冷规划复验

2026-10-15；全部为离线操作。NO ROBOT MOTION。

## 结论

模型证据支持将 `base_link_inertia ↔ EnvironmentCollision_Floor` 识别为正常固定支撑接触，已仅对此特定对象对建立版本绑定例外。没有移除底座碰撞网格、移动地面、放宽碰撞阈值或忽略整个机器人/工具对地面的碰撞。

实际 HMI 当前请求从零重新规划：RealHome、PreWeld、PostWeld、返回 Home 端点全部 PASS；CBi-RRT* 接近/退出分别成功；ACCP 配置搜索与笛卡尔细化成功，47个焊接节点。新173节点预览的3785次节点/边采样中，运动部件/焊枪/焊丝的地面碰撞数为 **0**。

**完整任务仍为 WARNING，executionReady=false**。剩余问题是接近→焊接的 J6 关节圈数接口导致工件碰撞、预览转换后的关节越界，以及 ACCP 现有时间参数化/控制器插值验收失败。没有将这些结果误标为整程 PASS，也没有生成 execute-ready 文件。

## 1. 固定安装接触证据

详见 [`fixed_support_verification.json`](fixed_support_verification.json)。

- 固定链：`world → base_joint(fixed) → base_link → base_link-base_link_inertia(fixed) → base_link_inertia`，不包含任何运动关节。
- 10组关节角下底座 FK 最大变化为0；固定链本身也表明底座不受 J1–J6 运动影响。
- 实际 URDF collision 引用 `/opt/ros/humble/share/ur_description/meshes/ur5/collision/base.stl`。以原始 STL、URDF collision origin 和地面实际 pose 计算，无居中、缩放或人工偏移。
- 底座完整落在5×5m水平地面范围内。
- 底座最低点和地面顶面之间的垂向 bounds 重叠约 **0.002646mm**。这不是 native signed penetration depth。
- 0.01mm只用于“是否符合该固定安装模型”的分类核对，不是规划碰撞容差；任何运动件检测阈值都未改变。
- 结论为 `MODEL_VERIFIED_FIXED_SUPPORT`；`physical_installation_independently_measured=false`。这是模型证据，不宣称完成了现场实物安装测量。仍需确保现场底座实际固定、场景注册与实物一致。

## 2. 仅一个对象对的实现

独立记录：`UR5/fixed_environment_contacts.json`，版本 `fixed-base-floor-20261015-v1`，SHA256 `a787e02ffe3bd27e5ef49eb5d3f46240eb3c8125e4b4dc3e406d1d1841f0adfb`。

记录绑定 URDF、底座STL、环境 JSON 的 SHA。REAL scene 每次加载均重新核实固定链/网格支撑关系；版本变化、地面位姿/尺寸变化、底座位置变化或对象改名均拒绝沿用例外。

`checkEnvironmentCollision` 先执行原生 exhaustive collision query，核对原生标志与距离矩阵一致，然后只将 **base_link_inertia 行 × 指定 Floor 列**排除。保留原始 contact flag，报告 `EXEMPT_FIXED_INSTALLATION_CONTACT`。该矩阵单元设为 Inf 而不是0，避免把允许的固定接触冒充最小安全间隙。其他所有矩阵单元逐值保留。

仍检查：全部运动link↔Floor、Gun↔Floor、Wire↔Floor、base↔其他障碍物、自碰撞、工具/工件碰撞。既有 Gun-Wire 固定内部结构排除不变；没有新增其他 self skip。

CBi-RRT* 配置只透传该显式 policy 到公共状态碰撞后端；ACCP core 只把两处环境查询改接同一后端，候选生成器、Cartesian refinement、控制器离线验证使用相同 policy。搜索流程、IK数学、方向约束、精度阈值、采样参数、TCP/枪/焊丝/工件几何均不变。

`verify_source_scope.py` 将ACCP core中这两处调用替换还原后，逐字SHA恢复为先前 `ffae45d1f8415fc8a2bb6fbcf125504a06fb31d8bbbdb4802c14362a43aa09bd`；见 [`source_scope_verification.json`](source_scope_verification.json)。因此未在该核心中夹带搜索/参数修改。

policy SHA 加入 REAL collision_scene_version、HMI/MATLAB请求和加载依赖快照。冻结仿真原有 scene digest 不受新增可选 policy 字段影响。旧轨迹仍为 STALE。当前 REAL scene version 为 `SCENE-20241f44dfca1a55fad4a387ca3637ce2793c8ccf450706f65a6cd95714ea3ef`。

## 3. 负向回归

[`fixed_contact_regression.json`](fixed_contact_regression.json) 为最终 PASS：

- 无policy时当前Home原生判碰；有policy时Home PASS，原始base-floor接触标志仍保存。
- 10组状态中，原生距离矩阵只有允许单元改变，所有其他项（包括NaN碰撞项）完全一致。
- 全零姿态仍检出 `wrist_2_link/wrist_3_link/WeldingGunMount/WeldingGunWire ↔ Floor`。
- 在独立fixture中给底座增加另一个障碍物，仍判碰；fixture不写入真实环境。
- 地面移动1mm、Floor改名、底座移动1mm均使例外失效。
- 尝试把例外改成shoulder↔Floor被拒绝。
- 原robot body collision数量/标识、全部FK和环境pose不变。
- MATLAB公共CBi-RRT*后端正确接收policy；没有policy的调用保持严格。
- MATLAB静态检查无语法错误（仅残留suppression/性能建议）。

## 4. 当前 HMI 请求冷规划

请求/完整日志在 [`cold_full_preview_v2/`](cold_full_preview_v2/)，汇总在 [`cold_run_summary.json`](cold_run_summary.json)。HMI worker显式禁用，仅捕获按钮实际payload，再调用本机 MATLAB；不读取旧qPath、正式anchor或旧candidate bank。

| 项目 | 结果 |
|---|---|
| Start / Goal / Exit Start / Exit Goal | 全部PASS |
| CBi-RRT*接近段 | 成功，30.003295s，原始15节点，21+62次迭代 |
| CBi-RRT*退出段 | 成功，30.001662s，原始13节点，2+59次迭代 |
| ACCP连续构型 | 成功，2426次IK，24个含桥接的粗节点 |
| Cartesian Bridge | 原第17层恢复，插入3个中间点，后续全部焊点完成 |
| Cartesian Refinement | 47节点，最大TCP几何误差0.244253mm，节点/边碰撞PASS |
| ACCP总时间 | 302.859404s（含构型94.711291s、细化180.796293s及时间阶段） |
| 输出预览 | 接近66 + 焊接47 + 退出60 = 173节点 |
| 完整预览节点/边采样 | 3785次；所有173个节点无碰撞 |
| 非预期地面碰撞 | 0 |
| 完整预览其他碰撞 | 18个边采样，全部在接近→焊接接口 |
| 最终状态 | PREVIEW WARNING；executionReady=false |

CBi-RRT* 的 `timeout_with_solution` 表示在30s anytime预算耗尽时已经找到路径，不是未找到解。

## 5. 剩余问题：与固定接触例外无关

### 接近→焊接关节表示/边界

新预览第66→67帧：J6从约 **371.227115° → 11.227115°**，原始关节差 **-360°**，但TCP XYZ和姿态差均为0。`lift_continuous_joint_path` 的连续表示与后续焊接段的圈数未统一，预览保留未修复接口并插入了一整圈运动。原RRT输出节点本身在限位内；预览lift之后第63–66帧J6分别为362.693°/365.538°/368.382°/371.227°，超出当前上限。

18个工件碰撞采样都位于第66帧出边，首先在u=0.05、t=39.833782s检出 `WeldingGunWire ↔ WorkpieceCollision_wall_y1`，后续还涉及Gun↔bottom/wall_x0/wall_x1/wall_y1和Wire↔bottom。

本轮没有擅自更改CBi-RRT*路径、全圈分支、HMI拼接规则或关节限位。下一步应在轨迹表示/边界衔接层解决圈数一致性，重新验收全部边；不能按周期等价就忽略实际360°运动或把越界节点直接放行。

### ACCP控制器时间轨迹

本次重新生成的焊缝时间参数化并未通过完整控制器等价验收：

- 轮廓误差0.406375mm；
- 时间同步误差2.055985mm（>0.5mm）；
- 姿态误差0.738561°（>0.5°）；
- 最大速度0.091117459rad/s（>0.087266463rad/s）；
- 最大加速度0.835528804rad/s²（>0.261799388rad/s²）；
- 此焊缝控制器采样碰撞和关节限位PASS，但不能替代其余指标。

因此只可说ACCP**几何阶段**成功，不能说完整ACCP/Full Task可执行。没有复用旧47点D速度或改变时间优化模块来取得假PASS。

## 6. HMI与安全状态

[`hmi_completion_test.json`](hmi_completion_test.json)：173节点真实输出可加载，显示PREVIEW WARNING及具体工件pair；live关节状态不被覆盖，最终执行门关闭；旧369点轨迹显示碰撞场景版本不一致/STALE。生产Home配置字节未修改。HMI需重新打开才能加载本轮代码。

没有运行第二阶段执行准备，没有生成 `trajectory_execute_ready` 或正式新anchor，没有发送任何ROS2/UR机器人运动指令。3mm焊丝碰撞缩短仍是原有近似，未改变；0.071979mm最小正间隙不代表已经满足实物独立安全余量。密集采样也不是连续路径的数学安全证明。

## 修改范围与回滚

- 新增：`UR5/fixed_environment_contacts.json`、`RRT/checkEnvironmentCollision.m`、MATLAB `verify_fixed_base_floor_contact/load_fixed_environment_contacts/check_scene_environment_collision/test_fixed_base_floor_contact/summarize_fixed_support_cold_run`。
- 接入：REAL scene builder、Endpoint、preview audit、ACCP core两处环境调用、actual-axis候选环境调用、Cartesian refinement、时间比较/控制器碰撞验证、HMI/MATLAB版本与依赖快照。
- CBi-RRT*仅改公共后端的policy透传；其default/normalize config和公共state/edge接口增加可选policy。RRT这6份对应源码副本同步；历史MATLAB算法源码档案没有用新核心覆盖。
- 未修改：ROS2执行/STOP、IK算法、CBi-RRT*/ACCP搜索、TCP、URDF、底座STL、枪/焊丝/工件/地面几何、冻结scene_geometry/标定/ttt、原始旧轨迹。
- 要恢复严格地面原生检查，可将独立 `fixed_environment_contacts.json` 移出项目根目录保存并重新构建场景。缺policy时不再排除任何base-floor单元，版本改变会使旧轨迹STALE；不要删除底座网格或地面。

完整失败样本、原始q和源模型都保留，可后续针对接口和时间阶段继续诊断。
