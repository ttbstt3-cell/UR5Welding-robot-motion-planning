# 简化的真机调试执行入口

2026-10-19。用户先要求补齐执行准备页，随后明确要求取消多项外部验收/授权设置，使用规划结果进行人工确认执行。
本报告对应最终简化方案，不是此前已取消的多表单方案。

## 最终界面

轨迹页顶部：加载轨迹、检查执行条件、3D轨迹预演、确认并执行（按轨迹速度）。
自动读取轨迹同目录report.json；仅缺少报告时手动选择。不需要外部现场报告、草稿或独立许可文件。
每次发送仍弹出原有四项人工现场确认，确认结束后再检查一次，防止模态对话框期间状态变化。
一秒定时刷新软件条件；尚未进行的物理/伺服验收不会被标成PASS。

新增GUI模块`gui/execution_preparation_panel.py`和数据校验` safety/execution_preparation.py`。
`main_window.py`接入简化检查、把旧5%误导名称改为按轨迹速度、保留全部实时门并增加确认后复核；
长文件名另行显示、长值允许收缩/换行，避免右半屏水平裁切。

用户明确选择人工现场调试执行，因此完整离线输出在软件检查通过、现场人工二次确认后可以沿现有发送入口执行；
原离线元数据executionReady=false/PHYSICAL_VALIDATION_PENDING仍不变，不伪造正式物理验收，也不导出正式许可文件。
这不是自动运动，本文及测试期间没有执行机器人。

## 必要检查保留

要求原始MAT与报告记录一致、内存q/time/velocity与JSON参考一致、完整离线数值/碰撞/限位/版本门通过、
RealHome及全部真实模型/场景绑定一致；标准ROS关节顺序/弧度端点和已验证的P+V字段一致。
完整离线审计复用同一不可变轨迹的证据，不在点击检查时重跑约8分钟优化。
文件哈希缓存以inode/size/mtime_ns/ctime_ns为键，内存轨迹用全部原始浮点字节计算摘要，无取模/近似关节缓存。
机器人新鲜反馈/真实链路、程序运行、RUNNING、NORMAL、正倍率、controller active、action ready、1°起点门及并发保护保留。
CBi-RRT*、ACCP、IK、TCP/几何、碰撞参数/过滤、ROS2发送器/控制器未修改。

## 验证

`simple_execution_tests.json`：25项反例拒绝，包括碰撞失败、限位失败、误差/加速度超限、样本不足、部分成功、
伪造executionReady、版本失败、q/速度/时间被改、不同控制器字段、Home变化、原始预览、控制器/程序/安全模式/电源异常、
action不就绪、零倍率、断链、反馈过期、起点不符。
正例使用真实469点离线结果；GUI ready状态是SYNTHETIC，仅验证按钮条件，不是现场物理证明。
测试只执行取消确认及确认期间掉线两条发送拒绝分支；没有调用成功发送分支，motion signal为0。
原MAT和生产YAML逐字节不变。

`simple_offline_contract.json`：原16项离线假PASS反例全部拒绝。
100%/125%/150%离屏GUI截图分别为simple_execution_100/125/150.png；新区域宽度均不超过右侧viewport。
截图显示无真实连接的pending状态；左侧是离屏骨架回退，不是实时VTK桌面截图。

早期复杂界面及测试源/截图保存在advanced_ui_superseded/，不在运行路径内。
before/main_window.py是本轮前版本，保留回滚。完整操作见ur5_hmi/docs/continuous_planning_and_execution.md。
NO ROBOT MOTION。
