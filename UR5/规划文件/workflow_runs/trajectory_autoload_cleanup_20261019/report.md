# 规划结果自动导入、易读命名与旧轨迹清理

## 用户现在如何操作

重新打开HMI后，正常流程为“规划轨迹 → 轨迹连续规划”。预览结束自动载入预览；连续规划成功自动载入对应结果并切换“轨迹执行”页，不再手工寻找文件。此后仍需“检查执行条件 → 人工确认执行”，本轮没有发送机器人运动。

手动导入按钮改为“导入轨迹…”，默认定位`规划文件/当前轨迹/`中最新结果。显示区明确区分预览与连续规划完成结果，并显示实际文件名；导入带报告的预览可恢复连续规划输入，原版本校验不变。

## 文件命名及一致性

- `预览轨迹_仅预演_YYYY-MM-DD_HH-MM-SS_请求序号.mat`
- `连续规划完成轨迹_YYYY-MM-DD_HH-MM-SS_请求序号.mat`
- 同名`_report.json`配套报告；`最新轨迹索引.json`记录当前选择。

发布的是原MAT/report逐字节一致副本，没有重新序列化MAT、改q/时间/速度/metadata，未改变原executionReady=false。执行软件验收仍用原完整JSON参考/数值门/版本绑定。副本不是自包含场景包，索引记录source_run，清理必须保留当前请求及快照等依赖。新预览清除旧连续规划的“当前结果”选择，避免误导。

当前保留的真实结果是`preview_20261019_110448_976674`的416点预览，匹配当前RealHome `REALHOME-5b20a8f35e931ffdb3df84a9b7e3998f1a39af8d712baf6f625e6f3da9bd952a`及场景版本。当前易读文件：

`/home/ttbstt/jixiebi/UR5/规划文件/当前轨迹/预览轨迹_仅预演_2026-10-19_11-04-48_976674.mat`

它不是当前RealHome下已完成连续规划的执行轨迹；此前连续规划结果绑定旧Home，按用户要求作为旧HMI轨迹清理，不能自动套用旧PASS。

## 修改范围

新增`trajectory/result_store.py`负责字节一致副本和最新索引。
`gui/planning_workflow.py`在成功回调发布/导入易读副本，连续规划后切换执行页；发布失败只记录命名/复制问题并仍载入原结果，不伪造算法失败或PASS。
`gui/main_window.py`提供易读名称、最新结果定位及带报告预览的导入恢复。
`gui/execution_preparation_panel.py`优先关联同名报告，兼容旧report.json。
新增定向清理工具及独立回归工具。CBi-RRT*/ACCP-D/IK、速度优化、TCP/模型、碰撞阈值/过滤和ROS2控制链不变。

## 验证

- `result_store_tests.json`：真实MAT/报告改名副本逐字节一致，配套报告完整软件验收通过；阶段错误、碰撞FAIL发布、被篡改最新轨迹均拒绝；新预览不能选择旧连续规划结果。
- `vtk_autoload/responsiveness_tests.json`：实际VTK界面、实际QPushButton/QProcess回放真实已保存优化结果；自动导入易读副本、关联报告、切换执行Tab和取消全部PASS，发送0次。使用历史隔离Home及合成未就绪状态，未启动ROS/RTDE，不是新一轮算法规划或真机验收。
- `execution_regression/simple_execution_tests.json`：26项失败反例拒绝，100/125/150%布局通过；名称/导入没有绕过软件门。
- `current_import_after_cleanup.json`：清理后全新HMI手动导入，默认选中实际当前416点易读预览，配套报告和连续规划输入恢复正确，原生产YAML未改，执行禁用。

## 已执行的可恢复删除

先生成`cleanup_inventory.json`，按完整清单摘要`f854882a80caad2834d92a56088474e39193b48d4b4d3ef3a308d561c9501995`复核，再逐个校验SHA并通过系统`gio trash`移动到回收站。未使用永久删除。操作时没有HMI或MATLAB规划进程。

共78个旧HMI轨迹负载文件、8,953,839字节：旧`planning_runs/hmi_*`及其接近/退出轨迹36个；`规划文件`旧日常发布/preview/prepare轨迹40个；`轨迹文件`旧轨迹2个。未删除完整请求目录，旧日志/报告仍在；没有清理MATLAB算法实验数据、comparison结果、冻结anchors、模型、标定或备份，也没有删除当前请求依赖。

`cleanup_inventory_trashed.json`记录逐项成功结果；`cleanup_verification.json`确认78个原路径均不存在，78个都能在系统回收站找到原路径恢复信息，当前副本/源请求完好。可从桌面回收站恢复。

注意：部分历史诊断脚本引用已删除的旧HMI轨迹，若以后需要重放该历史请求，请先从回收站恢复相应原路径；不能把已删历史负载仍当作现存文件。

NO ROBOT MOTION。
