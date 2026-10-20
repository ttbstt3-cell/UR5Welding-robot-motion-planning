# 当前交接

更新时间：2026-10-20（Asia/Shanghai）。启动先读AGENTS、PROJECT_STATE和本文件；详细历史按需检索。

## 当前任务

建立低Token上下文、明确文件范围的本地Git、可校验/恢复/复制同步的资料备份机制。
本轮不改算法/控制链，不启动HMI/ROS或机器人，不重新跑MATLAB规划，不推远程。

## 已实际确认

- 根目录此前无Git；现已初始化main并提交基线9a46f22（3557文件，原有格式不批量修改）。第三方`UR5/third_party/IKLink`保留，HEAD d2edc00…，已跟踪修改为空。
- 资料约20GB，主要MAT约15GB、视频约1.4GB；既有UR5/backups和backup_before_hmi_weld_20261008是局部历史副本，不是完整/异地备份。
- 项目磁盘尚有约586GB空闲（检查时）。3410个文本文件高可信密钥模式扫描无发现；二进制和两个>16MiB的cr7几何JSON不在内容扫描覆盖内，不能声称完全无敏感信息。
- 已建立三份精简入口和维护规则；保留619KB历史AGENT_HANDOFF，旧首页不是最新架构。
- GitHub已配置origin到用户提供的ttbstt3-cell/UR5Welding-robot-motion-planning。首次核实空仓库/public/push=false；用户要求先改私有。后续只读API返回404，无法区分私有无权与不存在；不能据此确认。推送预检被安全审查拒绝，未执行、未上传，不绕过。需用户核实私有状态及授予Git写权限。Git LFS未安装，不自动迁移。

## 当前真实业务状态（只读磁盘核对，非本轮新验收）

- 用户后续生成了新Home结果，最新索引已不是旧14:58结果：
  `UR5/规划文件/当前轨迹/保守调试轨迹_50pct_2026-10-20_15-40-06_930602.mat`，报告同名_report.json。
- 当前Home版本：REALHOME-3aff7a1cd79f52495e9a1611ec109078a38c0bf2f4e3d40a9d28caf237af547c；与当前配置匹配。
- 保存报告：454.011896秒，638节点/63701采样；全部offline_checks=true，轮廓0.248498mm、同步0.286316mm、姿态0.261429°，executionReady=false。
- 不推断当前控制器active/External Control/机器人电源；这些是动态状态，本轮未连真机调查。

## 本轮修改与验证进度

- 新AGENTS.md/PROJECT_STATE.md/HANDOFF.md、.gitignore、维护工具/策略/操作说明；12项隔离测试PASS（源变动失败关闭、损坏检测、恢复不覆盖、预暂存保护、浅克隆重建等）。
- 明确候选清单约3557文件/620MB，必要URDF/TCP/Home/配置已在清单；大资料另存约20.8GB快照。研究PPT不排除；Git忽略规则不等于备份排除。
- 首个快照15432文件/14215唯一对象、19.4GB实际对象，SHA校验PASS；12个关键文件恢复PASS，主Git历史恢复PASS。但IKLink浅克隆bundle无法直接clone，首个快照不能视为该嵌套历史完全可恢复。
- 已修复备份工具：记录原shallow边界、离线重建仓库/fsck；不fetch或改变原IKLink。接下来提交工具修复并生成新快照，完成真实重建后才报告整体PASS。

## 失败尝试与注意事项

- 初次宿主只读Git授权审查超时，单条git status重试确认无根仓库后才初始化；未覆盖原仓库。
- 焊枪README有早期“未激活”等历史描述，实际manifest ACTIVE/10凸组件；状态以代码与当前记录为准。
- `.editable_build`包含研究PPT，不能一刀切视为可再生成缓存。大型实验资产必须进入独立快照，即使不直接入Git。
- Git 2.34的bundle verify会把原浅克隆包报告“complete”，实际clone缺父提交；必须额外保存shallow并真正重建验证，不能只相信verify退出0。旧快照保留不覆盖。

## 下一会话

完成上面的维护配置后，用维护工具读取真实最新HEAD/快照；未解决的业务任务需用户指定。
不恢复旧TCP/PreWeld/轨迹缓存、不重跑已PASS规划来“验证文档”、不启动真机来测试备份。
