# 当前交接

更新时间：2026-10-20（Asia/Shanghai）。启动先读AGENTS、PROJECT_STATE和本文件；详细历史按需检索。

## 当前任务

低Token上下文、本地Git和可恢复快照已建立；用户授权的GitHub私有仓库首次同步已完成并核对远端提交。
本轮只做Git备份/维护记录，不改算法/控制链，不启动HMI/ROS或机器人，不跑MATLAB；大型快照未上传。

## 已实际确认

- main已有基线9a46f22，原代码格式未批量修改；IKLink独立浅仓库保留，HEAD d2edc00…、无已跟踪修改。
- 全量快照约20.8GB、去重约19.4GB。文本高可信密钥扫描无发现；二进制和两个>16MiB的cr7几何JSON未做内容扫描，不是全面安全审计。
- 已建立三份精简入口和维护规则；保留619KB历史AGENT_HANDOFF，旧首页不是最新架构。
- GitHub origin=ttbstt3-cell/UR5Welding-robot-motion-planning。2026-10-20本机gh API实证账号ttbstt3-cell、private=true、push=true；Git正常HTTPS认证可用，未读取或输出凭据，未改变连接器账号。
- 远端原为空；获用户明确授权后正常首次推送main。首个远端提交d0e28e809275a1733e3b81d4a050fe8fbd447317，经API及ls-remote双重核对一致；当前HEAD/远端最终值另实查，不写自身哈希循环。
- 上传范围3559个Git文件，未含快照payload/密钥文件；3414文本高可信扫描无发现，12维护测试PASS，体积与范围证据见setup_verification.json。

## 当前真实业务状态（只读磁盘核对，非本轮新验收）

- 用户后续生成了新Home结果，最新索引已不是旧14:58结果：
  `UR5/规划文件/当前轨迹/保守调试轨迹_50pct_2026-10-20_15-40-06_930602.mat`，报告同名_report.json。
- 当前Home版本：REALHOME-3aff7a1cd79f52495e9a1611ec109078a38c0bf2f4e3d40a9d28caf237af547c；与当前配置匹配。
- 保存报告454.011896秒/638节点，offline_checks全PASS、executionReady=false；详细误差与63701采样证据见同名报告。
- 不推断当前控制器active/External Control/机器人电源；这些是动态状态，本轮未连真机调查。

## 本轮完成与验证

- 新AGENTS.md/PROJECT_STATE.md/HANDOFF.md、.gitignore、维护工具/策略/操作说明；12项隔离测试PASS（源变动失败关闭、损坏检测、恢复不覆盖、预暂存保护、浅克隆重建等）。
- 初始Git明确清单3557文件/620MB，必要URDF/TCP/Home/配置在库。Git外11875个资产由`maintenance/asset_catalog.json`关联已验证快照，研究PPT/原始模型/历史实验不丢弃。
- 已验证快照20261020_174823_9db3a83e：15432文件、14215唯一对象逐一SHA PASS；主仓库+IKLink现有浅克隆历史隔离重建/fsck PASS。
- 12个真实关键文件隔离恢复PASS（含枪STL、ur5.ttt、冻结MAT、真实配置、最新轨迹）；完整20.8GB逐对象校验已做，全体文件恢复未做。证据：`maintenance/setup_verification.json`。
- 备份工具后续checkpoint自动生成/校验增量快照；最终提交和最新snapshot用status实查，避免文档写自身提交哈希的循环。全部同盘，不是异地备份。

## 失败尝试与注意事项

- 初次宿主只读Git授权审查超时，单条git status重试确认无根仓库后才初始化；未覆盖原仓库。
- 焊枪README有早期“未激活”等历史描述，实际manifest ACTIVE/10凸组件；状态以代码与当前记录为准。
- `.editable_build`包含研究PPT，不能一刀切视为可再生成缓存。大型实验资产必须进入独立快照，即使不直接入Git。
- Git 2.34的bundle verify会把原浅克隆包报告“complete”，实际clone缺父提交；必须额外保存shallow并真正重建验证，不能只相信verify退出0。旧快照保留不覆盖。
- 首个快照20261020_174014_6c7011fc的资料字节有效，但缺第三方shallow边界，不用它作为完整历史恢复入口；已由上面的新快照替代，不删除旧数据。
- 授权失败历史：旧连接器账号不同/私库404；本机gh 2.4.0不支持login --git-protocol，改用--web并由用户自行输入终端设备码；审批服务曾因额度未执行检查，之后用户要求继续才重试。均未绕过审批。
- Git曾提示0字节对象临时文件，未清理/GC，不在提交树内且不影响已成功push。

## 下一会话

只读三份入口后运行`python3 tools/project_maintenance.py status`，再按用户新任务定位代码。
同步证据以已核对的首次提交为锚；后续维护checkpoint正常推送，最终HEAD通过status/ls-remote实查。
后续同步仍需当次用户明确授权；不自动推送/强推。GitHub只含已提交内容，不能单独恢复约20GB大型实验资产。
异地待办：用户提供外接盘/存储目标后再试算复制整个快照库并校验；不默认上传20GB资料到GitHub。
不恢复旧TCP/PreWeld/轨迹缓存、不重跑已PASS规划来“验证文档”、不启动真机来测试备份。
