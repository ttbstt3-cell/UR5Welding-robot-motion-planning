# 项目维护、版本与恢复

本页为操作说明；已执行结果见`HANDOFF.md`、`setup_verification.json`和维护工具status。
备份工具只用Python标准库/Git，不导入ROS或MATLAB，不启停任何机器人进程。

## 少读上下文，按事件保存

新会话先读根目录AGENTS.md、PROJECT_STATE.md、HANDOFF.md，再按任务查文件。
三份入口合计不超过16KiB；长日志/论文/完整实验保留原目录，只存证据链接。
AGENTS是Codex自动发现的项目规则；STATE和HANDOFF由该规则要求读取，不是假定软件自动载入。
依据：[官方AGENTS规则与发现顺序](https://learn.chatgpt.com/docs/agent-configuration/agents-md)。

每次重要修改：相关测试 → 更新HANDOFF → 审查差异 → 显式checkpoint。
不设置后台定时提交，不在任务中途自动纳入用户尚未完成的修改。

```bash
python3 tools/project_maintenance.py status
python3 tools/project_maintenance.py verify-context
git diff --stat
git diff -- path/to/changed_file
python3 tools/project_maintenance.py checkpoint --message "修复/功能与原因" --tests "真实运行的测试及结论" --paths HANDOFF.md path/to/changed_file
```

checkpoint遇到预暂存修改、疑似密钥或上下文超预算会停止；不重置暂存区。
提交只作用于明确列出的文件，禁止用`git add .`。提交成功但后续备份失败时，提交保留、命令返回失败，不声称已有备份。
Git权限受Codex沙箱限制时，申请针对该checkpoint命令的权限，不改变.git权限或安全配置。
未配置Git身份时，仅该次提交使用明确的`Local Project Maintenance <local-maintenance@localhost>`身份；不修改全局身份。
首次基线原样保留既有文件的空白格式，记录diff检查告警但不批量格式化用户代码；后续增量提交空白检查失败则停止。

## Git与大数据分工

- `backup_policy.json`是范围规则。源代码、URDF、必要配置、精简实验报告及当前请求的小型依赖入Git。
- 初次入库范围保存在`git_paths.txt`，由inventory生成后审查，之后按任务显式增量提交。
- 默认8MiB以上的资产、非当前大型MAT/TTT、视频/PPT/历史局部备份进独立数据快照，不强制全部上传Git。
- `asset_catalog.json`记录Git之外的快照资产路径、大小和SHA256。恢复研究环境需要**Git历史+数据快照**，仅clone Git不等于恢复全部MAT/STL数据。
- `UR5/third_party/IKLink`保留其独立仓库；工作文件进入快照、提交历史另存nested bundle，不改变它的remote/分支。
- IKLink原本是浅克隆，只保存本机已有历史和shallow边界，不能声称拥有上游完整历史。Git 2.34的bundle verify不能独自证明浅克隆可恢复；维护工具另在隔离目录重建并fsck。
- 没有一刀切忽略MAT/STL/实验日志；`.editable_build/notes_draft`中研究PPT仍备份。
- `.gitignore`只排除明确缓存、ROS构建/运行日志、编辑器临时文件、私有工具目录及密钥；不等于数据快照的排除规则。
- 当前未安装Git LFS。若以后采用LFS，只对评估后的路径操作；未获授权不改写历史、不上传大数据。

```bash
python3 tools/project_maintenance.py inventory --write
python3 tools/project_maintenance.py scan
```

扫描只报告路径/行号/类型，不输出秘密值；高可信模式不等于完整安全审计。二进制资产及大于16MiB文本未做内容级密钥扫描。已知密钥文件不入Git/快照；运行密钥需另行安全保管。上传前仍需人工检查，特别是截图/二进制资料。

## 本地增量备份与校验

```bash
python3 tools/project_maintenance.py backup
python3 tools/project_maintenance.py verify
python3 tools/project_maintenance.py catalog
```

`.project_backups/`以内容SHA256去重，保存真实文件字节、原路径/权限、Git bundle和完整清单。对象不与活动源文件硬链接；旧快照不覆盖、不自动清理。
复制前后检查源文件身份和目录清单，有并发变更则失败关闭，不把不一致快照设为latest。不要通过停机器人来制造备份窗口；重要源文件持续变化时，等相关编辑/写入完成后重试。
verify读取所有唯一payload校验SHA256，验证Git bundle并隔离重建每个仓库/fsck；只有全部成功才发布verified/latest。
这属于**同盘恢复点**，不能防磁盘损坏/整机丢失，绝不是已完成云备份。
符号链接只记录目标、不越出项目读取；恢复链接需人工核对。系统MATLAB/ROS/CoppeliaSim安装、许可证及外部链接依赖不在项目快照内。

## 恢复：只能到新的隔离目录

先status取得真实snapshot ID，verify后恢复指定文件；不覆盖现有工作目录。

```bash
python3 tools/project_maintenance.py restore --snapshot SNAPSHOT_ID --destination /tmp/ur5-restore-unique --paths AGENTS.md HANDOFF.md UR5/real_tcp_calibration_active.json
git clone --no-checkout .project_backups/snapshots/SNAPSHOT_ID/root.bundle /tmp/ur5-git-recovery-unique
python3 tools/project_maintenance.py recovery-test --paths AGENTS.md UR5/real_tcp_calibration_active.json UR5/ur5_tool_calibration.mat
```

工具逐文件校验SHA再复制；示例目标必须尚不存在。没有“覆盖当前项目”的自动恢复选项。
完整数据恢复可使用`restore --all --destination 新隔离目录`，仍不覆盖任何既有目录；符号链接需另行审查。
Git历史在另一个新目录clone root/nested bundle，核实后再组织新工作副本；不直接在活动目录批量checkout。外部依赖、密钥和ROS可再生成缓存需独立准备。

## 外接盘/其他机器同步（尚需用户提供目标）

完整备份包是`.project_backups`，其中包含代码/配置/资料payload及Git历史。需复制整个对象库、snapshots和latest，不能只复制manifest。
获得用户明确目标后，创建**新的目标目录**，先试算再复制，不用`--delete`，不覆盖旧备份。可用rsync预演和校验模式；本轮没有外接目标，不声称异地同步完成。
复制后在目标对manifest和对象SHA256复核，并执行Git bundle verify；仅“复制命令退出0”不能代替内容校验。
第三方/研究资料上传权利及截图隐私须先确认。

## GitHub远程（有目标，尚未同步）

用户2026-10-20提供并授权整理后同步：`https://github.com/ttbstt3-cell/UR5Welding-robot-motion-planning.git`。
首次检查仓库为空、public，用户选择先改为private再同步；连接器push=false。本地认证另需实际验证。
在私有状态和写权限核实前不推送；当前上传结果以HANDOFF和最终验证记录为准。

用户在GitHub新建Private仓库，初次建议不自动添加README/许可证/ignore，避免与现有本地历史冲突。
参照[GitHub创建仓库官方说明](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-new-repository)。
提供实际仓库URL并明确授权后，才添加remote；验证身份与仓库可访问性，检查待推送差异/敏感信息/体积后首次push。没有URL时不猜测origin。
Git只同步已提交内容；大型快照仍需独立授权的存储目标。私有仓库也不应保存密钥。
不设置自动推送、强推或后台同步。未执行push且核对远端HEAD之前，不称为云端备份完成。
