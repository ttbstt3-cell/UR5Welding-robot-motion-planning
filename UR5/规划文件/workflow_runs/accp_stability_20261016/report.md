# ACCP预览失败根因、最小修复与回归报告

调查与复验日期：2026-10-16～17。NO ROBOT MOTION；所有结果executionReady=false。

## 结论

不是精确缓存造成随机失效，也不是性能优化放松碰撞后生成错误PASS。是此前Cartesian Bridge候选恢复逻辑漏检：诊断池本来保留了被硬方向约束拒绝的IK记录；Bridge恢复端点时只重新检查碰撞，将一个已拒绝端点重新加入构型图。最终严格Cartesian Refinement正确阻止了整条预览。

问题在性能优化前的归档源码中同样复现；相同冻结输入、缓存开关各5次，10次输出逐值一致并全部失败。已只补齐Bridge端点的原硬检查，未修改IK、角度/碰撞/限位/精度阈值或ROS2链路。

正式入口连续5次完整预览全部PASS；原成功请求47个q节点和TCP目标逐值不变；关闭缓存的修复后原生计算也与成功结果逐值一致。2026-10-17另通过真实MainWindow按钮→QProcess MATLAB→界面加载完成第6次同输入预览，Preview Ready、机器人实时状态未覆盖、执行门仍BLOCKED。

## 1. 原始失败结果到底完成了什么

来源：`../preview_20261016_144235_155027/preview_accp.mat`，实际变量只有result，不是已经通过的轨迹文件。

- 原始焊缝21层。第17层一度NO_CONNECTED_CANDIDATE，Cartesian Bridge执行并插入3层，恢复为24层，最终1条可达构型链。
- configurationComplete=true；configurationNodeCollisionFree=true；configurationEdgeCollisionFree=true；graph测试119条边、83条通过，36条被碰撞检查拒绝（包含Home到首点的直接边；该直接边不代表后续CBi-RRT*接近段）。
- Refinement调用1次、确实运行完成，但**验收失败**：24个稀疏节点形成45个诊断节点，失败稀疏边19/20，对应诊断输出边37/38，共享诊断节点38。
- adaptiveCorrectionComplete=false、complete=false、highPrecisionPathAvailable=false、nodeCollisionFree=true、edgeCollisionFree=false；公开qPath/qPathExecution为空。失败诊断45点不是可加载的合法路径。
- 最大节点位置误差0.000137mm，最大边位置误差0.656519mm（超过0.5mm），最大边姿态误差0.069892°。时间参数化未运行。
- PreWeld候选存在，单点限位及原生碰撞检查FREE；但没有完整合法焊缝，原请求Goal State=NOT_GENERATED、未导出正式PreWeld、未进入RRT。不能把候选首点当成已经通过的PreWeld。
- 最终分类是PRECISION_FAILURE（包含硬节点方向约束不合格），不是最终IK_FAILURE、未恢复的NO_CONNECTED_CANDIDATE或输出结构错误。曾恢复的第17层只是搜索事件，旧日志提取逻辑把它误当最终失败点，现已区分。

### 严格定位到一个端点

原稀疏节点20（对应原Point17的Bridge终点），也是诊断输出节点38：

- q(deg)约[-84.770881, -46.583401, 33.077515, -61.391720, -81.971684, -1.017088]。
- 实际轴误差**5.0000000335531976°**；原硬上限5°。
- 原诊断池159中的记录已经标为`rejection_reason="orientation_direction"`。证据：`offending_candidate.log`。
- Bridge将此数值/位姿有效但方向不合格的记录转成endpoint时，仅调用localFullStateFree（碰撞），没有重走方向/焊接侧/位姿硬检查。
- 细化的hardState没有容差放宽，拒绝该节点。第37边101点、第38边113点额外原生审计：**0个物理碰撞**；两边各在共享端点发现1次方向拒绝，无侧向位置或限位拒绝；最小有限正模型距离0.506009mm。
- `segmentCollisionFree`在原refinement中同时累计“碰撞且hardState”，不能将false直接翻译成真实碰撞。修复不增加5°容差，而是拒绝错误端点、选择其他合法分支。位置0.656519mm失败也保留，没有屏蔽。

## 2. 每层真实记录及边复验

下表来自saved_result_audit.json与saved_graph_replay.json；图回放逐层可达状态及总119/83边与原MAT完全一致。Bridge阶段共记录586次IK调用，原MAT没有按插入层保存分项，不能捏造。表中的“主IK”是原主连续搜索计数，不包含Bridge和评分lookahead；Point17覆盖前258次由日志/累计计数恢复。总体主IK2354、原性能探针实际记录3276次IK（涵盖额外调用）。

“池碰撞拒绝”是保留的数值/位姿有效诊断记录计数，可能含重复种子/缓存返回；不是独立构型总数，也不是最终路径碰撞数。

| 层 | 主IK调用 | 去重节点候选 | 保留状态 | 图边测试/通过 | 图边碰撞拒绝 | 池碰撞拒绝 |
|---|---|---|---|---|---|---|
| 1 | 32 | 1 | 1 | 1/0 | 1 | 17 |
| 2 | 72 | 2 | 1 | 1/1 | 0 | 8 |
| 3 | 72 | 2 | 1 | 1/1 | 0 | 8 |
| 4 | 72 | 2 | 1 | 1/1 | 0 | 8 |
| 5 | 72 | 2 | 1 | 1/1 | 0 | 8 |
| 6 | 72 | 2 | 1 | 1/1 | 0 | 8 |
| 7 | 72 | 2 | 1 | 1/1 | 0 | 8 |
| 8 | 72 | 2 | 1 | 1/1 | 0 | 8 |
| 9 | 72 | 2 | 1 | 1/1 | 0 | 8 |
| 10 | 72 | 2 | 1 | 1/1 | 0 | 8 |
| 11 | 72 | 2 | 1 | 1/1 | 0 | 8 |
| 12 | 72 | 2 | 1 | 1/1 | 0 | 8 |
| 13 | 96 | 2 | 1 | 1/1 | 0 | 32 |
| 14 | 72 | 2 | 1 | 1/1 | 0 | 8 |
| 15 | 96 | 2 | 1 | 1/1 | 0 | 32 |
| 16 | 72 | 2 | 1 | 1/1 | 0 | 8 |
| 17 | Bridge共享预算 | Bridge未保存 | 5 | 5/5 | 0 | 未逐层保存 |
| 18 | Bridge共享预算 | Bridge未保存 | 5 | 25/23 | 2 | 未逐层保存 |
| 19 | Bridge共享预算 | Bridge未保存 | 5 | 24/18 | 6 | 未逐层保存 |
| 20 | 258（覆盖前日志差分） | Bridge未保存 | 5 | 19/12 | 7 | 83 |
| 21 | 360 | 6 | 3 | 13/4 | 9 | 33 |
| 22 | 216 | 3 | 3 | 9/3 | 6 | 16 |
| 23 | 216 | 2 | 2 | 6/2 | 4 | 32 |
| 24 | 144 | 1 | 1 | 2/1 | 1 | 24 |

第一层：32次IK，32数值成功，14关节限位拒绝，18位姿成功，17工件碰撞拒绝，1接受；“与Home连续性拒绝18”仅为诊断，不能用焊缝步长限制否定单独的自由空间接近规划。

其余保存池合计2150条位姿有效记录，方向拒绝1316、环境拒绝356、自碰撞拒绝0、接受记录478。原生距离矩阵定位保存池中实际碰撞对为`WeldingGunMount ↔ WorkpieceCollision_wall_x1`。这些被拒绝候选不是最终失败边；最终边无碰撞，失败是上面的方向端点漏检与精度。完整逐层pair见failed_layer_statistics.csv。原MAT未保存的全部拒绝候选统计不冒充已知。

## 3. 成功请求与失败请求并非同输入

比较成功preview_20261016_142704_413362（以及performance目录）与失败144235：

- 成功seed=20260917，失败seed=20260919。
- RealHome由REALHOME-1758…变为REALHOME-00fce…。
- 六关节差(deg)：[-0.00136604, -0.42614916, 0.04324876, 0.05343940, -0.00620181, 0.00206272]。尤其J2变化约-0.426°。
- TCP、焊枪、安装变换、工件、URDF、地面、固定安装接触、焊丝及碰撞场景版本/哈希全部一致；完整calibration_binding逐字段相等。
- scene MAT哈希不同：保存了不同qMat0/Home，不能将其等同于几何不同。两次缓存绑定分别记录各自Home、seed和scene SHA，没有跨请求复用。
- 失败时核心/候选/细化/原生碰撞源版本与之前加速成功请求一致。优化入口仍冷构造solver/stateBank；没有读取旧qPreWeld/qPath；返回configuration与precision标志原本就不同。
- 全量参数见saved_result_audit.json。修复后与失败输入相比，除outputFile/sceneScript/代码版本provenance外，全部搜索、评分、预算、硬约束参数逐字段相同，仍seed=20260919。

## 4. 冻结输入重复实验

缓存开5次（秒）：103.267 / 104.501 / 107.493 / 108.141 / 102.062；5/5同一PRECISION_FAILURE。

缓存关5次（秒）：133.167 / 135.641 / 140.419 / 132.498 / 133.550；5/5同一PRECISION_FAILURE。

冻结的是失败场景、Home、seed和参数，场景/IK求解器/stateBank每次新建。缓存关采用独立诊断类副本，仅Enabled=false；保留与缓存无关的限位读取上提，从而隔离memoization效应。没有修改生产缓存类。10次稀疏q、失败细化诊断q、目标TCP矩阵逐值相同；无近似键/跨请求结论。均为45诊断点、0公开合法输出点。

另外使用性能优化前归档的core与refinement重跑同一失败输入：316.811s，同样24稀疏点、同样边19/20失败、同样0.656519mm，q数组逐值一致。这证明漏洞先于性能优化，不是新缓存/限位读取上提制造的回归。

## 5. 最小修复

1. `UR5/matlab仿真/replan_weld_actual_axis_adaptive_core.m:1771`：Bridge端点池加入原硬门复核。
2. 同文件`:1966`新localBridgeEndpointHardValid：使用相同FK、原关节边界、位姿误差、物理焊枪轴和焊接侧不等式；不重新IK、不编辑q、不增加方向/碰撞容差。此例14个去重端点中9个被拒绝，保留合法分支继续整条焊缝。
3. `hmi_accp_preview_status.m`：读取真实阶段证据，区分IK_FAILURE、NO_CONNECTED_CANDIDATE、COLLISION_FAILURE、PRECISION_FAILURE、OUTPUT_SCHEMA_FAILURE；构型连通不等于几何完整。
4. `plan_hmi_preview_request.m:92`：采用上述阶段信息及现有全部验证；已恢复的continuation事件不再覆盖实际终态失败。未将部分成功标为完整PASS。

refine_accp_cartesian_path、generate_actual_axis_ik_candidates、HmiExactSceneQueries、accp_score_candidates四项生产源码SHA未改。CBi-RRT*、碰撞过滤/阈值、IK数学、TCP/枪丝工件地面/URDF以及ROS2链路不改。旧失败数据、原冻结数据保留；无旧非法PreWeld恢复。修改前两文件在before/可回滚。

## 6. 修复后完整预览回归

通过正式plan_hmi_preview_request入口，同一失败请求连续5次：

| 次数 | 完整入口耗时(s) | 预览 | 焊接/任务节点 | 碰撞采样/命中 |
|---|---|---|---|---|
| 1 | 212.907 | PASS | 47 / 414 | 9087 / 0 |
| 2 | 212.382 | PASS | 47 / 414 | 9087 / 0 |
| 3 | 203.566 | PASS | 47 / 414 | 9087 / 0 |
| 4 | 197.336 | PASS | 47 / 414 | 9087 / 0 |
| 5 | 196.790 | PASS | 47 / 414 | 9087 / 0 |

平均 **204.596 ± 7.817s**（样本标准差，n=5）。每次189接近+47焊接+178退出=414点；START/GOAL PASS、所有节点限位PASS、完整原生碰撞审计9087采样零碰撞。47个焊接q/TCP目标五次逐值一致，最大TCP路径误差0.244252901mm <0.5mm，严格轴门通过。

修复后关闭精确缓存再冷规划1次：87.126s（仅ACCP，非完整任务），PASS且47个q/TCP与完整预览逐值相同。

旧成功请求再跑：173.499s，416任务点、9131采样PASS；原来的47个焊接q和TCP目标逐值未变。此为单次回归，不外推总体成功率。

真实HMI按钮再验证：MATLAB **195.674s**，按钮到Preview Ready **200.527s**；414点自动加载，实时关节状态和生产配置未改，执行门BLOCKED。请求仍是seed20260919及REALHOME-00fce…，2026-10-17检查当前模型与Home均未变化。证据hmi_button_e2e.json及其report路径。ROS工作线程禁用，未模拟MATLAB、规划或PASS结果。

### 耗时应怎样比较

原失败请求102.352s、原ACCP93.676s是提前失败，不是成功全任务；不能宣称205s成功任务比102s“加速”。原始性能结果189.660s属于旧Home/seed17；新输入5次均值204.596s不应伪称同一基准。旧成功请求本次173.499s，且原47点不变，说明没有恢复596s旧工作量。修复后的ACCP构型+细化均值60.173s；一些运行与诊断并行，耗时存在负载差异。

## 7. 安全与结果边界

这些PASS只针对已复验的冻结场景/Home/seed及旧成功请求，不保证任意新场景/随机种子必然有解。五次预览仍明确推迟时间参数化/控制器等价执行验收到第二阶段。

全部executionReady=false；无trajectory_execute_ready许可文件，无正式anchor覆盖，无机器人运动。原wire_collision_shorten_mm=3近似和小模型间隙仍需独立物理安全验证；有限采样PASS不是数学连续安全证明。不能把Preview Ready当作真机执行许可。

主证据：result_summary.json、final_numeric_regression.json、frozen_cache_comparison.json、saved_failure_audit/saved_result_audit.json、saved_graph_replay.json、failed_layer_statistics.csv、stage_status_regression.json；所有MAT与完整日志保留在对应子目录。
