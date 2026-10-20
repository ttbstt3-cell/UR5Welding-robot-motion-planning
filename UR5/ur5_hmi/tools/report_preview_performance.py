"""Summarize saved offline benchmark evidence; does not run a planner."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import statistics
import sys


def read(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def run(root: Path) -> None:
    comparison = read(root / "comparison.json")
    benchmark = read(root / "optimized" / "benchmark_summary.json")
    optimized = [r["report"] for r in benchmark["runs"]]
    baseline = read(root / "baseline_unprofiled" / "baseline_01" / "report.json")
    original = read(root.parent / "preview_20261016_124729_001415" / "report.json")
    profile = read(root / "baseline_profile" / "baseline_01" / "matlab_profiler_summary.json")
    deferred = read(root / "deferred_timing_regression.json")
    native = read(root / "native_reaudit_regression.json")
    cache = read(root / "exact_cache_regression.json")
    hmi = read(root / "hmi_button_e2e.json")
    if hmi:
        optimized.append(read(Path(hmi["report"])))
    old_times = [original["planning_time"], baseline["planning_time"]]
    new_times = [r["planning_time"] for r in optimized]
    avg = statistics.mean(new_times)
    mean_old = statistics.mean(old_times)
    def mean_stage(name):
        return statistics.mean(r["performance"]["stages"].get(name, 0) for r in optimized)
    def mean_accp(name):
        return statistics.mean(r["accp_performance"][name] for r in optimized)
    scene_counts = optimized[0]["stage_point_counts"]
    phase_rows = [
        ("REAL 场景与版本绑定", baseline["performance"]["stages"]["real_scene_build_and_binding_sec"], mean_stage("real_scene_build_and_binding_sec")),
        ("ACCP 构型搜索", baseline["accp_performance"]["configuration_sec"], mean_accp("configuration_sec")),
        ("ACCP Cartesian Refinement", baseline["accp_performance"]["cartesian_refinement_sec"], mean_accp("cartesian_refinement_sec")),
        ("ACCP Adaptive Time Scaling / 控制器验证", baseline["accp_performance"]["adaptive_time_scaling_sec"], mean_accp("adaptive_time_scaling_sec")),
        ("接近段搜索与物理分支审计", baseline["performance"]["stages"]["approach_planner_and_physical_audit_sec"], mean_stage("approach_planner_and_physical_audit_sec")),
        ("退出段搜索与物理分支审计", baseline["performance"]["stages"]["exit_planner_and_physical_audit_sec"], mean_stage("exit_planner_and_physical_audit_sec")),
        ("完整预览碰撞审计", baseline["performance"]["stages"]["full_preview_collision_audit_sec"], mean_stage("full_preview_collision_audit_sec")),
    ]
    protected_sources = ["RRT/checkEnvironmentCollision.m", "RRT/collisionFreeJointState.m", "RRT/collisionFreeJointEdge.m", "UR5/matlab仿真/check_scene_environment_collision.m", "UR5/matlab仿真/build_real_planning_scene.m", "UR5/matlab仿真/adaptive_time_scale_accp_output.m", "UR5/matlab仿真/validate_accp_controller_interpolation.m"]
    manifest = {r["path"]: r["sha256"] for r in read(root / "baseline_source_manifest.json")}
    project = Path(__file__).resolve().parents[3]
    source_checks = []
    for name in protected_sources:
        path = project / name
        source_checks.append({"path": str(path), "unchanged": hashlib.sha256(path.read_bytes()).hexdigest() == manifest[str(path)]})
    assert all(x["unchanged"] for x in source_checks)
    assert comparison["pass"] and native["pass"] and deferred["pass"] and cache["pass"]
    if hmi:
        assert hmi["pass"]
    evidence = {"baseline_sec": old_times, "optimized_sec": new_times,
                "baseline_mean_sec": mean_old, "baseline_std_sec": statistics.stdev(old_times),
                "optimized_mean_sec": avg, "optimized_std_sec": statistics.stdev(new_times),
                "reduction_percent": 100 * (1 - avg / mean_old),
                "speedup": mean_old / avg,
                "source_checks": source_checks, "comparison": comparison,
                "deferred_timing": deferred, "native_reaudit": native,
                "actual_hmi_button": hmi, "executionReady": False, "robot_motion": "NO"}
    (root / "performance_result.json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2), encoding="utf-8")
    stages = "\n".join(f"| {name} | {before:.3f} | {after:.3f} |" for name, before, after in phase_rows)
    runs_table = "\n".join(f"| 优化预览 {i+1} | {t:.3f} | {r['point_count']} | {r['collision_sample_count']} | PASS |" for i, (t, r) in enumerate(zip(new_times, optimized)))
    profiler_rows = sorted(profile, key=lambda r: r["inclusive_sec"], reverse=True)
    wanted = ["refine_accp_cartesian_path>hardState", "rigidBodyTree>rigidBodyTree.get.Bodies", "replan_weld_actual_axis_adaptive_core>localBuildGraph", "replan_weld_actual_axis_adaptive_core>localCartesianBridgeFallback", "inverseKinematics>inverseKinematics.solve", "validate_accp_controller_interpolation"]
    prof_table = "\n".join(f"| {r['name']} | {int(r['calls'])} | {r['inclusive_sec']:.3f} |" for r in profiler_rows if r["name"] in wanted)
    io_keys = ["scene_snapshot_save_sec", "trajectory_assemble_and_save_sec", "final_metadata_and_report_sec"]
    io_sec = sum(mean_stage(k) for k in io_keys)
    baseline_benchmark = read(root / "baseline_unprofiled" / "benchmark_summary.json")["runs"][0]
    rrt_search_sec = sum(a["planningTimeSec"] for attempts in baseline_benchmark["planner_attempts"].values() for a in attempts)
    branch_sec = (baseline["performance"]["stages"]["approach_planner_and_physical_audit_sec"]
                  + baseline["performance"]["stages"]["exit_planner_and_physical_audit_sec"] - rrt_search_sec)
    accp_algorithm_sec = sum(baseline["accp_performance"][name] for name in ("configuration_sec", "cartesian_refinement_sec", "adaptive_time_scaling_sec"))
    outside_sec = baseline["planning_time"] - accp_algorithm_sec
    accp_residual_sec = baseline["performance"]["stages"]["accp_total_wall_sec"] - accp_algorithm_sec
    other_sec = outside_sec - rrt_search_sec - branch_sec - baseline["performance"]["stages"]["full_preview_collision_audit_sec"] - baseline["performance"]["stages"]["real_scene_build_and_binding_sec"] - accp_residual_sec
    query_rows = []
    for key, title in (("configuration_queries", "构型搜索"), ("refinement_queries", "Cartesian Refinement")):
        q = optimized[0]["accp_performance"][key]
        for kind, name in (("environment", "环境碰撞"), ("self", "自碰撞"), ("fk", "TCP FK")):
            query_rows.append(f"| {title} / {name} | {q[kind+'_queries']} | {q[kind+'_native_queries']} | {q[kind+'_hits']} |")
    q = optimized[0]["audit_performance"]
    for kind, name in (("environment", "环境碰撞"), ("self", "自碰撞")):
        query_rows.append(f"| 完整预览 / {name} | {q[kind+'_queries']} | {q[kind+'_native_queries']} | {q[kind+'_hits']} |")
    queries_table = "\n".join(query_rows)
    pure_ik_sec = statistics.mean(r["accp_performance"]["configuration_queries"]["ik_native_sec"]
                                  + r["accp_performance"]["refinement_queries"]["ik_native_sec"]
                                  for r in optimized)
    old_attempts = [a for attempts in baseline_benchmark["planner_attempts"].values() for a in attempts]
    new_attempts = [a for attempts in benchmark["runs"][0]["planner_attempts"].values() for a in attempts]
    rrt_rows = "\n".join(
        f"| {title} | {sum(a[key] for a in old_attempts):.0f} | {sum(a[key] for a in new_attempts):.0f} |"
        for key, title in (("samples", "随机采样"), ("collisionConfigurationChecks", "原生状态检测"),
                           ("edgeCollisionChecks", "连接边检测"), ("rewireAttempts", "重连尝试"),
                           ("rewireSuccesses", "重连成功"), ("nodeCount", "各尝试树节点合计")))
    text = f"""# HMI 预览规划性能调查与优化报告

同一 REAL 场景、RealHome、TCP、焊枪、焊丝、工件、地面和随机种子 20260917。所有结果 executionReady=false，NO ROBOT MOTION。

## 结果

优化前均值 **{mean_old:.3f} ± {statistics.stdev(old_times):.3f} s**；优化后均值 **{avg:.3f} ± {statistics.stdev(new_times):.3f} s**，减少 **{100*(1-avg/mean_old):.2f}%**，加速 **{mean_old/avg:.2f} 倍**。± 为样本标准差。

基准包含用户原始实际 HMI 记录 {old_times[0]:.3f}s 和一次新跑的未剖析基准 {old_times[1]:.3f}s。优化有 {len(new_times)} 次运行；最后一轮通过实际 HMI 按钮→QProcess→MATLAB→HMI 回调运行，ROS 工作线程禁用。这里只验证同种子重复性，不外推为多种子总体成功率。

实际 HMI 按钮回归：MATLAB 规划 {hmi['matlab_planning_sec']:.3f}s，点击到显示 Preview Ready {hmi['button_to_preview_sec']:.3f}s，生产文件采用 {hmi['transfer']['file_load_and_scene_adoption_sec']:.3f}s；实时关节状态与原 YAML 均未改变。

| 运行 | 总耗时 s | 总节点 | 完整碰撞采样 | 预览 |
|---|---:|---:|---:|---|
| 用户原始 HMI | {old_times[0]:.3f} | {original['point_count']} | {original['collision_sample_count']} | PASS |
| 新跑旧流程 | {old_times[1]:.3f} | {baseline['point_count']} | {baseline['collision_sample_count']} | PASS |
{runs_table}

## 分阶段耗时

旧流程列为新跑未剖析基准，优化列为优化重复运行均值。父阶段与 IK/碰撞子耗时存在包含关系，不能把所有子耗时再次相加。

| 阶段 | 旧流程 s | 优化均值 s |
|---|---:|---:|
{stages}

每请求调用口径：REAL 场景建立 1 次、ACCP 搜索 1 次、Cartesian Refinement 1 次；原预览时间缩放/控制器验证各 1 次，优化预览均为 0 次（在第二阶段恢复）。CBi-RRT* 接近/退出由各 3 次变成各 1 次，完整审计与 HMI 采用各 1 次。主搜索表仍为 2426 IK；全部实际 IK 为 3384 次。

优化实测的纯 IK 调用区间平均约 {pure_ik_sec:.3f}s，与候选函数整段耗时区分；它包括全部 3384 次实际求解，不把碰撞/筛选包装时间都记作 IK。

第一轮优化的原生查询与缓存命中如下；这些是各阶段的子调用，不是额外阶段耗时：

| 阶段 / 查询 | 逻辑查询次数 | 原生计算次数 | 精确命中 |
|---|---:|---:|---:|
{queries_table}

CBi-RRT* 接近与退出合计的具体计数（新跑旧流程六次尝试，对比第一轮优化两次尝试）：

| 指标 | 旧流程 | 优化 |
|---|---:|---:|
{rrt_rows}

文件快照保存、轨迹组装保存、最终元数据处理合计约 {io_sec:.3f}s；ACCP 内场景快照载入、结果装配及保存的剩余时间单独由 ACCP wall 与算法时间之差保留。HMI 文件读取/场景采用约 0.04s（Qt 离线回调），不构成主要瓶颈。实际进程启动到界面就绪耗时见 hmi_button_e2e.json 的 button_to_preview_sec 与 transfer；不把 MATLAB 启动/退出算作算法耗时。

## 瓶颈排名与依据

1. Cartesian Refinement 的不变限位读取：3094 次 hardState 反复访问 rigidBodyTree.Bodies；全程剖析共记录约 81 万次刚体对象构造，主要来自这种重复读取。剖析中 hardState 266.159s / refinement 303.567s，约 88%。优化只把不变限位移到密集循环外；密度、轴约束、误差阈值保留。
2. CBi-RRT* anytime 与重试：原记录接近/退出各 3 次，每次约 30s，总搜索约 180.017s；首解仅约 0.64–1.40s。原报告选中 planner 的 30s 不是所有尝试之和。优化预览在首个经过实际数值边审计的合法路径处结束；先验证关节分支/真实限位/物理边，失败不接受，必要时保留有预算的 anytime 恢复。
3. 构型搜索重复密集边检查：图构建和 Bridge 都会再次查询已检查状态。保留全部节点/边采样，用精确缓存减少重复原生查询。
4. 预览提前执行时间缩放与控制器插值审计：原 ACCP 约 28s 为这部分；剖析中 47.140s，其中 controller validation 45.591s，fminsearch 仅 0.058s。它已延迟到第二阶段，未删除；几何细化仍在预览执行。
5. 完整模型碰撞审计仍是主要必要成本；优化只复用完全相同状态和向量化最近正距离提取。

剖析会放大耗时，整次剖析约 886s，不纳入优化前后均值。以下为剖析 inclusive 时间，用于定位调用来源，不能直接与非剖析时间相加：

| 函数 | 调用次数 | 剖析 inclusive s |
|---|---:|---:|
{prof_table}

原日志 2426 是主构型统计；完整剖析实际 IK solve 为 3384 次，包含评分/恢复/细化。优化中的 configuration_queries 与 refinement_queries 给出实际 IK、FK、环境碰撞、自碰撞次数、原生耗时及命中数，全部保留在各 report.json。

原 595.277s 的日志没有完整阶段计时，不能精确倒推每一项历史剩余耗时；可确认约 180.017s 来自六次 RRT 搜索，其余由场景、物理分支审计、完整碰撞审计和 I/O 构成。新跑基准的 performance.stages 给出不重叠分解与总和，可直接核对原约 288s 的算法外成本。

新跑未剖析基准中，ACCP 算法外的 {outside_sec:.3f}s 具体为：RRT 六次搜索 {rrt_search_sec:.3f}s；实际数值分支密集审计 {branch_sec:.3f}s；完整预览审计 {baseline['performance']['stages']['full_preview_collision_audit_sec']:.3f}s；场景/版本建立 {baseline['performance']['stages']['real_scene_build_and_binding_sec']:.3f}s；ACCP 快照载入/结果装配/保存剩余 {accp_residual_sec:.3f}s；端点、组装、文件与最终元数据等其余 {other_sec:.3f}s。以上不重复计算，总和与阶段账本一致。

## 几何、连续性、版本与失败状态

- 原 47 个焊接 q 节点与 TCP 目标逐值相同，最大差为 0；不是仅比较节点数量。
- TCP 最大几何误差保持 0.244253mm，最大节点误差约 0.000143mm；几何姿态误差约 0.018086°。
- 最大相邻数值关节步长约 7.119°；真实限位、自碰撞、焊枪、焊丝、工件、地面采样全部通过，接口没有取模或跳过整圈运动。
- 首解策略的路径为 {scene_counts['approach']} 接近 + 47 焊接 + {scene_counts['exit']} 退出 = {scene_counts['total']} 点；原路径为 165+47+147=359 点。它减少搜索优化时间，允许自由空间路径更长；不宣称路径最短或执行时间更短。
- 同一优化轨迹关闭缓存后重新审计 {native['samples']} 个相同采样点，逐帧状态、碰撞对、最近对象名称与距离一致；最大逐帧间隙差 {native['max_frame_gap_difference_mm']:.12g}mm。
- 本轮新增的场景查询缓存为单请求、单不可变模型的内存对象，使用六关节 IEEE754 完整 num2hex，不取模、不舍入、不邻近匹配。绑定 RealHome/qStart/qHome、TCP/URDF/枪/丝/工件/环境版本、快照 SHA、skip 表、限位和碰撞规则源 SHA。请求结束丢弃；没有跨场景、跨请求合法性缓存。
- 新请求、1e-12rad 差异不命中旧 q；缓存结果与原生环境/自碰撞矩阵、FK 完全相同。规划结束还重新核对场景与规则源；变更返回 STALE。
- 源码 SHA 证明原生碰撞后端、REAL 场景构建、时间缩放数学与控制器插值验证函数未改。TCP、URDF、焊枪/焊丝/工件/地面几何和控制链未写入。
- 同种子预览成功率为基准 2/2、优化 {len(optimized)}/{len(optimized)}；这里只说明该固定场景的重复性。

## 两阶段流程验证

预览保留 Continuous IK、Cartesian Bridge、Cartesian Refinement 和密集几何碰撞验证；输出显示专用时钟，并明确标注 time_parameterization_deferred。点击“轨迹连续规划”时 hmi_prepare_deferred_weld_timing 才调用原 Adaptive Time Scaling，再继续原速度/边界/控制器验证流程。

延迟阶段回归：q/TCP 不变，重新计算的时间戳及速度与原算法结果逐值相同；同一内存段二次进入不重复计算。原时间方案的 complete={deferred['initial_timing_acceptance']}，失败原因原样保留，没有把尚未通过的时间方案冒充可执行。

本轮未运行会发布/执行轨迹的 ROS 入口，未生成 trajectory_execute_ready 文件。所有预览、对比与诊断保持 executionReady=false。

## 修改文件

生产入口/工作流：plan_hmi_preview_request.m、prepare_hmi_continuous_request.m、gui/planning_workflow.py。
性能适配：replan_weld_actual_axis_adaptive_core.m、generate_actual_axis_ik_candidates.m、refine_accp_cartesian_path.m、HmiExactSceneQueries.m、hmi_preview_query_binding.m、hmi_prepare_deferred_weld_timing.m。
预览首解与审计：RRT/jointspace/FastConnectAnytimeRRTStarJoint.m、hmi_plan_interface_free_path.m、hmi_audit_joint_interface_path.m、audit_hmi_preview_display.m。
基准/回归：benchmark_hmi_preview_performance.m、compare_hmi_preview_performance.m、test_hmi_exact_scene_queries.m、test_hmi_deferred_preview_timing.m、verify_hmi_preview_native_reaudit.m、tools/test_preview_performance_hmi_load.py、tools/e2e_preview_performance.py、tools/report_preview_performance.py。README 与 AGENT_HANDOFF 同步记录。

## 证据与回滚

- comparison.json：逐值节点、目标、硬约束参数、版本与成功率验证。
- baseline_profile/baseline_01/matlab_profiler.mat 与 matlab_profiler_summary.json：完整调用剖析。
- baseline_unprofiled/baseline_01/：未剖析旧流程完整日志与阶段耗时。
- optimized/optimized_01、optimized_02/：两次优化完整日志、每次 planner 尝试、原生查询统计。
- native_reaudit/ 与 native_reaudit_regression.json：关闭缓存的独立完整复审。
- deferred_timing_regression.json：第二阶段时间重建与失败状态保留。
- exact_cache_regression.json、optimized/hmi_load_regression.json、hmi_button_e2e.json：缓存与实际 HMI 链路验证。
- baseline_sources/、baseline_source_manifest.json：优化前 MATLAB/RRT 源码副本；实验输出全部在独立目录。

可在离线请求设置 performance_mode='baseline' 回到原 preview anytime/提前时间计算流程作对照；默认 HMI 预览使用优化流程。此开关不改变碰撞或精度门限。
"""
    (root / "report.md").write_text(text, encoding="utf-8")
    print(json.dumps({k: evidence[k] for k in ["baseline_mean_sec", "optimized_mean_sec", "optimized_std_sec", "reduction_percent", "speedup"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    run(Path(sys.argv[1]))
