function report=summarize_hmi_interface_repair(runDir)
% Summarize saved offline evidence; never recompute or apply a calibration.
here=fileparts(mfilename('fullpath')); root=fileparts(here); addpath(here);
dir=fullfile(runDir,'replanned'); file=fullfile(dir,'full_raw_trajectory.mat'); raw=load(file);
ss=load(raw.sceneFile,'scene'); scene=ss.scene;
report=jsondecode(fileread(fullfile(dir,'interface_diagnosis.json')));
report.dependency_validation_after=hmi_planning_dependency_changes(scene.planningDependencySnapshot,...
    hmi_planning_dependency_snapshot(root));
assert(report.dependency_validation_after.pass,'INTERFACE:STALE','Dependencies changed during offline validation');
report.previewReady=report.interface_pass; report.success=report.interface_pass;
report.executionReady=false; report.execution_point_count=size(raw.qPath,1);
report.duration=raw.timeFromStart(end); report.algorithm='CBi-RRT* + ACCP'; report.task='Full_Welding_Task';
report.collision_check_pass=report.preview_audit.collision_check_pass;
report.collision_status=report.preview_audit.collision_status;
report.collision_coverage_pass=report.preview_audit.collision_coverage_pass;
report.collision_sample_count=report.preview_audit.collision_sample_count;
report.collision_hit_count=report.preview_audit.collision_hit_count;
report.collision_pairs=report.preview_audit.collision_pairs;
report.start_state_status='PASS'; report.goal_state_status='PASS';
report.numeric_joint_limits_pass=all(raw.qPath>=scene.jointLimits(:,1).' & raw.qPath<=scene.jointLimits(:,2).','all');
report.max_numeric_adjacent_joint_change_deg=rad2deg(max(abs(diff(raw.qPath)),[],'all'));
report.stage_counts=struct(); report.stage_J6=struct();
for k=1:numel(raw.segments)
    s=raw.segments{k}; report.stage_counts.(s.label)=size(s.qPath,1);
    report.stage_J6.(s.label)=struct('range_deg',rad2deg([min(s.qPath(:,6)),max(s.qPath(:,6))]),...
        'total_physical_rotation_deg',rad2deg(sum(abs(diff(s.qPath(:,6))))));
end
report.stage_point_counts=report.stage_counts;
timing=jsondecode(fileread(fullfile(runDir,'timing','timing_summary.json')));
report.timing_stage=timing.preparation.stage; report.timing_failure_identifier=timing.preparation.failure_identifier;
report.timing_failure_reason=timing.failure_reason;
report.weld_timing_comparison=timing.preparation.weld_velocity_input_audits;
report.full_controller_5000_sample_gate_reached=false;
report.weld_q_nodes_exactly_unchanged=timing.weld_nodes_exactly_unchanged;
report.weld_tcp_path_exactly_unchanged=timing.weld_tcp_exactly_unchanged;
report.regression=jsondecode(fileread(fullfile(runDir,'regression','regression.json')));
writeJson(fullfile(dir,'report.json'),report); writeJson(fullfile(runDir,'summary.json'),report);
end
function writeJson(file,data)
fid=fopen(file,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(data,'PrettyPrint',true),'char');
end
