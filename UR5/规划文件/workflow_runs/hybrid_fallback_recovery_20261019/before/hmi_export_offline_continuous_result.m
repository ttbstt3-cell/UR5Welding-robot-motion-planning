function report=hmi_export_offline_continuous_result(candidate,report,raw,current,out,scene)
% Diagnostic artifacts only; no execution permit, ROS message or model edits.
a=report.controller_validation; vr=report.velocity_optimization; dm=vr.derivatives;
checks=struct('interface_c2',vr.interface_c2_optimization.pass,...
    'tcp_contour',a.max_tcp_contour_error_mm<a.acceptance.max_tcp_contour_error_mm,...
    'tcp_time_sync',a.max_tcp_time_sync_error_mm<a.acceptance.max_tcp_time_sync_error_mm,...
    'tcp_orientation',a.max_tcp_orientation_error_deg<=a.acceptance.max_tcp_orientation_error_deg,...
    'velocity',a.max_joint_velocity_rad_per_sec<=a.acceptance.max_joint_velocity_rad_per_sec,...
    'acceleration',a.max_joint_acceleration_rad_per_sec2<=a.acceptance.max_joint_acceleration_rad_per_sec2,...
    'controller_interpolation',a.complete && a.analytic_extrema_pass && a.sample_count>=5000 &&...
        strcmp(a.interpolation_mode,'positions_plus_velocities_cubic_hermite'),...
    'collision',a.collision_pass,'joint_limits',a.joint_limits_pass && a.analytic_extrema_pass,...
    'versions',report.version_validation.pass);
report.offline_checks=checks;
assert(all(structfun(@(v)islogical(v)&&isscalar(v)&&v,checks)),...
    'Offline planning evidence incomplete; no validated artifact exported');
report.continuity_scope=struct('interface_c2_pass',checks.interface_c2,...
    'interface_max_acceleration_jump_rad_s2',max(abs(report.boundaries_after.acceleration_jumps),[],'all'),...
    'all_nodes_c2_guaranteed',false,'all_nodes_max_acceleration_jump_rad_s2',dm.max_acceleration_jump,...
    'note','C2 enforced at task-segment joins only; other node acceleration jumps are reported, not hidden.');
report.offline_controller_validation_pass=true; report.continuous_planning_complete=true;
report.continuous_planning_status='PASS'; report.success=true;
report.stage='PHYSICAL_VALIDATION_PENDING'; report.executionReady=false;
report.physical_validation_status='PENDING'; report.servo_validation_status='PENDING';
report.collision_check_pass=true; report.collision_status='PASS'; report.continuity_check_pass=true;
report.execution_point_count=size(candidate.qPath,1); report.duration=candidate.timeFromStart(end);
report.max_tcp_error_mm=a.max_tcp_time_sync_error_mm;
report.failure_reason='';
report.note='Offline planning PASS; independent physical safety and servo validation PENDING. Diagnostic artifacts only; executionReady=false.';
report.pending_reasons={'Independent physical clearance / calibration uncertainty validation',...
    'Actual servo tracking / execution commissioning',...
    'Sampled collision PASS is not a mathematical continuous-safety proof; wire shorten=3mm remains an approximation'};
qPath=candidate.qPath; tcpPosePath=candidate.tcpPosePath; timeFromStart=candidate.timeFromStart;
velocities=candidate.velocities; segmentLabel=candidate.segmentLabel; joint_names=raw.joint_names;
tcpPath=cell2mat(cellfun(@(T)T(1:3,4).',tcpPosePath,'UniformOutput',false).');
metadata=raw.metadata;
if isfield(report,'motion_limits'), metadata.motion_limits=report.motion_limits; end
metadata.source='MATLAB_HMI_OFFLINE_CONTINUOUS_PLANNING';
metadata.trajectory_name='offline_optimized_trajectory';
metadata.preview_only=true; metadata.diagnostic_only=true; metadata.executionReady=false;
metadata.workflow_stage='PHYSICAL_VALIDATION_PENDING'; metadata.calibration_binding=current;
metadata.collision_verified=true; metadata.collision_check_pass=true;
metadata.controller_interpolation=a.interpolation_mode; metadata.controller_sample_count=a.sample_count;
metadata.physical_safety_margin_validated=false; metadata.servo_validated=false;
metadata.time_parameterization_deferred=false;
% New optimized node count/order needs matching display FK, not the raw
% preview's old frame arrays. All these exact nodes belong to the dense
% controller audit above (u=0/1); do not reuse old per-frame collision data.
info=metadata.preview_display;
assert(strcmp(hmi_workflow_file_hash(info.snapshot_file),info.snapshot_sha256),...
    'Preview display snapshot changed');
snapshot=jsondecode(fileread(info.snapshot_file));
Tbase=getTransform(scene.robot,zeros(1,6),'base');
n=size(qPath,1); flangePosePath=cell(1,n); gunPosePath=cell(1,n); wirePosePath=cell(1,n);
collisionResult=cell(1,n);
for k=1:n
    flangePosePath{k}=Tbase\getTransform(scene.robot,qPath(k,:),'flange');
    gunPosePath{k}=flangePosePath{k}*snapshot.T_flange_to_gun;
    wirePosePath{k}=flangePosePath{k}*snapshot.T_flange_to_wire;
    collisionResult{k}=struct('collision_status','PASS','collision_pairs',{{}},'outgoing_edge_pairs',{{}},...
        'source','controller_validation: all exact nodes and cubic edges sampled PASS',...
        'continuous_safety_proven',false);
end
report.artifacts=struct('trajectory_mat',fullfile(out,'offline_optimized_trajectory.mat'),...
    'trajectory_json',fullfile(out,'offline_optimized_trajectory.json'),...
    'tracking_curves_csv',fullfile(out,'offline_tracking_curves.csv'));
save(report.artifacts.trajectory_mat,'qPath','tcpPosePath','tcpPath','timeFromStart',...
    'velocities','segmentLabel','joint_names','metadata','flangePosePath','gunPosePath',...
    'wirePosePath','collisionResult','-v7');
record=struct('qPath',qPath,'tcpPath',tcpPath,'timeFromStart',timeFromStart,'velocities',velocities,...
    'tcpPosePath',{tcpPosePath},'segmentLabel',{segmentLabel},'joint_names',{joint_names},'metadata',metadata,...
    'flangePosePath',{flangePosePath},'gunPosePath',{gunPosePath},'wirePosePath',{wirePosePath},'collisionResult',{collisionResult});
writeJson(report.artifacts.trajectory_json,record);
% Display curves are independently sampled FK diagnostics, NOT a replacement
% for the already completed >=5000-sample full collision/analytic audit.
[~,s]=hmi_cubic_tracking_diagnostics(qPath,tcpPosePath,timeFromStart,velocities,scene,...
    max(13,ceil(5000/(size(qPath,1)-1))+1),segmentLabel);
curves=table(s.time,s.contourError,s.positionError,s.orientationError,...
    'VariableNames',{'time_sec','tcp_contour_mm','tcp_sync_mm','orientation_deg'});
writetable(curves,report.artifacts.tracking_curves_csv);
report.curve_scope='FK display curves; certification metrics use controller_validation, not these plot samples';
candidate.executionReady=false; candidate.diagnostic_only=true;
save(fullfile(out,'controller_continuity_diagnostic.mat'),'candidate','report','-v7.3');
fprintf('HMI_WELD_STAGE stage=PHYSICAL_VALIDATION_PENDING\n');
end
function writeJson(path,value)
fid=fopen(path,'w'); assert(fid>=0,'Cannot write offline diagnostic artifact');
cleanup=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(value,'PrettyPrint',true),'char');
end
