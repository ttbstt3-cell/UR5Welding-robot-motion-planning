function report=test_hmi_request_binding_semantics(outputDir)
% Metadata-only regression; no scene building, IK, planning, or robot IO.
keys={'tcp_calibration_sha256','real_gun_geometry_sha256','gun_mount_sha256',...
    'workpiece_sha256','urdf_sha256','scene_geometry_sha256','wire_model_version',...
    'collision_scene_version','robot_collision_geometry_sha256','robot_collision_coverage'};
current=struct('planning_scene','REAL_CALIBRATION');
for k=1:numel(keys), current.(keys{k})=['fixture-' keys{k}]; end
request=struct('realhome_version','DIAGNOSTIC_BINDING_FIXTURE',...
    'collision_scene_version',current.collision_scene_version,'calibration_binding',current);
checks=struct('name',{},'pass',{});
reasons=hmi_request_binding_mismatches(request,current);
assert(isempty(reasons)); remember('matching_real_nonempty_binding');
request.calibration_binding.gun_mount_sha256='DIFFERENT_REAL_MOUNT';
reasons=hmi_request_binding_mismatches(request,current);
assert(any(contains(string(reasons),'gun_mount_sha256'))); remember('real_mount_mismatch_rejected');
request.calibration_binding=current; current.gun_mount_sha256='';
request.calibration_binding.gun_mount_sha256='';
reasons=hmi_request_binding_mismatches(request,current);
assert(~isempty(reasons)); remember('real_empty_mount_not_accepted');
current.planning_scene='FROZEN_SIMULATION'; current.robot_collision_geometry_sha256='';
current.robot_collision_coverage='MISSING'; request.calibration_binding=current;
reasons=hmi_request_binding_mismatches(request,current);
assert(isempty(reasons)); remember('matching_intentional_frozen_empty_hashes');
request.calibration_binding.gun_mount_sha256='OLD_FROZEN_DEPENDENCY';
reasons=hmi_request_binding_mismatches(request,current);
assert(~isempty(reasons)); remember('nonempty_frozen_dependency_mismatch_rejected');
request.calibration_binding=rmfield(current,'gun_mount_sha256');
reasons=hmi_request_binding_mismatches(request,current);
assert(~isempty(reasons)); remember('absent_required_key_not_same_as_intentional_empty');
request.calibration_binding=current; request.calibration_binding.robot_collision_coverage='';
reasons=hmi_request_binding_mismatches(request,current);
assert(~isempty(reasons)); remember('frozen_coverage_status_still_required');
request.calibration_binding=current; request.calibration_binding.robot_collision_coverage='COMPLETE';
reasons=hmi_request_binding_mismatches(request,current);
assert(~isempty(reasons)); remember('frozen_coverage_mismatch_rejected');
report=struct('pass',true,'checks',checks,'robot_motion','NO','executionReady',false,...
    'scope','Metadata-only fixtures; matching FROZEN emptiness does not certify collision coverage');
if nargin>=1 && ~isempty(outputDir)
    if ~isfolder(outputDir), mkdir(outputDir); end
    fid=fopen(fullfile(outputDir,'binding_semantics_report.json'),'w'); c=onCleanup(@()fclose(fid));
    fwrite(fid,jsonencode(report,'PrettyPrint',true),'char');
end
fprintf('HMI_BINDING_SEMANTICS_PASS %d checks\n',numel(checks));
    function remember(name)
        checks(end+1)=struct('name',name,'pass',true);
    end
end
