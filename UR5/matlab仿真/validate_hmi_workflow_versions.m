function result=validate_hmi_workflow_versions(binding,current,segments,readinessReasons)
% Final preparation gate. Preview creation never calls this function.
keys={'tcp_calibration_sha256','urdf_sha256','real_gun_geometry_sha256','gun_mount_sha256',...
    'workpiece_sha256','tcp_calibration_version','gun_geometry_version','workpiece_pose_version',...
    'urdf_version','wire_model_version','collision_scene_version'};
mismatches={};
for k=1:numel(keys)
    key=keys{k};
    if ~isfield(binding,key) || isempty(binding.(key)) || ~isfield(current,key) ||...
            ~isequal(binding.(key),current.(key)), mismatches{end+1}=key; end %#ok<AGROW>
end
for s=1:numel(segments)
    if ~isfield(segments{s},'source_binding')
        mismatches{end+1}=[segments{s}.label ':missing source binding']; continue; %#ok<AGROW>
    end
    b=segments{s}.source_binding;
    for k=1:numel(keys)
        key=keys{k};
        if ~isfield(b,key) || ~isfield(current,key) || ~isequal(b.(key),current.(key))
            mismatches{end+1}=[segments{s}.label ':' key]; %#ok<AGROW>
        end
    end
end
result=struct('pass',isempty(mismatches) && isempty(readinessReasons),...
    'mismatches',{mismatches},'real_scene_readiness_reasons',{readinessReasons},'current_binding',current);
end
