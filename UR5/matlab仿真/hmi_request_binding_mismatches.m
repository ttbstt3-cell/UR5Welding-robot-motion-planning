function reasons=hmi_request_binding_mismatches(request,current)
% Detect scene changes since the HMI took its request snapshot.
% qHome/PreWeld identity is separate from immutable collision-scene identity.
reasons={};
if isfield(request,'calibration_binding')
    expected=request.calibration_binding;
    keys={'tcp_calibration_sha256','real_gun_geometry_sha256','gun_mount_sha256',...
        'workpiece_sha256','urdf_sha256','scene_geometry_sha256',...
        'wire_model_version','collision_scene_version',...
        'robot_collision_geometry_sha256','robot_collision_coverage','environment_geometry_sha256',...
        'fixed_environment_contacts_sha256'};
    for k=1:numel(keys)
        key=keys{k};
        intentionalFrozenEmpty=isfield(current,'planning_scene') && ...
            strcmp(string(current.planning_scene),"FROZEN_SIMULATION") && ...
            any(strcmp(key,{'gun_mount_sha256','robot_collision_geometry_sha256','environment_geometry_sha256',...
            'fixed_environment_contacts_sha256'}));
        % The explicit support exception is optional. Removing it restores
        % strict native floor checks, rather than inventing a missing policy.
        intentionalFrozenEmpty=intentionalFrozenEmpty || ...
            (strcmp(key,'fixed_environment_contacts_sha256') && ...
            isfield(current,key) && isempty(current.(key)));
        if ~isfield(expected,key)
            reasons{end+1}=['Missing request binding: ' key]; %#ok<AGROW>
        elseif isempty(expected.(key))
            % Frozen display can intentionally have no real installation
            % record or loader report. Compare matching emptiness without
            % inventing a REAL dependency; endpoint coverage is still a
            % separate gate and its required status must remain present.
            if ~intentionalFrozenEmpty || ~isfield(current,key) || ~isempty(current.(key))
                reasons{end+1}=['Missing request binding: ' key]; %#ok<AGROW>
            end
        elseif ~isfield(current,key) || ~strcmp(string(expected.(key)),string(current.(key)))
            reasons{end+1}=['Version mismatch: ' key]; %#ok<AGROW>
        end
    end
else
    reasons{end+1}='Missing request calibration_binding';
end
if isfield(request,'collision_scene_version') && ...
        ~isempty(request.collision_scene_version) && ...
        ~strcmp(string(request.collision_scene_version),string(current.collision_scene_version))
    reasons{end+1}='Version mismatch: request.collision_scene_version';
end
if ~isfield(request,'realhome_version') || isempty(request.realhome_version)
    reasons{end+1}='Missing request realhome_version';
end
end
