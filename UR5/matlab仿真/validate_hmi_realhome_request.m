function report=validate_hmi_realhome_request(request)
% Read-only validation invoked after saving RealHome. No motion or planning.
here=fileparts(mfilename('fullpath')); root=fileparts(here);
addpath(genpath(fullfile(fileparts(root),'RRT'))); addpath(here);
setenv('RRT_HEADLESS','1'); out=char(request.outputDir);
if ~isfolder(out), mkdir(out); end
report=struct('task','RealHome_Validation','status','MISSING_DATA','success',false,...
    'realhome_validation_status','MISSING_DATA','previewReady',false,'executionReady',false,...
    'robot_motion','NO','collision_pairs',{{}},'binding_valid',false);
timer=tic;
if isfield(request,'realhome_version'), report.realhome_version=request.realhome_version; end
if isfield(request,'collision_scene_version'), report.collision_scene_version=request.collision_scene_version; end
try
    dependencyBefore=hmi_planning_dependency_snapshot(root);
    if isfield(request,'realhome_config_path') && ~isempty(request.realhome_config_path)
        scene=build_real_planning_scene(true,request.realhome_config_path);
    else
        scene=build_real_planning_scene(true);
    end
    report.realhome_source_file=scene.realHomeSourceFile;
    report.realhome_file_q_rad=scene.realHomeSourceQ;
    if ~isfield(request,'qStart') || numel(request.qStart)~=6 || ...
            any(~isfinite(double(request.qStart)))
        report.status='MISSING_DATA'; report.realhome_validation_status='MISSING_DATA';
        error('REALHOME:MISSING_DATA','Request RealHome joint vector is missing or invalid');
    end
    report.request_q_rad=reshape(double(request.qStart),1,6);
    if max(abs(report.request_q_rad-scene.realHomeSourceQ))>1e-10
        report.status='VERSION_MISMATCH'; report.realhome_validation_status='VERSION_MISMATCH';
        report.failure_reason='Saved RealHome file and validation request contain different joint samples';
        report.binding_valid=false; report.success=false;
        error('REALHOME:VERSION_MISMATCH','Saved RealHome file does not match the request snapshot');
    end
    scene=apply_wire_collision_shorten_diagnostic(scene,3);
    binding=hmi_workflow_binding(scene,request.calibration_binding,root);
    dependencyAfter=hmi_planning_dependency_snapshot(root);
    report.scene_dependency_validation=hmi_planning_dependency_changes(dependencyBefore,dependencyAfter);
    report.scene_dependency_snapshot=dependencyAfter;
    if ~report.scene_dependency_validation.pass
        report.status='VERSION_MISMATCH'; report.realhome_validation_status='VERSION_MISMATCH';
        error('HMI:SceneChangedDuringLoad','Scene dependency changed during loading: %s',...
            strjoin(report.scene_dependency_validation.changed_files,'; '));
    end
    scene.planningDependencySnapshot=dependencyAfter;
    report.calibration_binding=binding;
    report.realhome_version=request.realhome_version;
    report.collision_scene_version=binding.collision_scene_version;
    q=request.qStart;
    endpoint=validate_hmi_endpoint(scene,q,'RealHome',binding);
    report.endpoint_validation=endpoint; report.q_rad=endpoint.q_rad; report.q_deg=endpoint.q_deg;
    report.collision_pairs=endpoint.collision_pairs;
    report.binding_mismatches=hmi_request_binding_mismatches(request,binding);
    report.binding_valid=isempty(report.binding_mismatches);
    report.success=endpoint.valid && report.binding_valid;
    if report.success
        report.status='VALIDATION_PASS'; report.realhome_validation_status='VALIDATION_PASS';
    elseif ~report.binding_valid
        report.status='VERSION_MISMATCH'; report.realhome_validation_status='VERSION_MISMATCH';
        report.failure_reason=strjoin(report.binding_mismatches,'; ');
    else
        report.status='COLLISION_FAIL'; report.realhome_validation_status='COLLISION_FAIL';
        report.failure_reason=['Invalid RealHome: ' endpoint.failure_reason];
    end
    save(fullfile(out,'validation_scene.mat'),'scene','-v7.3');
catch err
    if startsWith(err.identifier,'REAL_SCENE:HomeMissing') || startsWith(err.identifier,'REALHOME:MISSING_DATA')
        report.status='MISSING_DATA'; report.realhome_validation_status='MISSING_DATA';
    elseif startsWith(err.identifier,'REAL_SCENE:HomeInvalid') || startsWith(err.identifier,'REALHOME:VERSION_MISMATCH') || ...
            startsWith(err.identifier,'HMI:SceneChangedDuringLoad')
        report.status='VERSION_MISMATCH'; report.realhome_validation_status='VERSION_MISMATCH';
    elseif strcmp(report.status,'MISSING_DATA')
        % Keep the initial class only for failures that happened before a
        % scene could be built; all other scene/runtime failures are not
        % fabricated as PASS or missing data.
        report.status='COLLISION_FAIL'; report.realhome_validation_status='COLLISION_FAIL';
    end
    report.failure_reason=err.message; report.failure_identifier=err.identifier;
    report.failure_stack=err.stack;
    fprintf('REALHOME_VALIDATION_FAILED %s\n',getReport(err,'extended','hyperlinks','off'));
end
report.validation_time=toc(timer);
file=fullfile(out,'report.json'); fid=fopen(file,'w'); c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(report,'PrettyPrint',true),'char');
fprintf('RealHome Validation: %s\n',report.realhome_validation_status);
end
