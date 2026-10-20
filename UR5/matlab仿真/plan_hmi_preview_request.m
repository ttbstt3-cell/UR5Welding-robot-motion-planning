function report=plan_hmi_preview_request(request)
% Preview orchestration only: no official anchors or execution preflight.
here=fileparts(mfilename('fullpath')); root=fileparts(here);
addpath(genpath(fullfile(fileparts(root),'RRT'))); addpath(here); setenv('RRT_HEADLESS','1');
out=char(request.outputDir); if ~isfolder(out), mkdir(out); end
report=struct('task',request.task,'algorithm','CBi-RRT* + ACCP-D',...
    'previewReady',false,'executionReady',false,'success',false,'robot_motion','NO',...
    'start_state_status','NOT_CHECKED','goal_state_status','NOT_GENERATED',...
    'rrt_entered',false,'seam_candidate_reused',false,'endpoint_validation',struct());
report.seed=request.seed;
freeAlgorithm=char(request.algorithm);
if contains(freeAlgorithm,'V1') || contains(freeAlgorithm,'FastConnect'), freeAlgorithm='CBi-RRT*'; end
if strcmp(request.task,'Weld_Seam'), report.algorithm='ACCP-D';
else, report.algorithm=[freeAlgorithm ' + ACCP-D']; end
timer=tic;
phaseTimes=struct(); phaseTimer=tic;
optimizePreview=~isfield(request,'performance_mode') || ~strcmp(request.performance_mode,'baseline');
report.preview_performance_optimized=optimizePreview;
try
    fprintf('HMI_WELD_STAGE stage=PREPARING\n');
    if strcmp(request.planning_scene,'REAL_CALIBRATION')
        dependencyBefore=hmi_planning_dependency_snapshot(root);
        scene=build_real_planning_scene(true);
        scene=apply_wire_collision_shorten_diagnostic(scene,3);
    else
        scene=loadUR5JointScene(fullfile(here,'sync_ur5_joint_states.m'));
    end
    binding=hmi_workflow_binding(scene,request.calibration_binding,root);
    if strcmp(request.planning_scene,'REAL_CALIBRATION')
        dependencyAfter=hmi_planning_dependency_snapshot(root);
        report.scene_dependency_validation=hmi_planning_dependency_changes(dependencyBefore,dependencyAfter);
        report.scene_dependency_snapshot=dependencyAfter;
        if ~report.scene_dependency_validation.pass
            report.status='STALE';
            error('HMI:SceneChangedDuringLoad','Scene dependency changed during loading: %s',...
                strjoin(report.scene_dependency_validation.changed_files,'; '));
        end
        scene.planningDependencySnapshot=dependencyAfter;
    end
    report.calibration_binding=binding;
    report.collision_scene_version=binding.collision_scene_version;
    if isfield(request,'realhome_version'), report.realhome_version=request.realhome_version; end
    report.binding_mismatches=hmi_request_binding_mismatches(request,binding);
    if ~isempty(report.binding_mismatches)
        report.status='STALE';
        error('PREVIEW:VersionMismatch','Request is STALE: %s',strjoin(report.binding_mismatches,'; '));
    end
    % The request is the authoritative start snapshot. Updating RealHome
    % must not leave the scene seed at a previously read YAML home value.
    scene.qMat0=reshape(double(request.qStart),1,6);
    phaseTimes.real_scene_build_and_binding_sec=toc(phaseTimer); phaseTimer=tic;
    sceneFile=fullfile(out,'raw_scene.mat'); save(sceneFile,'scene','-v7.3');
    script=fullfile(out,'preview_scene.m');
    fid=fopen(script,'w'); c=onCleanup(@()fclose(fid));
    fprintf(fid,"s=load('%s','scene'); scene=s.scene;\n",strrep(sceneFile,"'","''")); clear c;
    queryBinding=hmi_preview_query_binding(scene,request,binding,sceneFile);
    scene.previewExactQueryCache=optimizePreview;
    scene.previewFirstSolution=optimizePreview;
    scene.previewQueryBinding=queryBinding;
    phaseTimes.scene_snapshot_save_sec=toc(phaseTimer); phaseTimer=tic;
    fprintf('HMI_WELD_STAGE stage=VALIDATING_START\n');
    report.stage='VALIDATING_START';
    startCheck=validate_hmi_endpoint(scene,request.qStart,'START',binding);
    report.endpoint_validation.start=startCheck; report.start_state_status=startCheck.status;
    writeJson(fullfile(out,'endpoint_validation.json'),report.endpoint_validation);
    if ~startCheck.valid
        error('PREVIEW:InvalidStartState','Invalid Start State: %s %s',...
            startCheck.failure_reason,strjoin(startCheck.collision_pairs,'; '));
    end
    phaseTimes.start_endpoint_validation_sec=toc(phaseTimer); phaseTimer=tic;
    % Cold-start current ACCP against the current complete scene. Historical
    % seam paths are never loaded, even if an old caller supplies a file.
    report.legacy_seam_candidate_ignored=isfield(request,'seamCandidateFile');
    report.preweld_generation_source='CURRENT_SCENE_COLD_START_ACCP';
    if isfield(request,'preweld_version'), report.previous_preweld_version=request.preweld_version; end
    fprintf('HMI_WELD_STAGE stage=WELD\n');
    report.stage='GENERATING_PREWELD';
    recipe=fullfile(root,'规划文件','real_anchor_runs','accp_bridge_comparison_20261013_improved_full_cold_v2.mat');
    opts=struct('numPoints',16,'enableFallback',false);
    if isfile(recipe), old=load(recipe,'result'); opts=old.result.options; clear old; end
    opts.sceneScript=script; opts.outputFile=fullfile(out,'preview_accp.mat');
    opts.publish=false; opts.animate=false; opts.randomSeed=request.seed;
    opts.initialStateBank=[]; opts.initialStartLayer=1;
    opts.branchReferenceQ=reshape(request.qStart,1,6);
    opts.configurationOnlyDiagnostic=false; opts.disableAxisConstraint=false; opts.disable_axis_constraint=false;
    opts.deferTimeParameterization=optimizePreview;
    opts.performanceExactQueryCache=optimizePreview;
    opts.performanceBinding=queryBinding;
    report.accp_log_file=fullfile(out,'accp_cold_start.log');
    report.accp_report_file=opts.outputFile;
    weld=runCurrentAccp(opts,report.accp_log_file);
    report.accp_status=hmi_accp_preview_status(weld);
    if strcmp(report.accp_status.classification,'OUTPUT_SCHEMA_FAILURE')
        report.failure_class='OUTPUT_SCHEMA_FAILURE';
        error('PREVIEW:AccpOutputSchema','%s',report.accp_status.reason);
    end
    phaseTimes.accp_total_wall_sec=toc(phaseTimer); phaseTimer=tic;
    report.accp_performance=struct('configuration_sec',weld.configurationSearchTimeSec,...
        'cartesian_refinement_sec',weld.precisionAlgorithmTimeSec,...
        'adaptive_time_scaling_sec',weld.timeScalingAlgorithmTimeSec,...
        'configuration_ik_calls',weld.continuationStats.ikCalls,...
        'configuration_ik_elapsed_sec',weld.continuationStats.ikElapsedSec,...
        'refinement_candidate_generator_calls',weld.adaptiveIkCalls,...
        'refinement_candidate_generator_elapsed_sec',weld.adaptiveIkElapsedSec,...
        'refinement_collision_samples',weld.adaptiveCollisionChecks);
    if isfield(weld,'configurationPerformanceQueries')
        report.accp_performance.configuration_queries=weld.configurationPerformanceQueries;
    end
    if isfield(weld.cartesianRefinement,'performanceQueries')
        report.accp_performance.refinement_queries=weld.cartesianRefinement.performanceQueries;
    end
    if ~report.accp_status.geometry_complete
        report.accp_geometry_complete=false; report.accp_report_file=opts.outputFile;
        report.goal_state_status='NOT_GENERATED';
        report.failure_class=report.accp_status.classification;
        error('PREVIEW:NoSeamPath',...
            'Current-scene ACCP: %s; complete valid PreWeld/seam unavailable; see %s',...
            report.accp_status.reason,opts.outputFile);
    end
    % Timing/controller acceptance is deliberately deferred to preparation.
    qw=weld.qPath; weldSegment=makeSegment(scene,qw,'weld');
    if isfield(weld,'timeFromStart') && numel(weld.timeFromStart)==size(qw,1)
        weldSegment.timeFromStart=weld.timeFromStart(:);
        weldSegment.velocities=weld.velocities;
    end
    weldSegment.tcpPosePath=weld.tcpPosePath;
    weldSegment.source_binding=binding;
    weldSegment.time_parameterization_deferred=optimizePreview;
    weldSegment.timing_scope='DISPLAY_ONLY';
    weldSegment.accp_time_scaling_options=struct(...
        'maxVelocityRadPerSec',weld.options.timeScalingMaxVelocityRadPerSec,...
        'maxAccelerationRadPerSec2',weld.options.timeScalingMaxAccelerationRadPerSec2,...
        'maxDurationSec',weld.options.timeScalingMaxDurationSec,'lambda',weld.options.timeScalingLambda);
    weldSegment.geometry_validation=struct('complete',weld.adaptiveCorrectionComplete,...
        'node_collision_free',weld.nodeCollisionFree,'edge_collision_free',weld.edgeCollisionFree,...
        'collision_sample_count',weld.adaptiveCollisionChecks,...
        'max_tcp_error_mm',weld.tcpMetrics.maxSegmentPositionErrorMm);
    report.accp_geometry_complete=weld.adaptiveCorrectionComplete;
    report.accp_internal_timing_pass=weld.timeParameterizationComplete;
    report.accp_report_file=opts.outputFile;
    fprintf('HMI_WELD_STAGE stage=VALIDATING_GOAL\n');
    report.stage='VALIDATING_GOAL';
    goalCheck=validate_hmi_endpoint(scene,qw(1,:),'GOAL',binding);
    report.endpoint_validation.goal=goalCheck; report.goal_state_status=goalCheck.status;
    report.qPreWeld_rad=qw(1,:); report.qPreWeld_deg=rad2deg(qw(1,:));
    report.qPostWeld_rad=qw(end,:);
    writeJson(fullfile(out,'endpoint_validation.json'),report.endpoint_validation);
    if ~goalCheck.valid
        error('PREVIEW:InvalidGoalState','Invalid Goal State: %s %s',...
            goalCheck.failure_reason,strjoin(goalCheck.collision_pairs,'; '));
    end
    % preweld_version is the scene it was validated against, not a historical
    % MAT filename. Its separate record identity also binds current RealHome.
    report.preweld_version=binding.collision_scene_version;
    preweld=struct('q_rad',qw(1,:),'q_deg',rad2deg(qw(1,:)),...
        'TCP_pose_base',goalCheck.TCP_pose_base,'validation',goalCheck,...
        'preweld_version',binding.collision_scene_version,...
        'realhome_version',request.realhome_version,'calibration_binding',binding,...
        'source','CURRENT_SCENE_COLD_START_ACCP','executionReady',false);
    writeJson(fullfile(out,'preweld_current.json'),preweld);
    binding.realhome_version=request.realhome_version;
    binding.preweld_version=binding.collision_scene_version;
    weldSegment.source_binding=binding;
    segments={}; task=char(request.task);
    phaseTimes.weld_export_and_goal_validation_sec=toc(phaseTimer); phaseTimer=tic;
    if any(strcmp(task,{'RealHome_to_PreWeld','Full_Welding_Task'}))
        fprintf('HMI_WELD_STAGE stage=APPROACH\n');
        report.stage='APPROACH'; report.rrt_entered=true;
        [qa,approachPlanner]=freePath(scene,reshape(request.qStart,1,6),qw(1,:),request,out,'approach');
        phaseTimes.approach_planner_and_physical_audit_sec=toc(phaseTimer); phaseTimer=tic;
        report.approach_planner=plannerSummary(approachPlanner);
        segments{end+1}=makeSegment(scene,qa,'approach');
    end
    if any(strcmp(task,{'Weld_Seam','Full_Welding_Task'})), segments{end+1}=weldSegment; end
    if any(strcmp(task,{'PostWeld_to_RealHome','Full_Welding_Task'}))
        report.endpoint_validation.exit_start=validate_hmi_endpoint(scene,qw(end,:),'EXIT START',binding);
        report.endpoint_validation.exit_goal=validate_hmi_endpoint(scene,request.qHome,'EXIT GOAL',binding);
        writeJson(fullfile(out,'endpoint_validation.json'),report.endpoint_validation);
        if ~report.endpoint_validation.exit_start.valid
            error('PREVIEW:InvalidStartState','Invalid Exit Start State: %s',...
                strjoin(report.endpoint_validation.exit_start.collision_pairs,'; '));
        end
        if ~report.endpoint_validation.exit_goal.valid
            error('PREVIEW:InvalidGoalState','Invalid Exit Goal State: %s',...
                strjoin(report.endpoint_validation.exit_goal.collision_pairs,'; '));
        end
        fprintf('HMI_WELD_STAGE stage=EXIT\n');
        phaseTimes.segment_export_and_exit_endpoints_sec=toc(phaseTimer); phaseTimer=tic;
        report.stage='EXIT'; report.rrt_entered=true;
        [qe,exitPlanner]=freePath(scene,qw(end,:),reshape(request.qHome,1,6),request,out,'exit');
        phaseTimes.exit_planner_and_physical_audit_sec=toc(phaseTimer); phaseTimer=tic;
        report.exit_planner=plannerSummary(exitPlanner);
        segments{end+1}=makeSegment(scene,qe,'exit');
    end
    if isempty(segments), error('PREVIEW:Task','Unsupported task'); end
    for k=1:numel(segments)
        if ~isfield(segments{k},'source_binding'), segments{k}.source_binding=binding; end
    end
    rawFile=fullfile(out,'full_raw_trajectory.mat');
    raw=assemble_hmi_preview_trajectory(segments,binding,sceneFile,rawFile);
    phaseTimes.trajectory_assemble_and_save_sec=toc(phaseTimer); phaseTimer=tic;
    fprintf('HMI_WELD_STAGE stage=VALIDATING\n');
    % Display/FK/pair audit, not a new planning gate. Failed results remain
    % viewable but must never be described as collision-free Preview Ready.
    try
        displayAudit=audit_hmi_preview_display(rawFile,out,...
            struct('exactQueryCache',optimizePreview,'binding',queryBinding));
    catch auditError
        % Unsupported/legacy snapshot may still be inspected. Never turn a
        % failed diagnostic or missing geometry into a green collision PASS.
        fprintf('PREVIEW_DISPLAY_WARNING %s\n',auditError.message);
        displayAudit=struct('previewReady',false,'collision_check_pass',false,...
            'collision_status','UNKNOWN','collision_pairs',{{}},'collision_sample_count',0,...
            'collision_hit_count',0,'collision_coverage_pass',false,...
            'missing_robot_collision_bodies',{{}},'warning_reason',auditError.message,...
            'scope','Display audit unavailable; preview only');
        metadata=raw.metadata; metadata.collision_verified=false;
        metadata.collision_check_pass=false; metadata.collision_status='UNKNOWN';
        save(rawFile,'metadata','-append');
    end
    phaseTimes.full_preview_collision_audit_sec=toc(phaseTimer); phaseTimer=tic;
    saved=load(rawFile,'metadata'); metadata=saved.metadata;
    metadata.real_home_q=reshape(request.qHome,1,6); metadata.requested_start_q=reshape(request.qStart,1,6);
    metadata.realhome_version=request.realhome_version; metadata.preweld_version=binding.collision_scene_version;
    metadata.collision_scene_version=binding.collision_scene_version;
    metadata.endpoint_validation=report.endpoint_validation;
    metadata.algorithm=report.algorithm; metadata.seed=request.seed;
    save(rawFile,'metadata','-append');
    report.preview_available=true; report.previewReady=displayAudit.previewReady; report.success=true;
    report.collision_check_pass=displayAudit.collision_check_pass;
    report.collision_status=displayAudit.collision_status; report.collision_pairs=displayAudit.collision_pairs;
    report.collision_sample_count=displayAudit.collision_sample_count;
    report.collision_hit_count=displayAudit.collision_hit_count;
    report.collision_coverage_pass=displayAudit.collision_coverage_pass;
    report.missing_robot_collision_bodies=displayAudit.missing_robot_collision_bodies;
    if isfield(displayAudit,'warning_reason'), report.warning_reason=displayAudit.warning_reason; end
    report.collision_scope=displayAudit.scope;
    if isfield(displayAudit,'performance'), report.audit_performance=displayAudit.performance; end
    report.source_bindings_recorded=true;
    report.continuity_check_pass=false; % Kept for second-stage boundary validation.
    report.continuity_status='PENDING_EXECUTION_PREPARATION';
    report.seam_execution_point_count=size(qw,1);
    report.execution_point_count=size(raw.qPath,1); report.point_count=size(raw.qPath,1);
    report.tcp_point_count=size(raw.qPath,1); report.duration=raw.timeFromStart(end);
    report.calibration_binding=binding; report.planning_scene=request.planning_scene;
    report.preview_file=rawFile; report.scene_file=sceneFile;
    report.seam_planner='ACCP-D (candidate; execution preparation pending)';
    report.stage_point_counts=struct('approach',0,'weld',0,'exit',0,'total',size(raw.qPath,1));
    for k=1:numel(segments), report.stage_point_counts.(segments{k}.label)=size(segments{k}.qPath,1); end
    report.note='Preview clock only. No executionReady/official real anchor gate is used.';
    if strcmp(request.planning_scene,'REAL_CALIBRATION')
        finalDependencies=hmi_planning_dependency_snapshot(root);
        report.final_scene_dependency_validation=hmi_planning_dependency_changes(dependencyAfter,finalDependencies);
        if ~report.final_scene_dependency_validation.pass
            report.status='STALE'; report.previewReady=false;
            error('PREVIEW:SceneChangedDuringPlanning','Scene dependencies changed during planning: %s',...
                strjoin(report.final_scene_dependency_validation.changed_files,'; '));
        end
        for sourceIndex=1:numel(queryBinding.collision_rule_sources)
            source=queryBinding.collision_rule_sources{sourceIndex};
            if ~strcmp(source.sha256,hmi_workflow_file_hash(source.path))
                report.status='STALE'; report.previewReady=false; report.preview_available=false;
                error('PREVIEW:RuleChangedDuringPlanning','Collision-rule source changed during planning: %s',source.path);
            end
        end
    end
catch err
    report.success=false;
    report.previewReady=false;
    report.preview_available=false;
    report.failure_reason=err.message; report.failure_identifier=err.identifier;
    report.failure_stack=err.stack;
    if isfield(report,'stage'), report.failure_stage=report.stage; end
    if contains(err.message,'selected_fractions') || contains(err.message,'selected_level')
        report.failure_class='ACCP_CORE_SCHEMA_ERROR';
    end
    if isfield(report,'accp_log_file') && isfile(report.accp_log_file)
        text=fileread(report.accp_log_file);
        failed=regexp(text,'Continuation broke at point (\d+): ([A-Z_]+)','tokens');
        if ~isempty(failed)
            report.accp_continuation_event_point=str2double(failed{end}{1});
            report.accp_continuation_event_reason=failed{end}{2};
            recovered=isfield(report,'accp_status') && report.accp_status.configuration_complete;
            report.accp_search_failure_recovered=recovered;
            if ~recovered
                report.accp_failed_point=str2double(failed{end}{1});
                report.accp_search_failure_reason=failed{end}{2};
            end
        end
    end
    fprintf('PREVIEW_FAILED %s\n',getReport(err,'extended','hyperlinks','off'));
end
report.planning_time=toc(timer);
phaseTimes.final_metadata_and_report_sec=toc(phaseTimer);
report.performance=struct('schema_version','HMI_PREVIEW_PERFORMANCE_v1',...
    'stages',phaseTimes,'total_wall_sec',report.planning_time,...
    'phase_time_sum_sec',sum(struct2array(phaseTimes)),...
    'executionReady',false,'robot_motion','NO');
report.performance.call_counts=struct('scene_build',1,'accp',double(isfield(report,'accp_report_file')),...
    'approach',double(isfield(report,'approach_planner')),'exit',double(isfield(report,'exit_planner')),...
    'full_audit',double(isfield(report,'collision_sample_count')));
writeJson(fullfile(out,'report.json'),report);
end

function weld=runCurrentAccp(opts,logFile)
% Preserve diagnostics when the unchanged core fails before returning data.
previousFile=get(0,'DiaryFile'); previousState=get(0,'Diary');
fid=fopen(logFile,'w'); if fid>=0, fclose(fid); end
diary(logFile); cleanup=onCleanup(@()restoreDiary(previousFile,previousState)); %#ok<NASGU>
weld=replan_weld_actual_axis_adaptive_core(opts);
end
function restoreDiary(file,state)
diary('off'); set(0,'DiaryFile',file); if strcmp(state,'on'), diary('on'); end
end
function s=makeSegment(scene,Q,label)
[t,~]=parameterize_joint_path(Q,'MaxVelocityDegPerSec',5,'MaxAccelerationDegPerSec2',15);
P=cell(1,size(Q,1)); for k=1:numel(P), P{k}=getTransform(scene.robot,Q(k,:),scene.tcpBodyName); end
h=diff(t); V=[zeros(1,6);diff(Q)./h];
s=struct('label',label,'qPath',Q,'tcpPosePath',{P},'timeFromStart',t,'velocities',V);
end
function [Q,p]=freePath(scene,start,goal,r,out,label)
[Q,p]=hmi_plan_interface_free_path(scene,start,goal,r,out,label);
end
function summary=plannerSummary(p)
summary=struct('success',p.success,'termination_reason',p.terminationReason);
if isfield(p,'interfaceBranch'), summary.interface_branch=p.interfaceBranch; end
for key={'algorithm','planningTime','elapsedSec','stage1Iterations','stage2Iterations','collisionChecks',...
        'firstSolutionTimeSec','samples','collisionConfigurationChecks','edgeCollisionChecks',...
        'rewireAttempts','rewireSuccesses','nodeCount','interfaceTrials'}
    if isfield(p,key{1}), summary.(key{1})=p.(key{1}); end
end
end
function writeJson(file,data)
fid=fopen(file,'w'); c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(data,'PrettyPrint',true),'char');
end
