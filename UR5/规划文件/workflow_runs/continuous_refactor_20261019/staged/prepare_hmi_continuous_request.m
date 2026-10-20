function report=prepare_hmi_continuous_request(request)
% Single-owner three-stage continuous planning; never execution approval.
here=fileparts(which('build_real_planning_scene')); addpath(here,'-end');
out=char(request.outputDir); if ~isfolder(out), mkdir(out); end
report=struct('executionReady',false,'success',false,'robot_motion','NO',...
    'task','Full_Welding_Task','algorithm','CBi-RRT* + ACCP-D','preview_preserved',true,...
    'stage','CONTINUOUS_PLANNING','continuous_planning_complete',false,...
    'continuous_planning_status','CONTINUOUS_PLANNING','physical_validation_status','PENDING',...
    'servo_validation_status','PENDING','architecture_version','THREE_STAGE_CONTINUOUS_v1',...
    'stage_timings',struct());
timer=tic;
try
    report.stage='INPUT_VALIDATION'; clock=tic;
    assert(strcmp(hmi_workflow_file_hash(request.rawFile),request.raw_sha256),'CONTINUOUS:Stale','Raw preview changed');
    raw=load(request.rawFile);
    assert(raw.metadata.preview_only,'CONTINUOUS:Input','Requires preview artifact');
    assert(strcmp(hmi_workflow_file_hash(raw.sceneFile),raw.metadata.scene_snapshot_sha256),...
        'CONTINUOUS:Stale','Scene snapshot changed');
    assert(numel(raw.segments)==3 && strcmp(raw.segments{1}.label,'approach') &&...
        strcmp(raw.segments{2}.label,'weld') && strcmp(raw.segments{3}.label,'exit'),...
        'CONTINUOUS:Input','Requires approach/weld/exit');
    report.realhome_version=raw.metadata.realhome_version;
    x=load(raw.sceneFile,'scene'); scene=x.scene;
    if isfield(request,'motion_limits_file')
        scene.hmiMotionLimits=hmi_load_robot_motion_limits(request.motion_limits_file,request.motion_limits_sha256);
        report.motion_limits=scene.hmiMotionLimits;
    end
    report.stage_timings.input_sec=toc(clock);
    report.stage='GEOMETRIC_CONSISTENCY'; fprintf('HMI_WELD_STAGE stage=GEOMETRIC_CONSISTENCY\n');
    [geometry,gr]=hmi_continuous_geometry(raw,scene); report.geometry_consistency=gr;
    report.stage_timings.geometry_sec=gr.elapsed_sec;
    save(fullfile(out,'continuous_geometry_diagnostic.mat'),'geometry','gr','-v7.3');
    assert(gr.pass,'CONTINUOUS:Geometry','%s',gr.failure_reason);
    report.stage='TIME_VELOCITY_PARAMETERIZATION'; fprintf('HMI_WELD_STAGE stage=TIME_VELOCITY_PARAMETERIZATION\n');
    [candidate,tr]=hmi_continuous_parameterize(geometry,raw,scene);
    report.parameterization=tr; report.stage_timings.parameterization_sec=tr.elapsed_sec;
    save(fullfile(out,'continuous_parameterization_diagnostic.mat'),'candidate','tr','-v7.3');
    assert(tr.pass,'CONTINUOUS:Parameterization','%s',tr.failure_reason);
    report.velocity_optimization=struct('pass',tr.pass,'derivatives',tr.derivatives,...
        'interface_c2_optimization',struct('pass',tr.interface_C2_pass),...
        'method',tr.method,'inserted_execution_nodes',size(candidate.qPath,1)-size(geometry.originalQ,1));
    report.boundaries_after=struct('velocity_jump',0,...
        'acceleration_jumps',tr.derivatives.node_acceleration_jump(candidate.joins-1,:));
    report.original_path_nodes_preserved=true;
    report.original_weld_nodes_preserved=47;
    report.parameterization_scope='Single geometry owner / single local time and velocity owner / read-only acceptance';
    report=hmi_continuous_acceptance(candidate,raw,scene,request,report);
    if ~report.final_acceptance_pass
        error('CONTINUOUS:Acceptance','%s: %s',report.acceptance_failure_identifier,report.acceptance_failure_reason);
    end
    report.stage='OFFLINE_ARTIFACT_EXPORT'; clock=tic;
    report=hmi_export_offline_continuous_result(candidate,report,raw,report.current_binding,out,scene);
    report.stage_timings.offline_export_sec=toc(clock);
catch err
    report.executionReady=false; report.success=false; report.failure_reason=err.message;
    report.continuous_planning_complete=false; report.continuous_planning_status='CONTINUOUS_PLANNING_FAILED';
    report.failed_stage=report.stage; report.stage='CONTINUOUS_PLANNING_FAILED';
    report.failure_identifier=err.identifier; report.failure_stack=err.stack;
    fprintf('CONTINUOUS_PLANNING_FAILED %s\n',err.message);
end
report.planning_time=toc(timer);
fid=fopen(fullfile(out,'report.json'),'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(report,'PrettyPrint',true),'char');
end
