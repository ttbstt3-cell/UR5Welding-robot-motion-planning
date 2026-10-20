function comparison=compare_hmi_preview_performance(originalDir,baselineDir,optimizedRoot,outputDir)
% Compare geometry/limits/version invariants and unprofiled timings offline.
if ~isfolder(outputDir), mkdir(outputDir); end
base=load(fullfile(originalDir,'preview_accp.mat'),'result'); base=base.result;
originalReport=jsondecode(fileread(fullfile(originalDir,'report.json')));
freshReport=jsondecode(fileread(fullfile(baselineDir,'report.json')));
summary=jsondecode(fileread(fullfile(optimizedRoot,'benchmark_summary.json')));
if iscell(summary.runs), runs=summary.runs; else, runs=num2cell(summary.runs); end
actualHmiFile=fullfile(outputDir,'hmi_button_e2e.json');
if isfile(actualHmiFile)
    actualHmi=jsondecode(fileread(actualHmiFile));
    actualReport=jsondecode(fileread(actualHmi.report));
    runs{end+1}=struct('run',fileparts(actualHmi.report),...
        'protected_dependencies',actualReport.final_scene_dependency_validation);
end
originalRaw=load(fullfile(originalDir,'full_raw_trajectory.mat'));
comparison=struct('scope','Same REAL scene/seed; cold ACCP; no execution trajectory',...
    'original_actual_hmi_seconds',originalReport.planning_time,...
    'baseline_seconds',[originalReport.planning_time freshReport.planning_time],...
    'optimized_seconds',zeros(1,numel(runs)),'runs',{{}},...
    'executionReady',false,'robot_motion','NO');
for k=1:numel(runs)
    p=fullfile(runs{k}.run,'report.json'); report=jsondecode(fileread(p));
    d=load(fullfile(runs{k}.run,'preview_accp.mat'),'result'); weld=d.result;
    raw=load(report.preview_file); ss=load(raw.sceneFile,'scene'); scene=ss.scene;
    qDelta=max(abs(base.qPath-weld.qPath),[],'all');
    poseDelta=0;
    for n=1:numel(base.tcpPosePath)
        poseDelta=max(poseDelta,max(abs(base.tcpPosePath{n}-weld.tcpPosePath{n}),[],'all'));
    end
    stableQ=isequaln(base.qPath,weld.qPath);
    stablePose=isequaln(base.tcpPosePath,weld.tcpPosePath);
    protectedOptions={'maxJointStepDeg','edgeResolutionDeg','positionToleranceM',...
        'orientationToleranceDeg','sideToleranceDeg','sideMarginM','axis_angle_tolerance_deg',...
        'wire_collision_shorten_mm','maxSegmentTcpErrorMm','errorSamplesPerEdge',...
        'cartesianBridgeMaxEdgeSamples','candidate_score_weights','numPoints','beamWidth','randomSeed'};
    for name=protectedOptions
        assert(isequaln(base.options.(name{1}),weld.options.(name{1})),...
            'PERF:ConstraintMutation','Changed protected option: %s',name{1});
    end
    bindingNames={'tcp_calibration_version','urdf_version','gun_geometry_version',...
        'wire_model_version','workpiece_pose_version','collision_scene_version','realhome_version'};
    for name=bindingNames
        assert(isequaln(originalRaw.metadata.calibration_binding.(name{1}),...
            raw.metadata.calibration_binding.(name{1})),'PERF:Stale','Binding differs: %s',name{1});
    end
    lim=scene.jointLimits;
    limitsPass=all(raw.qPath>=lim(:,1).' & raw.qPath<=lim(:,2).','all');
    maxJump=rad2deg(max(abs(diff(raw.qPath)),[],'all'));
    assert(stableQ && stablePose && qDelta==0 && poseDelta==0,'PERF:WeldMutation','Weld q/TCP nodes changed');
    assert(report.previewReady && report.collision_check_pass && limitsPass);
    assert(report.executionReady==false && raw.metadata.executionReady==false);
    assert(report.accp_performance.adaptive_time_scaling_sec==0 && weld.timeParameterizationDeferred);
    assert(weld.tcpMetrics.maxSegmentPositionErrorMm<0.5 && weld.nodeCollisionFree && weld.edgeCollisionFree);
    comparison.optimized_seconds(k)=report.planning_time;
    comparison.runs{end+1}=struct('directory',runs{k}.run,'pass',true,...
        'weld_q_nodes_identical',stableQ,'weld_tcp_targets_identical',stablePose,...
        'max_q_node_delta_rad',qDelta,'max_target_pose_matrix_delta',poseDelta,...
        'weld_node_count',size(weld.qPath,1),'total_node_count',size(raw.qPath,1),...
        'weld_max_tcp_geometry_error_mm',weld.tcpMetrics.maxSegmentPositionErrorMm,...
        'weld_max_node_tcp_error_mm',weld.tcpMetrics.maxNodePositionErrorMm,...
        'weld_max_geometry_orientation_error_deg',weld.tcpMetrics.maxSegmentOrientationErrorDeg,...
        'max_raw_joint_step_deg',maxJump,'joint_limits_pass',limitsPass,...
        'collision_pass',report.collision_check_pass,'collision_samples',report.collision_sample_count,...
        'source_dependencies_stable',runs{k}.protected_dependencies.pass,...
        'timing_deferred',weld.timeParameterizationDeferred,'bindings_identical',true,...
        'hard_constraint_options_identical',true,'executionReady',false);
end
comparison.baseline_mean_sec=mean(comparison.baseline_seconds);
comparison.baseline_std_sec=std(comparison.baseline_seconds);
comparison.optimized_mean_sec=mean(comparison.optimized_seconds);
comparison.optimized_std_sec=std(comparison.optimized_seconds);
comparison.reduction_percent=100*(1-comparison.optimized_mean_sec/comparison.baseline_mean_sec);
comparison.baseline_success_rate=double(originalReport.previewReady+freshReport.previewReady)/2;
comparison.optimized_success_rate=mean(cellfun(@(r)r.pass,comparison.runs));
comparison.pass=all(cellfun(@(r)r.pass,comparison.runs));
comparison.sample_size_note='Baseline includes supplied actual-HMI run plus one fresh unprofiled run; optimized includes two benchmarks and actual QPushButton/QProcess run when available. Profiled run excluded. Same seed only; not multi-seed statistical evidence.';
writeJson(fullfile(outputDir,'comparison.json'),comparison);
end
function writeJson(file,value)
fid=fopen(file,'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(value,'PrettyPrint',true),'char');
end
