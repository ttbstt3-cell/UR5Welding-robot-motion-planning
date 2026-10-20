function summary=verify_accp_stability_regression(root)
% Read completed evidence only; never plan, export execution or alter models.
addpath('/home/ttbstt/jixiebi/UR5/matlab仿真');
addpath(genpath('/home/ttbstt/jixiebi/RRT'));
oldFile='/home/ttbstt/jixiebi/UR5/规划文件/workflow_runs/preview_20261016_144235_155027/preview_accp.mat';
a=load(oldFile,'result'); original=a.result;
a=load(fullfile(root,'preperformance_case','preview_accp.mat'),'result'); pre=a.result;
assert(isequal(pre.qPathInitial,original.qPathInitial));
assert(isequal(pre.cartesianRefinement.qPathExecution,original.cartesianRefinement.qPathExecution));
assert(~pre.adaptiveCorrectionComplete && ~original.adaptiveCorrectionComplete);
summary=struct('preperformance_defect_reproduced',true,...
    'preperformance_and_failed_numeric_paths_identical',true,'fixed_runs',{{}},...
    'executionReady',false,'robot_motion','NO');
base=[];
for k=1:5
    runDir=fullfile(root,'fixed_full_preview',sprintf('optimized_%02d',k));
    a=load(fullfile(runDir,'preview_accp.mat'),'result'); r=a.result;
    raw=load(fullfile(runDir,'full_raw_trajectory.mat')); report=jsondecode(fileread(fullfile(runDir,'report.json')));
    if isempty(base), base=r; end
    assert(r.configurationComplete && r.adaptiveCorrectionComplete && r.nodeCollisionFree && r.edgeCollisionFree);
    assert(size(r.qPath,1)==47 && isequal(r.qPath,base.qPath) && isequal(r.tcpPosePath,base.tcpPosePath));
    assert(isequal(raw.segments{2}.qPath,r.qPath));
    assert(report.success && report.previewReady && report.collision_check_pass && ~report.executionReady);
    assert(report.start_state_status=="PASS" && report.goal_state_status=="PASS");
    assert(all(r.finalTcpMetrics.nodeHardConstraintsPass) && all(r.finalTcpMetrics.segmentCollisionFree));
    assert(r.tcpMetrics.maxSegmentPositionErrorMm<r.options.maxSegmentTcpErrorMm);
    limits=deriveUR5JointLimits(r.scene.robot);
    assert(all(raw.qPath>=limits(:,1).' & raw.qPath<=limits(:,2).','all'));
    assert(~r.timeParameterizationComplete && ~r.executionReady && ~report.executionReady);
    assert(isempty(dir(fullfile(runDir,'trajectory_execute_ready.*'))));
    axis=reshape(r.scene.weldGunAxisWP,[],1); axis=axis/norm(axis);
    RWB=r.scene.T_MatBase_Workpiece(1:3,1:3).'; minDot=1;
    for j=1:size(r.qPath,1)
        T=getTransform(r.scene.robot,r.qPath(j,:),r.scene.tcpBodyName);
        g=RWB*T(1:3,1:3)*r.scene.wireAxisTcp(:); g=g/norm(g);
        minDot=min(minDot,dot(g,axis));
    end
    assert(minDot>=cosd(r.options.sideToleranceDeg));
    summary.fixed_runs{end+1}=struct('run',k,'planning_time_sec',report.planning_time,...
        'weld_nodes',size(r.qPath,1),'total_nodes',size(raw.qPath,1),...
        'collision_samples',report.collision_sample_count,'collision_pass',report.collision_check_pass,...
        'joint_limits_pass',true,'start_pass',true,'preweld_pass',true,...
        'same_weld_q_and_targets',true,'max_tcp_geometry_error_mm',r.tcpMetrics.maxSegmentPositionErrorMm,...
        'minimum_axis_dot',minDot,'required_axis_dot',cosd(r.options.sideToleranceDeg),...
        'geometry_complete',r.adaptiveCorrectionComplete,'time_stage','DEFERRED_TO_PREPARATION',...
        'executionReady',false); %#ok<AGROW>
end
a=load(fullfile(root,'fixed_cache_off','run_01','preview_accp.mat'),'result'); native=a.result;
assert(native.adaptiveCorrectionComplete && ~native.configurationPerformanceQueries.enabled);
assert(isequal(native.qPath,base.qPath) && isequal(native.tcpPosePath,base.tcpPosePath));
summary.fixed_native_cache_off_numeric_match=true;
oldGood='/home/ttbstt/jixiebi/UR5/规划文件/workflow_runs/preview_20261016_142704_413362/preview_accp.mat';
a=load(oldGood,'result'); prior=a.result;
a=load(fullfile(root,'previous_success_regression','optimized_01','preview_accp.mat'),'result'); new=a.result;
assert(isequal(prior.qPath,new.qPath) && isequal(prior.tcpPosePath,new.tcpPosePath));
summary.previous_success_47_nodes_and_tcp_unchanged=true;
% Compare immutable options against failure input, except output identity and
% refreshed provenance. Hard limits/thresholds/scoring/seed remain identical.
op=base.options; prev=original.options;
for f={'outputFile','sceneScript','performanceBinding'}, op=rmfield(op,f{1}); prev=rmfield(prev,f{1}); end
assert(isequal(op,prev)); summary.all_planning_parameters_unchanged=true;
f=fopen(fullfile(root,'final_numeric_regression.json'),'w'); c=onCleanup(@()fclose(f));
fwrite(f,jsonencode(summary,'PrettyPrint',true),'char');
fprintf('FINAL_NUMERIC_REGRESSION_PASS five full previews, native cache-off match, prior 47 nodes unchanged\n');
end
