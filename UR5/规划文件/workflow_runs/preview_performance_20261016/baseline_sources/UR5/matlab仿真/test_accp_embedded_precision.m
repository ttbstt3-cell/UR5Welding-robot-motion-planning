function report=test_accp_embedded_precision(baselineFile,outputDir)
% Cold production ACCP regression. No anchor generation/export/ROS motion.
S=load(baselineFile,'result');baseline=S.result;
assert(baseline.complete && size(baseline.qPath,1)==16);
assert(baseline.options.sideToleranceDeg==5 && ~baseline.options.disableAxisConstraint);
if ~isfolder(outputDir),mkdir(outputDir);end
o=baseline.options;o.initialPoints=16;o.animate=false;o.publish=false;
o.outputFile=fullfile(outputDir,'embedded_accp.mat');
o.roundOutputPrefix=fullfile(outputDir,'embedded_round');
r=replan_weld_actual_axis_adaptive_feasibility(o);
assert(r.complete && r.configurationComplete && r.adaptiveCorrectionComplete);
assert(r.precisionStageInvocationCount==1,'Precision must be invoked once inside core');
assert(size(r.qPathInitial,1)==16 && size(r.qPath,1)==31);
assert(isequal(r.qPath,r.qPathExecution));
assert(size(r.tcpPath,1)==size(r.qPath,1) && numel(r.tcpPosePath)==size(r.qPath,1));
assert(isequal(baseline.qPath,r.qPathInitial),'Configuration search changed');
assert(isequal(r.scene.calibrationBinding,baseline.scene.calibrationBinding));
previous=load(fullfile(fileparts(baselineFile),'refined_qPath.mat'),'qPath');
maxDifference=max(abs(r.qPath-previous.qPath),[],'all');
assert(maxDifference<1e-10,'Embedded precision changed the validated path');
assert(max(r.finalTcpMetrics.segmentPositionErrorMm)<0.5);
assert(r.nodeCollisionFree && r.edgeCollisionFree);
assert(r.timeParameterizationComplete && r.timeParameterization.executionReady);
assert(size(r.timeFromStart,1)==31 && size(r.velocities,1)==31 && size(r.accelerations,1)==31);
assert(r.timeParameterization.tcpContourMaxMm<0.5 && r.timeParameterization.timeSyncMaxMm<0.5);
assert(r.timeParameterization.collisionPass && r.timeParameterization.jointLimitsPass);
assert(abs(r.totalPlanningTimeSec-r.feasibilityAdaptiveAlgorithmTimeSec-r.precisionAlgorithmTimeSec-r.timeScalingAlgorithmTimeSec)<1e-8);
round=load([char(o.roundOutputPrefix) '_00.mat'],'result');
assert(round.result.complete && round.result.precisionStageInvocationCount==1);
assert(isequal(round.result.qPath,r.qPath),'Coordinator replaced the core output');
qPath=r.qPathInitial;tcpTarget=r.cartesianRefinement.tcpTargetOriginal;
save(fullfile(outputDir,'original_qPath.mat'),'qPath','tcpTarget');
qPath=r.qPath;tcpTarget=r.tcpTargetExecution;precision=r.cartesianRefinement;
save(fullfile(outputDir,'refined_qPath.mat'),'qPath','tcpTarget','precision','-v7.3');
% Configuration-only diagnostics cannot produce a formal ACCP success.
diag=o;diag.numPoints=16;diag.configurationOnlyDiagnostic=true;
diag.outputFile=fullfile(outputDir,'configuration_diagnostic.mat');
d=replan_weld_actual_axis_adaptive_core(diag);
assert(d.configurationComplete && ~d.complete && ~d.highPrecisionPathAvailable);
assert(isempty(d.qPathExecution) && d.precisionStageInvocationCount==0);
assert(isequal(d.qPathInitial,r.qPathInitial));
report=struct('algorithm','ACCP','complete',r.complete, ...
    'configuration_point_count',size(r.qPathInitial,1),'output_point_count',size(r.qPath,1), ...
    'precision_stage_invocations',r.precisionStageInvocationCount, ...
    'max_tcp_error_mm',max(r.finalTcpMetrics.segmentPositionErrorMm), ...
    'max_difference_from_previous_refined_q_rad',maxDifference, ...
    'collision_pass',r.nodeCollisionFree && r.edgeCollisionFree, ...
    'calibration_binding_identical',isequal(r.scene.calibrationBinding,baseline.scene.calibrationBinding), ...
    'configuration_search_identical',isequal(r.qPathInitial,baseline.qPath), ...
    'qPathExecution_alias_identical',isequal(r.qPath,r.qPathExecution), ...
    'tcp_path_rows_match',size(r.tcpPath,1)==size(r.qPath,1), ...
    'configuration_only_not_formal_pass',~d.complete, ...
    'search_time_sec',r.feasibilityAdaptiveAlgorithmTimeSec, ...
    'precision_time_sec',r.precisionAlgorithmTimeSec,'total_algorithm_time_sec',r.totalPlanningTimeSec, ...
    'robot_motion','NO');
fid=fopen(fullfile(outputDir,'report.json'),'w');assert(fid>=0);
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));disp(report);
end
