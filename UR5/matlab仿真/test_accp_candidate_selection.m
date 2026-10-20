function summary=test_accp_candidate_selection(outputDir,sceneName)
% Offline comparison only. No anchors, time-parameterized trajectory or ROS.
if nargin<2, sceneName='real'; end
here=fileparts(mfilename('fullpath'));
if strcmp(sceneName,'real'), sceneScript=fullfile(here,'sync_ur5_real_scene.m');
else, sceneScript=fullfile(here,'sync_ur5_joint_states.m'); end
if ~isfolder(outputDir), mkdir(outputDir); end
summary=cell(1,2);
for scoring=0:1
    runDir=fullfile(outputDir,sprintf('%s_scoring_%d',sceneName,scoring));
    if ~isfolder(runDir), mkdir(runDir); end
    opts=struct('sceneScript',sceneScript,'numPoints',16,'randomSeed',20260917, ...
        'beamWidth',1,'firstPointMaxStates',2,'maxPoseSeeds',3, ...
        'coarseTravelDeg',[-15 0 15],'numRandomSeedsFirstPoint',10, ...
        'candidate_diagnostics',true,'candidate_scoring_mode',logical(scoring), ...
        'candidate_score_weights',[1 1 0.1 0.1], ...
        'disableAxisConstraint',false,'enableFallback',false, ...
        'animate',false,'publish',false,'configurationOnlyDiagnostic',true, ...
        'outputFile',fullfile(runDir,'diagnostic_core.mat'));
    result=replan_weld_actual_axis_adaptive_core(opts);
    summary{scoring+1}=struct('scene',sceneName,'scoring',logical(scoring), ...
        'complete',result.configurationComplete,'accp_high_precision_complete',result.complete, ...
        'point_candidates',cellfun(@numel,result.stateBank), ...
        'diagnostics',result.candidateSelectionDiagnosticsFile);
    summary{scoring+1}.point1_filter_counts=rmfield(result.firstPointDiagnostics,'candidates');
    summary{scoring+1}.elapsed_sec=result.algorithmElapsedSec;
    summary{scoring+1}.max_joint_jump_deg=result.maxJointJumpDeg;
    summary{scoring+1}.node_collision_free=result.nodeCollisionFree;
    summary{scoring+1}.edge_collision_free=result.edgeCollisionFree;
    summary{scoring+1}.max_segment_tcp_error_mm=result.tcpMetrics.maxSegmentPositionErrorMm;
end
fid=fopen(fullfile(outputDir,[sceneName '_comparison.json']),'w');
if fid<0, error('Cannot write comparison'); end
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(summary,'PrettyPrint',true));
disp(summary);
end
