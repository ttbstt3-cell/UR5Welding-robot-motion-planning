function report=test_hmi_realhome_flow(outputDir,runColdPlanning)
% Isolated offline regressions. Never save a diagnostic pose as real Home.
if nargin<1
    root=fileparts(fileparts(mfilename('fullpath')));
    outputDir=fullfile(root,'规划文件','workflow_runs','realhome_update_regression_20261014');
end
if nargin<2, runColdPlanning=true; end
here=fileparts(mfilename('fullpath')); root=fileparts(here);
addpath(here); addpath(genpath(fullfile(fileparts(root),'RRT')));
if ~isfolder(outputDir), mkdir(outputDir); end
protected={fullfile(root,'ur5_hmi','config','hmi_config.yaml'),...
    fullfile(root,'real_tcp_calibration_active.json'),...
    fullfile(root,'real_gun_geometry','real_gun_geometry.json'),...
    fullfile(root,'workpiece_pose_real.json'),...
    fullfile(root,'real_robot_description_calibrated.urdf'),...
    fullfile(here,'replan_weld_actual_axis_adaptive_core.m'),...
    fullfile(fileparts(root),'RRT','jointspace','FastConnectAnytimeRRTStarJoint.m')};
before=cellfun(@hmi_workflow_file_hash,protected,'UniformOutput',false);
scene=build_real_planning_scene(true); scene=apply_wire_collision_shorten_diagnostic(scene,3);
b=hmi_workflow_binding(scene,struct(),root);
qSafe=deg2rad([180 -90 0 -90 0 0]);
qCurrent=reshape(scene.qMat0,1,6);
oldRequest=jsondecode(fileread(fullfile(root,'规划文件','workflow_runs',...
    'preview_20261014_143725_608305','request.json')));
qInvalid=reshape(oldRequest.qStart,1,6);
good=validate_hmi_endpoint(scene,qSafe,'DIAGNOSTIC SAFE START',b);
current=validate_hmi_endpoint(scene,qCurrent,'CURRENT SAVED HOME',b);
bad=validate_hmi_endpoint(scene,qInvalid,'HISTORICAL COLLIDING START',b);
assert(good.valid,'Diagnostic q0 is no longer free in current scene');
req=struct('task','RealHome_to_PreWeld','qStart',qSafe,'qHome',qSafe,...
    'seed',20260917,'algorithm','V1 FastConnectAnytimeRRTStar',...
    'maxPlanningTimeSec',30,'planning_scene','REAL_CALIBRATION',...
    'calibration_binding',b,'realhome_version','DIAGNOSTIC_Q0_NOT_PRODUCTION_HOME',...
    'collision_scene_version',b.collision_scene_version,'preweld_version','STALE_OLD_SCENE',...
    'outputDir',fullfile(outputDir,'safe_home_validation'));
safeSaved=validate_hmi_realhome_request(req);
assert(safeSaved.success && any(strcmp(safeSaved.status,{'PASS','VALIDATION_PASS'})));
req.qStart=qInvalid; req.qHome=qInvalid; req.realhome_version='DIAGNOSTIC_CURRENT_SAVED_HOME';
req.outputDir=fullfile(outputDir,'invalid_home_validation');
invalidSaved=validate_hmi_realhome_request(req);
assert(invalidSaved.success==bad.valid);
% A legacy candidate path points to an impossible file. The orchestration
% must reject the invalid start before touching that path or calling RRT.
req.seamCandidateFile=fullfile(outputDir,'MUST_NOT_LOAD_cached_seam.mat');
req.outputDir=fullfile(outputDir,'invalid_start_preview');
invalidPreview=plan_hmi_preview_request(req);
assert(~bad.valid && ~invalidPreview.success && ~invalidPreview.rrt_entered);
assert(strcmp(invalidPreview.failure_identifier,'PREVIEW:InvalidStartState'));
assert(strcmp(invalidPreview.start_state_status,'FAIL'));
req.qStart=qSafe; req.qHome=qSafe; req.realhome_version='DIAGNOSTIC_Q0_NOT_PRODUCTION_HOME';
req.collision_scene_version='STALE_TEST_SCENE';
req.outputDir=fullfile(outputDir,'stale_request_preview');
stalePreview=plan_hmi_preview_request(req);
assert(~stalePreview.success && ~stalePreview.rrt_entered && strcmp(stalePreview.status,'STALE'));
report=struct('robot_motion','NO','executionReady',false,...
    'safe_diagnostic_realhome_rad',qSafe,'safe_diagnostic_realhome_deg',rad2deg(qSafe),...
    'production_home_not_overwritten',true,'safe_home_validation',safeSaved,...
    'current_saved_home_endpoint',current,'historical_invalid_home_validation',invalidSaved,...
    'invalid_start_preview',invalidPreview,...
    'stale_request_preview',stalePreview,'cold_planning_run',false);
if runColdPlanning
    req.collision_scene_version=b.collision_scene_version;
    req.outputDir=fullfile(outputDir,'cold_current_scene_preview');
    cold=plan_hmi_preview_request(req);
    assert(strcmp(cold.start_state_status,'PASS'));
    assert(~cold.seam_candidate_reused && cold.legacy_seam_candidate_ignored);
    assert(strcmp(cold.preweld_generation_source,'CURRENT_SCENE_COLD_START_ACCP'));
    snap=load(fullfile(req.outputDir,'raw_scene.mat'),'scene');
    assert(isequal(snap.scene.qMat0,qSafe),'Request RealHome did not become the scene start');
    report.cold_planning_run=true; report.cold_current_scene_preview=cold;
end
after=cellfun(@hmi_workflow_file_hash,protected,'UniformOutput',false);
assert(isequal(before,after),'Protected model/algorithm/production-home file changed');
report.protected_files=protected; report.protected_sha256_before=before;
report.protected_sha256_after=after; report.protected_files_unchanged=true;
fid=fopen(fullfile(outputDir,'regression_report.json'),'w'); c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(report,'PrettyPrint',true),'char');
fprintf('REALHOME_FLOW_REGRESSION_PASS invalid/stale gates, explicit start and no cached PreWeld; coldRun=%d\n',runColdPlanning);
end
