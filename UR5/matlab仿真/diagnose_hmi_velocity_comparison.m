function result=diagnose_hmi_velocity_comparison(preparationDir,outputDir)
% Explicit offline experiment, never called by either HMI planning button.
% Replays one recorded preparation's immutable scene and exact timed weld.
% Does not change Home, calibration, scene, trajectory or execution permission.
preparationDir=char(preparationDir);
if nargin<2
    outputDir=fullfile(preparationDir,['velocity_comparison_' datestr(now,'yyyymmdd_HHMMSS_FFF')]);
end
outputDir=char(outputDir);
assert(~isfolder(outputDir) && ~isfile(outputDir),...
    'VELOCITY_COMPARISON:OutputExists','Choose a new diagnostic output directory');
request=jsondecode(fileread(fullfile(preparationDir,'request.json')));
assert(strcmp(hmi_workflow_file_hash(request.rawFile),request.raw_sha256),...
    'VELOCITY_COMPARISON:Stale','Recorded raw preview file changed');
raw=load(request.rawFile,'sceneFile','metadata');
assert(strcmp(hmi_workflow_file_hash(raw.sceneFile),raw.metadata.scene_snapshot_sha256),...
    'VELOCITY_COMPARISON:Stale','Recorded scene snapshot changed');
ss=load(raw.sceneFile,'scene');
saved=load(fullfile(preparationDir,'velocity_profile_input_diagnostic.mat'),'base');
base=saved.base;
assert(isequal(base.calibrationBinding,raw.metadata.calibration_binding),...
    'VELOCITY_COMPARISON:Binding','Velocity input and recorded scene bindings differ');
mkdir(outputDir); diary(fullfile(outputDir,'matlab.log'));
cleanup=onCleanup(@()diary('off')); %#ok<NASGU>
result=generate_accp_adaptive_velocity_profile(base.qPath,base.tcpPosePath,...
    base.timeFromStart,ss.scene,struct('compareStrategies',true,'outputDirectory',outputDir));
assert(isequal(base.qPath,result.qPath) && isequal(base.tcpPosePath,result.tcpPosePath) &&...
    isequal(base.timeFromStart,result.timeFromStart),'Recorded inputs changed');
fprintf('OFFLINE_COMPARISON_COMPLETE executionReady=0 robot_motion=NO\n');
end
