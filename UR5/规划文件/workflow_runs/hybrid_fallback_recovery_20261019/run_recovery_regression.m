function run_recovery_regression()
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT')));
request=jsondecode(fileread(fullfile(fileparts(here),'prepare_20261019_152229_336637','request.json')));
request.outputDir=fullfile(here,'full_prepare'); request.request_id='hybrid_fallback_offline_20261019';
if ~isfolder(request.outputDir), mkdir(request.outputDir); end
writeJson(fullfile(request.outputDir,'request.json'),request);
report=prepare_hmi_continuous_request(request);
assert(~report.executionReady,'Offline recovery must not grant execution permission');
assert(strcmp(report.hybrid_velocity_candidate.source,'C_ACCURACY_FALLBACK'));
assert(report.hybrid_velocity_candidate.nonoptimal_fallback);
input=load(fullfile(request.outputDir,'velocity_profile_input_diagnostic.mat'),'base');
h=load(fullfile(request.outputDir,'hybrid_velocity_candidate_diagnostic.mat'),'hybrid');
raw=load(request.rawFile,'sceneFile','segments','metadata'); ss=load(raw.sceneFile,'scene'); scene=ss.scene;
scene.hmiMotionLimits=hmi_load_robot_motion_limits(request.motion_limits_file,request.motion_limits_sha256);
contract=test_hmi_hybrid_candidate_contract(input.base,scene,h.hybrid);
writeJson(fullfile(here,'candidate_contract_tests.json'),contract);
assert(~isfile(fullfile(request.outputDir,'trajectory_execute_ready.mat')) &&...
    ~isfile(fullfile(request.outputDir,'trajectory_execute_ready.json')));
if report.success
    assert(report.continuous_planning_complete && report.controller_validation.complete);
    assert(all(structfun(@(x)isequal(x,true),report.offline_checks)));
    assert(strcmp(report.stage,'PHYSICAL_VALIDATION_PENDING'));
    output=load(report.artifacts.trajectory_mat,'qPath','tcpPosePath','metadata');
    assert(~output.metadata.executionReady && output.metadata.diagnostic_only);
    detail=load(fullfile(request.outputDir,'controller_continuity_diagnostic.mat'),'candidate');
    c=detail.candidate;
    for k=1:3
        assert(isequal(c.qPath(c.maps{k},:),raw.segments{k}.qPath));
        p=c.tcpPosePath(c.maps{k}); ref=raw.segments{k}.tcpPosePath;
        assert(isequal(p(:),ref(:)));
    end
    fprintf('FULL_RECOVERY_OFFLINE_PASS executionReady=0\n');
else
    assert(strcmp(report.stage,'CONTINUOUS_PLANNING_FAILED') && ~report.continuous_planning_complete);
    fprintf('FULL_RECOVERY_FAIL_CLOSED stage=%s reason=%s\n',report.failed_stage,report.failure_reason);
end
end
function writeJson(path,value)
fid=fopen(path,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(value,'PrettyPrint',true),'char');
end
