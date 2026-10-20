function test_dependency_guards
% Independent cold-start negative tests, not a robot controller connection.
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
restoredefaultpath; addpath(fullfile(root,'matlab仿真')); addpath(fullfile(fileparts(root),'RRT','benchmark'));
launch=jsondecode(fileread(fullfile(here,'after_launch.json')));
req=jsondecode(fileread(launch.request));
addpath(fullfile(here,'shadow'));
req.outputDir=fullfile(here,'shadow_rejected'); a=prepare_hmi_continuous_request(req);
assert(~a.success && ~a.executionReady && strcmp(a.failure_identifier,'CONTINUOUS:Dependency'));
assert(strcmp(a.failed_stage,'DEPENDENCY_VALIDATION'));
assert(~isfile(fullfile(req.outputDir,'offline_optimized_trajectory.mat')));
rmpath(fullfile(here,'shadow')); clear checkEnvironmentCollision;
req.raw_sha256=repmat('0',1,64); req.outputDir=fullfile(here,'stale_rejected');
b=prepare_hmi_continuous_request(req);
assert(~b.success && ~b.executionReady && strcmp(b.failure_identifier,'CONTINUOUS:Stale'));
assert(strcmp(b.dependency_paths.environment_collision,fullfile(fileparts(root),'RRT','checkEnvironmentCollision.m')));
assert(~isfile(fullfile(req.outputDir,'offline_optimized_trajectory.mat')));
result=struct('shadow_adapter_rejected',true,'stale_input_still_rejected',true,...
    'no_optimized_artifact_on_failure',true,'executionReady',false,'robot_motion','NO');
fid=fopen(fullfile(here,'dependency_guard_tests.json'),'w'); c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(result,'PrettyPrint',true),'char'); disp(result);
end
