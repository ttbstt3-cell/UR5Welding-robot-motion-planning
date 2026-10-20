function run_production_current
% Full production entry, isolated output only. No ROS, global index or robot.
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT')));
assert(strcmp(which('hmi_continuous_parameterize'),fullfile(root,'matlab仿真','hmi_continuous_parameterize.m')));
assert(strcmp(which('validate_accp_controller_interpolation'),fullfile(root,'matlab仿真','validate_accp_controller_interpolation.m')));
req=jsondecode(fileread(fullfile(fileparts(here),'prepare_20261019_152229_336637','request.json')));
req.outputDir=fullfile(here,'production_current');
if ~isfolder(req.outputDir), mkdir(req.outputDir); end
fid=fopen(fullfile(req.outputDir,'request.json'),'w'); fwrite(fid,jsonencode(req,'PrettyPrint',true),'char'); fclose(fid);
r=prepare_hmi_continuous_request(req);
assert(r.success && r.final_acceptance_pass && ~r.executionReady);
assert(r.parameterization.smoothness_preferences_pass && r.parameterization.rest_end_acceleration_pass);
assert(strcmp(r.stage,'PHYSICAL_VALIDATION_PENDING'));
fields={'qPath','velocities','timeFromStart','tcpPosePath'};
a=load(r.artifacts.trajectory_mat,fields{:});
b=load(fullfile(here,'staged_current','offline_optimized_trajectory.mat'),fields{:});
assert(isequaln(a,b),'SMOOTH:StagedMismatch','Production does not reproduce staged trajectory');
assert(r.controller_validation.collision_sample_count==63701);
fprintf('PRODUCTION_FULL_PASS total=%.6f executionReady=%d\n',r.planning_time,r.executionReady);
run_backend_contracts(true);
end
