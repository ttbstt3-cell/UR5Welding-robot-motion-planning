function run_staged_current
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT'))); addpath(fullfile(here,'staged'));
req=jsondecode(fileread(fullfile(fileparts(here),'prepare_20261019_152229_336637','request.json')));
req.outputDir=fullfile(here,'staged_current');
if ~isfolder(req.outputDir), mkdir(req.outputDir); end
fid=fopen(fullfile(req.outputDir,'request.json'),'w'); fwrite(fid,jsonencode(req,'PrettyPrint',true),'char'); fclose(fid);
r=prepare_hmi_continuous_request(req);
assert(r.success && r.final_acceptance_pass && ~r.executionReady);
z=load(r.artifacts.trajectory_mat,'qPath','velocities','timeFromStart','tcpPosePath');
old=load(fullfile(fileparts(here),'continuous_refactor_20261019','staged_current','offline_optimized_trajectory.mat'),'qPath','tcpPosePath');
assert(isequal(z.qPath,old.qPath) && isequal(z.tcpPosePath,old.tcpPosePath),'SMOOTH:FixedNodes','Execution geometry changed');
assert(r.parameterization.smoothness_preferences_pass && r.parameterization.rest_end_acceleration_pass);
assert(strcmp(r.stage,'PHYSICAL_VALIDATION_PENDING'));
end
