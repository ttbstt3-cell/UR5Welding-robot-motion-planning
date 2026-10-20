function run_staged_current
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT')));
addpath(fullfile(here,'staged'));
req=jsondecode(fileread(fullfile(fileparts(here),'prepare_20261019_152229_336637','request.json')));
req.outputDir=fullfile(here,'staged_current');
r=prepare_hmi_continuous_request(req);
assert(r.success && ~r.executionReady && r.final_acceptance_pass,...
    'CONTINUOUS:Staged','Staged whole-task preparation failed');
assert(strcmp(r.stage,'PHYSICAL_VALIDATION_PENDING'));
assert(isfile(r.artifacts.trajectory_mat));
end
