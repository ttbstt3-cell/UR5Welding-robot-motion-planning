function report = run_hmi_weld_seam_batch(task,seed,runDir)
%RUN_HMI_WELD_SEAM_BATCH Headless test entry for plan_hmi_weld_request.
%
% Runs the HMI weld tasks entirely through matlab -batch, without the GUI.
% It only reads the validated scene/anchors, plans, validates and writes the
% trajectory MAT + report JSON.  It never sends ROS2, never touches the
% CoppeliaSim scene and never commands the robot.
%
%   run_hmi_weld_seam_batch()                              -> Weld_Seam
%   run_hmi_weld_seam_batch('Full_Welding_Task',20260917)
%   run_hmi_weld_seam_batch('Weld_Seam',20260917,'/tmp/x')

if nargin < 1 || isempty(task), task = 'Weld_Seam'; end
if nargin < 2 || isempty(seed), seed = 20260917; end
if nargin < 3 || isempty(runDir)
    runDir = fullfile('/home/ttbstt/jixiebi/UR5/planning_runs', ...
        ['hmi_weld_' lower(task) '_' char(datetime('now','Format','yyyyMMdd_HHmmss'))]);
end
if ~isfolder(runDir), mkdir(runDir); end

benchmarkDir = '/home/ttbstt/jixiebi/RRT/benchmark';
addpath(genpath('/home/ttbstt/jixiebi/RRT'));
addpath('/home/ttbstt/jixiebi/UR5/matlab仿真');
addpath(benchmarkDir);
setenv('RRT_HEADLESS','1');

request = struct();
request.task = task;
request.algorithm = 'V1 FastConnectAnytimeRRTStar';
request.seed = seed;
request.request_id = sprintf('batch_%s_seed%d',lower(task),seed);
request.outputFile = fullfile(runDir,'trajectory.mat');
request.reportFile = fullfile(runDir,'report.json');
request.requestFile = fullfile(runDir,'request.json');

% Explicit endpoints per task, matching what the HMI sends.  Leaving qGoal
% empty previously produced a silent Home->Home zero-length plan.
anchorFile = '/home/ttbstt/jixiebi/UR5/规划文件/weld_anchor_points.json';
A = jsondecode(fileread(anchorFile));
switch task
    case 'RealHome_to_PreWeld'
        request.qStart = double(A.qHome(:).');
        request.qGoal = double(A.qPreWeld(:).');
    case 'PostWeld_to_RealHome'
        request.qStart = double(A.qPostWeld(:).');
        request.qGoal = double(A.qHome(:).');
    case 'Weld_Seam'
        request.qStart = double(A.qPreWeld(:).');
        request.qGoal = double(A.qPostWeld(:).');
    case 'Full_Welding_Task'
        request.qStart = double(A.qHome(:).');
        request.qGoal = double(A.qHome(:).');
    otherwise
        error('Unknown task: %s',task);
end
fprintf('BATCH_TEST_ENDPOINTS start=[%s]\n',sprintf('%.2f ',rad2deg(request.qStart)));
fprintf('BATCH_TEST_ENDPOINTS goal =[%s]\n',sprintf('%.2f ',rad2deg(request.qGoal)));

fid = fopen(request.requestFile,'w');
fwrite(fid,jsonencode(request),'char'); fclose(fid);

fprintf('BATCH_TEST_START task=%s seed=%d dir=%s\n',task,seed,runDir);
tWall = tic;
ok = true;
try
    report = plan_hmi_weld_request(request);
catch err
    ok = false;
    report = struct('success',false,'task',task,'seed',seed, ...
        'error_message',string(err.message),'run_dir',runDir, ...
        'wall_time_sec',toc(tWall));
    fprintf('BATCH_TEST_ERROR task=%s\n  %s\n',task,err.message);
    fprintf('BATCH_TEST_DONE task=%s success=0\n',task);
    return;
end

report.run_dir = runDir;
report.batch_wall_time_sec = toc(tWall);

fprintf('\n================ BATCH TEST SUMMARY ================\n');
fprintf('task                     : %s\n',report.task);
fprintf('success                  : %d\n',report.success);
fprintf('collision PASS           : %d\n',report.collision_check_pass);
fprintf('continuity PASS          : %d\n',report.continuity_check_pass);
fprintf('qPath/tcpPath rows match : %d (%d / %d)\n', ...
    report.qpath_tcpath_rows_match,report.execution_point_count,report.tcp_point_count);
fprintf('seam planning points     : %s\n',mat2str(report.seam_point_count));
fprintf('seam execution points    : %s\n',mat2str(report.seam_execution_point_count));
fprintf('seam direction           : %s (%s)\n',report.seam_direction, ...
    localFieldText(report,'seam_direction_cascade',''));
fprintf('branch reference (deg)   : [%s]\n',sprintf('%.3f ',report.branch_reference_rad));
fprintf('branch reference source  : %s\n',report.branch_reference_source);
fprintf('max joint jump (deg)     : %.4f\n',report.max_joint_jump_deg);
fprintf('max TCP error (mm)       : %.6f\n',report.max_tcp_error_mm);
fprintf('max TCP step (mm)        : %.4f\n',report.max_tcp_step_mm);
fprintf('dense nodes / edges      : %d / %d\n', ...
    report.dense_validation.node_count,report.dense_validation.edge_count);
fprintf('dense collision checks   : %d\n',report.dense_validation.collision_configuration_checks);
fprintf('duration (s)             : %.3f\n',report.duration);
fprintf('wrapper wall time (s)    : %.3f\n',report.wall_time_sec);
fprintf('batch wall time (s)      : %.3f\n',report.batch_wall_time_sec);
fprintf('seam planner wall (s)    : %.3f\n',localFieldNumeric(report,'seam_planner_time_sec',NaN));
fprintf('seam algorithm time (s)  : %.3f\n',localFieldNumeric(report,'seam_algorithm_time_sec',NaN));
fprintf('trajectory MAT           : %s\n',report.output_file);
fprintf('report JSON              : %s\n',report.report_file);
fprintf('===================================================\n');
fprintf('BATCH_TEST_DONE task=%s success=%d\n',task,ok && report.success);
end

function t = localFieldText(s,name,default)
if isstruct(s) && isfield(s,name) && ~isempty(s.(name))
    t = char(string(s.(name)));
else
    t = char(string(default));
end
end

function v = localFieldNumeric(s,name,default)
if isstruct(s) && isfield(s,name) && ~isempty(s.(name))
    v = double(s.(name));
else
    v = default;
end
end