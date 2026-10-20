function report = run_real_welding_pipeline(outputDir, seed)
%RUN_REAL_WELDING_PIPELINE Run the version-bound REAL welding workflow.
%
% This is an offline orchestration entry point for the HMI.  It never opens
% a ROS connection and never sends a FollowJointTrajectory goal.  The strict
% scene builder is deliberately called before any planner: an INVALID or
% unverified workpiece/TCP, an unconfirmed gun package, or an invalid
% wire/tip definition produces PLAN_BLOCKED/PLAN_FAILED and no executable trajectory.
%
% On a fully reviewed scene the run directory contains:
%   request.json
%   weld_anchor_points_real.json
%   trajectory_real.mat
%   report.json

if nargin < 1 || isempty(outputDir)
    root = fileparts(fileparts(mfilename('fullpath')));
    outputDir = fullfile(root, '规划文件', 'real_planning_runs', ...
        ['real_' datestr(now,'yyyymmdd_HHMMSSFFF')]);
end
if nargin < 2 || isempty(seed), seed = 20260917; end
outputDir = char(string(outputDir));
if ~localAbsolute(outputDir)
    root = fileparts(fileparts(mfilename('fullpath')));
    outputDir = fullfile(root, outputDir);
end
if ~isfolder(outputDir), mkdir(outputDir); end

report = struct('success',false,'status','PLAN_BLOCKED','robot_motion','NO', ...
    'planning_scene','REAL_CALIBRATION','algorithm_free_space','CBi-RRT*', ...
    'algorithm_weld','ACCP','output_dir',outputDir,'seed',double(seed));
try
    here = fileparts(mfilename('fullpath'));
    addpath(genpath(here));
    addpath(genpath(fullfile(fileparts(fileparts(here)),'RRT')));

    % Strict preflight.  This is intentionally not the diagnostic scene.
    scene = build_real_planning_scene(false);
    report.calibration_binding = scene.calibrationBinding;
    report.calibration_versions = localField(scene,'calibrationVersions',struct());
    report.real_tcp_file = scene.realTcpFile;
    report.real_workpiece_file = scene.realWorkpieceFile;
    report.gun_geometry_manifest = scene.realGunGeometryManifest;
    report.wire_axis_status = char(string(scene.realWireAxisStatus));
    wireStatus = upper(string(scene.realWireAxisStatus));
    if ~any(wireStatus == ["MODEL_AXIS_TO_TCP_ACTIVE","MODEL_DEFINED_ACTIVE","VERIFIED","CONFIRMED"])
        error('REAL_PIPELINE:WireAxis','Real wire/tip geometry is unavailable or invalid; expected MODEL_DEFINED_ACTIVE, VERIFIED, or CONFIRMED after manifest checks.');
    end

    % First generate fresh seam anchors.  No frozen qPreWeld is read here.
    anchorReport = generate_real_weld_anchors(outputDir,false);
    report.anchor_report = anchorReport;
    if ~anchorReport.success || ~isfile(fullfile(outputDir,'weld_anchor_points_real.json'))
        error('REAL_PIPELINE:Anchors','ACCP did not produce a validated real anchor file.');
    end

    anchorFile = fullfile(outputDir,'weld_anchor_points_real.json');
    anchor = jsondecode(fileread(anchorFile));
    request = struct('request_id',localRequestId(), ...
        'task','Full_Welding_Task','algorithm','CBi-RRT*','seed',double(seed), ...
        'qStart',double(scene.qMat0),'qGoal',double(anchor.qHome), ...
        'sceneScript',fullfile(here,'sync_ur5_real_scene.m'), ...
        'anchorFile',anchorFile,'planning_scene','REAL_CALIBRATION', ...
        'calibration_binding',scene.calibrationBinding,'maxPlanningTimeSec',60);
    requestFile = fullfile(outputDir,'request.json');
    trajectoryFile = fullfile(outputDir,'trajectory_real.mat');
    reportFile = fullfile(outputDir,'report.json');
    request.outputFile = trajectoryFile;
    request.reportFile = reportFile;
    request.requestFile = requestFile;
    writeJson(requestFile,request);
    planReport = plan_hmi_weld_request(request);
    report.plan_report = planReport;
    if ~isfile(trajectoryFile)
        error('REAL_PIPELINE:Trajectory','Planner reported no trajectory_real.mat.');
    end
    report.success = logical(localField(planReport,'success',false));
    report.status = ternary(report.success,'VALIDATED_REAL_TRAJECTORY','PLAN_FAILED');
    report.trajectory_file = trajectoryFile;
catch err
    report.success = false;
    report.status = ternary(startsWith(string(err.identifier),'REAL_SCENE'), ...
        'PLAN_BLOCKED','PLAN_FAILED');
    report.failure_reason = err.message;
    report.failure_identifier = err.identifier;
    report.failure_stack = err.stack;
    fprintf('REAL_PIPELINE_%s %s\n',report.status,err.message);
end
writeJson(fullfile(outputDir,'report.json'),report);
end

function value = localField(s,name,default)
if isstruct(s) && isfield(s,name) && ~isempty(s.(name)), value=s.(name); else, value=default; end
end
function id = localRequestId()
id = char(string(java.util.UUID.randomUUID()));
end
function writeJson(file,data)
fid=fopen(file,'w'); if fid<0, error('REAL_PIPELINE:IO','Cannot write %s',file); end
cleanup=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(data),'char');
end
function tf = localAbsolute(path)
tf=startsWith(string(path),filesep) || (ispc && strlength(string(path))>=2 && extractBetween(string(path),2,2)==':');
end
function value=ternary(condition,a,b)
if condition, value=a; else, value=b; end
end
