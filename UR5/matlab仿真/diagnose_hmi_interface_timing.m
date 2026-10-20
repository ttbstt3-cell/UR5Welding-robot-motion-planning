function report=diagnose_hmi_interface_timing(repairedDir,sourceDir,outputDir)
% Run the unchanged timing solvers only AFTER interface geometry is valid.
here=fileparts(mfilename('fullpath')); addpath(here);
diagnosis=jsondecode(fileread(fullfile(repairedDir,'interface_diagnosis.json')));
assert(diagnosis.interface_pass,'TIMING:Interface','Interface must pass before timing');
file=fullfile(repairedDir,'full_raw_trajectory.mat'); raw=load(file);
source=load(fullfile(sourceDir,'preview_accp.mat'),'result');
if ~isfolder(outputDir), mkdir(outputDir); end
request=struct('rawFile',file,'raw_sha256',hmi_workflow_file_hash(file),'outputDir',outputDir,...
    'current_binding',raw.metadata.calibration_binding,'minControllerSamples',5000,...
    'maxBoundaryRefinementDepth',2,'maxBoundaryIKCalls',100,'maxBoundaryTimeSec',30,...
    'execution_gate_reasons',{{'OFFLINE_INTERFACE_DIAGNOSTIC_ONLY: physical safety not certified'}});
report=struct('executionReady',false,'robot_motion','NO','interface_pass',true,...
    'weld_nodes',size(raw.segments{2}.qPath,1),...
    'weld_nodes_exactly_unchanged',isequal(raw.segments{2}.qPath,source.result.qPath),...
    'weld_tcp_exactly_unchanged',isequal(raw.segments{2}.tcpPosePath,source.result.tcpPosePath),...
    'original_timing_failure',source.result.timeParameterizationFailureReason);
cv=source.result.timeParameterization.controllerInterpolationValidation;
report.original_controller_audit=summary(cv);
report.preparation=prepare_hmi_continuous_request(request);
report.failure_reason='';
if isfield(report.preparation,'failure_reason'), report.failure_reason=report.preparation.failure_reason; end
report.execute_ready_files_generated=isfile(fullfile(outputDir,'trajectory_execute_ready.mat')) ||...
    isfile(fullfile(outputDir,'trajectory_execute_ready.json'));
assert(~report.execute_ready_files_generated && ~report.preparation.executionReady);
fid=fopen(fullfile(outputDir,'timing_summary.json'),'w'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(report,'PrettyPrint',true),'char');
end
function r=summary(a)
r=struct();
for f=fieldnames(a).'
    v=a.(f{1});
    if ((isnumeric(v)||islogical(v)) && isscalar(v)) || ischar(v) || isstring(v), r.(f{1})=v; end
end
end
