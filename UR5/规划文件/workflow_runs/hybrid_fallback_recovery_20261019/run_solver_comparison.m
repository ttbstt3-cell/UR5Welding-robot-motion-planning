function run_solver_comparison()
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT')));
addpath(fullfile(here,'solver_copy'));
request=jsondecode(fileread(fullfile(fileparts(here),'prepare_20261019_152229_336637','request.json')));
input=load(fullfile(fileparts(here),'prepare_20261019_152229_336637','velocity_profile_input_diagnostic.mat'),'base');
raw=load(request.rawFile,'sceneFile'); ss=load(raw.sceneFile,'scene'); scene=ss.scene;
scene.hmiMotionLimits=hmi_load_robot_motion_limits(request.motion_limits_file,request.motion_limits_sha256);
assert(strcmp(hmi_workflow_file_hash(request.rawFile),request.raw_sha256));
algorithms={'active-set','interior-point-convex'}; results=cell(1,2);
for k=1:2
    clock=tic;
    r=hybrid_solver_comparison_copy(input.base,scene,struct('compareReferences',false,'solverAlgorithm',algorithms{k}));
    elapsed=toc(clock);
    assert(isequal(r.qPath,input.base.qPath) && isequal(r.timeFromStart,input.base.timeFromStart));
    assert(isequal(r.tcpPosePath,input.base.tcpPosePath) && ~r.executionReady);
    audit=r.audits{4}; metrics=r.derivativeMetrics{4};
    results{k}=struct('algorithm',algorithms{k},'budget_iterations',1000,...
        'hybridCandidateFound',r.hybridCandidateFound,'selected_candidate',r.candidate,...
        'attempts',{r.attempts},'elapsed_sec',elapsed,'weld_audit',audit,'derivatives',metrics,...
        'full_task_validated',false,'executionReady',false);
    save(fullfile(here,sprintf('solver_%d_diagnostic.mat',k)),'r','-v7.3');
    writeJson(fullfile(here,sprintf('solver_%d_report.json',k)),results{k});
    fprintf('SOLVER_COMPARISON algorithm=%s D_found=%d elapsed=%.6f weld_pass=%d executionReady=0\n',...
        algorithms{k},r.hybridCandidateFound,elapsed,audit.complete);
end
writeJson(fullfile(here,'solver_comparison.json'),struct('scope','INDEPENDENT_COPY_ONLY',...
    'same_Q_TCP_time_constraints_objective',true,'raw_sha256',request.raw_sha256,...
    'results',{results},'production_solver_changed',false,'executionReady',false,'robot_motion','NO'));
end
function writeJson(path,value)
fid=fopen(path,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(value,'PrettyPrint',true),'char');
end
