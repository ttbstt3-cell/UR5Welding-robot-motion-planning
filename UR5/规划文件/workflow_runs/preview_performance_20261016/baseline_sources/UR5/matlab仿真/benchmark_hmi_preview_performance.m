function summary=benchmark_hmi_preview_performance(sourceRequest,outputRoot,mode,repeats,profileFirst)
% Offline benchmark of the production entry, immutable request and sources.
% No ROS client, execution-ready output or calibration writes.
if nargin<3, mode='baseline'; end
if nargin<4, repeats=2; end
if nargin<5, profileFirst=false; end
here=fileparts(mfilename('fullpath')); root=fileparts(here);
addpath(here); addpath(genpath(fullfile(fileparts(root),'RRT')));
if ~isfolder(outputRoot), mkdir(outputRoot); end
request=jsondecode(fileread(sourceRequest));
request.performance_mode=mode; request.executionReady=false; request.preview_only=true;
summary=struct('mode',mode,'source_request',sourceRequest,'seed',request.seed,...
    'request_qStart',request.qStart,'runs',{{}},'executionReady',false,'robot_motion','NO');
for repetition=1:repeats
    run=fullfile(outputRoot,sprintf('%s_%02d',mode,repetition));
    if isfolder(run), error('BENCHMARK:OutputExists','Refusing to overwrite %s',run); end
    mkdir(run); request.outputDir=run;
    writeJson(fullfile(run,'request.json'),request);
    diary(fullfile(run,'matlab.log'));
    before=hmi_planning_dependency_snapshot(root);
    profiled=profileFirst && repetition==1;
    if profiled, profile clear; profile on; end
    wall=tic; report=plan_hmi_preview_request(request); wallSec=toc(wall);
    if profiled
        profile off; profiler=profile('info');
        save(fullfile(run,'matlab_profiler.mat'),'profiler','-v7');
        profileSummary=reduceProfile(profiler);
        writeJson(fullfile(run,'matlab_profiler_summary.json'),profileSummary);
    end
    after=hmi_planning_dependency_snapshot(root);
    stable=hmi_planning_dependency_changes(before,after);
    entry=struct('run',run,'profiled',profiled,'wall_sec',wallSec,...
        'report',report,'protected_dependencies',stable,'executionReady',false);
    planners=struct();
    for label={'approach','exit'}
        files=dir(fullfile(run,[label{1} '_planner_attempt_*.mat'])); attempts={};
        for k=1:numel(files)
            data=load(fullfile(files(k).folder,files(k).name),'candidate','branch'); p=data.candidate;
            keys={'success','planningTimeSec','firstSolutionTimeSec','samples','stage1Iterations',...
                'stage2Iterations','collisionConfigurationChecks','edgeCollisionChecks',...
                'rewireAttempts','rewireSuccesses','nodeCount','terminationReason'};
            a=struct('file',files(k).name,'physical_branch_pass',data.branch.pass);
            for key=keys, if isfield(p,key{1}), a.(key{1})=p.(key{1}); end, end
            attempts{end+1}=a; %#ok<AGROW>
        end
        planners.(label{1})=attempts;
    end
    entry.planner_attempts=planners;
    summary.runs{end+1}=entry;
    writeJson(fullfile(outputRoot,'benchmark_summary.json'),summary);
    fprintf('BENCHMARK_DONE mode=%s repetition=%d profiled=%d seconds=%.6f success=%d preview=%d\n',...
        mode,repetition,profiled,wallSec,report.success,report.previewReady);
    diary off;
end
end

function rows=reduceProfile(info)
f=info.FunctionTable; rows=struct('name',{},'file',{},'calls',{},'inclusive_sec',{},'self_sec',{});
for k=1:numel(f)
    child=0;
    if ~isempty(f(k).Children), child=sum([f(k).Children.TotalTime]); end
    rows(k)=struct('name',f(k).FunctionName,'file',f(k).FileName,'calls',f(k).NumCalls,...
        'inclusive_sec',f(k).TotalTime,'self_sec',max(0,f(k).TotalTime-child));
end
[~,order]=sort([rows.self_sec],'descend'); rows=rows(order);
end
function writeJson(path,value)
fid=fopen(path,'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(value,'PrettyPrint',true),'char');
end
