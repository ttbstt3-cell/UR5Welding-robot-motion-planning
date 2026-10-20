function report = run_phase12a_anytime_seamA_two_successful(options)
%RUN_PHASE12A_ANYTIME_SEAMA_TWO_SUCCESSFUL
% Run the two successful method-validation algorithms on both free-space
% segments of Seam A: Home->PreWeld and PostWeld->Home.

if nargin < 1 || isempty(options), options=struct(); end
root=fileparts(fileparts(mfilename('fullpath')));
defaults=struct('approachResult',fullfile(root,'results','phase12a_seam_legacy_rrtconnect_repro.mat'), ...
    'plannerResolutionDeg',2.0,'validationResolutionDeg',0.5, ...
    'maxPlanningTimeSec',30.0,'maxIterations',1000000,'maxJointStepRad',deg2rad(12), ...
    'goalBias',0.20,'randomSeedBase',20260917,'exitSeedOffset',7919, ...
    'continueAfterFirstSolution',true, ...
    'outputFile',fullfile(root,'results','phase12a_anytime_seamA_two_successful.mat'), ...
    'csvFile',fullfile(root,'results','phase12a_anytime_seamA_two_successful.csv'));
options=localMergeOptions(defaults,options);
addpath(genpath(root)); setenv('RRT_HEADLESS','1');
if ~isfile(options.approachResult), error('Missing frozen result: %s',options.approachResult); end
scene=loadUR5JointScene(); s=load(options.approachResult,'result'); source=s.result;
qHome=double(source.qHome(:).'); qPre=double(source.preWeld.q(:).'); qPost=double(source.postWeld.q(:).');
tasks(1)=makeTask("Home_to_PreWeld","Home -> PreWeld",qHome,qPre);
tasks(2)=makeTask("PostWeld_to_Home","PostWeld -> Home",qPost,qHome);
algorithms={'RRTConnectJoint','BiRRTStarJoint'};
labels=["RRT-Connect","Bi-RRT*"];
rows=repmat(emptyRow(),numel(tasks)*numel(algorithms),1); paths=cell(numel(tasks),numel(algorithms));
idx=0; createdAt=string(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
for ti=1:numel(tasks)
    effectiveSeed=options.randomSeedBase;
    if ti==2, effectiveSeed=effectiveSeed+options.exitSeedOffset; end
    fprintf('\n%s seed=%d\n',tasks(ti).description,effectiveSeed);
    for ai=1:numel(algorithms)
        idx=idx+1; isConnect=ai==1;
        cfg=struct('scene',scene,'start',tasks(ti).start,'goal',tasks(ti).goal, ...
            'maxIterations',options.maxIterations,'maxJointStepRad',options.maxJointStepRad, ...
            'jointEdgeResolutionRad',deg2rad(options.plannerResolutionDeg), ...
            'goalBias',options.goalBias,'seed',effectiveSeed, ...
            'maxPlanningTimeSec',options.maxPlanningTimeSec,'anytime',true, ...
            'continueAfterFirstSolution',options.continueAfterFirstSolution && ~isConnect, ...
            'checkpointTimesSec',[5 10 15 20 30], ...
            'candidateValidatorFcn',@(path,abortFcn)strictValidatePath(scene,path,tasks(ti), ...
                deg2rad(options.validationResolutionDeg),abortFcn));
        t=feval(algorithms{ai},cfg); r=emptyRow();
        r.taskID=tasks(ti).taskID; r.description=tasks(ti).description; r.algorithm=labels(ai);
        r.seed=1; r.effectiveSeed=effectiveSeed; r.success=logical(t.success); r.certifiedSuccess=logical(t.certifiedSuccess);
        r.outcome=string(t.terminationReason); r.planningTime_ms=t.planningTimeMs;
        r.firstSolutionTime_ms=t.firstSolutionTimeMs; r.firstPathLengthL2_rad=t.firstPathCostL2;
        r.bestPathLengthAt30s=t.bestPathCostL2; r.nodeCount=t.nodeCount;
        r.iterations=t.iterations; r.firstSolutionIteration=t.firstSolutionIteration; r.samples=t.samples;
        r.edgeCollisionChecks=t.edgeCollisionChecks; r.collisionConfigurationChecks=t.collisionConfigurationChecks;
        r.candidateCount=t.candidateCount; r.candidateValidationFailures=t.candidateValidationFailures;
        r.totalJointMotion_rad=sum(t.totalJointMotionPerAxis,'omitnan'); paths{ti,ai}=t.path; rows(idx)=r;
        fprintf('  %-16s first=%8.3f s Lfirst=%9.4f Lbest=%9.4f outcome=%s\n',char(labels(ai)), ...
            r.firstSolutionTime_ms/1000,r.firstPathLengthL2_rad,r.bestPathLengthAt30s,char(r.outcome));
    end
end
report=struct('phase',"Phase 12A anytime Seam A two successful algorithms",'createdAt',createdAt, ...
    'options',options,'tasks',tasks,'algorithms',{algorithms},'algorithmLabels',labels, ...
    'rows',rows,'paths',{paths},'source',source, ...
    'note',"Only free-space segments were replanned; PreWeld->Point1, Point1->Point80 and Point80->PostWeld remain fixed.");
save(options.outputFile,'report','-v7.3'); writetable(struct2table(rows),options.csvFile);
fprintf('\nDONE output=%s\n',options.outputFile);
end

function [pass,detail]=strictValidatePath(scene,path,task,resolutionRad,abortFcn)
detail=struct('collisionConfigurationChecks',0,'edgeCollisionChecks',0,'timedOut',false); pass=false;
if abortFcn() || isempty(path) || size(path,2)~=6 || any(~isfinite(path(:))), detail.timedOut=abortFcn(); return; end
if max(abs(path(1,:)-task.start))>1e-8 || max(abs(path(end,:)-task.goal))>1e-8, return; end
if any(~arrayfun(@(i)jointWithinLimits(path(i,:),scene.jointLimits),1:size(path,1))), return; end
for i=1:size(path,1)
    if abortFcn(), detail.timedOut=true; return; end
    [free,~]=collisionFreeJointState(scene.robot,path(i,:),scene.environment,scene.selfCollisionSkipPairs);
    detail.collisionConfigurationChecks=detail.collisionConfigurationChecks+1; if ~free, return; end
end
for i=1:size(path,1)-1
    if abortFcn(), detail.timedOut=true; return; end
    opts=struct(); opts.wrapJoints=true;
    opts.selfCollisionSkipPairs=scene.selfCollisionSkipPairs; opts.timeoutFcn=abortFcn;
    [free,~,count,timedOut]=collisionFreeJointEdge(scene.robot,path(i,:),path(i+1,:),scene.environment,resolutionRad,opts);
    detail.edgeCollisionChecks=detail.edgeCollisionChecks+1; detail.collisionConfigurationChecks=detail.collisionConfigurationChecks+count;
    if timedOut || abortFcn(), detail.timedOut=true; return; end
    if ~free, return; end
end
pass=true;
end

function task=makeTask(id,description,start,goal)
task=struct('taskID',id,'description',description,'start',start,'goal',goal);
end

function row=emptyRow()
row=struct('taskID',"",'description',"",'algorithm',"",'seed',NaN,'effectiveSeed',NaN, ...
    'success',false,'certifiedSuccess',false,'outcome',"",'planningTime_ms',NaN,'firstSolutionTime_ms',NaN, ...
    'firstPathLengthL2_rad',NaN,'bestPathLengthAt30s',NaN,'iterations',0,'firstSolutionIteration',NaN, ...
    'samples',0,'nodeCount',0,'edgeCollisionChecks',0, ...
    'collisionConfigurationChecks',0,'candidateCount',0,'candidateValidationFailures',0,'totalJointMotion_rad',NaN);
end

function out=localMergeOptions(defaults,in)
out=defaults; f=fieldnames(defaults); for i=1:numel(f), if isfield(in,f{i}) && ~isempty(in.(f{i})), out.(f{i})=in.(f{i}); end, end
end
