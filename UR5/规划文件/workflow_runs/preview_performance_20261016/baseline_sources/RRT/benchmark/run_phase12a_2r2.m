function report = run_phase12a_2r2(options)
%run_phase12a_2r2 Two-stage Seam A collision certification calibration.
%
% The planner searches with plannerResolutionDeg.  Every internally valid
% candidate is immediately certified with validationResolutionDeg before it
% can be returned as a success.  The planner-owned wall clock includes both
% search and strict certification.

if nargin < 1 || isempty(options), options=struct(); end
root=fileparts(fileparts(mfilename('fullpath')));
defaults=struct( ...
    'taskSet','seamA', ...
    'plannerResolutionDeg',0.5, ...
    'validationResolutionDeg',0.125, ...
    'maxPlanningTimeSec',2.0, ...
    'maxIterations',3000, ...
    'maxJointStepRad',deg2rad(25), ...
    'goalBias',0.20, ...
    'seeds',1:5, ...
    'sceneScript','/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m', ...
    'approachResult','/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_approach_exit.mat', ...
    'outputFile',fullfile(root,'results','phase12a_2r2_0p5deg_40.mat'), ...
    'rawCsvFile',fullfile(root,'results','phase12a_2r2_0p5deg_40_runs.csv'), ...
    'summaryCsvFile',fullfile(root,'results','phase12a_2r2_0p5deg_40_summary.csv'), ...
    'checkpointFile','', ...
    'saveResult',true);
options=mergeOptions(defaults,options);
if isempty(options.checkpointFile)
    options.checkpointFile=[options.outputFile '.checkpoint.mat'];
end
addpath(genpath(root));
setenv('RRT_HEADLESS','1');

scene=loadUR5JointScene(options.sceneScript);
s=load(options.approachResult,'result');
source=s.result;
if ~source.fullPass || isempty(source.qWeld) || size(source.qWeld,2)~=6
    error('The current approach/exit result is not a valid full-pass source.');
end
qHome=double(source.qHome(:).');
qWeld=unwrap(double(source.qWeld),[],1);
tasks=makeSeamTasks(qHome,qWeld(1,:),qWeld(end,:),options.taskSet);
algorithms={'RRTConnectJoint','BiRRTStarJoint','ClassGoalLiteJoint','ClassifiedRRTStarJoint'};
nRuns=numel(tasks)*numel(algorithms)*numel(options.seeds);
rows=repmat(emptyRow(),nRuns,1);
paths=cell(nRuns,1);
metadata=cell(nRuns,1);
createdAt=string(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
runIndex=0;

fprintf('\nPHASE 12A-2R2: %d tasks x %d algorithms x %d seeds = %d runs\n', ...
    numel(tasks),numel(algorithms),numel(options.seeds),nRuns);
fprintf('planner=%.3f deg, certification=%.3f deg, deadline=%.3f s\n', ...
    options.plannerResolutionDeg,options.validationResolutionDeg,options.maxPlanningTimeSec);

for ti=1:numel(tasks)
    fprintf('\n[%s] %s\n',tasks(ti).taskID,tasks(ti).description);
    for si=1:numel(options.seeds)
        seed=options.seeds(si);
        for ai=1:numel(algorithms)
            runIndex=runIndex+1;
            cfg=struct('scene',scene,'start',tasks(ti).start,'goal',tasks(ti).goal, ...
                'maxIterations',options.maxIterations, ...
                'maxJointStepRad',options.maxJointStepRad, ...
                'jointEdgeResolutionRad',deg2rad(options.plannerResolutionDeg), ...
                'goalBias',options.goalBias,'seed',seed, ...
                'maxPlanningTimeSec',options.maxPlanningTimeSec);
            cfg.candidateValidatorFcn=@(path,abortFcn) certifyCandidate( ...
                path,tasks(ti),scene,deg2rad(options.validationResolutionDeg),abortFcn);
            t=feval(algorithms{ai},cfg);

            r=emptyRow();
            r.taskID=string(tasks(ti).taskID);
            r.description=string(tasks(ti).description);
            r.algorithm=string(t.algorithm);
            r.seed=seed;
            r.plannerResolutionDeg=options.plannerResolutionDeg;
            r.validationResolutionDeg=options.validationResolutionDeg;
            r.timeout=t.timedOut;
            r.firstCandidateTime_ms=t.firstCandidateTimeMs;
            r.firstCertifiedTime_ms=t.firstCertifiedTimeMs;
            r.candidateCount=t.candidateCount;
            r.candidateValidationFailures=t.candidateValidationFailures;
            r.certifiedSuccess=logical(t.certifiedSuccess && t.success && ~t.timedOut);
            r.success=r.certifiedSuccess;
            if r.certifiedSuccess
                r.outcome="VALID_SUCCESS";
            elseif t.timedOut
                r.outcome="TIMEOUT";
            elseif t.candidateValidationFailures>0
                r.outcome="NO_CERTIFIED_PATH";
            else
                r.outcome="PLANNER_FAIL";
            end
            r.planningTime_ms=t.planningTimeMs;
            r.jointPathLengthL2_rad=t.jointPathCostL2;
            r.totalJointMotion_rad=sum(t.totalJointMotionPerAxis,'omitnan');
            r.jointMotion1_rad=t.totalJointMotionPerAxis(1);
            r.jointMotion2_rad=t.totalJointMotionPerAxis(2);
            r.jointMotion3_rad=t.totalJointMotionPerAxis(3);
            r.jointMotion4_rad=t.totalJointMotionPerAxis(4);
            r.jointMotion5_rad=t.totalJointMotionPerAxis(5);
            r.jointMotion6_rad=t.totalJointMotionPerAxis(6);
            r.edgeCollisionChecks=t.edgeCollisionChecks;
            r.collisionConfigurationChecks=t.collisionConfigurationChecks;
            r.nodeCount=t.nodeCount;
            r.pathStateCount=t.pathStateCount;
            r.failedCandidateEdge=t.lastCandidateFailureEdge;
            r.failedCandidateSample=t.lastCandidateFailureSample;
            r.failedCandidateType=t.lastCandidateFailureType;
            r.candidateValidationTimedOut=t.lastCandidateValidationTimedOut;
            rows(runIndex)=r;
            paths{runIndex}=t.path;
            metadata{runIndex}=struct('taskID',r.taskID,'algorithm',r.algorithm, ...
                'seed',seed,'plannerResolutionDeg',r.plannerResolutionDeg, ...
                'validationResolutionDeg',r.validationResolutionDeg, ...
                'generatedAt',createdAt);
            fprintf('  seed=%02d %-24s outcome=%-17s cand=%d reject=%d firstCand=%8.1f ms firstCert=%8.1f ms checks=%d/%d\n', ...
                seed,t.algorithm,r.outcome,r.candidateCount, ...
                r.candidateValidationFailures,r.firstCandidateTime_ms, ...
                r.firstCertifiedTime_ms,r.edgeCollisionChecks, ...
                r.collisionConfigurationChecks);

            if options.saveResult
                checkpoint=makeReport(createdAt,options,tasks,algorithms,rows,paths,metadata,runIndex);
                checkpoint.note='Per-run checkpoint for two-stage strict certification calibration.';
                save(options.checkpointFile,'checkpoint','-v7.3');
            end
        end
    end
end

summary=makeSummary(rows,tasks,algorithms,options);
report=makeReport(createdAt,options,tasks,algorithms,rows,paths,metadata,runIndex);
report.summary=summary;
report.certifiedSuccessCount=sum([rows.certifiedSuccess]);
report.certifiedSuccessRate=report.certifiedSuccessCount/max(1,nRuns);
report.note='Only paths passing the in-budget strict certification are successes.';
if options.saveResult
    save(options.outputFile,'report','-v7.3');
    writetable(struct2table(rows),options.rawCsvFile);
    writetable(struct2table(summary),options.summaryCsvFile);
end
fprintf('\nPHASE12A2R2_DONE certified=%d/%d rate=%.3f\n', ...
    report.certifiedSuccessCount,nRuns,report.certifiedSuccessRate);
end

function [pass,detail]=certifyCandidate(path,task,scene,resolutionRad,abortFcn)
detail=struct('timedOut',false,'failureEdgeIndex',NaN, ...
    'failureSampleIndex',NaN,'failureCollisionType',"", ...
    'collisionConfigurationChecks',0,'edgeCollisionChecks',0);
pass=false;
if abortFcn(), detail.timedOut=true; return; end
if isempty(path) || size(path,2)~=6 || any(~isfinite(path(:))), return; end
if max(abs(path(1,:)-task.start))>1e-8 || max(abs(path(end,:)-task.goal))>1e-8, return; end
if ~all(arrayfun(@(i)jointWithinLimits(path(i,:),scene.jointLimits),1:size(path,1))), return; end
opts=struct();
opts.wrapJoints=true;
opts.selfCollisionSkipPairs=scene.selfCollisionSkipPairs;
opts.timeoutFcn=abortFcn;
for k=1:size(path,1)-1
    if abortFcn(), detail.timedOut=true; return; end
    detail.edgeCollisionChecks=detail.edgeCollisionChecks+1;
    [free,collisionIndex,count,timedOut]=collisionFreeJointEdge( ...
        scene.robot,path(k,:),path(k+1,:),scene.environment,resolutionRad,opts);
    detail.collisionConfigurationChecks=detail.collisionConfigurationChecks+count;
    if timedOut
        detail.timedOut=true;
        detail.failureEdgeIndex=k;
        detail.failureSampleIndex=collisionIndex;
        return;
    end
    if ~free
        detail.failureEdgeIndex=k;
        detail.failureSampleIndex=collisionIndex;
        % collisionFreeJointEdge deliberately returns only the first failing
        % sample.  Avoid an extra checkCollision call here: the strict
        % validator is itself inside the shared wall-clock budget.
        detail.failureCollisionType="collision";
        return;
    end
end
pass=true;
end

function tasks=makeSeamTasks(qHome,qStart,qEnd,taskSet)
if nargin<4 || isempty(taskSet), taskSet='seamA'; end
if ~strcmpi(taskSet,'seamA'), error('Phase 12A-2R2 only supports taskSet=seamA.'); end
tasks(1)=makeTask('T01_Safe_to_Point1', ...
    'Seam A entry: qSafe(qHome) -> Point 1',qHome,qStart);
tasks(2)=makeTask('T02_Point80_to_Safe', ...
    'Seam A exit: Point 80 -> qSafe(qHome)',qEnd,qHome);
end

function task=makeTask(taskID,description,start,goal)
task=struct('taskID',taskID,'description',description,'start',start,'goal',goal);
end

function row=emptyRow()
row=struct('taskID',"",'description',"",'algorithm',"",'seed',NaN, ...
    'plannerResolutionDeg',NaN,'validationResolutionDeg',NaN, ...
    'timeout',false,'outcome',"NOT_RUN",'success',false, ...
    'certifiedSuccess',false,'planningTime_ms',NaN, ...
    'firstCandidateTime_ms',NaN,'firstCertifiedTime_ms',NaN, ...
    'candidateCount',0,'candidateValidationFailures',0, ...
    'candidateValidationTimedOut',false,'jointPathLengthL2_rad',NaN, ...
    'totalJointMotion_rad',NaN,'jointMotion1_rad',NaN,'jointMotion2_rad',NaN, ...
    'jointMotion3_rad',NaN,'jointMotion4_rad',NaN,'jointMotion5_rad',NaN, ...
    'jointMotion6_rad',NaN,'edgeCollisionChecks',0, ...
    'collisionConfigurationChecks',0,'nodeCount',0,'pathStateCount',0, ...
    'failedCandidateEdge',NaN,'failedCandidateSample',NaN, ...
    'failedCandidateType',"");
end

function report=makeReport(createdAt,options,tasks,algorithms,rows,paths,metadata,completedRuns)
report=struct('phase',"Phase 12A-2R2",'createdAt',createdAt, ...
    'options',options,'tasks',tasks,'algorithms',{algorithms}, ...
    'rows',rows,'paths',{paths},'pathMetadata',{metadata}, ...
    'completedRuns',completedRuns);
end

function summary=makeSummary(rows,tasks,algorithms,options)
summary=repmat(struct('taskID',"",'algorithm',"", ...
    'plannerResolutionDeg',options.plannerResolutionDeg, ...
    'validationResolutionDeg',options.validationResolutionDeg, ...
    'seedCount',0,'certifiedSuccessCount',0,'certifiedSuccessRate',NaN, ...
    'timeoutCount',0,'candidateCountMedian',NaN, ...
    'candidateValidationFailuresMedian',NaN,'candidateRejectionRate',NaN, ...
    'firstCertifiedTimeMedian_ms',NaN,'firstCertifiedTimeP90_ms',NaN, ...
    'planningTimeMedian_ms',NaN,'jointPathLengthMedian_rad',NaN, ...
    'totalJointMotionMedian_rad',NaN,'collisionConfigurationChecksMedian',NaN), ...
    numel(tasks)*numel(algorithms),1);
idx=0;
for ti=1:numel(tasks)
    for ai=1:numel(algorithms)
        idx=idx+1;
        subset=rows([rows.taskID]==string(tasks(ti).taskID) & ...
            [rows.algorithm]==algorithmLabel(algorithms{ai}));
        if isempty(subset), continue; end
        certified=[subset.certifiedSuccess];
        certTime=[subset.firstCertifiedTime_ms]; certTime=certTime(isfinite(certTime));
        pathLength=[subset.jointPathLengthL2_rad]; pathLength=pathLength(isfinite(pathLength));
        planTime=[subset.planningTime_ms]; planTime=planTime(isfinite(planTime));
        failures=[subset.candidateValidationFailures];
        candidates=[subset.candidateCount];
        summary(idx).taskID=string(tasks(ti).taskID);
        summary(idx).algorithm=algorithmLabel(algorithms{ai});
        summary(idx).seedCount=numel(subset);
        summary(idx).certifiedSuccessCount=sum(certified);
        summary(idx).certifiedSuccessRate=mean(certified);
        summary(idx).timeoutCount=sum([subset.timeout]);
        summary(idx).candidateCountMedian=median(candidates);
        summary(idx).candidateValidationFailuresMedian=median(failures);
        summary(idx).candidateRejectionRate=sum(failures)/max(1,sum(candidates));
        summary(idx).firstCertifiedTimeMedian_ms=finiteMedian(certTime);
        summary(idx).firstCertifiedTimeP90_ms=finitePercentile(certTime,90);
        summary(idx).planningTimeMedian_ms=finiteMedian(planTime);
        summary(idx).jointPathLengthMedian_rad=finiteMedian(pathLength);
        motions=[subset.totalJointMotion_rad]; motions=motions(isfinite(motions));
        checks=[subset.collisionConfigurationChecks];
        summary(idx).totalJointMotionMedian_rad=finiteMedian(motions);
        summary(idx).collisionConfigurationChecksMedian=finiteMedian(checks);
    end
end
end

function label=algorithmLabel(name)
switch char(name)
    case 'RRTConnectJoint', label="RRT-Connect";
    case 'BiRRTStarJoint', label="Bi-RRT*";
    case 'ClassGoalLiteJoint', label="ClassGoal-Lite";
    case 'ClassifiedRRTStarJoint', label="Full Classified-RRT*";
    otherwise, label=string(name);
end
end

function out=mergeOptions(defaults,in)
out=defaults;
names=fieldnames(defaults);
for i=1:numel(names)
    if isfield(in,names{i}) && ~isempty(in.(names{i})), out.(names{i})=in.(names{i}); end
end
end

function value=finiteMedian(values)
if isempty(values), value=NaN; else, value=median(values); end
end

function value=finitePercentile(values,p)
if isempty(values), value=NaN; return; end
values=sort(values(:));
position=1+(numel(values)-1)*p/100;
lo=floor(position); hi=ceil(position);
if lo==hi, value=values(lo); else, value=values(lo)+(position-lo)*(values(hi)-values(lo)); end
end
