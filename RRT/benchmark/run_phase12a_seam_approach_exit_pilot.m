function report = run_phase12a_seam_approach_exit_pilot(options)
%run_phase12a_seam_approach_exit_pilot Compare four 6D planners on the
% already validated Seam A approach and exit free-space segments.
%
% The fixed task geometry is deliberately the historical working setup:
%   qHome -> qPreWeld
%   qPostWeld -> qHome
% The local qPreWeld->Point1 and Point80->qPostWeld connections and the
% 80-point weld path are not replanned here.

if nargin < 1 || isempty(options), options=struct(); end
root=fileparts(fileparts(mfilename('fullpath')));
defaults=struct( ...
    'approachResult',fullfile(root,'results','phase12a_seam_legacy_rrtconnect_repro.mat'), ...
    'sceneScript','/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m', ...
    'plannerResolutionDeg',2.0, ...
    'validationResolutionDeg',0.5, ...
    'maxPlanningTimeSec',30.0, ...
    'maxIterations',8000, ...
    'maxJointStepRad',deg2rad(12), ...
    'goalBias',0.20, ...
    'randomSeedBase',20260917, ...
    'exitSeedOffset',7919, ...
    'seeds',1:5, ...
    'outputFile',fullfile(root,'results','phase12a_seam_approach_exit_pilot_40.mat'), ...
    'rawCsvFile',fullfile(root,'results','phase12a_seam_approach_exit_pilot_40_runs.csv'), ...
    'summaryCsvFile',fullfile(root,'results','phase12a_seam_approach_exit_pilot_40_summary.csv'), ...
    'checkpointFile','', ...
    'saveResult',true);
options=mergeOptions(defaults,options);
if isempty(options.checkpointFile)
    options.checkpointFile=[options.outputFile '.checkpoint.mat'];
end
if ~isfile(options.approachResult)
    error('Frozen approach/exit result not found: %s',options.approachResult);
end
addpath(genpath(root));
setenv('RRT_HEADLESS','1');

scene=loadUR5JointScene(options.sceneScript);
s=load(options.approachResult,'result');
source=s.result;
if ~isfield(source,'fullPass') || ~source.fullPass
    error('Frozen approach/exit result is not fullPass.');
end
qHome=double(source.qHome(:).');
qPre=double(source.preWeld.q(:).');
qPost=double(source.postWeld.q(:).');
tasks(1)=makeTask('T01_Safe_to_PreWeld','qSafe(qHome) -> qPreWeld',qHome,qPre);
tasks(2)=makeTask('T02_PostWeld_to_Safe','qPostWeld -> qSafe(qHome)',qPost,qHome);
algorithms={'RRTConnectJoint','BiRRTStarJoint','ClassGoalLiteJoint','ClassifiedRRTStarJoint'};
nRuns=numel(tasks)*numel(algorithms)*numel(options.seeds);
rows=repmat(emptyRow(),nRuns,1);
paths=cell(nRuns,1); metadata=cell(nRuns,1);
createdAt=string(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
runIndex=0;

fprintf('\nPHASE 12A SEAM APPROACH/EXIT PILOT: %d tasks x %d algorithms x %d seeds = %d runs\n', ...
    numel(tasks),numel(algorithms),numel(options.seeds),nRuns);
fprintf('planner edge=%.1f deg, validation edge=%.1f deg, guard=%.1f s, step=%.1f deg\n', ...
    options.plannerResolutionDeg,options.validationResolutionDeg, ...
    options.maxPlanningTimeSec,rad2deg(options.maxJointStepRad));

for ti=1:numel(tasks)
    fprintf('\n[%s] %s\n',tasks(ti).taskID,tasks(ti).description);
    for si=1:numel(options.seeds)
        seed=options.seeds(si);
        effectiveSeed=options.randomSeedBase+seed-1;
        if ti==2, effectiveSeed=effectiveSeed+options.exitSeedOffset; end
        for ai=1:numel(algorithms)
            runIndex=runIndex+1;
            cfg=struct('scene',scene,'start',tasks(ti).start,'goal',tasks(ti).goal, ...
                'maxIterations',options.maxIterations, ...
                'maxJointStepRad',options.maxJointStepRad, ...
                'jointEdgeResolutionRad',deg2rad(options.plannerResolutionDeg), ...
                'goalBias',options.goalBias,'seed',effectiveSeed, ...
                'maxPlanningTimeSec',options.maxPlanningTimeSec);
            t=feval(algorithms{ai},cfg);
            validation=validatePath(scene,t.path,tasks(ti),deg2rad(options.validationResolutionDeg));

            r=emptyRow();
            r.taskID=string(tasks(ti).taskID);
            r.description=string(tasks(ti).description);
            r.algorithm=string(t.algorithm);
            r.seed=seed;
            r.effectiveSeed=effectiveSeed;
            r.plannerSuccess=logical(t.success);
            r.validationPass=logical(validation.pass);
            r.success=r.plannerSuccess && r.validationPass && ~t.timedOut;
            r.plannerTimedOut=logical(t.timedOut);
            if r.success
                r.outcome="VALID_SUCCESS";
            elseif r.plannerTimedOut
                r.outcome="TIMEOUT";
            elseif r.plannerSuccess && ~r.validationPass
                r.outcome="INVALID_PATH";
            else
                r.outcome="PLANNER_FAIL";
            end
            r.planningTime_ms=t.planningTimeMs;
            r.firstSolutionTime_ms=t.firstSolutionTimeMs;
            r.jointPathLengthL2_rad=t.jointPathCostL2;
            r.totalJointMotion_rad=sum(t.totalJointMotionPerAxis,'omitnan');
            r.jointMotion1_rad=t.totalJointMotionPerAxis(1);
            r.jointMotion2_rad=t.totalJointMotionPerAxis(2);
            r.jointMotion3_rad=t.totalJointMotionPerAxis(3);
            r.jointMotion4_rad=t.totalJointMotionPerAxis(4);
            r.jointMotion5_rad=t.totalJointMotionPerAxis(5);
            r.jointMotion6_rad=t.totalJointMotionPerAxis(6);
            r.nodeCount=t.nodeCount;
            r.edgeCollisionChecks=t.edgeCollisionChecks;
            r.collisionConfigurationChecks=t.collisionConfigurationChecks;
            r.pathStateCount=t.pathStateCount;
            r.validationEdgeCollisionChecks=validation.edgeChecks;
            r.validationConfigurationChecks=validation.configurationChecks;
            r.plannerResolutionDeg=options.plannerResolutionDeg;
            r.validationResolutionDeg=options.validationResolutionDeg;
            rows(runIndex)=r;
            paths{runIndex}=t.path;
            metadata{runIndex}=struct('taskID',r.taskID,'algorithm',r.algorithm, ...
                'seed',seed,'effectiveSeed',effectiveSeed, ...
                'plannerResolutionDeg',r.plannerResolutionDeg, ...
                'validationResolutionDeg',r.validationResolutionDeg, ...
                'generatedAt',createdAt);
            fprintf('  seed=%02d %-24s outcome=%-14s time=%8.1f ms Lq=%9.4f checks=%d/%d\n', ...
                seed,t.algorithm,r.outcome,r.planningTime_ms, ...
                r.jointPathLengthL2_rad,r.edgeCollisionChecks, ...
                r.collisionConfigurationChecks);
            if options.saveResult
                checkpoint=makeReport(createdAt,options,source,tasks,algorithms,rows,paths,metadata,runIndex);
                checkpoint.note='Per-run checkpoint; fixed historical approach/exit task definition.';
                save(options.checkpointFile,'checkpoint','-v7.3');
            end
        end
    end
end

summary=makeSummary(rows,tasks,algorithms);
report=makeReport(createdAt,options,source,tasks,algorithms,rows,paths,metadata,runIndex);
report.summary=summary;
report.validSuccessCount=sum([rows.success]);
report.successRate=report.validSuccessCount/max(1,nRuns);
report.note='Only qHome->qPreWeld and qPostWeld->qHome are replanned; local weld approach/exit links and the 80-point weld path remain fixed.';
if options.saveResult
    save(options.outputFile,'report','-v7.3');
    writetable(struct2table(rows),options.rawCsvFile);
    writetable(struct2table(summary),options.summaryCsvFile);
end
fprintf('\nPHASE12A_SEAM_PILOT_DONE valid=%d/%d rate=%.3f\n', ...
    report.validSuccessCount,nRuns,report.successRate);
end

function validation=validatePath(scene,path,task,resolutionRad)
validation=struct('pass',false,'endpointPass',false,'jointLimitPass',false, ...
    'collisionFree',false,'edgeChecks',0,'configurationChecks',0, ...
    'failureEdge',NaN,'failureSample',NaN);
if isempty(path) || size(path,2)~=6 || any(~isfinite(path(:))), return; end
validation.endpointPass=max(abs(path(1,:)-task.start))<=1e-8 && ...
    max(abs(path(end,:)-task.goal))<=1e-8;
validation.jointLimitPass=all(arrayfun(@(i)jointWithinLimits(path(i,:),scene.jointLimits),1:size(path,1)));
if ~(validation.endpointPass && validation.jointLimitPass), return; end
opts=struct(); opts.wrapJoints=true; opts.selfCollisionSkipPairs=scene.selfCollisionSkipPairs;
validation.collisionFree=true;
for i=1:size(path,1)-1
    validation.edgeChecks=validation.edgeChecks+1;
    [free,idx,count]=collisionFreeJointEdge(scene.robot,path(i,:),path(i+1,:), ...
        scene.environment,resolutionRad,opts);
    validation.configurationChecks=validation.configurationChecks+count;
    if ~free
        validation.collisionFree=false;
        validation.failureEdge=i; validation.failureSample=idx;
        break;
    end
end
validation.pass=validation.endpointPass && validation.jointLimitPass && validation.collisionFree;
end

function tasks=makeTask(taskID,description,start,goal)
tasks=struct('taskID',taskID,'description',description,'start',start,'goal',goal);
end

function row=emptyRow()
row=struct('taskID',"",'description',"",'algorithm',"",'seed',NaN,'effectiveSeed',NaN, ...
    'plannerSuccess',false,'validationPass',false,'success',false,'plannerTimedOut',false, ...
    'outcome',"NOT_RUN",'planningTime_ms',NaN,'firstSolutionTime_ms',NaN, ...
    'jointPathLengthL2_rad',NaN,'totalJointMotion_rad',NaN, ...
    'jointMotion1_rad',NaN,'jointMotion2_rad',NaN,'jointMotion3_rad',NaN, ...
    'jointMotion4_rad',NaN,'jointMotion5_rad',NaN,'jointMotion6_rad',NaN, ...
    'nodeCount',0,'edgeCollisionChecks',0,'collisionConfigurationChecks',0, ...
    'pathStateCount',0,'validationEdgeCollisionChecks',0, ...
    'validationConfigurationChecks',0,'plannerResolutionDeg',NaN, ...
    'validationResolutionDeg',NaN);
end

function summary=makeSummary(rows,tasks,algorithms)
summary=repmat(struct('taskID',"",'algorithm',"",'seedCount',0, ...
    'successCount',0,'successRate',NaN,'timeoutCount',0, ...
    'planningTimeMedian_ms',NaN,'planningTimeP90_ms',NaN, ...
    'planningTimeIQR_ms',NaN,'jointPathLengthMedian_rad',NaN, ...
    'totalJointMotionMedian_rad',NaN,'collisionConfigurationChecksMedian',NaN, ...
    'nodeCountMedian',NaN),numel(tasks)*numel(algorithms),1);
idx=0;
for ti=1:numel(tasks)
    for ai=1:numel(algorithms)
        idx=idx+1;
        subset=rows([rows.taskID]==string(tasks(ti).taskID) & ...
            [rows.algorithm]==algorithmLabel(algorithms{ai}));
        summary(idx).taskID=string(tasks(ti).taskID);
        summary(idx).algorithm=algorithmLabel(algorithms{ai});
        summary(idx).seedCount=numel(subset);
        summary(idx).successCount=sum([subset.success]);
        summary(idx).successRate=mean([subset.success]);
        summary(idx).timeoutCount=sum([subset.plannerTimedOut]);
        plan=finite([subset.planningTime_ms]);
        paths=finite([subset.jointPathLengthL2_rad]);
        motions=finite([subset.totalJointMotion_rad]);
        summary(idx).planningTimeMedian_ms=medianOrNaN(plan);
        summary(idx).planningTimeP90_ms=percentileOrNaN(plan,90);
        summary(idx).planningTimeIQR_ms=percentileOrNaN(plan,75)-percentileOrNaN(plan,25);
        summary(idx).jointPathLengthMedian_rad=medianOrNaN(paths);
        summary(idx).totalJointMotionMedian_rad=medianOrNaN(motions);
        summary(idx).collisionConfigurationChecksMedian=median([subset.collisionConfigurationChecks]);
        summary(idx).nodeCountMedian=median([subset.nodeCount]);
    end
end
end

function report=makeReport(createdAt,options,source,tasks,algorithms,rows,paths,metadata,completedRuns)
report=struct('phase',"Phase 12A Seam approach/exit",'createdAt',createdAt, ...
    'options',options,'source',source,'tasks',tasks,'algorithms',{algorithms}, ...
    'rows',rows,'paths',{paths},'pathMetadata',{metadata},'completedRuns',completedRuns);
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

function values=finite(values)
values=values(isfinite(values));
end

function value=medianOrNaN(values)
if isempty(values), value=NaN; else, value=median(values); end
end

function value=percentileOrNaN(values,p)
if isempty(values), value=NaN; return; end
values=sort(values(:)); position=1+(numel(values)-1)*p/100;
lo=floor(position); hi=ceil(position);
if lo==hi, value=values(lo); else, value=values(lo)+(position-lo)*(values(hi)-values(lo)); end
end

function out=mergeOptions(defaults,in)
out=defaults; names=fieldnames(defaults);
for i=1:numel(names)
    if isfield(in,names{i}) && ~isempty(in.(names{i})), out.(names{i})=in.(names{i}); end
end
end
