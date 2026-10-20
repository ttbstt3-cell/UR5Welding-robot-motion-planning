function report = run_v1_fast_connect_anytime_taskA(options)
%run_v1_fast_connect_anytime_taskA Validate V1 on Seam A Task A.
%
% Task A is the frozen free-space transition qHome -> qPreWeld.  The three
% compared planners use the historical 2-degree search edge rule and receive
% the historical 0.5-degree dense validation after planning.  V1 itself does
% not add candidate certification during the search.

if nargin < 1 || isempty(options), options=struct(); end
root=fileparts(fileparts(mfilename('fullpath')));
defaults=struct( ...
    'approachResult',fullfile(root,'results','phase12a_seam_legacy_rrtconnect_repro.mat'), ...
    'sceneScript','/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m', ...
    'plannerResolutionDeg',2.0,'validationResolutionDeg',0.5, ...
    'maxPlanningTimeSec',30.0,'maxIterations',1000000, ...
    'maxJointStepRad',deg2rad(12),'goalBias',0.20, ...
    'seed',20260917,'checkpointTimesSec',[5 10 15 20 30], ...
    'task','A', ...
    'outputFile',fullfile(root,'results','v1_fast_connect_anytime_taskA_seed20260917.mat'), ...
    'csvFile',fullfile(root,'results','v1_fast_connect_anytime_taskA_seed20260917.csv'), ...
    'eventsCsvFile',fullfile(root,'results','v1_fast_connect_anytime_taskA_seed20260917_improvement_events.csv'), ...
    'figureFile',fullfile(root,'results','v1_fast_connect_anytime_taskA_time_quality.png'), ...
    'saveResult',true);
options=mergeOptions(defaults,options);

if ~isfile(options.approachResult)
    error('Frozen approach result not found: %s',options.approachResult);
end
addpath(genpath(root));
setenv('RRT_HEADLESS','1');
scene=loadUR5JointScene(options.sceneScript);
s=load(options.approachResult,'result'); source=s.result;
if ~isfield(source,'fullPass') || ~source.fullPass
    error('Frozen approach result is not fullPass.');
end

if strcmpi(char(string(options.task)),'B')
    task=struct('taskID',"T02_PostWeld_to_Safe", ...
        'description',"qPostWeld -> qHome(qSafe)", ...
        'start',double(source.postWeld.q(:).'),'goal',double(source.qHome(:).'));
else
    task=struct('taskID',"T01_Safe_to_PreWeld", ...
        'description',"qHome(qSafe) -> qPreWeld", ...
        'start',double(source.qHome(:).'),'goal',double(source.preWeld.q(:).'));
end

algorithms={'RRTConnectJoint','BiRRTStarJoint', ...
    'FastConnectAnytimeRRTStarJoint'};
labels=["RRT-Connect","Bi-RRT*","V1 FastConnectAnytimeRRTStar"];
rows=repmat(emptyRow(),numel(algorithms),1);
paths=cell(numel(algorithms),1);
checkpointPaths=cell(numel(algorithms),1);
v1Events=struct('eventTimeSec',{},'bestPathLengthL2Rad',{}, ...
    'nodeCount',{},'edgeCollisionChecks',{}, ...
    'collisionConfigurationChecks',{},'rewireAttempts',{}, ...
    'rewireSuccesses',{});

fprintf('\nV1 FUNCTIONAL VALIDATION: %s\n',char(task.description));
fprintf('seed=%d budget=%.1f s search=%.1f deg validation=%.1f deg\n', ...
    options.seed,options.maxPlanningTimeSec,options.plannerResolutionDeg, ...
    options.validationResolutionDeg);

for ai=1:numel(algorithms)
    cfg=struct('scene',scene,'start',task.start,'goal',task.goal, ...
        'maxIterations',options.maxIterations, ...
        'maxJointStepRad',options.maxJointStepRad, ...
        'jointEdgeResolutionRad',deg2rad(options.plannerResolutionDeg), ...
        'goalBias',options.goalBias,'seed',options.seed, ...
        'maxPlanningTimeSec',options.maxPlanningTimeSec, ...
        'anytime',true,'continueAfterFirstSolution',ai>1, ...
        'checkpointTimesSec',options.checkpointTimesSec, ...
        'candidateValidatorFcn',[]);
    fprintf('  running %s ...\n',labels(ai));
    t=feval(algorithms{ai},cfg);
    if t.success
        validation=validateDensePath(scene,t.path,task, ...
            deg2rad(options.validationResolutionDeg));
    else
        validation=emptyValidation();
    end
    rows(ai)=makeRow(t,labels(ai),task,options,validation);
    paths{ai}=t.path;
    checkpointPaths{ai}=t.bestPathAtCheckpoint;
    if ai==3, v1Events=t.improvementEvents; end
    fprintf('    success=%d densePASS=%d Tfirst=%.3f s Lfirst=%.6f L30=%.6f events=%d\n', ...
        rows(ai).plannerSuccess,rows(ai).finalDenseValidationPass, ...
        rows(ai).firstSolutionTimeSec,rows(ai).firstPathLengthL2Rad, ...
        rows(ai).bestPathLengthAt30s,rows(ai).numberOfImprovements);
end

makeTimeQualityFigure(rows,options.checkpointTimesSec,options.figureFile,v1Events);
report=struct('phase',"V1 FastConnectAnytimeRRTStar "+string(task.taskID), ...
    'createdAt',string(datetime('now','Format','yyyy-MM-dd HH:mm:ss')), ...
    'task',task,'options',options,'algorithms',{algorithms}, ...
    'algorithmLabels',labels,'rows',rows,'paths',{paths}, ...
    'checkpointPaths',{checkpointPaths},'v1ImprovementEvents',v1Events, ...
    'improvementEventsCsvFile',string(options.eventsCsvFile), ...
    'source',source,'figureFile',string(options.figureFile), ...
    'note',"V1 only: RRT-Connect first connection followed by dual-tree RRT* optimization.");
if options.saveResult
    save(options.outputFile,'report','-v7.3');
    writetable(struct2table(rows),options.csvFile);
    if isempty(v1Events)
        writetable(table(),options.eventsCsvFile);
    else
        writetable(struct2table(v1Events),options.eventsCsvFile);
    end
end
fprintf('V1_DONE output=%s\n',options.outputFile);
disp(struct2table(rows));
end

function row=makeRow(t,label,task,options,validation)
row=emptyRow();
row.taskID=task.taskID; row.description=task.description;
row.algorithm=label; row.seed=options.seed;
row.plannerResolutionDeg=options.plannerResolutionDeg;
row.validationResolutionDeg=options.validationResolutionDeg;
row.maxPlanningTimeSec=options.maxPlanningTimeSec;
row.plannerSuccess=logical(t.success);
row.finalDenseValidationPass=logical(validation.pass);
row.validSuccess=row.plannerSuccess && row.finalDenseValidationPass;
row.terminationReason=string(t.terminationReason);
row.planningTimeSec=t.planningTimeSec;
row.firstSolutionTimeSec=t.firstSolutionTimeSec;
row.firstPathLengthL2Rad=getFieldOr(t,'firstPathLengthL2Rad', ...
    getFieldOr(t,'firstPathCostL2',NaN));
row.finalBestPathLengthL2Rad=getFieldOr(t,'finalBestPathLengthL2Rad', ...
    getFieldOr(t,'bestPathCostL2',NaN));
checkpoint=getFieldOr(t,'bestPathLengthAtCheckpoint',NaN(1,5));
row.bestPathLengthAt5s=checkpoint(1);
row.bestPathLengthAt10s=checkpoint(2);
row.bestPathLengthAt15s=checkpoint(3);
row.bestPathLengthAt20s=checkpoint(4);
row.bestPathLengthAt30s=checkpoint(5);
row.totalJointMotionPerAxis=getFieldOr(t,'totalJointMotionPerAxis',NaN(1,6));
row.totalJointMotionRad=getFieldOr(t,'totalJointMotionRad', ...
    sum(row.totalJointMotionPerAxis,'omitnan'));
row.nodeCount=t.nodeCount;
row.edgeCollisionChecks=t.edgeCollisionChecks;
row.collisionConfigurationChecks=t.collisionConfigurationChecks;
row.stage1Iterations=getFieldOr(t,'stage1Iterations',NaN);
row.stage2Iterations=getFieldOr(t,'stage2Iterations',NaN);
row.stage1CollisionChecks=getFieldOr(t,'stage1CollisionChecks',NaN);
row.stage2CollisionChecks=getFieldOr(t,'stage2CollisionChecks',NaN);
row.rewireAttempts=getFieldOr(t,'rewireAttempts',NaN);
row.rewireSuccesses=getFieldOr(t,'rewireSuccesses',NaN);
row.numberOfImprovements=getFieldOr(t,'numberOfImprovements',NaN);
row.denseValidationCollisionConfigurationChecks=validation.collisionConfigurationChecks;
row.denseValidationEdgeChecks=validation.edgeCollisionChecks;
end

function row=emptyRow()
row=struct('taskID',"",'description',"",'algorithm',"",'seed',NaN, ...
    'plannerResolutionDeg',NaN,'validationResolutionDeg',NaN, ...
    'maxPlanningTimeSec',NaN,'plannerSuccess',false, ...
    'finalDenseValidationPass',false,'validSuccess',false, ...
    'terminationReason',"",'planningTimeSec',NaN, ...
    'firstSolutionTimeSec',NaN,'firstPathLengthL2Rad',NaN, ...
    'finalBestPathLengthL2Rad',NaN,'bestPathLengthAt5s',NaN, ...
    'bestPathLengthAt10s',NaN,'bestPathLengthAt15s',NaN, ...
    'bestPathLengthAt20s',NaN,'bestPathLengthAt30s',NaN, ...
    'totalJointMotionRad',NaN,'totalJointMotionPerAxis',NaN(1,6), ...
    'nodeCount',0,'edgeCollisionChecks',0, ...
    'collisionConfigurationChecks',0,'stage1Iterations',NaN, ...
    'stage2Iterations',NaN,'stage1CollisionChecks',NaN, ...
    'stage2CollisionChecks',NaN,'rewireAttempts',NaN, ...
    'rewireSuccesses',NaN,'numberOfImprovements',NaN, ...
    'denseValidationCollisionConfigurationChecks',0, ...
    'denseValidationEdgeChecks',0);
end

function value=getFieldOr(s,name,defaultValue)
if isfield(s,name), value=s.(name); else, value=defaultValue; end
end

function v=emptyValidation()
v=struct('pass',false,'collisionConfigurationChecks',0,'edgeCollisionChecks',0);
end

function v=validateDensePath(scene,path,task,resolutionRad)
v=emptyValidation();
if isempty(path) || size(path,2)~=6 || any(~isfinite(path(:))), return; end
if max(abs(path(1,:)-task.start))>1e-8 || max(abs(path(end,:)-task.goal))>1e-8, return; end
if any(~arrayfun(@(i)jointWithinLimits(path(i,:),scene.jointLimits),1:size(path,1))), return; end
for i=1:size(path,1)
    [free,~]=collisionFreeJointState(scene.robot,path(i,:), ...
        scene.environment,scene.selfCollisionSkipPairs);
    v.collisionConfigurationChecks=v.collisionConfigurationChecks+1;
    if ~free, return; end
end
for i=1:size(path,1)-1
    opts=struct();
    opts.wrapJoints=true;
    opts.selfCollisionSkipPairs=scene.selfCollisionSkipPairs;
    [free,~,count,timedOut]=collisionFreeJointEdge(scene.robot,path(i,:), ...
        path(i+1,:),scene.environment,resolutionRad,opts);
    v.edgeCollisionChecks=v.edgeCollisionChecks+1;
    v.collisionConfigurationChecks=v.collisionConfigurationChecks+count;
    if timedOut || ~free, return; end
end
v.pass=true;
end

function makeTimeQualityFigure(rows,checkpoints,figureFile,v1Events)
fig=figure('Visible','off','Color','w','Position',[100 100 980 600]);
ax=axes(fig); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
colors=[0.15 0.15 0.15; 0.00 0.45 0.74; 0.85 0.33 0.10];
for i=1:numel(rows)
    t0=rows(i).firstSolutionTimeSec; y0=rows(i).firstPathLengthL2Rad;
    if ~isfinite(t0) || ~isfinite(y0), continue; end
    cp=[rows(i).bestPathLengthAt5s,rows(i).bestPathLengthAt10s, ...
        rows(i).bestPathLengthAt15s,rows(i).bestPathLengthAt20s, ...
        rows(i).bestPathLengthAt30s];
    keep=isfinite(cp) & checkpoints>=t0;
    tx=[t0,checkpoints(keep)]; ly=[y0,cp(keep)];
    if i==3 && ~isempty(v1Events)
        % V1 has event-level logging; use every strict improvement rather
        % than reducing its curve to the five checkpoint values.
        tx=[reshape([v1Events.eventTimeSec],1,[]),30];
        ly=[reshape([v1Events.bestPathLengthL2Rad],1,[]), ...
            v1Events(end).bestPathLengthL2Rad];
        stairs(ax,tx,ly,'LineWidth',2.0,'Color',colors(i,:), ...
            'DisplayName',char(rows(i).algorithm));
        plot(ax,tx(1:end-1),ly(1:end-1),'o','Color',colors(i,:), ...
            'MarkerFaceColor',colors(i,:),'HandleVisibility','off');
        continue;
    end
    stairs(ax,tx,ly,'LineWidth',2.0,'Color',colors(i,:), ...
        'DisplayName',char(rows(i).algorithm));
    plot(ax,tx,ly,'o','Color',colors(i,:),'MarkerFaceColor',colors(i,:), ...
        'HandleVisibility','off');
end
xlabel(ax,'Planning time (s)'); ylabel(ax,'Best joint-space path length L_q (rad)');
title(ax,'V1 Fast-Connect Anytime RRT*: Task A time-quality trade-off');
legend(ax,'Location','best'); xlim(ax,[0 max([checkpoints 30])]);
if ~isempty(figureFile), exportgraphics(fig,figureFile,'Resolution',150); end
close(fig);
end

function out=mergeOptions(defaults,in)
out=defaults; names=fieldnames(in);
for i=1:numel(names), out.(names{i})=in.(names{i}); end
end
