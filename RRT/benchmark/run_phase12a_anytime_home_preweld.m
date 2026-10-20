function report = run_phase12a_anytime_home_preweld(options)
%run_phase12a_anytime_home_preweld Compare first solution and best-so-far.
%
% This benchmark deliberately restores the historical Seam A task and
% collision conditions:
%   qHome -> qPreWeld
%   planner edge interpolation       : 2 deg
%   independent path validation      : 0.5 deg
%   joint steering step              : 12 deg
%   goal bias                        : 0.20
%
% RRT-Connect returns after its first certified solution.  The three RRT*
% variants continue until the common wall-clock budget and update their
% best path only after the common 0.5-degree validator accepts a candidate.
% The 0.5-degree value is kept fixed from the historical implementation;
% it is not an experimental factor in this benchmark.

if nargin < 1 || isempty(options), options=struct(); end
root=fileparts(fileparts(mfilename('fullpath')));
defaults=struct( ...
    'approachResult',fullfile(root,'results','phase12a_seam_legacy_rrtconnect_repro.mat'), ...
    'sceneScript','/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m', ...
    'plannerResolutionDeg',2.0, ...
    'validationResolutionDeg',0.5, ...
    'maxPlanningTimeSec',30.0, ...
    'maxIterations',1000000, ...
    'maxJointStepRad',deg2rad(12), ...
    'goalBias',0.20, ...
    'seed',1, ...
    'randomSeed',20260917, ...
    'outputFile',fullfile(root,'results','phase12a_anytime_home_preweld_seed1.mat'), ...
    'csvFile',fullfile(root,'results','phase12a_anytime_home_preweld_seed1.csv'), ...
    'figureFile',fullfile(root,'results','phase12a_anytime_home_preweld_seed1_time_quality.png'), ...
    'saveResult',true);
options=localMergeOptions(defaults,options);
if ~isfile(options.approachResult)
    error('Frozen approach/exit result not found: %s',options.approachResult);
end
if ~isscalar(options.seed) || options.seed<1 || options.seed~=floor(options.seed)
    error('seed must be a positive integer.');
end

addpath(genpath(root));
setenv('RRT_HEADLESS','1');
scene=loadUR5JointScene(options.sceneScript);
s=load(options.approachResult,'result'); source=s.result;
if ~isfield(source,'fullPass') || ~source.fullPass
    error('Frozen approach/exit result is not fullPass.');
end

task=struct('taskID',"T01_Safe_to_PreWeld", ...
    'description',"qHome(qSafe) -> qPreWeld", ...
    'start',double(source.qHome(:).'), ...
    'goal',double(source.preWeld.q(:).'));
algorithms={ ...
    'RRTConnectJoint', ...
    'BiRRTStarJoint', ...
    'ClassGoalLiteJoint', ...
    'ClassifiedRRTStarJoint'};
algorithmLabels=["RRT-Connect","Bi-RRT*","ClassGoal-Lite","Full Classified-RRT*"];
checkpoints=[5 10 15 20 30];
rows=repmat(emptyRow(),numel(algorithms),1);
paths=cell(numel(algorithms),1);
checkpointPaths=cell(numel(algorithms),1);
createdAt=string(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));

fprintf('\nPHASE 12A ANYTIME: Home -> PreWeld\n');
fprintf('algorithms=%d, budget=%.1f s, checkpoints=[%s]\n', ...
    numel(algorithms),options.maxPlanningTimeSec,sprintf(' %.1f',checkpoints));
fprintf('historical planner edge=%.1f deg, validation edge=%.1f deg, step=%.1f deg\n', ...
    options.plannerResolutionDeg,options.validationResolutionDeg,rad2deg(options.maxJointStepRad));

for ai=1:numel(algorithms)
    isRRTConnect=algorithmLabels(ai)=="RRT-Connect";
    effectiveSeed=options.randomSeed+options.seed-1;
    cfg=struct( ...
        'scene',scene,'start',task.start,'goal',task.goal, ...
        'maxIterations',options.maxIterations, ...
        'maxJointStepRad',options.maxJointStepRad, ...
        'jointEdgeResolutionRad',deg2rad(options.plannerResolutionDeg), ...
        'goalBias',options.goalBias,'seed',effectiveSeed, ...
        'maxPlanningTimeSec',options.maxPlanningTimeSec, ...
        'anytime',true, ...
        'continueAfterFirstSolution',~isRRTConnect, ...
        'checkpointTimesSec',checkpoints, ...
        'candidateValidatorFcn',@(path,abortFcn)strictValidatePath( ...
            scene,path,task,deg2rad(options.validationResolutionDeg),abortFcn));

    t=feval(algorithms{ai},cfg);
    r=emptyRow();
    r.taskID=task.taskID;
    r.description=task.description;
    r.algorithm=algorithmLabels(ai);
    r.seed=options.seed;
    r.effectiveSeed=effectiveSeed;
    r.plannerResolutionDeg=options.plannerResolutionDeg;
    r.validationResolutionDeg=options.validationResolutionDeg;
    r.maxPlanningTimeSec=options.maxPlanningTimeSec;
    r.success=logical(t.success);
    r.certifiedSuccess=logical(t.certifiedSuccess);
    r.outcome=string(t.terminationReason);
    r.planningTime_ms=t.planningTimeMs;
    r.firstSolutionTime_ms=t.firstSolutionTimeMs;
    r.firstPathLengthL2_rad=t.firstPathCostL2;
    r.finalBestPathLengthL2_rad=t.bestPathCostL2;
    r.nodeCount=t.nodeCount;
    r.edgeCollisionChecks=t.edgeCollisionChecks;
    r.collisionConfigurationChecks=t.collisionConfigurationChecks;
    r.candidateCount=t.candidateCount;
    r.candidateValidationFailures=t.candidateValidationFailures;
    r.stateCacheHits=t.stateCacheHits;
    r.edgeCacheHits=t.edgeCacheHits;
    r.bestPathLengthAt5s=t.bestPathLengthAtCheckpoint(1);
    r.bestPathLengthAt10s=t.bestPathLengthAtCheckpoint(2);
    r.bestPathLengthAt15s=t.bestPathLengthAtCheckpoint(3);
    r.bestPathLengthAt20s=t.bestPathLengthAtCheckpoint(4);
    r.bestPathLengthAt30s=t.bestPathLengthAtCheckpoint(5);
    if t.success
        r.totalJointMotion_rad=sum(t.totalJointMotionPerAxis,'omitnan');
    end
    rows(ai)=r;
    paths{ai}=t.path;
    checkpointPaths{ai}=t.bestPathAtCheckpoint;

    fprintf('  %-24s first=%8.1f ms Lfirst=%9.4f L30=%9.4f outcome=%s\n', ...
        char(r.algorithm),r.firstSolutionTime_ms,r.firstPathLengthL2_rad, ...
        r.bestPathLengthAt30s,char(r.outcome));
end

report=struct('phase',"Phase 12A anytime Home->PreWeld", ...
    'createdAt',createdAt,'options',options,'task',task, ...
    'algorithms',{algorithms},'algorithmLabels',algorithmLabels, ...
    'checkpointTimesSec',checkpoints,'rows',rows,'paths',{paths}, ...
    'checkpointPaths',{checkpointPaths},'source',source, ...
    'figureFile',string(options.figureFile), ...
    'note',"Original Seam A collision conditions fixed; only planner variant changes.");

if options.saveResult
    makeTimeQualityFigure(rows,checkpoints,options.figureFile);
end

if options.saveResult
    save(options.outputFile,'report','-v7.3');
    writetable(struct2table(rows),options.csvFile);
end
fprintf('ANYTIME_DONE output=%s\n',options.outputFile);
end

function [pass,detail]=strictValidatePath(scene,path,task,resolutionRad,abortFcn)
detail=struct('collisionConfigurationChecks',0,'edgeCollisionChecks',0, ...
    'failureEdgeIndex',NaN,'failureSampleIndex',NaN, ...
    'failureCollisionType',"",'timedOut',false);
pass=false;
if abortFcn(), detail.timedOut=true; return; end
if isempty(path) || size(path,2)~=6 || any(~isfinite(path(:))), return; end
if max(abs(path(1,:)-task.start))>1e-8 || max(abs(path(end,:)-task.goal))>1e-8, return; end
if any(~arrayfun(@(i)jointWithinLimits(path(i,:),scene.jointLimits),1:size(path,1))), return; end

for i=1:size(path,1)
    if abortFcn(), detail.timedOut=true; return; end
    [free,stateDetail]=collisionFreeJointState(scene.robot,path(i,:), ...
        scene.environment,scene.selfCollisionSkipPairs);
    detail.collisionConfigurationChecks=detail.collisionConfigurationChecks+1;
    if ~free
        detail.failureSampleIndex=i;
        if stateDetail.environmentCollision, detail.failureCollisionType="environment";
        elseif stateDetail.selfCollision, detail.failureCollisionType="self";
        end
        return;
    end
end

for i=1:size(path,1)-1
    if abortFcn(), detail.timedOut=true; return; end
    opts=struct();
    opts.wrapJoints=true;
    opts.selfCollisionSkipPairs=scene.selfCollisionSkipPairs;
    opts.timeoutFcn=abortFcn;
    [free,idx,count,timedOut]=collisionFreeJointEdge(scene.robot,path(i,:), ...
        path(i+1,:),scene.environment,resolutionRad,opts);
    detail.edgeCollisionChecks=detail.edgeCollisionChecks+1;
    detail.collisionConfigurationChecks=detail.collisionConfigurationChecks+count;
    if timedOut || abortFcn(), detail.timedOut=true; return; end
    if ~free
        detail.failureEdgeIndex=i;
        detail.failureSampleIndex=idx;
        detail.failureCollisionType="edge";
        return;
    end
end
pass=true;
end

function row=emptyRow()
row=struct('taskID',"",'description',"",'algorithm',"", ...
    'seed',NaN,'effectiveSeed',NaN,'plannerResolutionDeg',NaN, ...
    'validationResolutionDeg',NaN,'maxPlanningTimeSec',NaN, ...
    'success',false,'certifiedSuccess',false,'outcome',"", ...
    'planningTime_ms',NaN,'firstSolutionTime_ms',NaN, ...
    'firstPathLengthL2_rad',NaN,'finalBestPathLengthL2_rad',NaN, ...
    'bestPathLengthAt5s',NaN,'bestPathLengthAt10s',NaN, ...
    'bestPathLengthAt15s',NaN,'bestPathLengthAt20s',NaN, ...
    'bestPathLengthAt30s',NaN,'totalJointMotion_rad',NaN, ...
    'nodeCount',0,'edgeCollisionChecks',0,'collisionConfigurationChecks',0, ...
    'candidateCount',0,'candidateValidationFailures',0, ...
    'stateCacheHits',0,'edgeCacheHits',0);
end

function out=localMergeOptions(defaults,in)
out=defaults;
names=fieldnames(defaults);
for i=1:numel(names)
    name=names{i};
    if isfield(in,name) && ~isempty(in.(name)), out.(name)=in.(name); end
end
end

function makeTimeQualityFigure(rows,checkpoints,figureFile)
fig=figure('Visible','off','Color','w','Position',[100 100 900 560]);
ax=axes(fig); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
colors=lines(numel(rows));
for i=1:numel(rows)
    firstSec=rows(i).firstSolutionTime_ms/1000;
    if ~isfinite(firstSec) || ~isfinite(rows(i).firstPathLengthL2_rad), continue; end
    cp=[rows(i).bestPathLengthAt5s,rows(i).bestPathLengthAt10s, ...
        rows(i).bestPathLengthAt15s,rows(i).bestPathLengthAt20s, ...
        rows(i).bestPathLengthAt30s];
    keep=isfinite(cp) & checkpoints>=firstSec;
    tx=[firstSec,checkpoints(keep)];
    ly=[rows(i).firstPathLengthL2_rad,cp(keep)];
    plot(ax,tx,ly,'-o','LineWidth',1.8,'MarkerSize',5, ...
        'Color',colors(i,:),'DisplayName',char(rows(i).algorithm));
end
xlabel(ax,'Planning time (s)');
ylabel(ax,'Best certified joint-space path length (rad)');
title(ax,'Home to PreWeld: time-quality trade-off');
legend(ax,'Location','best');
xlim(ax,[0 max([30,checkpoints])]);
if ~isempty(figureFile)
    exportgraphics(fig,figureFile,'Resolution',150);
end
close(fig);
end
