function report = run_phase12a_pilot(options)
%run_phase12a_pilot Phase 12A-2: 6D speed and joint-path-length pilot.
%
% Runs 6 cases x 4 algorithms x 20 matched seeds. Planning uses 0.5 degree
% discrete joint-edge checking; every successful path is independently
% validated at 0.25 degree. No trajectory timing or execution model is used.

if nargin < 1 || isempty(options), options=struct(); end
here=fileparts(mfilename('fullpath'));
root=fileparts(here);
defaults=struct('sceneScript','/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m', ...
    'approachResult','/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_approach_exit.mat', ...
    'outputFile',fullfile(root,'results','phase12a_pilot_480.mat'), ...
    'rawCsvFile',fullfile(root,'results','phase12a_pilot_480_runs.csv'), ...
    'summaryCsvFile',fullfile(root,'results','phase12a_pilot_480_summary.csv'), ...
    'figureDir',fullfile(root,'results'), ...
    'checkpointFile','', ...
    'taskSet','all6','maxPlanningTimeSec',Inf, ...
    'maxIterations',300,'maxJointStepRad',deg2rad(25),'goalBias',0.20, ...
    'jointEdgeResolutionRad',deg2rad(0.5),'validationResolutionRad',deg2rad(0.25), ...
    'seeds',1:20,'bootstrapReplicates',1000,'saveResult',true, ...
    'caseIndices',[],'headless',true);
options=mergeOptions(defaults,options);
if isempty(options.checkpointFile)
    options.checkpointFile=[options.outputFile '.checkpoint.mat'];
end
addpath(genpath(root));

% Benchmark runs should not create the UR5 scene figure.  The scene builder
% still supports an explicit RRT_SHOW_SCENE=1 override for manual inspection.
if options.headless
    setenv('RRT_HEADLESS','1');
end

scene=loadUR5JointScene(options.sceneScript);
s=load(options.approachResult,'result');
source=s.result;
if ~source.fullPass || isempty(source.qWeld) || size(source.qWeld,2)~=6
    error('The current approach/exit result is not a valid full-pass source.');
end
qHome=double(source.qHome(:).');
qWeld=unwrap(double(source.qWeld),[],1);
qPre=double(source.preWeld.q(:).');
qPost=double(source.postWeld.q(:).');
qStart=qWeld(1,:); qEnd=qWeld(end,:);
cases=makeCases(qHome,qPre,qPost,qStart,qEnd,options.taskSet);
if ~isempty(options.caseIndices)
    cases=cases(options.caseIndices);
end
algorithms={'RRTConnectJoint','BiRRTStarJoint','ClassGoalLiteJoint','ClassifiedRRTStarJoint'};
nRuns=numel(cases)*numel(algorithms)*numel(options.seeds);
rows=repmat(emptyRow(),nRuns,1);
paths=cell(nRuns,1);
pathMetadata=cell(nRuns,1);
createdAt=string(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
runIndex=0;

fprintf('\nPHASE 12A-2 PILOT: %d cases x %d algorithms x %d seeds = %d runs\n', ...
    numel(cases),numel(algorithms),numel(options.seeds),nRuns);
for c=1:numel(cases)
    fprintf('\n[%s] %s\n',cases(c).caseID,cases(c).description);
    for si=1:numel(options.seeds)
        seed=options.seeds(si);
        for a=1:numel(algorithms)
            runIndex=runIndex+1;
            cfg=struct('scene',scene,'start',cases(c).start,'goal',cases(c).goal, ...
                'maxIterations',options.maxIterations,'maxJointStepRad',options.maxJointStepRad, ...
                'jointEdgeResolutionRad',options.jointEdgeResolutionRad, ...
                'goalBias',options.goalBias,'seed',seed, ...
                'maxPlanningTimeSec',options.maxPlanningTimeSec);
            t=feval(algorithms{a},cfg);
            validation=validatePilotPath(t.path,cases(c),scene,options.validationResolutionRad);
            r=emptyRow();
            r.caseID=string(cases(c).caseID); r.description=string(cases(c).description);
            r.algorithm=string(t.algorithm); r.seed=seed;
            r.success=t.success && ~t.timedOut;
            r.plannerPathValid=r.success;
            r.plannerTimedOut=t.timedOut;
            r.finalValidationPass=r.success && validation.pass;
            if t.timedOut
                r.outcome="TIMEOUT";
            elseif ~t.success
                r.outcome="PLANNER_FAIL";
            elseif ~validation.pass
                r.outcome="INVALID_PATH";
            else
                r.outcome="VALID_SUCCESS";
            end
            r.planningTime_ms=t.planningTimeMs;
            r.firstSolutionTime_ms=t.firstSolutionTimeMs;
            r.jointPathLengthL2_rad=t.jointPathCostL2;
            r.jointPathLengthL2_deg=rad2deg(t.jointPathCostL2);
            r.totalJointMotion_rad=sum(t.totalJointMotionPerAxis,'omitnan');
            r.totalJointMotion_deg=rad2deg(r.totalJointMotion_rad);
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
            r.plannerResolutionDeg=rad2deg(options.jointEdgeResolutionRad);
            r.validationResolutionDeg=rad2deg(options.validationResolutionRad);
            r.validationConfigurationChecks=validation.collisionConfigurationChecks;
            r.validationEdgeCollisionChecks=validation.edgeCollisionChecks;
            r.validationFailureEdge=validation.failureEdgeIndex;
            r.validationFailureSample=validation.failureSampleIndex;
            r.validationCollisionType=validation.failureCollisionType;
            rows(runIndex)=r;
            paths{runIndex}=t.path;
            pathMetadata{runIndex}=struct('caseID',r.caseID,'algorithm',r.algorithm, ...
                'seed',seed,'plannerResolutionDeg',r.plannerResolutionDeg, ...
                'validationResolutionDeg',r.validationResolutionDeg, ...
                'generatedAt',createdAt);
            fprintf('  seed=%02d %-24s success=%d final025=%d time=%.1f ms Lq=%.5f checks=%d/%d\n', ...
                seed,t.algorithm,t.success,validation.pass,t.planningTimeMs, ...
                t.jointPathCostL2,t.edgeCollisionChecks,t.collisionConfigurationChecks);
            if options.saveResult
                checkpoint=makeReport(createdAt,options,cases,algorithms,rows,paths,pathMetadata,runIndex);
                checkpoint.note='Per-run checkpoint; final report is written only after all runs complete.';
                save(options.checkpointFile,'checkpoint','-v7.3');
            end
        end
    end
end

rows=assignRelativePathLength(rows);
summary=aggregateRows(rows,cases,algorithms,options.bootstrapReplicates);
report=makeReport(createdAt,options,cases,algorithms,rows,paths,pathMetadata,runIndex);
report.summary=summary;
report.allSuccessful=all([rows.success]);
report.allFinalValidated=all([rows.finalValidationPass]);
report.note='Planning speed and joint-space path length only; no trajectory timing, ROS2, or CoppeliaSim.';
if options.saveResult
    save(options.outputFile,'report','-v7.3');
    writeRunRows(rows,options.rawCsvFile);
    writeSummaryRows(summary,options.summaryCsvFile);
    makeCoreFigures(rows,algorithms,options.figureDir);
end
fprintf('\nPHASE12A2_DONE success=%d/%d final025=%d/%d\n', ...
    sum([rows.success]),nRuns,sum([rows.finalValidationPass]),nRuns);
end

function cases=makeCases(qHome,qPre,qPost,qStart,qEnd,taskSet)
if nargin<6 || isempty(taskSet), taskSet='all6'; end
cases=repmat(struct('caseID','','description','','start',zeros(1,6),'goal',zeros(1,6)),0,1);
if strcmpi(taskSet,'seamA')
    % The authoritative UR5 handoff defines qHome as the current Seam A
    % welding-safe point.  Point1--Point80 remains the fixed weld path.
    cases(end+1)=makeCase('T01_Safe_to_Point1', ...
        'Seam A entry: qSafe(qHome) -> Point 1',qHome,qStart);
    cases(end+1)=makeCase('T02_Point80_to_Safe', ...
        'Seam A exit: Point 80 -> qSafe(qHome)',qEnd,qHome);
    return;
end
cases(end+1)=makeCase('C01_Home_to_PreWeld','real approach transition',qHome,qPre);
cases(end+1)=makeCase('C02_PostWeld_to_Home','real exit transition',qPost,qHome);
cases(end+1)=makeCase('C03_PreWeld_to_Point1','real weld-entry transition',qPre,qStart);
cases(end+1)=makeCase('C04_Point80_to_PostWeld','real weld-exit transition',qEnd,qPost);
cases(end+1)=makeCase('C05_Home_to_Point1','diagnostic near-workpiece/tool case',qHome,qStart);
qSimple=qHome; qSimple(1)=qSimple(1)+deg2rad(5);
cases(end+1)=makeCase('C06_Simple_5deg','simple local free-space case',qHome,qSimple);
end

function c=makeCase(id,description,start,goal)
c=struct('caseID',id,'description',description,'start',start,'goal',goal);
end

function row=emptyRow()
row=struct('caseID',"",'description',"",'algorithm',"",'seed',NaN, ...
    'success',false,'plannerPathValid',false,'finalValidationPass',false, ...
    'plannerTimedOut',false,'outcome',"NOT_RUN", ...
    'planningTime_ms',NaN,'firstSolutionTime_ms',NaN, ...
    'jointPathLengthL2_rad',NaN,'jointPathLengthL2_deg',NaN, ...
    'relativePathLength',NaN,'totalJointMotion_rad',NaN,'totalJointMotion_deg',NaN, ...
    'jointMotion1_rad',NaN,'jointMotion2_rad',NaN,'jointMotion3_rad',NaN, ...
    'jointMotion4_rad',NaN,'jointMotion5_rad',NaN,'jointMotion6_rad',NaN, ...
    'nodeCount',0,'edgeCollisionChecks',0,'collisionConfigurationChecks',0, ...
    'pathStateCount',0,'plannerResolutionDeg',NaN,'validationResolutionDeg',NaN, ...
    'validationConfigurationChecks',0,'validationEdgeCollisionChecks',0, ...
    'validationFailureEdge',NaN,'validationFailureSample',NaN, ...
    'validationCollisionType',"");
end

function validation=emptyValidation()
validation=struct('pass',false,'endpointValid',false,'nodeLimitValid',false, ...
    'collisionFree',false,'collisionConfigurationChecks',0,'edgeCollisionChecks',0, ...
    'failureEdgeIndex',NaN,'failureSampleIndex',NaN,'failureCollisionType',"");
end

function validation=validatePilotPath(path,c,scene,resolutionRad)
validation=emptyValidation();
if isempty(path) || size(path,2)~=6 || any(~isfinite(path(:))), return; end
validation.endpointValid=max(abs(path(1,:)-c.start))<=1e-8 && ...
    max(abs(path(end,:)-c.goal))<=1e-8;
validation.nodeLimitValid=all(arrayfun(@(i)jointWithinLimits(path(i,:),scene.jointLimits),1:size(path,1)));
if ~(validation.endpointValid && validation.nodeLimitValid), return; end
opts=struct('wrapJoints',true);
opts.selfCollisionSkipPairs=scene.selfCollisionSkipPairs;
validation.collisionFree=true;
for k=1:size(path,1)-1
    validation.edgeCollisionChecks=validation.edgeCollisionChecks+1;
    [free,collisionIndex,count]=collisionFreeJointEdge(scene.robot,path(k,:),path(k+1,:), ...
        scene.environment,resolutionRad,opts);
    validation.collisionConfigurationChecks=validation.collisionConfigurationChecks+count;
    if ~free
        validation.collisionFree=false;
        validation.failureEdgeIndex=k;
        validation.failureSampleIndex=collisionIndex;
        dq=atan2(sin(path(k+1,:)-path(k,:)),cos(path(k+1,:)-path(k,:)));
        nseg=max(1,ceil(max(abs(dq))/resolutionRad));
        qFailure=path(k,:)+((collisionIndex-1)/nseg)*dq;
        [~,detail]=collisionFreeJointState(scene.robot,qFailure,scene.environment,scene.selfCollisionSkipPairs);
        if detail.environmentCollision && detail.selfCollision
            validation.failureCollisionType="environment+self";
        elseif detail.environmentCollision
            validation.failureCollisionType="environment";
        elseif detail.selfCollision
            validation.failureCollisionType="self";
        else
            validation.failureCollisionType="unknown";
        end
        break;
    end
end
validation.pass=validation.endpointValid && validation.nodeLimitValid && validation.collisionFree;
end

function rows=assignRelativePathLength(rows)
for i=1:numel(rows)
    if ~(rows(i).success && rows(i).finalValidationPass), continue; end
    same=arrayfun(@(x) string(x.caseID)==rows(i).caseID && x.seed==rows(i).seed && ...
        x.success && x.finalValidationPass,rows);
    vals=[rows(same).jointPathLengthL2_rad];
    rows(i).relativePathLength=rows(i).jointPathLengthL2_rad/min(vals);
end
end

function summary=aggregateRows(rows,cases,algorithms,bootstrapReplicates)
summary=repmat(emptySummaryRow(),0,1);
for c=1:numel(cases)
    for a=1:numel(algorithms)
        algName=algorithmLabel(algorithms{a});
        mask=arrayfun(@(x) string(x.caseID)==string(cases(c).caseID) && ...
            string(x.algorithm)==algName,rows);
        r=rows(mask);
        valid=[r.success] & [r.finalValidationPass];
        times=[r(valid).planningTime_ms];
        lengths=[r(valid).jointPathLengthL2_rad];
        s=emptySummaryRow(); s.caseID=string(cases(c).caseID); s.algorithm=algName;
        s.seedCount=numel(r); s.successCount=sum([r.success]);
        s.successRate=s.successCount/max(s.seedCount,1);
        s.finalValidationCount=sum(valid); s.finalValidationRate=s.finalValidationCount/max(s.seedCount,1);
        s.validSuccessCount=s.finalValidationCount;
        s.validSuccessRate=s.finalValidationRate;
        s.timeoutCount=sum([r.plannerTimedOut]);
        s.invalidPathCount=sum(string({r.outcome})=="INVALID_PATH");
        s.planningTimeMedian_ms=statMedian(times); s.planningTimeIQR_ms=statIQR(times);
        s.planningTimeP90_ms=statPercentile(times,90); s.planningTimeMean_ms=statMean(times);
        s.planningTimeStd_ms=statStd(times);
        [s.planningTimeCI95Low_ms,s.planningTimeCI95High_ms]=bootstrapMedianCI(times,bootstrapReplicates);
        s.jointPathLengthMedian_rad=statMedian(lengths); s.jointPathLengthIQR_rad=statIQR(lengths);
        s.jointPathLengthP90_rad=statPercentile(lengths,90); s.jointPathLengthMean_rad=statMean(lengths);
        s.jointPathLengthStd_rad=statStd(lengths);
        [s.jointPathLengthCI95Low_rad,s.jointPathLengthCI95High_rad]=bootstrapMedianCI(lengths,bootstrapReplicates);
        s.totalJointMotionMedian_rad=statMedian([r(valid).totalJointMotion_rad]);
        s.collisionConfigurationChecksMedian=statMedian([r.collisionConfigurationChecks]);
        s.edgeCollisionChecksMedian=statMedian([r.edgeCollisionChecks]);
        s.nodeCountMedian=statMedian([r.nodeCount]);
        summary(end+1,1)=s; %#ok<AGROW>
    end
end
end

function row=emptySummaryRow()
row=struct('caseID',"",'algorithm',"",'seedCount',0,'successCount',0,'successRate',NaN, ...
    'finalValidationCount',0,'finalValidationRate',NaN, ...
    'validSuccessCount',0,'validSuccessRate',NaN,'timeoutCount',0,'invalidPathCount',0, ...
    'planningTimeMedian_ms',NaN,'planningTimeIQR_ms',NaN,'planningTimeP90_ms',NaN, ...
    'planningTimeMean_ms',NaN,'planningTimeStd_ms',NaN,'planningTimeCI95Low_ms',NaN, ...
    'planningTimeCI95High_ms',NaN,'jointPathLengthMedian_rad',NaN, ...
    'jointPathLengthIQR_rad',NaN,'jointPathLengthP90_rad',NaN, ...
    'jointPathLengthMean_rad',NaN,'jointPathLengthStd_rad',NaN, ...
    'jointPathLengthCI95Low_rad',NaN,'jointPathLengthCI95High_rad',NaN, ...
    'totalJointMotionMedian_rad',NaN,'collisionConfigurationChecksMedian',NaN, ...
    'edgeCollisionChecksMedian',NaN,'nodeCountMedian',NaN);
end

function report=makeReport(createdAt,options,cases,algorithms,rows,paths,metadata,completedRuns)
phase='Phase 12A-2';
if isfield(options,'taskSet') && strcmpi(options.taskSet,'seamA')
    phase='Phase 12A-2R';
end
report=struct('created',createdAt,'phase',phase, ...
    'sceneScript',options.sceneScript,'approachResult',options.approachResult, ...
    'cases',cases,'algorithms',{algorithms},'options',options,'rows',rows, ...
    'paths',{paths},'pathMetadata',{metadata},'completedRuns',completedRuns);
end

function writeRunRows(rows,file)
writetable(struct2table(rows),file);
end

function writeSummaryRows(rows,file)
writetable(struct2table(rows),file);
end

function makeCoreFigures(rows,algorithms,figureDir)
if ~isfolder(figureDir), mkdir(figureDir); end
valid=[rows.finalValidationPass] & [rows.success];
times=[rows(valid).planningTime_ms];
lengths=[rows(valid).jointPathLengthL2_rad];
groups=cellstr(string({rows(valid).algorithm}));
f=figure('Visible','off'); boxplot(times,groups); ylabel('Planning time (ms)'); grid on;
title('Phase 12A-2 planning time'); saveas(f,fullfile(figureDir,'phase12a_pilot_planning_time_boxplot.png')); close(f);
f=figure('Visible','off'); boxplot(lengths,groups); ylabel('Joint path length L_q (rad)'); grid on;
title('Phase 12A-2 joint path length'); saveas(f,fullfile(figureDir,'phase12a_pilot_path_length_boxplot.png')); close(f);
f=figure('Visible','off'); hold on; colors=lines(numel(algorithms));
for a=1:numel(algorithms)
    name=algorithmLabel(algorithms{a});
    mask=valid(:).' & arrayfun(@(x) string(x.algorithm)==name,rows).';
    scatter([rows(mask).planningTime_ms],[rows(mask).jointPathLengthL2_rad],24,colors(a,:), ...
        'filled','DisplayName',name);
end
xlabel('Planning time (ms)'); ylabel('Joint path length L_q (rad)'); grid on;
legend('Location','best'); title('Phase 12A-2 speed vs path length');
saveas(f,fullfile(figureDir,'phase12a_pilot_planning_vs_path_scatter.png')); close(f);
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

function v=statMedian(x)
if isempty(x), v=NaN; else, v=median(x); end
end
function v=statMean(x)
if isempty(x), v=NaN; else, v=mean(x); end
end
function v=statStd(x)
if numel(x)<2, v=NaN; else, v=std(x); end
end
function v=statIQR(x)
if isempty(x), v=NaN; else, v=statPercentile(x,75)-statPercentile(x,25); end
end
function v=statPercentile(x,p)
if isempty(x), v=NaN; return; end
x=sort(x(:)); pos=1+(numel(x)-1)*p/100;
lo=floor(pos); hi=ceil(pos);
if lo==hi, v=x(lo); else, v=x(lo)+(pos-lo)*(x(hi)-x(lo)); end
end
function [lo,hi]=bootstrapMedianCI(x,B)
if isempty(x), lo=NaN; hi=NaN; return; end
if nargin<2 || isempty(B), B=1000; end
state=rng; cleanup=onCleanup(@()rng(state)); %#ok<NASGU>
rng(20260927+numel(x),'twister');
idx=randi(numel(x),numel(x),B);
boot=median(x(idx),1);
lo=statPercentile(boot,2.5); hi=statPercentile(boot,97.5);
end

function out=mergeOptions(defaults,in)
out=defaults; names=fieldnames(defaults);
for i=1:numel(names)
    if isfield(in,names{i}) && ~isempty(in.(names{i})), out.(names{i})=in.(names{i}); end
end
end
