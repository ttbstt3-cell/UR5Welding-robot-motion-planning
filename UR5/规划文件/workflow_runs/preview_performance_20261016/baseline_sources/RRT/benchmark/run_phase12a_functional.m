function report = run_phase12a_functional(options)
%run_phase12a_functional Phase 12A-1 offline UR5 functional validation.
%
% This is deliberately a small case-based test, not a 100-seed benchmark.
% It reads the current validated weld/approach result and plans only free
% space transitions.  The 80-point weld path itself is never replanned.

if nargin < 1 || isempty(options), options=struct(); end
here=fileparts(mfilename('fullpath'));
root=fileparts(here);
defaults=struct('sceneScript','/home/ttbstt/jixiebi/UR5/matlab仿真/sync_ur5_joint_states.m', ...
    'approachResult','/home/ttbstt/jixiebi/UR5/matlab仿真/weld_rrt_connect_approach_exit.mat', ...
    'maxIterations',600,'maxJointStepRad',deg2rad(25), ...
    'jointEdgeResolutionRad',deg2rad(2),'goalBias',0.20, ...
    'seed',1,'saveResult',true,'phase','Phase 12A-1', ...
    'archivalRerun',false, ...
    'outputFile',fullfile(root,'results','phase12a_functional.mat'), ...
    'summaryFile',fullfile(root,'results','phase12a_functional_summary.csv'));
options=mergeOptions(defaults,options);
addpath(genpath(root));

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

% These are real task states from the canonical UR5 handoff.  The two extra
% diagnostic cases stay outside the weld segment and do not replace the
% prescribed Home -> PreWeld / PostWeld -> Home task transitions.
cases=repmat(struct('caseID','', 'description','', 'start',zeros(1,6), ...
    'goal',zeros(1,6)),0,1);
cases(end+1)=makeCase('C01_Home_to_PreWeld','real approach transition',qHome,qPre);
cases(end+1)=makeCase('C02_PostWeld_to_Home','real exit transition',qPost,qHome);
cases(end+1)=makeCase('C03_PreWeld_to_Point1','real weld-entry transition',qPre,qStart);
cases(end+1)=makeCase('C04_Point80_to_PostWeld','real weld-exit transition',qEnd,qPost);
cases(end+1)=makeCase('C05_Home_to_Point1','diagnostic near-workpiece/tool case',qHome,qStart);
qSimple=qHome; qSimple(1)=qSimple(1)+deg2rad(5);
cases(end+1)=makeCase('C06_Simple_5deg','simple local free-space case',qHome,qSimple);

algorithms={'RRTConnectJoint','BiRRTStarJoint','ClassGoalLiteJoint','ClassifiedRRTStarJoint'};
rows=repmat(emptyRow(),0,1);
paths=cell(0,1);
pathMetadata=cell(0,1);
generatedAt=string(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
codeVersion=detectCodeVersion(root);
fprintf('\nPHASE 12A-1: %d cases x %d planners (offline MATLAB only)\n', ...
    numel(cases),numel(algorithms));
for c=1:numel(cases)
    fprintf('\n[%s] %s\n',cases(c).caseID,cases(c).description);
    for a=1:numel(algorithms)
        cfg=struct('scene',scene,'start',cases(c).start,'goal',cases(c).goal, ...
            'maxIterations',options.maxIterations,'maxJointStepRad',options.maxJointStepRad, ...
            'jointEdgeResolutionRad',options.jointEdgeResolutionRad, ...
            'goalBias',options.goalBias,'seed',options.seed+c-1);
        t=feval(algorithms{a},cfg);
        row=emptyRow();
        row.caseID=string(cases(c).caseID); row.description=string(cases(c).description);
        row.algorithm=string(t.algorithm); row.seed=cfg.seed; row.success=t.success;
        row.planningTime_ms=t.planningTimeMs; row.firstSolutionTime_ms=t.firstSolutionTimeMs;
        row.jointPathCostL2=t.jointPathCostL2; row.nodeCount=t.nodeCount;
        row.collisionChecks=t.collisionChecks;
        row.collisionConfigurationChecks=t.collisionConfigurationChecks;
        row.edgeCollisionChecks=t.edgeCollisionChecks;
        row.pathStateCount=t.pathStateCount;
        row.maxJointEdgeStepDeg=rad2deg(t.maxJointEdgeStepRad);
        row.firstSolutionIteration=t.firstSolutionIteration;
        row.jointLimitsViolated=~isempty(t.path) && any(~arrayfun(@(i)jointWithinLimits(t.path(i,:),t.jointLimits),1:size(t.path,1)));
        row.pathValid=validateOutputPath(t,cases(c),scene);
        rows(end+1,1)=row; %#ok<AGROW>
        paths{end+1,1}=t.path; %#ok<AGROW>
        cfgRecord=rmfield(cfg,'scene');
        cfgRecord.jointLimits=scene.jointLimits;
        pathMetadata{end+1,1}=struct('caseID',string(cases(c).caseID), ...
            'description',string(cases(c).description), ...
            'algorithm',string(t.algorithm),'seed',cfg.seed, ...
            'generatedAt',generatedAt,'jointEdgeResolutionRad', ...
            cfg.jointEdgeResolutionRad,'config',cfgRecord, ...
            'codeVersion',codeVersion); %#ok<AGROW>
        fprintf('  %-24s success=%d valid=%d time=%.1f ms first=%.1f ms cost=%.4f checks=%d nodes=%d\n', ...
            t.algorithm,t.success,row.pathValid,t.planningTimeMs,t.firstSolutionTimeMs, ...
            t.jointPathCostL2,t.collisionChecks,t.nodeCount);
    end
end

report=struct();
report.created=generatedAt;
report.phase=options.phase; report.archivalRerun=logical(options.archivalRerun);
report.codeVersion=codeVersion; report.sceneScript=options.sceneScript;
report.approachResult=options.approachResult; report.cases=cases;
report.rows=rows; report.paths=paths; report.pathMetadata=pathMetadata;
report.algorithms=algorithms; report.options=options;
report.allSuccessful=all([rows.success]); report.allPathValid=all([rows.pathValid]);
report.note='Offline joint-space migration validation only; ROS2/CoppeliaSim was not started.';

if options.saveResult
    resultDir=fullfile(root,'results');
    if ~isfolder(resultDir), mkdir(resultDir); end
    save(options.outputFile,'report','-v7.3');
    writeRows(rows,options.summaryFile);
    fprintf('\nSaved Phase 12A report under %s\n',options.outputFile);
end
end

function c=makeCase(id,description,start,goal)
c=struct('caseID',id,'description',description,'start',start,'goal',goal);
end

function row=emptyRow()
row=struct('caseID',"",'description',"",'algorithm',"",'seed',NaN, ...
    'success',false,'pathValid',false,'planningTime_ms',NaN, ...
    'firstSolutionTime_ms',NaN,'jointPathCostL2',NaN,'nodeCount',0, ...
    'collisionChecks',0,'collisionConfigurationChecks',0,'edgeCollisionChecks',0, ...
    'pathStateCount',0,'maxJointEdgeStepDeg',NaN, ...
    'firstSolutionIteration',NaN,'jointLimitsViolated',false);
end

function pass=validateOutputPath(result,c,scene)
pass=false;
if ~result.success || isempty(result.path) || size(result.path,2)~=6, return; end
if max(abs(result.path(1,:)-c.start))>1e-8 || max(abs(result.path(end,:)-c.goal))>1e-8, return; end
if any(~arrayfun(@(i)jointWithinLimits(result.path(i,:),result.jointLimits),1:size(result.path,1))), return; end
for i=1:size(result.path,1)-1
    opts=struct('wrapJoints',true);
    opts.selfCollisionSkipPairs=scene.selfCollisionSkipPairs;
    [free,~,~]=collisionFreeJointEdge(scene.robot,result.path(i,:),result.path(i+1,:), ...
        scene.environment,result.config.jointEdgeResolutionRad,opts);
    if ~free, return; end
end
pass=true;
end

function writeRows(rows,file)
fid=fopen(file,'w');
if fid<0, error('Cannot write %s',file); end
cleanup=onCleanup(@()fclose(fid));
header=['caseID,description,algorithm,seed,success,pathValid,planningTime_ms,', ...
    'firstSolutionTime_ms,jointPathCostL2,nodeCount,collisionChecks,', ...
    'collisionConfigurationChecks,edgeCollisionChecks,pathStateCount,', ...
    'maxJointEdgeStepDeg,firstSolutionIteration,jointLimitsViolated',char(10)];
fprintf(fid,'%s',header);
for i=1:numel(rows)
    r=rows(i);
    fprintf(fid,'"%s","%s","%s",%d,%d,%d,%.9g,%.9g,%.9g,%d,%d,%d,%d,%d,%.9g,%.9g,%d%s', ...
        r.caseID,r.description,r.algorithm,r.seed,r.success,r.pathValid, ...
        r.planningTime_ms,r.firstSolutionTime_ms,r.jointPathCostL2,r.nodeCount, ...
        r.collisionChecks,r.collisionConfigurationChecks,r.edgeCollisionChecks, ...
        r.pathStateCount,r.maxJointEdgeStepDeg, ...
        r.firstSolutionIteration,r.jointLimitsViolated,char(10));
end
end

function out=mergeOptions(defaults,in)
out=defaults; names=fieldnames(defaults);
for i=1:numel(names)
    if isfield(in,names{i}) && ~isempty(in.(names{i})), out.(names{i})=in.(names{i}); end
end
end

function version=detectCodeVersion(root)
[status,text]=system(sprintf('git -C "%s" rev-parse --short HEAD 2>/dev/null',root));
if status==0 && ~isempty(strtrim(text))
    version=string(strtrim(text));
else
    version="unavailable (RRT directory is not a Git repository)";
end
end
