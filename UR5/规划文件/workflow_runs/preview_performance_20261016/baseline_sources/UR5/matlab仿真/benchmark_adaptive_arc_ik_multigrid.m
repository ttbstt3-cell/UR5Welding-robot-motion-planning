function report = benchmark_adaptive_arc_ik_multigrid(options)
%BENCHMARK_ADAPTIVE_ARC_IK_MULTIGRID
% Compare deterministic sparse 16-point seam anchor grids on the three
% remaining ACCP failure seeds.  Every run is a cold start and uses
% the existing A-ARC-IK feasibility + conditional recovery wrapper.
%
% This experiment does not load an old qPath and does not change the main
% planner defaults.  It is intended to decide whether anchor placement can
% rescue failures before a Global Layered Graph fallback is added.

if nargin<1 || isempty(options), options=struct(); end
options=localOptions(options);
if ~isfolder(options.outputDir), mkdir(options.outputDir); end
[gridNames,gridU]=localGrids(options.numPoints);
seeds=options.seeds(:).';

rows=repmat(localEmptyRow(),0,1);
results=cell(0,1);
fprintf('\n==============================================\n');
fprintf('A-ARC-IK MULTI-GRID COLD-START EXPERIMENT\n');
fprintf('%d seeds x %d deterministic %d-point grids\n',numel(seeds),numel(gridNames),options.numPoints);
fprintf('==============================================\n');

runIndex=0;
for iseed=1:numel(seeds)
    seed=seeds(iseed);
    endpointFile=options.referenceFiles{iseed};
    if ~isfile(endpointFile), error('Reference MAT not found: %s',endpointFile); end
    ref=load(endpointFile).result;
    p0=ref.seam.startWP(:).'; p1=ref.seam.endWP(:).';
    for ig=1:numel(gridNames)
        runIndex=runIndex+1;
        positions=(1-gridU(:,ig))*p0 + gridU(:,ig)*p1;
        outFile=fullfile(options.outputDir,sprintf('multigrid_seed%d_%s.mat',seed,gridNames(ig)));
        fprintf('\n[%02d/%02d] seed=%d grid=%s\n',runIndex,numel(seeds)*numel(gridNames),seed,gridNames(ig));
        o=options.plannerOptions;
        o.initialPoints=options.numPoints;
        % The adaptive wrapper now accepts an explicit first-round seam.
        % This is distinct from the core's per-round seamPositionsWP, which
        % is managed internally after refinement.
        o.initialSeamPositionsWP=positions;
        o.randomSeed=seed;
        o.outputFile=outFile;
        o.roundOutputPrefix=fullfile(options.outputDir,sprintf('round_seed%d_%s',seed,gridNames(ig)));
        t=tic;
        try
            r=replan_weld_actual_axis_adaptive_feasibility(o);
            wall=toc(t);
            row=localMakeRow(r,seed,gridNames(ig),wall,outFile);
            results{end+1,1}=r; %#ok<AGROW>
        catch ME
            wall=toc(t);
            row=localEmptyRow();
            row.seed=seed; row.grid=string(gridNames(ig)); row.initialPoints=options.numPoints;
            row.complete=false; row.wallTimeSec=wall; row.outputFile=string(outFile);
            row.errorMessage=string(ME.message);
            warning('seed %d grid %s failed with MATLAB error: %s',seed,gridNames(ig),ME.message);
            results{end+1,1}=struct('error',ME); %#ok<AGROW>
        end
        rows(end+1,1)=row; %#ok<AGROW>
        fprintf('  PASS=%d time=%.3fs IK=%d firstFail=%d finalPoints=%d maxTCP=%.4fmm\n', ...
            row.complete,row.algorithmTimeSec,row.ikCalls,row.firstFailureLayer,row.finalPointCount,row.maxTcpSegmentErrorMm);
    end
end

report=struct();
report.generatedAt=string(datetime('now'));
report.options=options;
report.gridNames=gridNames;
report.gridU=gridU;
report.rows=rows;
report.results=results;
report.outputMat=options.outputMat;
report.outputCsv=options.outputCsv;
save(options.outputMat,'report','-v7.3');
localWriteCsv(rows,options.outputCsv);
fprintf('\nSaved MAT: %s\nSaved CSV: %s\n',options.outputMat,options.outputCsv);
localPrintSummary(rows);
end

function options=localOptions(options)
root='/home/ttbstt/jixiebi/UR5/matlab仿真';
defaults=struct( ...
    'seeds',[20269018 20269019 20269022], ...
    'referenceFiles',{{ ...
        fullfile(root,'adaptive_arc_ik_recovery_runs','adaptive_recovery_seed20269018.mat'), ...
        fullfile(root,'adaptive_arc_ik_recovery_runs','adaptive_recovery_seed20269019.mat'), ...
        fullfile(root,'adaptive_arc_ik_recovery_runs','adaptive_recovery_seed20269022.mat')}}, ...
    'numPoints',16, ...
    'outputDir',fullfile(root,'adaptive_arc_ik_multigrid_corrected_runs'), ...
    'outputMat',fullfile(root,'adaptive_arc_ik_multigrid_corrected.mat'), ...
    'outputCsv',fullfile(root,'adaptive_arc_ik_multigrid_corrected.csv'), ...
    'plannerOptions',struct( ...
        'maxRefinementRounds',4,'maxPoints',40,'maxSegmentTcpErrorMm',0.5, ...
        'maxIntermediatePoints',4,'maxPrecisionRounds',2, ...
        'animate',false,'publish',false,'enableFallback',false, ...
        'beamWidth',1,'firstPointMaxStates',2,'maxPoseSeeds',3, ...
        'coarseTravelDeg',0,'coarseRollDeg',0:45:315, ...
        'seedPerturbDeg',8,'maxPoseNeighbors',9, ...
        'maxPoseStepRollDeg',15,'maxPoseStepTravelDeg',5, ...
        'numRandomSeedsFirstPoint',3,'maxRollStepDeg',45, ...
        'maxTravelStepDeg',15,'maxJointStepDeg',15,'edgeResolutionDeg',3, ...
        'maxBacktrackLayers',8,'positionToleranceM',1e-4, ...
        'orientationToleranceDeg',0.5,'sideToleranceDeg',5,'sideMarginM',1e-3, ...
        'checkSelfCollision',true,'reuseIKSolver',true,'reuseCandidateCache',true, ...
        'lazyCollision',true,'progressivePoseNeighbors',true, ...
        'progressiveInitialPoseNeighbors',5,'progressiveMaxPoseNeighbors',9, ...
        'disableLocalRecovery',false,'poseNeighborhoodCost','zero', ...
        'angularRedundancyFallback',false,'angularFallbackRollDeg',[10 20], ...
        'angularFallbackTravelDeg',[0 5 10],'enablePostRecovery',true, ...
        'recoveryBeamWidths',[2 3],'recoveryRollFallbackDeg',[10 20], ...
        'recoveryTravelFallbackDeg',[0 5 10],'recoveryMaxPoseNeighbors',25, ...
        'recoveryPoseNeighborhoodCost','previous','recoveryMaxPoseStepRollDeg',20, ...
        'recoveryMaxPoseStepTravelDeg',10));
n=fieldnames(defaults);
for i=1:numel(n)
    if ~isfield(options,n{i}) || isempty(options.(n{i})), options.(n{i})=defaults.(n{i}); end
end
end

function [names,U]=localGrids(n)
u=linspace(0,1,n).'; h=1/(n-1);
names=["grid0_uniform","grid1_shift_plus","grid2_shift_minus","grid3_alternating","grid4_center_dense"];
U=zeros(n,numel(names)); U(:,1)=u;
v=u; v(2:end-1)=v(2:end-1)+0.25*h; U(:,2)=v;
v=u; v(2:end-1)=v(2:end-1)-0.25*h; U(:,3)=v;
v=u; v(2:end-1)=v(2:end-1)+0.20*h*(-1).^(2:n-1).'; U(:,4)=v;
U(:,5)=(1-cos(pi*u))/2;
for j=1:size(U,2)
    U(1,j)=0; U(end,j)=1;
    if any(diff(U(:,j))<=0), error('Grid %s is not strictly increasing.',names(j)); end
end
end

function row=localMakeRow(r,seed,grid,wall,outFile)
row=localEmptyRow();
row.seed=seed; row.grid=string(grid); row.initialPoints=r.initialPoints;
row.complete=logical(r.complete); row.nodeCollisionFree=logical(r.nodeCollisionFree);
row.edgeCollisionFree=logical(r.edgeCollisionFree); row.wallTimeSec=wall;
row.algorithmTimeSec=localField(r,'feasibilityAdaptiveAlgorithmTimeSec',r.totalPlanningTimeSec);
row.ikCalls=localField(r,'feasibilityAdaptiveIkCalls',r.continuationStats.ikCalls);
row.finalPointCount=r.finalPointCount;
row.firstFailureLayer=localFirstFailure(r.stateBank);
row.maxJointJumpDeg=r.maxJointJumpDeg;
if isfield(r,'finalTcpMetrics'), m=r.finalTcpMetrics; elseif isfield(r,'tcpMetrics'), m=r.tcpMetrics; else, m=struct(); end
row.maxTcpSegmentErrorMm=localField(m,'maxSegmentPositionErrorMm',NaN);
row.postRecoveryUsed=localField(r,'postRecoveryUsed',false);
row.postRecoveryAttempts=localField(r,'postRecoveryAttempts',0);
row.postRecoveryTimeSec=localField(r,'postRecoveryTimeSec',0);
row.postRecoveryIkCalls=localField(r,'postRecoveryIkCalls',0);
row.outputFile=string(outFile);
end

function x=localField(s,name,default)
if isstruct(s) && isfield(s,name) && ~isempty(s.(name)), x=s.(name); else, x=default; end
end

function idx=localFirstFailure(bank)
idx=find(cellfun(@isempty,bank),1,'first');
if isempty(idx), idx=NaN; end
end

function x=localEmptyRow()
x=struct('seed',NaN,'grid',"",'initialPoints',0,'complete',false, ...
    'nodeCollisionFree',false,'edgeCollisionFree',false,'wallTimeSec',NaN, ...
    'algorithmTimeSec',NaN,'ikCalls',0,'firstFailureLayer',NaN,'finalPointCount',0, ...
    'maxJointJumpDeg',NaN,'maxTcpSegmentErrorMm',NaN,'postRecoveryUsed',false, ...
    'postRecoveryAttempts',0,'postRecoveryTimeSec',0,'postRecoveryIkCalls',0, ...
    'outputFile',"",'errorMessage',"");
end

function localWriteCsv(rows,file)
fid=fopen(file,'w');
fprintf(fid,'seed,grid,initialPoints,complete,nodeCollisionFree,edgeCollisionFree,wallTimeSec,algorithmTimeSec,ikCalls,firstFailureLayer,finalPointCount,maxJointJumpDeg,maxTcpSegmentErrorMm,postRecoveryUsed,postRecoveryAttempts,postRecoveryTimeSec,postRecoveryIkCalls,outputFile,errorMessage\n');
for i=1:numel(rows)
    x=rows(i);
    fprintf(fid,'%d,%s,%d,%d,%d,%d,%.6f,%.6f,%d,%.0f,%d,%.6f,%.6f,%d,%d,%.6f,%d,"%s","%s"\n', ...
        x.seed,char(x.grid),x.initialPoints,x.complete,x.nodeCollisionFree,x.edgeCollisionFree, ...
        x.wallTimeSec,x.algorithmTimeSec,x.ikCalls,x.firstFailureLayer,x.finalPointCount, ...
        x.maxJointJumpDeg,x.maxTcpSegmentErrorMm,x.postRecoveryUsed,x.postRecoveryAttempts, ...
        x.postRecoveryTimeSec,x.postRecoveryIkCalls,char(x.outputFile),char(x.errorMessage));
end
fclose(fid);
end

function localPrintSummary(rows)
fprintf('\nSummary by seed:\n');
for s=unique([rows.seed])
    x=rows([rows.seed]==s);
    fprintf('seed %d: %d/%d PASS\n',s,sum([x.complete]),numel(x));
end
end
