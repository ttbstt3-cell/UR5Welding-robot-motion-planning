function result = replan_weld_actual_axis_adaptive_feasibility(options)
%REPLAN_WELD_ACTUAL_AXIS_ADAPTIVE_FEASIBILITY
% ACCP adaptive configuration-search coordinator.
%
% Start from a sparse seam (normally 16 points).  If bounded continuation
% fails at layer k, bisect only the preceding geometric interval and retry
% from a cold start.  This file is independent of the stable 80-point
% continuation and does not load an old qPath/state graph.
%
% This first stage deliberately separates feasibility refinement from the
% ACCP's internal Cartesian precision stage. The core owns both stages;
% this coordinator only schedules configuration bisection/recovery retries.

if nargin < 1 || isempty(options), options = struct(); end
% Formal real-scene ACCP default: shorten only the collision-cylinder length
% by 3 mm. The visual wire, TCP, IK data and thresholds remain unchanged.
defaults = struct( ...
    "initialPoints",16, ...
    "initialSeamPositionsWP",[], ...
    "maxRefinementRounds",4, ...
    "maxPoints",40, ...
    "maxSegmentTcpErrorMm",0.5, ...
    "maxIntermediatePoints",4, ...
    "maxPrecisionRounds",2, ...
    "animate",false,"publish",false, ...
    "enableFallback",false, ...
    "beamWidth",1,"firstPointMaxStates",2,"maxPoseSeeds",3, ...
    "endMarginM",0.025, ...
    "coarseTravelDeg",0, ...
    "coarseRollDeg",0:45:315, ...
    "seedPerturbDeg",8,"maxPoseNeighbors",9, ...
    "maxPoseStepRollDeg",15,"maxPoseStepTravelDeg",5, ...
    "numRandomSeedsFirstPoint",3, ...
    "maxRollStepDeg",45,"maxTravelStepDeg",15, ...
    "maxJointStepDeg",15,"edgeResolutionDeg",3, ...
    "maxBacktrackLayers",8, ...
    "positionToleranceM",1e-4,"orientationToleranceDeg",0.5, ...
    "sideToleranceDeg",5,"sideMarginM",1e-3, ...
    "axis_angle_tolerance_deg",[], ...
    "disableAxisConstraint",false, ...
    "wire_collision_shorten_mm",3, ...
    "runAxisConstraintDiagnostic",false, ...
    "disable_axis_constraint",false, ...
    "run_axis_constraint_diagnostic",false, ...
    "candidate_scoring_mode",false,"candidate_diagnostics",false, ...
    "candidate_score_weights",[1 1 0.1 0.1], ...
    "checkSelfCollision",true, ...
    "reuseIKSolver",true,"reuseCandidateCache",true,"lazyCollision",true, ...
    "progressivePoseNeighbors",true, ...
    "progressiveInitialPoseNeighbors",5, ...
    "progressiveMaxPoseNeighbors",9, ...
    "disableLocalRecovery",false, ...
    "poseNeighborhoodCost","zero", ...
    "angularRedundancyFallback",false, ...
    "angularFallbackRollDeg",[10 20], ...
    "angularFallbackTravelDeg",[0 5 10], ...
    "enablePostRecovery",false, ...
    "recoveryBeamWidths",[2 3], ...
    "recoveryRollFallbackDeg",[10 20], ...
    "recoveryTravelFallbackDeg",[0 5 10], ...
    "recoveryMaxPoseNeighbors",25, ...
    "recoveryPoseNeighborhoodCost","previous", ...
    "recoveryMaxPoseStepRollDeg",20, ...
    "recoveryMaxPoseStepTravelDeg",10, ...
    "branchReferenceQ",[], ...
    "maxPlanningTimeSec",[], ...
    "randomSeed",20260923, ...
    "branchReferenceSource","", ...
    "outputFile","weld_actual_axis_adaptive_feasibility.mat", ...
    "roundOutputPrefix","weld_actual_axis_adaptive_feasibility_round");
options = localMergeOptions(defaults,options);
% Accept the explicit snake_case diagnostic switch requested by the HMI
% audit, while keeping the formal planner default unchanged (false).
options.disableAxisConstraint = logical(options.disableAxisConstraint || ...
    options.disable_axis_constraint);
options.runAxisConstraintDiagnostic = logical(options.runAxisConstraintDiagnostic || ...
    options.run_axis_constraint_diagnostic);

here = fileparts(mfilename("fullpath"));
% Optional experiment-only initial sparse seam.  When supplied, the first
% adaptive round starts from these exact anchor positions instead of
% regenerating the default uniform seam.  The normal planner behavior is
% unchanged because the default is empty.
currentPositions = options.initialSeamPositionsWP;
if ~isempty(currentPositions)
    if size(currentPositions,2) ~= 3 || size(currentPositions,1) < 2
        error("initialSeamPositionsWP must be an N-by-3 matrix with N >= 2.");
    end
    options.initialPoints = size(currentPositions,1);
end
rounds = repmat(localEmptyRound(),0,1);
totalTimer = tic;
strongFirstPointTried = false;
finalCore = struct();
latestPoint1Diagnostics = struct();

fprintf("\n========================================\n");
fprintf("ACCP: ADAPTIVE CONFIGURATION AND EMBEDDED PRECISION\n");
fprintf("========================================\n");
fprintf("Initial seam points : %d\n",options.initialPoints);
fprintf("Max refinement     : %d rounds / %d points\n", ...
    options.maxRefinementRounds,options.maxPoints);
if ~isempty(options.maxPlanningTimeSec)
    fprintf("Cooperative budget : %.1f s (checked between rounds only)\n", ...
        options.maxPlanningTimeSec);
end

% Cooperative time budget.  MATLAB cannot interrupt a running IK/collision
% call, so this only prevents *starting* another round once the budget is
% already spent.  A single in-progress round may still overrun; the elapsed
% time is always reported so the caller can see the real overrun.
budgetExceeded = false;
for iround = 0:options.maxRefinementRounds
    if ~isempty(options.maxPlanningTimeSec) && toc(totalTimer) > options.maxPlanningTimeSec
        budgetExceeded = true;
        rr = localEmptyRound();
        rr.round = iround;
        rr.pointsBefore = size(currentPositions,1);
        rr.algorithmTimeSec = 0;
        rr.wallTimeSec = 0;
        rr.ikCalls = 0;
        rr.recovery = "time_budget_exhausted";
        rounds(end+1,1) = rr; %#ok<AGROW>
        fprintf("[STOP] Cooperative time budget exhausted before round %d.\n",iround);
        break;
    end
    coreOptions = localCoreOptions(options);
    coreOptions.randomSeed = options.randomSeed;
    coreOptions.animate = false;
    coreOptions.publish = false;
    coreOptions.enableFallback = false;
    if isempty(currentPositions)
        coreOptions.numPoints = options.initialPoints;
        coreOptions.seamPositionsWP = [];
    else
        coreOptions.numPoints = size(currentPositions,1);
        coreOptions.seamPositionsWP = currentPositions;
    end
    roundFile = sprintf("%s_%02d.mat",options.roundOutputPrefix,iround);
    if ~startsWith(string(roundFile),filesep)
        roundFile = fullfile(here,roundFile);
    end
    coreOptions.outputFile = roundFile;

    fprintf("\n[Adaptive round %d] planning %d seam points...\n", ...
        iround,coreOptions.numPoints);
    tRound = tic;
    finalCore = replan_weld_actual_axis_adaptive_core(coreOptions);
    if isfield(finalCore,"firstPointDiagnostics")
        latestPoint1Diagnostics = finalCore.firstPointDiagnostics;
    end
    roundWall = toc(tRound);

    % The core always returns the actual point list used in this round.
    currentPositions = finalCore.seam.positionsWP;
    failIdx = localFirstEmptyLayer(finalCore.stateBank);
    if isempty(failIdx), failIdx = NaN; end
    rr = localEmptyRound();
    rr.round = iround;
    rr.pointsBefore = size(currentPositions,1);
    rr.complete = finalCore.configurationComplete;
    rr.furthestPoint = localFurthestPoint(finalCore.stateBank);
    rr.failedLayer = failIdx;
    rr.algorithmTimeSec = finalCore.configurationSearchTimeSec;
    rr.wallTimeSec = roundWall;
    rr.ikCalls = finalCore.continuationStats.ikCalls;
    rr.maxJointJumpDeg = finalCore.configurationMaxJointJumpDeg;
    rr.maxSegmentTcpErrorMm = finalCore.configurationTcpMetrics.maxSegmentPositionErrorMm;

    if finalCore.configurationComplete
        rounds(end+1,1) = rr; %#ok<AGROW>
        fprintf("[PASS] Sparse/adaptively refined seam is feasible at %d points.\n", ...
            size(currentPositions,1));
        break;
    end

    % A failure in the first layer is a seed/pose-branch issue, not a seam
    % spacing issue.  Give it one stronger first-point attempt before any
    % geometric subdivision.
    if isequal(failIdx,1) && ~strongFirstPointTried
        strongFirstPointTried = true;
        options.numRandomSeedsFirstPoint = max(options.numRandomSeedsFirstPoint,10);
        options.coarseTravelDeg = [-15 0 15];
        options.maxPoseSeeds = max(options.maxPoseSeeds,8);
        options.firstPointMaxStates = max(options.firstPointMaxStates,2);
        rr.recovery = "strong_first_point_retry";
        rounds(end+1,1) = rr; %#ok<AGROW>
        fprintf("[Recovery] first layer failed; retrying with stronger first-point seeds.\n");
        continue;
    end

    if isempty(failIdx) || failIdx <= 1 || size(currentPositions,1) >= options.maxPoints
        rr.recovery = "no_bisection_possible";
        rounds(end+1,1) = rr; %#ok<AGROW>
        fprintf("[STOP] No safe interval available for another refinement.\n");
        break;
    end

    % Bisect only the interval immediately preceding the failed layer.
    left = failIdx-1;
    right = failIdx;
    midpoint = 0.5*(currentPositions(left,:)+currentPositions(right,:));
    currentPositions = [currentPositions(1:left,:); midpoint; ...
        currentPositions(right:end,:)];
    rr.recovery = "bisect_failed_interval";
    rr.insertedPoint = midpoint;
    rr.insertedInterval = [left right];
    rr.pointsAfter = size(currentPositions,1);
    rounds(end+1,1) = rr; %#ok<AGROW>
    fprintf("[Refine] failed layer %d: bisect interval %d-%d -> %d points.\n", ...
        failIdx,left,right,size(currentPositions,1));
end

% Enter this stage only after sparse bisection has failed.  Normal samples
% therefore pay no beam/angular-redundancy cost.
postRecoveryUsed = false;
postRecoveryAttempts = 0;
postRecoveryTimeSec = 0;
postRecoveryIkCalls = 0;
if options.enablePostRecovery && ~finalCore.configurationComplete && ~isempty(currentPositions)
    fprintf("\n[Post-recovery] sparse refinement failed; trying bounded beam/angular recovery.\n");
    for ibeam = 1:numel(options.recoveryBeamWidths)
        if ~isempty(options.maxPlanningTimeSec) && toc(totalTimer) > options.maxPlanningTimeSec
            fprintf("[STOP] Cooperative time budget exhausted before beam=%d recovery.\n", ...
                options.recoveryBeamWidths(ibeam));
            budgetExceeded = true;
            break;
        end
        b = options.recoveryBeamWidths(ibeam);
        recoveryOptions = localCoreOptions(options);
        recoveryOptions.numPoints = size(currentPositions,1);
        recoveryOptions.seamPositionsWP = currentPositions;
        recoveryOptions.beamWidth = b;
        recoveryOptions.firstPointMaxStates = max(options.firstPointMaxStates,b);
        recoveryOptions.maxPoseSeeds = max(options.maxPoseSeeds,12);
        recoveryOptions.maxPoseNeighbors = options.recoveryMaxPoseNeighbors;
        recoveryOptions.poseNeighborhoodCost = options.recoveryPoseNeighborhoodCost;
        recoveryOptions.maxPoseStepRollDeg = options.recoveryMaxPoseStepRollDeg;
        recoveryOptions.maxPoseStepTravelDeg = options.recoveryMaxPoseStepTravelDeg;
        recoveryOptions.progressivePoseNeighbors = false;
        recoveryOptions.angularRedundancyFallback = true;
        recoveryOptions.angularFallbackRollDeg = options.recoveryRollFallbackDeg;
        recoveryOptions.angularFallbackTravelDeg = options.recoveryTravelFallbackDeg;
        recoveryOptions.enableFallback = false;
        recoveryFile = sprintf("%s_recovery_beam%d.mat", ...
            options.roundOutputPrefix,b);
        if ~startsWith(string(recoveryFile),filesep)
            recoveryFile = fullfile(here,recoveryFile);
        end
        recoveryOptions.outputFile = recoveryFile;
        postRecoveryAttempts = postRecoveryAttempts + 1;
        tRecovery = tic;
        candidate = replan_weld_actual_axis_adaptive_core(recoveryOptions);
        if isfield(candidate,"firstPointDiagnostics")
            latestPoint1Diagnostics = candidate.firstPointDiagnostics;
        end
        dtRecovery = toc(tRecovery);
        postRecoveryTimeSec = postRecoveryTimeSec + dtRecovery;
        postRecoveryIkCalls = postRecoveryIkCalls + candidate.continuationStats.ikCalls;
        postRecoveryUsed = true;
        rr = localEmptyRound();
        rr.round = options.maxRefinementRounds + ibeam;
        rr.pointsBefore = size(currentPositions,1);
        rr.pointsAfter = size(candidate.seam.positionsWP,1);
        rr.complete = candidate.configurationComplete;
        rr.furthestPoint = localFurthestPoint(candidate.stateBank);
        recoveryFail = localFirstEmptyLayer(candidate.stateBank);
        if isempty(recoveryFail), recoveryFail = NaN; end
        rr.failedLayer = recoveryFail;
        rr.algorithmTimeSec = candidate.configurationSearchTimeSec;
        rr.wallTimeSec = dtRecovery;
        rr.ikCalls = candidate.continuationStats.ikCalls;
        rr.maxJointJumpDeg = candidate.configurationMaxJointJumpDeg;
        rr.maxSegmentTcpErrorMm = candidate.configurationTcpMetrics.maxSegmentPositionErrorMm;
        rr.recovery = sprintf("post_recovery_beam_%d_angular",b);
        rounds(end+1,1) = rr; %#ok<AGROW>
        finalCore = candidate;
        currentPositions = candidate.seam.positionsWP;
        fprintf("[Post-recovery] beam=%d, complete=%d, points=%d, time=%.3fs\n", ...
            b,candidate.configurationComplete,size(currentPositions,1),dtRecovery);
        if candidate.configurationComplete, break; end
    end
end

result = finalCore;
result.algorithmName = "Adaptive Configuration Continuation Planning (ACCP)";
result.initialPoints = options.initialPoints;
result.finalPointCount = size(currentPositions,1);
result.finalSeamPositionsWP = currentPositions;
result.refinementRounds = rounds;
result.refinementRoundCount = numel(rounds);
result.feasibilityAdaptiveComplete = finalCore.configurationComplete;
result.feasibilityAdaptiveWallTimeSec = toc(totalTimer);
result.feasibilityAdaptiveAlgorithmTimeSec = sum([rounds.algorithmTimeSec]);
result.feasibilityAdaptiveIkCalls = sum([rounds.ikCalls]);
result.postRecoveryUsed = postRecoveryUsed;
result.postRecoveryAttempts = postRecoveryAttempts;
result.postRecoveryTimeSec = postRecoveryTimeSec;
result.postRecoveryIkCalls = postRecoveryIkCalls;
result.timeBudgetSec = localNumOrNaN(options.maxPlanningTimeSec);
result.timeBudgetExceeded = budgetExceeded;
% Record the branch reference actually used by the core.  This is provenance
% only; it never changes a hard constraint.
if isfield(result.scene,"branchRefQ")
    result.branchReferenceRad = double(result.scene.branchRefQ(:).');
    result.branchReferenceDeg = rad2deg(result.branchReferenceRad);
end
result.branchReferenceSource = char(string(options.branchReferenceSource));
% Persist the full first-point candidate audit separately from the MAT result.
% This is diagnostic data only; this function never promotes weld anchors.
if isfield(finalCore,"firstPointDiagnostics")
    latestPoint1Diagnostics = finalCore.firstPointDiagnostics;
end
if ~isempty(fieldnames(latestPoint1Diagnostics))
    [diagnosticsDir,~,~] = fileparts(options.outputFile);
    if isempty(diagnosticsDir), diagnosticsDir = here; end
    if ~isfolder(diagnosticsDir), mkdir(diagnosticsDir); end
    diagnosticsFile = fullfile(diagnosticsDir,"accp_point1_diagnostics.json");
    localWriteJson(diagnosticsFile,latestPoint1Diagnostics);
    result.point1DiagnosticsFile = diagnosticsFile;
    result.point1FilterCounts = localPoint1FilterCounts(latestPoint1Diagnostics);
else
    result.point1DiagnosticsFile = "";
    result.point1FilterCounts = struct();
end

% Optional comparison run.  This is deliberately a separate core invocation
% and never changes finalCore, anchors, safety gates, collision parameters,
% or the trajectory returned by the normal planner.
result.axisConstraintDiagnostic = localEmptyAxisConstraintDiagnostic();
if options.runAxisConstraintDiagnostic
    result.axisConstraintDiagnostic = localRunAxisConstraintDiagnostic( ...
        options,currentPositions,here);
end

% All precision outputs are inherited from ACCP core. Never run a separate
% postprocessor here or count the same precision stage twice.
timingCost=0;
if isfield(result,'timeScalingAlgorithmTimeSec') && isfinite(result.timeScalingAlgorithmTimeSec)
    timingCost=double(result.timeScalingAlgorithmTimeSec);
end
result.timeScalingAlgorithmTimeSec = timingCost;
result.algorithmElapsedSec = result.feasibilityAdaptiveAlgorithmTimeSec + result.precisionAlgorithmTimeSec + result.timeScalingAlgorithmTimeSec;
result.totalPlanningTimeSec = result.algorithmElapsedSec;
result.outputFile = localSaveResult(result,here,options.outputFile);

fprintf("\n========================================\n");
fprintf("ACCP result (configuration + embedded precision)\n");
fprintf("Initial points       : %d\n",options.initialPoints);
fprintf("Final points         : %d\n",result.finalPointCount);
fprintf("Refinement rounds    : %d\n",result.refinementRoundCount);
fprintf("Complete             : %d\n",result.feasibilityAdaptiveComplete);
fprintf("Configuration time   : %.3f s\n",result.feasibilityAdaptiveAlgorithmTimeSec);
fprintf("ACCP total time      : %.3f s\n",result.totalPlanningTimeSec);
fprintf("Adaptive time scale  : %.3f s\n",result.timeScalingAlgorithmTimeSec);
fprintf("Total wall time      : %.3f s\n",result.feasibilityAdaptiveWallTimeSec);
if isfield(result,"finalTcpMetrics")
    fprintf("Execution points     : %d\n",size(result.qPathExecution,1));
    fprintf("Precision points     : %d\n",result.precisionInsertedPoints);
    fprintf("Final segment TCP    : %.6f mm\n",max(result.finalTcpMetrics.segmentPositionErrorMm));
    fprintf("Precision PASS       : %d\n",result.adaptiveCorrectionComplete);
end
fprintf("Overall PASS         : %d\n",result.complete);
if isfield(result,"point1FilterCounts") && ~isempty(fieldnames(result.point1FilterCounts))
    c = result.point1FilterCounts;
    fprintf("Point1 diagnostics   : IK numeric=%d, pose OK=%d, direction reject=%d, position reject=%d, joint limit reject=%d\n", ...
        c.ik_numeric_success,c.ik_success,c.orientation_direction_rejected, ...
        c.position_error_rejected,c.joint_limit_rejected);
    fprintf("  continuity=%d, self=%d, gun=%d, workpiece=%d, accepted=%d\n", ...
        c.joint_continuity_rejected,c.self_collision_rejected, ...
        c.gun_collision_rejected,c.workpiece_collision_rejected, ...
        c.accepted_after_all_filters);
    fprintf("Point1 diagnostics JSON: %s\n",result.point1DiagnosticsFile);
end
if isfield(result,"axisConstraintDiagnostic") && ...
        result.axisConstraintDiagnostic.requested
    d = result.axisConstraintDiagnostic;
    fprintf("Axis-disabled diagnostic: complete=%d, point1 accepted=%d, output=%s\n", ...
        d.complete,d.point1Accepted,d.diagnosticsFile);
end
fprintf("Saved result         : %s\n",result.outputFile);
end

function opts = localCoreOptions(options)
names = { ...
    "numPoints","endMarginM","coarseTravelDeg","coarseRollDeg", ...
    "beamWidth","firstPointMaxStates","maxPoseSeeds","seedPerturbDeg", ...
    "maxPoseNeighbors","maxPoseStepRollDeg","maxPoseStepTravelDeg", ...
    "numRandomSeedsFirstPoint","maxRollStepDeg","maxTravelStepDeg", ...
    "maxJointStepDeg","edgeResolutionDeg","maxBacktrackLayers", ...
    "positionToleranceM","orientationToleranceDeg","sideToleranceDeg", ...
    "sideMarginM","checkSelfCollision","animate","publish", ...
    "disableAxisConstraint", ...
    "wire_collision_shorten_mm", ...
    "axis_angle_tolerance_deg", ...
    "candidate_scoring_mode","candidate_diagnostics","candidate_score_weights", ...
    "reuseIKSolver","reuseCandidateCache","lazyCollision", ...
    "progressivePoseNeighbors","progressiveInitialPoseNeighbors", ...
    "progressiveMaxPoseNeighbors","disableLocalRecovery", ...
    "poseNeighborhoodCost","angularRedundancyFallback", ...
    "angularFallbackRollDeg","angularFallbackTravelDeg","enableFallback", ...
    "randomSeed","outputFile","seamPositionsWP","branchReferenceQ","sceneScript", ...
    "maxSegmentTcpErrorMm","maxIntermediatePoints","errorSamplesPerEdge", ...
    "timeScalingMaxVelocityRadPerSec","timeScalingMaxAccelerationRadPerSec2", ...
    "timeScalingMaxDurationSec","timeScalingLambda"};
opts = struct();
for i=1:numel(names)
    name = names{i};
    if isfield(options,name), opts.(name) = options.(name); end
end
end

function idx = localFirstEmptyLayer(stateBank)
idx = find(cellfun(@isempty,stateBank),1,"first");
end

function idx = localFurthestPoint(stateBank)
idx = find(~cellfun(@isempty,stateBank),1,"last");
if isempty(idx), idx = 0; end
end

function r = localEmptyRound()
r = struct("round",NaN,"pointsBefore",NaN,"pointsAfter",NaN, ...
    "complete",false,"furthestPoint",NaN,"failedLayer",NaN, ...
    "algorithmTimeSec",NaN,"wallTimeSec",NaN,"ikCalls",NaN, ...
    "maxJointJumpDeg",NaN,"maxSegmentTcpErrorMm",NaN, ...
    "recovery","","insertedPoint",[NaN NaN NaN], ...
    "insertedInterval",[NaN NaN]);
end

function out = localMergeOptions(defaults,user)
out = defaults;
if isempty(user), return; end
names = fieldnames(user);
for i=1:numel(names), out.(names{i}) = user.(names{i}); end
end

function v = localNumOrNaN(value)
if isempty(value), v = NaN; else, v = double(value); end
end

function file = localSaveResult(result,here,fileName)
file = fileName;
if ~isfile(file) && ~startsWith(string(file),filesep)
    file = fullfile(here,file);
end
save(file,"result","-v7.3");
end

function counts = localPoint1FilterCounts(diagnostics)
names = {"ik_numeric_success","ik_success", ...
    "orientation_direction_rejected","position_error_rejected", ...
    "orientation_error_rejected","joint_limit_rejected", ...
    "joint_continuity_rejected","self_collision_rejected", ...
    "gun_collision_rejected","workpiece_collision_rejected", ...
    "welding_side_position_rejected","accepted_after_all_filters"};
counts = struct();
for i=1:numel(names)
    name = names{i};
    if isfield(diagnostics,name), counts.(name) = diagnostics.(name); else, counts.(name) = 0; end
end
end

function diagnostic = localEmptyAxisConstraintDiagnostic()
diagnostic = struct( ...
    "schema_version","ACCP_AXIS_CONSTRAINT_DIAGNOSTIC_v1", ...
    "requested",false, ...
    "disable_axis_constraint",false, ...
    "complete",false, ...
    "point1Accepted",0, ...
    "finalPointCount",0, ...
    "reason","not requested", ...
    "coreResultFile","", ...
    "diagnosticsFile","");
end

function diagnostic = localRunAxisConstraintDiagnostic(options,currentPositions,here)
diagnostic = localEmptyAxisConstraintDiagnostic();
diagnostic.requested = true;
diagnostic.disable_axis_constraint = true;
diagDir = fileparts(options.outputFile);
if isempty(diagDir), diagDir = here; end
diagDir = fullfile(diagDir,"axis_constraint_disabled");
if ~isfolder(diagDir), mkdir(diagDir); end
coreFile = fullfile(diagDir,"accp_axis_constraint_disabled_core.mat");
jsonFile = fullfile(diagDir,"accp_axis_disabled_point1_diagnostics.json");
diagOptions = localCoreOptions(options);
% Axis-disabled comparison is isolated and never uses formal scored selection.
diagOptions.candidate_scoring_mode = false;
diagOptions.candidate_diagnostics = false;
diagOptions.disableAxisConstraint = true;
diagOptions.configurationOnlyDiagnostic = true;
diagOptions.enableFallback = false;
diagOptions.animate = false;
diagOptions.publish = false;
diagOptions.seamPositionsWP = currentPositions;
diagOptions.numPoints = size(currentPositions,1);
diagOptions.outputFile = coreFile;
try
    core = replan_weld_actual_axis_adaptive_core(diagOptions);
    diagnostic.complete = logical(core.configurationComplete);
    diagnostic.finalPointCount = size(core.seam.positionsWP,1);
    if isfield(core,"firstPointDiagnostics")
        diagnostic.point1Accepted = core.firstPointDiagnostics.accepted_after_all_filters;
        point1 = core.firstPointDiagnostics;
    else
        point1 = struct();
    end
    diagnostic.coreResultFile = coreFile;
    if diagnostic.complete
        diagnostic.reason = "ACCP completed with axis constraint disabled; inspect mount-axis calibration.";
    elseif diagnostic.point1Accepted == 0
        diagnostic.reason = "Point1 still infeasible with axis constraint disabled; failure is not explained by the axis gate alone.";
    else
        diagnostic.reason = "Point1 accepted with axis constraint disabled, but continuation/collision/continuity failed later.";
    end
    payload = struct( ...
        "schema_version",diagnostic.schema_version, ...
        "disable_axis_constraint",true, ...
        "complete",diagnostic.complete, ...
        "point1_accepted",diagnostic.point1Accepted, ...
        "final_point_count",diagnostic.finalPointCount, ...
        "reason",diagnostic.reason, ...
        "core_result_file",coreFile, ...
        "point1_diagnostics",point1);
    localWriteJson(jsonFile,payload);
    diagnostic.diagnosticsFile = jsonFile;
catch ME
    diagnostic.reason = "diagnostic core failed: " + string(ME.message);
    payload = struct("schema_version",diagnostic.schema_version, ...
        "disable_axis_constraint",true,"complete",false, ...
        "point1_accepted",0,"final_point_count",0, ...
        "reason",diagnostic.reason,"core_result_file",coreFile, ...
        "point1_diagnostics",struct());
    localWriteJson(jsonFile,payload);
    diagnostic.diagnosticsFile = jsonFile;
end
end

function localWriteJson(file,payload)
fid = fopen(file,'w');
if fid < 0, error("无法写入 ACCP 点1诊断文件: %s",file); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(payload),'char');
end

