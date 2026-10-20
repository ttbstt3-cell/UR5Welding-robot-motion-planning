function result = bridge_capture_core(options)
%REPLAN_WELD_ACTUAL_AXIS_ADAPTIVE_CORE ACCP configuration and precision core.
% ACCP owns pose constraints, adaptive configuration continuation, Cartesian
% position/SLERP refinement, incremental IK and continuous collision checks.
% qPath is the validated high-precision output; qPathInitial records the
% sparse configuration chain. Refinement is not a separate planning algorithm.
%
% This independent experiment does not load a previous qPath/state graph.
% It keeps a small beam of collision-free IK branches and propagates them
% with local pose neighborhoods plus a constant-velocity joint prediction.
% The original layered planner is available as an explicit fallback only.
%
% This is an independent baseline: no previous weld path or state graph is
% read.  The MATLAB wire collision cylinder from sync_ur5_joint_states is
% included in every environment/self collision check.  The stable ROS2
% controller and the CoppeliaSim scene are not modified.

if nargin < 1 || isempty(options), options = struct(); end
% The 25 mm trim is required by the imported conservative V-HACD hull to
% keep the gun body clear of the x=0/x=L end walls.  Override only if a
% smaller physical end clearance has been validated separately.
% Formal real-scene ACCP default: shorten only the collision-cylinder length
% by 3 mm. The visual wire, TCP, IK data and thresholds remain unchanged.
defaults = struct( ...
    "sceneScript","", ...
    "numPoints",80,"endMarginM",0.025,"seamPositionsWP",[], ...
    "coarseTravelDeg",-15:15:15,"coarseRollDeg",0:45:315, ...
    "beamWidth",6,"firstPointMaxStates",6, ...
    "maxPoseSeeds",8,"seedPerturbDeg",8, ...
    "maxPoseNeighbors",9,"maxPoseStepRollDeg",15, ...
    "maxPoseStepTravelDeg",5,"numRandomSeedsFirstPoint",10, ...
    "maxRollStepDeg",45,"maxTravelStepDeg",15,"maxJointStepDeg",15, ...
    "edgeResolutionDeg",3,"maxBacktrackLayers",8, ...
    "positionToleranceM",1e-4,"orientationToleranceDeg",0.5, ...
    "sideToleranceDeg",5,"sideMarginM",1e-3, ...
    "axis_angle_tolerance_deg",[], ...
    "checkSelfCollision",true,"animate",false,"publish",false, ...
    "reuseIKSolver",true,"reuseCandidateCache",true, ...
    "lazyCollision",true, ...
    "disableAxisConstraint",false, ...
    "wire_collision_shorten_mm",3, ...
    "disable_axis_constraint",false, ...
    "candidate_scoring_mode",false,"candidate_diagnostics",false, ...
    "candidate_score_weights",[1 1 0.1 0.1], ...
    "progressivePoseNeighbors",false, ...
    "progressiveInitialPoseNeighbors",5, ...
    "progressiveMaxPoseNeighbors",9, ...
    "disableLocalRecovery",false, ...
    "poseNeighborhoodCost","zero", ...
    "angularRedundancyFallback",false, ...
    "angularFallbackRollDeg",[10 20], ...
    "angularFallbackTravelDeg",[0 5 10], ...
    "initialStateBank",[],"initialStartLayer",1, ...
    "enableFallback",true, ...
    "enableCartesianBridgeFallback",true, ...
    "cartesianBridgeFractions",[0.25 0.50 0.75], ...
    "cartesianBridgeMaxDepth",2, ...
    "cartesianBridgeMaxIKCalls",3000, ...
    "cartesianBridgeMaxTimeSec",60, ...
    "cartesianBridgeMaxEdgeSamples",101, ...
    "cartesianBridgeBeamWidth",24, ...
    "cartesianBridgeMaxPaths",6, ...
    "cartesianBridgeRandomSeeds",24, ...
    "cartesianBridgeJointPerturbDeg",[15 30], ...
    "branchReferenceQ",[], ...
    "configurationOnlyDiagnostic",false, ...
    "maxSegmentTcpErrorMm",0.5,"maxIntermediatePoints",4, ...
    "errorSamplesPerEdge",21, ...
    "timeScalingMaxVelocityRadPerSec",deg2rad(5), ...
    "timeScalingMaxAccelerationRadPerSec2",deg2rad(15), ...
    "timeScalingMaxDurationSec",50,"timeScalingLambda",0.001, ...
    "randomSeed",20260923,"outputFile", ...
    "weld_actual_axis_replan_continuation.mat");
options = localMergeOptions(defaults,options);
if ~isempty(options.axis_angle_tolerance_deg)
    validateattributes(options.axis_angle_tolerance_deg,{'numeric'}, ...
        {'scalar','real','finite','>=',0,'<=',180});
    options.sideToleranceDeg=double(options.axis_angle_tolerance_deg);
end
options.disableAxisConstraint = logical(options.disableAxisConstraint || ...
    options.disable_axis_constraint);
if options.configurationOnlyDiagnostic && (options.animate || options.publish)
    error('ACCP:DiagnosticMotion','Configuration-only diagnosis cannot animate/publish a trajectory.');
end

matlabDir = fileparts(mfilename("fullpath"));
rootDir = fileparts(matlabDir); %#ok<NASGU>
fprintf("\n========================================\n");
fprintf("REPLAN WELD WITH LOCAL IK CONTINUATION\n");
fprintf("========================================\n");
fprintf("No previous weld path is used as an input.\n");

sceneScript=fullfile(matlabDir,"sync_ur5_joint_states.m");
if isfield(options,'sceneScript') && ~isempty(options.sceneScript), sceneScript=options.sceneScript; end
[scene] = localLoadScene(sceneScript);
scene=apply_wire_collision_shorten_diagnostic(scene,options.wire_collision_shorten_mm);
% rigidBodyTree uses row DataFormat. Preserve all saved joint values; only
% normalize YAML-derived RealHome vector shape for collision/FK inputs.
scene.qMat0=reshape(scene.qMat0,1,[]);
validateattributes(scene.qMat0,{'numeric'},{'numel',6,'finite'});
scene.selfCollisionSkipPairs = localStructuralSelfCollisionSkips(scene.robot);
scene.selfCollisionSkipPairs = ignore_fixed_gun_wire_self_collision(scene.robot,scene.selfCollisionSkipPairs);

% Branch reference.  It only steers which IK branch is kept/expanded; it does
% not change any hard constraint.  The default is scene.qMat0, which is the
% historical behavior for every existing experiment.  Supplying
% branchReferenceQ (for example the validated qWeldStart anchor) makes the
% first-point seed, the continuation cost and the beam selection prefer the
% branch family that passes through that configuration.  Start-connectivity
% reporting in localBuildGraph still uses scene.qMat0 on purpose.
scene.branchRefQ = localBranchReference(scene,options);
if options.candidate_scoring_mode || options.candidate_diagnostics
    if options.disableAxisConstraint
        error('ACCP:ScoringAxis','Candidate selection diagnostics require the formal axis constraint.');
    end
    validateattributes(options.candidate_score_weights,{'numeric'}, ...
        {'vector','numel',4,'finite','nonnegative'});
    scene.candidateAuditStore = containers.Map('KeyType','double','ValueType','any');
end

% This experiment uses one IK solver, one joint-limit vector and one cache.
if options.reuseIKSolver || options.reuseCandidateCache
    scene.ikSolver = inverseKinematics("RigidBodyTree",scene.robot);
    scene.jointLimits = localRobotJointLimits(scene.robot);
end
if options.reuseCandidateCache
    scene.ikCandidateCache = containers.Map( ...
        "KeyType","char","ValueType","any");
end
% U-interior side for bottom x wall_y1: y < inner wall and z > bottom top.
scene.weldGunAxisWP = [0;1;-1]/sqrt(2);
scene.weldGunAxisName = "measured_wire_axis_target";
scene.weldingSide = "U-interior: y < wall_y1 inner face, z > bottom top face";
scene.weldingContactPolicy = ...
    "Wire/environment contact is hard; no target-seam collision exemption";

scene.analyticalIK = localCheckAnalyticalIK(scene.robot);
fprintf("Analytical IK valid for loaded universalUR5: %d\n", ...
    scene.analyticalIK.isValidGroupForIK);
if ~scene.analyticalIK.isValidGroupForIK
    fprintf("Analytical IK is not used; numeric IK remains the validated solver.\n");
end

% Algorithm-only timer: scene construction/model loading above is excluded.
% The measured interval covers seam generation, candidate IK, collision
% checks, graph construction/refinement and final path search.
algorithmTimer = tic;
fprintf("ACCP_STAGE stage=CONTINUOUS_IK\n");

fprintf("\nSelf-collision structural exclusions (attachment only):\n");
for k = 1:size(scene.selfCollisionSkipPairs,1)
    fprintf("  %s <-> %s\n",scene.selfCollisionSkipPairs{k,1},scene.selfCollisionSkipPairs{k,2});
end
fprintf("All other self-collision pairs remain hard constraints.\n");

if isempty(options.seamPositionsWP)
    seam = localInferSeam(scene,options.numPoints,options.endMarginM);
else
    seam = localBuildCustomSeam(scene,options.seamPositionsWP);
end
% Keep the unmodified seam count for the comparison report.  A Cartesian
% bridge may insert diagnostic/validated intermediate layers, but it must
% never overwrite the original weld anchors.
originalSeamPointCount = seam.numPoints;
cartesianBridge = localEmptyCartesianBridgeReport();
tangentWP = seam.tangentWP(:);
% Expose the geometric seam tangent to the Point-1 diagnostic records only.
% It is not used to alter pose generation or any formal safety gate.
scene.weldTangentWP = tangentWP;
scene.auditSeamPositions = seam.positionsMat;
selectionDiagnostics = cell(1,seam.numPoints);
gunAxisWP0 = scene.weldGunAxisWP(:);
fprintf("\nPhysical welding frame:\n");
fprintf("  seam = bottom x wall_y1\n");
fprintf("  start WP = [%.4f %.4f %.4f] m\n",seam.startWP);
fprintf("  end   WP = [%.4f %.4f %.4f] m\n",seam.endWP);
fprintf("  tangent = [%.4f %.4f %.4f]\n",tangentWP);
fprintf("  measured gun axis target = [%.4f %.4f %.4f]\n",gunAxisWP0);
fprintf("  work angle to bottom normal = %.2f deg\n", ...
    rad2deg(acos(abs(dot(gunAxisWP0,[0;0;-1])))));
fprintf("  plate thickness = %.1f mm\n",seam.plateThicknessM*1000);
fprintf("  welding side = %s\n",scene.weldingSide);

rng(options.randomSeed,"twister");
stateBank = cell(1,seam.numPoints);
layerStats = repmat(localEmptyLayerStats(),1,seam.numPoints);

fprintf("\n--- Cold-start bounded continuation ---\n");
stats = localEmptyContinuationStats();
startLayer = 1;
if ~isempty(options.initialStateBank)
    % Optional experiment-only seeded prefix.  Default behavior remains a
    % cold start.  The caller is responsible for supplying already checked
    % states and a valid prefix; all subsequent layers still use the normal
    % continuation and edge checks.
    seeded = options.initialStateBank;
    if ~iscell(seeded), error("initialStateBank must be a cell array."); end
    if numel(seeded) > seam.numPoints, error("initialStateBank is longer than the seam."); end
    for k = 1:numel(seeded), stateBank{k} = seeded{k}; end
    startLayer = min(max(1,round(options.initialStartLayer)),seam.numPoints);
    fprintf("Seeded continuation prefix: layers 1..%d\n",startLayer);
    for k = 1:startLayer
        if isempty(stateBank{k}), error("initialStateBank has an empty layer %d.",k); end
        layerStats(k).coarseStates = numel(stateBank{k});
    end
else
    [tLayer] = tic;
    [stateBank{1},firstStats,firstPointDiagnostics] = ...
        localGenerateFirstBeam(scene,seam,options);
    stats = localAddContinuationStats(stats,firstStats);
    [stateBank{1},selectionDiagnostics{1}] = localSelectCandidates( ...
        scene,seam,1,stateBank{1},[],options.firstPointMaxStates,options);
    layerStats(1).coarseStates = numel(stateBank{1});
    layerStats(1).elapsedSec = toc(tLayer);
    layerStats(1).ikCalls = firstStats.ikCalls;
    layerStats(1).ikElapsedSec = firstStats.ikElapsedSec;
    layerStats(1).edgeTests = firstStats.edgeTests;
    layerStats(1).edgeCollisionChecks = firstStats.edgeCollisionChecks;
    fprintf("Point %2d/%2d: beam states = %d\n",1,seam.numPoints,numel(stateBank{1}));
end

% A bridge inserts layers into stateBank/seam, so this is intentionally a
% bounded while-loop rather than a fixed for-loop.  The loop still visits
% every original weld layer after the recovered endpoint.
i = startLayer + 1;
while i <= seam.numPoints
    tLayer = tic;
    layerOptions = options;
    if options.progressivePoseNeighbors
        layerOptions.maxPoseNeighbors = options.progressiveInitialPoseNeighbors;
        layerOptions.disableLocalRecovery = true;
    end
    if options.enableCartesianBridgeFallback
        % Retain the numeric/pose-valid candidate population at a failed
        % layer so the bridge can reconnect branches that the normal direct
        % edge gate intentionally discarded.
        layerOptions.diagnosticCollectIKCandidates = true;
    end
    [states,stepStats] = localGenerateContinuationLayer( ...
        scene,seam,i,stateBank{i-1},layerOptions);
    if isempty(states) && options.progressivePoseNeighbors
        % Expand only the failed layer.  Keep all hard pose, IK, joint-limit
        % and full node/edge collision constraints unchanged.
        expandedOptions = options;
        expandedOptions.maxPoseNeighbors = options.progressiveMaxPoseNeighbors;
        expandedOptions.disableLocalRecovery = false;
        if options.enableCartesianBridgeFallback
            expandedOptions.diagnosticCollectIKCandidates = true;
        end
        [expandedStates,expandedStats] = localGenerateContinuationLayer( ...
            scene,seam,i,stateBank{i-1},expandedOptions);
        stepStats = localAddContinuationStats(stepStats,expandedStats);
        states = expandedStates;
    end
    stats = localAddContinuationStats(stats,stepStats);
    [stateBank{i},selectionDiagnostics{i}] = localSelectCandidates( ...
        scene,seam,i,states,stateBank{i-1},options.beamWidth,options);
    layerStats(i).coarseStates = numel(stateBank{i});
    layerStats(i).elapsedSec = toc(tLayer);
    layerStats(i).ikCalls = stepStats.ikCalls;
    layerStats(i).ikElapsedSec = stepStats.ikElapsedSec;
    layerStats(i).edgeTests = stepStats.edgeTests;
    layerStats(i).edgeCollisionChecks = stepStats.edgeCollisionChecks;
    if mod(i,5)==0 || i==seam.numPoints || isempty(stateBank{i})
        fprintf("Point %2d/%2d: beam states = %d, IK calls = %d\n", ...
            i,seam.numPoints,numel(stateBank{i}),stats.ikCalls);
    end
    if isempty(stateBank{i})
        fprintf("Continuation broke at point %d: NO_CONNECTED_CANDIDATE.\n",i);
        if options.enableCartesianBridgeFallback
            pointIndex=i; prev=stateBank{i-1}; rng_state=rng;
            candidatePools=stepStats.diagnosticCandidatePools;
            save('/home/ttbstt/jixiebi/UR5/规划文件/workflow_runs/bridge_struct_fix_20261014/point14_input_fixture.mat','scene','seam','pointIndex','prev','options', ...
                'candidatePools','stateBank','layerStats','stepStats','rng_state','-v7.3');
            error('TEST:BridgeInputCaptured','Captured real failure before private bridge call');
            [bridge,bridgeStats] = localCartesianBridgeFallback( ...
                scene,seam,i,stateBank{i-1},options,stepStats.diagnosticCandidatePools);
            cartesianBridge = localMergeCartesianBridgeReport( ...
                cartesianBridge,bridgeStats);
            if bridge.success
                [stateBank,seam,layerStats,selectionDiagnostics] = ...
                    localInsertCartesianBridge(scene,stateBank,seam,layerStats, ...
                    selectionDiagnostics,i,bridge,options);
                cartesianBridge.recovered_point_indices(end+1) = i; %#ok<AGROW>
                cartesianBridge.recovery_count = cartesianBridge.recovery_count + 1;
                % The endpoint is now at i+numel(fractions); continue with
                % the next original weld point.  All inserted nodes remain
                % in the graph and are rechecked by localBuildGraph.
                i = i + bridge.inserted_layer_count + 1;
                continue;
            end
            cartesianBridge.failed_point_indices(end+1) = i; %#ok<AGROW>
            fprintf("Cartesian Bridge did not recover point %d: %s\n", ...
                i,bridge.failure_reason);
        end
        break;
    end
    i = i + 1;
end

if all(~cellfun(@isempty,stateBank))
    [graph,plan] = localBuildGraph(scene,stateBank,seam,options);
else
    graph = localEmptyGraph(seam.numPoints);
    plan = localEmptyPlan();
end

fallbackUsed = false;
fallbackResult = struct();
if (~plan.complete || isempty(plan.qPath)) && options.enableFallback
    fprintf("\n[Fallback] Calling original layered planner from a fresh scene.\n");
    fallbackOptions = struct( ...
        "animate",false,"publish",false,"reuseIKSolver",true, ...
        "reuseCandidateCache",true,"lazyCollision",true, ...
        "randomSeed",options.randomSeed, ...
        "outputFile","weld_actual_axis_replan_continuation_fallback.mat");
    fallbackResult = replan_weld_actual_axis(fallbackOptions);
    fallbackUsed = true;
    if fallbackResult.complete
        stateBank = fallbackResult.stateBank;
        graph = fallbackResult.graph;
        plan.qPath = fallbackResult.qPath;
        plan.complete = fallbackResult.complete;
        plan.nodeCollisionFree = fallbackResult.nodeCollisionFree;
        plan.edgeCollisionFree = fallbackResult.edgeCollisionFree;
        plan.maxJointJumpDeg = fallbackResult.maxJointJumpDeg;
    end
end

algorithmElapsedSec = toc(algorithmTimer);

result = struct();
sceneResult = scene;
if isfield(sceneResult,"candidateAuditStore")
    sceneResult = rmfield(sceneResult,"candidateAuditStore");
end
% Do not serialize performance-only handle objects into the result file.
if isfield(sceneResult,"ikSolver")
    sceneResult = rmfield(sceneResult,"ikSolver");
end
if isfield(sceneResult,"jointLimits")
    sceneResult = rmfield(sceneResult,"jointLimits");
end
if isfield(sceneResult,"ikCandidateCache")
    sceneResult = rmfield(sceneResult,"ikCandidateCache");
end
if isfield(sceneResult,"analyticalIK")
    sceneResult = rmfield(sceneResult,"analyticalIK");
end
result.scene = sceneResult;
result.seam = seam;
result.originalSeamPointCount = originalSeamPointCount;
result.stateBank = stateBank;
result.layerStats = layerStats;
result.graph = graph;
result.options = options;
if isfield(result.options,"initialStateBank")
    result.options = rmfield(result.options,"initialStateBank");
end
result.qPath = plan.qPath;
result.complete = plan.complete;
result.nodeCollisionFree = plan.nodeCollisionFree;
result.edgeCollisionFree = plan.edgeCollisionFree;
result.ikSuccessAll = all(~cellfun(@isempty,stateBank));
result.maxJointJumpDeg = plan.maxJointJumpDeg;
result.actualGunAxisTcp = scene.wireAxisTcp;
result.gunAxisTargetWP = scene.weldGunAxisWP;
result.weldingSideConstraint = scene.weldingSide;
result.approachPathPlanned = false;
result.exitPathPlanned = false;
result.startConnectedStateCount = graph.startConnectedCount;
result.startConnectivityChecked = true;
result.approachExitNote = "自由空间接近/收焊退出未纳入ACCP焊缝轨迹，需单独规划。";
result.algorithmElapsedSec = algorithmElapsedSec;
result.totalPlanningTimeSec = algorithmElapsedSec;
result.continuationStats = stats;
result.cartesianBridge = cartesianBridge;
result.cartesianBridge.executionReady = false;
result.cartesianBridge.note = ["Bounded offline fallback only; dense samples do not prove " ...
    "continuous collision safety and wire_collision_shorten_mm=3 remains an approximate parameter."];
if options.candidate_scoring_mode || options.candidate_diagnostics
    for auditPoint=1:numel(selectionDiagnostics)
        if isempty(selectionDiagnostics{auditPoint})
            selectionDiagnostics{auditPoint}=struct('point_index',auditPoint, ...
                'point_status','NOT_VISITED','candidates',[]);
        end
    end
end
result.candidateSelectionDiagnostics = selectionDiagnostics;
result.candidateScoringMode = logical(options.candidate_scoring_mode);
if options.candidate_scoring_mode || options.candidate_diagnostics
    auditDir = fileparts(options.outputFile);
    if isempty(auditDir), auditDir = matlabDir; end
    if ~isfolder(auditDir), mkdir(auditDir); end
    [~,auditStem]=fileparts(options.outputFile);
    result.candidateSelectionDiagnosticsFile = fullfile(auditDir, ...
        [char(auditStem) '_candidate_selection_diagnostics.json']);
    payload = struct('schema_version','ACCP_CANDIDATE_SELECTION_v1', ...
        'scene_script',sceneScript,'axis_constraint_enabled',true, ...
        'candidate_scoring_mode',logical(options.candidate_scoring_mode), ...
        'weights',options.candidate_score_weights,'points',{selectionDiagnostics}, ...
        'complete',logical(plan.complete));
    if isfield(scene,'calibrationBinding'), payload.calibration_binding=scene.calibrationBinding; end
    fid=fopen(result.candidateSelectionDiagnosticsFile,'w');
    if fid<0, error('Cannot write candidate selection diagnostics'); end
    cleanup=onCleanup(@() fclose(fid));
    fwrite(fid,jsonencode(payload),'char'); clear cleanup;
end
if exist("firstPointDiagnostics","var")
    result.firstPointDiagnostics = firstPointDiagnostics;
else
    result.firstPointDiagnostics = localEmptyFirstPointDiagnostics();
end
result.fallbackUsed = fallbackUsed;
result.analyticalIK = scene.analyticalIK;
result.tcpMetrics = localComputeTcpPathErrors(scene,seam,stateBank,plan,options);

% ACCP final precision stage: invoked here once, after a feasible sparse
% branch is selected. The outer feasibility coordinator never refines again.
result.algorithmName = "Adaptive Configuration Continuation Planning (ACCP)";
result.configurationComplete = logical(plan.complete);
result.configurationSearchTimeSec = algorithmElapsedSec;
result.qPathInitial = plan.qPath;
result.configurationTcpMetrics = result.tcpMetrics;
result.configurationNodeCollisionFree = plan.nodeCollisionFree;
result.configurationEdgeCollisionFree = plan.edgeCollisionFree;
result.configurationMaxJointJumpDeg = plan.maxJointJumpDeg;
result.qPathExecution = zeros(0,6);
result.tcpPath = zeros(0,3);
result.refinedTcpPath = zeros(0,3);
result.tcpPosePath = cell(0,1);
result.tcpTargetExecution = cell(0,1);
result.cartesianRefinement = struct();
result.adaptiveCorrectionComplete = false;
result.adaptiveIkCalls = 0; result.adaptiveIkElapsedSec = 0;
result.adaptiveCollisionChecks = 0; result.precisionAlgorithmTimeSec = 0;
result.precisionInsertedPoints = 0;
result.precisionStageInvocationCount = 0;
result.highPrecisionPathAvailable = false;
result.complete = false;
if plan.complete && ~options.configurationOnlyDiagnostic && ~options.disableAxisConstraint
    tPrecision = tic;
    fprintf("ACCP_STAGE stage=CARTESIAN_REFINEMENT\n");
    % This temporary copy carries only the already validated sparse chain.
    sparse = result; sparse.qPath = plan.qPath; sparse.complete = true;
    precision = refine_accp_cartesian_path(sparse,options);
    result.precisionStageInvocationCount = 1;
    result.precisionAlgorithmTimeSec = toc(tPrecision);
    result.cartesianRefinement = precision;
    result.initialTcpMetrics = precision.initialTcpMetrics;
    result.finalTcpMetrics = precision.finalTcpMetrics;
    result.adaptiveSegmentReport = precision.segmentReport;
    result.adaptiveCorrectionComplete = precision.complete;
    result.adaptiveIkCalls = precision.ikCalls;
    result.adaptiveIkElapsedSec = precision.ikElapsedSec;
    result.adaptiveCollisionChecks = precision.collisionChecks;
    result.precisionInsertedPoints = size(precision.qPathExecution,1)-size(plan.qPath,1);
    result.complete = logical(precision.complete);
    result.highPrecisionPathAvailable = result.complete;
    result.nodeCollisionFree = precision.finalTcpMetrics.nodeCollisionFree;
    result.edgeCollisionFree = precision.finalTcpMetrics.edgeCollisionFree;
    result.maxJointJumpDeg = max(precision.finalTcpMetrics.segmentJointJumpDeg);
    result.tcpMetrics = precision.finalTcpMetrics;
    result.tcpMetrics.maxNodePositionErrorMm = max(precision.finalTcpMetrics.nodePositionErrorMm);
    result.tcpMetrics.maxNodeOrientationErrorDeg = max(precision.finalTcpMetrics.nodeOrientationErrorDeg);
    result.tcpMetrics.maxSegmentPositionErrorMm = max(precision.finalTcpMetrics.segmentPositionErrorMm);
    result.tcpMetrics.maxSegmentOrientationErrorDeg = max(precision.finalTcpMetrics.segmentOrientationErrorDeg);
    if result.complete
        result.qPath = precision.qPathExecution;
        result.qPathExecution = result.qPath; % Legacy HMI/anchor-loader alias.
        result.tcpPosePath = precision.tcpTargetExecution;
        result.tcpTargetExecution = result.tcpPosePath;
        result.tcpPath = cell2mat(cellfun(@(T)T(1:3,4).',result.tcpPosePath,'UniformOutput',false));
        result.refinedTcpPath = result.tcpPath;
        % ACCP Stage 3: adaptive time scaling.  The validated qPath and TCP
        % targets are immutable; only node timestamps are optimized here.
        fprintf("ACCP_STAGE stage=ADAPTIVE_TIME_SCALING\n");
        result.timeParameterization = adaptive_time_scale_accp_output(...
            result.qPath,result.tcpPosePath,result.scene,struct(...
            'maxVelocityRadPerSec',options.timeScalingMaxVelocityRadPerSec,...
            'maxAccelerationRadPerSec2',options.timeScalingMaxAccelerationRadPerSec2,...
            'maxDurationSec',options.timeScalingMaxDurationSec,...
            'lambda',options.timeScalingLambda,...
            'collisionPass',result.nodeCollisionFree && result.edgeCollisionFree,...
            'collisionSampleCount',result.adaptiveCollisionChecks));
        result.timeFromStart = result.timeParameterization.timeFromStart;
        result.velocities = result.timeParameterization.velocities;
        result.accelerations = result.timeParameterization.accelerations;
        result.timeParameterizationComplete = result.timeParameterization.complete;
        result.timeParameterizationFailureReason = result.timeParameterization.failureReason;
        % This invocation is an offline planner/diagnostic.  Even if the
        % numerical time-parameterization checks pass, it must not become an
        % executable real trajectory without the existing HMI final gate and
        % human confirmation.
        result.timeParameterization.executionReady = false;
        result.executionReady = false;
        if ~result.timeParameterizationComplete
            result.complete = false;
            result.highPrecisionPathAvailable = false;
            result.executionReady = false;
        end
    else
        result.qPath = zeros(0,6); % Failed precision candidates remain diagnostic only.
        result.refinedTcpPath = zeros(0,3);
        result.timeParameterization = struct('stage','Adaptive Time Scaling',...
            'complete',false,'executionReady',false,...
            'failureReason','ACCP_TIME_PARAMETERIZATION_SKIPPED: Cartesian refinement did not pass.');
        result.timeParameterizationComplete = false;
        result.executionReady = false;
        result.timeFromStart = zeros(0,1); result.velocities=zeros(0,6); result.accelerations=zeros(0,6);
    end
elseif options.configurationOnlyDiagnostic
    % Explicit historical search diagnosis, never a high-precision PASS.
    result.qPath = plan.qPath;
    result.executionReady = false;
else
    result.qPath = zeros(0,6);
    result.executionReady = false;
end
result.timeScalingAlgorithmTimeSec = 0;
if isfield(result,'timeParameterization') && isfield(result.timeParameterization,'algorithmTimeSec')
    result.timeScalingAlgorithmTimeSec = result.timeParameterization.algorithmTimeSec;
end
result.algorithmElapsedSec = result.configurationSearchTimeSec + result.precisionAlgorithmTimeSec + result.timeScalingAlgorithmTimeSec;
result.totalPlanningTimeSec = result.algorithmElapsedSec;
result.algorithmStages = {"pose_constraints","adaptive_configuration_continuation", ...
    "cartesian_refinement_incremental_ik","adaptive_time_scaling",...
    "continuous_collision_validation"};
result.acppStages = {"Continuous IK","Cartesian Refinement","Adaptive Time Scaling"};
result.offlineOnly = true;
result.executionReadinessOverride = "OFFLINE_DIAGNOSTIC_ONLY";

if result.complete
    fprintf("\n[PASS] ACCP high-precision path: %d sparse -> %d output points.\n",seam.numPoints,size(result.qPath,1));
elseif result.configurationComplete
    fprintf("\n[CONFIGURATION PASS] Sparse chain exists; high-precision ACCP PASS=%d.\n",result.complete);
else
    fprintf("\n[FAIL] No complete %d-point path under hard constraints.\n",seam.numPoints);
end
fprintf("Weld points             : %d\n",seam.numPoints);
fprintf("Layers with IK states   : %d / %d\n", ...
    sum(~cellfun(@isempty,stateBank)),seam.numPoints);
fprintf("Reachable final states  : %d\n",graph.finalReachableCount);
fprintf("Node collision free     : %d\n",result.nodeCollisionFree);
fprintf("Edge collision free     : %d\n",result.edgeCollisionFree);
fprintf("Max joint jump          : %.3f deg\n",result.maxJointJumpDeg);
fprintf("IK calls                : %d\n",stats.ikCalls);
fprintf("IK elapsed              : %.6f s\n",stats.ikElapsedSec);
fprintf("IK pose-success rate    : %.2f %%\n", ...
    100*stats.ikPoseSuccess/max(1,stats.ikCalls));
fprintf("Max node TCP error      : %.6f mm\n",result.tcpMetrics.maxNodePositionErrorMm);
fprintf("Max segment TCP error   : %.6f mm\n",result.tcpMetrics.maxSegmentPositionErrorMm);
fprintf("Max segment TCP orient  : %.6f deg\n",result.tcpMetrics.maxSegmentOrientationErrorDeg);
fprintf("Fallback used           : %d\n",fallbackUsed);
fprintf("Wire collision model    : %s\n",ternary(scene.wireCollisionEnabled,"enabled","disabled"));
fprintf("Approach path planned   : %d\n",result.approachPathPlanned);
fprintf("Exit path planned       : %d\n",result.exitPathPlanned);
fprintf("Algorithm time (scene excluded): %.6f s\n",result.algorithmElapsedSec);
fprintf("ACCP configuration time : %.6f s\n",result.configurationSearchTimeSec);
fprintf("ACCP precision time     : %.6f s\n",result.precisionAlgorithmTimeSec);

outputFile = options.outputFile;
if ~isfile(outputFile) && ~startsWith(string(outputFile),filesep)
    outputFile = fullfile(matlabDir,outputFile);
end
result.outputFile = outputFile;
bridgeJsonFile = [outputFile '_cartesian_bridge.json'];
try
    fid=fopen(bridgeJsonFile,'w');
    if fid>=0
        cleanup=onCleanup(@()fclose(fid));
        fwrite(fid,jsonencode(struct('schema_version','ACCP_CARTESIAN_BRIDGE_v1', ...
            'diagnostic_only',true,'executionReady',false, ...
            'original_seam_point_count',originalSeamPointCount, ...
            'expanded_seam_point_count',seam.numPoints, ...
            'bridge',cartesianBridge)),'char');
        clear cleanup;
    end
catch bridgeErr
    fprintf('[WARN] Could not write Cartesian bridge report: %s\n',bridgeErr.message);
end
result.cartesianBridgeDiagnosticsFile = bridgeJsonFile;
save(outputFile,"result","-v7.3");
fprintf("Saved result: %s\n",outputFile);

if options.animate && result.complete, localAnimate(scene,seam,result.qPath); end
if options.publish && result.complete
    publish_weld_trajectory(result.qPath,struct("durationPerSegment",0.15));
end
end

function seam = localInferSeam(scene,N,endMarginM)
info = scene.workpieceProxyInfo;
bottom = localFindProxy(info,"WorkpieceCollision_bottom");
wallX0 = localFindProxy(info,"WorkpieceCollision_wall_x0");
wallX1 = localFindProxy(info,"WorkpieceCollision_wall_x1");
wallY1 = localFindProxy(info,"WorkpieceCollision_wall_y1");
bottomMin = bottom.T_workpiece_proxy(1:3,4)-bottom.size_m(:)/2;
bottomMax = bottom.T_workpiece_proxy(1:3,4)+bottom.size_m(:)/2;
xMin = max(bottomMin(1),wallX0.T_workpiece_proxy(1,4)+wallX0.size_m(1)/2);
xMax = min(bottomMax(1),wallX1.T_workpiece_proxy(1,4)-wallX1.size_m(1)/2);
ySeam = wallY1.T_workpiece_proxy(2,4)-wallY1.size_m(2)/2;
zSeam = bottomMax(3);
if xMax <= xMin, error("无法从 Workpiece boxes 推导焊缝交线。"); end
p0 = [xMin+endMarginM;ySeam;zSeam];
p1 = [xMax-endMarginM;ySeam;zSeam];
if p1(1)<=p0(1), error("焊缝端部余量 %.1f mm 过大。",endMarginM*1000); end
s = linspace(0,1,N).';
seam = struct();
seam.startWP = p0; seam.endWP = p1;
seam.tangentWP = (p1-p0)/norm(p1-p0);
seam.positionsWP = (1-s).*p0.'+s.*p1.';
P_B = (scene.T_MatBase_Workpiece*[seam.positionsWP,ones(N,1)].').';
seam.positionsMat = P_B(:,1:3);
seam.startMat = seam.positionsMat(1,:);
seam.endMat = seam.positionsMat(end,:);
seam.numPoints = N; seam.plateThicknessM = bottom.size_m(3);
seam.wallY1InnerY = ySeam; seam.bottomTopZ = zSeam;
seam.freeSpaceCondition = "y < wall_y1 inner face and z > bottom top face";
end

function seam = localBuildCustomSeam(scene,positionsWP)
% Build the same seam structure as localInferSeam, but preserve an arbitrary
% non-uniform point list.  This is used by adaptive feasibility refinement:
% only a failed interval is bisected; no fixed-density re-sampling is done.
positionsWP = double(positionsWP);
if size(positionsWP,2) ~= 3 || size(positionsWP,1) < 2
    error("seamPositionsWP must be an N-by-3 matrix with N >= 2.");
end
info = scene.workpieceProxyInfo;
bottom = localFindProxy(info,"WorkpieceCollision_bottom");
p0 = positionsWP(1,:).';
p1 = positionsWP(end,:).';
tangent = p1-p0;
if norm(tangent) < 1e-12
    error("Custom seam endpoints must be distinct.");
end
seam = struct();
seam.startWP = p0;
seam.endWP = p1;
seam.tangentWP = tangent/norm(tangent);
seam.positionsWP = positionsWP;
P_B = (scene.T_MatBase_Workpiece*[positionsWP,ones(size(positionsWP,1),1)].').';
seam.positionsMat = P_B(:,1:3);
seam.startMat = seam.positionsMat(1,:);
seam.endMat = seam.positionsMat(end,:);
seam.numPoints = size(positionsWP,1);
seam.plateThicknessM = bottom.size_m(3);
seam.wallY1InnerY = NaN;
seam.bottomTopZ = NaN;
seam.freeSpaceCondition = "y < wall_y1 inner face and z > bottom top face";
end

function scene = localLoadScene(syncFile)
% sync_ur5_joint_states is a script beginning with clear/clc/close all.
% Execute it in this helper workspace so it cannot erase replan options.
run(syncFile);
end

function s = localFindProxy(info,name)
idx = find(strcmp(string({info.name}),string(name)),1);
if isempty(idx), error("找不到工件代理 %s。",name); end
s = info(idx);
end

function [states,stats,diagnostics] = localGenerateFirstBeam(scene,seam,options)
% Generate only the first layer from a cold start.  No old qPath is read.
stats = localEmptyContinuationStats();
diagnostics = localEmptyFirstPointDiagnostics();
diagnostics.joint_continuity_threshold_deg = options.maxJointStepDeg;
diagnostics.axis_constraint_disabled = logical(options.disableAxisConstraint);
diagnosticOptions = options;
diagnosticOptions.diagnosticCollectIKCandidates = true;
diagnosticOptions.diagnosticReferenceQ = scene.branchRefQ;
diagnosticOptions.diagnosticMaxJointDeltaDeg = options.maxJointStepDeg;
% The branch reference leads the seed list so the first accepted solution is
% already in its branch family; the random seeds are still appended so other
% branches remain reachable.
qSeeds = localBranchSeeds(scene,options.numRandomSeedsFirstPoint);
for k = 1:options.numRandomSeedsFirstPoint
    qSeeds(end+1,:) = randomConfiguration(scene.robot); %#ok<AGROW>
end
states = repmat(localEmptyState(),0,1);
for it = 1:numel(options.coarseTravelDeg)
    for ir = 1:numel(options.coarseRollDeg)
        [Ttarget,poseInfo] = localTargetForPose(scene,seam,1, ...
            options.coarseTravelDeg(it),options.coarseRollDeg(ir)); %#ok<ASGLU>
        [r,ikElapsed] = localTimedIKCall(scene,Ttarget, ...
            options.coarseTravelDeg(it),options.coarseRollDeg(ir),qSeeds,diagnosticOptions);
        stats = localAddGeneratorStats(stats,r,ikElapsed);
        diagnostics = localAddFirstPointDiagnostics(diagnostics,r);
        for k = 1:numel(r.states)
            r.states(k).pointIndex = 1;
            r.states(k).poseInfo = poseInfo;
            r.states(k).velocityQ = zeros(1,6);
            r.states(k).parentQ = scene.branchRefQ;
            r.states(k).continuationCost = sum(localWrapRad(r.states(k).q-scene.branchRefQ).^2);
            r.states(k).bridgeNode = false;
            r.states(k).bridgeFraction = NaN;
            r.states(k).bridgeLevel = 0;
            r.states(k).bridgeSource = "";
        end
        states = localAppendUniqueStates(states,r.states,0.5);
    end
end
end

function diagnostics = localEmptyFirstPointDiagnostics()
diagnostics = struct( ...
    "schema_version","ACCP_POINT1_FILTER_DIAGNOSTICS_v1", ...
    "point_index",1, ...
    "ik_calls",0, ...
    "ik_numeric_success",0, ...
    "ik_success",0, ...
    "orientation_direction_rejected",0, ...
    "position_error_rejected",0, ...
    "orientation_error_rejected",0, ...
    "joint_limit_rejected",0, ...
    "joint_continuity_rejected",0, ...
    "joint_continuity_threshold_deg",NaN, ...
    "joint_continuity_is_diagnostic_only",true, ...
    "joint_continuity_reference","branchReferenceQ", ...
    "axis_constraint_disabled",false, ...
    "axis_constraint_definition","dot(WeldingGunMount axis, target_weld_axis_world) >= cos(sideToleranceDeg)", ...
    "world_frame","MATLAB planning base frame", ...
    "self_collision_rejected",0, ...
    "gun_collision_rejected",0, ...
    "workpiece_collision_rejected",0, ...
    "welding_side_position_rejected",0, ...
    "accepted_after_all_filters",0, ...
    "candidates",repmat(localEmptyDiagnosticCandidate(),0,1));
end

function diagnostics = localAddFirstPointDiagnostics(diagnostics,r)
diagnostics.ik_calls = diagnostics.ik_calls + r.numIKCalls;
diagnostics.ik_numeric_success = diagnostics.ik_numeric_success + ...
    localResultField(r,"numIKNumericSuccess",0);
diagnostics.ik_success = diagnostics.ik_success + r.numIKPoseOK;
diagnostics.orientation_direction_rejected = diagnostics.orientation_direction_rejected + ...
    localResultField(r,"numDirectionRejected",0);
diagnostics.position_error_rejected = diagnostics.position_error_rejected + ...
    localResultField(r,"numPositionRejected",0);
diagnostics.orientation_error_rejected = diagnostics.orientation_error_rejected + ...
    localResultField(r,"numOrientationRejected",0);
diagnostics.joint_limit_rejected = diagnostics.joint_limit_rejected + r.numJointRejected;
diagnostics.joint_continuity_rejected = diagnostics.joint_continuity_rejected + ...
    localResultField(r,"numJointContinuityRejected",0);
diagnostics.self_collision_rejected = diagnostics.self_collision_rejected + r.numRobotSelfRejected;
diagnostics.gun_collision_rejected = diagnostics.gun_collision_rejected + r.numGunRejected;
diagnostics.workpiece_collision_rejected = diagnostics.workpiece_collision_rejected + ...
    localResultField(r,"numWorkpieceRejected",0);
diagnostics.welding_side_position_rejected = diagnostics.welding_side_position_rejected + ...
    localResultField(r,"numWeldingSidePositionRejected",0);
diagnostics.accepted_after_all_filters = diagnostics.accepted_after_all_filters + r.numAccepted;
if isfield(r,"diagnosticCandidates") && ~isempty(r.diagnosticCandidates)
    diagnostics.candidates = [diagnostics.candidates; r.diagnosticCandidates(:)]; %#ok<AGROW>
end
end

function value = localResultField(s,name,default)
if isfield(s,name), value = s.(name); else, value = default; end
end

function s = localEmptyDiagnosticCandidate()
s = struct("q_rad",zeros(1,6),"q_deg",zeros(1,6),"tcp_pose",eye(4), ...
    "tcp_rotation_matrix",eye(3), ...
    "welding_gun_mount_axis_world",zeros(1,3), ...
    "target_weld_axis_world",zeros(1,3), ...
    "seam_tangent_world",zeros(1,3), ...
    "axis_angle_deg",NaN,"axis_vs_seam_angle_deg",NaN, ...
    "position_error_mm",NaN,"orientation_error_deg",NaN, ...
    "joint_delta_from_seed_deg",zeros(1,6), ...
    "joint_delta_from_reference_deg",zeros(1,6), ...
    "max_joint_delta_deg",NaN,"travel_deg",NaN,"roll_deg",NaN, ...
    "seed_index",NaN,"rejection_reason","accepted");
end

function ref = localBranchReference(scene,options)
% Resolve the joint configuration used to steer branch selection.
ref = double(scene.qMat0(:).');
if isfield(options,"branchReferenceQ") && ~isempty(options.branchReferenceQ)
    b = double(options.branchReferenceQ(:).');
    if numel(b) ~= 6 || any(~isfinite(b))
        error("branchReferenceQ must be a finite 1x6 joint vector.");
    end
    ref = b;
    fprintf("Branch reference overridden: [%.3f %.3f %.3f %.3f %.3f %.3f] deg\n",rad2deg(ref));
    fprintf("  distance from qMat0 = %.3f deg\n", ...
        max(abs(rad2deg(localWrapRad(ref-double(scene.qMat0(:).'))))));
end
end

function seeds = localBranchSeeds(scene,~)
% Leading seed row is the branch reference.  Kept separate from the random
% seeds so the caller controls how many random seeds follow it.
seeds = double(scene.branchRefQ(:).');
end

function [states,stats] = localGenerateContinuationLayer(scene,seam,i,prev,options)
% Expand a bounded local pose neighborhood around the previous beam.
stats = localEmptyContinuationStats();
states = repmat(localEmptyState(),0,1);
if isempty(prev), return; end

for ip = 1:numel(prev)
    qPrev = prev(ip).q;
    if isfield(prev,"velocityQ")
        qPred = qPrev + prev(ip).velocityQ;
    else
        qPred = qPrev;
    end
    dRoll = [-options.maxPoseStepRollDeg 0 options.maxPoseStepRollDeg];
    dTravel = [-options.maxPoseStepTravelDeg 0 options.maxPoseStepTravelDeg];
    poseList = zeros(0,2);
    for it = 1:numel(dTravel)
        for ir = 1:numel(dRoll)
            poseList(end+1,:) = [prev(ip).travelDeg+dTravel(it), ...
                mod(prev(ip).rollDeg+dRoll(ir),360)]; %#ok<AGROW>
        end
    end
    poseCenter = [0 0];
    if isfield(options,"poseNeighborhoodCost") && ...
            strcmpi(string(options.poseNeighborhoodCost),"previous")
        poseCenter = [prev(ip).travelDeg prev(ip).rollDeg];
    end
    poseList = localUniquePoseList(poseList,options.maxPoseNeighbors,poseCenter);

    seeds = [qPred;qPrev];
    perturb = deg2rad(options.seedPerturbDeg);
    if options.maxPoseSeeds >= 3, q=qPred; q(4)=q(4)+perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds >= 4, q=qPred; q(4)=q(4)-perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds >= 5, q=qPred; q(5)=q(5)+perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds >= 6, q=qPred; q(5)=q(5)-perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds >= 7, q=qPred; q(6)=q(6)+perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds >= 8, q=qPred; q(6)=q(6)-perturb; seeds(end+1,:)=q; end
    seeds = seeds(1:min(size(seeds,1),options.maxPoseSeeds),:);

    for ipose = 1:size(poseList,1)
        [Ttarget,poseInfo] = localTargetForPose(scene,seam,i, ...
            poseList(ipose,1),poseList(ipose,2)); %#ok<ASGLU>
        [r,ikElapsed] = localTimedIKCall(scene,Ttarget, ...
            poseList(ipose,1),poseList(ipose,2),seeds,options);
        stats = localAddGeneratorStats(stats,r,ikElapsed);
        if isfield(r,'diagnosticCandidates') && ~isempty(r.diagnosticCandidates)
            stats.diagnosticCandidatePools{end+1}=struct('Ttarget',Ttarget, ...
                'travelDeg',poseList(ipose,1),'rollDeg',poseList(ipose,2), ...
                'poseInfo',poseInfo,'rows',r.diagnosticCandidates); %#ok<AGROW>
        end
        for ks = 1:numel(r.states)
            st = r.states(ks);
            st.pointIndex = i;
            st.poseInfo = poseInfo;
            [parentIdx,parentDq] = localBestParent(scene,prev,st,options);
            if parentIdx == 0, continue; end
            st.parentQ = prev(parentIdx).q;
            st.velocityQ = localWrapRad(st.q-st.parentQ);
            parentCost = 0;
            if isfield(prev,"continuationCost"), parentCost=prev(parentIdx).continuationCost; end
            st.continuationCost = parentCost + sum(parentDq.^2) + ...
                1e-4*(abs(st.rollDeg-prev(parentIdx).rollDeg)^2 + ...
                (st.travelDeg-prev(parentIdx).travelDeg)^2);
            st.bridgeNode = false;
            st.bridgeFraction = NaN;
            st.bridgeLevel = 0;
            st.bridgeSource = "";
            states(end+1,1) = st; %#ok<AGROW>
            stats.edgeTests = stats.edgeTests + 1;
            stats.edgeCollisionChecks = stats.edgeCollisionChecks + 1;
        end
    end
end

states = localAppendUniqueStates(repmat(localEmptyState(),0,1),states,0.5);
if isempty(states) && options.angularRedundancyFallback && ...
        ~options.disableLocalRecovery
    % Conditional angular-redundancy recovery.  The measured wire axis is
    % already used by localTargetForPose/make_actual_gun_axis_pose; only the
    % redundant travel/roll neighborhood is widened here.
    for kStep = 1:numel(options.angularFallbackRollDeg)
        retry = options;
        retry.maxPoseStepRollDeg = options.angularFallbackRollDeg(kStep);
        retry.maxPoseStepTravelDeg = options.angularFallbackTravelDeg(min( ...
            kStep,numel(options.angularFallbackTravelDeg)));
        retry.maxPoseNeighbors = 9;
        retry.disableLocalRecovery = true;
        [candidate,candidateStats] = localGenerateContinuationLayer( ...
            scene,seam,i,prev,retry);
        stats = localAddContinuationStats(stats,candidateStats);
        if ~isempty(candidate)
            states = candidate;
            break;
        end
    end
end
if isempty(states) && ~options.disableLocalRecovery
    % Local bounded recovery: more seeds and the complete 3x3 pose stencil,
    % still subject to the same hard pose/joint/collision constraints.
    retry = options;
    retry.maxPoseSeeds = max(options.maxPoseSeeds,12);
    retry.maxPoseNeighbors = 25;
    retry.maxPoseStepRollDeg = max(options.maxPoseStepRollDeg,45);
    retry.maxPoseStepTravelDeg = max(options.maxPoseStepTravelDeg,15);
    retry.seedPerturbDeg = max(options.seedPerturbDeg,15);
    [states,retryStats] = localGenerateContinuationLayerNoRetry( ...
        scene,seam,i,prev,retry);
    stats = localAddContinuationStats(stats,retryStats);
end
end

function [states,stats] = localGenerateContinuationLayerNoRetry(scene,seam,i,prev,options)
% Same bounded expansion without recursive recovery.
stats = localEmptyContinuationStats();
states = repmat(localEmptyState(),0,1);
for ip = 1:numel(prev)
    qPrev = prev(ip).q;
    if isfield(prev,"velocityQ"), qPred=qPrev+prev(ip).velocityQ; else, qPred=qPrev; end
    dRoll = [-options.maxPoseStepRollDeg 0 options.maxPoseStepRollDeg];
    dTravel = [-options.maxPoseStepTravelDeg 0 options.maxPoseStepTravelDeg];
    poseList = zeros(0,2);
    for it=1:numel(dTravel), for ir=1:numel(dRoll)
        poseList(end+1,:)=[prev(ip).travelDeg+dTravel(it),mod(prev(ip).rollDeg+dRoll(ir),360)]; %#ok<AGROW>
    end, end
    poseCenter = [0 0];
    if isfield(options,"poseNeighborhoodCost") && ...
            strcmpi(string(options.poseNeighborhoodCost),"previous")
        poseCenter = [prev(ip).travelDeg prev(ip).rollDeg];
    end
    poseList=localUniquePoseList(poseList,options.maxPoseNeighbors,poseCenter);
    seeds=[qPred;qPrev]; perturb=deg2rad(options.seedPerturbDeg);
    if options.maxPoseSeeds>=3, q=qPred; q(4)=q(4)+perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds>=4, q=qPred; q(4)=q(4)-perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds>=5, q=qPred; q(5)=q(5)+perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds>=6, q=qPred; q(5)=q(5)-perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds>=7, q=qPred; q(6)=q(6)+perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds>=8, q=qPred; q(6)=q(6)-perturb; seeds(end+1,:)=q; end
    if options.maxPoseSeeds>=10, q=qPred; q(5)=q(5)+2*perturb; seeds(end+1,:)=q; q=qPred; q(5)=q(5)-2*perturb; seeds(end+1,:)=q; end
    seeds=seeds(1:min(size(seeds,1),options.maxPoseSeeds),:);
    for ipose=1:size(poseList,1)
        [Ttarget,poseInfo]=localTargetForPose(scene,seam,i,poseList(ipose,1),poseList(ipose,2)); %#ok<ASGLU>
        [r,ikElapsed]=localTimedIKCall(scene,Ttarget,poseList(ipose,1),poseList(ipose,2),seeds,options);
        stats=localAddGeneratorStats(stats,r,ikElapsed);
        if isfield(r,'diagnosticCandidates') && ~isempty(r.diagnosticCandidates)
            stats.diagnosticCandidatePools{end+1}=struct('Ttarget',Ttarget, ...
                'travelDeg',poseList(ipose,1),'rollDeg',poseList(ipose,2), ...
                'poseInfo',poseInfo,'rows',r.diagnosticCandidates); %#ok<AGROW>
        end
        for ks=1:numel(r.states)
            st=r.states(ks); st.pointIndex=i; st.poseInfo=poseInfo;
            [parentIdx,parentDq]=localBestParent(scene,prev,st,options); if parentIdx==0, continue; end
            st.parentQ=prev(parentIdx).q; st.velocityQ=localWrapRad(st.q-st.parentQ);
            parentCost=0; if isfield(prev,"continuationCost"), parentCost=prev(parentIdx).continuationCost; end
            st.continuationCost=parentCost+sum(parentDq.^2);
            st.bridgeNode = false;
            st.bridgeFraction = NaN;
            st.bridgeLevel = 0;
            st.bridgeSource = "";
            states(end+1,1)=st; %#ok<AGROW>
            stats.edgeTests=stats.edgeTests+1; stats.edgeCollisionChecks=stats.edgeCollisionChecks+1;
        end
    end
end
states=localAppendUniqueStates(repmat(localEmptyState(),0,1),states,0.5);
end

function [idx,dq] = localBestParent(scene,prev,st,options)
idx=0; dq=zeros(1,6); best=inf;
for k=1:numel(prev)
    if ~localPoseStepOK(prev(k),st,options), continue; end
    d=localWrapRad(st.q-prev(k).q);
    if max(abs(rad2deg(d)))>options.maxJointStepDeg, continue; end
    if sum(d.^2)>=best, continue; end
    if ~localEdgeFree(scene,prev(k).q,st.q,options), continue; end
    best=sum(d.^2); idx=k; dq=d;
end
end

function [Ttarget,poseInfo] = localTargetForPose(scene,seam,i,travelDeg,rollDeg)
[RtcpWP,poseInfo] = make_actual_gun_axis_pose(scene,seam.tangentWP, ...
    scene.weldGunAxisWP,travelDeg,rollDeg);
T_W_TCP=eye(4); T_W_TCP(1:3,1:3)=RtcpWP; T_W_TCP(1:3,4)=seam.positionsWP(i,:).';
Ttarget=scene.T_MatBase_Workpiece*T_W_TCP;
end

function optionsOut = localCandidateOptions(options)
optionsOut=struct("positionToleranceM",options.positionToleranceM, ...
    "orientationToleranceDeg",options.orientationToleranceDeg, ...
    "sideToleranceDeg",options.sideToleranceDeg,"sideMarginM",options.sideMarginM, ...
    "checkSelfCollision",options.checkSelfCollision,"lazyCollision",options.lazyCollision, ...
    "reuseCandidateCache",options.reuseCandidateCache, ...
    "disableAxisConstraint",options.disableAxisConstraint);
if isfield(options,"diagnosticCollectIKCandidates")
    optionsOut.diagnosticCollectIKCandidates = options.diagnosticCollectIKCandidates;
end
if isfield(options,"diagnosticReferenceQ")
    optionsOut.diagnosticReferenceQ = options.diagnosticReferenceQ;
end
if isfield(options,"diagnosticMaxJointDeltaDeg")
    optionsOut.diagnosticMaxJointDeltaDeg = options.diagnosticMaxJointDeltaDeg;
end
if isfield(options,"diagnosticTargetTangentWP")
    optionsOut.diagnosticTargetTangentWP = options.diagnosticTargetTangentWP;
end
end

function poses = localUniquePoseList(poses,maxCount,center)
if isempty(poses), return; end
if nargin < 3 || isempty(center), center=[0 0]; end
poses(:,2)=mod(poses(:,2),360);
keep=true(size(poses,1),1);
for i=1:size(poses,1)
    if ~keep(i), continue; end
    for j=i+1:size(poses,1)
        if keep(j) && abs(poses(i,1)-poses(j,1))<1e-9 && ...
                localAngleDistanceDeg(poses(i,2),poses(j,2))<1e-9
            keep(j)=false;
        end
    end
end
poses=poses(keep,:);
% Prefer the smallest relative travel/roll change.  This implements the
% angular-redundancy continuity cost used by the paper-guided experiment.
score=abs(poses(:,1)-center(1))+arrayfun( ...
    @(x)localAngleDistanceDeg(x,center(2)),poses(:,2));
[~,order]=sort(score,"ascend");
poses=poses(order,:);
if size(poses,1)>maxCount, poses=poses(1:maxCount,:); end
end

function states = localKeepBeam(states,maxCount,refQ)
if isempty(states), return; end
if isfield(states,"continuationCost")
    score=vertcat(states.continuationCost);
else
    score=zeros(numel(states),1);
    for i=1:numel(states), score(i)=sum(localWrapRad(states(i).q-refQ).^2); end
end

[~,idx]=sort(score,"ascend");
states=states(idx(1:min(maxCount,numel(idx))));
end

function [selected,audit] = localSelectCandidates(scene,seam,i,states,prev,maxCount,options)
audit=[];
if options.candidate_scoring_mode || options.candidate_diagnostics
    [states,audit] = accp_score_candidates(scene,seam,i,states,prev,maxCount,options);
    if options.candidate_scoring_mode
        selected=states(1:min(maxCount,numel(states)));
        return;
    end
end
selected=localKeepBeam(states,maxCount,scene.branchRefQ);
end

function stats = localEmptyContinuationStats()
stats=struct("ikCalls",0,"ikPoseSuccess",0,"acceptedStates",0, ...
    "candidatePoseCalls",0,"edgeTests",0,"edgeCollisionChecks",0, ...
    "ikElapsedSec",0,"diagnosticCandidatePools",{{}});
end

function [r,elapsed] = localTimedIKCall(scene,Ttarget,travelDeg,rollDeg,seeds,options)
t=tic;
r=generate_actual_axis_ik_candidates(scene,Ttarget,travelDeg,rollDeg,seeds, ...
    localCandidateOptions(options));
elapsed=toc(t);
if isfield(scene,'candidateAuditStore') && ~isempty(r.states)
    [~,point]=min(vecnorm(scene.auditSeamPositions-Ttarget(1:3,4).',2,2));
    bank=r.states(:);
    if isKey(scene.candidateAuditStore,point)
        bank=[scene.candidateAuditStore(point);bank];
    end
    scene.candidateAuditStore(point)=bank;
end
end

function stats = localAddGeneratorStats(stats,r,elapsed)
stats.ikCalls=stats.ikCalls+r.numIKCalls;
stats.ikPoseSuccess=stats.ikPoseSuccess+r.numIKPoseOK;
stats.acceptedStates=stats.acceptedStates+r.numAccepted;
stats.candidatePoseCalls=stats.candidatePoseCalls+1;
if nargin>=3, stats.ikElapsedSec=stats.ikElapsedSec+elapsed; end
end

function out = localAddContinuationStats(a,b)
out=a;
names=fieldnames(a);
for i=1:numel(names)
    name=names{i};
    if strcmp(name,'diagnosticCandidatePools')
        out.(name)=[out.(name),b.(name)];
    else
        out.(name)=a.(name)+b.(name);
    end
end
end

function graph = localEmptyGraph(N)
graph=struct("reachable",{cell(1,N)},"cost",{cell(1,N)}, ...
    "parent",{cell(1,N)},"edgeTests",0,"collisionFreeEdges",0, ...
    "finalReachableCount",0,"startConnectedCount",0);
end

function info = localCheckAnalyticalIK(robot)
info = struct("available",false,"isValidGroupForIK",false, ...
    "kinematicGroupType","","configIdx",[]);
try
    if exist("analyticalInverseKinematics","class") ~= 8
        return;
    end
    a = analyticalInverseKinematics(robot);
    info.available = true;
    info.isValidGroupForIK = logical(a.IsValidGroupForIK);
    info.kinematicGroupType = char(a.KinematicGroupType);
    info.configIdx = double(a.KinematicGroupConfigIdx);
catch err
    info.error = string(err.message);
end
end

function metrics = localComputeTcpPathErrors(scene,seam,stateBank,plan,options)
metrics = struct("maxNodePositionErrorMm",NaN, ...
    "maxNodeOrientationErrorDeg",NaN,"maxSegmentPositionErrorMm",NaN, ...
    "maxSegmentOrientationErrorDeg",NaN,"checkedSegments",0, ...
    "allSegmentsChecked",false);
if ~plan.complete || isempty(plan.qPath) || ~isfield(plan,"stateIndices")
    return;
end
N=size(plan.qPath,1);
nodePos=zeros(N,1); nodeOri=zeros(N,1);
for i=1:N
    st=stateBank{i}(plan.stateIndices(i));
    T=getTransform(scene.robot,plan.qPath(i,:),scene.tcpBodyName);
    nodePos(i)=norm(T(1:3,4)-st.Ttarget(1:3,4));
    c=(trace(st.Ttarget(1:3,1:3).'*T(1:3,1:3))-1)/2;
    nodeOri(i)=acos(max(-1,min(1,c)));
end
metrics.maxNodePositionErrorMm=max(nodePos)*1000;
metrics.maxNodeOrientationErrorDeg=max(rad2deg(nodeOri));
segPos=zeros(N-1,1); segOri=zeros(N-1,1);
samplesPerSegment=max(2,ceil(max(abs(rad2deg(diff(plan.qPath,1,1))),[],2)/options.edgeResolutionDeg)+1);
for i=1:N-1
    a=stateBank{i}(plan.stateIndices(i)); b=stateBank{i+1}(plan.stateIndices(i+1));
    for k=0:samplesPerSegment(i)-1
        s=k/(samplesPerSegment(i)-1);
        q=plan.qPath(i,:)+s*localWrapRad(plan.qPath(i+1,:)-plan.qPath(i,:));
        T=getTransform(scene.robot,q,scene.tcpBodyName);
        pRef=(1-s)*a.Ttarget(1:3,4)+s*b.Ttarget(1:3,4);
        RRef=localQuatSlerp(a.Ttarget(1:3,1:3),b.Ttarget(1:3,1:3),s);
        segPos(i)=max(segPos(i),norm(T(1:3,4)-pRef));
        c=(trace(RRef.'*T(1:3,1:3))-1)/2;
        segOri(i)=max(segOri(i),acos(max(-1,min(1,c))));
    end
end
metrics.maxSegmentPositionErrorMm=max(segPos)*1000;
metrics.maxSegmentOrientationErrorDeg=max(rad2deg(segOri));
metrics.checkedSegments=N-1;
% The seam can contain validated Cartesian bridge layers, so the metric is
% defined by the actual selected chain rather than a hard-coded 80-point
% assumption.
metrics.allSegmentsChecked=(N>=2);
end

function R = localQuatSlerp(R1,R2,s)
q1=rotm2quat(R1); q2=rotm2quat(R2);
if dot(q1,q2)<0, q2=-q2; end
q=(1-s)*q1+s*q2; q=q/norm(q);
R=quat2rotm(q);
end

function states = localGenerateLayer(scene,seam,i,travelCandidates,rollCandidates,options,existing)
seedBank = localSeedBank(scene,existing,options.numRandomSeeds,options.maxContinuationSeeds);
states = repmat(localEmptyState(),0,1);
lastPoseStates = repmat(localEmptyState(),0,1);
pWP = seam.positionsWP(i,:).';
for it = 1:numel(travelCandidates)
    for ir = 1:numel(rollCandidates)
        poseSeeds = seedBank;
        if options.reusePoseCandidates && ~isempty(lastPoseStates)
            nSeed = min(numel(lastPoseStates),options.maxContinuationSeeds);
            idxSeed = unique(round(linspace(1,numel(lastPoseStates),nSeed)));
            poseSeeds = [seedBank(1,:); vertcat(lastPoseStates(idxSeed).q)];
        end
        [RtcpWP,poseInfo] = make_actual_gun_axis_pose(scene,seam.tangentWP, ...
            scene.weldGunAxisWP,travelCandidates(it),rollCandidates(ir));
        T_W_TCP = eye(4); T_W_TCP(1:3,1:3)=RtcpWP; T_W_TCP(1:3,4)=pWP;
        Ttarget = scene.T_MatBase_Workpiece*T_W_TCP;
        r = generate_actual_axis_ik_candidates(scene,Ttarget, ...
            travelCandidates(it),rollCandidates(ir),poseSeeds,struct( ...
            "positionToleranceM",options.positionToleranceM, ...
            "orientationToleranceDeg",options.orientationToleranceDeg, ...
            "sideToleranceDeg",options.sideToleranceDeg, ...
            "sideMarginM",options.sideMarginM, ...
            "checkSelfCollision",options.checkSelfCollision, ...
            "lazyCollision",options.lazyCollision, ...
            "reuseCandidateCache",options.reuseCandidateCache));
        % Retry a failed continuation with the full seed bank, while
        % retaining the last successful pose states for the next pose.
        if options.reusePoseCandidates && isempty(r.states) && ...
                ~isempty(lastPoseStates) && size(poseSeeds,1) < size(seedBank,1)
            r = generate_actual_axis_ik_candidates(scene,Ttarget, ...
                travelCandidates(it),rollCandidates(ir),seedBank,struct( ...
                "positionToleranceM",options.positionToleranceM, ...
                "orientationToleranceDeg",options.orientationToleranceDeg, ...
                "sideToleranceDeg",options.sideToleranceDeg, ...
                "sideMarginM",options.sideMarginM, ...
                "checkSelfCollision",options.checkSelfCollision, ...
                "lazyCollision",options.lazyCollision, ...
                "reuseCandidateCache",options.reuseCandidateCache));
        end
        if options.reusePoseCandidates && ~isempty(r.states)
            lastPoseStates = r.states;
        end
        for k = 1:numel(r.states)
            r.states(k).pointIndex = i; r.states(k).poseInfo = poseInfo;
        end
        if ~isempty(r.states), states = [states;r.states(:)]; %#ok<AGROW>
        end
    end
end
states = localAppendUniqueStates(repmat(localEmptyState(),0,1),states,0.5);
end

function seeds = localSeedBank(scene,prior,numRandom,maxPrior)
seeds = localBranchSeeds(scene,numRandom);
for k = 1:numRandom, seeds(end+1,:)=randomConfiguration(scene.robot); %#ok<AGROW>
end
if ~isempty(prior)
    n = min(numel(prior),maxPrior);
    idx = unique(round(linspace(1,numel(prior),n)));
    for k = idx, seeds(end+1,:) = prior(k).q; %#ok<AGROW>
    end
end
end

function states = localAppendUniqueStates(a,b,tolDeg)
states = a;
for i = 1:numel(b)
    dup = false;
    for j = 1:numel(states)
        if max(abs(rad2deg(localWrapRad(b(i).q-states(j).q)))) < tolDeg
            dup = true; break;
        end
    end
    if ~dup, states(end+1,1)=b(i); %#ok<AGROW>
    end
end
end

function [graph,plan] = localBuildGraph(scene,stateBank,seam,options)
N = numel(stateBank);
graph = struct("reachable",{cell(1,N)},"cost",{cell(1,N)}, ...
    "parent",{cell(1,N)},"edgeTests",0,"collisionFreeEdges",0, ...
    "finalReachableCount",0,"startConnectedCount",0);
nodeFree = cell(1,N);
q0Free = localFullStateFree(scene,scene.qMat0);
if ~q0Free
    fprintf("[WARN] qMat0 is not free under the complete self/environment check; start connectivity is unavailable.\n");
end
if isempty(stateBank{1}), plan = localEmptyPlan(); return; end
% The seam path itself starts at the first validated weld state.  Test the
% real qMat0-to-first-state approach separately, but do not discard an
% otherwise valid weld path merely because free-space approach is not yet
% planned (it is explicitly reported as incomplete below).
graph.reachable{1}=true(1,numel(stateBank{1}));
graph.cost{1}=zeros(1,numel(stateBank{1}));
graph.parent{1}=zeros(1,numel(stateBank{1}));
nodeFree{1}=true(1,numel(stateBank{1}));
for j=1:numel(stateBank{1})
    graph.edgeTests=graph.edgeTests+1;
    if q0Free && nodeFree{1}(j) && localEdgeFree(scene,scene.qMat0,stateBank{1}(j).q,options)
        graph.startConnectedCount=graph.startConnectedCount+1;
        graph.collisionFreeEdges=graph.collisionFreeEdges+1;
    end
end
for i=2:N
    nNow=numel(stateBank{i}); nPrev=numel(stateBank{i-1});
    graph.reachable{i}=false(1,nNow);
    graph.cost{i}=inf(1,nNow); graph.parent{i}=zeros(1,nNow);
    nodeFree{i}=true(1,nNow);
    for j=1:nNow
        for k=1:nPrev
            if ~graph.reachable{i-1}(k), continue; end
            if ~localPoseStepOK(stateBank{i-1}(k),stateBank{i}(j),options), continue; end
            dq=localWrapRad(stateBank{i}(j).q-stateBank{i-1}(k).q);
            if max(abs(rad2deg(dq)))>options.maxJointStepDeg, continue; end
            if ~nodeFree{i}(j), continue; end
            graph.edgeTests=graph.edgeTests+1;
            edgeOK=localEdgeFreeForStates(scene,stateBank{i-1}(k), ...
                stateBank{i}(j),options);
            if ~edgeOK, continue; end
            graph.collisionFreeEdges=graph.collisionFreeEdges+1;
            edgeCost=sum(dq.^2)+1e-4*( ...
                localAngleDistanceDeg(stateBank{i-1}(k).rollDeg,stateBank{i}(j).rollDeg)^2+ ...
                (stateBank{i}(j).travelDeg-stateBank{i-1}(k).travelDeg)^2);
            candidateCost=graph.cost{i-1}(k)+edgeCost;
            if candidateCost<graph.cost{i}(j)
                graph.cost{i}(j)=candidateCost; graph.parent{i}(j)=k;
                graph.reachable{i}(j)=true;
            end
        end
    end
end
graph.finalReachableCount=sum(graph.reachable{N});
plan=localEmptyPlan();
plan.nodeCollisionFree=graph.finalReachableCount>0 && all(~cellfun(@isempty,stateBank));
plan.edgeCollisionFree=graph.collisionFreeEdges>0;
plan.complete=graph.finalReachableCount>0 && N==seam.numPoints;
if ~plan.complete, return; end
[~,idx]=min(graph.cost{N});
stateIndices=zeros(1,N); stateIndices(N)=idx;
for i=N:-1:2, stateIndices(i-1)=graph.parent{i}(stateIndices(i)); end
qPath=zeros(N,6);
for i=1:N, qPath(i,:)=stateBank{i}(stateIndices(i)).q; end
plan.qPath=qPath; plan.stateIndices=stateIndices;
dq=localWrapRad(diff(qPath,1,1));
plan.maxJointJumpDeg=max(abs(rad2deg(dq)),[],'all');
plan.nodeCollisionFree=all(arrayfun(@(i)localFullStateFree(scene,qPath(i,:)),1:N));
plan.edgeCollisionFree=true;
for i=1:N-1
    plan.edgeCollisionFree=plan.edgeCollisionFree && ...
        localEdgeFreeForStates(scene,stateBank{i}(stateIndices(i)), ...
        stateBank{i+1}(stateIndices(i+1)),options);
    if ~plan.edgeCollisionFree, break; end
end
end

function ok=localPoseStepOK(a,b,options)
ok=localAngleDistanceDeg(a.rollDeg,b.rollDeg)<=options.maxRollStepDeg+1e-9 && ...
    abs(a.travelDeg-b.travelDeg)<=options.maxTravelStepDeg+1e-9;
end

function ok=localEdgeFree(scene,q1,q2,options)
dq=localWrapRad(q2-q1);
n=max(1,ceil(max(abs(rad2deg(dq)))/options.edgeResolutionDeg));
ok=true;
for k=0:n
    q=q1+(k/n)*dq;
    if ~localFullStateFree(scene,q), ok=false; return; end
end
end

function ok=localEdgeFreeForStates(scene,a,b,options)
% Bridge segments were independently audited with 101 samples.  Re-run
% those edges densely when the graph is built; ordinary ACCP edges retain
% the configured edgeResolutionDeg policy.
isBridge = (isfield(a,'bridgeNode') && a.bridgeNode) || ...
    (isfield(b,'bridgeNode') && b.bridgeNode);
if isBridge
    ok=localDenseEdgeFree(scene,a.q,b.q,options,options.cartesianBridgeMaxEdgeSamples);
elseif options.lazyCollision
    ok=localEdgeFreeInterior(scene,a.q,b.q,options);
else
    ok=localEdgeFree(scene,a.q,b.q,options);
end
end

function ok=localDenseEdgeFree(scene,q1,q2,options,nSamples)
nSamples=max(2,round(nSamples));
dq=localWrapRad(q2-q1); ok=true;
for k=0:nSamples-1
    q=q1+(k/(nSamples-1))*dq;
    if ~localFullStateFree(scene,q), ok=false; return; end
end
end

function ok=localEdgeFreeInterior(scene,q1,q2,options)
%LAZY edge test: endpoints were already checked by the node cache.
dq=localWrapRad(q2-q1);
n=max(1,ceil(max(abs(rad2deg(dq)))/options.edgeResolutionDeg));
ok=true;
for k=1:n-1
    q=q1+(k/n)*dq;
    if ~localFullStateFree(scene,q), ok=false; return; end
end
end

function ok=localFullStateFree(scene,q)
try
    [env,~,~]=checkCollision(scene.robot,q,scene.environment, ...
        "IgnoreSelfCollision","on","Exhaustive","on");
catch
    env=checkCollision(scene.robot,q,scene.environment,"IgnoreSelfCollision","on");
end
if any(env(:)), ok=false; return; end
if isfield(scene,"selfCollisionSkipPairs"), skips=scene.selfCollisionSkipPairs;
else, skips="parent"; end
try
    self=checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",skips,"Exhaustive","on");
catch
    self=checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",skips);
end
ok=~any(self(:));
end

function pairs=localStructuralSelfCollisionSkips(robot)
names=robot.BodyNames; pairs=cell(0,2);
% Mechanical attachment interfaces; not free-space collision exclusions.
for i=1:min(10,numel(names)-1), pairs(end+1,:)={names{i},names{i+1}}; %#ok<AGROW>
end
pairs(end+1,:)={"WeldingGunMount","wrist_3_link"};
pairs(end+1,:)={"WeldingGunMount","ee_link"};
pairs(end+1,:)={"WeldingGunMount","WeldTCP"};
end

function limits=localRobotJointLimits(robot)
limits=zeros(6,2);
j=0;
for i=1:numel(robot.Bodies)
    jt=robot.Bodies{i}.Joint;
    if strcmpi(jt.Type,"revolute") || strcmpi(jt.Type,"continuous")
        j=j+1;
        limits(j,:)=double(jt.PositionLimits);
    end
end
if j~=6, error("UR5 关节数应为 6，实际为 %d。",j); end
end

function gap=localFirstReachabilityGap(reachable)
gap=[];
for i=1:numel(reachable)
    if isempty(reachable{i}) || ~any(reachable{i}), gap=i; return; end
end
end

function p=localEmptyPlan()
p=struct("complete",false,"qPath",zeros(0,6),"stateIndices",zeros(1,0), ...
    "nodeCollisionFree",false,"edgeCollisionFree",false,"maxJointJumpDeg",NaN);
end

function s=localEmptyState()
s=struct("q",zeros(1,6),"travelDeg",0,"rollDeg",0,"Ttarget",eye(4), ...
    "Tactual",eye(4),"positionErrorM",NaN,"orientationErrorDeg",NaN, ...
    "gunAxisWP",zeros(1,3),"sideDot",NaN,"headPositionWP",zeros(1,3), ...
    "environmentCollision",false,"selfCollision",false,"pointIndex",0, ...
    "poseInfo",struct(),"velocityQ",zeros(1,6),"parentQ",zeros(1,6), ...
    "continuationCost",0,"bridgeNode",false,"bridgeFraction",NaN, ...
    "bridgeLevel",0,"bridgeSource","");
end

function s=localEmptyLayerStats()
s=struct("coarseStates",0,"fineStates",0,"elapsedSec",0, ...
    "ikCalls",0,"ikElapsedSec",0,"edgeTests",0, ...
    "edgeCollisionChecks",0);
end

function out=localMergeOptions(defaults,in)
out=defaults; names=fieldnames(defaults);
for i=1:numel(names)
    if isfield(in,names{i}) && ~isempty(in.(names{i})), out.(names{i})=in.(names{i}); end
end
end

function d=localAngleDistanceDeg(a,b)
d=abs(rad2deg(atan2(sind(b-a),cosd(b-a))));
end

function x=localWrapRad(x), x=atan2(sin(x),cos(x)); end

function out=ternary(condition,a,b)
if condition, out=a; else, out=b; end
end

function localAnimate(scene,seam,qPath)
fig=figure("Name","Measured wire-axis weld replan","Color","w");
ax=axes(fig); hold(ax,"on");
for k=1:numel(scene.workpieceCollision), show(scene.workpieceCollision{k},"Parent",ax); end
for i=1:size(qPath,1)
    show(scene.robot,qPath(i,:),"Parent",ax,"Visuals","on", ...
        "Collisions","on","Frames","off","PreservePlot",false);
    hold(ax,"on");
    for k=1:numel(scene.workpieceCollision), show(scene.workpieceCollision{k},"Parent",ax); end
    plot3(ax,seam.positionsMat(:,1),seam.positionsMat(:,2),seam.positionsMat(:,3), ...
        "Color",[0.85 0 0],"LineWidth",2);
    T=getTransform(scene.robot,qPath(i,:),scene.tcpBodyName); p=T(1:3,4);
    plot3(ax,p(1),p(2),p(3),"go","MarkerFaceColor","g","MarkerSize",6);
    axis(ax,"equal"); grid(ax,"on"); view(ax,135,25);
    title(ax,sprintf("Measured gun-axis weld point %d/%d",i,size(qPath,1)));
    drawnow;
end
end

% -------------------------------------------------------------------------
% Bounded local Cartesian bridge fallback
% -------------------------------------------------------------------------

function report = localEmptyCartesianBridgeReport()
report = struct('enabled',true,'attempts',repmat(localEmptyBridgeAttempt(),0,1), ...
    'recovery_count',0,'recovered_point_indices',zeros(1,0), ...
    'failed_point_indices',zeros(1,0),'inserted_nodes',0,'ik_calls',0, ...
    'edge_checks',0,'max_depth_used',0,'min_positive_clearance_mm',NaN, ...
    'selected_path_count',0,'selected_path_added_nodes',0, ...
    'selected_path_min_positive_clearance_mm',NaN, ...
    'selected_path_max_joint_delta_deg',NaN, ...
    'selected_path_max_tcp_error_mm',NaN, ...
    'selected_path_edge_sample_count',0, ...
    'selected_level',0,'selected_fractions',zeros(1,0), ...
    'max_bridge_depth_budget',0,'max_bridge_ik_budget',0, ...
    'max_bridge_time_budget_sec',0, ...
    'attempt_min_positive_clearance_mm',NaN, ...
    'executionReady',false);
end

function a = localEmptyBridgeAttempt()
a = struct('success',false,'level',0,'fractions',zeros(1,0), ...
    'endpoint_q_deg',zeros(1,6),'edge_sample_count',0,'ik_calls',0, ...
    'edge_checks',0,'added_nodes',0,'max_joint_delta_deg',NaN, ...
    'max_tcp_error_mm',NaN,'min_positive_clearance_mm',NaN, ...
    'failure_reason','NO_CONNECTED_CANDIDATE','runtime_sec',0);
end

function p = localEmptyBridgePath()
p = struct('success',false,'status','FAILED','intermediate_states', ...
    repmat(localEmptyState(),0,1),'endpoint_state',localEmptyState(), ...
    'cost',inf,'added_nodes',0,'max_joint_delta_deg',NaN, ...
    'max_tcp_error_mm',NaN,'min_positive_clearance_mm',NaN,'runtime_sec',0);
end

function out = localMergeCartesianBridgeReport(a,b)
out=a;
if isempty(b), return; end
if isfield(b,'attempts') && ~isempty(b.attempts)
    out.attempts=[out.attempts(:);b.attempts(:)];
end
for k={'ik_calls','edge_checks','inserted_nodes'}
    name=k{1}; if isfield(b,name), out.(name)=out.(name)+double(b.(name)); end
end
if isfield(b,'max_depth_used'), out.max_depth_used=max(out.max_depth_used,double(b.max_depth_used)); end
if isfield(b,'min_positive_clearance_mm') && isfinite(b.min_positive_clearance_mm)
    out.min_positive_clearance_mm=localMinFinite(out.min_positive_clearance_mm, ...
        double(b.min_positive_clearance_mm));
end
% Keep the accepted bridge path separate from near-miss/rejected attempts.
% The latter can have a much smaller clearance and must never be reported as
% the clearance of the path that was actually inserted into the ACCP graph.
if isfield(b,'selected_path_count') && double(b.selected_path_count)>0
    out.selected_path_count=out.selected_path_count+double(b.selected_path_count);
    out.selected_path_added_nodes=double(b.selected_path_added_nodes);
    out.selected_path_min_positive_clearance_mm=double(b.selected_path_min_positive_clearance_mm);
    out.selected_path_max_joint_delta_deg=double(b.selected_path_max_joint_delta_deg);
    out.selected_path_max_tcp_error_mm=double(b.selected_path_max_tcp_error_mm);
    out.selected_path_edge_sample_count=double(b.selected_path_edge_sample_count);
    if isfield(b,'selected_level'), out.selected_level=double(b.selected_level); end
    if isfield(b,'selected_fractions'), out.selected_fractions=double(b.selected_fractions); end
    if isfield(b,'max_bridge_depth_budget'), out.max_bridge_depth_budget=double(b.max_bridge_depth_budget); end
    if isfield(b,'max_bridge_ik_budget'), out.max_bridge_ik_budget=double(b.max_bridge_ik_budget); end
    if isfield(b,'max_bridge_time_budget_sec'), out.max_bridge_time_budget_sec=double(b.max_bridge_time_budget_sec); end
end
if isfield(b,'attempt_min_positive_clearance_mm') && isfinite(b.attempt_min_positive_clearance_mm)
    out.attempt_min_positive_clearance_mm=localMinFinite( ...
        out.attempt_min_positive_clearance_mm,double(b.attempt_min_positive_clearance_mm));
end
end

function levels=localCartesianBridgeFractionLevels(options)
base=double(options.cartesianBridgeFractions(:).');
base=sort(unique(base(isfinite(base) & base>0 & base<1)));
if isempty(base), base=[.25 .50 .75]; end
levels={base};
for k=1:max(0,round(options.cartesianBridgeMaxDepth))
    n=2^(k+2); levels{end+1}=((1:n-1)/n); %#ok<AGROW>
end
end

function poses=localBridgePoseList(st,options)
dRoll=[-options.maxPoseStepRollDeg 0 options.maxPoseStepRollDeg];
dTravel=[-options.maxPoseStepTravelDeg 0 options.maxPoseStepTravelDeg];
poses=zeros(0,2);
for it=1:numel(dTravel)
    for ir=1:numel(dRoll)
        poses(end+1,:)=[st.travelDeg+dTravel(it),mod(st.rollDeg+dRoll(ir),360)]; %#ok<AGROW>
    end
end
poses=localUniquePoseList(poses,max(9,options.maxPoseNeighbors), ...
    [st.travelDeg st.rollDeg]);
end

function seeds=localBridgeSeeds(scene,prev,idx,options,pointIndex)
%LOCALBRIDGESEEDS Build a deterministic, bounded IK seed bank for recovery.
% The normal continuation seeds remain first.  Recovery is allowed to add a
% finite bank of random configurations because a Cartesian bridge may need
% to enter a different IK branch than the direct joint-space edge.  The RNG
% state is restored before returning, so this diagnostic fallback never
% changes the reproducibility of the formal ACCP search.
if nargin < 5 || isempty(pointIndex), pointIndex=0; end
q=prev(idx).q;
if isfield(prev(idx),'velocityQ'), qPred=q+prev(idx).velocityQ; else, qPred=q; end
seeds=[qPred;q]; d=deg2rad(options.seedPerturbDeg);
for j=[4 5 6]
    if size(seeds,1)>=options.maxPoseSeeds, break; end
    qq=qPred; qq(j)=qq(j)+d; seeds(end+1,:)=qq; %#ok<AGROW>
    if size(seeds,1)>=options.maxPoseSeeds, break; end
    qq=qPred; qq(j)=qq(j)-d; seeds(end+1,:)=qq; %#ok<AGROW>
end
for k=1:numel(prev)
    if size(seeds,1)>=2*options.maxPoseSeeds, break; end
    seeds(end+1,:)=prev(k).q; %#ok<AGROW>
end
% Structured branch exploration is deterministic and much cheaper than
% making the entire fallback random.  The validated Point19->Point20 branch
% differs mainly in J1/J5 by roughly 20--32 degrees, so cover every joint at
% the configured perturbation magnitudes while retaining all existing
% continuation seeds.
if isfield(options,'cartesianBridgeJointPerturbDeg')
    perturbDeg=double(options.cartesianBridgeJointPerturbDeg(:).');
    perturbDeg=perturbDeg(isfinite(perturbDeg) & perturbDeg>0);
    for jp=1:numel(perturbDeg)
        dd=deg2rad(perturbDeg(jp));
        for jj=1:6
            qq=qPred; qq(jj)=qq(jj)+dd; seeds(end+1,:)=qq; %#ok<AGROW>
            qq=qPred; qq(jj)=qq(jj)-dd; seeds(end+1,:)=qq; %#ok<AGROW>
        end
    end
end
% The independently validated Point19->Point20 bridge used the full numeric
% IK candidate population rather than only the previous beam branch.  A
% bounded deterministic random bank recreates that branch exploration while
% keeping the production search finite and auditable.
nRandom=0;
if isfield(options,'cartesianBridgeRandomSeeds') && ...
        isfinite(options.cartesianBridgeRandomSeeds)
    nRandom=max(0,round(options.cartesianBridgeRandomSeeds));
end
if nRandom>0
    oldRng=rng;
    cleanup=onCleanup(@()rng(oldRng)); %#ok<NASGU>
    rng(double(options.randomSeed)+1009*double(pointIndex)+17*double(idx),'twister');
    for kr=1:nRandom
        try
            qr=double(randomConfiguration(scene.robot));
        catch
            qr=zeros(1,0);
        end
        if isnumeric(qr) && isvector(qr) && numel(qr)==6 && all(isfinite(qr))
            seeds(end+1,:)=reshape(qr,1,6); %#ok<AGROW>
        end
    end
end
seeds=localUniqueQRows(seeds);
end

function [bridge,summary] = localCartesianBridgeFallback(scene,seam,pointIndex,prev,options,candidatePools)
% Generate Cartesian intermediate targets and solve/validate every edge.
% This is a bounded fallback inside ACCP; all nodes are produced by the
% existing hard-filtered IK interface and all edges are densely checked.
tAll=tic;
bridge=struct('success',false,'failure_reason','NO_CONNECTED_CANDIDATE', ...
    'inserted_layer_count',0,'paths',repmat(localEmptyBridgePath(),0,1), ...
    'selected_level',0,'selected_fractions',zeros(1,0),'ik_calls',0, ...
    'edge_checks',0,'min_positive_clearance_mm',NaN, ...
    'selected_path_count',0,'selected_path_added_nodes',0, ...
    'selected_path_min_positive_clearance_mm',NaN, ...
    'selected_path_max_joint_delta_deg',NaN, ...
    'selected_path_max_tcp_error_mm',NaN, ...
    'selected_path_edge_sample_count',0, ...
    'max_bridge_depth_budget',options.cartesianBridgeMaxDepth, ...
    'max_bridge_ik_budget',options.cartesianBridgeMaxIKCalls, ...
    'max_bridge_time_budget_sec',options.cartesianBridgeMaxTimeSec, ...
    'attempt_min_positive_clearance_mm',NaN);
summary=struct('attempts',repmat(localEmptyBridgeAttempt(),0,1), ...
    'ik_calls',0,'edge_checks',0,'inserted_nodes',0,'max_depth_used',0, ...
    'min_positive_clearance_mm',NaN,'selected_path_count',0, ...
    'selected_path_added_nodes',0,'selected_path_min_positive_clearance_mm',NaN, ...
    'selected_path_max_joint_delta_deg',NaN,'selected_path_max_tcp_error_mm',NaN, ...
    'selected_path_edge_sample_count',0,'selected_level',0, ...
    'selected_fractions',zeros(1,0),'max_bridge_depth_budget',options.cartesianBridgeMaxDepth, ...
    'max_bridge_ik_budget',options.cartesianBridgeMaxIKCalls, ...
    'max_bridge_time_budget_sec',options.cartesianBridgeMaxTimeSec, ...
    'attempt_min_positive_clearance_mm',NaN);
if isempty(prev), bridge.failure_reason='NO_PREVIOUS_LAYER'; return; end
if nargin<6 || isempty(candidatePools), candidatePools={}; end
levels=localCartesianBridgeFractionLevels(options);
maxPaths=max(1,round(options.cartesianBridgeMaxPaths));
endpoints=repmat(localEmptyState(),0,1);
rawEndpointRows=0; freeEndpointRows=0;
% Prefer the exact numeric candidate pool retained by the failed direct
% layer.  This is what allows the bridge to reconnect a branch that was
% rejected solely by the direct edge/step gate.
for kp=1:numel(candidatePools)
    pool=candidatePools{kp};
    if ~isfield(pool,'rows'), continue; end
    for kd=1:numel(pool.rows)
        dc=pool.rows(kd); qd=double(dc.q_rad(:).');
        if ~localFullStateFree(scene,qd), continue; end
        st0=localEmptyState(); st0.q=qd; st0.Ttarget=pool.Ttarget;
        st0.Tactual=double(dc.tcp_pose); st0.positionErrorM=dc.position_error_mm/1000;
        st0.orientationErrorDeg=dc.orientation_error_deg;
        st0.travelDeg=pool.travelDeg; st0.rollDeg=pool.rollDeg;
        st0.gunAxisWP=localUnitRow(st0.Tactual(1:3,1:3)*scene.wireAxisTcp(:));
        st0.pointIndex=pointIndex; st0.poseInfo=pool.poseInfo;
        st0.bridgeNode=false; st0.bridgeFraction=1; st0.bridgeLevel=0;
        st0.bridgeSource="cartesian_bridge_endpoint_pool";
        endpoints(end+1,1)=st0; %#ok<AGROW>
    end
end
for ip=1:numel(prev)
    if toc(tAll)>options.cartesianBridgeMaxTimeSec
        bridge.failure_reason='BRIDGE_TIME_BUDGET_EXCEEDED'; break;
    end
    poseList=localBridgePoseList(prev(ip),options);
    seeds=localBridgeSeeds(scene,prev,ip,options,pointIndex);
    for jp=1:size(poseList,1)
        if toc(tAll)>options.cartesianBridgeMaxTimeSec, break; end
        [Ttarget,poseInfo]=localTargetForPose(scene,seam,pointIndex, ...
            poseList(jp,1),poseList(jp,2));
        endpointOptions=options;
        endpointOptions.diagnosticCollectIKCandidates=true;
        % Re-evaluate the endpoint candidate population rather than reusing
        % a cached continuation result that intentionally discarded rejected
        % node candidates.
        endpointOptions.reuseCandidateCache=false;
        [rr,~]=localTimedIKCall(scene,Ttarget,poseList(jp,1), ...
            poseList(jp,2),seeds,endpointOptions);
        summary.ik_calls=summary.ik_calls+rr.numIKCalls;
        bridge.ik_calls=bridge.ik_calls+rr.numIKCalls;
        if bridge.ik_calls>options.cartesianBridgeMaxIKCalls
            bridge.failure_reason='BRIDGE_IK_BUDGET_EXCEEDED'; break;
        end
        endpointRows=rr.states;
        % The ordinary generator intentionally removes node-colliding
        % endpoints before returning states.  A bridge must inspect those
        % numeric/pose-valid candidates as possible endpoints, then apply the
        % complete node check here; this is the same candidate population used
        % by the independent Point19->Point20 audit.
        if isempty(endpointRows) && isfield(rr,'diagnosticCandidates') && ...
                ~isempty(rr.diagnosticCandidates)
            endpointRows=repmat(localEmptyState(),0,1);
            for kd=1:numel(rr.diagnosticCandidates)
                dc=rr.diagnosticCandidates(kd);
                qd=double(dc.q_rad(:).');
                rawEndpointRows=rawEndpointRows+1;
                if ~localFullStateFree(scene,qd), continue; end
                freeEndpointRows=freeEndpointRows+1;
                st0=localEmptyState(); st0.q=qd; st0.Ttarget=Ttarget;
                st0.Tactual=double(dc.tcp_pose); st0.positionErrorM=dc.position_error_mm/1000;
                st0.orientationErrorDeg=dc.orientation_error_deg;
                st0.travelDeg=poseList(jp,1); st0.rollDeg=poseList(jp,2);
                st0.gunAxisWP=localUnitRow(st0.Tactual(1:3,1:3)*scene.wireAxisTcp(:));
                endpointRows(end+1,1)=st0; %#ok<AGROW>
            end
        end
        for ks=1:numel(endpointRows)
            st=endpointRows(ks); st.pointIndex=pointIndex; st.poseInfo=poseInfo;
            st.bridgeNode=false; st.bridgeFraction=1; st.bridgeLevel=0;
            st.bridgeSource="cartesian_bridge_endpoint";
            st.parentQ=zeros(1,6); st.velocityQ=zeros(1,6); st.continuationCost=0;
            endpoints(end+1,1)=st; %#ok<AGROW>
        end
    end
    if bridge.ik_calls>options.cartesianBridgeMaxIKCalls, break; end
end
endpoints=localAppendUniqueStates(repmat(localEmptyState(),0,1),endpoints,0.5);
if isempty(endpoints)
    if isempty(bridge.failure_reason) || strcmp(bridge.failure_reason,'NO_CONNECTED_CANDIDATE')
        bridge.failure_reason='NO_CONSTRAINED_NODE_CANDIDATE';
    end
    attempt=localEmptyBridgeAttempt(); attempt.failure_reason=bridge.failure_reason;
    summary.attempts(end+1,1)=attempt;
    return;
end

for ilevel=1:numel(levels)
    fractions=levels{ilevel};
    summary.max_depth_used=max(summary.max_depth_used,ilevel-1);
    successes=repmat(localEmptyBridgePath(),0,1);
    for ke=1:numel(endpoints)
        if toc(tAll)>options.cartesianBridgeMaxTimeSec
            bridge.failure_reason='BRIDGE_TIME_BUDGET_EXCEEDED'; break;
        end
        [path,attempt]=localTryCartesianBridgePath(scene,prev,endpoints(ke), ...
            fractions,options,ilevel,tAll,bridge.ik_calls);
        bridge.ik_calls=bridge.ik_calls+attempt.ik_calls;
        bridge.edge_checks=bridge.edge_checks+attempt.edge_checks;
        summary.ik_calls=summary.ik_calls+attempt.ik_calls;
        summary.edge_checks=summary.edge_checks+attempt.edge_checks;
        summary.attempts(end+1,1)=attempt; %#ok<AGROW>
        summary.min_positive_clearance_mm=localMinFinite( ...
            summary.min_positive_clearance_mm,attempt.min_positive_clearance_mm);
        if path.success
            successes(end+1,1)=path; %#ok<AGROW>
            if numel(successes)>=maxPaths, break; end
        end
        if bridge.ik_calls>=options.cartesianBridgeMaxIKCalls, break; end
    end
    if ~isempty(successes)
        costs=vertcat(successes.cost); [~,ord]=sort(costs,'ascend');
        successes=successes(ord(1:min(maxPaths,numel(ord))));
        bridge.success=true; bridge.failure_reason=""; bridge.paths=successes;
        bridge.selected_level=ilevel; bridge.selected_fractions=fractions;
        bridge.inserted_layer_count=numel(fractions);
        bridge.min_positive_clearance_mm=min([successes.min_positive_clearance_mm]);
        % `successes(1)` is the accepted/selected bridge path after the cost
        % ordering above.  Preserve its metrics separately from the global
        % attempt minimum (which includes rejected near misses).
        bridge.selected_path_count=1;
        bridge.selected_path_added_nodes=successes(1).added_nodes;
        bridge.selected_path_min_positive_clearance_mm=successes(1).min_positive_clearance_mm;
        bridge.selected_path_max_joint_delta_deg=successes(1).max_joint_delta_deg;
        bridge.selected_path_max_tcp_error_mm=successes(1).max_tcp_error_mm;
        bridge.selected_path_edge_sample_count=options.cartesianBridgeMaxEdgeSamples;
        bridge.selected_level=ilevel;
        bridge.selected_fractions=fractions;
        bridge.attempt_min_positive_clearance_mm=summary.min_positive_clearance_mm;
        summary.selected_path_count=1;
        summary.selected_path_added_nodes=successes(1).added_nodes;
        summary.selected_path_min_positive_clearance_mm=successes(1).min_positive_clearance_mm;
        summary.selected_path_max_joint_delta_deg=successes(1).max_joint_delta_deg;
        summary.selected_path_max_tcp_error_mm=successes(1).max_tcp_error_mm;
        summary.selected_path_edge_sample_count=options.cartesianBridgeMaxEdgeSamples;
        summary.selected_level=ilevel;
        summary.selected_fractions=fractions;
        summary.max_bridge_depth_budget=options.cartesianBridgeMaxDepth;
        summary.max_bridge_ik_budget=options.cartesianBridgeMaxIKCalls;
        summary.max_bridge_time_budget_sec=options.cartesianBridgeMaxTimeSec;
        summary.attempt_min_positive_clearance_mm=summary.min_positive_clearance_mm;
        summary.inserted_nodes=summary.inserted_nodes+numel(fractions)*numel(successes);
        summary.min_positive_clearance_mm=localMinFinite( ...
            summary.min_positive_clearance_mm,bridge.min_positive_clearance_mm);
        break;
    end
    if bridge.ik_calls>=options.cartesianBridgeMaxIKCalls
        bridge.failure_reason='BRIDGE_IK_BUDGET_EXCEEDED'; break;
    end
end
if ~bridge.success && isempty(bridge.failure_reason), bridge.failure_reason='NO_CONNECTED_CANDIDATE'; end
bridge.edge_checks=summary.edge_checks;
bridge.attempt_min_positive_clearance_mm=summary.min_positive_clearance_mm;
end

function [path,attempt]=localTryCartesianBridgePath(scene,prev,endpoint, ...
    fractions,options,level,tAll,priorIkCalls)
t0=tic; path=localEmptyBridgePath(); attempt=localEmptyBridgeAttempt();
attempt.level=level; attempt.fractions=fractions;
attempt.endpoint_q_deg=rad2deg(endpoint.q);
attempt.edge_sample_count=options.cartesianBridgeMaxEdgeSamples;
attempt.failure_reason='NO_CONNECTED_CANDIDATE';
for ip=1:numel(prev)
    start=prev(ip); T0=start.Tactual; T1=endpoint.Ttarget;
    startNode=struct('q',start.q,'state',start,'cost',0,'parent',0);
    layers={startNode}; frontier=startNode; failed=''; minGap=NaN;
    for jf=1:numel(fractions)
        if toc(tAll)>options.cartesianBridgeMaxTimeSec || ...
                priorIkCalls+attempt.ik_calls>=options.cartesianBridgeMaxIKCalls
            failed='BRIDGE_BUDGET_EXCEEDED'; break;
        end
        alpha=fractions(jf); Tgoal=localInterpolateTcpPose(T0,T1,alpha);
        seeds=localUniqueQRows([vertcat(frontier.q);endpoint.q;start.q]);
        rr=generate_actual_axis_ik_candidates(scene,Tgoal, ...
            localInterpolateAngle(start.travelDeg,endpoint.travelDeg,alpha), ...
            localInterpolateAngle(start.rollDeg,endpoint.rollDeg,alpha),seeds, ...
            localBridgeCandidateOptions(options,start.q));
        attempt.ik_calls=attempt.ik_calls+rr.numIKCalls;
        next=repmat(struct('q',zeros(1,6),'state',localEmptyState(), ...
            'cost',0,'parent',0),0,1);
        nodeStep=false; nodeEdge=false;
        for ks=1:numel(rr.states)
            % generate_actual_axis_ik_candidates owns a smaller state
            % structure.  Normalize it before putting it into the bridge
            % graph so bridge layers have the same schema as ordinary ACCP
            % layers (and can be inserted without heterogeneous-struct
            % assignment errors).
            st=localNormalizeBridgeState(rr.states(ks));
            st.pointIndex=endpoint.pointIndex;
            st.bridgeNode=true; st.bridgeFraction=alpha; st.bridgeLevel=level;
            st.bridgeSource="ACCP_local_cartesian_bridge";
            for jp=1:numel(frontier)
                dq=localWrapRad(st.q-frontier(jp).q);
                if max(abs(rad2deg(dq)))>options.maxJointStepDeg+1e-9
                    nodeStep=true; continue;
                end
                [ok,gap]=localDenseEdgeFreeWithGap(scene,frontier(jp).q,st.q, ...
                    options,options.cartesianBridgeMaxEdgeSamples);
                attempt.edge_checks=attempt.edge_checks+1;
                if isfinite(gap), minGap=localMinFinite(minGap,gap); end
                if ~ok, nodeEdge=true; continue; end
                st.parentQ=frontier(jp).q; st.velocityQ=dq;
                st.continuationCost=frontier(jp).cost+sum(dq.^2);
                n=struct('q',st.q,'state',st,'cost',frontier(jp).cost+sum(dq.^2), ...
                    'parent',jp);
                next(end+1,1)=n; %#ok<AGROW>
            end
        end
        if isempty(next)
            if nodeEdge, failed='NODE_CANDIDATES_BUT_EDGE_COLLISION';
            elseif nodeStep, failed='IK_CONFIGURATION_BRANCH_DISCONTINUITY';
            elseif isempty(rr.states), failed='NO_CONSTRAINED_NODE_CANDIDATE';
            else, failed='NO_CONNECTED_CANDIDATE'; end
            break;
        end
        [~,ord]=sort([next.cost],'ascend');
        next=next(ord(1:min(options.cartesianBridgeBeamWidth,numel(ord))));
        frontier=next; layers{end+1}=frontier; %#ok<AGROW>
    end
    if isempty(failed)
        endpointEdge=false; endpointStep=false; finalNode=[]; finalCost=inf;
        for jp=1:numel(frontier)
            dq=localWrapRad(endpoint.q-frontier(jp).q);
            if max(abs(rad2deg(dq)))>options.maxJointStepDeg+1e-9
                endpointStep=true; continue;
            end
            [ok,gap]=localDenseEdgeFreeWithGap(scene,frontier(jp).q,endpoint.q, ...
                options,options.cartesianBridgeMaxEdgeSamples);
            attempt.edge_checks=attempt.edge_checks+1;
            if isfinite(gap), minGap=localMinFinite(minGap,gap); end
            if ~ok, endpointEdge=true; continue; end
            c=frontier(jp).cost+sum(dq.^2);
            if c<finalCost, finalCost=c; finalNode=jp; end
        end
        if isempty(finalNode)
            if endpointEdge, failed='NODE_CANDIDATE_BUT_FINAL_EDGE_COLLISION';
            elseif endpointStep, failed='FINAL_IK_CONFIGURATION_BRANCH_DISCONTINUITY';
            else, failed='NO_CONNECTED_CANDIDATE'; end
        else
            selected=repmat(localEmptyState(),0,1); idx=finalNode;
            for jl=numel(layers):-1:2
                n=layers{jl}(idx); selected(end+1,1)=n.state; idx=n.parent;
            end
            selected=selected(end:-1:1);
            ep=endpoint; ep.bridgeNode=true; ep.bridgeFraction=1; ep.bridgeLevel=level;
            ep.bridgeSource="ACCP_local_cartesian_bridge_endpoint";
            if ~isempty(selected), ep.parentQ=selected(end).q; ep.velocityQ=localWrapRad(ep.q-ep.parentQ); end
            path=localEmptyBridgePath(); path.success=true; path.status='PASS';
            path.intermediate_states=selected; path.endpoint_state=ep;
            path.cost=finalCost; path.added_nodes=numel(selected);
            path.max_joint_delta_deg=maxBridgePathJointDelta(start,selected,ep);
            path.max_tcp_error_mm=maxBridgePathTcpError(scene,selected,ep,T0,T1,fractions);
            path.min_positive_clearance_mm=minGap; path.runtime_sec=toc(t0);
            attempt.success=true; attempt.failure_reason=''; attempt.added_nodes=numel(selected);
            attempt.max_joint_delta_deg=path.max_joint_delta_deg;
            attempt.max_tcp_error_mm=path.max_tcp_error_mm;
            attempt.min_positive_clearance_mm=minGap; attempt.runtime_sec=toc(t0);
            return;
        end
    end
    attempt.failure_reason=failed; attempt.min_positive_clearance_mm=minGap;
end
attempt.runtime_sec=toc(t0); path.runtime_sec=attempt.runtime_sec;
end

function out=localNormalizeBridgeState(st)
out=localEmptyState();
f=fieldnames(st);
for k=1:numel(f)
    name=f{k};
    if isfield(out,name), out.(name)=st.(name); end
end
end

function co=localBridgeCandidateOptions(options,qRef)
co=localCandidateOptions(options); co.checkSelfCollision=true; co.lazyCollision=false;
co.reuseCandidateCache=false; co.diagnosticCollectIKCandidates=true;
co.diagnosticReferenceQ=qRef; co.diagnosticMaxJointDeltaDeg=inf;
co.disableAxisConstraint=false; co.disable_axis_constraint=false;
end

function [stateBank,seam,layerStats,selectionDiagnostics] = localInsertCartesianBridge( ...
    scene,stateBank,seam,layerStats,selectionDiagnostics,pointIndex,bridge,options)
paths=bridge.paths; m=bridge.inserted_layer_count;
bridgeLayers=cell(1,m); for j=1:m, bridgeLayers{j}=repmat(localEmptyState(),0,1); end
endpointStates=repmat(localEmptyState(),0,1);
for k=1:numel(paths)
    ns=paths(k).intermediate_states;
    for j=1:m, bridgeLayers{j}(end+1,1)=ns(j); end
    endpointStates(end+1,1)=paths(k).endpoint_state; %#ok<AGROW>
end
p0=seam.positionsWP(pointIndex-1,:); p1=seam.positionsWP(pointIndex,:);
fractions=bridge.selected_fractions(:);
mids=(1-fractions)*p0+fractions*p1;
seam.positionsWP=[seam.positionsWP(1:pointIndex-1,:);mids;seam.positionsWP(pointIndex:end,:)];
seam.positionsMat=(scene.T_MatBase_Workpiece*[seam.positionsWP, ...
    ones(size(seam.positionsWP,1),1)].').';
seam.positionsMat=seam.positionsMat(:,1:3); seam.numPoints=size(seam.positionsWP,1);
stateBank=[stateBank(1:pointIndex-1),bridgeLayers,{endpointStates}, ...
    stateBank(pointIndex+1:end)];
bridgeStats=repmat(localEmptyLayerStats(),1,m+1);
for j=1:m, bridgeStats(j).coarseStates=numel(bridgeLayers{j}); end
bridgeStats(m+1).coarseStates=numel(endpointStates);
layerStats=[layerStats(1:pointIndex-1),bridgeStats,layerStats(pointIndex+1:end)];
if ~isempty(selectionDiagnostics)
    bridgeAudit=repmat({struct('point_status','CARTESIAN_BRIDGE', ...
        'candidates',[])},1,m);
    selectionDiagnostics=[selectionDiagnostics(1:pointIndex-1),bridgeAudit, ...
        {struct('point_status','CARTESIAN_BRIDGE_ENDPOINT','candidates',[])}, ...
        selectionDiagnostics(pointIndex+1:end)];
end
if options.cartesianBridgeMaxEdgeSamples<101
    error('Cartesian bridge edge sampling must be at least 101.');
end
end

function [ok,gap]=localDenseEdgeFreeWithGap(scene,q1,q2,options,nSamples)
nSamples=max(101,round(nSamples)); dq=localWrapRad(q2-q1); ok=true; gap=NaN;
for k=0:nSamples-1
    q=q1+(k/(nSamples-1))*dq;
    [free,g]=localStateFreeWithGap(scene,q);
    if isfinite(g), gap=localMinFinite(gap,g); end
    if ~free, ok=false; return; end
end
end

function [ok,gap]=localStateFreeWithGap(scene,q)
gap=NaN;
try
    [env,distEnv]=checkCollision(scene.robot,q,scene.environment, ...
        'IgnoreSelfCollision','on','Exhaustive','on');
catch
    [env,distEnv]=checkCollision(scene.robot,q,scene.environment, ...
        'IgnoreSelfCollision','on');
end
try
    [self,distSelf]=checkCollision(scene.robot,q,'IgnoreSelfCollision','off', ...
        'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
catch
    [self,distSelf]=checkCollision(scene.robot,q,'IgnoreSelfCollision','off', ...
        'SkippedSelfCollisions',scene.selfCollisionSkipPairs);
end
gap=localDistanceGapMm([distEnv(:);distSelf(:)]);
ok=~any(env(:)) && ~any(self(:));
end

function g=localDistanceGapMm(d)
d=double(d(isfinite(d) & d>0));
if isempty(d), g=NaN; else, g=min(d)*1000; end
end

function g=localMinFinite(a,b)
if ~isfinite(b), g=a; elseif ~isfinite(a), g=b; else, g=min(a,b); end
end

function rows=localUniqueQRows(rows)
if isempty(rows), return; end
keep=true(size(rows,1),1);
for i=1:size(rows,1)
    if ~keep(i), continue; end
    for j=i+1:size(rows,1)
        if keep(j) && max(abs(localWrapRad(rows(i,:)-rows(j,:))))<1e-9
            keep(j)=false;
        end
    end
end
rows=rows(keep,:);
end

function v=localUnitRow(v)
v=double(v(:)); n=norm(v);
if n<1e-12 || any(~isfinite(v)), v=zeros(1,3); else, v=(v/n).'; end
end

function T=localInterpolateTcpPose(T0,T1,a)
T=eye(4); T(1:3,4)=(1-a)*T0(1:3,4)+a*T1(1:3,4);
q0=rotm2quat(T0(1:3,1:3)); q1=rotm2quat(T1(1:3,1:3));
if dot(q0,q1)<0, q1=-q1; end
d=max(-1,min(1,dot(q0,q1)));
if d>0.9995, q=(1-a)*q0+a*q1; q=q/norm(q);
else, th=acos(d); q=(sin((1-a)*th)*q0+sin(a*th)*q1)/sin(th); end
T(1:3,1:3)=quat2rotm(q/norm(q));
end

function a=localInterpolateAngle(a0,a1,t)
a=double(a0)+t*rad2deg(localWrapRad(deg2rad(double(a1)-double(a0))));
end

function d=maxBridgePathJointDelta(start,states,endpoint)
q=[start.q;vertcat(states.q);endpoint.q];
if size(q,1)<2, d=0; else, d=max(abs(rad2deg(diff(q,1,1))),[],'all'); end
end

function e=maxBridgePathTcpError(~,states,endpoint,T0,T1,fractions)
e=0;
for k=1:numel(states)
    Tg=localInterpolateTcpPose(T0,T1,fractions(k));
    e=max(e,norm(states(k).Tactual(1:3,4)-Tg(1:3,4))*1000);
end
if isempty(states), e=norm(endpoint.Tactual(1:3,4)-T1(1:3,4))*1000; end
end
