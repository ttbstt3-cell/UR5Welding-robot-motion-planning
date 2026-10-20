function report = diagnose_accp_point19_point20_local_recovery(inputFile, outputDir)
%DIAGNOSE_ACCP_POINT19_POINT20_LOCAL_RECOVERY
%
% Diagnostic-only investigation of the real-scene ACCP Point19 -> Point20
% failure. This file does not change production ACCP, CBi-RRT*, scene files,
% collision filters, TCP, or any trajectory/anchor file.
%
% It performs:
%   1) a 101-sample joint-space edge audit for existing edge candidates;
%   2) a Cartesian local-connection experiment using IK at 25/50/75 percent
%      (and dyadic diagnostic refinements if needed).
%
% Output is marked DIAGNOSTIC_PASS only. It is never execution-ready.

if nargin < 1 || isempty(inputFile)
    inputFile = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
        '规划文件','real_anchor_runs','full_accp_wire3_20261013', ...
        'real_seam_round_recovery_beam3.mat');
end
if nargin < 2 || isempty(outputDir)
    outputDir = fullfile(fileparts(inputFile), ...
        'point20_local_recovery_20261013');
end
if ~isfolder(outputDir), mkdir(outputDir); end

tAll = tic;
loaded = load(inputFile, 'result');
assert(isfield(loaded, 'result'), 'Saved ACCP MAT does not contain result.');
result = loaded.result;
scene = result.scene;
scene.selfCollisionSkipPairs = ignore_fixed_gun_wire_self_collision( ...
    scene.robot, scene.selfCollisionSkipPairs);

rootDir = fileparts(fileparts(mfilename('fullpath')));
auditDir = fullfile(fileparts(inputFile), 'point20_audit_20261013');
summaryFile = fullfile(auditDir, 'point20_filter_summary.json');
topFile = fullfile(auditDir, 'point20_top10_nearest.json');
fullReportFile = fullfile(fileparts(inputFile), 'report.json');
assert(isfile(summaryFile), 'Existing Point20 summary is missing: %s', summaryFile);
assert(isfile(topFile), 'Existing Point20 top-candidate file is missing: %s', topFile);
assert(isfile(fullReportFile), 'Full ACCP report is missing: %s', fullReportFile);
point20Summary = jsondecode(fileread(summaryFile));
topPayload = jsondecode(fileread(topFile));
fullAccpReport = jsondecode(fileread(fullReportFile));
topCandidates = topPayload.candidates;
if isempty(topCandidates)
    topCandidates = repmat(emptyCandidate(), 0, 1);
end

snapshot = verifySnapshot(rootDir, result, scene);
writeJson(fullfile(outputDir, 'scene_snapshot_check.json'), snapshot);

% A mismatch is a hard STALE result, never a reason to silently rebuild or
% relax the scene.
if ~snapshot.match
    report = baseReport(inputFile, outputDir, result, point20Summary, ...
        fullAccpReport, snapshot);
    report.status = 'STALE';
    report.diagnostic_pass = false;
    report.executionReady = false;
    report.reason = 'Scene/calibration snapshot differs from the saved Point20 audit.';
    report.full_accp_report_file = fullReportFile;
    writeJson(fullfile(outputDir, 'local_recovery_summary.json'), report);
    fprintf('Point20 diagnostic ABORTED: STALE scene snapshot.\n');
    return;
end

assert(abs(double(scene.wireCollisionShortenMm) - 3) < 1e-9, ...
    'The saved diagnostic scene does not use wire_collision_shorten_mm=3.');
assert(abs(double(result.options.wire_collision_shorten_mm) - 3) < 1e-9, ...
    'The saved ACCP options do not use wire_collision_shorten_mm=3.');

prevLayer = result.stateBank{19};
assert(~isempty(prevLayer), 'Point19 has no valid ACCP state.');
prevState = prevLayer(1);
q19 = double(prevState.q(:).');
o = result.options;

% Task 1: audit existing endpoint candidates without regenerating or changing
% production search. Exactly 101 points (including endpoints) are sampled.
edgeCandidates = filterCandidates(topCandidates, 'edge_collision');
stepCandidates = filterCandidates(topCandidates, 'joint_step');
edgeAudits = repmat(emptyEdgeAudit(), 0, 1);
for k = 1:min(3, numel(edgeCandidates))
    edgeAudits(end+1,1) = auditDenseEdge(scene, q19, ...
        double(edgeCandidates(k).q_rad(:).'), 101, ...
        sprintf('edge_candidate_%d', k)); %#ok<AGROW>
end
writeJson(fullfile(outputDir, 'point19_point20_edge_audits.json'), ...
    struct('schema_version','ACCP_POINT19_POINT20_EDGE_AUDIT_v1', ...
    'point19',19,'point20',20,'sample_count',101, ...
    'wire_collision_shorten_mm',3,'audits',{edgeAudits}, ...
    'diagnostic_only',true,'executionReady',false, ...
    'calibration_binding',scene.calibrationBinding));

% A step-limit candidate is not accepted merely because it could be split in
% joint space. The local recovery must find IK states at Cartesian poses and
% must pass complete edge collision checks.
stepAudits = repmat(emptyStepAudit(), 0, 1);
for k = 1:min(2, numel(stepCandidates))
    q20 = double(stepCandidates(k).q_rad(:).');
    dq = wrapRad(q20 - q19);
    a = emptyStepAudit();
    a.rank = k;
    a.q19_deg = rad2deg(q19);
    a.q20_deg = rad2deg(q20);
    a.delta_deg = rad2deg(dq);
    a.absolute_delta_deg = abs(a.delta_deg);
    a.max_joint_delta_deg = max(a.absolute_delta_deg);
    a.max_joint_step_limit_deg = double(o.maxJointStepDeg);
    a.direct_joint_step_pass = a.max_joint_delta_deg <= ...
        a.max_joint_step_limit_deg + 1e-9;
    a.candidate_id = sprintf('step_candidate_%d', k);
    a.continuous_branch_status = 'PENDING_LOCAL_IK_RECOVERY';
    stepAudits(end+1,1) = a; %#ok<AGROW>
end

% Task 2: independent local Cartesian recovery. Every new node is produced
% by existing numeric IK; no q-linear waypoint is inserted.
endpointCandidates = [edgeCandidates(:); stepCandidates(:)];
endpointCandidates = uniqueCandidateRows(endpointCandidates);
if isempty(endpointCandidates)
    recovery = struct('status','NO_ENDPOINT_CANDIDATES', ...
        'attempts',repmat(emptyRecoveryAttempt(),0,1), ...
        'ik_calls',0,'added_nodes',0,'executionReady',false, ...
        'diagnosticPass',false);
else
    fractionsList = { [0.25 0.50 0.75], ...
        [0.125 0.25 0.375 0.50 0.625 0.75 0.875], ...
        [0.0625 0.125 0.1875 0.25 0.375 0.50 0.625 0.75 ...
         0.8125 0.875 0.9375] };
    attempts = repmat(emptyRecoveryAttempt(), 0, 1);
    recovery = struct('status','NO_VALID_LOCAL_CONNECTION', ...
        'attempts',attempts,'ik_calls',0,'added_nodes',0, ...
        'executionReady',false,'diagnosticPass',false);
    for k = 1:numel(endpointCandidates)
        for level = 1:numel(fractionsList)
            attempt = tryCartesianConnection(scene, prevState, ...
                endpointCandidates(k), topCandidates, fractionsList{level}, ...
                o, level, k);
            attempts(end+1,1) = attempt; %#ok<AGROW>
            recovery.ik_calls = recovery.ik_calls + attempt.ik_calls;
            recovery.added_nodes = recovery.added_nodes + attempt.added_nodes;
            if attempt.success
                % Keep the first successful local bridge as the headline
                % result, but continue auditing the remaining endpoint
                % branches so the two step-limit candidates are classified
                % independently.
                if ~recovery.diagnosticPass
                    recovery.status = 'DIAGNOSTIC_PASS';
                    recovery.diagnosticPass = true;
                    recovery.executionReady = false;
                    recovery.success_attempt = attempt;
                end
                break;
            end
        end
    end
    recovery.attempts = attempts;
end

% Back-fill the legal-branch decision for the two direct step candidates.
for k = 1:numel(stepAudits)
    qTarget = stepAudits(k).q20_deg;
    for j = 1:numel(recovery.attempts)
        if recovery.attempts(j).success && ...
                max(abs(recovery.attempts(j).endpoint_q_deg - qTarget)) < 1e-6
            stepAudits(k).continuous_branch_status = ...
                'FOUND_IN_DIAGNOSTIC_CARTESIAN_RECOVERY';
            break;
        end
    end
    if strcmp(stepAudits(k).continuous_branch_status, 'PENDING_LOCAL_IK_RECOVERY')
        stepAudits(k).continuous_branch_status = ...
            'NOT_FOUND_IN_DIAGNOSTIC_SEARCH';
    end
end
writeJson(fullfile(outputDir, 'point19_point20_step_candidate_audit.json'), ...
    struct('schema_version','ACCP_POINT19_POINT20_STEP_AUDIT_v1', ...
    'audits',{stepAudits},'diagnostic_only',true,'executionReady',false, ...
    'calibration_binding',scene.calibrationBinding));

writeJson(fullfile(outputDir, 'full_accp_report.json'), fullAccpReport);
report = baseReport(inputFile, outputDir, result, point20Summary, ...
    fullAccpReport, snapshot);
report.full_accp_report_file = fullReportFile;
report.status = recovery.status;
report.diagnostic_pass = logical(recovery.diagnosticPass);
report.executionReady = false;
report.formal_accp_modified = false;
report.formal_trajectory_written = false;
report.anchor_file_written = false;
report.original_connection = struct('success',false, ...
    'status','NO_CONNECTED_CANDIDATE','point19',19,'point20',20, ...
    'point20_audit_summary',point20Summary.filter_counts, ...
    'edge_candidate_count',numel(edgeCandidates), ...
    'joint_step_candidate_count',numel(stepCandidates));
report.edge_audits = edgeAudits;
report.step_audits = stepAudits;
report.local_recovery = recovery;
report.runtime_sec = toc(tAll);
report.note = ['Independent diagnostic experiment only. No formal ACCP parameter, ', ...
    'collision filter, TCP, geometry, anchor or trajectory file was changed.'];
writeJson(fullfile(outputDir, 'local_recovery_summary.json'), report);

fprintf('\nPoint19 -> Point20 local diagnostic: %s\n', report.status);
fprintf('  existing connection: NO_CONNECTED_CANDIDATE\n');
fprintf('  dense edge audits: %d (101 samples each)\n', numel(edgeAudits));
fprintf('  local recovery IK calls: %d, added IK nodes: %d\n', ...
    recovery.ik_calls, recovery.added_nodes);
if report.diagnostic_pass
    a = recovery.success_attempt;
    fprintf('  DIAGNOSTIC_PASS: %d intermediate nodes, max TCP error %.6f mm, max joint step %.6f deg\n', ...
        a.added_nodes, a.max_tcp_error_mm, a.max_joint_delta_deg);
else
    fprintf('  no valid local Cartesian connection found.\n');
end
end

function report = baseReport(inputFile, outputDir, result, point20Summary, ...
    fullAccpReport, snapshot)
report = struct('schema_version','ACCP_POINT19_POINT20_LOCAL_RECOVERY_v1', ...
    'input_file',inputFile,'output_dir',outputDir, ...
    'point_index',20,'previous_point_index',19, ...
    'robot_motion','NO','diagnostic_only',true, ...
    'calibration_binding',result.scene.calibrationBinding, ...
    'wire_collision_shorten_mm',double(result.scene.wireCollisionShortenMm), ...
    'scene_snapshot',snapshot,'point20_summary',point20Summary, ...
    'full_accp_report',fullAccpReport, ...
    'status','UNSET','diagnostic_pass',false,'executionReady',false, ...
    'formal_accp_modified',false,'formal_trajectory_written',false, ...
    'anchor_file_written',false);
end

function snapshot = verifySnapshot(rootDir, result, scene)
% Compare every byte-level binding used by the failed run.
expected = result.scene.calibrationBinding;
files = { ...
    'tcp_calibration_sha256', fullfile(rootDir,'real_tcp_calibration_active.json'); ...
    'real_gun_geometry_sha256', fullfile(rootDir,'real_gun_geometry','real_gun_geometry.json'); ...
    'gun_mount_sha256', fullfile(rootDir,'real_gun_mount_calibration.json'); ...
    'workpiece_sha256', fullfile(rootDir,'workpiece_pose_real.json'); ...
    'urdf_sha256', fullfile(rootDir,'real_robot_description_calibrated.urdf')};
rows = repmat(struct('binding_key','','path','','exists',false, ...
    'expected_hash','','current_hash','','match',false), 0, 1);
allMatch = true;
for k = 1:size(files,1)
    row = struct('binding_key',files{k,1},'path',files{k,2}, ...
        'exists',false,'expected_hash','','current_hash','','match',false);
    row.exists = isfile(row.path);
    if row.exists, row.current_hash = sha256File(row.path); end
    if isfield(expected, files{k,1})
        row.expected_hash = char(string(expected.(files{k,1})));
    elseif strcmp(files{k,1}, 'urdf_sha256') && isfield(expected,'urdf_version')
        row.expected_hash = char(extractAfter(string(expected.urdf_version), 'URDF-'));
    end
    row.match = row.exists && strcmpi(row.current_hash, row.expected_hash);
    allMatch = allMatch && row.match;
    rows(end+1,1) = row; %#ok<AGROW>
end
wireValue = NaN;
if isfield(scene,'wireCollisionShortenMm'), wireValue = double(scene.wireCollisionShortenMm); end
optValue = NaN;
if isfield(result,'options') && isfield(result.options,'wire_collision_shorten_mm')
    optValue = double(result.options.wire_collision_shorten_mm);
end
shortenMatch = isfinite(wireValue) && isfinite(optValue) && ...
    abs(wireValue-3) < 1e-9 && abs(optValue-3) < 1e-9;
snapshot = struct('match',logical(allMatch && shortenMatch), ...
    'calibration_binding_match',logical(allMatch), ...
    'wire_collision_shorten_mm_match',logical(shortenMatch), ...
    'saved_wire_collision_shorten_mm',wireValue, ...
    'saved_option_wire_collision_shorten_mm',optValue, ...
    'files',{rows},'expected_binding',expected, ...
    'status',ternary(allMatch && shortenMatch,'CURRENT','STALE'), ...
    'execution_authorized',false);
end

function c = filterCandidates(candidates, reason)
if isempty(candidates), c = candidates; return; end
keep = false(numel(candidates),1);
for k = 1:numel(candidates)
    if isfield(candidates(k),'parent_rejection_reason')
        keep(k) = strcmp(char(string(candidates(k).parent_rejection_reason)), reason);
    end
end
c = candidates(keep);
end

function c = uniqueCandidateRows(candidates)
if isempty(candidates), c = candidates; return; end
c = repmat(candidates(1), 0, 1);
for k = 1:numel(candidates)
    duplicate = false;
    for j = 1:numel(c)
        if max(abs(double(c(j).q_rad(:).') - double(candidates(k).q_rad(:).'))) < 1e-7
            duplicate = true; break;
        end
    end
    if ~duplicate, c(end+1,1) = candidates(k); end %#ok<AGROW>
end
end

function audit = auditDenseEdge(scene, qA, qB, nSamples, label)
audit = emptyEdgeAudit();
audit.label = label;
audit.q_start_rad = qA;
audit.q_end_rad = qB;
audit.q_start_deg = rad2deg(qA);
audit.q_end_deg = rad2deg(qB);
audit.sample_count = nSamples;
audit.delta_deg = rad2deg(wrapRad(qB-qA));
samples = repmat(emptyEdgeSample(), 0, 1);
first = 0;
lastSafe = 0;
for i = 1:nSamples
    alpha = (i-1)/(nSamples-1);
    q = qA + alpha*wrapRad(qB-qA);
    s = checkPose(scene, q);
    s.sample_index = i;
    s.interpolation_fraction = alpha;
    s.q_rad = q;
    s.q_deg = rad2deg(q);
    try
        s.tcp_pose = getTransform(scene.robot, q, scene.tcpBodyName);
    catch
        s.tcp_pose = eye(4);
    end
    s.is_safe = ~(s.environment_collision || s.self_collision);
    samples(end+1,1) = s; %#ok<AGROW>
    if s.is_safe
        lastSafe = i;
    elseif first == 0
        first = i;
    end
end
audit.samples = samples;
audit.first_collision_sample_index = first;
if first > 0
    audit.first_collision_fraction = samples(first).interpolation_fraction;
    audit.first_collision_q_rad = samples(first).q_rad;
    audit.first_collision_q_deg = samples(first).q_deg;
    audit.first_collision_tcp_pose = samples(first).tcp_pose;
    audit.first_collision_pairs = samples(first).collision_pairs;
else
    audit.first_collision_fraction = NaN;
    audit.first_collision_q_rad = zeros(1,6);
    audit.first_collision_q_deg = zeros(1,6);
    audit.first_collision_tcp_pose = eye(4);
    audit.first_collision_pairs = {};
end
if lastSafe > 0
    audit.last_safe_sample_index = lastSafe;
    audit.last_safe_fraction = samples(lastSafe).interpolation_fraction;
    audit.last_safe_q_rad = samples(lastSafe).q_rad;
    audit.last_safe_q_deg = samples(lastSafe).q_deg;
    audit.last_safe_tcp_pose = samples(lastSafe).tcp_pose;
else
    audit.last_safe_sample_index = 0;
    audit.last_safe_fraction = NaN;
    audit.last_safe_q_rad = zeros(1,6);
    audit.last_safe_q_deg = zeros(1,6);
    audit.last_safe_tcp_pose = eye(4);
end
audit.success = first == 0;
pairList = {};
for i = 1:numel(samples)
    pairList = [pairList; samples(i).collision_pairs(:)]; %#ok<AGROW>
end
audit.all_collision_pairs = unique(pairList);
audit.collision_sample_count = sum(~[samples.is_safe]);
audit.first_collision_objects = audit.first_collision_pairs;
end

function attempt = tryCartesianConnection(scene, prevState, endpoint, ...
    allEndpoints, fractions, o, level, branchIndex)
t0 = tic;
attempt = emptyRecoveryAttempt();
attempt.level = level;
attempt.branch_index = branchIndex;
attempt.fractions = fractions;
attempt.endpoint_q_rad = double(endpoint.q_rad(:).');
attempt.endpoint_q_deg = rad2deg(attempt.endpoint_q_rad);
attempt.endpoint_original_parent_reason = ...
    char(string(endpoint.parent_rejection_reason));
attempt.edge_sample_count = 101;

T0 = prevState.Tactual;
T1 = double(endpoint.tcp_pose);
allQ = zeros(0,6);
for k = 1:numel(allEndpoints)
    allQ(end+1,:) = double(allEndpoints(k).q_rad(:).'); %#ok<AGROW>
end
allQ(end+1,:) = attempt.endpoint_q_rad;
allQ(end+1,:) = double(prevState.q(:).');
allQ = uniqueRows(allQ);

co = candidateOptions(o, prevState.q);
layers = cell(1, numel(fractions)+1);
layers{1} = struct('q',double(prevState.q(:).'),'Tactual',T0, ...
    'Ttarget',T0,'fraction',0,'cost',0,'parent',0);
stageStats = repmat(emptyStageStat(),0,1);
failure = '';
for i = 1:numel(fractions)
    alpha = fractions(i);
    Ttarget = interpolatePose(T0, T1, alpha);
    travel = interpolateAngle(prevState.travelDeg, endpoint.travel_deg, alpha);
    roll = interpolateAngle(prevState.rollDeg, endpoint.roll_deg, alpha);
    previous = layers{i};
    seeds = zeros(0,6);
    for k = 1:numel(previous)
        seeds(end+1,:) = previous(k).q; %#ok<AGROW>
    end
    seeds = [seeds; allQ];
    seeds = uniqueRows(seeds);
    rr = generate_actual_axis_ik_candidates(scene, Ttarget, travel, ...
        roll, seeds, co);
    attempt.ik_calls = attempt.ik_calls + rr.numIKCalls;
    attempt.node_candidates = attempt.node_candidates + numel(rr.states);
    stat = emptyStageStat();
    stat.fraction = alpha;
    stat.ik_calls = rr.numIKCalls;
    stat.ik_numeric_success = rr.numIKNumericSuccess;
    stat.ik_pose_success = rr.numIKPoseOK;
    stat.node_candidates = numel(rr.states);
    stat.direction_rejected = rr.numDirectionRejected;
    stat.workpiece_collision_rejected = rr.numEnvironmentRejected;
    stat.self_collision_rejected = rr.numSelfRejected;
    stat.generated_states = numel(rr.states);
    next = repmat(layers{1}(1),0,1);
    nodeHadStep = false;
    nodeHadEdge = false;
    for k = 1:numel(rr.states)
        st = rr.states(k);
        for j = 1:numel(previous)
            dq = wrapRad(st.q - previous(j).q);
            if max(abs(rad2deg(dq))) > o.maxJointStepDeg + 1e-9
                nodeHadStep = true;
                continue;
            end
            nodeEdge = auditDenseEdge(scene, previous(j).q, st.q, 101, ...
                sprintf('level%d_fraction%.6f_parent%d',level,alpha,j));
            attempt.edge_checks = attempt.edge_checks + 1;
            if ~nodeEdge.success
                nodeHadEdge = true;
                continue;
            end
            candidate = struct('q',st.q,'Tactual',st.Tactual, ...
                'Ttarget',Ttarget,'fraction',alpha, ...
                'cost',previous(j).cost + sum(dq.^2),'parent',j);
            next(end+1,1) = candidate; %#ok<AGROW>
        end
    end
    stat.joint_step_rejections = double(nodeHadStep);
    stat.edge_collision_rejections = double(nodeHadEdge);
    if isempty(next)
        if isempty(rr.states)
            failure = 'NO_CONSTRAINED_NODE_CANDIDATE';
        elseif nodeHadEdge
            failure = 'NODE_CANDIDATES_BUT_EDGE_COLLISION';
        elseif nodeHadStep
            failure = 'IK_CONFIGURATION_BRANCH_DISCONTINUITY';
        else
            failure = 'NO_CONNECTED_CANDIDATE';
        end
        stat.failure_reason = failure;
        stageStats(end+1,1) = stat; %#ok<AGROW>
        break;
    end
    next = uniqueNodeRows(next);
    [~,ord] = sort([next.cost], 'ascend');
    keepN = min(24, numel(ord)); % diagnostic beam only
    layers{i+1} = next(ord(1:keepN));
    stat.connected_nodes = numel(layers{i+1});
    stageStats(end+1,1) = stat; %#ok<AGROW>
end

if isempty(failure)
    previous = layers{end};
    endNodes = repmat(layers{1}(1),0,1);
    endpointStep = false;
    endpointEdge = false;
    for j = 1:numel(previous)
        dq = wrapRad(attempt.endpoint_q_rad - previous(j).q);
        if max(abs(rad2deg(dq))) > o.maxJointStepDeg + 1e-9
            endpointStep = true;
            continue;
        end
        edge = auditDenseEdge(scene, previous(j).q, ...
            attempt.endpoint_q_rad, 101, ...
            sprintf('level%d_final_parent%d',level,j));
        attempt.edge_checks = attempt.edge_checks + 1;
        if ~edge.success
            endpointEdge = true;
            continue;
        end
        n = struct('q',attempt.endpoint_q_rad,'Tactual',T1, ...
            'Ttarget',T1,'fraction',1, ...
            'cost',previous(j).cost + sum(dq.^2),'parent',j);
        endNodes(end+1,1) = n; %#ok<AGROW>
    end
    if isempty(endNodes)
        if endpointEdge
            failure = 'NODE_CANDIDATE_BUT_FINAL_EDGE_COLLISION';
        elseif endpointStep
            failure = 'FINAL_IK_CONFIGURATION_BRANCH_DISCONTINUITY';
        else
            failure = 'NO_CONNECTED_CANDIDATE';
        end
    else
        [~,ix] = min([endNodes.cost]);
        selected = traceSelectedPath(layers, endNodes(ix), T1);
        attempt.success = true;
        attempt.status = 'DIAGNOSTIC_PASS';
        attempt.failure_reason = '';
        attempt.selected_path = selected;
        attempt.added_nodes = numel(selected)-2;
        attempt.max_joint_delta_deg = maxPathJointDelta(selected);
        attempt.max_tcp_error_mm = maxPathTcpError(selected);
        attempt.selected_node_checks = selectedNodeChecks(scene, selected);
        attempt.selected_edge_audits = repmat(emptyEdgeAudit(),0,1);
        for ie = 2:numel(selected)
            attempt.selected_edge_audits(end+1,1) = auditDenseEdge( ...
                scene, selected(ie-1).q, selected(ie).q, 101, ...
                sprintf('selected_path_edge_%d',ie-1)); %#ok<AGROW>
        end
        attempt.min_collision_gap_mm = minPathGapFromAudits( ...
            attempt.selected_edge_audits);
    end
end
if ~attempt.success
    attempt.status = failure;
    attempt.failure_reason = failure;
end
attempt.stage_statistics = stageStats;
attempt.runtime_sec = toc(t0);
end

function co = candidateOptions(o, qRef)
co = struct('positionToleranceM',o.positionToleranceM, ...
    'orientationToleranceDeg',o.orientationToleranceDeg, ...
    'sideToleranceDeg',o.sideToleranceDeg,'sideMarginM',o.sideMarginM, ...
    'checkSelfCollision',true,'lazyCollision',false, ...
    'reuseCandidateCache',false,'diagnosticCollectIKCandidates',true, ...
    'diagnosticReferenceQ',qRef,'diagnosticMaxJointDeltaDeg',inf, ...
    'disableAxisConstraint',false,'disable_axis_constraint',false, ...
    'verbose',false);
if isfield(o,'axis_angle_tolerance_deg') && ~isempty(o.axis_angle_tolerance_deg)
    co.axis_angle_tolerance_deg = o.axis_angle_tolerance_deg;
end
end

function path = traceSelectedPath(layers, endpointNode, T1)
% Follow diagnostic parent indices. This path is never written officially.
path = repmat(endpointNode,0,1);
path(end+1,1) = endpointNode; %#ok<AGROW>
node = endpointNode;
for layer = numel(layers):-1:2
    parentIndex = node.parent;
    node = layers{layer}(parentIndex);
    path(end+1,1) = node; %#ok<AGROW>
end
path(end+1,1) = layers{1}(1); %#ok<AGROW>
path = path(end:-1:1);
if ~isempty(path), path(end).Ttarget = T1; end
end

function rows = uniqueRows(rows)
if isempty(rows), return; end
keep = true(size(rows,1),1);
for i = 1:size(rows,1)
    if ~keep(i), continue; end
    for j = i+1:size(rows,1)
        if keep(j) && max(abs(wrapRad(rows(i,:)-rows(j,:)))) < 1e-7
            keep(j) = false;
        end
    end
end
rows = rows(keep,:);
end

function nodes = uniqueNodeRows(nodes)
if isempty(nodes), return; end
keep = true(numel(nodes),1);
for i = 1:numel(nodes)
    if ~keep(i), continue; end
    for j = i+1:numel(nodes)
        if keep(j) && max(abs(wrapRad(nodes(i).q-nodes(j).q))) < deg2rad(0.05)
            keep(j) = false;
        end
    end
end
nodes = nodes(keep);
end

function a = checkPose(scene, q)
% MATLAB reports NaN distance for collision and Inf for a pair with no
% finite distance; signed penetration depth is unavailable from this API.
names = scene.robot.BodyNames;
envInfo = scene.workpieceProxyInfo;
envNames = cell(1,numel(envInfo));
for k = 1:numel(envInfo)
    envNames{k} = char(string(envInfo(k).name));
end
try
    [envMask, envDist] = checkCollision(scene.robot,q,scene.environment, ...
        'IgnoreSelfCollision','on','Exhaustive','on');
catch
    [envMask, envDist] = checkCollision(scene.robot,q,scene.environment, ...
        'IgnoreSelfCollision','on');
end
try
    [selfMask, selfDist] = checkCollision(scene.robot,q, ...
        'IgnoreSelfCollision','off','SkippedSelfCollisions', ...
        scene.selfCollisionSkipPairs,'Exhaustive','on');
catch
    [selfMask, selfDist] = checkCollision(scene.robot,q, ...
        'IgnoreSelfCollision','off','SkippedSelfCollisions', ...
        scene.selfCollisionSkipPairs);
end
labels = {};
colliding = {};
finiteDistances = [];
% checkCollision returns one aggregate boolean plus a pairwise distance
% matrix.  A NaN in the distance matrix identifies the colliding pair;
% treating the aggregate boolean as a pair mask would label every near pair
% as colliding.
envCollision = any(isnan(envDist(:)));
selfCollision = any(isnan(selfDist(:)));
for i = 1:min(numel(names),size(envDist,1))
    for j = 1:min(numel(envNames),size(envDist,2))
        d = envDist(i,j);
        if ~isinf(d)
            label = sprintf('%s <-> %s', char(names{i}), envNames{j});
            labels{end+1,1} = label; %#ok<AGROW>
            if isfinite(d), finiteDistances(end+1) = d; end %#ok<AGROW>
            if isnan(d)
                colliding{end+1,1} = label; %#ok<AGROW>
            end
        end
    end
end
for i = 1:min(numel(names),size(selfDist,1))
    for j = i+1:min(numel(names),size(selfDist,2))
        if isSkipped(scene.selfCollisionSkipPairs, names{i}, names{j}), continue; end
        d = selfDist(i,j);
        if ~isinf(d)
            label = sprintf('%s <-> %s', char(names{i}), char(names{j}));
            labels{end+1,1} = label; %#ok<AGROW>
            if isfinite(d), finiteDistances(end+1) = d; end %#ok<AGROW>
            if isnan(d)
                colliding{end+1,1} = label; %#ok<AGROW>
            end
        end
    end
end
if isempty(finiteDistances)
    nearest = NaN;
    positive = NaN;
else
    nearest = min(finiteDistances)*1000;
    q = finiteDistances(finiteDistances > 0);
    if isempty(q), positive = NaN; else, positive = min(q)*1000; end
end
a = struct('environment_collision',envCollision, ...
    'self_collision',selfCollision,'collision_pairs',{colliding}, ...
    'all_pairs',{labels},'nearest_distance_mm',nearest, ...
    'min_positive_clearance_mm',positive,'penetration_depth_mm',NaN, ...
    'penetration_depth_status','UNAVAILABLE_CHECKCOLLISION_API');
end

function tf = isSkipped(pairs,a,b)
tf = false;
for k = 1:size(pairs,1)
    pair = string(pairs(k,:));
    if isequal(sort(pair), sort(string({char(a),char(b)})))
        tf = true; return;
    end
end
end

function T = interpolatePose(T0,T1,alpha)
p = (1-alpha)*T0(1:3,4) + alpha*T1(1:3,4);
q0 = rotmToQuat(T0(1:3,1:3));
q1 = rotmToQuat(T1(1:3,1:3));
if dot(q0,q1) < 0, q1 = -q1; end
dotq = max(-1,min(1,dot(q0,q1)));
if dotq > 0.9995
    q = (1-alpha)*q0 + alpha*q1;
    q = q/norm(q);
else
    th = acos(dotq);
    q = sin((1-alpha)*th)/sin(th)*q0 + ...
        sin(alpha*th)/sin(th)*q1;
    q = q/norm(q);
end
T = eye(4);
T(1:3,1:3) = quatToRotm(q);
T(1:3,4) = p;
end

function q = rotmToQuat(R)
tr = trace(R);
if tr > 0
    s = sqrt(tr+1)*2;
    w = 0.25*s; x=(R(3,2)-R(2,3))/s;
    y=(R(1,3)-R(3,1))/s; z=(R(2,1)-R(1,2))/s;
elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
    s = sqrt(1+R(1,1)-R(2,2)-R(3,3))*2;
    w=(R(3,2)-R(2,3))/s; x=0.25*s;
    y=(R(1,2)+R(2,1))/s; z=(R(1,3)+R(3,1))/s;
elseif R(2,2) > R(3,3)
    s = sqrt(1+R(2,2)-R(1,1)-R(3,3))*2;
    w=(R(1,3)-R(3,1))/s; x=(R(1,2)+R(3,1))/s;
    y=0.25*s; z=(R(2,3)+R(3,2))/s;
else
    s = sqrt(1+R(3,3)-R(1,1)-R(2,2))*2;
    w=(R(2,1)-R(1,2))/s; x=(R(1,3)+R(3,1))/s;
    y=(R(2,3)+R(3,2))/s; z=0.25*s;
end
q = [w x y z];
q = q/norm(q);
end

function R = quatToRotm(q)
q = q/norm(q);
w=q(1); x=q(2); y=q(3); z=q(4);
R = [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w); ...
     2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w); ...
     2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)];
end

function x = interpolateAngle(a,b,t)
x = a + t*atan2d(sind(b-a),cosd(b-a));
end

function x = wrapRad(x)
x = atan2(sin(x),cos(x));
end

function x = sha256File(file)
md = java.security.MessageDigest.getInstance('SHA-256');
fid = fopen(file,'r');
if fid < 0, x = ''; return; end
c = onCleanup(@() fclose(fid)); %#ok<NASGU>
bytes = fread(fid,Inf,'*uint8');
md.update(bytes);
x = lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end

function writeJson(file,data)
fid = fopen(file,'w');
if fid < 0, error('Unable to write %s',file); end
c = onCleanup(@() fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(data),'char');
end

function value = ternary(condition,a,b)
if condition, value = a; else, value = b; end
end

function s = emptyCandidate()
s = struct('rank',NaN,'q_rad',zeros(1,6),'q_deg',zeros(1,6), ...
    'tcp_pose',eye(4),'parent_rejection_reason','');
end

function s = emptyEdgeAudit()
s = struct('label','','q_start_rad',zeros(1,6),'q_end_rad',zeros(1,6), ...
    'q_start_deg',zeros(1,6),'q_end_deg',zeros(1,6), ...
    'delta_deg',zeros(1,6),'sample_count',0, ...
    'first_collision_sample_index',0,'first_collision_fraction',NaN, ...
    'first_collision_q_rad',zeros(1,6),'first_collision_q_deg',zeros(1,6), ...
    'first_collision_tcp_pose',eye(4),'first_collision_pairs',{{}}, ...
    'last_safe_sample_index',0,'last_safe_fraction',NaN, ...
    'last_safe_q_rad',zeros(1,6),'last_safe_q_deg',zeros(1,6), ...
    'last_safe_tcp_pose',eye(4),'all_collision_pairs',{{}}, ...
    'collision_sample_count',0,'first_collision_objects',{{}}, ...
    'success',false,'samples',repmat(emptyEdgeSample(),0,1));
end

function s = emptyEdgeSample()
s = struct('sample_index',0,'interpolation_fraction',NaN, ...
    'q_rad',zeros(1,6),'q_deg',zeros(1,6),'tcp_pose',eye(4), ...
    'is_safe',false,'environment_collision',false,'self_collision',false, ...
    'collision_pairs',{{}},'all_pairs',{{}}, ...
    'nearest_distance_mm',NaN,'min_positive_clearance_mm',NaN, ...
    'penetration_depth_mm',NaN, ...
    'penetration_depth_status','UNAVAILABLE_CHECKCOLLISION_API');
end

function s = emptyStepAudit()
s = struct('rank',NaN,'candidate_id','','q19_deg',zeros(1,6), ...
    'q20_deg',zeros(1,6),'delta_deg',zeros(1,6), ...
    'absolute_delta_deg',zeros(1,6),'max_joint_delta_deg',NaN, ...
    'max_joint_step_limit_deg',NaN,'direct_joint_step_pass',false, ...
    'continuous_branch_status','');
end

function s = emptyRecoveryAttempt()
s = struct('level',NaN,'branch_index',NaN,'fractions',zeros(1,0), ...
    'endpoint_q_rad',zeros(1,6),'endpoint_q_deg',zeros(1,6), ...
    'endpoint_original_parent_reason','','edge_sample_count',101, ...
    'ik_calls',0,'node_candidates',0,'edge_checks',0,'added_nodes',0, ...
    'stage_statistics',repmat(emptyStageStat(),0,1), ...
    'success',false,'status','','failure_reason', ... 
    '','max_joint_delta_deg',NaN,'max_tcp_error_mm',NaN, ...
    'min_collision_gap_mm',NaN,'runtime_sec',NaN, ...
    'selected_path',repmat(struct('q',zeros(1,6), ...
    'Tactual',eye(4),'Ttarget',eye(4),'fraction',NaN,'cost',NaN, ...
    'parent',0),0,1), ...
    'selected_node_checks',repmat(emptyNodeCheck(),0,1), ...
    'selected_edge_audits',repmat(emptyEdgeAudit(),0,1));
end

function s = emptyNodeCheck()
s = struct('fraction',NaN,'q_rad',zeros(1,6),'q_deg',zeros(1,6), ...
    'environment_collision',false,'self_collision',false, ...
    'collision_pairs',{{}},'nearest_distance_mm',NaN, ...
    'min_positive_clearance_mm',NaN,'is_safe',false);
end

function s = emptyStageStat()
s = struct('fraction',NaN,'ik_calls',0,'ik_numeric_success',0, ...
    'ik_pose_success',0,'node_candidates',0,'generated_states',0, ...
    'direction_rejected',0,'workpiece_collision_rejected',0, ...
    'self_collision_rejected',0,'joint_step_rejections',0, ...
    'edge_collision_rejections',0,'connected_nodes',0, ...
    'failure_reason','');
end

function v = maxPathJointDelta(path)
v = 0;
for k = 2:numel(path)
    v = max(v, max(abs(rad2deg(wrapRad(path(k).q-path(k-1).q)))));
end
end

function v = maxPathTcpError(path)
v = 0;
for k = 1:numel(path)
    v = max(v, norm(path(k).Tactual(1:3,4)- ...
        path(k).Ttarget(1:3,4))*1000);
end
end

function v = minPathGap(scene, path)
v = Inf;
for k = 2:numel(path)
    e = auditDenseEdge(scene,path(k-1).q,path(k).q,101,'selected_path');
    for j = 1:numel(e.samples)
        g = e.samples(j).min_positive_clearance_mm;
        if isfinite(g), v = min(v,g); end
    end
end
if isinf(v), v = NaN; end
end

function checks = selectedNodeChecks(scene, path)
checks = repmat(emptyNodeCheck(),0,1);
for k = 1:numel(path)
    a = checkPose(scene,path(k).q);
    c = emptyNodeCheck();
    c.fraction = path(k).fraction;
    c.q_rad = path(k).q;
    c.q_deg = rad2deg(path(k).q);
    c.environment_collision = a.environment_collision;
    c.self_collision = a.self_collision;
    c.collision_pairs = a.collision_pairs;
    c.nearest_distance_mm = a.nearest_distance_mm;
    c.min_positive_clearance_mm = a.min_positive_clearance_mm;
    c.is_safe = ~(c.environment_collision || c.self_collision);
    checks(end+1,1) = c; %#ok<AGROW>
end
end

function v = minPathGapFromAudits(audits)
v = Inf;
for k = 1:numel(audits)
    for j = 1:numel(audits(k).samples)
        g = audits(k).samples(j).min_positive_clearance_mm;
        if isfinite(g), v = min(v,g); end
    end
end
if isinf(v), v = NaN; end
end

