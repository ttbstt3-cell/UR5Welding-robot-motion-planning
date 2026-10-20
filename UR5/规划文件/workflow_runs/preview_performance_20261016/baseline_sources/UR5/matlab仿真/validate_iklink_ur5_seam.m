function report = validate_iklink_ur5_seam(resultFile, motionFile, reportFile)
% Validate IKLink's UR5 motion with the same MATLAB tool/environment model.
% This is an adapter experiment; it never changes the official IKLink source
% or the frozen A-ARC-IK/Gao results.

if nargin < 1 || isempty(resultFile)
    resultFile = fullfile(fileparts(mfilename('fullpath')), ...
        'task_level_arc_vs_gao_runs', 'Seam_A_arc_001_forward.mat');
end
if nargin < 2 || isempty(motionFile)
    motionFile = fullfile(fileparts(mfilename('fullpath')), ...
        '..', 'third_party', 'IKLink', 'output_motions', ...
        'ur5_seamA_arc001.csv');
end
if nargin < 3 || isempty(reportFile)
    reportFile = fullfile(fileparts(mfilename('fullpath')), ...
        'iklink_ur5_seamA_validation.txt');
end

S = load(resultFile, 'result');
R = S.result;
M = readmatrix(motionFile, 'NumHeaderLines', 1);
if size(M,2) < 7, error('IKLink motion CSV 列数不足。'); end
qPath = M(:,2:7);
N = size(qPath,1);

% Build the already validated MATLAB scene without displaying its figure.
oldVisible = get(0,'DefaultFigureVisible');
set(0,'DefaultFigureVisible','off');
cleanup = onCleanup(@() set(0,'DefaultFigureVisible',oldVisible)); %#ok<NASGU>
% The scene script begins with clear, so preserve input paths on disk.
save('/tmp/iklink_validator_inputs.mat','resultFile','motionFile','reportFile');
run(fullfile(fileparts(mfilename('fullpath')), 'sync_ur5_joint_states.m'));
close all force;

% sync_ur5_joint_states.m starts with clear; reload the reference result
% after the scene has been built because that script clears caller variables.
load('/tmp/iklink_validator_inputs.mat','resultFile','motionFile','reportFile');
S = load(resultFile, 'result');
R = S.result;
M = readmatrix(motionFile, 'NumHeaderLines', 1);
qPath = M(:,2:7);
N = size(qPath,1);

T_tool0_tcp = R.scene.T_tool0_mount * R.scene.T_mount_tcp;
TtargetTool0 = zeros(4,4,N);
for i = 1:N
    if i > numel(R.stateBank), error('IKLink 点数超过参考焊缝点数。'); end
    st = localStateMatchingQ(R.stateBank{i}, R.qPath(i,:));
    TtargetTool0(:,:,i) = st.Ttarget / T_tool0_tcp;
end

nodeCollision = false(N,1);
positionErrorMm = zeros(N,1);
orientationErrorDeg = zeros(N,1);
for i = 1:N
    Tf = getTransform(scene.robot, qPath(i,:), 'tool0');
    dT = inv(TtargetTool0(:,:,i))*Tf;
    positionErrorMm(i) = 1000*norm(dT(1:3,4));
    axang = rotm2axang(dT(1:3,1:3));
    orientationErrorDeg(i) = rad2deg(abs(axang(4)));
    c = checkCollision(scene.robot, qPath(i,:), scene.environment, ...
        'IgnoreSelfCollision','on');
    nodeCollision(i) = any(c(:));
end

maxJointJumpDeg = 0;
edgeCollision = false(N-1,1);
edgeSamples = 0;
edgeStepDeg = 3;
for i = 1:N-1
    dq = wrapToPi(qPath(i+1,:) - qPath(i,:));
    maxJointJumpDeg = max(maxJointJumpDeg, rad2deg(max(abs(dq))));
    nInterp = max(1,ceil(rad2deg(max(abs(dq)))/edgeStepDeg));
    for k = 1:nInterp
        a = k/nInterp;
        q = qPath(i,:) + a*dq;
        c = checkCollision(scene.robot, q, scene.environment, ...
            'IgnoreSelfCollision','on');
        edgeCollision(i) = edgeCollision(i) || any(c(:));
        edgeSamples = edgeSamples + 1;
    end
end

report = struct();
report.resultFile = string(resultFile);
report.motionFile = string(motionFile);
report.numWaypoints = N;
report.nodeCollisionFree = ~any(nodeCollision);
report.edgeCollisionFree = ~any(edgeCollision);
report.tcpPositionPass = max(positionErrorMm) <= 0.5;
report.tcpOrientationPass = max(orientationErrorDeg) <= 0.5;
report.maxNodeTcpErrorMm = max(positionErrorMm);
report.maxNodeTcpOrientationErrorDeg = max(orientationErrorDeg);
report.maxJointJumpDeg = maxJointJumpDeg;
report.edgeSamples = edgeSamples;
report.nodeCollisionIndices = find(nodeCollision);
report.edgeCollisionIndices = find(edgeCollision);
report.complete = report.nodeCollisionFree && report.edgeCollisionFree && ...
    report.tcpPositionPass && report.tcpOrientationPass;

fid = fopen(reportFile,'w');
if fid >= 0
    fprintf(fid,'IKLink UR5 Seam A validation\n');
    fprintf(fid,'waypoints = %d\n',N);
    fprintf(fid,'node_collision_free = %d\n',report.nodeCollisionFree);
    fprintf(fid,'edge_collision_free = %d\n',report.edgeCollisionFree);
    fprintf(fid,'tcp_position_pass = %d\n',report.tcpPositionPass);
    fprintf(fid,'tcp_orientation_pass = %d\n',report.tcpOrientationPass);
    fprintf(fid,'max_node_tcp_error_mm = %.6f\n',report.maxNodeTcpErrorMm);
    fprintf(fid,'max_node_tcp_orientation_error_deg = %.6f\n',report.maxNodeTcpOrientationErrorDeg);
    fprintf(fid,'max_joint_jump_deg = %.6f\n',report.maxJointJumpDeg);
    fprintf(fid,'edge_samples = %d\n',report.edgeSamples);
    fprintf(fid,'node_collision_indices = %s\n',mat2str(report.nodeCollisionIndices));
    fprintf(fid,'edge_collision_indices = %s\n',mat2str(report.edgeCollisionIndices));
    fprintf(fid,'complete = %d\n',report.complete);
    fclose(fid);
end

fprintf('IKLink UR5 Seam A validation: complete=%d, node=%d, edge=%d, TCP=%.4f mm, jump=%.3f deg\n', ...
    report.complete,report.nodeCollisionFree,report.edgeCollisionFree, ...
    report.maxNodeTcpErrorMm,report.maxJointJumpDeg);
end

function st = localStateMatchingQ(layer,q)
best = inf; idx = 1;
for k = 1:numel(layer)
    d = max(abs(wrapToPi(layer(k).q-q)));
    if d < best, best=d; idx=k; end
end
st = layer(idx);
end
