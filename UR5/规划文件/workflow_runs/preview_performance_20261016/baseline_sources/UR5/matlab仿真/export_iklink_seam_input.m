function outFile = export_iklink_seam_input(resultFile, outFile)
% Export one selected A-ARC-IK seam as IKLink's pose-trajectory CSV.
% This is an independent adapter: it does not modify any planner result.
% IKLink's UR5 end-effector is tool0, while the MATLAB target is WeldTCP;
% therefore targets are converted with tool0->WeldTCP calibration.

if nargin < 1 || isempty(resultFile)
    resultFile = fullfile(fileparts(mfilename('fullpath')), ...
        'task_level_arc_vs_gao_runs', 'Seam_A_arc_001_forward.mat');
end
if nargin < 2 || isempty(outFile)
    outFile = fullfile(fileparts(mfilename('fullpath')), ...
        '..', 'third_party', 'IKLink', 'input_trajectories', ...
        'ur5_seamA_arc001.csv');
end

S = load(resultFile, 'result');
R = S.result;
if ~isfield(R, 'qPath') || ~R.complete
    error('输入结果必须包含完整 qPath。');
end

% Use the calibrated tool0->WeldTCP transform, not a guessed TCP offset.
T_tool0_tcp = R.scene.T_tool0_mount * R.scene.T_mount_tcp;
n = size(R.qPath, 1);
parentDir = fileparts(outFile);
if ~isfolder(parentDir), mkdir(parentDir); end

fid = fopen(outFile, 'w');
if fid < 0, error('无法写入 %s', outFile); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'time,target-POS_X,target-POS_Y,target-POS_Z,target-ROT_X,target-ROT_Y,target-ROT_Z,target-ROT_W\n');

for i = 1:n
    st = localStateMatchingQ(R.stateBank{i}, R.qPath(i,:));
    T_tool0 = st.Ttarget / T_tool0_tcp;
    quatWxyz = rotm2quat(T_tool0(1:3,1:3));
    % IKLink input order is qx,qy,qz,qw; its Rust loader reconstructs w first.
    fprintf(fid, '%.16g,%.16g,%.16g,%.16g,%.16g,%.16g,%.16g,%.16g\n', ...
        (i-1)/max(1,n-1), T_tool0(1,4), T_tool0(2,4), T_tool0(3,4), ...
        quatWxyz(2), quatWxyz(3), quatWxyz(4), quatWxyz(1));
end

fprintf('IKLink UR5 input written: %s (%d waypoints)\n', outFile, n);
end

function st = localStateMatchingQ(layer, q)
if isempty(layer), error('状态层为空。'); end
best = inf; idx = 1;
for k = 1:numel(layer)
    d = max(abs(wrapToPi(layer(k).q - q)));
    if d < best, best = d; idx = k; end
end
if best > 1e-6
    warning('未找到完全相同的 qPath 状态，使用最近状态，误差 %.3g rad。', best);
end
st = layer(idx);
end
