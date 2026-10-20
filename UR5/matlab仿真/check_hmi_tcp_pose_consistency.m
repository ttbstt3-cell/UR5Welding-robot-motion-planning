function report = check_hmi_tcp_pose_consistency(trajectoryFile, outputFile)
%CHECK_HMI_TCP_POSE_CONSISTENCY Read-only MATLAB/HMI pose audit.
%
% This helper evaluates the exact frozen MATLAB scene at the same qPath rows
% used by the HMI.  It does not write calibration files, alter the robot
% model, start ROS, or command a robot.  The Python companion compares this
% JSON with the HMI URDF FK and reports the node/interpolation errors.

if nargin < 1 || isempty(trajectoryFile)
    trajectoryFile = '/home/ttbstt/jixiebi/UR5/planning_runs/hmi_20261010_105205_168803/trajectory.mat';
end
if nargin < 2 || isempty(outputFile)
    outputFile = fullfile(fileparts(trajectoryFile), 'matlab_tcp_pose_consistency.json');
end

trajectoryFile = char(string(trajectoryFile));
outputFile = char(string(outputFile));
if ~isfile(trajectoryFile)
    error('Trajectory file not found: %s', trajectoryFile);
end

setenv('RRT_HEADLESS','1');
rrtRoot = '/home/ttbstt/jixiebi/RRT';
matlabDir = '/home/ttbstt/jixiebi/UR5/matlab仿真';
addpath(genpath(rrtRoot));
addpath(matlabDir);

scene = loadUR5JointScene(fullfile(matlabDir,'sync_ur5_joint_states.m'));
s = load(trajectoryFile, 'qPath', 'tcpPath');
if ~isfield(s,'qPath') || isempty(s.qPath)
    error('Trajectory has no qPath.');
end
qPath = double(s.qPath);
if size(qPath,2) ~= 6 || any(~isfinite(qPath(:)))
    error('qPath must be finite N-by-6.');
end

% Use exactly ten deterministic samples (all rows when N<10), while keeping
% the complete node arrays for the interpolation audit on the Python side.
n = size(qPath,1);
sampleIdx = unique(round(linspace(1,n,min(10,n))));

% MATLAB's stock universalUR5 tree ends at ``tool0`` and does not expose a
% separate ``flange`` body.  In that model tool0 is the flange-equivalent
% mounting frame; the HMI calibrated URDF has both frames and supplies the
% fixed flange->tool0 transform.  Keep this distinction explicit instead of
% silently applying the same transform twice.
T0tool0 = getTransform(scene.robot, zeros(1,6), 'tool0');
hasMatlabFlange = any(strcmp(scene.robot.BodyNames,'flange'));
if hasMatlabFlange
    T0flange = getTransform(scene.robot, zeros(1,6), 'flange');
    T_flange_tool0 = T0flange \ T0tool0;
    matlabFlangeFrame = 'flange';
else
    T0flange = T0tool0;
    T_flange_tool0 = eye(4);
    matlabFlangeFrame = 'tool0';
end
T_flange_tcp_nom = T_flange_tool0 * scene.T_tool0_mount * scene.T_mount_tcp;

% The legacy HMI calibration is position-only.  A manually confirmed active
% pendant record may additionally provide a complete flange->TCP transform;
% in that case use it without changing the frozen simulation scene.
activeFile = '/home/ttbstt/jixiebi/UR5/real_tcp_calibration_active.json';
legacyFile = '/home/ttbstt/jixiebi/UR5/real_tcp_calibration_applied.json';
realFile = legacyFile;
if isfile(activeFile)
    try
        activeRaw = jsondecode(fileread(activeFile));
        if isfield(activeRaw,'valid') && logical(activeRaw.valid) && ...
                isfield(activeRaw,'applied_to_planning_model') && logical(activeRaw.applied_to_planning_model)
            realFile = activeFile;
        end
    catch
        % Keep the legacy approved record if the active pointer is malformed.
    end
end
realOffset = [];
realLoaded = false;
realTransform = [];
if isfile(realFile)
    raw = jsondecode(fileread(realFile));
    if isfield(raw,'valid') && logical(raw.valid) && ...
            isfield(raw,'applied_to_planning_model') && logical(raw.applied_to_planning_model)
        realOffset = double(raw.position_flange_tcp_m(:));
        realLoaded = numel(realOffset)==3 && all(isfinite(realOffset));
        if isfield(raw,'T_flange_tcp')
            candidateT = double(raw.T_flange_tcp);
            if isequal(size(candidateT),[4 4]) && all(isfinite(candidateT(:)))
                realTransform = candidateT;
            end
        end
    end
end
T_flange_tcp_real = T_flange_tcp_nom;
if realLoaded
    if ~isempty(realTransform)
        T_flange_tcp_real = realTransform;
    else
        T_flange_tcp_real(1:3,4) = realOffset;
    end
end

flange = zeros(n,3); tool0 = zeros(n,3); mount = zeros(n,3);
tcpNom = zeros(n,3); tcpReal = zeros(n,3);
flangeT = zeros(4,4,n); tool0T = zeros(4,4,n); mountT = zeros(4,4,n);
tcpNomT = zeros(4,4,n); tcpRealT = zeros(4,4,n);
gunRelErr = zeros(n,1); wireRelErr = zeros(n,1);
for k = 1:n
    Tf = getTransform(scene.robot,qPath(k,:),matlabFlangeFrame);
    T0 = getTransform(scene.robot,qPath(k,:),'tool0');
    Tm = getTransform(scene.robot,qPath(k,:),scene.mountBodyName);
    Tn = getTransform(scene.robot,qPath(k,:),scene.tcpBodyName);
    flangeT(:,:,k) = Tf; tool0T(:,:,k) = T0; mountT(:,:,k) = Tm;
    tcpNomT(:,:,k) = Tn;
    flange(k,:) = Tf(1:3,4).';
    tool0(k,:) = T0(1:3,4).';
    mount(k,:) = Tm(1:3,4).';
    tcpNom(k,:) = Tn(1:3,4).';
    Treal = Tf*T_flange_tcp_real;
    tcpRealT(:,:,k) = Treal;
    tcpReal(k,:) = Treal(1:3,4).';
    % These are deliberately relative-to-flange checks.  Switching the TCP
    % marker is not allowed to move the gun mount or the wire attachment.
    gunRelErr(k) = norm((Tf\Tm) - T_flange_tool0*scene.T_tool0_mount, 'fro');
    wireRelErr(k) = norm((Tm\Tn) - scene.T_mount_tcp, 'fro');
end

sample = repmat(struct('index',0,'q_rad',[],'flange_xyz_m',[],'tool0_xyz_m',[], ...
    'welding_gun_mount_xyz_m',[],'tcp_nominal_xyz_m',[],'tcp_real_xyz_m',[], ...
    'flange_T',[],'tool0_T',[],'welding_gun_mount_T',[],'tcp_nominal_T',[],'tcp_real_T',[]), numel(sampleIdx),1);
for j = 1:numel(sampleIdx)
    k = sampleIdx(j);
    sample(j).index = k;
    sample(j).q_rad = qPath(k,:);
    sample(j).flange_xyz_m = flange(k,:);
    sample(j).tool0_xyz_m = tool0(k,:);
    sample(j).welding_gun_mount_xyz_m = mount(k,:);
    sample(j).tcp_nominal_xyz_m = tcpNom(k,:);
    sample(j).tcp_real_xyz_m = tcpReal(k,:);
    sample(j).flange_T = flangeT(:,:,k);
    sample(j).tool0_T = tool0T(:,:,k);
    sample(j).welding_gun_mount_T = mountT(:,:,k);
    sample(j).tcp_nominal_T = tcpNomT(:,:,k);
    sample(j).tcp_real_T = tcpRealT(:,:,k);
end

report = struct();
report.schema_version = 1;
report.trajectory_file = trajectoryFile;
report.point_count = n;
report.sample_indices = sampleIdx;
report.samples = sample;
report.q_path_rad = qPath;
report.matlab_flange_xyz_m = flange;
report.matlab_tool0_xyz_m = tool0;
report.matlab_welding_gun_mount_xyz_m = mount;
report.matlab_tcp_nominal_xyz_m = tcpNom;
report.matlab_tcp_real_xyz_m = tcpReal;
report.t_flange_tool0 = T_flange_tool0;
report.t_flange_tcp_nominal = T_flange_tcp_nom;
report.t_flange_tcp_real = T_flange_tcp_real;
report.real_tcp_calibration_file = realFile;
report.real_tcp_loaded = realLoaded;
report.matlab_flange_frame = matlabFlangeFrame;
report.gun_mount_relative_transform_max_fro = max(gunRelErr);
report.wire_relative_transform_max_fro = max(wireRelErr);
report.fixed_geometry_check_pass = max(gunRelErr) < 1e-10 && max(wireRelErr) < 1e-10;

parent = fileparts(outputFile);
if ~isempty(parent) && ~isfolder(parent), mkdir(parent); end
fid = fopen(outputFile,'w');
if fid < 0, error('Cannot write %s',outputFile); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(report),'char');
fprintf('MATLAB_TCP_CONSISTENCY_REPORT=%s\n',outputFile);
end
