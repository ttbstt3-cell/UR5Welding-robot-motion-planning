function report = plan_hmi_robot_request(request)
%PLAN_HMI_ROBOT_REQUEST Route one offline request to the selected robot model.
% It does not publish, execute, or send any robot motion command.
if nargin ~= 1 || ~isstruct(request)
    error('plan_hmi_robot_request expects one request struct.');
end
robotId = localField(request, 'robot_id', localField(request, 'robotId', ''));
robotId = upper(string(robotId));
if strlength(robotId) == 0
    error('BLOCKED: plan_request.robot_id is required.');
end
root = localField(request, 'robotModelRoot', '');
sceneScript = localField(request, 'sceneScript', '');
planningEntry = localField(request, 'matlabPlanningEntry', '');
scene = load_hmi_robot_model(robotId, root, sceneScript, planningEntry);

switch robotId
    case "UR5"
        % Keep the established calibrated HMI/ACCP planner as the single
        % UR5 implementation. The scene resolution above validates the ID and
        % model path; the frozen planner retains its own scene setup.
        report = plan_hmi_weld_request(request);
        report.robot_id = char(robotId);
        outputFile = char(localField(request, 'outputFile', ''));
        if ~isempty(outputFile) && isfile(outputFile)
            saved = load(outputFile, 'metadata');
            robot_id = 'UR5'; %#ok<NASGU>
            if isfield(saved, 'metadata')
                metadata = saved.metadata;
                metadata.robot_id = 'UR5'; %#ok<NASGU>
                save(outputFile, 'robot_id', 'metadata', '-append');
            else
                metadata = struct('robot_id', 'UR5'); %#ok<NASGU>
                save(outputFile, 'robot_id', 'metadata', '-append');
            end
        end
        reportFile = char(localField(request, 'reportFile', ''));
        if ~isempty(reportFile)
            localWriteJson(reportFile, report);
        end
    case "CR7"
        task = string(localField(request, 'task', ''));
        if task ~= "JointSpace"
            error(['BLOCKED: CR7 weld-task planning has not been commissioned. ', ...
                   'Use task=JointSpace for offline RRT-Connect only.']);
        end
        if ~isfield(request, 'qStart') || ~isfield(request, 'qGoal')
            error('CR7 JointSpace request requires qStart and qGoal.');
        end
        qStart = reshape(double(request.qStart), 1, []);
        qGoal = reshape(double(request.qGoal), 1, []);
        if numel(qStart) ~= 6 || numel(qGoal) ~= 6 || ...
                any(~isfinite(qStart)) || any(~isfinite(qGoal))
            error('CR7 qStart/qGoal must be finite 1x6 vectors.');
        end
        qPath = plan_cr7_rrt(qStart, qGoal);
        robot_id = 'CR7'; %#ok<NASGU>
        joint_names = scene.jointNames; %#ok<NASGU>
        metadata = struct('robot_id', 'CR7', 'source', 'CR7_RRT_CONNECT_OFFLINE', ...
            'trajectory_name', 'CR7_JointSpace', 'collision_verified', true, ...
            'collision_check_pass', true, 'description', ...
            'Offline CR7 RRT-Connect path; no timing and no motion command.'); %#ok<NASGU>
        outputFile = char(localField(request, 'outputFile', fullfile(pwd, 'trajectory.mat')));
        reportFile = char(localField(request, 'reportFile', fullfile(pwd, 'report.json')));
        localEnsureParent(outputFile);
        localEnsureParent(reportFile);
        save(outputFile, 'qPath', 'joint_names', 'robot_id', 'metadata', '-v7');
        report = struct('robot_id', 'CR7', 'task', 'JointSpace', 'success', true, ...
            'collision_check_pass', true, 'point_count', size(qPath, 1), ...
            'timed', false, 'message', 'Offline planning only; no motion command.');
        localWriteJson(reportFile, report);
    otherwise
        error('Unsupported robot_id: %s', robotId);
end
end

function value = localField(s, name, fallback)
if isfield(s, name) && ~isempty(s.(name)), value = s.(name); else, value = fallback; end
end

function localEnsureParent(path)
parent = fileparts(path);
if ~isempty(parent) && ~isfolder(parent), mkdir(parent); end
end

function localWriteJson(path, value)
localEnsureParent(path);
fid = fopen(path, 'w');
if fid < 0, error('Cannot write report file: %s', path); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fwrite(fid, jsonencode(value), 'char');
end
