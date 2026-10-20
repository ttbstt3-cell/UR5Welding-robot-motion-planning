function scene = load_hmi_robot_model(robotId, modelRoot, sceneScript, planningEntry)
%LOAD_HMI_ROBOT_MODEL Resolve the requested profile to its own planning scene.
% Read-only model construction only; this function never publishes motion.
if nargin < 1 || isempty(robotId), error('robot_id is required.'); end
robotId = upper(string(robotId));
if nargin >= 2 && ~isempty(modelRoot)
    addpath(genpath(char(modelRoot)));
end
if nargin >= 4 && ~isempty(planningEntry)
    entryDir = fileparts(char(planningEntry));
    addpath(genpath(fileparts(entryDir)));
end
switch robotId
    case "UR5"
        if nargin < 3 || isempty(sceneScript)
            sceneScript = fullfile(char(modelRoot), 'sync_ur5_joint_states.m');
        end
        scene = loadUR5JointScene(sceneScript);
    case "CR7"
        scene = load_cr7_scene();
    otherwise
        error('Unsupported robot_id: %s', robotId);
end
end
