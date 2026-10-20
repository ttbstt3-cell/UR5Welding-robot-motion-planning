function [T_flange_adjustment, record] = load_real_gun_mount_calibration(path)
%LOAD_REAL_GUN_MOUNT_CALIBRATION Read the independent HMI mount adjustment.
%
% This loader is intentionally side-effect free.  It validates the
% flange-local record and returns a homogeneous transform for a real-scene
% planner to apply explicitly.  It does not modify scene_geometry.json,
% ur5_tool_calibration.mat, or any frozen simulation baseline.

if nargin < 1 || isempty(path)
    path = fullfile(fileparts(mfilename('fullpath')), '..', 'real_gun_mount_calibration.json');
end
if ~isfile(path)
    error('load_real_gun_mount_calibration:MissingFile', 'Calibration file not found: %s', path);
end

record = jsondecode(fileread(path));
if ~isfield(record, 'coordinate_frame') || ~strcmpi(string(record.coordinate_frame), 'URDF flange')
    error('load_real_gun_mount_calibration:Frame', 'Expected coordinate_frame = URDF flange.');
end
if ~isfield(record, 'rotation_axis_flange') || ~isfield(record, 'roll_deg') || ~isfield(record, 'translation_flange_mm')
    error('load_real_gun_mount_calibration:Schema', 'Missing flange axis, roll_deg, or translation_flange_mm.');
end

axis = double(record.rotation_axis_flange(:));
translation_mm = double(record.translation_flange_mm(:));
roll_deg = double(record.roll_deg);
if numel(axis) ~= 3 || numel(translation_mm) ~= 3 || any(~isfinite(axis)) || ...
        any(~isfinite(translation_mm)) || ~isfinite(roll_deg) || roll_deg < -180 || roll_deg > 180
    error('load_real_gun_mount_calibration:Values', 'Invalid finite flange-local adjustment values.');
end
axis_norm = norm(axis);
if axis_norm < 1e-12
    error('load_real_gun_mount_calibration:Axis', 'Rotation axis is degenerate.');
end
axis = axis / axis_norm;

% Rodrigues right-hand rotation about the local flange axis.
theta = deg2rad(roll_deg);
K = [0 -axis(3) axis(2); axis(3) 0 -axis(1); -axis(2) axis(1) 0];
R = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K * K);
T_flange_adjustment = eye(4);
T_flange_adjustment(1:3, 1:3) = R;
T_flange_adjustment(1:3, 4) = translation_mm * 1e-3;

record.rotation_axis_flange_normalized = axis.';
record.source_file = path;
record.loaded_for_planning = false;
end
