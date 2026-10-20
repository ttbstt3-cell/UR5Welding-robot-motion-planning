function calib = load_real_tcp_calibration(jsonFile)
%LOAD_REAL_TCP_CALIBRATION Read the independently measured real TCP offset.
%
% The file is intentionally separate from ur5_tool_calibration.mat. It
% contains only a flange->TCP position measured by the HMI fixed-point
% calibration. Existing simulation planning code must opt in explicitly;
% this function never changes a robot model or a frozen calibration file.

if nargin < 1 || strlength(string(jsonFile)) == 0
    root = fullfile(fileparts(mfilename('fullpath')), '..');
    activeFile = fullfile(root, 'real_tcp_calibration_active.json');
    % The active file is a separate, manually confirmed version.  If it is
    % absent, retain the ordinary HMI calibration default; the frozen MAT
    % calibration is never modified by this loader.
    if isfile(activeFile)
        try
            activeRaw = jsondecode(fileread(activeFile));
            if isfield(activeRaw, 'valid') && logical(activeRaw.valid) && ...
                    isfield(activeRaw, 'applied_to_planning_model') && ...
                    logical(activeRaw.applied_to_planning_model)
                jsonFile = activeFile;
            else
                jsonFile = fullfile(root, 'real_tcp_calibration.json');
            end
        catch
            jsonFile = fullfile(root, 'real_tcp_calibration.json');
        end
    else
        jsonFile = fullfile(root, 'real_tcp_calibration.json');
    end
end
jsonFile = char(string(jsonFile));
if ~isfile(jsonFile)
    error('REAL_TCP_CALIBRATION:MissingFile', ...
        'Real TCP calibration file not found: %s', jsonFile);
end

raw = fileread(jsonFile);
data = jsondecode(raw);
required = {'position_flange_tcp_m','coordinate_frames','valid'};
for k = 1:numel(required)
    if ~isfield(data, required{k})
        error('REAL_TCP_CALIBRATION:Schema', ...
            'Missing field %s in %s', required{k}, jsonFile);
    end
end
if ~logical(data.valid)
    error('REAL_TCP_CALIBRATION:Invalid', ...
        'Calibration file is marked invalid: %s', jsonFile);
end

position = double(data.position_flange_tcp_m(:));
if numel(position) ~= 3 || any(~isfinite(position))
    error('REAL_TCP_CALIBRATION:Position', ...
        'position_flange_tcp_m must be a finite 3-vector.');
end
if ~isfield(data.coordinate_frames, 'flange') || ...
        ~contains(string(data.coordinate_frames.flange), 'flange', 'IgnoreCase', true)
    error('REAL_TCP_CALIBRATION:Frame', ...
        'Calibration is not declared in the URDF flange frame.');
end

T = eye(4);
% Pendant adoption records a complete flange->TCP transform.  Validate it
% before accepting it; legacy position-only records retain the frozen tool
% orientation by design.
if isfield(data, 'T_flange_tcp')
    candidateT = double(data.T_flange_tcp);
    if isequal(size(candidateT), [4 4]) && all(isfinite(candidateT(:))) && ...
            norm(candidateT(4,:) - [0 0 0 1]) < 1e-9 && ...
            norm(candidateT(1:3,1:3)' * candidateT(1:3,1:3) - eye(3), 'fro') < 1e-6 && ...
            abs(det(candidateT(1:3,1:3)) - 1) < 1e-6 && ...
            norm(candidateT(1:3,4) - position) < 1e-9
        T = candidateT;
    else
        error('REAL_TCP_CALIBRATION:Transform', ...
            'T_flange_tcp is present but is not a valid rigid transform.');
    end
else
    T(1:3,4) = position;
end
calib = data;
calib.T_flange_tcp = T;
calib.position = position;
calib.sourceFile = jsonFile;
calib.orientationApplied = isfield(data, 'T_flange_tcp');
end
