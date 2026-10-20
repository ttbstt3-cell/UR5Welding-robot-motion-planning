function calib = load_real_tcp_calibration(jsonFile)
%LOAD_REAL_TCP_CALIBRATION Read a validated real TCP record; no robot I/O.
%
% Default REAL planning uses only the controller-authoritative ACTIVE mirror.
% It never substitutes historical four-point or frozen simulation parameters.
% An explicitly supplied legacy file remains available for offline diagnostics.
% This loader never changes a robot model or a frozen calibration file.

requireControllerActive = nargin < 1 || strlength(string(jsonFile)) == 0;
if requireControllerActive
    root = fullfile(fileparts(mfilename('fullpath')), '..');
    jsonFile = fullfile(root, 'real_tcp_calibration_active.json');
end
jsonFile = char(string(jsonFile));
[~, recordName, recordExt] = fileparts(jsonFile);
requireControllerActive = requireControllerActive || ...
    strcmp([recordName recordExt], 'real_tcp_calibration_active.json');
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
if ~isscalar(data.valid) || ~logical(data.valid)
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
if requireControllerActive || isfield(data, 'tcp_authority')
    % Source validation does not claim a measured tool-tip calibration RMS.
    % Recompute it rather than trusting persisted PASS/threshold metadata.
    calib.controllerSourceValidation = validate_controller_tcp_calibration(data);
end
end
