function realGun = load_real_gun_geometry(rootDir)
%LOAD_REAL_GUN_GEOMETRY Load the independent real-gun manifest read-only.
%
% The frozen CoppeliaSim scene is deliberately not edited.  A manifest is
% consumable by the REAL planning scene only after an explicit human review
% changes status to ACTIVE and supplies a finite source->flange transform.

if nargin < 1 || strlength(string(rootDir)) == 0
    rootDir = fileparts(fileparts(mfilename("fullpath")));
end

manifestFile = fullfile(rootDir,"real_gun_geometry","real_gun_geometry.json");
realGun = struct(...
    "available",false,...
    "active",false,...
    "status","MISSING",...
    "manifestFile",manifestFile,...
    "wireError","Gun model is not ACTIVE or installation data is invalid",...
    "manifest",struct());

if ~isfile(manifestFile)
    return;
end

try
    manifest = jsondecode(fileread(manifestFile));
catch ME
    realGun.status = "INVALID_MANIFEST";
    realGun.error = ME.message;
    return;
end

realGun.available = true;
realGun.manifest = manifest;
if isfield(manifest,"status")
    realGun.status = string(manifest.status);
else
    realGun.status = "UNSPECIFIED";
end

% Staged geometry remains visible to diagnostics, but it cannot replace the
% frozen collision scene.  This explicit gate prevents guessed centring,
% rotation, or TCP/wire data from entering RRT.
if upper(realGun.status) ~= "ACTIVE"
    return;
end

if ~isfield(manifest,"visual") || ~isfield(manifest.visual,"T_flange_source")
    realGun.status = "ACTIVE_BUT_NO_FLANGE_TRANSFORM";
    return;
end

T = double(manifest.visual.T_flange_source);
if ~isequal(size(T),[4 4]) || any(~isfinite(T(:))) || ...
        norm(T(4,:)-[0 0 0 1]) > 1e-8
    realGun.status = "ACTIVE_BUT_INVALID_FLANGE_TRANSFORM";
    return;
end

if ~isfield(manifest,"collision") || ...
        ~isfield(manifest.collision,"components") || ...
        upper(string(manifest.collision.status)) ~= "CONFIRMED"
    realGun.status = "ACTIVE_BUT_COLLISION_NOT_CONFIRMED";
    return;
end

if ~isfield(manifest,"mounting") || ...
        upper(string(manifest.mounting.status)) ~= "CONFIRMED" || ...
        ~isfield(manifest.collision,"T_mount_source")
    realGun.status = "ACTIVE_BUT_MOUNT_NOT_CONFIRMED";
    return;
end

if isempty(manifest.collision.components) || ...
        norm(T(1:3,1:3)'*T(1:3,1:3)-eye(3),'fro') > 1e-6 || ...
        abs(det(T(1:3,1:3))-1) > 1e-6
    realGun.status = "ACTIVE_BUT_INVALID_GEOMETRY";
    return;
end

% Derive a fixed cylinder from the ACTIVE model reference to applied TCP.
% Old sphere-surface endpoints are not physical geometry and are ignored.
realGun.wireStatus = "ACTIVE_BUT_INVALID_WIRE_GEOMETRY";
realGun.wireGeometryValid = false;
try
    wire=model_axis_wire_to_tcp(manifest,rootDir);
    realGun.wireStatus=string(wire.status);
    realGun.wireGeometryValid=true;
    realGun.wireTipSource=wire.tip_source;
    realGun.wireTcpSource=wire.tcp_source;
    realGun.wireEndSource=wire.wire_end_source;
    realGun.wireAxisSource=wire.axis_source;
    realGun.wireLengthM=wire.length_m;
    realGun.wireDiameterM=wire.diameter_m;
    realGun.wireTcpFile=wire.tcp_source_reference;
catch ME
    realGun.wireError=ME.message;
end

realGun.active = true;
realGun.T_flange_source = T;
realGun.T_mount_source = double(manifest.collision.T_mount_source);
end
