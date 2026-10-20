function wire = model_axis_wire_to_tcp(manifest,rootDir,previewOnly)
% Same read-only definition as HMI calibration/model_axis_wire.py.
% No marker radius, standalone endpoint or measured-nozzle record required.
if nargin<3, previewOnly=false; end
if ~strcmpi(string(manifest.status),'ACTIVE') && ...
        ~(previewOnly && strcmpi(string(manifest.status),'STAGED_ALIGNMENT_REQUIRED'))
    error('REAL_GUN:WireModel','Gun model must be ACTIVE');
end
if isfield(manifest.wire,'gun_tip_model')
    start=double(manifest.wire.gun_tip_model(:));
else
    start=double(manifest.wire.tip_source(:));
end
if numel(start)~=3 || any(~isfinite(start))
    error('REAL_GUN:WireModel','Missing finite CAD gun_tip_model in STL_SOURCE metres');
end
T=double(manifest.visual.T_flange_source);
if ~isequal(size(T),[4 4]) || any(~isfinite(T(:))) || ...
        norm(T(4,:)-[0 0 0 1])>1e-9 || ...
        norm(T(1:3,1:3)'*T(1:3,1:3)-eye(3),'fro')>1e-6 || ...
        abs(det(T(1:3,1:3))-1)>1e-6
    error('REAL_GUN:WireFrame','Invalid flange <- STL_SOURCE transform');
end
% Match build_real_planning_scene, including full pendant TCP adoption.
file=fullfile(rootDir,'real_tcp_calibration_active.json');
tcp=load_real_tcp_calibration(file);
if ~isfield(tcp,'applied_to_planning_model') || ~tcp.applied_to_planning_model || ...
        ~strcmp(tcp.coordinate_frames.flange,'URDF flange')
    error('REAL_GUN:WireTcp','TCP must be applied in URDF flange');
end
endpoint=T\[tcp.position(:);1]; endpoint=endpoint(1:3);
delta=endpoint-start; lengthM=norm(delta); diameterM=0.0012;
if ~isfinite(lengthM) || lengthM<=0
    error('REAL_GUN:WireLength','MODEL_DEFINED_WIRE_INVALID: length must be > 0');
end
axis=delta/lengthM;
if diameterM<=0 || abs(norm(axis)-1)>1e-6
    error('REAL_GUN:WireAxis','MODEL_DEFINED_WIRE_INVALID: diameter/axis invalid');
end
status='MODEL_AXIS_TO_TCP_ACTIVE';
if previewOnly, status='MODEL_AXIS_TO_TCP_PREVIEW'; end
wire=struct('status',status,'preview_only',logical(previewOnly),'tip_source',start, ...
    'tcp_source',endpoint,'wire_end_source',endpoint,'axis_source',axis, ...
    'length_m',lengthM,'diameter_m',diameterM,'tcp_source_reference',file);
end
