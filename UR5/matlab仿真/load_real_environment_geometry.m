function [objects,info,report]=load_real_environment_geometry(file,TrootCopBase,TrootBase)
% Load exported surroundings without changing tool/workpiece/robot geometry.
% TrootCopBase is the existing frozen base calibration, not a fitted display
% offset. Vertices and dimensions are already in metres and stay unchanged.
assert(isfile(file),'REAL_ENVIRONMENT:ENVIRONMENT_GEOMETRY_MISSING',...
    'REAL environment export is missing: %s. Export the stopped CoppeliaSim scene first.',file);
payload=jsondecode(fileread(file));
assert(isfield(payload,'units') && strcmp(string(payload.units),"m"),...
    'REAL_ENVIRONMENT:Units','Environment export units must be metres');
assert(isfield(payload,'frame_id') && strcmp(string(payload.frame_id),"UR5_base"),...
    'REAL_ENVIRONMENT:Frame','Environment export must use CoppeliaSim UR5_base');
validateTransform(TrootCopBase,'T_URDFroot_UR5base'); validateTransform(TrootBase,'T_URDFroot_base');
assert(isfield(payload,'frames') && isfield(payload.frames,'T_base_UR5base'),...
    'REAL_ENVIRONMENT:FrameBinding','Export is missing its calibrated UR5_base-to-base transform');
savedMapping=double(payload.frames.T_base_UR5base); validateTransform(savedMapping,'export T_base_UR5base');
assert(norm(savedMapping-(TrootBase\TrootCopBase),'fro')<1e-9,...
    'REAL_ENVIRONMENT:STALE','Export base mapping differs from the current frozen calibration / URDF base transform');
if isfield(payload.frames,'registration_source')
    registration=payload.frames.registration_source;
    for kind={'calibration','urdf'}
        pathKey=[kind{1} '_path']; shaKey=[kind{1} '_sha256'];
        assert(isfield(registration,pathKey) && isfield(registration,shaKey) && ...
            isfile(registration.(pathKey)) && ...
            strcmp(string(hmi_workflow_file_hash(registration.(pathKey))),string(registration.(shaKey))),...
            'REAL_ENVIRONMENT:STALE','Environment registration %s source changed since export',kind{1});
    end
end
if isfield(payload,'source') && isfield(payload.source,'scene') && ...
        isfield(payload.source,'scene_sha256') && ~isempty(payload.source.scene_sha256)
    assert(isfile(payload.source.scene) && ...
        strcmp(string(hmi_workflow_file_hash(payload.source.scene)),string(payload.source.scene_sha256)),...
        'REAL_ENVIRONMENT:STALE','Source CoppeliaSim scene changed since environment export');
end
assert(isfield(payload,'objects'),'REAL_ENVIRONMENT:Schema','Environment objects are missing');
objects={}; info=repmat(recordTemplate(),1,0); skipped={};
sourceCount=numel(payload.objects);
for k=1:sourceCount
    spec=item(payload.objects,k);
    assert(isfield(spec,'name') && ~isempty(spec.name),'REAL_ENVIRONMENT:Name','Environment object has no name');
    name=char(string(spec.name));
    assert(isfield(spec,'collision_enabled') && ...
        (islogical(spec.collision_enabled)||isnumeric(spec.collision_enabled)) && ...
        isscalar(spec.collision_enabled) && isfinite(double(spec.collision_enabled)) && ...
        any(double(spec.collision_enabled)==[0 1]),...
        'REAL_ENVIRONMENT:Schema','Environment object %s needs a Boolean collision_enabled flag',name);
    if ~logical(spec.collision_enabled)
        skipped{end+1}=name; continue; %#ok<AGROW>
    end
    if isfield(spec,'pose_UR5_base'), p=spec.pose_UR5_base;
    elseif isfield(spec,'pose_base'), p=spec.pose_base;
    else, error('REAL_ENVIRONMENT:Pose','Environment object %s has no UR5_base pose',name); end
    if isstruct(p) && isfield(p,'frame')
        assert(strcmp(string(p.frame),"UR5_base"),'REAL_ENVIRONMENT:Frame','Wrong pose frame for %s',name);
    end
    Tcop=pose(p); Troot=TrootCopBase*Tcop;
    if isfield(spec,'geometry'), geometry=spec.geometry; else, geometry=spec; end
    assert(isfield(geometry,'type'),'REAL_ENVIRONMENT:Schema','Environment type missing for %s',name);
    type=lower(char(string(geometry.type)));
    if ~any(strcmp(type,{'box','cuboid','cylinder','sphere'})) && ...
            isfield(geometry,'components') && ~isempty(geometry.components)
        for j=1:numel(geometry.components)
            part=item(geometry.components,j); Tpart=eye(4);
            if isfield(part,'pose_object'), Tpart=pose(part.pose_object); end
            if isfield(part,'type'), partType=char(string(part.type)); else, partType='convex_hull'; end
            partName=sprintf('%s/component_%02d',name,j);
            [collider,record]=makeObject(part,partType,partName,spec,Troot*Tpart,TrootBase);
            objects{end+1}=collider; info(end+1)=record; %#ok<AGROW>
        end
    else
        [collider,record]=makeObject(geometry,type,name,spec,Troot,TrootBase);
        objects{end+1}=collider; info(end+1)=record; %#ok<AGROW>
    end
end
assert(~isempty(objects),'REAL_ENVIRONMENT:Empty',...
    'Environment export contains no enabled collision geometry');
assert(numel(unique({info.name}))==numel(info),'REAL_ENVIRONMENT:Names','Duplicate environment collision names');
sha=hmi_workflow_file_hash(file); version=['ENVIRONMENT-' sha];
if isfield(payload,'version') && ~isempty(payload.version), version=char(string(payload.version)); end
report=struct('pass',true,'source_file',file,'source_sha256',sha,'version',version,...
    'frame_id','UR5_base','units','m','source_object_count',sourceCount,...
    'loaded_collision_object_count',numel(objects),'skipped_noncollision_objects',{skipped},...
    'T_URDFroot_UR5base',TrootCopBase,'T_base_UR5base',TrootBase\TrootCopBase,...
    'object_names',{{info.name}},'source_status','COPPELIASIM_SCENE_EXPORT',...
    'robot_motion','NO');
if isfield(payload,'missing_categories'), report.missing_categories=payload.missing_categories; end
if isfield(payload,'warnings'), report.source_warnings=payload.warnings; end
if isfield(payload,'coverage'), report.source_coverage=payload.coverage; end
if isfield(payload,'scene_coverage'), report.source_coverage=payload.scene_coverage; end
fprintf('REAL environment loaded: %d source objects, %d enabled collision objects (%s)\n',...
    sourceCount,numel(objects),file);
end

function [collider,record]=makeObject(geometry,type,name,spec,Troot,TrootBase)
record=recordTemplate(); record.name=name; record.T_matbase_proxy=Troot;
record.T_base_proxy=TrootBase\Troot; record.source_path=char(string(spec.path));
record.category='Environment';
if isfield(spec,'category'), record.source_category=char(string(spec.category)); end
switch lower(type)
    case {'box','cuboid'}
        sizeM=vector(geometry.size_m,3,name); assert(all(sizeM>0),'REAL_ENVIRONMENT:Size','Invalid box %s',name);
        collider=collisionBox(sizeM(1),sizeM(2),sizeM(3)); record.type='box'; record.size_m=sizeM;
    case 'cylinder'
        radius=scalar(geometry.radius_m,name); len=scalar(geometry.length_m,name);
        assert(radius>0 && len>0,'REAL_ENVIRONMENT:Size','Invalid cylinder %s',name);
        collider=collisionCylinder(radius,len); record.type='cylinder'; record.radius_m=radius; record.length_m=len;
    case {'sphere','spheroid'}
        assert(isfield(geometry,'radius_m'),'REAL_ENVIRONMENT:Shape',...
            'Nonspherical spheroid %s requires its exact convex mesh, not a sphere approximation',name);
        radius=scalar(geometry.radius_m,name); assert(radius>0,'REAL_ENVIRONMENT:Size','Invalid sphere %s',name);
        collider=collisionSphere(radius); record.type='sphere'; record.radius_m=radius;
    case {'convex_hull','convex_mesh','mesh'}
        if strcmpi(type,'mesh')
            assert(isfield(geometry,'is_convex') && logical(geometry.is_convex),...
                'REAL_ENVIRONMENT:Nonconvex','Nonconvex environment %s needs component export; refusing a merged convex hull',name);
        end
        V=double(geometry.vertices_m);
        assert(size(V,2)==3 && size(V,1)>=4 && all(isfinite(V(:))) && rank(V-V(1,:))==3,...
            'REAL_ENVIRONMENT:Mesh','Invalid environment mesh %s',name);
        faces=[];
        if isfield(geometry,'faces') && ~isempty(geometry.faces)
            faces=double(geometry.faces);
            assert(size(faces,2)==3 && all(isfinite(faces(:))) && all(faces(:)==round(faces(:))) && ...
                all(faces(:)>=1) && all(faces(:)<=size(V,1)),...
                'REAL_ENVIRONMENT:Mesh','Invalid one-based triangles for %s',name);
        end
        collider=collisionMesh(V); record.type='mesh'; record.vertices_m=V; record.faces=faces;
    otherwise
        error('REAL_ENVIRONMENT:Shape','Unsupported exact environment geometry %s for %s',type,name);
end
validateTransform(Troot,name); collider.Pose=Troot;
end
function value=recordTemplate()
value=struct('name','','category','Environment','source_category','','source_path','',...
    'type','','size_m',[],'radius_m',[],'length_m',[],'vertices_m',[],'faces',[],...
    'T_matbase_proxy',eye(4),'T_base_proxy',eye(4));
end
function value=item(values,k)
if iscell(values), value=values{k}; else, value=values(k); end
end
function T=pose(value)
if isnumeric(value), T=double(value); validateTransform(T,'environment pose'); return; end
assert(isstruct(value) && isfield(value,'position') && isfield(value,'quaternion'),...
    'REAL_ENVIRONMENT:Pose','Environment pose needs position and xyzw quaternion');
p=vector(value.position,3,'position'); q=vector(value.quaternion,4,'quaternion');
assert(abs(norm(q)-1)<1e-6,'REAL_ENVIRONMENT:Quaternion','Environment quaternion is not normalized');
T=eye(4); T(1:3,1:3)=quat2rotm([q(4) q(1:3)]); T(1:3,4)=p(:);
validateTransform(T,'environment pose');
end
function validateTransform(T,name)
assert(isequal(size(T),[4 4]) && all(isfinite(T(:))) && ...
    norm(T(4,:)-[0 0 0 1])<1e-9 && norm(T(1:3,1:3)'*T(1:3,1:3)-eye(3),'fro')<1e-6 && ...
    abs(det(T(1:3,1:3))-1)<1e-6,'REAL_ENVIRONMENT:Transform','Invalid rigid transform: %s',name);
end
function v=vector(value,n,name)
v=reshape(double(value),1,[]);
assert(numel(v)==n && isreal(v) && all(isfinite(v)),'REAL_ENVIRONMENT:Value','Invalid numeric vector: %s',name);
end
function v=scalar(value,name)
v=double(value); assert(isscalar(v)&&isreal(v)&&isfinite(v),'REAL_ENVIRONMENT:Value','Invalid scalar: %s',name);
end
