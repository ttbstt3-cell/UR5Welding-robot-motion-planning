function [robot,report]=load_ur5_body_collision_geometry(robot,urdfFile)
% Fill ONLY empty UR5 body collisions from exact URDF collision STL/origin.
% No edits to joints, inertia, TCP/tool/environment, or collision skip pairs.
% base_link is a frame-only link; its fixed base_link_inertia owns base.stl.
robot=copy(robot); doc=xmlread(urdfFile); links=doc.getElementsByTagName('link');
requested={'base_link','shoulder_link','upper_arm_link','forearm_link','wrist1','wrist2','wrist3'};
targets={'base_link_inertia','shoulder_link','upper_arm_link','forearm_link',...
    'wrist_1_link','wrist_2_link','wrist_3_link'};
records=cell(1,numel(targets)); allLoaded=true;
for k=1:numel(targets)
    name=targets{k}; body=getBody(robot,name); before=numel(body.Collisions);
    link=[];
    for j=0:links.getLength-1
        candidate=links.item(j);
        if strcmp(char(candidate.getAttribute('name')),name), link=candidate; break; end
    end
    assert(~isempty(link),'UR5_COLLISION:URDF','Missing URDF link %s',name);
    shapes=link.getElementsByTagName('collision'); meshes={}; origins={}; meshHashes={};
    for j=0:shapes.getLength-1
        c=shapes.item(j); items=c.getElementsByTagName('mesh');
        if items.getLength==0, continue; end
        m=items.item(0); uri=char(m.getAttribute('filename')); path=resolve(uri,urdfFile);
        scale=triple(char(m.getAttribute('scale')),[1 1 1]);
        assert(all(scale>0),'UR5_COLLISION:Scale','Invalid mesh scale for %s',name);
        origin=eye(4); o=c.getElementsByTagName('origin');
        if o.getLength>0
            xyz=triple(char(o.item(0).getAttribute('xyz')),[0 0 0]);
            rpy=triple(char(o.item(0).getAttribute('rpy')),[0 0 0]);
            origin(1:3,1:3)=eul2rotm([rpy(3) rpy(2) rpy(1)],'ZYX'); origin(1:3,4)=xyz(:);
        end
        meshes{end+1}=path; origins{end+1}=origin; meshHashes{end+1}=hmi_workflow_file_hash(path); %#ok<AGROW>
        if before==0
            mesh=stlread(path); vertices=double(mesh.Points).*scale;
            assert(size(vertices,2)==3 && all(isfinite(vertices(:))) && rank(vertices-vertices(1,:))==3,...
                'UR5_COLLISION:Mesh','Invalid collision STL %s',path);
            addCollision(body,collisionMesh(vertices),origin);
        end
    end
    if before==0, replaceBody(robot,name,body); end
    count=numel(getBody(robot,name).Collisions); allLoaded=allLoaded && count>0;
    records{k}=struct('requested_link',requested{k},'geometry_link',name,...
        'before_count',before,'after_count',count,'added',before==0 && count>0,...
        'mesh_files',{meshes},'mesh_sha256',{meshHashes},'collision_origins',{origins});
end
report=struct('pass',allLoaded,'urdf_file',urdfFile,'urdf_sha256',hmi_workflow_file_hash(urdfFile),...
    'links',{records},'base_alias','base_link is frame-only; fixed base_link_inertia owns base.stl',...
    'units','metres, URDF scale only; no centering/scaling/alignment adjustment',...
    'robot_motion','NO');
assert(allLoaded,'UR5_COLLISION:Coverage','UR5 body collision geometry remains missing');
end
function values=triple(text,default)
values=default; if ~isempty(strtrim(text)), values=sscanf(text,'%f').'; end
assert(numel(values)==3 && all(isfinite(values)),'UR5_COLLISION:Origin','Invalid URDF vector');
end
function path=resolve(uri,urdfFile)
tokens=regexp(uri,'^package://([^/]+)/(.+)$','tokens','once');
if ~isempty(tokens)
    roots={'/opt/ros/humble/share'};
    prefixes=strsplit(getenv('AMENT_PREFIX_PATH'),':');
    for k=1:numel(prefixes), if ~isempty(prefixes{k}), roots{end+1}=fullfile(prefixes{k},'share'); end; end
    path='';
    for k=1:numel(roots)
        p=fullfile(roots{k},tokens{1},tokens{2}); if isfile(p), path=p; break; end
    end
elseif startsWith(uri,'/'), path=uri;
else, path=fullfile(fileparts(urdfFile),uri); end
assert(~isempty(path) && isfile(path),'UR5_COLLISION:MissingMesh','Cannot resolve %s',uri);
end
