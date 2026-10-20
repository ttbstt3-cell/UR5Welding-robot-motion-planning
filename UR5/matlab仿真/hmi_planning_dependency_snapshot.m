function snapshot=hmi_planning_dependency_snapshot(root)
% Content snapshot of the files actually used to build the REAL scene.
% Meshes are hashed, never parsed/recentered/transformed here. The original
% large CAD source and measurement evidence are not runtime collision data.
root=char(root);
paths={fullfile(root,'real_robot_description_calibrated.urdf'),...
    fullfile(root,'real_tcp_calibration_active.json'),...
    fullfile(root,'real_gun_geometry','real_gun_geometry.json'),...
    fullfile(root,'real_gun_mount_calibration.json'),...
    fullfile(root,'workpiece_pose_real.json'),...
    fullfile(root,'scene_geometry.json'),fullfile(root,'ur5_tool_calibration.mat'),...
    fullfile(root,'ur5_hmi','config','hmi_config.yaml'),...
    fullfile(root,'real_gun_geometry','welding_gun_visual_m.stl'),...
    fullfile(root,'environment_geometry_real.json'),fullfile(root,'fixed_environment_contacts.json')};
urdf=paths{1}; discoveryErrors={};
if isfile(urdf)
    try
        doc=xmlread(urdf); collisions=doc.getElementsByTagName('collision');
        for k=0:collisions.getLength-1
            meshes=collisions.item(k).getElementsByTagName('mesh');
            for j=0:meshes.getLength-1
                uri=char(meshes.item(j).getAttribute('filename'));
                paths{end+1}=resolveMesh(uri,root); %#ok<AGROW>
            end
        end
    catch err
        discoveryErrors{end+1}=['URDF collision discovery: ' err.message];
    end
end
manifest=paths{3};
if isfile(manifest)
    try
        gun=jsondecode(fileread(manifest));
        % Current collision vertices are inline in the manifest. Also bind
        % its explicitly referenced collision export/data provenance files.
        if isfield(gun,'collision')
            paths=[paths,collisionFiles(gun.collision,root)]; %#ok<AGROW>
        end
    catch err
        discoveryErrors{end+1}=['Gun collision discovery: ' err.message];
    end
end
environmentFile=fullfile(root,'environment_geometry_real.json');
if isfile(environmentFile)
    try
        surroundings=jsondecode(fileread(environmentFile));
        if isfield(surroundings,'source') && isfield(surroundings.source,'scene') && ...
                ~isempty(surroundings.source.scene)
            paths{end+1}=char(string(surroundings.source.scene)); %#ok<AGROW>
        end
    catch err
        discoveryErrors{end+1}=['Environment source discovery: ' err.message];
    end
end
paths=sort(unique(paths));
records=repmat(struct('path','','exists',false,'sha256','','bytes',[]),1,numel(paths));
descriptor=cell(1,numel(paths));
for k=1:numel(paths)
    file=paths{k}; records(k).path=file; records(k).exists=isfile(file);
    if records(k).exists
        records(k).sha256=streamHash(file);
        info=dir(file); records(k).bytes=info.bytes;
    end
    descriptor{k}=[file '|' num2str(records(k).exists) '|' records(k).sha256];
end
snapshot=struct('schema_version','HMI_PLANNING_DEPENDENCY_SNAPSHOT_v1',...
    'files',records,'content_sha256',textHash(strjoin(descriptor,newline)),...
    'discovery_errors',{discoveryErrors},'robot_motion','NO');
end
function paths=collisionFiles(value,root)
paths={};
if iscell(value)
    for k=1:numel(value), paths=[paths,collisionFiles(value{k},root)]; end %#ok<AGROW>
elseif isstruct(value)
    for j=1:numel(value)
        for key=fieldnames(value(j)).'
            field=key{1}; item=value(j).(field);
            if (endsWith(field,'_file') || endsWith(field,'_path') || ...
                    any(strcmp(field,{'file','path','mesh','stl'}))) && ...
                    (ischar(item) || (isstring(item)&&isscalar(item))) && ~isempty(item)
                file=char(item);
                if ~startsWith(file,filesep), file=fullfile(root,file); end
                paths{end+1}=file; %#ok<AGROW>
            elseif isstruct(item) || iscell(item)
                paths=[paths,collisionFiles(item,root)]; %#ok<AGROW>
            end
        end
    end
end
end
function file=resolveMesh(uri,root)
parts=regexp(uri,'^package://([^/]+)/(.+)$','tokens','once');
if isempty(parts)
    file=uri; if ~startsWith(file,filesep), file=fullfile(root,file); end
    return;
end
roots={'/opt/ros/humble/share'}; prefixes=strsplit(getenv('AMENT_PREFIX_PATH'),':');
for k=1:numel(prefixes)
    if ~isempty(prefixes{k}), roots{end+1}=fullfile(prefixes{k},'share'); end %#ok<AGROW>
end
file=fullfile(roots{1},parts{1},parts{2});
for k=1:numel(roots)
    candidate=fullfile(roots{k},parts{1},parts{2});
    if isfile(candidate), file=candidate; return; end
end
end
function hash=streamHash(file)
fid=fopen(file,'r'); assert(fid>=0,'HMI:SnapshotRead','Cannot read scene dependency: %s',file);
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
md=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid), bytes=fread(fid,1024*1024,'*uint8'); md.update(bytes); end
hash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
function hash=textHash(value)
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(unicode2native(value,'UTF-8'));
hash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
