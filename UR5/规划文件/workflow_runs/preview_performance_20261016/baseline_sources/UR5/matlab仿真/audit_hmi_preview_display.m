function report=audit_hmi_preview_display(rawFile,outputDir)
% Offline preview audit using the exact saved robot, tool and workpiece.
% No planner, motion, TCP, geometry or self-skip changes. A saved, explicitly
% certified fixed base/environment contact policy is applied read-only.
here=fileparts(mfilename('fullpath')); root=fileparts(here);
addpath(genpath(fullfile(fileparts(root),'RRT')));
if ~isfolder(outputDir), mkdir(outputDir); end
r=load(rawFile); s=load(r.sceneFile,'scene'); scene=s.scene;
assert(strcmp(hmi_workflow_file_hash(r.sceneFile),r.metadata.scene_snapshot_sha256),...
    'PREVIEW:SnapshotChanged','Saved scene snapshot hash mismatch');
robot=scene.robot; n=size(r.qPath,1);
names=cellstr(string([robot.BodyNames {robot.BaseName}]));
envNames=hmi_environment_names(scene);
workpieceNames=cellstr(string({scene.workpieceProxyInfo.name}));
required={'base_link_inertia','shoulder_link','upper_arm_link','forearm_link',...
    'wrist_1_link','wrist_2_link','wrist_3_link'};
missing={}; bodyCounts=struct('name',{},'collision_count',{});
for k=1:numel(required)
    b=getBody(robot,required{k}); count=numel(b.Collisions);
    bodyCounts(end+1)=struct('name',required{k},'collision_count',count); %#ok<AGROW>
    if count==0, missing{end+1}=required{k}; end %#ok<AGROW>
end
coveragePass=isempty(missing);
Tbf=getTransform(robot,zeros(1,6),'base');
Tfm=getTransform(robot,zeros(1,6),'WeldingGunMount','flange');
Tft=getTransform(robot,zeros(1,6),scene.tcpBodyName,'flange');
Tfw=getTransform(robot,zeros(1,6),char(scene.wireBodyName),'flange');
L=scene.wireLengthM; Lc=L;
if isfield(scene,'wireCollisionLengthM'), Lc=scene.wireCollisionLengthM; end
snapshot=struct('frame_id','base','T_flange_to_gun',scene.T_flange_source,...
    'T_flange_to_mount',Tfm,'T_flange_to_tcp',Tft,'T_flange_to_wire',Tfw,...
    'wire_length_m',L,'wire_collision_length_m',Lc,'wire_diameter_m',scene.wireDiameterM,...
    'wire_collision_shape_pose',eye(4),'components',{scene.gunProxyInfo},...
    'scene_sha256',r.metadata.scene_snapshot_sha256,'calibration_binding',r.metadata.calibration_binding);
snapshot.collision_coverage_pass=coveragePass;
snapshot.missing_robot_collision_bodies=missing;
snapshot.robot_collision_body_counts=bodyCounts;
snapshot.self_collision_skip_pairs=num2cell(scene.selfCollisionSkipPairs,2);
snapshot.attached_gun_wire_collision_policy=struct('status','SKIPPED_FIXED_ATTACHMENT',...
    'pair',{{'WeldingGunMount','WeldingGunWire'}},'checked',false,...
    'reason','Existing fixed internal attachment exclusion is retained');
snapshot.environment=environmentSnapshot(scene,Tbf,envNames,workpieceNames);
snapshot.environment_object_count=numel(scene.environment);
if isfield(scene,'fixedEnvironmentContact')
    snapshot.fixed_environment_contact=scene.fixedEnvironmentContact;
end
if isfield(scene,'realEnvironmentGeometryLoad')
    snapshot.environment_geometry_load=scene.realEnvironmentGeometryLoad;
end
snapshot.visual_path=''; snapshot.visual_units='m'; snapshot.visual_sha256='';
src=fullfile(root,'real_gun_geometry','welding_gun_visual_m.stl');
if isfile(src)
    dst=fullfile(outputDir,'preview_gun_visual_m.stl'); copyfile(src,dst);
    snapshot.visual_path=dst; snapshot.visual_sha256=hmi_workflow_file_hash(dst);
    snapshot.visual_provenance='Existing visual file copied at audit time; collision vertices remain exact saved snapshot';
end

flangePosePath=cell(1,n); gunPosePath=cell(1,n); wirePosePath=cell(1,n);
collisionResult=cell(1,n); frames=cell(1,n);
queryCount=0; nodeHitCount=0; edgeHitCount=0; allPairs={}; allCategories={};
firstNodeHit=[]; firstAnyHit=[]; minimumPositiveGap=inf; minGapSample=[];
firstByCategory=struct();
for i=1:n
    q=r.qPath(i,:); Tf=Tbf\getTransform(robot,q,'flange');
    Tg=Tf*scene.T_flange_source; Tw=Tf*Tfw; Tt=Tf*Tft;
    flangePosePath{i}=Tf; gunPosePath{i}=Tg; wirePosePath{i}=Tw;
    gunTip=Tw*[0;0;-L/2;1]; wireTip=Tw*[0;0;L/2;1];
    [node,stat]=check(q,i,'node',0,r.timeFromStart(i));
    queryCount=queryCount+1;
    if ~node.safe
        nodeHitCount=nodeHitCount+1; allPairs=[allPairs,node.pair_strings]; %#ok<AGROW>
        allCategories=[allCategories,unique(node.categories)]; %#ok<AGROW>
        if isempty(firstNodeHit), firstNodeHit=stat; end
        if isempty(firstAnyHit), firstAnyHit=stat; end
        rememberFirstCategory(stat);
    end
    if isfinite(node.gap_mm) && node.gap_mm<minimumPositiveGap
        minimumPositiveGap=node.gap_mm; minGapSample=stat;
    end
    nodeRecord=node;
    edgeRecords=emptySampleArray(); edgePairStrings={};
    if i<n
        for u=linspace(0,1,21)
            qi=(1-u)*q+u*r.qPath(i+1,:);
            ti=(1-u)*r.timeFromStart(i)+u*r.timeFromStart(i+1);
            [edge,edgeStat]=check(qi,i,'edge',u,ti); queryCount=queryCount+1;
            if isfinite(edge.gap_mm) && edge.gap_mm<minimumPositiveGap
                minimumPositiveGap=edge.gap_mm; minGapSample=edgeStat;
            end
            if ~edge.safe
                edgeHitCount=edgeHitCount+1;
                edgeRecords=[edgeRecords,edgeStat]; %#ok<AGROW>
                edgePairStrings=[edgePairStrings,edge.pair_strings]; %#ok<AGROW>
                allPairs=[allPairs,edge.pair_strings]; %#ok<AGROW>
                allCategories=[allCategories,unique(edge.categories)]; %#ok<AGROW>
                if isempty(firstAnyHit), firstAnyHit=edgeStat; end
                rememberFirstCategory(edgeStat);
            end
        end
    end
    edgePairStrings=uniqueText(edgePairStrings);
    status='PASS';
    if ~coveragePass, status='PREVIEW WARNING';
    elseif ~node.safe || ~isempty(edgeRecords), status='WARNING'; end
    collisionResult{i}=struct('collision_status',status,'collision_pairs',{nodeRecord.pair_strings},...
        'collision_details',nodeRecord.pair_details,'outgoing_edge_pairs',{edgePairStrings},...
        'outgoing_edge_collision_samples',edgeRecords,'min_positive_gap_mm',nodeRecord.gap_mm,...
        'min_gap_pair',nodeRecord.gap_pair,'min_gap_category',nodeRecord.gap_category,...
        'missing_collision_bodies',{missing});
    frames{i}=struct('index',i,'time_s',r.timeFromStart(i),'q_rad',q,...
        'segment',r.segmentLabel{i},'flange_pose',Tf,'gun_pose',Tg,'wire_pose',Tw,...
        'tcp_pose',Tt,'gun_tip_world',gunTip(1:3).','wire_tip_world',wireTip(1:3).',...
        'collision_status',status,'collision_pairs',{nodeRecord.pair_strings},...
        'collision_details',nodeRecord.pair_details,'outgoing_edge_collision_samples',edgeRecords,...
        'outgoing_edge_pairs',{edgePairStrings},'min_positive_gap_mm',nodeRecord.gap_mm,...
        'min_gap_pair',nodeRecord.gap_pair,'min_gap_category',nodeRecord.gap_category);
end

pairOccurrences=allPairs; pairNames=uniqueText(pairOccurrences); pairSummary=struct('pair',{},'occurrence_count',{});
for k=1:numel(pairNames)
    pairSummary(k)=struct('pair',pairNames{k},'occurrence_count',sum(strcmp(pairOccurrences,pairNames{k}))); %#ok<AGROW>
end
allPairs=pairNames; categoryNames={'RobotSelfCollision','RobotGunCollision','RobotWireCollision',...
    'RobotWorkpieceCollision','GunWorkpieceCollision','WireWorkpieceCollision','ToolInternalCollision',...
    'RobotEnvironmentCollision','GunEnvironmentCollision','WireEnvironmentCollision'};
categorySummary=struct('category',{},'sample_count',{});
for k=1:numel(categoryNames)
    categorySummary(k)=struct('category',categoryNames{k},...
        'sample_count',sum(strcmp(allCategories,categoryNames{k}))); %#ok<AGROW>
end
status='PASS'; if ~coveragePass, status='PREVIEW WARNING'; elseif queryCount>0 && (nodeHitCount+edgeHitCount)>0, status='WARNING'; end
snapshotFile=fullfile(outputDir,'preview_display_snapshot.json'); writeJson(snapshotFile,snapshot);
previewDisplay=struct('snapshot_file',snapshotFile,'snapshot_sha256',hmi_workflow_file_hash(snapshotFile));
outputFile=fullfile(outputDir,'full_raw_trajectory.mat');
if ~strcmp(char(rawFile),outputFile), copyfile(rawFile,outputFile); end
metadata=r.metadata; metadata.collision_verified=coveragePass;
metadata.collision_check_pass=coveragePass && (nodeHitCount+edgeHitCount)==0;
metadata.collision_status=status; metadata.collision_pairs=allPairs;
metadata.preview_display=previewDisplay;
save(outputFile,'metadata','flangePosePath','gunPosePath','wirePosePath','collisionResult','previewDisplay','-append');

report=struct('preview_available',true,'previewReady',coveragePass && nodeHitCount+edgeHitCount==0,...
    'executionReady',false,'collision_status',status,'collision_pairs',{allPairs},...
    'collision_check_pass',coveragePass && nodeHitCount+edgeHitCount==0,...
    'collision_sample_count',queryCount,'node_collision_sample_count',nodeHitCount,...
    'collision_hit_count',nodeHitCount+edgeHitCount,...
    'edge_collision_sample_count',edgeHitCount,'collision_category_summary',categorySummary,...
    'collision_pair_summary',pairSummary,'first_collision_by_category',firstByCategory,...
    'minimum_positive_distance_mm',finiteOrEmpty(minimumPositiveGap),...
    'minimum_positive_distance_scope','Non-colliding pair distances only; not clearance of a colliding path',...
    'minimum_positive_distance_sample',minGapSample,'first_collision_frame',firstCollisionFrame(),...
    'first_collision_objects',{firstCollisionObjects()},...
    'first_collision_sample',firstAnyHit,'robot_collision_body_counts',bodyCounts,...
    'collision_coverage_pass',coveragePass,'missing_robot_collision_bodies',{missing},...
    'environment_object_count',numel(scene.environment),'environment_object_names',{envNames},...
    'attached_gun_wire_collision_policy',snapshot.attached_gun_wire_collision_policy,...
    'frame_id','base','frames',{frames},'T_flange_to_gun',scene.T_flange_source,...
    'scene_sha256',snapshot.scene_sha256,'raw_sha256',hmi_workflow_file_hash(rawFile),...
    'robot_motion','NO','scope','Saved preview only; existing tool/workpiece geometry and self-collision skip pairs unchanged; 21 samples per displayed edge');
if ~coveragePass
    report.warning_reason='ROBOT_COLLISION_GEOMETRY_MISSING';
elseif nodeHitCount+edgeHitCount>0
    report.warning_reason='COLLISION_DETECTED';
end
report.geometry_load_report=[];
if isfield(scene,'robotBodyCollisionLoad'), report.geometry_load_report=scene.robotBodyCollisionLoad; end
if isfield(scene,'realEnvironmentGeometryLoad')
    report.environment_geometry_load=scene.realEnvironmentGeometryLoad;
    report.environment_collision_scope='Only objects in the saved exported environment; missing source categories are not certified';
end
writeJson(fullfile(outputDir,'preview_display_audit.json'),report);
fprintf('PREVIEW AUDIT nodes=%d queries=%d nodeHits=%d edgeHitSamples=%d status=%s minGapMm=%g\n',...
    n,queryCount,nodeHitCount,edgeHitCount,status,finiteOrEmpty(minimumPositiveGap));

    function [result,record]=check(q,frameIndex,kind,fraction,timeValue)
        [env,de]=check_scene_environment_collision(scene,q);
        [self,ds]=checkCollision(robot,q,'IgnoreSelfCollision','off',...
            'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
        safe=~any(env(:)) && ~any(self(:)); pairs={}; categories={}; details=emptyDetailArray();
        if any(env(:))
            [ii,jj]=find(isnan(de));
            for z=1:numel(ii)
                if ii(z)>numel(robot.BodyNames) || isempty(getBody(robot,names{ii(z)}).Collisions), continue; end
                a=names{ii(z)}; b=envNames{jj(z)}; cat=classifyPair(a,b);
                pairs{end+1}=[a ' <-> ' b]; categories{end+1}=cat; %#ok<AGROW>
                details=[details,makeDetail(cat,a,b,'environment collision flag; signed penetration not returned')]; %#ok<AGROW>
            end
        end
        if any(self(:))
            [ii,jj]=find(triu(isnan(ds),1));
            for z=1:numel(ii)
                if ii(z)>numel(robot.BodyNames) || jj(z)>numel(robot.BodyNames), continue; end
                if isempty(getBody(robot,names{ii(z)}).Collisions) || isempty(getBody(robot,names{jj(z)}).Collisions), continue; end
                if any((strcmp(scene.selfCollisionSkipPairs(:,1),names{ii(z)}) & strcmp(scene.selfCollisionSkipPairs(:,2),names{jj(z)})) | ...
                        (strcmp(scene.selfCollisionSkipPairs(:,1),names{jj(z)}) & strcmp(scene.selfCollisionSkipPairs(:,2),names{ii(z)}))), continue; end
                a=names{ii(z)}; b=names{jj(z)}; cat=classifyPair(a,b);
                pairs{end+1}=[a ' <-> ' b]; categories{end+1}=cat; %#ok<AGROW>
                details=[details,makeDetail(cat,a,b,'self collision flag; signed penetration not returned')]; %#ok<AGROW>
            end
        end
        gap=inf; gapPair=''; gapCategory='';
        [ii,jj]=find(isfinite(de)&de>0);
        for z=1:numel(ii)
            if de(ii(z),jj(z))*1000<gap
                gap=de(ii(z),jj(z))*1000; gapPair=[names{ii(z)} ' <-> ' envNames{jj(z)}];
                gapCategory=classifyPair(names{ii(z)},envNames{jj(z)});
            end
        end
        [ii,jj]=find(triu(isfinite(ds)&ds>0,1));
        for z=1:numel(ii)
            if ds(ii(z),jj(z))*1000<gap
                gap=ds(ii(z),jj(z))*1000; gapPair=[names{ii(z)} ' <-> ' names{jj(z)}];
                gapCategory=classifyPair(names{ii(z)},names{jj(z)});
            end
        end
        if ~isfinite(gap), gap=NaN; gapPair=''; gapCategory=''; end
        result=struct('safe',safe,'pair_strings',{uniqueText(pairs)},'categories',{uniqueText(categories)},...
            'pair_details',details,'gap_mm',gap,'gap_pair',gapPair,'gap_category',gapCategory);
        record=struct('frame_index',frameIndex,'sample_kind',kind,'edge_fraction',fraction,...
            'time_s',timeValue,'q_rad',q,'collision_pairs',{uniqueText(pairs)},...
            'collision_details',details,'categories',{categories},...
            'min_positive_gap_mm',finiteOrEmpty(gap),'min_gap_pair',gapPair,'min_gap_category',gapCategory);
    end
    function detail=makeDetail(cat,a,b,distanceStatus)
        detail=struct('category',cat,'object_a',a,'object_b',b,'pair',[a ' <-> ' b],...
            'separation_mm',[],'distance_status',distanceStatus);
    end
    function frame=firstCollisionFrame()
        if ~isempty(firstAnyHit), frame=firstAnyHit.frame_index;
        else, frame=[]; end
    end
    function objects=firstCollisionObjects()
        if isempty(firstAnyHit), objects={}; else, objects=firstAnyHit.collision_pairs; end
    end
    function rememberFirstCategory(sample)
        for ci=1:numel(sample.collision_details)
            detail=sample.collision_details(ci); key=detail.category;
            if ~isfield(firstByCategory,key)
                firstByCategory.(key)=struct('frame_index',sample.frame_index,...
                    'sample_kind',sample.sample_kind,'edge_fraction',sample.edge_fraction,...
                    'time_s',sample.time_s,'q_rad',sample.q_rad,'pair',detail.pair);
            end
        end
    end
    function cat=classifyPair(a,b)
        a=char(a); b=char(b);
        mount=char(scene.mountBodyName); wire=char(scene.wireBodyName); tcp=char(scene.tcpBodyName);
        isGun=strcmp(a,mount)||strcmp(b,mount); isWire=strcmp(a,wire)||strcmp(b,wire);
        isTcp=strcmp(a,tcp)||strcmp(b,tcp); isEnvA=any(strcmp(envNames,a)); isEnvB=any(strcmp(envNames,b));
        if (isGun&&isWire)||(isTcp&&(isGun||isWire)), cat='ToolInternalCollision'; return; end
        if (isEnvA||isEnvB)
            if any(strcmp(workpieceNames,a)) || any(strcmp(workpieceNames,b)), suffix='WorkpieceCollision';
            else, suffix='EnvironmentCollision'; end
            if isGun||isTcp, cat=['Gun' suffix];
            elseif isWire, cat=['Wire' suffix]; else, cat=['Robot' suffix]; end
            return;
        end
        if isGun||isTcp, cat='RobotGunCollision'; return; end
        if isWire, cat='RobotWireCollision'; return; end
        cat='RobotSelfCollision';
    end
end
function records=environmentSnapshot(scene,Trootbase,names,workpieceNames)
% Export exact collision objects for the HMI; never reconstruct their pose
% from a different workpiece or environment source after the audit.
records=cell(1,numel(scene.environment));
for k=1:numel(scene.environment)
    collider=scene.environment{k}; category='Environment';
    if any(strcmp(workpieceNames,names{k})), category='Workpiece'; end
    value=struct('name',names{k},'category',category,'frame_id','base',...
        'pose_base',Trootbase\collider.Pose);
    if isa(collider,'collisionBox')
        value.type='box'; value.size_m=[collider.X collider.Y collider.Z];
    elseif isa(collider,'collisionCylinder')
        value.type='cylinder'; value.radius_m=collider.Radius; value.length_m=collider.Length;
    elseif isa(collider,'collisionSphere')
        value.type='sphere'; value.radius_m=collider.Radius;
    elseif isa(collider,'collisionMesh')
        value.type='mesh'; value.vertices_m=collider.Vertices;
        value.faces=convhull(collider.Vertices);
    else
        error('REAL_ENVIRONMENT:Snapshot','Unsupported collider type %s',class(collider));
    end
    records{k}=value;
end
end
function value=emptyDetailArray()
value=struct('category',{},'object_a',{},'object_b',{},'pair',{},'separation_mm',{},'distance_status',{});
end
function value=emptySampleArray()
value=struct('frame_index',{},'sample_kind',{},'edge_fraction',{},'time_s',{},'q_rad',{},...
    'collision_pairs',{},'collision_details',{},'categories',{},'min_positive_gap_mm',{},...
    'min_gap_pair',{},'min_gap_category',{});
end
function value=finiteOrEmpty(x)
if isempty(x)||~isscalar(x)||~isfinite(x), value=[]; else, value=x; end
end
function value=uniqueText(items)
if isempty(items), value={}; return; end
value=cellfun(@char,items,'UniformOutput',false); value=unique(value);
end
function writeJson(file,data)
fid=fopen(file,'w'); assert(fid>=0,'Cannot write %s',file); c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(data,'PrettyPrint',true),'char');
end
