function report=diagnose_accp_point1_collision_pairs(inputFile,outputDir,candidateFilter)
% Exhaustive offline attribution of the EXACT saved Point1 candidates.
% Copies isolate each installed convex component; the original robot/gates
% are never modified. MATLAB returns NaN for intersecting geometry distance,
% and does not expose penetration depth through checkCollision.
if nargin<3, candidateFilter='historical_154'; end
S=load(inputFile,'result'); result=S.result; scene=result.scene;
assert(~result.options.disableAxisConstraint);
assert(result.options.checkSelfCollision && scene.wireCollisionEnabled);
assert(scene.realTcpLoaded && scene.realWorkpieceLoaded && scene.realGunComponentCount>0);
fresh=build_real_planning_scene(false);
sceneIsCurrent=isequal(scene.calibrationBinding,fresh.calibrationBinding);
% Explicit historical audit uses the serialized robot/environment unchanged.
% This does not promote stale candidates to planning or execution eligibility.
if ~strcmp(candidateFilter,'workpiece_collision')
    assert(sceneIsCurrent,'Saved scene version is stale');
end
candidates=result.firstPointDiagnostics.candidates;
originalIndices=1:numel(candidates);
if strcmp(candidateFilter,'all_ik')
    assert(numel(candidates)==result.firstPointDiagnostics.ik_success, ...
        'Pose-valid IK records do not match success count');
elseif strcmp(candidateFilter,'workpiece_collision')
    mask=arrayfun(@(c)strcmp(string(c.rejection_reason),"workpiece_collision"),candidates);
    originalIndices=originalIndices(mask); candidates=candidates(mask);
    assert(numel(candidates)==result.firstPointDiagnostics.workpiece_collision_rejected, ...
        'Saved workpiece rejection count does not match candidate records');
else
    assert(strcmp(candidateFilter,'historical_154'));
    assert(result.options.sideToleranceDeg==20 && numel(candidates)==154, ...
        'Expected historical 20-degree run and all 154 saved Point1 candidates');
end
assert(~isempty(candidates),'No candidates selected for collision audit');
if ~isfolder(outputDir), mkdir(outputDir); end
robot=scene.robot; names=[robot.BodyNames {robot.BaseName}];
mountIndex=find(strcmp(names,'WeldingGunMount')); wireIndex=find(strcmp(names,'WeldingGunWire'));
assert(~isempty(mountIndex) && ~isempty(wireIndex));
components=scene.gunProxyInfo; componentTrees=cell(1,numel(components));
componentNames=cell(1,numel(components));
for k=1:numel(components)
    if iscell(components), c=components{k}; else, c=components(k); end
    componentNames{k}=sprintf('GunCollisionHull/component_%02d',k);
    variant=copy(robot); body=getBody(variant,'WeldingGunMount');
    clearCollision(body);
    addCollision(body,collisionMesh(double(c.vertices_m)),scene.T_mount_source);
    replaceBody(variant,'WeldingGunMount',body); componentTrees{k}=variant;
end
envNames={scene.workpieceProxyInfo.name};
records=cell(1,numel(candidates)); objectCounts=containers.Map(); pairCounts=containers.Map();
for key=[envNames componentNames {'WireCollision'}]
    objectCounts(char(key{1}))=0;
end
for i=1:numel(candidates)
    q=candidates(i).q_rad;
    [envFlag,envDist]=checkCollision(robot,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on');
    [selfFlag,selfDist]=checkCollision(robot,q,'IgnoreSelfCollision','off', ...
        'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
    pairs=repmat(emptyPair(),0,1);
    % Non-mount bodies, including the wire, use unaltered full-model matrices.
    for b=1:numel(names)
        if b==mountIndex, continue; end
        object=bodyLabel(names{b});
        for e=1:numel(envNames)
            pairs=appendPair(pairs,object,envNames{e},envDist(b,e),'ENVIRONMENT');
        end
        for j=b+1:numel(names)
            if j==mountIndex, continue; end
            pairs=appendPair(pairs,object,bodyLabel(names{j}),selfDist(b,j),'SELF');
        end
    end
    % Attribute the mount's union to each exact V-HACD convex component.
    for k=1:numel(componentTrees)
        tree=componentTrees{k};
        [~,ed]=checkCollision(tree,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on');
        [~,sd]=checkCollision(tree,q,'IgnoreSelfCollision','off', ...
            'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
        for e=1:numel(envNames)
            pairs=appendPair(pairs,componentNames{k},envNames{e},ed(mountIndex,e),'ENVIRONMENT');
        end
        for b=1:numel(names)
            if b==mountIndex, continue; end
            pairs=appendPair(pairs,componentNames{k},bodyLabel(names{b}),sd(mountIndex,b),'SELF');
        end
    end
    collisionMask=[pairs.colliding]; colliding=pairs(collisionMask);
    % An exhaustive attribution must agree with the original hard checker.
    envPairs=strcmp({pairs.category},'ENVIRONMENT');
    selfPairs=strcmp({pairs.category},'SELF');
    assert(any(collisionMask & envPairs)==logical(any(envFlag(:))) && ...
        any(collisionMask & selfPairs)==logical(any(selfFlag(:))), ...
        'Pair attribution disagrees with original checker at candidate %d',i);
    involved=unique([{colliding.object_a} {colliding.object_b}]);
    for j=1:numel(involved), increment(objectCounts,involved{j}); end
    for j=1:numel(colliding)
        increment(pairCounts,[colliding(j).object_a ' <-> ' colliding(j).object_b]);
    end
    distances=[pairs.separation_distance_mm]; positives=distances(isfinite(distances) & distances>0);
    minPositive=NaN; if ~isempty(positives), minPositive=min(positives); end
    nearest=NaN; if ~isempty(pairs), nearest=min(distances); end
    records{i}=struct('candidate_index',originalIndices(i),'audit_index',i,'q_deg',candidates(i).q_deg, ...
        'q_rad',candidates(i).q_rad, ...
        'tcp_pose',candidates(i).tcp_pose,'axis_angle_deg',candidates(i).axis_angle_deg, ...
        'orientation_error_deg',candidates(i).orientation_error_deg, ...
        'original_first_rejection',candidates(i).rejection_reason, ...
        'environment_collision',logical(any(envFlag(:))),'self_collision',logical(any(selfFlag(:))), ...
        'collision_pair_count',numel(colliding),'nearest_distance_mm',nearest, ...
        'min_positive_clearance_mm',minPositive,'penetration_depth_mm',NaN, ...
        'penetration_depth_status','UNAVAILABLE_CHECKCOLLISION_API', ...
        'pairs',{num2cell(pairs)},'colliding_pairs',{num2cell(colliding)});
    % Exact Point1 side-position tests, separate from the switchable axis test.
    RWB=scene.T_MatBase_Workpiece(1:3,1:3).';
    gB=candidates(i).tcp_pose(1:3,1:3)*scene.wireAxisTcp(:);
    headDistance=0.020;
    if isfield(scene,'hullHeadDistanceM') && isfinite(scene.hullHeadDistanceM)
        headDistance=scene.hullHeadDistanceM;
    end
    pHeadB=candidates(i).tcp_pose(1:3,4)-headDistance*gB;
    pHeadWP=RWB*(pHeadB-scene.T_MatBase_Workpiece(1:3,4));
    records{i}.side_position_pass=pHeadWP(2)<0.230-result.options.sideMarginM && ...
        pHeadWP(3)>0.005+result.options.sideMarginM;
    records{i}.axis_pass=candidates(i).axis_angle_deg<=result.options.sideToleranceDeg;
    fprintf('COLLISION AUDIT %3d/%d: pairs=%d env=%d self=%d\n', ...
        i,numel(candidates),numel(colliding),any(envFlag(:)),any(selfFlag(:)));
end
% No penetration depth is available. This is a transparent heuristic, NOT
% a claim that these colliding candidates are feasible or minimally buried.
count=cellfun(@(r)r.collision_pair_count,records).';
clearance=cellfun(@(r)r.min_positive_clearance_mm,records).';
clearance(~isfinite(clearance))=0;
homeDistance=cellfun(@(r)norm(deg2rad(r.q_deg)-scene.qMat0),records).';
[~,order]=sortrows([count -clearance homeDistance],[1 2 3]);
rawTop=records(order(1:min(10,numel(order))));
distinct=[];
for idx=order(:).'
    duplicate=false;
    for prior=distinct
        if max(abs(records{idx}.q_deg-records{prior}.q_deg))<0.5
            duplicate=true; break;
        end
    end
    if ~duplicate, distinct(end+1)=idx; end %#ok<AGROW>
    if numel(distinct)==10, break; end
end
top=records(distinct);
report=struct('schema_version','ACCP_POINT1_COLLISION_PAIRS_v1','input_file',inputFile, ...
    'candidate_count',numel(records),'calibration_binding',scene.calibrationBinding, ...
    'current_calibration_binding',fresh.calibrationBinding,'scene_is_current',sceneIsCurrent, ...
    'audit_scene_source','Exact serialized candidate-generation scene', ...
    'execution_authorized',false, ...
    'axis_angle_tolerance_deg',result.options.sideToleranceDeg, ...
    'candidate_filter',candidateFilter,'robot_motion','NO','hard_constraints_unchanged',true, ...
    'gun_component_names',{componentNames},'environment_names',{envNames}, ...
    'penetration_depth_status','UNAVAILABLE_CHECKCOLLISION_API', ...
    'distance_definition','Intersecting solid sets have distance 0; NaN native result marks intersection. No signed penetration depth is inferred.', ...
    'count_definition','Unique rejected candidates involving each object. Exhaustive evaluation includes pairs after original early rejection. Counts across objects are not additive.', ...
    'closest_ranking','Fewest colliding pairs, then larger minimum positive clearance, then smaller Saved RealHome distance. Heuristic only; no candidate becomes feasible.', ...
    'object_counts',{mapRows(objectCounts)},'pair_counts',{mapRows(pairCounts)}, ...
    'closest_deduplication','Raw continuous q difference <0.5 deg per joint, diagnostic ranking only; no collision threshold changes.', ...
    'candidates',{records},'closest_10',{top},'closest_10_raw',{rawTop});
if strcmp(candidateFilter,'all_ik')
    d=result.firstPointDiagnostics;
    free=cellfun(@(r)~r.environment_collision && ~r.self_collision,records);
    side=cellfun(@(r)r.side_position_pass,records);
    axis=cellfun(@(r)r.axis_pass,records);
    for group=1:2
        direction=axis; if group==2, direction=true(size(axis)); end
        comparison(group)=struct('axis_constraint',group==1, ...
            'ik_calls',d.ik_calls,'ik_numeric_success',d.ik_numeric_success, ...
            'ik_pose_success',numel(records),'direction_pass',sum(direction), ...
            'collision_pass_independent',sum(free), ...
            'collision_pass_after_direction',sum(free & direction), ...
            'final_feasible_raw',sum(free & direction & side)); %#ok<AGROW>
    end
    report.ab_comparison=comparison;
    report.ab_method='Two exhaustive Point1 filter passes over the same pose-valid IK pool; B disables only axis comparison. Side-position/limits/pose tolerances/collision rules unchanged; no full-seam planning claim.';
end
if strcmp(candidateFilter,'historical_154')
    report.fixed_attachment_overlap=localTipHullEvidence(scene,components);
    writeJson(fullfile(outputDir,'fixed_attachment_overlap.json'),report.fixed_attachment_overlap);
end
writeJson(fullfile(outputDir,'collision_pairs.json'),report);
writeJson(fullfile(outputDir,'closest_10.json'),struct('ranking',report.closest_ranking,'candidates',{top}));
writeTopCsv(fullfile(outputDir,'closest_10.csv'),top);
fid=fopen(fullfile(outputDir,'object_counts.csv'),'w');
if fid<0, error('Cannot write counts'); end
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'object,candidate_count\n'); rows=report.object_counts;
for i=1:numel(rows), fprintf(fid,'"%s",%d\n',rows{i}.object,rows{i}.candidate_count); end
end

function writeTopCsv(path,records)
fid=fopen(path,'w'); if fid<0, error('Cannot write %s',path); end
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'rank,candidate_index,J1_deg,J2_deg,J3_deg,J4_deg,J5_deg,J6_deg,collision_pairs,min_noncolliding_clearance_mm,colliding_objects\n');
for i=1:numel(records)
    r=records{i}; pairs=r.colliding_pairs;
    labels=cellfun(@(p)[p.object_a ' <-> ' p.object_b],pairs,'UniformOutput',false);
    fprintf(fid,'%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.9f,"%s"\n', ...
        i,r.candidate_index,r.q_deg,r.collision_pair_count,r.min_positive_clearance_mm,strjoin(labels,'; '));
end
end

function evidence=localTipHullEvidence(scene,components)
% Point-vs-convex-halfspace evidence, explicitly NOT cylinder penetration.
manifest=jsondecode(fileread(scene.realGunGeometryManifest));
if iscell(components), c=components{4}; else, c=components(4); end
v=double(c.vertices_m); faces=convhulln(v); centre=mean(v,1);
planes=zeros(size(faces,1),4);
for i=1:size(faces,1)
    p=v(faces(i,:),:); n=cross(p(2,:)-p(1,:),p(3,:)-p(1,:)); n=n/norm(n);
    offset=-dot(n,p(1,:));
    if dot(n,centre)+offset>0, n=-n; offset=-offset; end
    planes(i,:)=[n offset];
end
tip=double(manifest.wire.tip_source(:));
signed=planes(:,1:3)*tip+planes(:,4);
evidence=struct('frame','STL_SOURCE','component','GunCollisionHull/component_04', ...
    'wire_tip_source_m',tip.','wire_tip_inside_component',all(signed<=0), ...
    'maximum_outward_halfspace_distance_mm',max(signed)*1000, ...
    'wire_length_mm',manifest.wire.length_m*1000, ...
    'interpretation','Fixed gun and wire share the mount; tip inside hull explains pose-independent tool-internal overlap. Halfspace distance is NOT cylinder penetration depth.', ...
    'collision_rules_changed',false);
end

function s=emptyPair()
s=struct('object_a','','object_b','','category','','colliding',false, ...
    'separation_distance_mm',NaN,'native_separation_distance_m',NaN, ...
    'penetration_depth_mm',NaN,'penetration_depth_status','UNAVAILABLE_CHECKCOLLISION_API');
end
function pairs=appendPair(pairs,a,b,d,category)
if isinf(d), return; end % Skipped/no collision geometry: not evaluated.
s=emptyPair(); s.object_a=char(a); s.object_b=char(b); s.category=category;
s.colliding=isnan(d); s.native_separation_distance_m=d;
if s.colliding, s.separation_distance_mm=0; else, s.separation_distance_mm=1000*d; end
pairs(end+1,1)=s;
end
function label=bodyLabel(name)
label=name;
if strcmp(name,'WeldingGunWire'), label='WireCollision'; end
end
function increment(map,key)
if isKey(map,key), map(key)=map(key)+1; else, map(key)=1; end
end
function rows=mapRows(map)
ks=keys(map); vals=cellfun(@(k)map(k),ks); [~,order]=sort(vals,'descend');
rows=cell(1,numel(order));
for i=1:numel(order), k=ks{order(i)}; rows{i}=struct('object',k,'candidate_count',map(k)); end
end
function writeJson(path,data)
fid=fopen(path,'w'); if fid<0, error('Cannot write %s',path); end
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(data));
end
