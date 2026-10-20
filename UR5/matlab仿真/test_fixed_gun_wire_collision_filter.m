function report=test_fixed_gun_wire_collision_filter(inputFile,outputDir)
% Identical saved 154 Point1 candidates. Do not publish/execute or change IK.
S=load(inputFile); scene=S.result.scene; robot=scene.robot;
before=scene.selfCollisionSkipPairs;
after=ignore_fixed_gun_wire_self_collision(robot,before);
assert(size(after,1)==size(before,1)+1);
assert(isequal(after(1:end-1,:),before));
names=robot.BodyNames; m=find(strcmp(names,'WeldingGunMount')); w=find(strcmp(names,'WeldingGunWire'));
mask=true(numel(names)+1); mask(m,w)=false; mask(w,m)=false;
d=S.result.firstPointDiagnostics.candidates; assert(numel(d)==154);
counts=zeros(2,5); records=cell(1,numel(d));
for i=1:numel(d)
    q=d(i).q_rad;
    [env,ed]=checkCollision(robot,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on');
    [cb,db]=checkCollision(robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',before,'Exhaustive','on');
    [ca,da]=checkCollision(robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',after,'Exhaustive','on');
    assert(isequaln(db(mask),da(mask)),'External collision pairs changed');
    assert(isinf(da(m,w)),'Attachment pair was not excluded');
    gunEnv=any(isnan(ed(m,:))); wireEnv=any(isnan(ed(w,:)));
    acceptedBefore=~any(env(:)) && ~any(cb(:)); acceptedAfter=~any(env(:)) && ~any(ca(:));
    counts(1,:)=counts(1,:)+[isnan(db(m,w)),any(env(:)),gunEnv,wireEnv,acceptedBefore];
    counts(2,:)=counts(2,:)+[isnan(da(m,w)),any(env(:)),gunEnv,wireEnv,acceptedAfter];
    records{i}=struct('candidate_index',i,'q_deg',d(i).q_deg,'environment_collision',any(env(:)), ...
        'self_before',any(cb(:)),'self_after',any(ca(:)),'accepted_after',acceptedAfter);
end
if ~isfolder(outputDir), mkdir(outputDir); end
report=struct('columns',{{'tool_internal','workpiece','gun_vs_workpiece','wire_vs_workpiece','final_raw_candidates'}}, ...
    'before',counts(1,:),'after',counts(2,:),'axis_angle_tolerance_deg',20, ...
    'added_skip_pair',{after(end,:)},'external_self_distances_identical',true, ...
    'candidate_count',numel(d),'records',{records},'robot_motion','NO');
report.gun_self_rejected_after_environment_before=sum(cellfun(@(x)~x.environment_collision && x.self_before,records));
report.gun_self_rejected_after_environment_after=sum(cellfun(@(x)~x.environment_collision && x.self_after,records));
fid=fopen(fullfile(outputDir,'filter_comparison.json'),'w'); if fid<0, error('Cannot write report'); end
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true)); disp(counts);
% Repeat actual cold planner using the exact prior options, not edited poses.
opts=S.result.options; opts.outputFile=fullfile(outputDir,'filtered_core.mat');
opts.publish=false; opts.animate=false;
opts.configurationOnlyDiagnostic=true;
r=replan_weld_actual_axis_adaptive_core(opts);
fprintf('FILTERED POINT1 connected unique candidates: %d, retained: %d\n', ...
    r.candidateSelectionDiagnostics{1}.connected_candidate_count,numel(r.stateBank{1}));
end
