function report=verify_hmi_preview_native_reaudit(cachedDirectory,nativeDirectory,outputFile)
% Independent full-grid native rerun must agree with cached audit per frame.
a=jsondecode(fileread(fullfile(cachedDirectory,'preview_display_audit.json')));
b=jsondecode(fileread(fullfile(nativeDirectory,'preview_display_audit.json')));
assert(a.collision_sample_count==b.collision_sample_count && ...
    a.collision_check_pass==b.collision_check_pass && a.collision_hit_count==b.collision_hit_count);
assert(isempty(a.collision_pairs) && isempty(b.collision_pairs));
assert(abs(a.minimum_positive_distance_mm-b.minimum_positive_distance_mm)<1e-10);
af=toCells(a.frames); bf=toCells(b.frames); assert(numel(af)==numel(bf));
maxGapDelta=0;
for k=1:numel(af)
    x=af{k}; y=bf{k};
    assert(isequaln(x.q_rad,y.q_rad) && strcmp(x.collision_status,y.collision_status));
    assert(isequal(string(x.collision_pairs),string(y.collision_pairs)));
    assert(isequal(string(x.outgoing_edge_pairs),string(y.outgoing_edge_pairs)));
    assert(strcmp(x.min_gap_pair,y.min_gap_pair));
    if ~isempty(x.min_positive_gap_mm)
        maxGapDelta=max(maxGapDelta,abs(x.min_positive_gap_mm-y.min_positive_gap_mm));
    else
        assert(isempty(y.min_positive_gap_mm));
    end
end
assert(maxGapDelta<1e-10);
report=struct('pass',true,'nodes',numel(af),'samples',a.collision_sample_count,...
    'collision_sample_count_identical',true,'collision_pairs_identical',true,...
    'frame_statuses_identical',true,'nearest_pair_names_identical',true,...
    'max_frame_gap_difference_mm',maxGapDelta,...
    'minimum_positive_gap_mm',a.minimum_positive_distance_mm,...
    'executionReady',false,'robot_motion','NO');
fid=fopen(outputFile,'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(report,'PrettyPrint',true),'char');
end
function c=toCells(value)
if iscell(value), c=value; else, c=num2cell(value); end
end
