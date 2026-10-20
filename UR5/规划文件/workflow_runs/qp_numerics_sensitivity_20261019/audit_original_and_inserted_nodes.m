function audit_original_and_inserted_nodes()
here=fileparts(mfilename('fullpath')); d=load(fullfile(here,'last_input.mat'),'maps','Q','P','t','labels');
a=jsondecode(fileread(fullfile(here,'fixed_node_audit.json')));
original=unique([d.maps{:}]); inserted=setdiff(1:size(d.Q,1),original);
bad=find(a.node_errors(:,1)>.5);
assert(all(ismember(bad,inserted)),'An original node is unexpectedly above the formal gate');
a.original_node_count=numel(original); a.inserted_node_count=numel(inserted);
a.original_node_position_error_max_mm=max(a.node_errors(original,1));
a.original_node_orientation_error_max_deg=max(a.node_errors(original,2));
a.all_bad_nodes_are_inserted=all(ismember(bad,inserted));
fid=fopen(fullfile(here,'fixed_node_audit.json'),'w'); c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(a,'PrettyPrint',true),'char');
fprintf('NODE_INVARIANT original_max_mm=%.9f inserted_bad=%d\n',a.original_node_position_error_max_mm,numel(bad));
end
