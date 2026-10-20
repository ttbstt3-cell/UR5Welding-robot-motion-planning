function result=hmi_planning_dependency_changes(before,after)
% Compare immutable source identities around scene loading/version binding.
paths=sort(unique([{before.files.path},{after.files.path}])); changed={};
for k=1:numel(paths)
    path=paths{k}; ib=find(strcmp({before.files.path},path),1);
    ia=find(strcmp({after.files.path},path),1);
    if isempty(ib) || isempty(ia) || ...
            before.files(ib).exists~=after.files(ia).exists || ...
            ~strcmp(before.files(ib).sha256,after.files(ia).sha256)
        changed{end+1}=path; %#ok<AGROW>
    end
end
result=struct('pass',isempty(changed),'status','PASS','changed_files',{changed},...
    'before_sha256',before.content_sha256,'after_sha256',after.content_sha256,...
    'scope','Source-file content stability around scene load; no geometry or planner changes');
if ~result.pass, result.status='STALE'; end
end
