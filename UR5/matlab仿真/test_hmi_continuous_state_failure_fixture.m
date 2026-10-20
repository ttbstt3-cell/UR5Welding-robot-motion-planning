function requestFile=test_hmi_continuous_state_failure_fixture(priorRequestFile,out)
% Intentionally invalid TEST COPY. Never changes the user's preview or model.
req=jsondecode(fileread(priorRequestFile)); raw=load(req.rawFile);
if ~isfolder(out), mkdir(out); end
raw.segments{1}.qPath(2,1)=raw.segments{1}.qPath(2,1)+20;
badRaw=fullfile(out,'invalid_limit_test_preview.mat'); save(badRaw,'-struct','raw','-v7');
req.rawFile=badRaw; req.raw_sha256=hmi_workflow_file_hash(badRaw);
req.outputDir=fullfile(out,'matlab_failure');
requestFile=fullfile(out,'invalid_request.json');
fid=fopen(requestFile,'w'); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(req,'PrettyPrint',true),'char');
fprintf('Failure fixture only: one approach node intentionally violates limits. Source artifact unchanged.\n');
end
