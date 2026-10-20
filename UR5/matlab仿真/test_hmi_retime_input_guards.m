function result=test_hmi_retime_input_guards(requestFile,outputRoot)
% Invalid requests must fail before collision audit/export. Offline only.
if ~isfolder(outputRoot), mkdir(outputRoot); end
base=jsondecode(fileread(requestFile)); names={};
for k=1:5
    r=base; r.outputDir=fullfile(outputRoot,sprintf('rejected_%d',k));
    switch k
        case 1, r.source_sha256=repmat('0',1,64); name='source_SHA';
        case 2, r.raw_sha256=repmat('0',1,64); name='raw_SHA';
        case 3, r.offline_speed_percent=101; name='overspeed_percent';
        case 4
            old=jsondecode(fileread(base.sourceReport)); old.offline_checks.collision=false;
            r.sourceReport=fullfile(outputRoot,'invalid_report.json'); write(r.sourceReport,old);
            r.source_report_sha256=hmi_workflow_file_hash(r.sourceReport); name='partial_source_PASS';
        case 5
            old=jsondecode(fileread(base.sourceRequest)); old.rawFile='WRONG_PREVIEW';
            r.sourceRequest=fullfile(outputRoot,'invalid_request.json'); write(r.sourceRequest,old);
            r.source_request_sha256=hmi_workflow_file_hash(r.sourceRequest); name='wrong_source_preview';
    end
    report=retime_hmi_continuous_request(r);
    assert(~report.success && ~report.continuous_planning_complete && ~report.executionReady);
    assert(~isfield(report,'controller_validation'),'Invalid input reached dense collision audit');
    assert(~isfile(fullfile(r.outputDir,'offline_optimized_trajectory.mat')),...
        'Invalid request exported an optimized trajectory');
    names{end+1}=struct('name',name,'identifier',report.failure_identifier,...
        'reason',report.failure_reason,'no_output',true); %#ok<AGROW>
end
result=struct('pass',true,'cases',{names},'robot_motion','NO','executionReady',false);
write(fullfile(outputRoot,'guard_results.json'),result);
fprintf('RETIME_INPUT_GUARDS_PASS\n');
end
function write(file,value)
fid=fopen(file,'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(value,'PrettyPrint',true),'char');
end
