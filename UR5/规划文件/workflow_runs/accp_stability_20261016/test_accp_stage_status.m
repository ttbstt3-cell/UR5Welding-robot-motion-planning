function report=test_accp_stage_status(failedFile,passedFile,out)
addpath('/home/ttbstt/jixiebi/UR5/matlab仿真');
a=load(failedFile,'result'); b=load(passedFile,'result');
s=hmi_accp_preview_status(a.result); assert(strcmp(s.classification,'PRECISION_FAILURE') && isempty(s.failed_point));
t=hmi_accp_preview_status(b.result); assert(t.geometry_complete && strcmp(t.classification,'PASS'));
x=rmfield(b.result,'qPath'); s=hmi_accp_preview_status(x); assert(strcmp(s.classification,'OUTPUT_SCHEMA_FAILURE'));
x=b.result; x.qPath=zeros(0,6); s=hmi_accp_preview_status(x); assert(~s.geometry_complete);
x=a.result; x.cartesianRefinement.finalTcpMetrics.nodeCollisionFree=false;
s=hmi_accp_preview_status(x); assert(strcmp(s.classification,'COLLISION_FAILURE'));
x=a.result; x.configurationComplete=false; x.stateBank{1}=[]; x.firstPointDiagnostics.ik_success=0;
s=hmi_accp_preview_status(x); assert(strcmp(s.classification,'IK_FAILURE'));
x.firstPointDiagnostics.ik_success=1;
s=hmi_accp_preview_status(x); assert(strcmp(s.classification,'NO_CONNECTED_CANDIDATE'));
report=struct('pass',true,'classifications',{{'PASS','PRECISION_FAILURE','COLLISION_FAILURE',...
    'IK_FAILURE','NO_CONNECTED_CANDIDATE','OUTPUT_SCHEMA_FAILURE'}},...
    'partial_success_not_promoted',true,'recovered_layer_not_reported_as_final_failure',true,...
    'executionReady',false,'robot_motion','NO');
f=fopen(out,'w'); c=onCleanup(@()fclose(f)); fwrite(f,jsonencode(report,'PrettyPrint',true),'char');
end
