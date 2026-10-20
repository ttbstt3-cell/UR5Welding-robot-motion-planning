function report=run_preperformance_case(source,out)
root='/home/ttbstt/jixiebi'; addpath(fullfile(root,'UR5','matlab仿真'));
addpath(genpath(fullfile(root,'RRT')));
addpath(fullfile(fileparts(mfilename('fullpath')),'preperformance_source'),'-begin');
if ~isfolder(out), mkdir(out); end
s=load(source,'result'); options=s.result.options;
options.outputFile=fullfile(out,'preview_accp.mat'); options.performanceExactQueryCache=false;
diary(fullfile(out,'matlab.log')); clock=tic;
r=replan_weld_actual_axis_adaptive_core(options);
report=struct('scope','pre-performance archived core + refinement; same failed input/seed; no timing stage on geometry failure',...
    'wall_sec',toc(clock),'configuration_complete',r.configurationComplete,'geometry_complete',r.adaptiveCorrectionComplete,...
    'max_tcp_error_mm',r.tcpMetrics.maxSegmentPositionErrorMm,'sparse_nodes',size(r.qPathInitial,1),...
    'failed_refinement_edges',find(~[r.adaptiveSegmentReport.correctionAccepted]),'executionReady',false);
f=fopen(fullfile(out,'summary.json'),'w'); fwrite(f,jsonencode(report,'PrettyPrint',true),'char'); fclose(f);
fprintf('PREPERFORMANCE_CASE geometry=%d time=%.3f\n',r.adaptiveCorrectionComplete,report.wall_sec); diary off;
end
