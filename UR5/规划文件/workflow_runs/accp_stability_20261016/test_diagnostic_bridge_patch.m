function result=test_diagnostic_bridge_patch(source,out)
root='/home/ttbstt/jixiebi'; here=fullfile(root,'UR5','matlab仿真');
addpath(here); addpath(genpath(fullfile(root,'RRT')));
addpath(fullfile(fileparts(mfilename('fullpath')),'diagnostic_patch','UR5','matlab仿真'),'-begin');
if ~isfolder(out), mkdir(out); end
s=load(source,'result'); options=s.result.options;
options.outputFile=fullfile(out,'preview_accp.mat');
diary(fullfile(out,'matlab.log'));
result=replan_weld_actual_axis_adaptive_core(options);
fprintf('DIAGNOSTIC_PATCH_RESULT configuration=%d geometry=%d nodes=%d\n',...
    result.configurationComplete,result.adaptiveCorrectionComplete,size(result.qPathExecution,1));
diary off;
end
