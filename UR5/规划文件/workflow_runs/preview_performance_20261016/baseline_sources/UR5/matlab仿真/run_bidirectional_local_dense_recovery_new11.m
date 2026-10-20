function report=run_bidirectional_local_dense_recovery_new11()
%RUN_BIDIRECTIONAL_LOCAL_DENSE_RECOVERY_NEW11
% Run the frozen BLDR diagnostic against the 11 new100 failure cases.
% This is a diagnostic/fallback feasibility test only; it does not modify
% the normal ACCP planner.

root=fileparts(mfilename('fullpath'));
opts=struct( ...
    'denseReportFile',fullfile(root,'local_dense_connectivity_new11.mat'), ...
    'outputMat',fullfile(root,'bidirectional_local_dense_recovery_new11.mat'), ...
    'outputCsv',fullfile(root,'bidirectional_local_dense_recovery_new11.csv'));
report=run_bidirectional_local_dense_recovery(opts);
end
