function report = profile_weld_actual_axis_gao_ar_rrtstar()
%PROFILE_WELD_ACTUAL_AXIS_GAO_AR_RRTSTAR
% One reproducible cold-start run for the Gao-style comparison baseline.
% No previous qPath or state graph is loaded.

matlabDir = fileparts(mfilename("fullpath"));
seed = 20260924;
opts = struct( ...
    "randomSeed",seed, ...
    "maxIterations",10000, ...
    "timeBudgetSec",120, ...
    "goalBias",0.20, ...
    "gaussianSigmaTravelDeg",6, ...
    "gaussianSigmaRollDeg",12, ...
    "samplesPerLayer",12, ...
    "maxStatesPerLayer",12, ...
    "maxIKSeeds",8, ...
    "neighborParents",8, ...
    "edgeResolutionDeg",3, ...
    "outputFile","weld_actual_axis_gao_ar_rrtstar.mat");

wallTimer = tic;
result = replan_weld_actual_axis_gao_ar_rrtstar(opts);
wallElapsed = toc(wallTimer);

report = struct();
report.algorithm = "Gao-AR-RRT*";
report.randomSeed = seed;
report.algorithmTimeSec = result.algorithmElapsedSec;
report.wallTimeSec = wallElapsed;
report.firstFeasibleTimeSec = result.firstFeasibleTimeSec;
report.iterations = result.stats.iterations;
report.ikCalls = result.stats.ikCalls;
report.ikPoseSuccess = result.stats.ikPoseSuccess;
report.ikSuccessRatePct = 100*result.stats.ikPoseSuccess/max(1,result.stats.ikCalls);
report.acceptedStates = result.stats.acceptedStates + result.stats.acceptedStartStates;
report.edgeTests = result.stats.edgeTests;
report.edgeCollisionFree = result.stats.edgeCollisionFree;
report.complete = result.complete;
report.nodeCollisionFree = result.nodeCollisionFree;
report.edgeCollisionFreePass = result.edgeCollisionFree;
report.maxJointJumpDeg = result.maxJointJumpDeg;
report.maxNodeTcpErrorMm = result.tcpMetrics.maxNodePositionErrorMm;
report.maxSegmentTcpErrorMm = result.tcpMetrics.maxSegmentPositionErrorMm;
report.maxSegmentTcpOrientationDeg = result.tcpMetrics.maxSegmentOrientationErrorDeg;
report.outputFile = result.outputFile;

reportFile = fullfile(matlabDir,"weld_gao_ar_rrtstar_report.txt");
fid = fopen(reportFile,"w");
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,"GAO-AR-RRT* COLD-START COMPARISON\n");
fprintf(fid,"================================\n");
fprintf(fid,"seed = %d\n",seed);
fprintf(fid,"algorithm_time_sec = %.9f\n",report.algorithmTimeSec);
fprintf(fid,"wall_time_sec = %.9f\n",report.wallTimeSec);
fprintf(fid,"first_feasible_time_sec = %.9f\n",report.firstFeasibleTimeSec);
fprintf(fid,"iterations = %d\n",report.iterations);
fprintf(fid,"ik_calls = %d\n",report.ikCalls);
fprintf(fid,"ik_pose_success = %d\n",report.ikPoseSuccess);
fprintf(fid,"ik_success_rate_pct = %.6f\n",report.ikSuccessRatePct);
fprintf(fid,"accepted_states = %d\n",report.acceptedStates);
fprintf(fid,"edge_tests = %d\n",report.edgeTests);
fprintf(fid,"edge_collision_free = %d\n",report.edgeCollisionFree);
fprintf(fid,"complete_80_point_path = %d\n",report.complete);
fprintf(fid,"node_collision_free = %d\n",report.nodeCollisionFree);
fprintf(fid,"edge_collision_free_pass = %d\n",report.edgeCollisionFreePass);
fprintf(fid,"max_joint_jump_deg = %.9f\n",report.maxJointJumpDeg);
fprintf(fid,"max_node_tcp_error_mm = %.9f\n",report.maxNodeTcpErrorMm);
fprintf(fid,"max_segment_tcp_error_mm = %.9f\n",report.maxSegmentTcpErrorMm);
fprintf(fid,"max_segment_tcp_orientation_deg = %.9f\n",report.maxSegmentTcpOrientationDeg);
fprintf(fid,"result_mat = %s\n",report.outputFile);
fprintf(fid,"\nThis is a single cold-start run on the same scene/model.\n");
fprintf(fid,"It is not a direct numerical comparison with Gao et al. paper times.\n");
fprintf(fid,"Run repeated seeds for mean/std/success-rate statistics.\n");
fprintf(fid,"Stable continuation planner was not modified or loaded as a result.\n");
fprintf("Saved report: %s\n",reportFile);
end
