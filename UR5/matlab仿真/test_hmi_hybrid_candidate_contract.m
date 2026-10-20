function report=test_hmi_hybrid_candidate_contract(base,scene,hybrid)
% Tests use genuine audited C; mutated solver/audit data below are FIXTURES.
assert(~hybrid.hybridCandidateFound,'Use an actual stopped-D result for fallback regression');
[V,s]=hmi_select_hybrid_velocity_candidate(hybrid,base,scene);
assert(isequal(V,base.velocities{4}) && s.nonoptimal_fallback && ~s.executionReady);
assert(strcmp(s.source,'C_ACCURACY_FALLBACK') && s.requires_full_task_validation);
assert(strcmp(hmi_quadprog_exit_status(0),'ITERATION_LIMIT'));
assert(strcmp(hmi_quadprog_exit_status(-2),'QP_INFEASIBLE'));
assert(strcmp(hmi_quadprog_exit_status(-6),'SOLVER_FAILURE'));
assert(strcmp(hmi_quadprog_exit_status(1),'CONVERGED'));
bad={}; names={};
b=hybrid; b.qPath(1,1)=b.qPath(1,1)+1e-7; push(b,'q changed');
b=hybrid; b.tcpPosePath{1}(1,4)=b.tcpPosePath{1}(1,4)+1e-6; push(b,'TCP changed');
b=hybrid; b.timeFromStart(end)=b.timeFromStart(end)+1e-6; push(b,'time changed');
b=hybrid; b.calibrationBinding.test_bad_binding=true; push(b,'version mismatch');
b=hybrid; b.executionReady=true; push(b,'execution claim');
b=hybrid; b.velocities{4}(1,1)=b.velocities{4}(1,1)+1e-7; push(b,'stopped iterate substituted for C');
b=hybrid; b.velocities{4}(1,1)=NaN; push(b,'nonfinite candidate');
for f={'complete','collision_pass','joint_limits_pass','analytic_derivative_and_position_extrema_pass'}
    b=hybrid; b.audits{3}.(f{1})=false; push(b,['failed ' f{1}]);
end
for f={'max_tcp_contour_error_mm','max_tcp_time_sync_error_mm','max_tcp_orientation_error_deg'}
    b=hybrid; b.audits{3}.(f{1})=.500001; push(b,['precision exceeded ' f{1}]);
end
b=hybrid; b.audits{3}.max_tcp_time_sync_error_mm=NaN; push(b,'NaN audit');
b=hybrid; b.audits{3}.sample_count=3000; push(b,'insufficient samples');
b=hybrid; b.audits{3}.interpolation_mode='quintic'; push(b,'wrong interpolation');
b=hybrid; b.hybridCandidateFound=true; b.audits{4}.complete=false; push(b,'invalid D cannot masquerade as C');
for k=1:numel(bad)
    rejected=false;
    try, hmi_select_hybrid_velocity_candidate(bad{k},base,scene);
    catch err, rejected=startsWith(err.identifier,'PREPARE:'); end
    assert(rejected,'Invalid fixture accepted: %s',names{k});
end
bb=base; bb.audits{4}.complete=false; rejected=false;
try, hmi_select_hybrid_velocity_candidate(hybrid,bb,scene);
catch err, rejected=strcmp(err.identifier,'PREPARE:HybridCandidateInvalid'); end
assert(rejected,'Invalid original C accepted'); names{end+1}='original C failed';
% D-success selection contract fixture only, not a claim of a new D solve.
d=hybrid; d.hybridCandidateFound=true;
[~,ds]=hmi_select_hybrid_velocity_candidate(d,base,scene);
assert(~ds.nonoptimal_fallback && strcmp(ds.source,'D_HYBRID') && ~ds.executionReady);
report=struct('pass',true,'rejected_cases',{names},'fallback_C_exact',true,...
    'D_success_branch_fixture_pass',true,'solver_exit_classification_pass',true,...
    'executionReady',false,'robot_motion','NO');
    function push(value,name)
        bad{end+1}=value; names{end+1}=name;
    end
end
