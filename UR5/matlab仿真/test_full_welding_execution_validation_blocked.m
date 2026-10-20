function test_full_welding_execution_validation_blocked()
% Regression: absent/currently-unbound segments can never become executable.
approach=struct('available',false,'reason','test missing path');
weld=struct('available',false,'reason','test ACCP path missing');
exit=struct('available',false,'reason','test missing path');
r=validate_full_welding_execution(approach,weld,exit,[],struct(),struct());
assert(~r.executionReady);
assert(~r.trajectory_exported);
assert(strcmp(r.validation_status,'BLOCKED'));
assert(any(strcmp(r.classification,'MISSING_CURRENT_SEGMENT_PATH')));
assert(any(strcmp(r.classification,'STALE_OR_INCOMPLETE_VERSION_BINDING')));
assert(any(strcmp(r.classification,'FULL_CONTROLLER_INTERPOLATION_NOT_RUN')));
assert(any(strcmp(r.classification,'FULL_TASK_COLLISION_NOT_RUN')));
assert(strcmp(r.splice_checks{1}.status,'NOT_VERIFIED'));
assert(strcmp(r.splice_checks{2}.status,'NOT_VERIFIED'));
fprintf('FULL_WELDING_VALIDATION_MISSING_SEGMENT_BLOCK_TEST_PASS\n');
end
