function status=hmi_quadprog_exit_status(flag)
% Do not label a budget stop as proof of infeasibility. Keep raw solver data.
if flag>0, status='CONVERGED';
elseif flag==0, status='ITERATION_LIMIT';
elseif flag==-2, status='QP_INFEASIBLE';
else, status='SOLVER_FAILURE'; end
end
