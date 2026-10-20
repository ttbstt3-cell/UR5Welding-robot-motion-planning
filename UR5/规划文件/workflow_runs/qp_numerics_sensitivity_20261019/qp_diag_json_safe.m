function value=qp_diag_json_safe(value)
% MATLAB solver statistics may be sparse scalars; preserve numeric values.
if issparse(value), value=full(value); end
if isstruct(value)
    for k=1:numel(value)
        for field=fieldnames(value).', value(k).(field{1})=qp_diag_json_safe(value(k).(field{1})); end
    end
elseif iscell(value)
    value=cellfun(@qp_diag_json_safe,value,'UniformOutput',false);
end
end
