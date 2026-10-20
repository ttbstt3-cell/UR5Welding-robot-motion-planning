function state = updateGDRRTParameters(pcf,baseStep)
% updateGDRRTParameters  Implement the paper's ParamUpdate rule.
%
% pcf is collision frequency, not collision-free probability.  Because the
% planner updates it after ten trials, pcf takes values 0, 0.1, ..., 1.0.

if ~isscalar(pcf) || ~isfinite(pcf) || pcf < 0 || pcf > 1
    error('pcf must be a finite scalar in [0, 1].');
end
if ~isscalar(baseStep) || ~isfinite(baseStep) || baseStep <= 0
    error('baseStep must be a finite positive scalar.');
end

if pcf <= 0.2
    state.alpha = false;
    state.currentStep = 2*baseStep;
    state.currentDepth = 1;
elseif pcf <= 0.5
    state.alpha = false;
    state.currentStep = baseStep;
    state.currentDepth = 2;
else
    % With ten-sample windows, pcf > 0.5 is exactly pcf >= 0.6.
    state.alpha = true;
    state.currentStep = baseStep;
    state.currentDepth = 3;
end

state.pcf = pcf;
state.baseStep = baseStep;

end
