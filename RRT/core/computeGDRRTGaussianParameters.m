function params = computeGDRRTGaussianParameters(localNodes,currentStep,pcf)
% computeGDRRTGaussianParameters  Evaluate GD-RRT* Eq.(4)--Eq.(14).

localNodes = reshape(localNodes,[],2);
if isempty(localNodes) || any(~isfinite(localNodes(:)))
    error('localNodes must contain at least one finite 2-D point.');
end
if ~isscalar(currentStep) || ~isfinite(currentStep) || currentStep <= 0
    error('currentStep must be a finite positive scalar.');
end
if ~isscalar(pcf) || ~isfinite(pcf) || pcf < 0 || pcf > 1
    error('pcf must be a finite scalar in [0, 1].');
end

params.xgmax = max(localNodes(:,1));
params.xgmin = min(localNodes(:,1));
params.ygmax = max(localNodes(:,2));

% Eq.(5)--Eq.(8): symmetric two-component Gaussian mixture in x.
params.sigma11 = 0.5*currentStep*exp(pcf);
params.sigma12 = params.sigma11;
params.mu11 = params.xgmax+params.sigma11;
params.mu12 = params.xgmin-params.sigma12;
params.k1 = 0.5;
params.k2 = 0.5;

% Eq.(9)--Eq.(14): forward/backward mixture in y.
params.sigma21 = currentStep*exp(pcf);
params.sigma22 = params.sigma21;
params.mu21 = params.ygmax+params.sigma21;
params.mu22 = params.ygmax-params.sigma22;
params.l1 = 1-pcf^2;
params.l2 = pcf^2;

params.currentStep = currentStep;
params.pcf = pcf;
params.localNodeCount = size(localNodes,1);

end
