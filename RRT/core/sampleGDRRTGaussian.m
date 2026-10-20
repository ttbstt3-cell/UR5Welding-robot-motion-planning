function [worldPoint,info] = sampleGDRRTGaussian(frame,params)
% sampleGDRRTGaussian  Sample the Eq.(4)--Eq.(14) Gaussian mixture.

required = {'mu11','mu12','sigma11','sigma12','k1','k2', ...
    'mu21','mu22','sigma21','sigma22','l1','l2'};
for i = 1:numel(required)
    if ~isfield(params,required{i})
        error('params is missing field %s.',required{i});
    end
end

if rand < params.k1/(params.k1+params.k2)
    localX = params.mu11+params.sigma11*randn;
    info.xComponent = 1;
else
    localX = params.mu12+params.sigma12*randn;
    info.xComponent = 2;
end

if rand < params.l1
    localY = params.mu21+params.sigma21*randn;
    info.yComponent = 1;
else
    localY = params.mu22+params.sigma22*randn;
    info.yComponent = 2;
end

info.localPoint = [localX localY];
worldPoint = gaussianLocalToWorld(frame,info.localPoint);
worldPoint = reshape(worldPoint,1,2);

end
