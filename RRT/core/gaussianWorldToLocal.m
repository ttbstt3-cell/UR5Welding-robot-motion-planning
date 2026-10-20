function localPoint = gaussianWorldToLocal(frame,worldPoint)
% gaussianWorldToLocal  Transform row-vector world points to Gaussian frame.

if ~isstruct(frame) || ~isfield(frame,'origin') || ~isfield(frame,'R')
    error('frame must be returned by computeGaussianFrame.');
end
worldPoint = reshape(worldPoint,[],2);
if any(~isfinite(worldPoint(:)))
    error('worldPoint must contain finite two-dimensional points.');
end

localPoint = (worldPoint-frame.origin(:).')*frame.R;

end
