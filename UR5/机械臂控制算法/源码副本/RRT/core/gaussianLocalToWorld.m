function worldPoint = gaussianLocalToWorld(frame,localPoint)
% gaussianLocalToWorld  Transform row-vector Gaussian points to world frame.

if ~isstruct(frame) || ~isfield(frame,'origin') || ~isfield(frame,'R')
    error('frame must be returned by computeGaussianFrame.');
end
localPoint = reshape(localPoint,[],2);
if any(~isfinite(localPoint(:)))
    error('localPoint must contain finite two-dimensional points.');
end

worldPoint = localPoint*frame.R.'+frame.origin(:).';

end
