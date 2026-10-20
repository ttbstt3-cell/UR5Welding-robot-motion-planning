function point = sampleUniform(config)
% sampleUniform  Uniformly sample one point inside config.mapBounds.

bounds = config.mapBounds;
point = [
    bounds(1)+rand*(bounds(2)-bounds(1)), ...
    bounds(3)+rand*(bounds(4)-bounds(3))
];
point = reshape(point,1,2);

end
