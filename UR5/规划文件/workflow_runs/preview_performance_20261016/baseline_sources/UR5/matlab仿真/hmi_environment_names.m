function names=hmi_environment_names(scene)
% Stable names for every environment collision object, in collision order.
% Historical frozen scenes have only the four workpiece proxies. REAL
% scenes publish explicit names when independent surroundings are appended.
count=numel(scene.environment);
if isfield(scene,'environmentNames') && numel(scene.environmentNames)==count
    names=cellstr(string(scene.environmentNames));
elseif isfield(scene,'environmentProxyInfo') && numel(scene.environmentProxyInfo)==count
    names=cellstr(string({scene.environmentProxyInfo.name}));
elseif isfield(scene,'workpieceProxyInfo') && numel(scene.workpieceProxyInfo)==count
    names=cellstr(string({scene.workpieceProxyInfo.name}));
else
    error('REAL_ENVIRONMENT:Names','Environment collision geometry has no complete object-name mapping');
end
names=reshape(names,1,[]);
assert(numel(unique(names))==count && all(~cellfun(@isempty,names)),...
    'REAL_ENVIRONMENT:Names','Environment names must be nonempty and unique');
end
