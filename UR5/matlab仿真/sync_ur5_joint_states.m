clear;
clc;
close all;

%% ============================================================
% MAIN_UR5_PLANNING_SCENE
%
% MATLAB 路径规划主场景
%
% 已冻结并通过数值验收：
%   1. qOffset
%   2. T_CopBase_MatBase
%   3. T_tool0_mount
%   4. T_mount_tcp
%
% 本脚本不重新标定以上参数。
%
% 本阶段增加：
%   - UR5 原有碰撞模型
%   - GunCollisionHull（优先）或 5 个 WeldingGun collision proxies（回退）
%   - WeldTCP
%   - 4 个 Workpiece collision proxies
%   - 初始碰撞检测
%
% MATLAB 后续 RRT 直接使用：
%
%   scene.robot
%   scene.environment
%   scene.qOffset
%   scene.T_CopBase_MatBase
%   scene.tcpBodyName
%
%% ============================================================


%% ============================================================
% 0. 文件路径
%% ============================================================

% 本脚本位于 UR5/matlab仿真；场景 JSON 和标定文件位于上一级 UR5。
matlabDir = fileparts(mfilename("fullpath"));
if isempty(matlabDir)
    matlabDir = pwd;
end
rootDir = fileparts(matlabDir);

calibrationFile = fullfile( ...
    rootDir, ...
    "ur5_tool_calibration.mat");

geometryFile = fullfile( ...
    rootDir, ...
    "scene_geometry.json");

% Replacement welding-gun geometry is independent from the frozen scene.
% Staged data is reported for diagnostics but cannot silently replace the
% accepted GunCollisionHull until the flange transform/collision/wire data
% have been explicitly confirmed.
realGun = load_real_gun_geometry(rootDir);
% This entry point is the FROZEN SIMULATION baseline. Real geometry must be
% applied explicitly by apply_real_gun_geometry to a calibrated URDF scene.
% An ACTIVE real manifest must never silently change the frozen scene.
realGun.active = false;


if ~isfile(calibrationFile)
    error( ...
        "找不到标定文件：\n%s", ...
        calibrationFile);
end

if ~isfile(geometryFile)
    error( ...
        "找不到场景几何文件：\n%s", ...
        geometryFile);
end


%% ============================================================
% 1. 加载已经验收通过的标定参数
%% ============================================================

calib = load(calibrationFile);


requiredFields = [
    "qOffset"
    "T_CopBase_MatBase"
    "T_tool0_mount"
    "T_mount_tcp"
];


for i = 1:numel(requiredFields)

    if ~isfield(calib, requiredFields(i))

        error( ...
            "标定文件缺少变量：%s", ...
            requiredFields(i));

    end

end


qOffset = double(calib.qOffset);

T_CopBase_MatBase = ...
    double(calib.T_CopBase_MatBase);

T_tool0_mount = ...
    double(calib.T_tool0_mount);

T_mount_tcp = ...
    double(calib.T_mount_tcp);


if isfield(calib,"qOffsetDeg")

    qOffsetDeg = ...
        double(calib.qOffsetDeg);

else

    qOffsetDeg = ...
        rad2deg(qOffset);

end


fprintf("\n========================================\n");
fprintf("Frozen calibration loaded\n");
fprintf("========================================\n");

fprintf("qOffset [deg]:\n");
disp(qOffsetDeg);

fprintf("T_CopBase_MatBase:\n");
disp(T_CopBase_MatBase);

fprintf("T_tool0_mount:\n");
disp(T_tool0_mount);

fprintf("T_mount_tcp:\n");
disp(T_mount_tcp);


%% ============================================================
% 1b. 焊丝/喷嘴可视化参数
%
% 冻结适配器默认保留旧的 WeldTCP 线段；REAL manifest 分支稍后会把
% 参数化焊丝作为 WeldingGunMount 的固定子对象加入，不覆盖 TCP。
%% ============================================================

if isfield(calib,"wireLengthM")
    wireLengthM = double(calib.wireLengthM);
else
    wireLengthM = 0.018;
end

if isfield(calib,"wireDiameterM")
    wireDiameterM = double(calib.wireDiameterM);
    wireDiameterSource = "ur5_tool_calibration.mat";
else
    wireDiameterM = 0.0012;
    wireDiameterSource = "temporary_visual_default_NEEDS_CONFIRMATION";
end

if ~isscalar(wireLengthM) || ~isfinite(wireLengthM) || wireLengthM<=0
    error("wireLengthM 必须是正数（单位 m）。");
end
if ~isscalar(wireDiameterM) || ~isfinite(wireDiameterM) || wireDiameterM<=0
    error("wireDiameterM 必须是正数（单位 m）。");
end

fprintf("\nFrozen-adapter wire length  : %.1f mm (frozen baseline only; REAL scene replaces this)\n",wireLengthM*1000);
fprintf("Frozen-adapter wire diameter: %.3f mm (%s)\n", ...
    wireDiameterM*1000,wireDiameterSource);
if wireDiameterSource == "temporary_visual_default_NEEDS_CONFIRMATION"
    warning("当前焊丝直径仅为临时显示值，请按实际焊丝规格确认后更新。");
end


%% ============================================================
% 2. 读取 CoppeliaSim 场景几何 JSON
%% ============================================================

data = jsondecode( ...
    fileread(geometryFile));


fprintf("\nScene geometry loaded:\n");
fprintf("%s\n", geometryFile);
fprintf("Real-gun geometry manifest: %s (%s)\n", ...
    realGun.manifestFile,realGun.status);
if realGun.available && ~realGun.active
    fprintf("Real-gun replacement is not applied in this frozen-scene adapter; REAL scene applies it explicitly.\n");
end


%% ============================================================
% 3. 加载 MATLAB UR5
%% ============================================================

robot = loadrobot( ...
    "universalUR5", ...
    "DataFormat","row", ...
    "Gravity",[0 0 -9.81]);


fprintf("\nMATLAB universalUR5 loaded.\n");


%% ============================================================
% 4. 原始末端 body
%% ============================================================

lastBody = ...
    robot.BodyNames{end};


fprintf("Original MATLAB last body = %s\n", ...
    lastBody);


%% ============================================================
% 5. 创建 WeldingGunMount
%% ============================================================

mountBody = ...
    rigidBody("WeldingGunMount");


mountJoint = ...
    rigidBodyJoint( ...
        "WeldingGunMount_fixed", ...
        "fixed");


setFixedTransform( ...
    mountJoint, ...
    T_tool0_mount);


mountBody.Joint = ...
    mountJoint;


%% ============================================================
% 6. 加入焊枪碰撞模型
%
% 当前优先使用 CoppeliaSim 中新建的整体 GunCollisionHull。
% 旧的 5 个 box/cylinder 代理保留为 JSON 回退路径，便于调试。
%% ============================================================

fprintf("\n========================================\n");
fprintf("Adding welding-gun collision model\n");
fprintf("========================================\n");

useGunHull = ...
    isfield(data.welding_gun,"collision_hull");

if realGun.active

    % The active manifest stores vertices in one STL source frame and an
    % explicitly confirmed source->mount transform.  No nearest-vertex
    % inference or frozen GunCollisionHull data is used in this branch.
    manifest = realGun.manifest;
    collisionSpec = manifest.collision;
    T_mount_source = double(collisionSpec.T_mount_source);
    if ~isequal(size(T_mount_source),[4 4]) || any(~isfinite(T_mount_source(:)))
        error("ACTIVE real-gun manifest has an invalid T_mount_source.");
    end

    components = collisionSpec.components;
    numHullComponents = numel(components);
    gunProxyInfo = repmat(struct(...
        "name","", "type","", "size_m",[], ...
        "T_mount_proxy",[], "component_count",1),1,numHullComponents);
    for componentIndex = 1:numHullComponents
        if iscell(components)
            componentSpec = components{componentIndex};
        else
            componentSpec = components(componentIndex);
        end
        componentVertices = double(componentSpec.vertices_m);
        if size(componentVertices,2) ~= 3
            error("ACTIVE real-gun component %d vertices must be N×3.",componentIndex);
        end
        addCollision(mountBody,collisionMesh(componentVertices),T_mount_source);
        gunProxyInfo(componentIndex).name = string(componentSpec.name);
        gunProxyInfo(componentIndex).type = string(componentSpec.type);
        gunProxyInfo(componentIndex).size_m = max(componentVertices,[],1)-min(componentVertices,[],1);
        gunProxyInfo(componentIndex).T_mount_proxy = T_mount_source;
        fprintf("Real gun component %d: %s (%d vertices)\n", ...
            componentIndex,string(componentSpec.name),size(componentVertices,1));
    end
    gunCollisionMode = "real_gun_manifest";
    expectedGunCollisionCount = numHullComponents;
    hullHeadDistanceM = NaN;
    tipSource = double(manifest.wire.tip_source(:));
    if numel(tipSource) ~= 3
        error("ACTIVE real-gun wire.tip_source must be a 3-vector.");
    end
    pGunHeadMount = (T_mount_source*[tipSource;1]);
    pGunHeadMount = pGunHeadMount(1:3);
    wireLengthM = double(manifest.wire.length_m);
    wireDiameterM = double(manifest.wire.diameter_m);
    wireDiameterSource = "real_gun_geometry.json";

elseif useGunHull

    hullSpec = ...
        data.welding_gun.collision_hull;

    hullVertices = ...
        double(hullSpec.vertices_m);

    hullFaces = ...
        double(hullSpec.faces);

    if size(hullVertices,2) ~= 3 || size(hullFaces,2) ~= 3

        error( ...
            "GunCollisionHull 的 vertices/faces 必须分别为 N×3 和 M×3。 ");

    end

    if any(hullFaces(:) < 1) || ...
            any(hullFaces(:) > size(hullVertices,1))

        error( ...
            "GunCollisionHull faces 包含无效的 MATLAB 1-based 顶点索引。 ");

    end

    T_mount_hull = ...
        poseStructToTform( ...
            hullSpec.pose_mount);

    % Use the current collision hull as the geometric source for the gun
    % head direction.  The hull vertex nearest to the calibrated TCP is a
    % conservative estimate of the existing gun-head end; no new NozzleTip
    % object is created.
    hullVerticesMountHom = ...
        (T_mount_hull*[hullVertices ones(size(hullVertices,1),1)].').';
    hullVerticesMount = ...
        hullVerticesMountHom(:,1:3);
    tcpMountPosition = ...
        T_mount_tcp(1:3,4).';
    hullHeadDistance = ...
        vecnorm(hullVerticesMount-tcpMountPosition,2,2);
    [hullHeadDistanceM,hullHeadIndex] = ...
        min(hullHeadDistance);
    pGunHeadMount = ...
        hullVerticesMount(hullHeadIndex,:).';

    % MATLAB R2025b 的 collisionMesh 接口只接受 N×3 顶点；
    % 它会从这些顶点重新构造凸网格。若 CoppeliaSim 的对象是
    % convex_compound，则每个组件必须单独建立，不能把所有组件
    % 顶点再次合并成一个更大的凸包。
    if isfield(hullSpec,"components") && ...
            ~isempty(hullSpec.components)

        hullComponents = ...
            hullSpec.components;

        numHullComponents = ...
            numel(hullComponents);

    else

        hullComponents = ...
            struct( ...
                "vertices_m",hullVertices);

        numHullComponents = 1;

    end

    for componentIndex = 1:numHullComponents

        if iscell(hullComponents)
            componentSpec = ...
                hullComponents{componentIndex};
        else
            componentSpec = ...
                hullComponents(componentIndex);
        end

        componentVertices = ...
            double(componentSpec.vertices_m);

        if size(componentVertices,2) ~= 3

            error( ...
                "GunCollisionHull component %d 的 vertices 必须为 N×3。", ...
                componentIndex);

        end

        collisionObj = ...
            collisionMesh( ...
                componentVertices);

        addCollision( ...
            mountBody, ...
            collisionObj, ...
            T_mount_hull);

    end

    gunCollisionMode = ...
        "convex_hull";

    expectedGunCollisionCount = ...
        numHullComponents;

    hullSize = ...
        double(hullSpec.bbox_size_m(:).');

    gunProxyInfo = struct( ...
        "name",string(hullSpec.name), ...
        "type",string(hullSpec.type), ...
        "size_m",hullSize, ...
        "T_mount_proxy",T_mount_hull, ...
        "component_count",numHullComponents);

    fprintf( ...
        "GunCollisionHull: %d components, %d vertices, %d triangular faces\n", ...
        numHullComponents, ...
        size(hullVertices,1), ...
        size(hullFaces,1));

    fprintf( ...
        "Hull pose_mount = [%.6f %.6f %.6f] m\n", ...
        T_mount_hull(1,4), ...
        T_mount_hull(2,4), ...
        T_mount_hull(3,4));

else

    warning( ...
        "JSON 未包含 collision_hull，回退使用 5 个 GunCollision_* 代理。 ");

    gunSpecs = ...
        data.welding_gun.collision_proxies;

    numGunProxy = ...
        numel(gunSpecs);

    if numGunProxy ~= 5

        warning( ...
            "预期 5 个焊枪碰撞代理，实际读取到 %d 个。", ...
            numGunProxy);

    end

    gunProxyInfo = repmat( ...
        struct( ...
            "name","", ...
            "type","", ...
            "size_m",[], ...
            "T_mount_proxy",[]), ...
        1, ...
        numGunProxy);

    gunCollisionMode = ...
        "five_proxies";

    expectedGunCollisionCount = ...
        numGunProxy;

    % Legacy fallback has no mesh vertex from which to infer the direction.
    % Keep the calibrated TCP +Z convention only for this fallback path.
    hullHeadDistanceM = NaN;
    pGunHeadMount = ...
        T_mount_tcp(1:3,4)-[0;0;wireLengthM];

    fprintf("gunSpecs class = %s\n", class(gunSpecs));

    for k = 1:numGunProxy

        % jsondecode 可能返回 cell 或 struct 数组。
        if iscell(gunSpecs)
            spec = gunSpecs{k};
        else
            spec = gunSpecs(k);
        end

        collisionObj = ...
            makeCollisionObject(spec);

        T_mount_proxy = ...
            poseStructToTform( ...
                spec.pose_mount);

        addCollision( ...
            mountBody, ...
            collisionObj, ...
            T_mount_proxy);

        gunProxyInfo(k).name = ...
            string(spec.name);

        gunProxyInfo(k).type = ...
            string(spec.type);

        gunProxyInfo(k).size_m = ...
            double(spec.size_m(:).');

        gunProxyInfo(k).T_mount_proxy = ...
            T_mount_proxy;

        fprintf( ...
            "%d. %-24s %-10s size = [%.4f %.4f %.4f] m\n", ...
            k, ...
            string(spec.name), ...
            string(spec.type), ...
            gunProxyInfo(k).size_m(1), ...
            gunProxyInfo(k).size_m(2), ...
            gunProxyInfo(k).size_m(3));

    end

end


%% ============================================================
% 7. 将 WeldingGunMount 加到机器人
%% ============================================================

addBody( ...
    robot, ...
    mountBody, ...
    lastBody);


%% ============================================================
% 8. 添加 WeldTCP
%% ============================================================

tcpBody = ...
    rigidBody("WeldTCP");


tcpJoint = ...
    rigidBodyJoint( ...
        "WeldTCP_fixed", ...
        "fixed");


setFixedTransform( ...
    tcpJoint, ...
    T_mount_tcp);


tcpBody.Joint = ...
    tcpJoint;


%% 焊丝几何：可视化 + 碰撞
%
% 参数化 REAL 焊丝沿“CAD 枪尖 -> TCP”连线，长度严格等于两点距离。
% REAL 分支把它作为 WeldingGunMount 的固定子对象；冻结回退分支继续
% 保留原有 WeldTCP 线段，避免改变冻结基准。
T_tcp_mount = ...
    T_mount_tcp \ eye(4);
pGunHeadTcpHom = ...
    T_tcp_mount*[pGunHeadMount;1];
pGunHeadTcp = ...
    pGunHeadTcpHom(1:3);
if norm(pGunHeadTcp)<=eps
    error("Gun head endpoint 与 WeldTCP 重合，无法确定焊丝方向。");
end
if realGun.active
    axisSource = double(manifest.wire.axis_source(:));
    if numel(axisSource) ~= 3 || any(~isfinite(axisSource)) || norm(axisSource) <= eps
        error("ACTIVE real-gun wire.axis_source 必须是非零有限 3-vector。");
    end
    axisMount = T_mount_source(1:3,1:3) * (axisSource/norm(axisSource));
    zWireTcp = T_tcp_mount(1:3,1:3) * axisMount;
    zWireTcp = zWireTcp/norm(zWireTcp);
else
    zWireTcp = ...
        -pGunHeadTcp/norm(pGunHeadTcp);
end
xReference = [1;0;0];
if abs(dot(xReference,zWireTcp))>0.95
    xReference = [0;1;0];
end
xWireTcp = cross(xReference,zWireTcp);
xWireTcp = xWireTcp/norm(xWireTcp);
yWireTcp = cross(zWireTcp,xWireTcp);
yWireTcp = yWireTcp/norm(yWireTcp);
T_tcp_wire = eye(4);
T_tcp_wire(1:3,1:3) = [xWireTcp yWireTcp zWireTcp];
% Cylinder 的 -Z 端固定在 Gun head，+Z 端朝 WeldTCP。
T_tcp_wire(1:3,4) = pGunHeadTcp + wireLengthM/2*zWireTcp;
if realGun.active
    % Explicit real chain: flange -> WeldingGunMount -> WeldingGunWire.
    T_mount_wire = T_mount_tcp*T_tcp_wire;
    wireBody = rigidBody("WeldingGunWire");
    wireJoint = rigidBodyJoint("WeldingGunWire_fixed","fixed");
    setFixedTransform(wireJoint,T_mount_wire);
    wireBody.Joint = wireJoint;
    addVisual(wireBody,"Cylinder",[wireDiameterM/2 wireLengthM],eye(4), ...
        "FaceColor",[0.95 0.75 0.15],"FaceAlpha",1.0);
    addCollision(wireBody,collisionCylinder(wireDiameterM/2,wireLengthM),eye(4));
else
    addVisual(tcpBody,"Cylinder",[wireDiameterM/2 wireLengthM],T_tcp_wire, ...
        "FaceColor",[0.95 0.75 0.15],"FaceAlpha",1.0);
    addCollision(tcpBody,collisionCylinder(wireDiameterM/2,wireLengthM),T_tcp_wire);
end

addBody(robot,tcpBody,"WeldingGunMount");
if realGun.active
    addBody(robot,wireBody,"WeldingGunMount");
end


%% ============================================================
% 9. 检查焊枪碰撞体确实已经加入
%% ============================================================

mountCheck = ...
    getBody( ...
        robot, ...
        "WeldingGunMount");


numMountedCollisions = ...
    numel(mountCheck.Collisions);


fprintf("\nWeldingGunMount collision count = %d\n", ...
    numMountedCollisions);


if numMountedCollisions ~= expectedGunCollisionCount

    error( ...
        "焊枪碰撞模型没有正确加入 rigidBodyTree。 ");

end

tcpCheck = ...
    getBody( ...
        robot, ...
        "WeldTCP");

numTcpCollisions = ...
    numel(tcpCheck.Collisions);

fprintf("WeldTCP collision count = %d\n", ...
    numTcpCollisions);

if realGun.active
    expectedTcpCollisions = 0;
else
    expectedTcpCollisions = 1;
end
if numTcpCollisions ~= expectedTcpCollisions

    error( ...
        "焊丝碰撞圆柱的固定父对象不符合当前场景链。 ");

end
if realGun.active
    wireCheck = getBody(robot,"WeldingGunWire");
    if numel(wireCheck.Collisions) ~= 1
        error("REAL 焊丝碰撞圆柱没有正确加入 WeldingGunWire。 ");
    end
end


%% ============================================================
% 10. CoppeliaSim Workpiece pose
%
% JSON 中 Workpiece pose 是相对于：
%
%       CoppeliaSim /UR5 base
%
% 但是 MATLAB checkCollision 的环境几何必须处于：
%
%       MATLAB robot base
%
% 因此必须经过已经验收通过的：
%
%       T_CopBase_MatBase
%
% 转换。
%% ============================================================

T_CopBase_Workpiece = ...
    poseStructToTform( ...
        data.workpiece.pose_base);


%% 已知：
%
% T_CopBase_Workpiece
% =
% T_CopBase_MatBase * T_MatBase_Workpiece
%
% 所以：
%
% T_MatBase_Workpiece
% =
% inv(T_CopBase_MatBase) * T_CopBase_Workpiece

T_MatBase_Workpiece = ...
    T_CopBase_MatBase \ ...
    T_CopBase_Workpiece;


fprintf("\n========================================\n");
fprintf("Workpiece transformation\n");
fprintf("========================================\n");

fprintf("Workpiece pose in Coppelia base:\n");
disp(T_CopBase_Workpiece);

fprintf("Workpiece pose in MATLAB base:\n");
disp(T_MatBase_Workpiece);


%% ============================================================
% 11. 创建 Workpiece 四个碰撞代理
%
% U 型工件近似：
%
%   bottom
%   wall_x0
%   wall_x1
%   wall_y1（开口方向为 y=0）
%% ============================================================

wpSpecs = ...
    data.workpiece.collision_proxies;


numWorkpieceProxy = ...
    numel(wpSpecs);


if numWorkpieceProxy ~= 4

    warning( ...
        "预期 4 个工件碰撞代理，实际读取到 %d 个。", ...
        numWorkpieceProxy);

end


workpieceCollision = ...
    cell(1,numWorkpieceProxy);


workpieceProxyInfo = repmat( ...
    struct( ...
        "name","", ...
        "type","", ...
        "size_m",[], ...
        "T_workpiece_proxy",[], ...
        "T_matbase_proxy",[]), ...
    1, ...
    numWorkpieceProxy);


fprintf("\n========================================\n");
fprintf("Creating Workpiece collision proxies\n");
fprintf("========================================\n");
fprintf("wpSpecs class = %s\n", class(wpSpecs));


for k = 1:numWorkpieceProxy

    % jsondecode 在不同 MATLAB 版本中可能返回 cell 或 struct 数组。
    if iscell(wpSpecs)
        spec = wpSpecs{k};
    else
        spec = wpSpecs(k);
    end


    %% 创建碰撞体
    collisionObj = ...
        makeCollisionObject(spec);


    %% Workpiece -> collision proxy
    T_workpiece_proxy = ...
        poseStructToTform( ...
            spec.pose_workpiece);


    %% MATLAB base -> proxy
    T_matbase_proxy = ...
        T_MatBase_Workpiece * ...
        T_workpiece_proxy;


    %% MATLAB environment collision object
    collisionObj.Pose = ...
        T_matbase_proxy;


    workpieceCollision{k} = ...
        collisionObj;


    %% 保存
    workpieceProxyInfo(k).name = ...
        string(spec.name);

    workpieceProxyInfo(k).type = ...
        string(spec.type);

    workpieceProxyInfo(k).size_m = ...
        double(spec.size_m(:).');

    workpieceProxyInfo(k).T_workpiece_proxy = ...
        T_workpiece_proxy;

    workpieceProxyInfo(k).T_matbase_proxy = ...
        T_matbase_proxy;


    fprintf( ...
        "%d. %-28s size = [%.4f %.4f %.4f] m\n", ...
        k, ...
        string(spec.name), ...
        workpieceProxyInfo(k).size_m(1), ...
        workpieceProxyInfo(k).size_m(2), ...
        workpieceProxyInfo(k).size_m(3));

end


%% ============================================================
% 12. Environment
%
% 后续 RRT 的 collision checker 直接使用它
%% ============================================================

environment = ...
    workpieceCollision;


fprintf("\nEnvironment collision object count = %d\n", ...
    numel(environment));


%% ============================================================
% 13. 初始关节姿态
%
% CoppeliaSim 已恢复零位：
%
% qCop0 = [0 0 0 0 0 0]
%
% MATLAB：
%
% qMat0 = qCop0 + qOffset
%% ============================================================

qCop0 = ...
    zeros(1,6);


qMat0 = ...
    qCop0 + qOffset;


fprintf("\nInitial Coppelia q [deg]:\n");
disp(rad2deg(qCop0));

fprintf("Initial MATLAB q [deg]:\n");
disp(rad2deg(qMat0));


%% ============================================================
% 14. 当前 WeldTCP
%% ============================================================

T_MatBase_TCP0 = ...
    getTransform( ...
        robot, ...
        qMat0, ...
        "WeldTCP");


pTcpMat0 = ...
    tform2trvec( ...
        T_MatBase_TCP0);

% 计算焊枪头端和焊丝靠近 TCP 的末端，保证显示线段和实体完全重合。
pGunHeadTcpHom = T_tcp_wire*[0;0;-wireLengthM/2;1];
pGunHeadBaseHom = T_MatBase_TCP0*pGunHeadTcpHom;
pGunHeadMat0 = pGunHeadBaseHom(1:3).';

pWireEndTcpHom = T_tcp_wire*[0;0;wireLengthM/2;1];
pWireEndBaseHom = T_MatBase_TCP0*pWireEndTcpHom;
pWireEndMat0 = pWireEndBaseHom(1:3).';
wireTcpGapM = norm(pTcpMat0-pWireEndMat0);


%% 转到 Coppelia base，仅用于核对
T_CopBase_TCP0 = ...
    T_CopBase_MatBase * ...
    T_MatBase_TCP0;


pTcpCop0 = ...
    tform2trvec( ...
        T_CopBase_TCP0);


fprintf("\nWeldTCP in MATLAB base [m]:\n");
disp(pTcpMat0);

fprintf("WeldTCP predicted in Coppelia base [m]:\n");
disp(pTcpCop0);


%% ============================================================
% 15. 初始 Robot <-> Workpiece 碰撞检查
%
% IgnoreSelfCollision = on
%
% 含义：
%   暂时忽略机器人自身碰撞，
%   只检查：
%
%   UR5 + 焊枪碰撞模型
%
%            VS
%
%   Workpiece
%
%% ============================================================

fprintf("\n========================================\n");
fprintf("INITIAL COLLISION CHECK\n");
fprintf("========================================\n");


try

    collisionResult = ...
        checkCollision( ...
            robot, ...
            qMat0, ...
            environment, ...
            "IgnoreSelfCollision","on", ...
            "Exhaustive","on");

catch

    % 某些 MATLAB 版本若不接受 Exhaustive，
    % 自动退回基础调用。
    collisionResult = ...
        checkCollision( ...
            robot, ...
            qMat0, ...
            environment, ...
            "IgnoreSelfCollision","on");

end


hasEnvironmentCollision = ...
    any(collisionResult(:));


if hasEnvironmentCollision

    fprintf("\n[COLLISION] 当前零位存在 Robot/Gun 与 Workpiece 碰撞。\n");

else

    fprintf("\n[FREE] 当前零位 Robot/Gun 与 Workpiece 无碰撞。\n");

end


%% ============================================================
% 16. MATLAB 场景显示
%
% Visuals    = MATLAB UR5 外观 + 焊丝可视化圆柱
% Collisions = UR5 原有碰撞体 + GunCollisionHull + 焊丝碰撞圆柱
%
% 环境：
%   4个 Workpiece collision boxes
%% ============================================================

% 批处理规划默认不创建场景窗口。需要人工检查模型时，可在 MATLAB
% 桌面中设置 RRT_SHOW_SCENE=1 后重新运行本脚本。
% -batch 模式下 usejava("desktop") 通常为 false，因此不会弹出窗口。
showPlanningScene = ...
    usejava("desktop") && ...
    ~strcmp(strtrim(getenv("RRT_HEADLESS")),"1");

if strcmp(strtrim(getenv("RRT_SHOW_SCENE")),"1")
    showPlanningScene = true;
end

if showPlanningScene

fig = figure( ...
    "Name", ...
    "UR5 + Welding Gun + Workpiece Planning Scene", ...
    "Color","w");


ax = axes(fig);


show( ...
    robot, ...
    qMat0, ...
    "Parent",ax, ...
    "Visuals","on", ...
    "Collisions","on", ...
    "Frames","off", ...
    "PreservePlot",false);


hold(ax,"on");


%% ------------------------------------------------------------
% 显示 Workpiece 三个碰撞代理
%% ------------------------------------------------------------

axes(ax); %#ok<LAXES>


for k = 1:numel(workpieceCollision)

    show( ...
        workpieceCollision{k}, ...
        "Parent",ax);

end


%% ============================================================
% 17. 标记 WeldTCP
%% ============================================================

plot3( ...
    ax, ...
    pTcpMat0(1), ...
    pTcpMat0(2), ...
    pTcpMat0(3), ...
    "ro", ...
    "MarkerSize",10, ...
    "LineWidth",2);

% 只画实际焊丝：从现有焊枪头端到焊丝末端。
plot3( ...
    ax, ...
    [pGunHeadMat0(1) pWireEndMat0(1)], ...
    [pGunHeadMat0(2) pWireEndMat0(2)], ...
    [pGunHeadMat0(3) pWireEndMat0(3)], ...
    "-", ...
    "Color",[0.95 0.65 0.05], ...
    "LineWidth",4);

% 若焊丝长度小于枪头到 TCP 的距离，使用虚线显示有意保留的间隙。
if wireTcpGapM > 1e-9
    plot3( ...
        ax, ...
        [pWireEndMat0(1) pTcpMat0(1)], ...
        [pWireEndMat0(2) pTcpMat0(2)], ...
        [pWireEndMat0(3) pTcpMat0(3)], ...
        "--", ...
        "Color",[0.45 0.45 0.45], ...
        "LineWidth",1.2);
end

text( ...
    ax, ...
    pGunHeadMat0(1), ...
    pGunHeadMat0(2), ...
    pGunHeadMat0(3), ...
    "  Gun head", ...
    "FontSize",10);


text( ...
    ax, ...
    pTcpMat0(1), ...
    pTcpMat0(2), ...
    pTcpMat0(3), ...
    "  WeldTCP", ...
    "FontSize",11);


%% TCP 坐标系
drawFrame( ...
    ax, ...
    T_MatBase_TCP0, ...
    0.08, ...
    "TCP");


%% Workpiece 坐标系
drawFrame( ...
    ax, ...
    T_MatBase_Workpiece, ...
    0.08, ...
    "Workpiece");


%% ============================================================
% 18. 图形设置
%% ============================================================

axis(ax,"equal");
grid(ax,"on");

xlabel(ax,"MATLAB base X (m)");
ylabel(ax,"MATLAB base Y (m)");
zlabel(ax,"MATLAB base Z (m)");

view(ax,135,25);


title( ...
    ax, ...
    "UR5 + Welding Gun Collision Model + Workpiece");

end


%% ============================================================
% 19. 汇总为后续 RRT 使用的 scene
%% ============================================================

scene = struct();


%% Robot
scene.robot = ...
    robot;


%% Names
scene.mountBodyName = ...
    "WeldingGunMount";

scene.tcpBodyName = ...
    "WeldTCP";


%% Frozen calibration
scene.qOffset = ...
    qOffset;

scene.qOffsetDeg = ...
    qOffsetDeg;

scene.T_CopBase_MatBase = ...
    T_CopBase_MatBase;

scene.T_tool0_mount = ...
    T_tool0_mount;

scene.T_mount_tcp = ...
    T_mount_tcp;

%% Independent real-gun package status
scene.realGunGeometryManifest = ...
    realGun.manifestFile;
scene.realGunGeometryStatus = ...
    string(realGun.status);
scene.realGunGeometryActive = ...
    logical(realGun.active);


%% Welding wire geometry (visual + collision)
scene.wireLengthM = ...
    wireLengthM;

scene.wireDiameterM = ...
    wireDiameterM;

scene.wireDiameterSource = ...
    wireDiameterSource;

scene.wireCollisionEnabled = ...
    true;

scene.wireCollisionRadiusM = ...
    wireDiameterM/2;

scene.T_tcp_wire = ...
    T_tcp_wire;

if realGun.active
    scene.T_mount_wire = T_mount_wire;
    scene.wireBodyName = "WeldingGunWire";
    scene.wireParentBody = "WeldingGunMount";
else
    scene.T_mount_wire = T_mount_tcp*T_tcp_wire;
    scene.wireBodyName = "WeldTCP";
    scene.wireParentBody = "WeldingGunMount";
end

scene.wireAxisTcp = ...
    zWireTcp;

scene.hullHeadDistanceM = ...
    hullHeadDistanceM;

%% Gun
scene.gunProxyInfo = ...
    gunProxyInfo;

scene.gunCollisionMode = ...
    gunCollisionMode;


%% Workpiece
scene.T_MatBase_Workpiece = ...
    T_MatBase_Workpiece;

scene.workpieceCollision = ...
    workpieceCollision;

scene.workpieceProxyInfo = ...
    workpieceProxyInfo;


%% Environment
scene.environment = ...
    environment;


%% Initial configuration
scene.qCop0 = ...
    qCop0;

scene.qMat0 = ...
    qMat0;


%% TCP
scene.T_MatBase_TCP0 = ...
    T_MatBase_TCP0;

scene.pTcpMat0 = ...
    pTcpMat0;

scene.pGunHeadMat0 = ...
    pGunHeadMat0;

scene.pWireEndMat0 = ...
    pWireEndMat0;

scene.wireTcpGapM = ...
    wireTcpGapM;


%% Collision status
scene.initialEnvironmentCollision = ...
    hasEnvironmentCollision;


%% Raw geometry JSON
scene.geometryData = ...
    data;


%% ============================================================
% 20. 最终检查
%% ============================================================

fprintf("\n========================================\n");
fprintf("PLANNING SCENE READY\n");
fprintf("========================================\n");

fprintf("UR5 bodies             : %d\n", ...
    numel(robot.BodyNames));

fprintf("Gun collision proxies  : %d\n", ...
    numMountedCollisions);

fprintf("Workpiece proxies       : %d\n", ...
    numel(environment));

fprintf("TCP body                : %s\n", ...
    scene.tcpBodyName);

fprintf("Wire visual             : %.1f mm long, %.3f mm diameter\n", ...
    scene.wireLengthM*1000,scene.wireDiameterM*1000);
fprintf("Wire collision          : enabled (collisionCylinder)\n");
fprintf("Wire axis in TCP        : [%.4f %.4f %.4f]\n", ...
    scene.wireAxisTcp(1),scene.wireAxisTcp(2),scene.wireAxisTcp(3));
if isfinite(scene.hullHeadDistanceM)
    fprintf("Nearest hull head gap   : %.1f mm (geometry estimate)\n", ...
        scene.hullHeadDistanceM*1000);
end
fprintf("Gun head end in MATLAB base [m]:\n");
disp(scene.pGunHeadMat0);
fprintf("Wire end near TCP in MATLAB base [m]:\n");
disp(scene.pWireEndMat0);
fprintf("Wire-to-TCP remaining gap: %.1f mm\n", ...
    scene.wireTcpGapM*1000);

fprintf("MATLAB planning q0 [deg]:\n");
disp(rad2deg(scene.qMat0));


fprintf("\n后续 RRT 直接使用：\n");
fprintf("    scene.robot\n");
fprintf("    scene.environment\n");
fprintf("    scene.qOffset\n");
fprintf("    scene.tcpBodyName\n");

fprintf("\n场景建立完成。\n");


%% ============================================================
% Local Function 1
% JSON pose -> MATLAB homogeneous transform
%% ============================================================

function T = poseStructToTform(pose)

    %% position 强制为 1x3
    p = ...
        reshape( ...
            double(pose.position), ...
            1,3);


    %% Coppelia quaternion:
    %
    % [qx qy qz qw]
    qCop = ...
        reshape( ...
            double(pose.quaternion), ...
            1,4);


    %% MATLAB:
    %
    % [qw qx qy qz]
    qMat = [ ...
        qCop(4), ...
        qCop(1), ...
        qCop(2), ...
        qCop(3)];


    T = ...
        trvec2tform(p) * ...
        quat2tform(qMat);

end


%% ============================================================
% Local Function 2
% JSON collision spec -> MATLAB collision object
%% ============================================================

function collisionObj = makeCollisionObject(spec)

    sizeM = ...
        double(spec.size_m(:).');


    switch lower(string(spec.type))

        case "box"

            collisionObj = ...
                collisionBox( ...
                    sizeM(1), ...
                    sizeM(2), ...
                    sizeM(3));


        case "cylinder"

            %% Coppelia size：
            %
            % [diameter diameter length]
            %
            % MATLAB：
            %
            % collisionCylinder(radius,length)

            collisionObj = ...
                collisionCylinder( ...
                    sizeM(1)/2, ...
                    sizeM(3));


        otherwise

            error( ...
                "不支持的碰撞几何类型：%s", ...
                string(spec.type));

    end

end


%% ============================================================
% Local Function 3
% 简单绘制坐标系
%% ============================================================

function drawFrame(ax,T,scale,labelText)

    p = ...
        T(1:3,4);


    R = ...
        T(1:3,1:3);


    xEnd = ...
        p + scale * R(:,1);

    yEnd = ...
        p + scale * R(:,2);

    zEnd = ...
        p + scale * R(:,3);


    plot3( ...
        ax, ...
        [p(1) xEnd(1)], ...
        [p(2) xEnd(2)], ...
        [p(3) xEnd(3)], ...
        "r-", ...
        "LineWidth",2);


    plot3( ...
        ax, ...
        [p(1) yEnd(1)], ...
        [p(2) yEnd(2)], ...
        [p(3) yEnd(3)], ...
        "g-", ...
        "LineWidth",2);


    plot3( ...
        ax, ...
        [p(1) zEnd(1)], ...
        [p(2) zEnd(2)], ...
        [p(3) zEnd(3)], ...
        "b-", ...
        "LineWidth",2);


    text( ...
        ax, ...
        p(1), ...
        p(2), ...
        p(3), ...
        "  " + string(labelText));

end
