function summary = inspect_workpiece_geometry(scene)
%INSPECT_WORKPIECE_GEOMETRY
% 输出工件4个碰撞板在 Workpiece 局部坐标系中的
% 尺寸、中心位置和旋转矩阵。

    %% ========================================================
    % 1. 输入检查
    %% ========================================================

    if ~isstruct(scene)
        error("scene 必须是结构体。");
    end

    if ~isfield(scene,"T_MatBase_Workpiece")
        error("scene 缺少 T_MatBase_Workpiece。");
    end

    if ~isfield(scene,"workpieceProxyInfo")
        error("scene 缺少 workpieceProxyInfo。");
    end


    %% ========================================================
    % 2. Workpiece 坐标系
    %% ========================================================

    T_B_W = scene.T_MatBase_Workpiece;

    fprintf("\n");
    fprintf("========================================\n");
    fprintf("WORKPIECE GEOMETRY INSPECTION\n");
    fprintf("========================================\n");

    fprintf("\nT_MatBase_Workpiece =\n");
    disp(T_B_W);

    pW = T_B_W(1:3,4);
    RW = T_B_W(1:3,1:3);

    fprintf("Workpiece origin in MATLAB base [m]:\n");
    disp(pW.');

    fprintf("Workpiece +X axis in MATLAB base:\n");
    disp(RW(:,1).');

    fprintf("Workpiece +Y axis in MATLAB base:\n");
    disp(RW(:,2).');

    fprintf("Workpiece +Z axis in MATLAB base:\n");
    disp(RW(:,3).');


    %% ========================================================
    % 3. 读取4个工件碰撞代理
    %% ========================================================

    info = scene.workpieceProxyInfo;
    N = numel(info);

    fprintf("\n");
    fprintf("========================================\n");
    fprintf("WORKPIECE COLLISION PROXIES\n");
    fprintf("========================================\n");

    fprintf("Number of proxies = %d\n",N);


    summary = repmat(struct( ...
        "name","", ...
        "type","", ...
        "size_m",[], ...
        "center_wp",[], ...
        "rotation_wp",[], ...
        "T_wp_proxy",[]),1,N);


    for k = 1:N

        fprintf("\nProxy %d\n",k);
        fprintf("----------------------------------------\n");


        %% 名称
        if isfield(info(k),"name")
            name = string(info(k).name);
        else
            name = "Proxy_" + string(k);
        end


        %% 类型
        if isfield(info(k),"type")
            type = string(info(k).type);
        else
            type = "unknown";
        end


        %% 尺寸
        if isfield(info(k),"size_m")
            sizeM = double(info(k).size_m(:).');
        else
            sizeM = [];
        end


        %% Workpiece -> Proxy 变换
        if isfield(info(k),"T_workpiece_proxy")

            T_W_P = double(info(k).T_workpiece_proxy);

        elseif isfield(info(k),"pose_workpiece")

            T_W_P = double(info(k).pose_workpiece);

        else

            error( ...
                "Proxy %d (%s) 缺少 T_workpiece_proxy。", ...
                k,name);

        end


        center = T_W_P(1:3,4).';
        rotation = T_W_P(1:3,1:3);


        %% 打印
        fprintf("name:\n");
        disp(name);

        fprintf("type:\n");
        disp(type);

        fprintf("size [m]:\n");
        disp(sizeM);

        fprintf("center in Workpiece frame [m]:\n");
        disp(center);

        fprintf("rotation in Workpiece frame:\n");
        disp(rotation);


        %% 保存
        summary(k).name = name;
        summary(k).type = type;
        summary(k).size_m = sizeM;
        summary(k).center_wp = center;
        summary(k).rotation_wp = rotation;
        summary(k).T_wp_proxy = T_W_P;

    end


    %% ========================================================
    % 4. 汇总
    %% ========================================================

    fprintf("\n");
    fprintf("========================================\n");
    fprintf("SUMMARY\n");
    fprintf("========================================\n");


    for k = 1:N

        fprintf("\n%d. %s\n", ...
            k,summary(k).name);

        if numel(summary(k).size_m) >= 3

            fprintf( ...
                "   size   = [%.6f %.6f %.6f] m\n", ...
                summary(k).size_m(1), ...
                summary(k).size_m(2), ...
                summary(k).size_m(3));

        end

        fprintf( ...
            "   center = [%.6f %.6f %.6f] m\n", ...
            summary(k).center_wp(1), ...
            summary(k).center_wp(2), ...
            summary(k).center_wp(3));

    end


    fprintf("\n");
    fprintf("========================================\n");
    fprintf("END\n");
    fprintf("========================================\n");

end