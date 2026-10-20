function report = diagnose_weld_collisions(scene, qWeld)
%DIAGNOSE_WELD_COLLISIONS
%
% 找出焊接轨迹中到底是：
%
%   哪个机器人 body
%       VS
%   哪个 Workpiece collision object
%
% 发生碰撞。
%
% 用法：
%
%   report = diagnose_weld_collisions(scene, qWeld);

    robot = scene.robot;
    environment = scene.environment;

    N = size(qWeld,1);
    W = numel(environment);

    %% ========================================================
    % Robot body names
    %% ========================================================

    bodyNames = string(robot.BodyNames);

    if isprop(robot,"BaseName")
        baseName = string(robot.BaseName);
    else
        baseName = "base";
    end

    rowNames = [bodyNames, baseName];


    %% ========================================================
    % Environment names
    %% ========================================================

    worldNames = strings(1,W);

    for j = 1:W

        if isfield(scene,"workpieceProxyInfo") && ...
                numel(scene.workpieceProxyInfo) >= j

            worldNames(j) = ...
                string(scene.workpieceProxyInfo(j).name);

        else

            worldNames(j) = ...
                "Environment_" + string(j);

        end

    end


    %% ========================================================
    % Collision count matrix
    %
    % 行：
    % robot body
    %
    % 列：
    % workpiece collision object
    %% ========================================================

    pairCount = zeros( ...
        numel(rowNames), ...
        W);

    collidingPoint = false(N,1);


    fprintf("\n");
    fprintf("========================================\n");
    fprintf("WELD COLLISION DIAGNOSIS\n");
    fprintf("========================================\n");


    %% ========================================================
    % Check all weld poses
    %% ========================================================

    for i = 1:N

        [isColliding, sepDist] = checkCollision( ...
            robot, ...
            qWeld(i,:), ...
            environment, ...
            "IgnoreSelfCollision","on", ...
            "Exhaustive","on");


        %% 整体碰撞状态
        collidingPoint(i) = any(isColliding(:));


        %% ----------------------------------------------------
        % separationDist 最右边 W 列就是 world objects
        %% ----------------------------------------------------

        if size(sepDist,2) < W

            error( ...
                "separationDist 列数异常：%d", ...
                size(sepDist,2));

        end


        worldSep = ...
            sepDist(:, end-W+1:end);


        %% ----------------------------------------------------
        % NaN 表示该 pair 正在碰撞
        %% ----------------------------------------------------

        [r,c] = find(isnan(worldSep));


        for n = 1:numel(r)

            if r(n) <= size(pairCount,1)

                pairCount(r(n),c(n)) = ...
                    pairCount(r(n),c(n)) + 1;

            end

        end

    end


    %% ========================================================
    % Print collision pair statistics
    %% ========================================================

    fprintf("\nCollision pair counts over %d weld poses:\n\n",N);

    found = false;


    for r = 1:size(pairCount,1)

        for c = 1:W

            if pairCount(r,c) > 0

                found = true;

                fprintf( ...
                    "%-25s  <-->  %-35s : %3d / %d\n", ...
                    rowNames(r), ...
                    worldNames(c), ...
                    pairCount(r,c), ...
                    N);

            end

        end

    end


    if ~found

        fprintf("No robot-world collision pairs found.\n");

    end


    %% ========================================================
    % Detailed inspection at start / middle / end
    %% ========================================================

    testIndices = unique([ ...
        1, ...
        round(N/2), ...
        N]);


    fprintf("\n");
    fprintf("========================================\n");
    fprintf("START / MIDDLE / END DETAILS\n");
    fprintf("========================================\n");


    for ii = 1:numel(testIndices)

        i = testIndices(ii);

        [~,sepDist] = checkCollision( ...
            robot, ...
            qWeld(i,:), ...
            environment, ...
            "IgnoreSelfCollision","on", ...
            "Exhaustive","on");


        worldSep = ...
            sepDist(:,end-W+1:end);

        [r,c] = find(isnan(worldSep));


        fprintf("\nWeld point %d / %d\n",i,N);


        if isempty(r)

            fprintf("  No collision.\n");

        else

            for n = 1:numel(r)

                if r(n) <= numel(rowNames)

                    fprintf( ...
                        "  %s  <-->  %s\n", ...
                        rowNames(r(n)), ...
                        worldNames(c(n)));

                end

            end

        end

    end


    %% ========================================================
    % Output
    %% ========================================================

    report = struct();

    report.pairCount = pairCount;

    report.robotBodyNames = rowNames;

    report.worldNames = worldNames;

    report.collidingPoint = collidingPoint;

    report.numCollidingPoints = nnz(collidingPoint);

end