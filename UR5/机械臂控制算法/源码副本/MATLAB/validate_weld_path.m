function report = validate_weld_path(scene, qWeld)

%VALIDATE_WELD_PATH
% 检查整条焊接轨迹：
%
% 1. Robot + WeldingGun 与环境碰撞
% 2. 关节连续性
%
% 后续还可以加入：
% 3. 自碰撞
% 4. 关节限位
% 5. 速度/加速度

    robot = scene.robot;
    environment = scene.environment;

    N = size(qWeld,1);

    collision = false(N,1);

    maxJointJump = zeros(N-1,1);

    %% --------------------------------------------------------
    % 1. 每个焊接点碰撞检测
    %% --------------------------------------------------------

    for i = 1:N

        c = checkCollision( ...
            robot, ...
            qWeld(i,:), ...
            environment, ...
            "IgnoreSelfCollision","on");

        collision(i) = any(c(:));

    end

    %% --------------------------------------------------------
    % 2. 检查关节跳变
    %% --------------------------------------------------------

    for i = 1:N-1

        dq = ...
            wrapToPi( ...
                qWeld(i+1,:) - ...
                qWeld(i,:));

        maxJointJump(i) = ...
            max(abs(dq));

    end

    %% --------------------------------------------------------
    % 3. 报告
    %% --------------------------------------------------------

    report = struct();

    report.collision = collision;

    report.collisionIndices = ...
        find(collision);

    report.isCollisionFree = ...
        ~any(collision);

    report.maxJointJumpRad = ...
        max(maxJointJump);

    report.maxJointJumpDeg = ...
        rad2deg(report.maxJointJumpRad);

end