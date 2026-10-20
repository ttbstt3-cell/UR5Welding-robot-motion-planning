function result = RRT_basic_animation()
%% ================================================================
% 基础二维 RRT + 搜索动画 + 路径运动动画 + 时间统计
%
% 运行：
% >> result = RRT_basic_animation;
%
% 绿色：RRT 搜索树
% 灰色：障碍物
% 蓝色：最终规划路径
% 紫色：机器人运动轨迹
%
% 时间统计：
% 1. 纯算法计算时间
% 2. 搜索阶段总时间
% 3. 机器人运动动画时间
% 4. 程序总运行时间
%% ================================================================

clc;
close all;

rng(1);

%% ================================================================
% 1. 程序总计时
%% ================================================================

programTimer = tic;

%% ================================================================
% 2. 地图范围
%% ================================================================

xmin = 0;
xmax = 100;

ymin = 0;
ymax = 100;

%% ================================================================
% 3. 起点与终点
%% ================================================================

startPoint = [10 10];

goalPoint = [90 90];

%% ================================================================
% 4. 障碍物
%
% 格式：
% [左下角X 左下角Y 宽度 高度]
%% ================================================================

obstacles = [

    25   0   15   55

    55  45   15   55

];

%% ================================================================
% 5. RRT 参数
%% ================================================================

stepSize = 4;

goalThreshold = 5;

maxIterations = 5000;

% 统一二维碰撞检测安全距离
clearance = 0;

%% ================================================================
% 6. 动画参数
%% ================================================================

% 是否显示搜索动画
showSearchAnimation = true;

% 是否显示机器人运动动画
showRobotAnimation = true;

% 搜索动画暂停时间
treePauseTime = 0.003;

% 机器人运动动画暂停时间
robotPauseTime = 0.01;

% 运动插值间隔
animationStep = 0.2;

%% ================================================================
% 7. 初始化随机树
%% ================================================================

nodes = startPoint;

parent = 0;

goalReached = false;

goalIndex = -1;

%% ================================================================
% 8. 初始化统计变量
%% ================================================================

% 纯 RRT 计算时间
purePlanningTime = 0;

% 搜索阶段总时间
searchWallTime = 0;

% 运动动画时间
motionTime = 0;

% 程序总时间
totalTime = 0;

% 实际迭代次数
iterationsUsed = 0;

% 路径长度
pathLength = NaN;

% 最终路径
path = [];

%% ================================================================
% 9. 创建图形窗口
%% ================================================================

figure( ...
    'Name','Basic RRT Animation', ...
    'NumberTitle','off', ...
    'Color','w');

hold on;

grid on;

box on;

axis equal;

xlim([xmin xmax]);

ylim([ymin ymax]);

xlabel('X');

ylabel('Y');

title('Basic RRT Path Planning');

%% ================================================================
% 10. 绘制障碍物
%% ================================================================

for i = 1:size(obstacles,1)

    rectangle( ...
        'Position',obstacles(i,:), ...
        'FaceColor',[0.55 0.55 0.55], ...
        'EdgeColor','k', ...
        'LineWidth',1.5);

end

%% ================================================================
% 11. 绘制起点和终点
%% ================================================================

plot( ...
    startPoint(1), ...
    startPoint(2), ...
    'go', ...
    'MarkerFaceColor','g', ...
    'MarkerSize',10);

plot( ...
    goalPoint(1), ...
    goalPoint(2), ...
    'ro', ...
    'MarkerFaceColor','r', ...
    'MarkerSize',10);

text( ...
    startPoint(1)+2, ...
    startPoint(2), ...
    'Start');

text( ...
    goalPoint(1)+2, ...
    goalPoint(2), ...
    'Goal');

drawnow;

%% ================================================================
% 12. 开始 RRT 规划
%% ================================================================

fprintf('\n');

fprintf('====================================\n');

fprintf('开始 RRT 路径规划\n');

fprintf('====================================\n');

% 搜索阶段开始计时
searchTimer = tic;

%% ================================================================
% 13. RRT 主循环
%% ================================================================

for iter = 1:maxIterations

    iterationsUsed = iter;

    %% ------------------------------------------------------------
    % 开始本轮纯算法计算计时
    %% ------------------------------------------------------------

    algorithmTimer = tic;

    %% ------------------------------------------------------------
    % Step 1：随机采样
    %% ------------------------------------------------------------

    xRand = [

        xmin + rand*(xmax-xmin), ...

        ymin + rand*(ymax-ymin)

    ];

    xRand = reshape(xRand,1,2);

    %% ------------------------------------------------------------
    % Step 2：寻找距离随机点最近的节点
    %% ------------------------------------------------------------

    dx = nodes(:,1)-xRand(1);

    dy = nodes(:,2)-xRand(2);

    distanceAll = sqrt(dx.^2+dy.^2);

    [~,nearestIndex] = min(distanceAll);

    xNear = nodes(nearestIndex,:);

    xNear = reshape(xNear,1,2);

    %% ------------------------------------------------------------
    % Step 3：朝随机点方向扩展
    %% ------------------------------------------------------------

    direction = xRand-xNear;

    distanceToRand = norm(direction);

    %% 避免除零

    if distanceToRand < 1e-12

        purePlanningTime = ...
            purePlanningTime+toc(algorithmTimer);

        continue;

    end

    %% 固定步长扩展

    moveDistance = min(stepSize,distanceToRand);

    xNew = xNear + ...
        moveDistance*direction/distanceToRand;

    xNew = reshape(xNew,1,2);

    %% ------------------------------------------------------------
    % Step 4：地图边界检测
    %% ------------------------------------------------------------

    if xNew(1) < xmin || ...
       xNew(1) > xmax || ...
       xNew(2) < ymin || ...
       xNew(2) > ymax

        purePlanningTime = ...
            purePlanningTime+toc(algorithmTimer);

        continue;

    end

    %% ------------------------------------------------------------
    % Step 5：碰撞检测
    %% ------------------------------------------------------------

    if ~collisionFree( ...
            xNear, ...
            xNew, ...
            obstacles, ...
            clearance)

        purePlanningTime = ...
            purePlanningTime+toc(algorithmTimer);

        continue;

    end

    %% ------------------------------------------------------------
    % Step 6：将新节点加入随机树
    %% ------------------------------------------------------------

    nodes(end+1,:) = xNew;

    parent(end+1,1) = nearestIndex;

    newIndex = size(nodes,1);

    %% ------------------------------------------------------------
    % Step 7：判断是否接近目标
    %% ------------------------------------------------------------

    distanceToGoal = norm(xNew-goalPoint);

    if distanceToGoal <= goalThreshold

        %% 检查新节点到目标点是否无碰撞

        if collisionFree( ...
                xNew, ...
                goalPoint, ...
                obstacles, ...
                clearance)

            %% 将终点加入随机树

            nodes(end+1,:) = goalPoint;

            parent(end+1,1) = newIndex;

            goalIndex = size(nodes,1);

            goalReached = true;

        end

    end

    %% ------------------------------------------------------------
    % 本轮算法计算结束
    %
    % 注意：
    % 绘图和动画放在计时结束之后。
    %% ------------------------------------------------------------

    purePlanningTime = ...
        purePlanningTime+toc(algorithmTimer);

    %% ============================================================
    % Step 8：动态绘制随机树
    %% ============================================================

    if showSearchAnimation

        plot( ...
            [xNear(1) xNew(1)], ...
            [xNear(2) xNew(2)], ...
            '-', ...
            'Color',[0.45 0.75 0.45], ...
            'LineWidth',0.8);

        plot( ...
            xNew(1), ...
            xNew(2), ...
            '.', ...
            'Color',[0.20 0.55 0.20], ...
            'MarkerSize',6);

        drawnow;

        if treePauseTime > 0

            pause(treePauseTime);

        end

    end

    %% ============================================================
    % Step 9：规划成功
    %% ============================================================

    if goalReached

        %% 绘制最后一条连接边

        if showSearchAnimation

            plot( ...
                [xNew(1) goalPoint(1)], ...
                [xNew(2) goalPoint(2)], ...
                '-', ...
                'Color',[0.45 0.75 0.45], ...
                'LineWidth',0.8);

            drawnow;

        end

        fprintf('\n');

        fprintf('路径规划成功！\n');

        fprintf('迭代次数：%d\n',iter);

        fprintf('随机树节点数量：%d\n',size(nodes,1));

        break;

    end

end

%% ================================================================
% 14. 路径回溯和路径长度计算
%% ================================================================

if goalReached

    %% 路径回溯属于规划计算
    % 单独统计计算时间，不包含绘图

    postTimer = tic;

    currentIndex = goalIndex;

    while currentIndex ~= 0

        path = [

            nodes(currentIndex,:);

            path

        ];

        currentIndex = parent(currentIndex);

    end

    %% ------------------------------------------------------------
    % 计算路径长度
    %% ------------------------------------------------------------

    pathLength = 0;

    for i = 1:size(path,1)-1

        pathLength = pathLength + ...
            norm(path(i+1,:)-path(i,:));

    end

    %% 将路径回溯和长度计算加入纯算法时间

    purePlanningTime = ...
        purePlanningTime+toc(postTimer);

end

%% ================================================================
% 15. 搜索阶段计时结束
%% ================================================================

searchWallTime = toc(searchTimer);

%% ================================================================
% 16. 输出规划结果
%% ================================================================

if goalReached

    fprintf('\n');

    fprintf('====================================\n');

    fprintf('RRT 规划性能统计\n');

    fprintf('====================================\n');

    fprintf('规划成功：是\n');

    fprintf('规划迭代次数：%d\n',iterationsUsed);

    fprintf('随机树节点数量：%d\n',size(nodes,1));

    fprintf('最终路径节点数量：%d\n',size(path,1));

    fprintf('最终路径长度：%.3f\n',pathLength);

else

    fprintf('\n');

    fprintf('====================================\n');

    fprintf('RRT 路径规划失败\n');

    fprintf('====================================\n');

    fprintf('最大迭代次数：%d\n',maxIterations);

    fprintf('随机树节点数量：%d\n',size(nodes,1));

    title('RRT Planning Failed');

end

%% ================================================================
% 17. 绘制最终路径
%% ================================================================

if goalReached

    plot( ...
        path(:,1), ...
        path(:,2), ...
        'b-', ...
        'LineWidth',3);

    plot( ...
        path(:,1), ...
        path(:,2), ...
        'bo', ...
        'MarkerFaceColor','b', ...
        'MarkerSize',4);

    title(sprintf( ...
        'RRT Complete - Planning Time = %.3f ms', ...
        purePlanningTime*1000));

    drawnow;

end

%% ================================================================
% 18. 机器人运动动画
%% ================================================================

if goalReached && showRobotAnimation

    %% 暂停后开始运动动画

    pause(1);

    fprintf('\n');

    fprintf('开始播放机器人运动动画...\n');

    %% 运动动画开始计时

    motionTimer = tic;

    %% ------------------------------------------------------------
    % 创建移动机器人
    %% ------------------------------------------------------------

    hRobot = plot( ...
        path(1,1), ...
        path(1,2), ...
        'mo', ...
        'MarkerFaceColor','m', ...
        'MarkerEdgeColor','k', ...
        'MarkerSize',13, ...
        'LineWidth',1.5);

    %% ------------------------------------------------------------
    % 创建已运动轨迹
    %% ------------------------------------------------------------

    hTrail = plot( ...
        path(1,1), ...
        path(1,2), ...
        'm-', ...
        'LineWidth',2);

    trailX = path(1,1);

    trailY = path(1,2);

    %% ------------------------------------------------------------
    % 沿最终路径逐段运动
    %% ------------------------------------------------------------

    for i = 1:size(path,1)-1

        pStart = path(i,:);

        pEnd = path(i+1,:);

        segmentVector = pEnd-pStart;

        segmentLength = norm(segmentVector);

        %% 当前路径段的动画帧数

        nFrames = max(1,ceil(segmentLength/animationStep));

        %% --------------------------------------------------------
        % 当前路径段插值
        %% --------------------------------------------------------

        for k = 1:nFrames

            alpha = k/nFrames;

            currentPosition = ...
                pStart+alpha*segmentVector;

            %% 更新机器人位置

            set( ...
                hRobot, ...
                'XData',currentPosition(1), ...
                'YData',currentPosition(2));

            %% 更新已走过的轨迹

            trailX(end+1) = currentPosition(1);

            trailY(end+1) = currentPosition(2);

            set( ...
                hTrail, ...
                'XData',trailX, ...
                'YData',trailY);

            %% 刷新 MATLAB 图形

            drawnow;

            %% 控制动画速度

            if robotPauseTime > 0

                pause(robotPauseTime);

            end

        end

    end

    %% ------------------------------------------------------------
    % 运动动画计时结束
    %% ------------------------------------------------------------

    motionTime = toc(motionTimer);

    fprintf('机器人运动动画播放完成！\n');

end

%% ================================================================
% 19. 程序总计时结束
%% ================================================================

totalTime = toc(programTimer);

%% ================================================================
% 20. 最终耗时统计
%% ================================================================

fprintf('\n');

fprintf('====================================\n');

fprintf('最终耗时统计\n');

fprintf('====================================\n');

fprintf('RRT 纯算法时间：%.6f s\n', ...
    purePlanningTime);

fprintf('RRT 纯算法时间：%.3f ms\n', ...
    purePlanningTime*1000);

fprintf('搜索阶段总时间：%.6f s\n', ...
    searchWallTime);

fprintf('机器人运动动画时间：%.6f s\n', ...
    motionTime);

fprintf('程序总运行时间：%.6f s\n', ...
    totalTime);

fprintf('====================================\n');

%% ================================================================
% 21. 保存规划结果
%% ================================================================

result.success = goalReached;

result.path = path;

result.pathLength = pathLength;

result.planningTime = purePlanningTime;

result.searchWallTime = searchWallTime;

result.motionTime = motionTime;

result.totalTime = totalTime;

result.iterations = iterationsUsed;

result.nodes = nodes;

result.parent = parent;

end
