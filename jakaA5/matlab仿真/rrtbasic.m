function RRT_basic_animation
%% ================================================================
% 基础二维 RRT 路径规划 + 搜索动画 + 路径运动动画
%
% 直接运行：
% >> RRT_basic_animation
%
% 绿色树枝：RRT 搜索树
% 灰色区域：障碍物
% 蓝色粗线：最终规划路径
% 紫色圆点：沿最终路径运动的机器人
%% ================================================================

clc;
close all;

%% 固定随机种子
% 每次运行结果基本一致
rng(1);

%% ================================================================
% 1. 地图范围
%% ================================================================

xmin = 0;
xmax = 100;

ymin = 0;
ymax = 100;

%% ================================================================
% 2. 起点与终点
%% ================================================================

startPoint = [10 10];
goalPoint  = [90 90];

%% ================================================================
% 3. 障碍物
%
% 格式：
% [左下角X 左下角Y 宽度 高度]
%% ================================================================

obstacles = [
    25   0   15   55
    55  45   15   55
];

%% ================================================================
% 4. RRT 参数
%% ================================================================

stepSize = 4;             % 每次扩展步长

goalThreshold = 5;        % 距离终点小于该值时尝试连接

maxIterations = 5000;     % 最大迭代次数

collisionResolution = 0.2; % 碰撞检测分辨率

%% ================================================================
% 5. 动画速度
%% ================================================================

% RRT 树生长速度
treePauseTime = 0.003;

% 最终运动动画速度
robotPauseTime = 0.01;

% 最终运动插值距离
animationStep = 0.2;

%% ================================================================
% 6. 初始化 RRT 树
%% ================================================================

% nodes 每一行表示一个节点 [x y]
nodes = startPoint;

% parent(i) 为第 i 个节点的父节点
parent = 0;

goalReached = false;

goalIndex = -1;

%% ================================================================
% 7. 创建图形窗口
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
% 8. 绘制障碍物
%% ================================================================

for i = 1:size(obstacles,1)

    rectangle( ...
        'Position', obstacles(i,:), ...
        'FaceColor',[0.55 0.55 0.55], ...
        'EdgeColor','k', ...
        'LineWidth',1.5);

end

%% ================================================================
% 9. 绘制起点
%% ================================================================

plot( ...
    startPoint(1), ...
    startPoint(2), ...
    'go', ...
    'MarkerFaceColor','g', ...
    'MarkerSize',10, ...
    'LineWidth',1.5);

text( ...
    startPoint(1)+2, ...
    startPoint(2), ...
    'Start', ...
    'FontSize',10);

%% ================================================================
% 10. 绘制终点
%% ================================================================

plot( ...
    goalPoint(1), ...
    goalPoint(2), ...
    'ro', ...
    'MarkerFaceColor','r', ...
    'MarkerSize',10, ...
    'LineWidth',1.5);

text( ...
    goalPoint(1)+2, ...
    goalPoint(2), ...
    'Goal', ...
    'FontSize',10);

drawnow;

fprintf('\n');
fprintf('====================================\n');
fprintf('开始 RRT 路径规划\n');
fprintf('====================================\n');

%% ================================================================
% 11. RRT 主循环
%% ================================================================

for iter = 1:maxIterations

    %% ------------------------------------------------------------
    % Step 1：随机采样
    %% ------------------------------------------------------------

    xRand = [
        xmin + rand*(xmax-xmin), ...
        ymin + rand*(ymax-ymin)
    ];

    %% ------------------------------------------------------------
    % Step 2：寻找距离随机点最近的树节点
    %% ------------------------------------------------------------

    dx = nodes(:,1) - xRand(1);
    dy = nodes(:,2) - xRand(2);

    distanceAll = sqrt(dx.^2 + dy.^2);

    [~,nearestIndex] = min(distanceAll);

    xNear = nodes(nearestIndex,:);

    %% ------------------------------------------------------------
    % Step 3：沿随机采样点方向扩展
    %% ------------------------------------------------------------

    direction = xRand - xNear;

    distanceToRand = norm(direction);

    if distanceToRand < 1e-9
        continue;
    end

    direction = direction / distanceToRand;

    if distanceToRand <= stepSize

        xNew = xRand;

    else

        xNew = xNear + stepSize*direction;

    end

    %% ------------------------------------------------------------
    % Step 4：边界检测
    %% ------------------------------------------------------------

    if xNew(1) < xmin || ...
       xNew(1) > xmax || ...
       xNew(2) < ymin || ...
       xNew(2) > ymax

        continue;

    end

    %% ------------------------------------------------------------
    % Step 5：碰撞检测
    %% ------------------------------------------------------------

    if ~collisionFree( ...
            xNear, ...
            xNew, ...
            obstacles, ...
            collisionResolution)

        continue;

    end

    %% ------------------------------------------------------------
    % Step 6：将节点加入随机树
    %% ------------------------------------------------------------

    nodes(end+1,:) = xNew;

    parent(end+1) = nearestIndex;

    newIndex = size(nodes,1);

    %% ------------------------------------------------------------
    % Step 7：动态绘制随机树
    %% ------------------------------------------------------------

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

    %% ------------------------------------------------------------
    % Step 8：判断是否接近目标
    %% ------------------------------------------------------------

    distanceToGoal = norm(xNew-goalPoint);

    if distanceToGoal <= goalThreshold

        %% 检查新节点是否能直接连接目标

        if collisionFree( ...
                xNew, ...
                goalPoint, ...
                obstacles, ...
                collisionResolution)

            %% 将目标节点加入树

            nodes(end+1,:) = goalPoint;

            parent(end+1) = newIndex;

            goalIndex = size(nodes,1);

            %% 绘制最后一条连接边

            plot( ...
                [xNew(1) goalPoint(1)], ...
                [xNew(2) goalPoint(2)], ...
                '-', ...
                'Color',[0.45 0.75 0.45], ...
                'LineWidth',0.8);

            drawnow;

            goalReached = true;

            fprintf('\n');
            fprintf('路径规划成功！\n');
            fprintf('迭代次数：%d\n',iter);
            fprintf('树节点数量：%d\n',size(nodes,1));

            break;

        end

    end

end

%% ================================================================
% 12. 判断规划是否成功
%% ================================================================

if ~goalReached

    fprintf('\n');
    fprintf('路径规划失败！\n');
    fprintf('最大迭代次数：%d\n',maxIterations);

    title('RRT Planning Failed');

    return;

end

%% ================================================================
% 13. 根据父节点回溯最终路径
%% ================================================================

path = [];

currentIndex = goalIndex;

while currentIndex ~= 0

    path = [
        nodes(currentIndex,:);
        path
    ];

    currentIndex = parent(currentIndex);

end

%% ================================================================
% 14. 绘制最终路径
%% ================================================================

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

drawnow;

%% ================================================================
% 15. 计算路径长度
%% ================================================================

pathLength = 0;

for i = 1:size(path,1)-1

    pathLength = pathLength + ...
        norm(path(i+1,:) - path(i,:));

end

fprintf('最终路径节点数量：%d\n',size(path,1));
fprintf('最终路径长度：%.2f\n',pathLength);

title(sprintf( ...
    'RRT Planning Success - Path Length = %.2f', ...
    pathLength));

%% ================================================================
% 16. 暂停后开始运动动画
%% ================================================================

pause(1);

fprintf('\n');
fprintf('开始播放运动动画...\n');

%% ================================================================
% 17. 创建移动机器人
%% ================================================================

hRobot = plot( ...
    path(1,1), ...
    path(1,2), ...
    'mo', ...
    'MarkerFaceColor','m', ...
    'MarkerEdgeColor','k', ...
    'MarkerSize',13, ...
    'LineWidth',1.5);

%% 创建已运动轨迹

hTrail = plot( ...
    path(1,1), ...
    path(1,2), ...
    'm-', ...
    'LineWidth',2);

trailX = path(1,1);
trailY = path(1,2);

%% ================================================================
% 18. 沿最终路径运动
%% ================================================================

for i = 1:size(path,1)-1

    pStart = path(i,:);
    pEnd   = path(i+1,:);

    segmentVector = pEnd - pStart;

    segmentLength = norm(segmentVector);

    nFrames = max(2,ceil(segmentLength/animationStep));

    for k = 1:nFrames

        alpha = k/nFrames;

        currentPosition = ...
            pStart + alpha*segmentVector;

        %% 更新机器人位置

        set( ...
            hRobot, ...
            'XData',currentPosition(1), ...
            'YData',currentPosition(2));

        %% 更新机器人已经走过的轨迹

        trailX(end+1) = currentPosition(1);
        trailY(end+1) = currentPosition(2);

        set( ...
            hTrail, ...
            'XData',trailX, ...
            'YData',trailY);

        drawnow;

        if robotPauseTime > 0
            pause(robotPauseTime);
        end

    end

end

%% ================================================================
% 19. 完成
%% ================================================================

fprintf('运动动画播放完成！\n');

title(sprintf( ...
    'RRT Complete - Path Length = %.2f', ...
    pathLength));

end


%% =================================================================
% 碰撞检测函数
%% =================================================================

function free = collisionFree( ...
    p1, ...
    p2, ...
    obstacles, ...
    resolution)

free = true;

%% 两点间距离

segmentLength = norm(p2-p1);

%% 根据距离确定检测点数量

numPoints = max( ...
    2, ...
    ceil(segmentLength/resolution));

%% 生成线段检测点

x = linspace(p1(1),p2(1),numPoints);

y = linspace(p1(2),p2(2),numPoints);

%% 检查每一个矩形障碍物

for i = 1:size(obstacles,1)

    obsX = obstacles(i,1);

    obsY = obstacles(i,2);

    obsW = obstacles(i,3);

    obsH = obstacles(i,4);

    %% 判断检测点是不是进入障碍物

    insideX = ...
        x >= obsX & ...
        x <= obsX + obsW;

    insideY = ...
        y >= obsY & ...
        y <= obsY + obsH;

    if any(insideX & insideY)

        free = false;

        return;

    end

end

end