function result = Bi_RRT_animation()
%% ================================================================
% 基础 Bi-RRT 双向快速扩展随机树
%
% 功能：
% 1. 起点树和终点树交替生长
% 2. 双向 RRT 路径搜索
% 3. 矩形障碍物碰撞检测
% 4. 双树连接
% 5. 最终路径回溯
% 6. RRT 搜索动画
% 7. 机器人沿路径运动动画
% 8. 规划时间和路径长度统计
%
% 运行：
% >> result = Bi_RRT_animation;
%
% 注意：
% 当前为二维 Bi-RRT，不是 UR5 六维关节空间规划。
%% ================================================================

clc;
close all;

%% ================================================================
% 1. 初始化
%% ================================================================

% 固定随机种子，方便重复实验
rng(1);

% 程序总计时
programTimer = tic;

%% ================================================================
% 2. 地图范围
%% ================================================================

map.xmin = 0;
map.xmax = 100;

map.ymin = 0;
map.ymax = 100;

%% ================================================================
% 3. 起点和终点
%% ================================================================

% 强制使用 1×2 行向量

startPoint = [10, 10];

goalPoint = [90, 90];

startPoint = reshape(startPoint, 1, 2);

goalPoint = reshape(goalPoint, 1, 2);

%% ================================================================
% 4. 障碍物
%% ================================================================

% 每一行表示一个矩形：
%
% [左下角X, 左下角Y, 宽度, 高度]

obstacles = [

    25   0   15   55

    55  45   15   55

];

%% ================================================================
% 5. Bi-RRT 参数
%% ================================================================

% 单次树扩展步长
stepSize = 4;

% 两棵树的连接距离阈值
connectThreshold = stepSize;

% 最大搜索迭代次数
maxIterations = 5000;

% 统一二维碰撞检测安全距离
clearance = 0;

%% ================================================================
% 6. 动画参数
%% ================================================================

% 是否显示随机树搜索动画
showSearchAnimation = true;

% 是否显示机器人运动动画
showRobotAnimation = true;

% 搜索动画暂停时间
treePauseTime = 0.003;

% 机器人运动动画暂停时间
robotPauseTime = 0.01;

% 机器人运动动画插值间隔
animationStep = 0.2;

%% ================================================================
% 7. 初始化两棵随机树
%% ================================================================

% 起点树
Tstart.nodes = startPoint;

Tstart.parent = 0;

% 终点树
Tgoal.nodes = goalPoint;

Tgoal.parent = 0;

% 使用 cell 存储两棵树
trees = {Tstart, Tgoal};

% 两棵树的绘制颜色
treeColors = {

    [0.25 0.70 0.30]    % 起点树：绿色

    [0.10 0.65 0.85]    % 终点树：青色

};

%% ================================================================
% 8. 初始化规划状态
%% ================================================================

goalReached = false;

% 起点树连接节点编号
idxStart = 0;

% 终点树连接节点编号
idxGoal = 0;

% 累计纯算法计算时间
purePlanningTime = 0;

% 搜索阶段总耗时
searchWallTime = 0;

% 运动动画耗时
motionTime = 0;

% 初始化返回结果
result = struct();

result.success = false;

result.path = [];

result.pathLength = NaN;

result.planningTime = NaN;

result.searchWallTime = NaN;

result.motionTime = 0;

result.totalTime = NaN;

result.iterations = 0;

result.startTree = Tstart;

result.goalTree = Tgoal;

%% ================================================================
% 9. 检查起点与终点是否有效
%% ================================================================

if ~collisionFree(startPoint, startPoint, obstacles, clearance)

    error('起点位于障碍物内部，请重新设置起点。');

end

if ~collisionFree(goalPoint, goalPoint, obstacles, clearance)

    error('终点位于障碍物内部，请重新设置终点。');

end

%% ================================================================
% 10. 创建 MATLAB 图形窗口
%% ================================================================

figure( ...
    'Name', 'Bi-RRT Path Planning Animation', ...
    'NumberTitle', 'off', ...
    'Color', 'w');

hold on;

axis equal;

grid on;

box on;

xlim([map.xmin, map.xmax]);

ylim([map.ymin, map.ymax]);

xlabel('X');

ylabel('Y');

title('Bi-RRT Bidirectional Path Planning');

%% 绘制障碍物

for i = 1:size(obstacles,1)

    rectangle( ...
        'Position', obstacles(i,:), ...
        'FaceColor', [0.55 0.55 0.55], ...
        'EdgeColor', 'k', ...
        'LineWidth', 1.5);

end

%% 绘制起点

plot( ...
    startPoint(1), ...
    startPoint(2), ...
    'go', ...
    'MarkerFaceColor', 'g', ...
    'MarkerSize', 10, ...
    'LineWidth', 1.5);

text( ...
    startPoint(1)+2, ...
    startPoint(2), ...
    'Start');

%% 绘制终点

plot( ...
    goalPoint(1), ...
    goalPoint(2), ...
    'ro', ...
    'MarkerFaceColor', 'r', ...
    'MarkerSize', 10, ...
    'LineWidth', 1.5);

text( ...
    goalPoint(1)+2, ...
    goalPoint(2), ...
    'Goal');

drawnow;

%% ================================================================
% 11. 开始路径规划
%% ================================================================

fprintf('\n');

fprintf('====================================\n');

fprintf('开始 Bi-RRT 双向路径规划\n');

fprintf('====================================\n');

searchTimer = tic;

%% ================================================================
% 12. Bi-RRT 主循环
%% ================================================================

for iter = 1:maxIterations

    %% ------------------------------------------------------------
    % Step 1：确定当前主动扩展的树
    %
    % 奇数：起点树主动扩展
    % 偶数：终点树主动扩展
    %% ------------------------------------------------------------

    if mod(iter,2) == 1

        activeTree = 1;

    else

        activeTree = 2;

    end

    % 另一棵树
    otherTree = 3 - activeTree;

    %% ------------------------------------------------------------
    % Step 2：开始算法计算计时
    %% ------------------------------------------------------------

    algorithmTimer = tic;

    %% ------------------------------------------------------------
    % Step 3：随机采样
    %% ------------------------------------------------------------

    xRand = [

        map.xmin + rand*(map.xmax-map.xmin), ...

        map.ymin + rand*(map.ymax-map.ymin)

    ];

    % 强制转换为 1×2 行向量
    xRand = reshape(xRand,1,2);

    %% ------------------------------------------------------------
    % Step 4：主动树向随机采样点扩展
    %% ------------------------------------------------------------

    Ta = trees{activeTree};

    [Ta, newIndexA, addedA] = extendOne( ...
        Ta, ...
        xRand, ...
        stepSize, ...
        obstacles, ...
        clearance);

    % 保存主动树
    trees{activeTree} = Ta;

    %% ------------------------------------------------------------
    % Step 5：主动树扩展失败
    %% ------------------------------------------------------------

    if ~addedA

        % 本轮计算时间
        purePlanningTime = purePlanningTime + toc(algorithmTimer);

        continue;

    end

    %% ------------------------------------------------------------
    % Step 6：获取主动树的新节点
    %% ------------------------------------------------------------

    pointA = Ta.nodes(newIndexA,:);

    pointA = reshape(pointA,1,2);

    %% ------------------------------------------------------------
    % Step 7：另一棵树向主动树的新节点扩展
    %% ------------------------------------------------------------

    Tb = trees{otherTree};

    [Tb, newIndexB, addedB] = extendOne( ...
        Tb, ...
        pointA, ...
        stepSize, ...
        obstacles, ...
        clearance);

    trees{otherTree} = Tb;

    %% ------------------------------------------------------------
    % Step 8：获取另一棵树的连接候选节点
    %% ------------------------------------------------------------

    pointB = Tb.nodes(newIndexB,:);

    pointB = reshape(pointB,1,2);

    %% ------------------------------------------------------------
    % Step 9：判断两棵树是否能够连接
    %% ------------------------------------------------------------

    distanceAB = norm(pointA-pointB);

    canConnect = false;

    if distanceAB <= connectThreshold

        % 两个连接节点之间必须无碰撞

        if collisionFree(pointA,pointB,obstacles,clearance)

            canConnect = true;

        end

    end

    %% ------------------------------------------------------------
    % Step 10：保存连接节点编号
    %% ------------------------------------------------------------

    if canConnect

        goalReached = true;

        % 保证 idxStart 属于起点树
        % 保证 idxGoal 属于终点树

        if activeTree == 1

            idxStart = newIndexA;

            idxGoal = newIndexB;

        else

            idxStart = newIndexB;

            idxGoal = newIndexA;

        end

    end

    %% ------------------------------------------------------------
    % Step 11：结束本轮算法计算计时
    %% ------------------------------------------------------------

    purePlanningTime = purePlanningTime + toc(algorithmTimer);

    %% ============================================================
    % Step 12：绘制主动树
    %
    % 注意：
    % 这里的绘图时间不计入 purePlanningTime。
    %% ============================================================

    if showSearchAnimation

        parentIndexA = Ta.parent(newIndexA);

        pNearA = Ta.nodes(parentIndexA,:);

        pNewA = Ta.nodes(newIndexA,:);

        drawTreeEdge( ...
            pNearA, ...
            pNewA, ...
            treeColors{activeTree}, ...
            treePauseTime);

    end

    %% ============================================================
    % Step 13：绘制另一棵树
    %% ============================================================

    if addedB && showSearchAnimation

        parentIndexB = Tb.parent(newIndexB);

        pNearB = Tb.nodes(parentIndexB,:);

        pNewB = Tb.nodes(newIndexB,:);

        drawTreeEdge( ...
            pNearB, ...
            pNewB, ...
            treeColors{otherTree}, ...
            treePauseTime);

    end

    %% ============================================================
    % Step 14：两棵树连接成功
    %% ============================================================

    if canConnect

        % 绘制两棵树的连接边

        plot( ...
            [pointA(1),pointB(1)], ...
            [pointA(2),pointB(2)], ...
            'b-', ...
            'LineWidth',2);

        drawnow;

        fprintf('\n');

        fprintf('两棵随机树连接成功！\n');

        fprintf('迭代次数：%d\n',iter);

        break;

    end

end

%% ================================================================
% 13. 如果规划失败
%% ================================================================

if ~goalReached

    searchWallTime = toc(searchTimer);

    fprintf('\n');

    fprintf('====================================\n');

    fprintf('Bi-RRT 路径规划失败\n');

    fprintf('====================================\n');

    fprintf('最大迭代次数：%d\n',maxIterations);

    fprintf('纯算法计算时间：%.6f s\n',purePlanningTime);

    fprintf('搜索阶段总时间：%.6f s\n',searchWallTime);

    result.success = false;

    result.planningTime = purePlanningTime;

    result.searchWallTime = searchWallTime;

    result.totalTime = toc(programTimer);

    result.iterations = maxIterations;

    result.startTree = trees{1};

    result.goalTree = trees{2};

    title('Bi-RRT Planning Failed');

    return;

end

%% ================================================================
% 14. 获取最终两棵树
%% ================================================================

Tstart = trees{1};

Tgoal = trees{2};

%% ================================================================
% 15. 开始路径回溯计时
%% ================================================================

postTimer = tic;

%% ================================================================
% 16. 起点树路径回溯
%% ================================================================

% 从起点到起点树的连接节点

pathStart = backtrackTree(Tstart,idxStart);

%% ================================================================
% 17. 终点树路径回溯
%% ================================================================

% 从终点到终点树的连接节点

pathGoal = backtrackTree(Tgoal,idxGoal);

%% ================================================================
% 18. 将终点树路径反向
%% ================================================================

% 原始：
% Goal -> 连接点
%
% 反向：
% 连接点 -> Goal

pathGoalReverse = flipud(pathGoal);

%% ================================================================
% 19. 拼接两棵树的路径
%% ================================================================

% 如果两个连接节点完全重合
% 删除其中一个重复点

if norm(pathStart(end,:)-pathGoalReverse(1,:)) < 1e-10

    pathGoalReverse(1,:) = [];

end

% 拼接最终完整路径
%
% Start -> 起点树连接点
%       -> 终点树连接点
%       -> Goal

path = [

    pathStart;

    pathGoalReverse

];

%% ================================================================
% 20. 检查最终路径
%% ================================================================

% 验证路径起点

if norm(path(1,:)-startPoint) > 1e-8

    error('最终路径起点错误，请检查路径回溯。');

end

% 验证路径终点

if norm(path(end,:)-goalPoint) > 1e-8

    error('最终路径终点错误，请检查路径回溯。');

end

% 逐段验证路径没有碰撞

for i = 1:size(path,1)-1

    if ~collisionFree( ...
            path(i,:), ...
            path(i+1,:), ...
            obstacles, ...
            clearance)

        error('最终路径第 %d 段发生碰撞。',i);

    end

end

%% ================================================================
% 21. 计算最终路径长度
%% ================================================================

pathLength = 0;

for i = 1:size(path,1)-1

    segmentLength = norm(path(i+1,:)-path(i,:));

    pathLength = pathLength + segmentLength;

end

%% 将路径回溯和验证时间计入算法时间

purePlanningTime = purePlanningTime + toc(postTimer);

%% 搜索阶段总时间

searchWallTime = toc(searchTimer);

%% ================================================================
% 22. 输出规划性能
%% ================================================================

fprintf('\n');

fprintf('====================================\n');

fprintf('Bi-RRT 规划性能统计\n');

fprintf('====================================\n');

fprintf('规划成功：是\n');

fprintf('规划迭代次数：%d\n',iter);

fprintf('起点树节点数量：%d\n',size(Tstart.nodes,1));

fprintf('终点树节点数量：%d\n',size(Tgoal.nodes,1));

fprintf('随机树总节点数量：%d\n', ...
    size(Tstart.nodes,1)+size(Tgoal.nodes,1));

fprintf('最终路径节点数量：%d\n',size(path,1));

fprintf('最终路径长度：%.3f\n',pathLength);

fprintf('\n');

fprintf('纯算法规划时间：%.6f s\n',purePlanningTime);

fprintf('纯算法规划时间：%.3f ms\n',purePlanningTime*1000);

fprintf('含搜索动画的阶段总时间：%.6f s\n',searchWallTime);

%% ================================================================
% 23. 绘制最终路径
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

title(sprintf( ...
    'Bi-RRT Complete - Planning Time = %.3f ms', ...
    purePlanningTime*1000));

drawnow;

%% ================================================================
% 24. 机器人运动动画
%% ================================================================

if showRobotAnimation

    pause(0.8);

    fprintf('\n');

    fprintf('开始播放机器人运动动画...\n');

    motionTimer = tic;

    %% ------------------------------------------------------------
    % 创建机器人移动点
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
    % 创建已经走过的轨迹
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

        % 当前路径段起点
        pStart = path(i,:);

        % 当前路径段终点
        pEnd = path(i+1,:);

        % 当前路径段向量
        segmentVector = pEnd-pStart;

        % 当前路径段长度
        segmentLength = norm(segmentVector);

        %% 计算动画插值帧数

        nFrames = max(1,ceil(segmentLength/animationStep));

        %% 当前路径段的运动动画

        for k = 1:nFrames

            alpha = k/nFrames;

            %% 当前机器人位置

            currentPosition = ...
                pStart + alpha*segmentVector;

            %% 强制为 1×2 行向量

            currentPosition = reshape(currentPosition,1,2);

            %% 更新机器人坐标

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

            %% 更新 MATLAB 图形

            drawnow;

            %% 控制运动动画速度

            if robotPauseTime > 0

                pause(robotPauseTime);

            end

        end

    end

    %% 运动动画耗时

    motionTime = toc(motionTimer);

    fprintf('机器人运动动画播放完成！\n');

end

%% ================================================================
% 25. 程序总耗时
%% ================================================================

totalTime = toc(programTimer);

fprintf('\n');

fprintf('====================================\n');

fprintf('最终耗时统计\n');

fprintf('====================================\n');

fprintf('Bi-RRT 纯算法时间：%.6f s\n',purePlanningTime);

fprintf('Bi-RRT 纯算法时间：%.3f ms\n',purePlanningTime*1000);

fprintf('搜索阶段总时间：%.6f s\n',searchWallTime);

fprintf('机器人运动动画时间：%.6f s\n',motionTime);

fprintf('程序总运行时间：%.6f s\n',totalTime);

fprintf('====================================\n');

%% ================================================================
% 26. 保存规划结果
%% ================================================================

result.success = true;

result.path = path;

result.pathLength = pathLength;

result.planningTime = purePlanningTime;

result.searchWallTime = searchWallTime;

result.motionTime = motionTime;

result.totalTime = totalTime;

result.iterations = iter;

result.startTree = Tstart;

result.goalTree = Tgoal;

end


%% =================================================================
% 局部函数 1：随机树单步扩展
%
% 已修复行向量与列向量不一致的问题。
%% =================================================================

function [tree,newIndex,added] = extendOne( ...
    tree, ...
    target, ...
    stepSize, ...
    obstacles, ...
    clearance)

%% 初始化

added = false;

%% ------------------------------------------------------------
% 1. 统一坐标维度
%% ------------------------------------------------------------

% 所有节点必须为 N×2 矩阵

if size(tree.nodes,2) ~= 2

    error('随机树节点矩阵必须为 N×2。');

end

% 目标点强制转换成 1×2 行向量

target = reshape(target,1,2);

%% ------------------------------------------------------------
% 2. 搜索最近节点
%% ------------------------------------------------------------

% differences 为 N×2

differences = tree.nodes-target;

% 每个节点到目标点的平方距离

distances = sum(differences.^2,2);

% 寻找最近节点

[~,nearestIndex] = min(distances);

% 最近节点

xNear = tree.nodes(nearestIndex,:);

% 强制为 1×2

xNear = reshape(xNear,1,2);

% 默认返回最近节点编号

newIndex = nearestIndex;

%% ------------------------------------------------------------
% 3. 计算扩展方向
%% ------------------------------------------------------------

direction = target-xNear;

% 确保 direction 为 1×2

direction = reshape(direction,1,2);

distance = norm(direction);

%% 避免两个点重合导致除零

if distance < 1e-12

    return;

end

%% ------------------------------------------------------------
% 4. 计算扩展步长
%% ------------------------------------------------------------

% 不超过固定步长
% 也不超过到目标点的实际距离

moveDistance = min(stepSize,distance);

%% ------------------------------------------------------------
% 5. 生成新节点
%% ------------------------------------------------------------

xNew = xNear + ...
    (moveDistance/distance)*direction;

% 强制转换为 1×2 行向量

xNew = reshape(xNew,1,2);

%% ------------------------------------------------------------
% 6. 碰撞检测
%% ------------------------------------------------------------

if ~collisionFree(xNear,xNew,obstacles,clearance)

    return;

end

%% ------------------------------------------------------------
% 7. 新节点加入随机树
%% ------------------------------------------------------------

tree.nodes(end+1,:) = xNew;

%% ------------------------------------------------------------
% 8. 保存父节点编号
%% ------------------------------------------------------------

tree.parent(end+1,1) = nearestIndex;

%% ------------------------------------------------------------
% 9. 返回新节点编号
%% ------------------------------------------------------------

newIndex = size(tree.nodes,1);

added = true;

end


%% =================================================================
% 局部函数 2：父节点回溯
%
% 输入：
% tree：随机树
% index：目标节点编号
%
% 输出：
% path：根节点到目标节点的路径
%% =================================================================

function path = backtrackTree(tree,index)

path = [];

while index ~= 0

    %% 当前节点坐标

    currentNode = tree.nodes(index,:);

    %% 强制转换成 1×2

    currentNode = reshape(currentNode,1,2);

    %% 插入路径最前面

    path = [

        currentNode;

        path

    ];

    %% 转移到父节点

    index = tree.parent(index);

end

end


%% =================================================================
% 局部函数 3：绘制随机树
%% =================================================================

function drawTreeEdge(p1,p2,color,pauseTime)

%% 统一坐标维度

p1 = reshape(p1,1,2);

p2 = reshape(p2,1,2);

%% 绘制树枝

plot( ...
    [p1(1),p2(1)], ...
    [p1(2),p2(2)], ...
    '-', ...
    'Color',color, ...
    'LineWidth',0.8);

%% 更新 MATLAB 图形

drawnow;

%% 控制动画速度

if pauseTime > 0

    pause(pauseTime);

end

end
