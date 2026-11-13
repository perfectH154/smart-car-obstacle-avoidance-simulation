
% 1. 创建新的规划地图
newMap = binaryOccupancyMap(8, 6, 20);

% 2. 添加原有障碍物
[X1,Y1] = meshgrid(3.8:0.05:4.2, ...
    2.7:0.05:3.3);
setOccupancy(newMap,[X1(:),Y1(:)],1);

% 3. 添加新出现的障碍物
[X2,Y2] = meshgrid(5.2:0.05:5.6, ...
    3.1:0.05:3.7);
setOccupancy(newMap,[X2(:),Y2(:)],1);

% 4. 给障碍物留出30厘米规划余量
inflate(newMap,0.30);

% 5. 创建路径规划空间
ssNew = stateSpaceSE2;

ssNew.StateBounds = [newMap.XWorldLimits;
    newMap.YWorldLimits;
    -pi pi];

% 6. 创建碰撞检查器
validatorNew = validatorOccupancyMap(ssNew);
validatorNew.Map = newMap;
validatorNew.ValidationDistance = 0.05;

% 7. 创建 Hybrid A* 规划器

plannerNew = plannerHybridAStar(validatorNew, ...
    MinTurningRadius=0.96, ...
    MotionPrimitiveLength=0.2, ...
    InterpolationDistance=0.05, ...
    MotionDirection="forward-reverse", ...
    ReverseCost=5, ...
    DirectionSwitchingCost=5);


% 8. 用小车停车位置作为新起点
startNew = stopPose;

% 终点不变
goalNew = [7,3,0];

% 9. 检查停车位置是否位于障碍区域
if checkOccupancy(newMap,startNew(1:2))
    error('停车位置位于膨胀障碍区，请检查停车时机');
end

% 10. 重新规划路线
[pathNew,dirsNew,infoNew] = ...
    plan(plannerNew,startNew,goalNew);

% 11. 画出新地图和路线
figure;
show(newMap);
hold on;

if infoNew.IsPathFound
    waypointsNew = pathNew.States(:,1:2);
    plot(waypointsNew(:,1),waypointsNew(:,2), ...
        'r-','LineWidth',2);
    disp('重新规划成功！');
else
    disp('没有找到新的可行路线');
end

plot(startNew(1),startNew(2),'go');
plot(goalNew(1),goalNew(2),'bx');

axis equal;
title('L150 Dynamic Replanning');

% 根据 Hybrid A* 的方向生成速度计划

% 提取新路线
waypointsNew = pathNew.States(:,1:2);

% 将方向数据整理成列向量
dirsNew = dirsNew(:);

% 创建速度数组
vPlan = zeros(size(dirsNew));

% 前进段
vPlan(dirsNew == 1) = 0.12;

% 倒车段
vPlan(dirsNew == -1) = -0.08;

% 找到前进与倒车切换的位置
switchIdx = find(diff(dirsNew) ~= 0) + 1;

% 显示信息
fprintf('前进采样点：%d\n', sum(dirsNew == 1));
fprintf('倒车采样点：%d\n', sum(dirsNew == -1));
fprintf('方向切换次数：%d\n', numel(switchIdx));

% 绘制速度计划
figure;
stairs(1:numel(vPlan), vPlan, 'LineWidth', 2);
hold on;
xline(switchIdx, '--r');
grid on;

xlabel('路径采样点编号');
ylabel('计划速度 (m/s)');
title('L150 前进与倒车速度计划');

% 单独保存新规划结果
save('L150_Replan_Path.mat', ...
    'waypointsNew', 'dirsNew', ...
    'vPlan', 'switchIdx');

% 根据方向变化划分路径段
changeIdx = find(diff(dirsNew) ~= 0) + 1;

starts = [1; changeIdx];
ends = [changeIdx - 1; length(dirsNew)];

numSegments = length(starts);

% 保存每段路径及对应行驶方向
segmentPaths = cell(numSegments,1);
segmentDirs = zeros(numSegments,1);

for k = 1:numSegments

    s = starts(k);
    e = ends(k);

    % 保留相邻两段之间的连接位置
    if k > 1
        s = s - 1;
    end

    segmentPaths{k} = pathNew.States(s:e,1:2);
    segmentDirs(k) = dirsNew(starts(k));

    fprintf('第%d段：方向=%d，路径点=%d\n', ...
        k, segmentDirs(k), size(segmentPaths{k},1));
end

% 绘制分段路线
figure;
show(newMap);
hold on;

for k = 1:numSegments

    P = segmentPaths{k};

    if segmentDirs(k) == 1
        plot(P(:,1),P(:,2),'b-', ...
            'LineWidth',2);
    else
        plot(P(:,1),P(:,2),'r-', ...
            'LineWidth',2);
    end
end

axis equal;
title('蓝色前进 / 红色倒车');
