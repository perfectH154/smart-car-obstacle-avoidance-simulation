
clear; clc; close all;

% 1. 创建之前的3m×3m地图
map = binaryOccupancyMap(8, 6, 20);

% 2. 放置矩形障碍物
[X,Y] = meshgrid(3.8:0.05:4.2, ...
                 2.7:0.05:3.3);
setOccupancy(map,[X(:),Y(:)],1);

% 3. 给障碍物留出安全间隙
inflate(map,0.30);

% 4. 建立小车的位置和朝向空间
ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits;
    map.YWorldLimits;
    -pi pi];

% 5. 创建碰撞检查器
validator = validatorOccupancyMap(ss);
validator.Map = map;
validator.ValidationDistance = 0.05;

% 6. 创建考虑转弯半径的路径规划器

planner = plannerHybridAStar(validator, ...
    MinTurningRadius=0.96, ...
    MotionPrimitiveLength=0.2, ...
    InterpolationDistance=0.05, ...
    MotionDirection="forward");

% 7. 设置起点和终点
startPose = [1.0, 3.0, 0];
goalPose  = [7.0, 3.0, 0];

% 8. 计算路径
[path, directions, info] = ...
    plan(planner,startPose,goalPose);

% 9. 显示结果
figure;
show(map);
hold on;

if info.IsPathFound
    plot(path.States(:,1),path.States(:,2), ...
        'b-','LineWidth',2);
    title('L150 Planned Path');
else
    title('No feasible path found');
    disp('当前约束下没有找到可行路径');
end

plot(startPose(1),startPose(2),'go');
plot(goalPose(1),goalPose(2),'rx');
axis equal;
