
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
    MotionDirection="forward");

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
