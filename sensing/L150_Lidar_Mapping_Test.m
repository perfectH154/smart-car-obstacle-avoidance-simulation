
%% L150 雷达自主发现障碍物测试
% 真实场景只提供给虚拟雷达
% 小车地图初始只知道原障碍物

clearvars;
clc;

%% 1. 创建真实场景
world = binaryOccupancyMap(8,6,20);

% 原障碍物
[X1,Y1] = meshgrid(3.8:0.05:4.2, ...
    2.7:0.05:3.3);
setOccupancy(world,[X1(:),Y1(:)],1);

% 新障碍物在第15秒出现
t = 16;

if t >= 15
    [X2,Y2] = meshgrid(5.2:0.05:5.6, ...
        3.1:0.05:3.7);
    setOccupancy(world,[X2(:),Y2(:)],1);
end

%% 2. 创建小车已知的地图
knownMap = binaryOccupancyMap(8,6,20);

% 注意：只添加原障碍物！
setOccupancy(knownMap,[X1(:),Y1(:)],1);

%% 3. 模拟一次360度雷达扫描
lidar = rangeSensor;
lidar.Range = [0.1,3];
lidar.HorizontalAngle = [-pi,pi];
lidar.HorizontalAngleResolution = pi/180;

% 测试时的小车位姿
pose = [4.8,3.9,0];

[ranges,angles] = lidar(pose,world);

ranges = ranges(:);
angles = angles(:);

%% 4. 将雷达测量转换为地图坐标
valid = isfinite(ranges) & ...
    ranges >= 0.1 & ranges < 2.99;

r = ranges(valid);
a = angles(valid) + pose(3);

hitX = pose(1) + r.*cos(a);
hitY = pose(2) + r.*sin(a);

hitPoints = [hitX,hitY];

% 排除地图范围以外的点
inside = hitX > 0 & hitX < 8 & ...
    hitY > 0 & hitY < 6;

hitPoints = hitPoints(inside,:);

%% 5. 根据雷达观测更新小车地图
% 查询哪些位置以前没有记录为障碍物
wasOccupied = checkOccupancy(knownMap,hitPoints);

newHits = hitPoints(wasOccupied == 0,:);

% 只用实际测量结果更新地图
setOccupancy(knownMap,hitPoints,1);

%% 6. 可视化
figure;

subplot(1,2,1);
show(world);
hold on;
plot(pose(1),pose(2),'bo', ...
    'MarkerFaceColor','b');
axis equal;
title('真实仿真场景');

subplot(1,2,2);
show(knownMap);
hold on;
plot(pose(1),pose(2),'bo', ...
    'MarkerFaceColor','b');
axis equal;
title('雷达更新后的小车地图');

fprintf('有效障碍物测距点：%d\n', ...
    size(hitPoints,1));

fprintf('原地图未记录的测距点：%d\n', ...
    size(newHits,1));
