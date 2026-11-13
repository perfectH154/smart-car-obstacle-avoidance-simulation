
% L150 第四关：虚拟激光雷达测试
clear; clc; close all;

% 1. 创建 3m × 3m 的二维地图
map = binaryOccupancyMap(3, 3, 20);

% 2. 放置一个矩形障碍物
[X, Y] = meshgrid(1.5:0.05:1.7, 1.2:0.05:1.8);
setOccupancy(map, [X(:), Y(:)], 1);

% 3. 设置小车位置和朝向
pose = [1.0, 1.5, 0];

% 4. 创建虚拟二维激光雷达
lidar = rangeSensor;
lidar.Range = [0, 3];
lidar.HorizontalAngle = [-pi, pi];
lidar.HorizontalAngleResolution = pi/180;

% 5. 雷达扫描
[ranges, angles] = lidar(pose, map);

% 选取车头正前方左右各10度的雷达数据
frontIdx = abs(angles) <= pi/18;

% 提取有效的前方距离数据
frontRanges = ranges(frontIdx & isfinite(ranges) & ranges > 0);

% 找出前方最近的障碍物
if isempty(frontRanges)
    d = inf;
else
    d = min(frontRanges);
end

% 根据距离判断小车是否需要停车
if d <= 0.4
    disp("STOP! 前方有障碍物");
else
    disp("GO! 可以继续前进");
end


% 6. 显示地图
figure;
show(map);
hold on;
plot(pose(1), pose(2), 'ro', ...
    'MarkerFaceColor', 'r');
title('L150 Virtual Environment');

% 7. 显示雷达扫描结果
scan = lidarScan(ranges, angles);
figure;
plot(scan);
title('L150 Virtual Lidar Scan');

% 8. 显示正前方的测距结果
[~, idx] = min(abs(angles));
fprintf('Front distance: %.2f m\n', ranges(idx));


% 在不同位置模拟雷达测距

radarX = 0.5:0.02:1.2;
radarD = zeros(size(radarX));

for k = 1:length(radarX)

    % 把虚拟雷达放到新的位置
    poseNow = [radarX(k), 1.5, 0];

    % 扫描障碍物
    [r, a] = lidar(poseNow, map);

    % 只取正前方左右10度的扫描数据
    front = r(abs(a) <= pi/18);

    % 去掉无效数据
    front = front(isfinite(front) & front > 0);

    % 保存最近障碍物距离
    if isempty(front)
        radarD(k) = 3;
    else
        radarD(k) = min(front);
    end
end

% 绘制距离随位置变化的曲线
figure;
plot(radarX, radarD, 'o-');
grid on;
xlabel('小车位置 x (m)');
ylabel('前方障碍物距离 d (m)');
title('虚拟雷达测距结果');

