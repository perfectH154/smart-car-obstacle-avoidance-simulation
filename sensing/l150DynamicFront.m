
function d = l150DynamicFront(pose, t)
% pose：小车当前位姿 [x,y,theta]
% t：当前仿真时间
% d：正前方最近的障碍物距离

persistent worldMap lidar newAdded lastT

% 第一次调用或重新开始仿真时，初始化地图
if isempty(worldMap) || ...
        (~isempty(lastT) && t < lastT - 1e-6)

    worldMap = binaryOccupancyMap(8, 6, 20);

    % 原有障碍物
    [X,Y] = meshgrid(3.8:0.05:4.2, ...
                     2.7:0.05:3.3);
    setOccupancy(worldMap,[X(:),Y(:)],1);

    % 创建虚拟雷达
    lidar = rangeSensor;
    lidar.Range = [0.1, 3];
    lidar.HorizontalAngle = [-pi, pi];
    lidar.HorizontalAngleResolution = pi/180;

    newAdded = false;
end

lastT = t;

% 第15秒：场地里出现新的障碍物
if t >= 15 && ~newAdded

    [X2,Y2] = meshgrid(5.2:0.05:5.6, ...
                       3.1:0.05:3.7);

    setOccupancy(worldMap,[X2(:),Y2(:)],1);

    newAdded = true;
end

% 根据实时位置和当前地图进行扫描
[ranges, angles] = lidar(pose, worldMap);

ranges = ranges(:);
angles = angles(:);

% 只检测车头左右15度
frontIdx = abs(angles) <= deg2rad(15);
front = ranges(frontIdx);

% 排除无效测距
front = front(isfinite(front) & front > 0);

% 输出最近距离
if isempty(front)
    d = 3;
else
    d = min(front);
end

end
