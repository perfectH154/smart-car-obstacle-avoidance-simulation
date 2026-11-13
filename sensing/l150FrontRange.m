
function d = l150FrontRange(pose)
% 输入：小车位置和朝向 [x, y, theta]
% 输出：正前方最近障碍物距离 d

persistent map lidar

% 第一次调用时创建地图和雷达
if isempty(map)
    map = binaryOccupancyMap(3, 3, 20);

    % 在地图中放置矩形障碍物
    [X, Y] = meshgrid(1.5:0.05:1.7, ...
                      1.2:0.05:1.8);
    setOccupancy(map, [X(:), Y(:)], 1);

    % 创建虚拟雷达
    lidar = rangeSensor;
    lidar.Range = [0, 3];
    lidar.HorizontalAngle = [-pi, pi];
    lidar.HorizontalAngleResolution = pi/180;
end

% 根据当前位姿重新扫描
[ranges, angles] = lidar(pose, map);

% 将扫描数据整理成列向量
ranges = ranges(:);
angles = angles(:);

% 只保留正前方左右10度的数据
front = ranges(abs(angles) <= pi/18);

% 删除无效测距
front = front(isfinite(front) & front > 0);

% 返回最近障碍物的距离
if isempty(front)
    d = 3;
else
    d = min(front);
end

end
