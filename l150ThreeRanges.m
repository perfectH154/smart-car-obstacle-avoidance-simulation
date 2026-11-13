
function [dLeft, dFront, dRight] = l150ThreeRanges(pose)
% 输入：小车位姿 [x, y, theta]
% 输出：左、前、右三个方向的最近障碍物距离

persistent map lidar

% 第一次运行时创建地图和虚拟雷达
if isempty(map)

    map = binaryOccupancyMap(8, 6, 20);

    % 创建矩形障碍物
    [X, Y] = meshgrid(3.8:0.05:4.2, ...
                  2.7:0.05:3.3);

    setOccupancy(map, [X(:), Y(:)], 1);

    % 创建雷达
    lidar = rangeSensor;
    lidar.Range = [0, 3];
    lidar.HorizontalAngle = [-pi, pi];
    lidar.HorizontalAngleResolution = pi/180;
end

% 根据小车当前位姿扫描
[ranges, angles] = lidar(pose, map);

ranges = ranges(:);
angles = angles(:);

% 排除无效距离
valid = isfinite(ranges) & ranges > 0;

% 选取三个检测方向

% 左侧：15°～50°
leftIdx = valid & angles >= deg2rad(15) ...
                & angles <= deg2rad(50);

% 正前方：-15°～15°
frontIdx = valid & abs(angles) <= deg2rad(15);

% 右侧：-50°～-15°
rightIdx = valid & angles >= deg2rad(-50) ...
                 & angles <= deg2rad(-15);

% 默认没有检测到障碍物时返回3米
dLeft = 3;
dFront = 3;
dRight = 3;

% 分别寻找三个区域的最近距离
if any(leftIdx)
    dLeft = min(ranges(leftIdx));
end

if any(frontIdx)
    dFront = min(ranges(frontIdx));
end

if any(rightIdx)
    dRight = min(ranges(rightIdx));
end

end
