
function [routes,dirs,nSeg,mode] = ...
    l150SenseAndPlanStep(pose,t)

% mode:
% 0 原路线行驶
% 1 停车等待
% 2 执行重新规划路线
% 3 规划失败

persistent world knownMap lidar
persistent added oldT phase switchTime
persistent savedRoutes savedDirs savedN

% 初始化，或重新启动仿真
if isempty(oldT) || t < oldT - 1e-6

    world = binaryOccupancyMap(8,6,20);
    knownMap = binaryOccupancyMap(8,6,20);

    % 原有障碍物：场景和已知地图都包含
    [X,Y] = meshgrid(3.8:0.05:4.2, ...
                     2.7:0.05:3.3);

    pts = [X(:),Y(:)];

    setOccupancy(world,pts,1);
    setOccupancy(knownMap,pts,1);

    lidar = rangeSensor;
    lidar.Range = [0.1,3];
    lidar.HorizontalAngle = [-pi,pi];
    lidar.HorizontalAngleResolution = pi/180;

    added = false;
    phase = 0;
    switchTime = 0;

    savedRoutes = zeros(256,2,10);
    savedDirs = zeros(10,1);
    savedN = 0;
end

oldT = t;

% 第15秒：真实场景出现新障碍物
% 注意：不添加到小车已知地图
if t >= 15 && ~added

    [X2,Y2] = meshgrid(5.2:0.05:5.6, ...
                       3.1:0.05:3.7);

    setOccupancy(world,[X2(:),Y2(:)],1);

    added = true;
end

% 虚拟雷达连续扫描真实场景
[ranges,angles] = lidar(pose(:)',world);

ranges = ranges(:);
angles = angles(:);

valid = isfinite(ranges) & ...
        ranges >= 0.1 & ranges < 2.99;

r = ranges(valid);
a = angles(valid);

% 测距转为世界坐标
hitX = pose(1) + r.*cos(a+pose(3));
hitY = pose(2) + r.*sin(a+pose(3));

inside = hitX > 0 & hitX < 8 & ...
         hitY > 0 & hitY < 6;

hits = [hitX(inside),hitY(inside)];
r = r(inside);
a = a(inside);

% 识别尚未记录的障碍物点
newHit = false(size(r));

if ~isempty(hits)

    occupied = checkOccupancy(knownMap,hits);

    newHit = occupied == 0;

    % 只有雷达测量结果才能更新已知地图
    setOccupancy(knownMap,hits,1);
end



% 排除原有障碍物附近的测距点
% 避免地图栅格边界误差触发重新规划

isOldObstacle = ...
    hits(:,1) >= 3.65 & hits(:,1) <= 4.35 & ...
    hits(:,2) >= 2.55 & hits(:,2) <= 3.45;

% 只使用原障碍物区域之外的新测距点触发
detected = any(newHit & ~isOldObstacle & r <= 2.5);



% 雷达触发自动规划
if phase == 0 && t >= 15 && detected

    fprintf('雷达触发时间：%.2f s，位置：(%.2f, %.2f)\n', ...
        t, pose(1), pose(2));

    phase = 1;
    switchTime = t;

    [R,D,N,ok] = ...
        l150AutoReplanSensed(pose(:)',knownMap);

    % 保存自动规划结果，方便检查
    assignin('base','debugRoutes',R);
    assignin('base','debugDirs',D);
    assignin('base','debugN',N);
    assignin('base','debugStart',pose(:)');

    if ok == 1 && N > 0
        savedRoutes = R;
        savedDirs = D;
        savedN = N;
    else
        phase = 3;
    end
end

% 停车0.6秒后执行新路线
if phase == 1 && t-switchTime >= 0.6
    phase = 2;
end

routes = savedRoutes;
dirs = savedDirs;
nSeg = savedN;
mode = phase;

end
