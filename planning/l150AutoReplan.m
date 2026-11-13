
function [routeData,routeDirs,nSeg,ok] = ...
    l150AutoReplan(stopPose)

% 输入：小车停车时的 [x y theta]
% 输出：重新规划后的分段路线和方向

% 固定输出数组大小，方便后续 Simulink 接线
maxPoints = 256;
maxSegments = 10;

routeData = zeros(maxPoints,2,maxSegments);
routeDirs = zeros(maxSegments,1);
nSeg = 0;
ok = 0;

% 1. 创建地图
newMap = binaryOccupancyMap(8,6,20);

% 原障碍物
[X1,Y1] = meshgrid(3.8:0.05:4.2, ...
                   2.7:0.05:3.3);
setOccupancy(newMap,[X1(:),Y1(:)],1);

% 新障碍物
[X2,Y2] = meshgrid(5.2:0.05:5.6, ...
                   3.1:0.05:3.7);
setOccupancy(newMap,[X2(:),Y2(:)],1);

% 2. 设置安全膨胀
inflate(newMap,0.30);

% 3. 创建规划器
ss = stateSpaceSE2;
ss.StateBounds = [0 8;0 6;-pi pi];

validator = validatorOccupancyMap(ss);
validator.Map = newMap;
validator.ValidationDistance = 0.05;

planner = plannerHybridAStar(validator, ...
    MinTurningRadius=0.96, ...
    MotionPrimitiveLength=0.2, ...
    InterpolationDistance=0.05, ...
    MotionDirection="forward-reverse", ...
    ReverseCost=5, ...
    DirectionSwitchingCost=5);

% 4. 停车位置作为规划起点
start = stopPose(:)';
goal = [7,3,0];

if ~isStateValid(validator,start)
    return;
end

% 5. 自动调用 Hybrid A*
[path,dirs,info] = plan(planner,start,goal);

if ~info.IsPathFound
    return;
end

% 6. 按前进、倒车方向分段
P = path.States(:,1:2);
dirs = dirs(:);

changeIdx = find(diff(dirs) ~= 0) + 1;

starts = [1;changeIdx];
ends = [changeIdx-1;length(dirs)];

nSeg = length(starts);

if nSeg > maxSegments
    nSeg = 0;
    return;
end

for k = 1:nSeg

    s = starts(k);
    e = ends(k);

    if k > 1
        s = s-1;
    end

    part = P(s:e,:);

    n = size(part,1);

    if n > maxPoints
        nSeg = 0;
        return;
    end

    % 后面不足的路径点用终点补齐
    routeData(:,:,k) = ...
        repmat(part(end,:),maxPoints,1);

    routeData(1:n,:,k) = part;

    routeDirs(k) = dirs(starts(k));
end

% 规划成功
ok = 1;

end
