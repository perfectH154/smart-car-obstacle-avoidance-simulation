
function [dFront,dRear] = l150ReplanLidar(pose)

persistent world lidar

if isempty(world)
    world = binaryOccupancyMap(8,6,20);

    % 原来的障碍物
    [X1,Y1] = meshgrid(3.8:0.05:4.2, ...
                       2.7:0.05:3.3);
    setOccupancy(world,[X1(:),Y1(:)],1);

    % 新出现的障碍物
    [X2,Y2] = meshgrid(5.2:0.05:5.6, ...
                       3.1:0.05:3.7);
    setOccupancy(world,[X2(:),Y2(:)],1);

    lidar = rangeSensor;
    lidar.Range = [0.1,3];
    lidar.HorizontalAngle = [-pi,pi];
    lidar.HorizontalAngleResolution = pi/180;
end

[ranges,angles] = lidar(pose,world);

ranges = ranges(:);
angles = angles(:);

valid = isfinite(ranges) & ranges > 0;

% 前方：车头左右15度
frontIdx = valid & abs(angles) <= pi/12;

% 后方：车尾左右15度
rearIdx = valid & abs(abs(angles)-pi) <= pi/12;

dFront = 3;
dRear = 3;

if any(frontIdx)
    dFront = min(ranges(frontIdx));
end

if any(rearIdx)
    dRear = min(ranges(rearIdx));
end

end
