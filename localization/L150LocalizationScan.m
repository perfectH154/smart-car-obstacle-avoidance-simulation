function [ranges,angles] = L150LocalizationScan(pose,reset)
% Full 360 deg scan for localization; returns fixed column vectors.
if nargin < 2
    reset = false;
end
persistent world lidar
if isempty(world) || reset
    world = L150LocalizationMap();

    % Temporary obstacle: present in the scan but absent from the known map.
    [X,Y] = meshgrid(5.2:0.05:5.6,3.1:0.05:3.7);
    setOccupancy(world,[X(:),Y(:)],1);

    lidar = rangeSensor( ...
        'Range',[0.1 4], ...
        'HorizontalAngle',[-pi pi], ...
        'HorizontalAngleResolution',pi/180, ...
        'RangeNoise',0.01);
end
[scan,scanAngles] = lidar(reshape(pose,1,3),world);
ranges = scan(:);
angles = scanAngles(:);
end
