function map = L150LocalizationMap()
% Localization map: 8 m x 6 m arena, 0.10 m boundary walls,
% the fixed L150 obstacle, and one asymmetric fixed radar landmark.
% The landmark is a simulation assumption used to remove mirror ambiguity.
map = binaryOccupancyMap(8,6,20);

x = 0.025:0.05:7.975;
yWall = [0.025 0.075 5.925 5.975];
[X,Y] = meshgrid(x,yWall);
setOccupancy(map,[X(:),Y(:)],1);

y = 0.025:0.05:5.975;
xWall = [0.025 0.075 7.925 7.975];
[X,Y] = meshgrid(xWall,y);
setOccupancy(map,[X(:),Y(:)],1);

% Fixed obstacle from the existing L150 simulation.
[X,Y] = meshgrid(3.8:0.05:4.2,2.7:0.05:3.3);
setOccupancy(map,[X(:),Y(:)],1);

% Asymmetric landmark near the upper-left boundary (not on the planned route).
[X,Y] = meshgrid(0.40:0.05:0.55,5.00:0.05:5.15);
setOccupancy(map,[X(:),Y(:)],1);
end
