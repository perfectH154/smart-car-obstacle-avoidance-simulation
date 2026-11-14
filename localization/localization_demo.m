
clc;
clear;
close all;

%% 1. 创建二维地图

% 地图尺寸：6m × 4m
% 分辨率：20格/m
grid = zeros(80,120);

% 设置外墙
grid([1 end],:) = 1;
grid(:,[1 end]) = 1;

% 添加固定环境特征
grid(20:30,35:45) = 1;
grid(48:58,78:95) = 1;
grid(12:20,90:96) = 1;

% 创建占据地图
map = binaryOccupancyMap(grid,20);

% 显示地图
figure('Name','Localization Map');
show(map);
title('L150 Localization Map');
axis equal;

% 保存地图
save(fullfile(fileparts(mfilename('fullpath')), ...
    'localization_map.mat'),'map');

%% 2. 创建虚拟激光雷达

% 模拟720方向的二维扫描
sensor = rangeSensor( ...
    'Range',[0.12 4], ...
    'HorizontalAngle',[-pi pi-pi/360], ...
    'HorizontalAngleResolution',pi/360, ...
    'RangeNoise',0.01);

%% 3. 设置真实位置（仅供仿真器使用）

truePose = [1 1 0];

% 在地图中生成虚拟雷达数据
[ranges,angles] = sensor(truePose,map);

% 创建雷达扫描对象
scan = lidarScan(ranges,angles);

% 显示扫描
figure('Name','Simulated Lidar');
plot(scan);
title('Simulated 2D Lidar Scan');
axis equal;

disp('地图与虚拟雷达创建成功');
disp(['扫描点数: ',num2str(numel(ranges))]);

%% 4. 初始化蒙特卡洛定位

rng(7);

mcl = monteCarloLocalization;

mcl.UseLidarScan = true;

% 设置地图
mcl.SensorModel.Map = map;

% 假设雷达安装在车体坐标原点
mcl.SensorModel.SensorPose = [0 0 0];

% 雷达参数
mcl.SensorModel.SensorLimits = [0.12 4];
mcl.SensorModel.NumBeams = 40;
mcl.SensorModel.MeasurementNoise = 0.1;

% 初始位置估计（带少量误差）
mcl.InitialPose = [1.05 2.05 0.05];

mcl.InitialCovariance = diag( ...
    [0.2^2 0.2^2 (10*pi/180)^2]);

% 粒子数量
mcl.ParticleLimits = [250 700];

% 运动模型噪声
mcl.MotionModel.Noise = [0.05 0.05 0.05 0.05];

% 定位更新阈值
mcl.UpdateThresholds = [0.015 0.015 0.025];

%% 5. 模拟小车运动与定位

dt = 0.2;
N = 70;

% 仿真真实位置
truePose = [1 2 0];

% 模拟里程计位置
odomPose = [1 2 0];

% 保存三种轨迹
trueLog = zeros(N,3);
odomLog = zeros(N,3);
estLog = zeros(N,3);

for k = 1:N

    % 小车速度
    v = 0.17;

    % 中间阶段向左转弯
    if k >= 21 && k <= 42
        w = 0.10;
    else
        w = 0;
    end

    if k > 1
        % 真实运动（仅用于仿真）
        truePose(1) = truePose(1) ...
            + v*cos(truePose(3))*dt;
        truePose(2) = truePose(2) ...
            + v*sin(truePose(3))*dt;
        truePose(3) = truePose(3)+w*dt;

        % 模拟带误差的里程计
        vo = v*1.05 + 0.008*randn;
        wo = w + 0.008 + 0.008*randn;

        odomPose(1) = odomPose(1) ...
            + vo*cos(odomPose(3))*dt;
        odomPose(2) = odomPose(2) ...
            + vo*sin(odomPose(3))*dt;
        odomPose(3) = odomPose(3)+wo*dt;
    end

    % 使用真实位置生成模拟雷达观测
    [ranges,angles] = sensor(truePose,map);
    scan = lidarScan(ranges,angles);

    % MCL 只接收里程计与雷达
    [~,estimatedPose,~] = mcl(odomPose,scan);

    % 保存数据
    trueLog(k,:) = truePose;
    odomLog(k,:) = odomPose;
    estLog(k,:) = estimatedPose;

end

%% 6. 绘制定位对比图

figure('Name','MCL Localization');

show(map);
hold on;

plot(trueLog(:,1),trueLog(:,2), ...
    'g-','LineWidth',2);

plot(odomLog(:,1),odomLog(:,2), ...
    'r--','LineWidth',1.5);

plot(estLog(:,1),estLog(:,2), ...
    'b-','LineWidth',2);

legend('Map','True','Odometry','MCL', ...
    'Location','best');

title('L150 Localization Comparison');
xlabel('X (m)');
ylabel('Y (m)');
axis equal;
grid on;

%% 7. 计算定位误差

errMCL = hypot( ...
    estLog(:,1)-trueLog(:,1), ...
    estLog(:,2)-trueLog(:,2));

errOdom = hypot( ...
    odomLog(:,1)-trueLog(:,1), ...
    odomLog(:,2)-trueLog(:,2));

figure('Name','Localization Error');

plot(errOdom,'r--','LineWidth',1.5);
hold on;
plot(errMCL,'b-','LineWidth',2);

legend('Odometry','MCL');
xlabel('Simulation Step');
ylabel('Position Error (m)');
title('Localization Error Comparison');
grid on;

fprintf('MCL 平均误差：%.4f m\n',mean(errMCL));
fprintf('里程计平均误差：%.4f m\n',mean(errOdom));
