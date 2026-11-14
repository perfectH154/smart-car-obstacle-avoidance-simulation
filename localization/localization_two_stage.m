
clc;
clear;
close all;
rng(7);

%% 1. 创建已知地图

grid = zeros(80,120);

grid([1 end],:) = 1;
grid(:,[1 end]) = 1;

grid(20:30,35:45) = 1;
grid(48:58,78:95) = 1;
grid(12:20,90:96) = 1;
grid(36:47,11:16) = 1;
grid(7:12,55:65) = 1;
grid(62:68,101:110) = 1;

map = binaryOccupancyMap(grid,20);

%% 2. 模拟真实小车与雷达

% 真实起点仅供仿真器使用
trueStart = [1 0.7 0];

sensor = rangeSensor( ...
    'Range',[0.12 4], ...
    'HorizontalAngle',[-pi pi-pi/360], ...
    'HorizontalAngleResolution',pi/360, ...
    'RangeNoise',0.01);

% 生成第一帧雷达观测
[r0,a0] = sensor(trueStart,map);

% 粗搜索使用72条光束，提高速度
idx = 1:10:720;
observed = r0(idx);
observed(~isfinite(observed)) = 4;

% 用于地图匹配的无噪声虚拟传感器
searchSensor = rangeSensor( ...
    'Range',[0.12 4], ...
    'HorizontalAngle',[-pi pi-pi/36], ...
    'HorizontalAngleResolution',pi/36, ...
    'RangeNoise',0);

%% 3. 第一阶段：全地图粗搜索

disp('开始全地图雷达匹配...');

bestScore = inf;
initPose = [0 0 0];

for x = 0.5:0.35:5.7
    for y = 0.45:0.35:3.6

        % 排除障碍物内部
        if getOccupancy(map,[x y]) ~= 0
            continue;
        end

        for theta = -pi:pi/6:(pi-pi/6)

            candidate = [x y theta];

            [pred,~] = searchSensor(candidate,map);
            pred(~isfinite(pred)) = 4;

            score = mean(abs(pred(:)-observed(:)));

            if score < bestScore
                bestScore = score;
                initPose = candidate;
            end
        end
    end
end

fprintf('粗搜索结果：%.3f, %.3f, %.2f deg\n', ...
    initPose(1),initPose(2),rad2deg(initPose(3)));

%% 4. 第二轮：局部精细搜索

coarsePose = initPose;
bestScore = inf;

for x = coarsePose(1)-0.35:0.0875:coarsePose(1)+0.35
    for y = coarsePose(2)-0.35:0.0875:coarsePose(2)+0.35

        if x <= 0.15 || x >= 5.85 || ...
           y <= 0.15 || y >= 3.85
            continue;
        end

        if getOccupancy(map,[x y]) ~= 0
            continue;
        end

        for da = deg2rad(-20:5:20)

            theta = coarsePose(3)+da;
            candidate = [x y theta];

            [pred,~] = searchSensor(candidate,map);
            pred(~isfinite(pred)) = 4;

            score = mean(abs(pred(:)-observed(:)));

            if score < bestScore
                bestScore = score;
                initPose = candidate;
            end
        end
    end
end

fprintf('精细搜索结果：%.3f, %.3f, %.2f deg\n', ...
    initPose(1),initPose(2),rad2deg(initPose(3)));

fprintf('雷达匹配平均距离差：%.4f m\n',bestScore);

%% 5. 创建局部 MCL 定位器

mcl = monteCarloLocalization;

mcl.UseLidarScan = true;
mcl.SensorModel.Map = map;
mcl.SensorModel.SensorPose = [0 0 0];

mcl.SensorModel.SensorLimits = [0.12 4];
mcl.SensorModel.NumBeams = 80;
mcl.SensorModel.MeasurementNoise = 0.08;

% 初始位置来自雷达搜索，不是真实位置
mcl.GlobalLocalization = false;
mcl.InitialPose = initPose;

mcl.InitialCovariance = diag( ...
    [0.15^2 0.15^2 deg2rad(12)^2]);

mcl.ParticleLimits = [600 1800];

mcl.MotionModel.Noise = ...
    [0.05 0.05 0.05 0.05];

mcl.UpdateThresholds = [0.015 0.015 0.025];

%% 6. 运动仿真与持续定位

dt = 0.2;
N = 150;

truePose = trueStart;

% 里程计在自身局部坐标系中积分
odomPose = [0 0 0];

trueLog = zeros(N,3);
odomLog = zeros(N,3);
estLog = zeros(N,3);

for k = 1:N

    v = 0.16;

    if k <= 30
        w = 0;
    elseif k <= 65
        w = 0.20;
    elseif k <= 100
        w = -0.17;
    else
        w = 0;
    end

    if k > 1

        % 真实运动
        th = truePose(3)+w*dt/2;

        truePose(1) = truePose(1)+v*cos(th)*dt;
        truePose(2) = truePose(2)+v*sin(th)*dt;
        truePose(3) = truePose(3)+w*dt;

        % 带误差的里程计
 vo = v*1.01 + 0.008*randn;
wo = w + 0.001 + 0.008*randn;

        tho = odomPose(3)+wo*dt/2;

        odomPose(1) = odomPose(1)+vo*cos(tho)*dt;
        odomPose(2) = odomPose(2)+vo*sin(tho)*dt;
        odomPose(3) = odomPose(3)+wo*dt;
    end

    % 从真实位置生成模拟雷达数据
    [ranges,angles] = sensor(truePose,map);
    scan = lidarScan(ranges,angles);

    % MCL只接收里程计和扫描
    [~,estimate,~] = mcl(odomPose,scan);

    trueLog(k,:) = truePose;
    odomLog(k,:) = odomPose;
    estLog(k,:) = estimate;
end

%% 7. 轨迹比较

% 只为评价绘图对齐里程计起点
% 不反馈给定位算法
odomShow = odomLog;

th0 = trueStart(3);
R = [cos(th0) -sin(th0); ...
     sin(th0)  cos(th0)];

odomShow(:,1:2) = ...
    (R*odomLog(:,1:2)')' + trueStart(1:2);

odomShow(:,3) = odomLog(:,3)+th0;

figure('Name','Two Stage Localization');

show(map);
hold on;

h1 = plot(trueLog(:,1),trueLog(:,2), ...
    'g-','LineWidth',2);

h2 = plot(odomShow(:,1),odomShow(:,2), ...
    'r--','LineWidth',1.5);

h3 = plot(estLog(:,1),estLog(:,2), ...
    'b-','LineWidth',2);

h4 = plot(initPose(1),initPose(2), ...
    'mo','MarkerSize',9,'LineWidth',2);

legend([h1 h2 h3 h4], ...
    {'True','Odometry','MCL','Initial Estimate'});

title('Two-Stage Localization');
xlabel('X (m)');
ylabel('Y (m)');
axis equal;
grid on;

%% 8. 误差分析

errMCL = hypot( ...
    estLog(:,1)-trueLog(:,1), ...
    estLog(:,2)-trueLog(:,2));

errOdom = hypot( ...
    odomShow(:,1)-trueLog(:,1), ...
    odomShow(:,2)-trueLog(:,2));

t = (0:N-1)*dt;

figure('Name','Localization Error');

plot(t,errMCL,'b-','LineWidth',2);
hold on;

plot(t,errOdom,'r--','LineWidth',1.5);

legend('MCL','Odometry');
xlabel('Time (s)');
ylabel('Position Error (m)');
title('Localization Error');
grid on;

%% 9. 粒子检查

[particles,weights] = getParticles(mcl);

inside = particles(:,1)>=0 & particles(:,1)<=6 & ...
         particles(:,2)>=0 & particles(:,2)<=4 & ...
         all(isfinite(particles),2);

figure('Name','Final Particle Distribution');

show(map);
hold on;

if any(inside)
    scatter(particles(inside,1), ...
            particles(inside,2), ...
            7,'b','filled');
end

plot(trueLog(:,1),trueLog(:,2), ...
    'g-','LineWidth',2);

plot(estLog(:,1),estLog(:,2), ...
    'r-','LineWidth',2);

title('Final Particle Distribution');
axis equal;
grid on;

%% 10. 输出结果

fprintf('\n===== 两阶段定位结果 =====\n');

fprintf('MCL平均误差：%.4f m\n',mean(errMCL));
fprintf('里程计平均误差：%.4f m\n',mean(errOdom));

fprintf('MCL最终误差：%.4f m\n',errMCL(end));

fprintf('地图内粒子：%d / %d\n', ...
    sum(inside),size(particles,1));

fprintf('真实终点：%.3f, %.3f\n', ...
    truePose(1),truePose(2));

fprintf('估计终点：%.3f, %.3f\n', ...
    estLog(end,1),estLog(end,2));
