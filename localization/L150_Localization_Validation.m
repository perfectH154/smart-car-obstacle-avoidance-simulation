function result = L150_Localization_Validation()
% Validate one-shot map matching using the same 8 m x 6 m L150 map.
rng(7);
map = L150LocalizationMap();
truePose = [1,3,0];
maxRange = 4;

% Obtain the full simulated scan. Use every fifth beam for the coarse search.
[fullScan,fullAngles] = L150LocalizationScan(truePose,true);
sampleIndex = 1:5:numel(fullScan);
observed = fullScan(sampleIndex);
angles = fullAngles(sampleIndex); %#ok<NASGU>
observed(~isfinite(observed)) = maxRange;

searchSensor = rangeSensor('Range',[0.1 maxRange], ...
    'HorizontalAngle',[-pi pi], ...
    'HorizontalAngleResolution',5*pi/180, ...
    'RangeNoise',0);

% Coarse global scan matching.
coarsePoses = zeros(0,3);
coarseScores = zeros(0,1);
for x = 0.5:0.4:7.5
    for y = 0.5:0.4:5.5
        if getOccupancy(map,[x y]) > 0.5
            continue
        end
        for theta = -pi:pi/12:(pi-pi/12)
            pose = [x y theta];
            score = scanScore(pose,searchSensor,map,observed,maxRange);
            coarsePoses(end+1,:) = pose; %#ok<AGROW>
            coarseScores(end+1,1) = score; %#ok<AGROW>
        end
    end
end
[sortedScores,order] = sort(coarseScores);
coarsePose = coarsePoses(order(1),:);

% Fine scan matching around the best coarse candidate.
finePoses = zeros(0,3);
fineScores = zeros(0,1);
for x = max(0.25,coarsePose(1)-0.4):0.05:min(7.75,coarsePose(1)+0.4)
    for y = max(0.25,coarsePose(2)-0.4):0.05:min(5.75,coarsePose(2)+0.4)
        if getOccupancy(map,[x y]) > 0.5
            continue
        end
        for da = -15:3:15
            pose = [x y wrapAngle(coarsePose(3)+deg2rad(da))];
            score = scanScore(pose,searchSensor,map,observed,maxRange);
            finePoses(end+1,:) = pose; %#ok<AGROW>
            fineScores(end+1,1) = score; %#ok<AGROW>
        end
    end
end
[bestScore,bestIndex] = min(fineScores);
estPose = finePoses(bestIndex,:);
posError = norm(estPose(1:2)-truePose(1:2));
angleError = abs(wrapAngle(estPose(3)-truePose(3)));

fprintf('真值位姿: x=%.2f m, y=%.2f m, theta=%.1f deg\n', ...
    truePose(1),truePose(2),rad2deg(truePose(3)));
fprintf('地图匹配: x=%.2f m, y=%.2f m, theta=%.1f deg\n', ...
    estPose(1),estPose(2),rad2deg(estPose(3)));
fprintf('误差: %.3f m / %.2f deg；扫描残差: %.3f m\n', ...
    posError,rad2deg(angleError),bestScore);
fprintf('粗搜索前三个残差: %.3f, %.3f, %.3f m\n', ...
    sortedScores(1),sortedScores(2),sortedScores(3));

figure('Name','L150 地图定位初始匹配');
show(map); hold on;
plot(truePose(1),truePose(2),'go','MarkerFaceColor','g','MarkerSize',8);
plot(estPose(1),estPose(2),'rx','LineWidth',2,'MarkerSize',10);
quiver(truePose(1),truePose(2),0.45*cos(truePose(3)), ...
    0.45*sin(truePose(3)),0,'g','LineWidth',1.5);
quiver(estPose(1),estPose(2),0.45*cos(estPose(3)), ...
    0.45*sin(estPose(3)),0,'r','LineWidth',1.5);
legend('真值位置','雷达匹配估计','真值朝向','估计朝向','Location','best');
title(sprintf('初始定位误差 %.3f m / %.1f deg',posError,rad2deg(angleError)));
axis equal; grid on; hold off;

result.truePose = truePose;
result.estimatedPose = estPose;
result.positionError = posError;
result.angleErrorDeg = rad2deg(angleError);
result.scanResidual = bestScore;
result.map = map;
end

function score = scanScore(pose,sensor,map,observed,maxRange)
predicted = sensor(pose,map);
predicted = predicted(:);
predicted(~isfinite(predicted)) = maxRange;
score = mean(abs(predicted-observed(:)));
end

function angle = wrapAngle(angle)
angle = atan2(sin(angle),cos(angle));
end
