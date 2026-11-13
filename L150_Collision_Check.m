
% L150 车身碰撞距离检查

% 1. 读取小车的实际轨迹
x = xy(:,1);
y = xy(:,2);

% 2. 障碍物边界
xmin = 3.8;
xmax = 4.2;
ymin = 2.7;
ymax = 3.3;

% 3. 计算每个位置到矩形障碍物的距离
dx = max(max(xmin-x, 0), x-xmax);
dy = max(max(ymin-y, 0), y-ymax);

distance = sqrt(dx.^2 + dy.^2);

% 4. 找到最小距离
[dmin, idx] = min(distance);

% 5. 用圆形保护区域近似车身
safetyRadius = 0.20;

% 6. 计算剩余安全间隙
clearance = dmin - safetyRadius;

% 7. 输出结果
fprintf('最小参考点距离：%.3f m\n', dmin);
fprintf('估算安全间隙：%.3f m\n', clearance);

if clearance <= 0
    disp('警告：可能发生碰撞！');
else
    disp('在当前近似下未发生碰撞。');
end

% 8. 标记最接近障碍物的位置
figure;
rectangle('Position',[3.8,2.7,0.4,0.6], ...
    'FaceColor',[0.7 0.7 0.7]);
hold on;
plot(x,y,'r-','LineWidth',2);
plot(x(idx),y(idx),'bo','MarkerSize',9);
axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('L150 Collision Distance Check');
