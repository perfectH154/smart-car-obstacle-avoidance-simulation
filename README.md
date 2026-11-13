# L150 Ackermann 小车避障与路径规划仿真

这里整理了几份 MATLAB/Simulink 小车仿真文件：主模型做 Ackermann 小车避障，旁边还有一个早期模型、虚拟雷达测试，以及一段基于 Hybrid A* 的路径规划实验。雷达和障碍物都在 MATLAB 里生成，当前内容是仿真，不是实车传感器或固件。

## 文件结构

- `ackermannbizhang.slx`：主避障模型，根据虚拟雷达的左、前、右测距结果控制转向或停车。
- `l150ThreeRanges.m`：主模型调用的虚拟雷达函数，返回三个方向最近的障碍物距离。
- `L150_Collision_Check.m`：可选的轨迹间隙检查脚本，需要工作区里有 `xy` 矩阵（第一列 x、第二列 y，单位米）。车身按半径 0.20 m 的圆近似，结果用于快速检查。
- `models/ackermann02.slx`：较早的 Ackermann 避障模型版本，可与主模型对照。
- `models/L150_Path_Follow.slx`：带路径跟踪、动态雷达避障和安全停车子系统的模型；它调用 `l150DynamicFront.m`，同时包含原有的三向测距逻辑。
- `models/L150_Reverse_Test.slx`：前进/倒车避障测试模型，使用虚拟雷达分别测量车头和车尾方向。
- `models/L150_Auto_Segments.slx`：路径分段和行驶方向管理模型，用于按前进、倒车路段组织路线。
- `planning/L150_Path_Plannin.m`：在占据栅格地图上运行 Hybrid A* 规划并绘制路径。
- `planning/L150_Path.mat`：保存的路径数据，包含 `waypoints`（126×2）、`directions`（126×1）、`startPose` 和 `goalPose`。
- `planning/L150_Replan_Test.m`：在出现新障碍物后用 Hybrid A* 重新规划，允许前进和倒车，并按行驶方向生成速度计划；运行前需要工作区里有停车位姿 `stopPose`。
- `planning/L150_Replan_Path.mat`：重新规划脚本的结果文件，保存新航点、行驶方向、速度计划和方向切换位置。
- `sensing/LidarTest.m`：独立的虚拟雷达场景测试，绘制地图、扫描结果和前向距离曲线。
- `sensing/l150FrontRange.m`：返回虚拟雷达正前方左右各 10° 范围内的最近障碍物距离。
- `sensing/l150DynamicFront.m`：在仿真进行到 15 秒时向地图加入新障碍物，并返回车头左右各 15° 内的最近测距。
- `sensing/l150ReplanLidar.m`：为重新规划模型提供虚拟雷达的前向和后向最近测距。

## 运行

在 MATLAB 中打开 `ackermannbizhang.slx`，并把仓库根目录加入 MATLAB Path，然后运行模型。路径跟踪、倒车测试和自动分段模型也可分别从 `models/` 打开；它们会调用 `sensing/` 中相应的测距函数。路径规划脚本和雷达测试脚本可以分别打开运行；它们各自创建示例地图。重新规划脚本需要先在工作区提供 `stopPose`，运行后会保存 `planning/L150_Replan_Path.mat`。

主模型文件由 MATLAB R2026a 保存。模型和脚本用到 Ackermann 运动学、占据栅格地图、距离传感器及 Hybrid A* 等组件；若 MATLAB 提示缺少组件，请检查对应工具箱是否已安装。路径规划脚本会生成新路径；`L150_Path.mat` 保存的是一组已有起终点和航点数据，主避障模型不会自动加载它。

## 轨迹间隙检查

准备好轨迹后，在 MATLAB 工作区创建 `xy = [x(:), y(:)]`，再运行：

```matlab
L150_Collision_Check
```

脚本会输出估算的最小安全间隙并画图。它没有完整考虑车身形状，也不是安全认证或真实碰撞验证。

## 范围

仓库只收录仿真模型、配套 MATLAB 代码和路径数据，不含实车固件、雷达驱动或硬件测试记录。
