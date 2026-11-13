# L150 Autonomous Navigation & Obstacle Avoidance Simulation

基于 MATLAB/Simulink 的阿克曼转向小车自主导航与激光雷达避障仿真项目。

## 1. 项目概述（Overview）

项目以 L150 阿克曼转向小车为研究对象，基于 MATLAB/Simulink 搭建车辆运动学模型，并结合虚拟激光雷达、二维占用栅格地图、Hybrid A* 路径规划和路径跟踪算法，实现静态障碍物环境下的自主导航，以及新增障碍物场景中的感知、地图更新、自动重新规划和路径执行。

目前项目主要完成仿真验证，尚未进行实车系统联调。

## 2. 已完成功能（Features）

### 2.1 车辆运动学建模

- 搭建 Ackermann Kinematic Model。
- 根据车辆轴距建立运动学关系。
- 实现速度、前轮转向角和转向角速度控制。
- 支持前进和倒车两种运动模式。

### 2.2 激光雷达感知与地图构建

- 使用 `rangeSensor` 模拟二维激光雷达扫描。
- 根据测距和扫描角度计算障碍物坐标。
- 使用 `binaryOccupancyMap` 构建二维栅格地图。
- 在闭环模型中区分真实仿真环境与车辆已知地图。
- 根据新增障碍物的雷达观测更新已知地图。

### 2.3 路径规划与重新规划

- 使用 Hybrid A* 进行路径规划，并考虑车辆最小转弯半径。
- 对障碍物设置规划安全膨胀区域。
- 支持前进和倒车混合规划。
- 检测到新增障碍物后，基于已更新的地图生成新路线。

### 2.4 路径跟踪与运动控制

- 基于 Pure Pursuit 几何思想实现路径跟踪。
- 实现前进、倒车转向控制。
- 根据规划方向自动分段。
- 实现停车换挡和自动切换路段。
- 实现到达当前路段终点及最终目标点后的停车控制。

### 2.5 自动避障闭环仿真

- 沿初始规划路径行驶。
- 在仿真过程中引入新增障碍物。
- 通过虚拟雷达识别新增障碍物并更新已知地图。
- 自动触发 Hybrid A* 重新规划。
- 跟踪新路线，绕过障碍物并驶向目标点。

## 3. 系统架构（System Architecture）

仿真环境 → 激光雷达扫描 → 障碍物坐标计算 → 已知地图更新 → 重新规划触发 → Hybrid A* → 路径分段管理 → 路径跟踪控制器 → Ackermann 运动学模型 → 状态反馈。

## 4. 仿真实验与结果（Experiments & Results）

### 实验一：静态障碍物路径规划

验证 Hybrid A* 在已知地图中规划无碰撞路径的能力。

### 实验二：前进与倒车路径跟踪

验证分段路径执行、正反向运动与自动换挡功能。

### 实验三：新增障碍物自动重新规划

验证虚拟雷达观测、已知地图更新、规划触发和路线切换流程。

### 实验四：终点精度验证

在预设新障碍物的自动重规划测试中，测得终点位置误差为 **0.079 m**。该数值对应预设障碍物地图的重规划测试；雷达感知地图驱动的闭环场景应以该仿真的最新测量结果单独记录。

### 实验五：转向约束验证

- 最大前轮转向角：**0.1677 rad**；设定上限：0.25 rad。
- 最大转向角速度：**0.1496 rad/s**；设定上限：0.35 rad/s。
- 检查结果：**满足仿真转向约束**。

## 5. 开发环境（Environment）

- MATLAB R2026a
- Simulink
- Robotics System Toolbox
- Navigation Toolbox
- 二维占用栅格地图与虚拟激光雷达
- Ackermann Kinematic Model

## 6. 核心文件说明（Project Files）

### 闭环模型与路径跟踪

- `models/L150_Closed_Loop.slx`：基于雷达感知地图的自主避障闭环模型。
- `models/L150_Closed_Loop2.slx`：预设障碍物地图的闭环重规划版本，可与感知地图版对照。
- `models/L150_Auto_Segments.slx`：分段路径跟踪与自动换挡模型。
- `models/L150_Path_Follow.slx`：基础路径跟踪模型。
- `models/L150_Reverse_Test.slx`：倒车跟踪测试模型。
- `models/ackermann02.slx`：较早的 Ackermann 避障模型，可作对照。
- `ackermannbizhang.slx`：主避障模型，根据虚拟雷达的左、前、右测距结果控制转向或停车。

### 感知与规划函数

- `planning/l150SenseAndPlanStep.m`：执行雷达扫描、已知地图更新和自动规划触发。
- `planning/l150AutoReplanSensed.m`：基于雷达更新后的地图运行 Hybrid A*，并整理前进/倒车分段路线。
- `planning/l150AutoReplan.m`：基于预设障碍物地图进行自动重新规划。
- `sensing/l150DynamicFront.m`：动态加入障碍物，并计算车头方向的虚拟雷达测距。
- `sensing/l150ReplanLidar.m`：计算车头和车尾方向的虚拟雷达测距。
- `l150ThreeRanges.m`：返回主避障模型所需的左、前、右三个方向最近距离。
- `sensing/l150FrontRange.m`：返回车头左右各 10° 范围内的最近距离。

### 测试与数据

- `sensing/L150_Lidar_Mapping_Test.m`：单次雷达扫描与占用地图更新测试。
- `sensing/LidarTest.m`：独立的虚拟雷达场景测试。
- `planning/L150_Replan_Test.m`：新增障碍物后的 Hybrid A* 重新规划测试；运行前需在工作区提供停车位姿 `stopPose`。
- `planning/L150_Path_Plannin.m`：在占用栅格地图上运行 Hybrid A* 并绘制路径。
- `planning/L150_Path.mat`：已有路径数据，包含 `waypoints`、`directions`、`startPose` 和 `goalPose`。
- `planning/L150_Replan_Path.mat`：重规划结果，包含新航点、行驶方向、速度计划和方向切换位置。
- `L150_Collision_Check.m`：轨迹参考点的安全间隙初步检查；需要工作区中的 `xy` 矩阵（第一列为 x，第二列为 y，单位米），车身以半径 0.20 m 的圆近似。

## 7. 当前局限性（Limitations）

- 目前使用理想化雷达及车辆运动学模型。
- 新障碍物检测包含针对测试场景的已知障碍物区域过滤。
- 尚未完成完整车身碰撞及扫掠空间验证。
- 路径跟踪调试阶段曾临时关闭距离触发的安全停车，实车应用前必须完善。
- 尚未完成实车定位、控制接口和雷达数据通信。

## 8. 后续工作（Future Work）

- 完善多帧激光雷达观测融合与通用障碍物识别。
- 优化运动安全控制与路径跟踪精度。
- 开展 L150 实车联调。
- 实现移动目标识别、相对距离估计与自动跟车控制。
- 研究基于地图的车辆定位与导航。

## 9. 项目状态

**当前阶段：自主避障仿真核心功能已完成，正在推进算法完善与实车验证。**

## 运行说明

在 MATLAB 中进入仓库根目录并运行 `addpath(genpath(pwd))`，让 `planning/` 和 `sensing/` 中的辅助函数也加入 MATLAB Path，再打开所需模型。模型使用 Ackermann 运动学、占用栅格地图、距离传感器及 Hybrid A* 等组件；若 MATLAB 提示缺少组件，请检查对应工具箱是否已安装。

当前仓库只收录仿真模型、配套 MATLAB 代码和仿真路径数据，不包含实车固件、雷达驱动或硬件测试记录。
