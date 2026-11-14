# L150 Ackermann Autonomous Navigation, Obstacle Avoidance & Follow-Car Simulation

MATLAB/Simulink 仿真项目，围绕 WHEELTEC L150-C30D SE 阿克曼转向小车，包含自主导航与避障，以及基于二维激光雷达观测的自动跟车两个部分。

## 1. 项目简介

项目使用 Ackermann 运动学模型，结合虚拟激光雷达、二维占用栅格地图、Hybrid A* 路径规划、路径跟踪和目标跟踪算法，研究小车在静态障碍物和动态场景中的自主运动。

自主导航部分可在发现新增障碍物后更新已知地图、重新规划路线，并按前进或倒车路段继续行驶。自动跟车部分通过雷达点云检测和跟踪前车，控制纵向距离与横向转向。

目前完成的是软件仿真。真实 LD14P 雷达接入、STM32 控制接口和实车运行效果尚未验证。

## 2. 系统架构

### 自主导航与避障

仿真环境 → 激光雷达扫描 → 障碍物坐标计算 → 已知地图更新 → 重新规划触发 → Hybrid A* → 路径分段管理 → 路径跟踪控制器 → Ackermann 运动学模型 → 状态反馈。

### 自动跟车

| 模块 | 功能 |
| --- | --- |
| FrontCar | 生成前车直行、转弯和停车轨迹 |
| TargetTracker | 根据仿真车辆状态计算目标相对位置 |
| VirtualLidar | 模拟 720 点二维激光雷达扫描 |
| LidarTargetTracker | 点云聚类、目标关联、运动预测与目标锁定 |
| FollowController | PI 距离控制、相对速度反馈、预测制动与转向控制 |
| Ackermann Kinematic Model | 模拟后车运动学行为 |
| XY Graph / Scope | 显示轨迹、跟车距离和状态变化 |

自动跟车数据流：

`FrontCar → TargetTracker → VirtualLidar → LidarTargetTracker → FollowController → Ackermann Model`

车辆状态同时反馈给目标跟踪和控制模块，形成闭环。

## 3. 主要功能与算法

### 3.1 车辆运动学与路径跟踪

- 基于 Ackermann Kinematic Model 建立车辆运动学模型。
- 根据车辆轴距计算转向关系，支持前进和倒车。
- 使用 Pure Pursuit 几何思想跟踪路径或前方目标。
- 按行驶方向分段，并在换挡时停车等待后切换路段。

### 3.2 激光雷达与地图更新

- 使用 `rangeSensor` 模拟二维扫描，并由测距和扫描角度计算障碍物坐标。
- 使用 `binaryOccupancyMap` 表示二维障碍物地图。
- 闭环避障模型分别维护仿真环境和车辆已知地图。
- 新障碍物由雷达观测加入已知地图；当前识别逻辑仍包含针对测试场景的区域过滤。

### 3.3 路径规划与重新规划

- 使用 Hybrid A* 规划路径，并考虑最小转弯半径。
- 对障碍物进行安全膨胀。
- 支持前进、倒车混合规划和按行驶方向整理路线段。
- 在新增障碍物场景中触发规划，并在短暂停车后切换到新路线。

### 3.4 自动跟车

- 通过前方区域筛选、连续点云聚类和目标中心计算提取候选目标。
- 使用连续帧的位置预测和数据关联门限降低目标误切换。
- 由距离变化估算相对接近速度，不需要直接读取前车真实速度。
- 使用 PI 距离控制、相对速度反馈和预测制动控制纵向速度，并用 Pure Pursuit 思想计算转向。
- 目标无效或丢失时向控制器报告无效状态并停车。

当前目标关联依赖几何位置和运动连续性，不具备可靠的车辆身份识别能力。

## 4. 仿真参数与结果

### 4.1 自动跟车参数

| 参数 | 当前设置 |
| --- | ---: |
| 小车轴距 | 0.144 m |
| 控制周期 | 0.02 s |
| 期望雷达测距 | 0.7 m |
| 等效车辆参考点间距 | 约 0.8 m |
| 最大后车速度 | 0.25 m/s |
| Kp / Ki / Kd | 0.8 / 0.25 / 1.0 |
| 转向角范围 | -0.49～0.32 rad |
| 仿真时长 | 25 s |

上述数值是当前仿真设置，尚未经过实车标定。

已进行的自动跟车初步测试包括前车匀速直行、左转、停车、目标丢失、轻微测距扰动、多目标干扰和运动预测跟踪。当前测试中，后车运动时的参考点距离约为 0.81 m，前车停车后稳定在约 0.79 m（根据仿真图像估读）。这些结果仅代表当前理想化仿真场景，不构成真实环境中的跟踪可靠性或碰撞安全保证。

### 4.2 自主导航与避障结果

- 静态障碍物路径规划：用于检查 Hybrid A* 在已知地图中的路径规划能力。
- 前进与倒车跟踪：用于检查分段路线执行和自动换挡。
- 新增障碍物重规划：用于检查雷达观测、地图更新、规划触发和路线切换。
- 在预设障碍物地图的自动重规划测试中，测得终点位置误差为 **0.079 m**。雷达感知地图驱动闭环的最终场景应以该仿真对应的最新测量结果单独记录。
- 一组避障控制仿真中的最大前轮转向角为 **0.1677 rad**（设定上限 0.25 rad），最大转向角速度为 **0.1496 rad/s**（设定上限 0.35 rad/s），该组结果满足相应的仿真转向约束。

以上数据对应各自的仿真模型和场景，不应直接视为实车测试结果。

## 5. 开发环境

- MATLAB R2026a
- Simulink
- Robotics System Toolbox
- Navigation Toolbox
- 二维占用栅格地图与虚拟激光雷达
- Ackermann Kinematic Model

打开模型前，在 MATLAB 中进入仓库根目录并运行 `addpath(genpath(pwd))`，使 `planning/` 和 `sensing/` 中的辅助函数也加入 MATLAB Path。若 MATLAB 提示缺少组件，请检查相应工具箱和 Ackermann 模型库是否可用。

## 6. 核心文件说明

### 跟车仿真

- `models/follow_car.slx`：自动跟车闭环主模型。FrontCar、VirtualLidar、LidarTargetTracker 和 FollowController 等逻辑内嵌在模型中。

### 自主导航与避障模型

- `models/L150_Closed_Loop3.slx`：包含雷达感知、已知地图更新和自动重规划触发的闭环模型。
- `models/L150_Closed_Loop2.slx`：基于预设障碍物地图的闭环重规划版本，可作对照。
- `models/L150_Closed_Loop.slx`：路径管理与车辆控制模型变体。
- `models/L150_Auto_Segments.slx`：分段路径跟踪与自动换挡模型。
- `models/L150_Path_Follow.slx`：基础路径跟踪模型。
- `models/L150_Reverse_Test.slx`：倒车跟踪测试模型。
- `models/ackermann02.slx`、`ackermannbizhang.slx`：较早的 Ackermann 避障模型。

### 感知、规划与辅助代码

- `planning/l150SenseAndPlanStep.m`：雷达扫描、已知地图更新和自动规划触发。
- `planning/l150AutoReplanSensed.m`：根据雷达更新后的地图运行 Hybrid A* 并整理路线段。
- `planning/l150AutoReplan.m`：根据预设障碍物地图生成前进/倒车路线。
- `planning/L150_Path_Plannin.m`：静态地图上的 Hybrid A* 规划脚本。
- `sensing/L150_Lidar_Mapping_Test.m`：雷达扫描点转换和占用地图更新测试。
- `sensing/LidarTest.m`：独立虚拟雷达场景测试。
- `sensing/l150DynamicFront.m`、`sensing/l150ReplanLidar.m`、`sensing/l150FrontRange.m`：不同仿真模型使用的虚拟雷达测距函数。
- `l150ThreeRanges.m`：主避障模型使用的左、前、右三向测距函数。
- `L150_Collision_Check.m`：轨迹参考点安全间隙的初步检查；需要工作区中的 `xy` 矩阵（第一列 x、第二列 y，单位米），并以半径 0.20 m 的圆近似车身。

### 路径数据

- `planning/L150_Path.mat`：已有路径数据，包含 `waypoints`、`directions`、`startPose` 和 `goalPose`。
- `planning/L150_Replan_Test.m`：新增障碍物后的 Hybrid A* 重规划测试；运行前需在工作区提供停车位姿 `stopPose`。
- `planning/L150_Replan_Path.mat`：重规划脚本生成的航点、行驶方向、速度计划和方向切换位置。

## 7. 当前局限性

- 雷达和车辆运动学均为理想化仿真模型。
- 跟车目标关联不等同于车辆身份识别。
- 新障碍物检测含有针对当前测试场景的区域过滤。
- 尚未完成完整车身碰撞及扫掠空间验证。
- 路径跟踪调试阶段曾临时关闭距离触发的安全停车，实车使用前必须恢复并完善。
- 尚未完成 LD14P 雷达数据接入、车辆定位、STM32 控制接口及传感器时间同步。

## 8. 后续工作

- 接入真实 LD14P 雷达扫描数据。
- 完善多帧观测融合，以及多目标交叉、遮挡和误识别测试。
- 引入编码器里程计并完成传感器时间同步。
- 对接 STM32 速度、转向和紧急停车接口。
- 标定真实车辆的制动能力与转向响应，并开展低速实车跟车测试。
- 研究基于地图的车辆定位与导航。

## 9. 项目状态

**当前阶段：自主导航、避障与自动跟车的仿真核心流程已搭建，正在完善算法并准备后续硬件验证。**
