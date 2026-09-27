# AUV / 水下具身智能机器人项目上下文

## 1. 项目目标

这是一个面向水下机器人竞赛的自主 AUV 项目。

机器人需要在无人工遥控的情况下完成：

- 自主启动
- AprilTag 识别
- 水下九宫格场地识别
- 交通锥检测与圆形/方形分类
- 语义地图构建
- 自主路径规划和格子遍历
- 海参目标识别、抓取和运输
- 转盘识别并旋转不少于 180°
- 返回起点并上浮
- 全过程任务日志、视觉数据和状态记录

当前开发目标不是做复杂通用 SLAM，而是优先完成一个稳定、可比赛的结构化场景 AUV。

---

# 2. 总体系统架构

系统采用双层控制架构：

Raspberry Pi / PC
负责高层计算：

- ROS 2
- 计算机视觉
- AprilTag
- OpenCV
- YOLO
- 语义地图
- 路径规划
- Mission FSM
- 日志记录
- 与 STM32 通信

STM32
负责实时底层控制：

- IMU 采样
- 深度传感器采样
- 姿态估计
- Roll / Pitch / Yaw 控制
- Depth 控制
- PID
- Thruster Mixer
- ESC PWM
- 机械执行器
- 心跳检测
- 漏水检测
- Failsafe

基本原则：

Linux / ROS 2 不直接承担高速实时姿态 PID。

树莓派只发送运动目标：

- vx
- vy
- vz / depth
- yaw

STM32 自己闭环控制推进器。

---

# 3. 当前开发平台

主机：

- Ubuntu 26.04
- codename: resolute
- fish shell
- Ghostty terminal
- VS Code
- Codex CLI

ROS：

- ROS 2 Lyrical
- /opt/ros/lyrical
- fish 环境通过：

  source /opt/ros/lyrical/setup.fish

ROS 2 Desktop 已安装，包括：

- rclcpp
- rclpy
- colcon
- rosdep
- vcstool
- RViz2
- rqt
- rosbag2
- tf2
- URDF
- robot_state_publisher
- cv_bridge
- image_transport
- OpenCV ROS integration

ROS workspace：

~/auv_ws

预期结构：

auv_ws/
├── src/
│   ├── auv_interfaces
│   ├── auv_bringup
│   ├── auv_description
│   ├── auv_stm32_bridge
│   ├── auv_control
│   ├── auv_vision
│   ├── auv_mapping
│   ├── auv_planning
│   └── auv_mission
├── build/
├── install/
└── log/

ROS 开发使用：

colcon build --symlink-install

fish 下加载 workspace：

source ~/auv_ws/install/setup.fish

不要建议使用 setup.bash，除非当前 shell 明确是 bash。

---

# 4. AI / 视觉开发环境

主机具有 NVIDIA RTX 3060 Laptop GPU。

现有环境包含：

- Python 3.14
- uv
- PyTorch
- CUDA
- Ultralytics / YOLO 开发环境

训练环境和系统 ROS Python 环境应尽量隔离。

原则：

系统 Python：
主要用于 ROS 2 / rclpy。

uv virtual environment：
用于 YOLO 训练、数据集、实验代码。

不要随意 pip install 到系统 Python。

视觉技术栈：

- OpenCV
- AprilTag
- YOLO11n 优先
- ONNX Runtime 或 NCNN 部署
- cv_bridge
- image_transport

---

# 5. 摄像头规划

预计至少两个摄像头。

Down Camera：

- AprilTag
- 九宫格
- 黄色基准边
- 交通锥
- 地面结构
- 地图定位

Front Camera：

- 海参
- 海龟
- 海星
- 转盘
- 抓取视觉

图像坐标和机器人坐标必须通过 TF 清晰管理。

---

# 6. 九宫格建图方案

第一版不要使用复杂 SLAM。

使用 OpenCV 建立 Semantic Map。

流程：

camera image
→ 图像预处理
→ 黄色边检测
→ 网格直线检测
→ 四角点
→ Homography
→ 标准俯视图
→ 3×3 cell segmentation
→ cone detection
→ cone classification
→ semantic map

推荐：

- HSV / LAB
- Canny
- HoughLines / HoughLinesP
- contours
- morphology
- cv2.getPerspectiveTransform
- cv2.warpPerspective

地图格式例如：

{
  "cells": [
    {"row": 0, "col": 0, "object": "circle_cone"},
    {"row": 0, "col": 2, "object": "square_cone"}
  ]
}

---

# 7. 交通锥识别

第一版优先 OpenCV，而不是 YOLO。

可用：

- threshold / segmentation
- findContours
- approxPolyDP
- circularity

圆度：

C = 4πA / P²

根据形状和轮廓区分：

- circle
- square

如果真实水下条件导致规则方法鲁棒性不足，再切换到学习模型。

---

# 8. 海参目标识别

海参阶段使用 YOLO。

类别预计：

- sea_cucumber
- turtle
- starfish

第一选择：

YOLO11n

部署目标：

训练：
RTX 3060 PC

导出：
ONNX

机器人部署：

- ONNX Runtime
或
- NCNN

树莓派 4B 上优先轻量模型，不默认使用 m/l/x 大模型。

---

# 9. 路径规划

地图只有 3×3 网格和少量目标。

不要过度设计。

交通锥只有 4 个时：

排列总数 4! = 24。

直接枚举目标访问顺序即可。

格子之间路径：

A*

高层目标顺序：

Brute-force TSP / permutation enumeration。

不要为了此问题引入：

- reinforcement learning
- genetic algorithm
- ant colony algorithm

除非以后明确需要研究实验。

---

# 10. Mission 架构

建议 Mission FSM：

INIT
→ SELF_CHECK
→ SEARCH_APRILTAG
→ BUILD_MAP
→ PLAN_CONES
→ VISIT_CONES
→ SEARCH_CUCUMBER
→ ALIGN_CUCUMBER
→ GRAB
→ TRANSPORT
→ RELEASE
→ SEARCH_VALVE
→ ALIGN_VALVE
→ ROTATE_VALVE
→ RETURN_HOME
→ SURFACE
→ COMPLETE

状态改变必须记录日志。

---

# 11. ROS 2 节点规划

建议：

auv_camera_down
auv_camera_front

auv_apriltag_detector

auv_cone_detector
auv_cucumber_detector
auv_valve_detector

auv_semantic_mapper

auv_planner

auv_mission_manager

auv_stm32_bridge

auv_state_estimator

auv_logger

robot_state_publisher

关键 topics：

/camera/down/image_raw
/camera/front/image_raw

/apriltag/detections
/cones/detections
/cucumber/detections
/valve/detection

/semantic_map

/mission/state

/cmd_vel
/cmd_depth
/cmd_yaw

/imu/data
/depth
/battery
/leak

/stm32/status
/stm32/thrusters

/odom

/tf
/tf_static

---

# 12. STM32 通信

第一版优先 UART。

不要一开始引入 micro-ROS，除非它能明显降低系统复杂度。

协议推荐：

0xAA 0x55
TYPE
LEN
PAYLOAD
CRC16

Pi → STM32：

- heartbeat
- arm/disarm
- vx
- vy
- depth_target
- yaw_target
- actuator command

STM32 → Pi：

- roll
- pitch
- yaw
- depth
- voltage
- leak
- error_flags
- thruster outputs

建议：

Pi → STM32：
20–50 Hz

STM32 控制环：
100–500 Hz

Heartbeat：

Pi 周期发送。

如果 STM32 超过约 500 ms 未收到心跳：

进入 failsafe。

不要让 Linux 程序卡死导致推进器保持危险输出。

---

# 13. STM32 控制

STM32 当前负责：

IMU
Depth
PID
Thruster Mixer
ESC
Safety

建议控制变量：

surge
sway
heave
roll
pitch
yaw

初期至少保证：

- depth hold
- yaw hold
- forward/backward
- lateral motion

推进器建议最终 6 个：

4 个水平推进器：
surge / sway / yaw

2 个垂直推进器：
heave / pitch compensation

实际 mixer 必须根据真实推进器位置和方向建立矩阵，不允许直接复制未经验证的 mixer。

---

# 14. 硬件

当前已有：

- Raspberry Pi 4B
- STM32F405RGT6（现有八推全矢量 CubeMX / Keil 工程位于 `firmware/stm32`）
- ESC
- motors

计划/建议：

- IMU: BMI088 或 ICM-42688 等
- depth sensor: MS5837 系列
- 2 cameras
- leak sensor
- battery
- DC-DC
- waterproof enclosure
- waterproof connectors
- thruster guards
- lighting
- underwater actuator / servo
- grabbing / funnel mechanism

STM32 型号已由现有工程确认为 STM32F405RGT6。ESC 型号和电机参数以后根据真实硬件更新。

不要自行假定 GPIO、PWM timer、ESC PWM 范围或推进方向。

---

# 15. 安全原则

任何涉及推进器、电调、电池、机械执行机构的代码都必须优先考虑安全。

必须支持：

- DISARM 默认状态
- Heartbeat timeout
- communication timeout
- sensor invalid detection
- leak detection
- kill / emergency state
- output saturation
- startup neutral PWM
- explicit ARM command

任何测试首次运行时应默认使用：

- 电机断电
或
- 拆桨
或
- 推进器安全固定

不要自动执行可能让推进器突然启动的命令。

---

# 16. Logging

使用 rosbag2。

至少记录：

/camera/down/image_raw
/camera/front/image_raw
/imu/data
/depth
/odom
/cmd_vel
/stm32/status
/detections
/semantic_map
/mission/state

额外保存：

logs/
videos/
maps/
trajectory/
events/

event log 应包含明确时间戳，例如：

START
APRILTAG_FOUND
MAP_COMPLETE
CONE_VISITED
CUCUMBER_FOUND
GRAB_COMPLETE
VALVE_ROTATED
HOME_REACHED
SURFACE
MISSION_COMPLETE

---

# 17. 软件工程原则

代码以可运行、可调试、可逐步测试为优先。

不要过度工程。

新增模块时：

1. 先检查已有代码和 package。
2. 保持 ROS package 边界清晰。
3. 为重要接口定义 msg / srv。
4. 不把硬件驱动和 Mission 逻辑混在一起。
5. 不把算法写死在 launch file。
6. 参数使用 ROS parameters 或 YAML。
7. 节点必须能单独运行和测试。
8. 重要数据必须能 ros2 topic echo。
9. 视觉算法最好支持读取录制视频离线复现。
10. 新增依赖前先检查 Ubuntu 26.04 / ROS 2 Lyrical 是否兼容。

---

# 18. Codex 工作方式

当你帮助开发这个项目时：

- 先查看当前 repo 内容，不要假定不存在的文件。
- 优先修改已有结构，而不是重复创建。
- 命令必须兼容 fish，除非明确开启 bash。
- ROS distro 是 lyrical。
- Ubuntu codename 是 resolute。
- 不要给 Jazzy/Humble 的包名作为默认命令。
- 不要随意修改系统 Python。
- 不要破坏当前 CUDA / PyTorch / YOLO 环境。
- 不要未经允许升级 ROS distro 或 Ubuntu。
- 编译 ROS package 后进行最小验证。
- STM32 代码改动要区分 autogenerated CubeMX 区域和 USER CODE 区域。
- 不要修改 CubeMX 生成机制会覆盖的代码，除非有明确理由。
- 硬件参数未知时，用 TODO / configurable parameter，而不是猜数值。

每次实现功能后尽量给出：

- 修改了什么
- 为什么
- 如何 build
- 如何 run
- 如何验证
- 可能的硬件风险

---

# 19. 当前近期目标

按优先级：

P0
ROS 2 workspace 正常工作。

P1
建立 auv_interfaces。

P2
建立 auv_stm32_bridge。

P3
定义 Pi ↔ STM32 serial protocol。

P4
STM32 heartbeat + failsafe。

P5
IMU / depth ROS topics。

P6
camera ROS nodes。

P7
AprilTag。

P8
九宫格 Semantic Mapping。

P9
Cone detection。

P10
Path planning。

P11
Mission FSM。

P12
Sea cucumber YOLO。

P13
Grabbing system。

P14
Valve system。

先把每一级做稳定，再进入下一层。
