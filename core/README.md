# 轻量自动控制核心

本目录是当前 AUV 原生算法与 UART 协议开发入口，不依赖 ROS、ament 或 colcon。`auv_core` 的 CMake 将各模块编为一个库，供 `runtime/` 使用。

| 模块 | 职责 |
|---|---|
| `auv_core` | 状态解码、语义地图数据与库导出 |
| `auv_vision` | 相机源、AprilTag 与交通锥检测 |
| `auv_mapping` | 网格建图、平面定位与度量位姿 |
| `auv_planning` | 网格路径与目标访问规划 |
| `auv_mission` | 任务状态机 |
| `auv_control` | 观测搜索、任务一遍历和路线执行 |
| `auv_stm32_bridge` | UART、帧协议、解析和遥测 |

保留现有 `auv_*` C++ 命名空间与 include 名称，避免改变协议或算法行为。新增自动控制能力应在这里实现可离线测试的算法，在 `runtime/` 接入配置、采集、记录和生命周期；ROS 历史节点在 `legacy/ros2/`。

从仓库根目录使用 CMake 构建和 CTest，见 [Runtime 手册](../runtime/README.md)。实时 PID、PWM 和硬件保护继续由 STM32 负责。
