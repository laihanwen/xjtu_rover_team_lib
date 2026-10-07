# 仓库布局与模块边界

本项目采用 monorepo。ROS 接口、树莓派运行时代码、STM32 固件、视觉训练配置和硬件文档需要共同演进，集中管理可以让一次协议修改在同一个提交中完成。若未来 STM32 固件需要独立发布，再将 `firmware/stm32` 拆分为独立仓库或 submodule。

架构和资源所有权见 [系统架构](system.md)。本轮保留源码路径，以兼容现有 Keil/CMake、Python import 和服务部署。

## 目录职责

| 目录 | 内容 | 构建方式 |
|---|---|---|
| `runtime/` | 原生C++双摄、HTTP与轻量任务运行时 | 根目录CMake |
| `tools/rov/` | PC驾驶台、Pi桥、日志/录像、维护脚本 | Python / PowerShell |
| `src/` | Raspberry Pi / PC 上运行的 ROS 2 packages | `colcon build` |
| `firmware/stm32/` | STM32F405 八推实时控制、安全和外设代码 | Keil MDK-ARM；CMake/GCC 编译检查 |
| `vision/` | YOLO 数据准备、训练、评估、导出及离线 OpenCV 实验 | 独立 uv 环境 |
| `models/` | 模型版本清单、类别定义、部署参数和校验和 | 不参与 colcon |
| `datasets/` | 数据集结构、来源与标注约定 | 大文件不进 Git |
| `hardware/` | BOM、接线、推进器布局、坐标系和机械资料 | 文档/CAD 工具 |
| `docs/` | 架构、串口协议、任务流程和测试计划 | 文档 |
| `tools/` | 开发环境、构建、部署和数据工具 | fish/Python 工具 |
| `annotation/` | P12 Label Studio 独立环境与标注配置 | `uv sync --frozen` |
| `logs/`、`videos/`、`maps/`、`trajectory/`、`events/` | 本地运行产物 | 默认不进入 Git |

## 已实现的 ROS packages

`src/` 当前包含八个可由 `colcon` 构建的 package：

1. `auv_interfaces`：公共 msg/srv。
2. `auv_stm32_bridge`：串口编解码、心跳和 ROS topics。
3. `auv_vision`：相机、AprilTag、OpenCV 检测和模型推理节点。
4. `auv_mapping`：九宫格透视矫正与语义地图。
5. `auv_planning`：A* 与目标访问顺序。
6. `auv_mission`：任务 FSM。
7. `auv_control`：默认禁用动力的格子路线执行与安全门控。
8. `auv_bringup`：系统启动和跨 package 参数。

`src/auv_core` 是共享原生算法库，供根目录CMake和ROS模块使用；不计入上述八个ROS package。

`auv_description` 仍是候选 package。只有形成清晰接口、独立实现和测试需求后再建立，
避免为了目录完整而产生空 package。

训练代码不放入 `auv_vision`，避免 ROS 系统 Python 与 uv/PyTorch 环境耦合。部署模型只通过明确的模型清单交给运行时节点。

## 版本关系

- ROS 与固件共享的帧格式记录在 `docs/protocol/serial-protocol.md`。
- 串口协议发生不兼容变更时递增协议主版本。
- 模型清单记录权重 SHA-256、类别、输入尺寸、导出格式和训练数据版本。
- 固件发布、ROS 发布和模型发布可以独立打 tag，但比赛基线必须在文档中固定三者组合。
