# 仓库布局与构建边界

当前自动控制研发中心是 `core/` + `runtime/`。由于性能不足和设计问题，ROS 相关功能暂时弃用并归档；决策与恢复条件见 [路线调整](../lightweight-transition.md)。

| 目录 | 职责 | 构建 / 使用入口 |
|---|---|---|
| `core/` | 原生视觉、建图、定位、规划、FSM、控制和 UART | 根 CMake / CTest |
| `runtime/` | 双摄采集矫正、任务调度、记录、HTTP、配置与部署 | 根 CMake；Linux/POSIX |
| `tools/rov/` | PC 驾驶台、Pi ROV 桥与维护工具 | Python / PowerShell |
| `firmware/stm32/` | 传感器、实时 PID、混控、PWM 和保护 | Keil / 固件测试 |
| `legacy/ros2/` | ROS 节点、msg/srv、launch 和旧构建配置 | 历史查阅；默认不构建、不部署 |
| `vision/` / `annotation/` | 离线标定、训练、数据处理与标注 | 隔离 Python 环境 |
| `models/` / `datasets/` | 模型清单与数据约定 | 大文件不进源码 Git |
| `hardware/` / `docs/` | 接线、坐标、协议、设计与验收 | 文档 |
| `logs/` / `videos/` / `maps/` / `trajectory/` / `events/` | 本地任务产物 | 默认忽略 |

`core/auv_core/CMakeLists.txt` 汇总各 `auv_*` 模块，保留原有 C++ include/namespace 名称。核心无 ROS/ament 依赖，不引用归档目录；ROS 节点包装留在归档，未来功能应先接入 Runtime。目录职责见 [核心手册](../../core/README.md)。

根目录构建完整 Runtime，`AUV_PERCEPTION_ONLY=ON` 可构建便携核心与测试。部署脚本打包 `core/`、`runtime/` 和需要的工具；默认不包含 ROS。旧布局构建缓存需重新生成，服务与配置路径保持兼容。

ROS 归档不再是独立可构建 workspace，`COLCON_IGNORE` 阻止扫描。复现原始 ROS 布局用独立 checkout 的 `8064e66`，见 [归档说明](../../legacy/ros2/README.md)。协议和固件组合仍需固定 commit、配置与验收报告；结构调整不代表实机验证通过。
