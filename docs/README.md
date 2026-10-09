# AUV 文档中心

根目录 [README.md](../README.md) 负责项目入口；本目录负责整理设计依据、接口协议、部署手册、验收门槛和评审结论。硬件参数未知时，以 [AGENTS.md](../AGENTS.md) 的安全约束为准，不从示例值推断真实接线。

ROS 相关功能目前因性能不足和设计问题暂时弃用。当前自动控制开发、调试与部署入口为 [原生核心](../core/README.md) 和 [Runtime](../runtime/README.md)，见 [路线调整](lightweight-transition.md)。下方 ROS package 文档仅为历史资料。

## 阅读规则

根README提供项目导航；模块README说明当前操作；project-status区分源码/部署/实机验证；reviews和deployment保存历史证据。历史文件中的参数不得覆盖当前配置。

- [系统架构与资源所有权](architecture/system.md)
- [轻量 AUV 开发边界与阶段验收](auv-development-boundary.md)
- [A0/A1实现、标定与验收](auv-a0-a1-implementation.md)
- [A2任务一闭环、标定与验收](auv-a2-implementation.md)
- [池底 AprilTag 接近测试](auv-tag-docking-test.md)
- [轻量定位首版](localization.md)
- [ROV当前操作](../tools/rov/README.md)
- [维护与部署](../tools/rov/MAINTENANCE.md)
- [ROV历史集成原文](../tools/rov/LEGACY_NOTES.md)

## 文档结构总览

```text
docs/
├── README.md                                # 文档入口，说明文档分层
├── project-status.md                        # 当前阶段、安全门和后续计划
├── newcomer-ai-development-guide.md         # 新成员和 AI 协作说明
├── p12-annotation-guide.md                 # P12 数据标注说明
├── wired-network.md                         # PC / Pi 联调网络说明
├── architecture/
│   ├── README.md                           # 架构导航入口
│   └── repository-layout.md                # 仓库布局与模块边界
├── deployment/
│   ├── README.md                           # 现场部署说明入口
│   └── pi-20261006/
│       ├── README.md                       # Raspberry Pi 现场部署与运行记录
│       └── *.json                          # 摄像头/启动/预检配置
├── protocol/
│   ├── README.md                           # 协议导航入口
│   └── serial-protocol.md                  # Pi ↔ STM32 串口协议 v1
├── reviews/
│   ├── README.md                           # 审查导航入口
│   └── p14-foundation-code-review.md      # P14 视觉基础审查
├── testing/
│   ├── README.md                           # 测试导航入口
│   ├── strategy.md                         # 总体测试策略
│   ├── p4-safety.md                        # heartbeat / failsafe 验收
│   ├── task1-apriltag-cones.md            # 任务一 AprilTag / 交通锥验收
│   ├── p13-gripper.md                      # P13 夹爪验收
│   └── p14-valve-foundation.md             # P14 转盘视觉基础验收
├── ...
└── ...
```

## 快速入口

### 项目状态与路线

- [项目状态：当前阶段、安全门与下一步](./project-status.md)
- [P14 视觉基础代码审查](./reviews/p14-foundation-code-review.md)

### 架构与接口

- [架构导航](./architecture/README.md)
- [仓库布局与模块边界](./architecture/repository-layout.md)
- [Pi ↔ STM32 串口协议 v1](./protocol/serial-protocol.md)
- [STM32 固件说明](../firmware/stm32/README.md)
- [ROS 历史归档导航](../core/README.md)

### 运行与部署

- [部署导航](./deployment/README.md)
- [PC 与树莓派有线联调](./wired-network.md)
- [P12 水下目标打标教程](./p12-annotation-guide.md)
- [开发环境和协作说明](./newcomer-ai-development-guide.md)

### 测试与验收

- [测试导航](./testing/README.md)
- [总体测试策略](./testing/strategy.md)
- [P4 heartbeat/failsafe 验收](./testing/p4-safety.md)
- [任务一 AprilTag 与交通锥遍历验收](./testing/task1-apriltag-cones.md)
- [P13 T35-L 单舵机夹爪验收](./testing/p13-gripper.md)
- [P14 转盘视觉基础验收](./testing/p14-valve-foundation.md)

### 模块文档（ROS 条目为历史资料）

| 模块 | 文档 |
|---|---|
| ROS 2 ↔ STM32 bridge | [`auv_stm32_bridge`](../legacy/ros2/src/auv_stm32_bridge/README.md) |
| 相机与视觉 | [`auv_vision`](../legacy/ros2/src/auv_vision/README.md) |
| 九宫格语义地图 | [`auv_mapping`](../legacy/ros2/src/auv_mapping/README.md) |
| 格子路径规划 | [`auv_planning`](../legacy/ros2/src/auv_planning/README.md) |
| 路线执行控制 | [`auv_control`](../legacy/ros2/src/auv_control/README.md) |
| Mission FSM | [`auv_mission`](../legacy/ros2/src/auv_mission/README.md) |
| 数据标注 | [`annotation`](../annotation/README.md) |
| 数据集约定 | [`datasets`](../datasets/README.md) |
| 模型注册 | [`models`](../models/README.md) |
| 硬件资料 | [`hardware`](../hardware/README.md) |

## 阅读顺序建议

1. 先用 [项目状态](./project-status.md) 确定当前阶段与安全门；
2. 再查看 [架构导航](./architecture/README.md) 和 [协议导航](./protocol/README.md)；
3. 需要现场部署时，阅读 [部署导航](./deployment/README.md)；
4. 进行任务验证时，按 [测试导航](./testing/README.md) 依次执行。

该结构在保持现有文件布局兼容的同时，提供更清晰的导航入口，方便团队沿同一文档目录顺序阅读和维护。
