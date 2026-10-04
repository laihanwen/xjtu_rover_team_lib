# AUV 文档中心

根 `README.md` 负责快速进入项目；本目录保存设计依据、接口协议、操作手册、测试门槛和
审查结论。硬件参数未知时，以 `AGENTS.md` 的安全约束为准，不从示例值推断真实接线。

## 项目状态

- [压缩状态：当前阶段、安全门与下一步](project-status.md)
- [P14 视觉基础代码审查](reviews/p14-foundation-code-review.md)

## 架构与接口

- [仓库布局与模块边界](architecture/repository-layout.md)
- [Pi ↔ STM32 串口协议 v1](protocol/serial-protocol.md)
- [STM32 固件说明](../firmware/stm32/README.md)
- [ROS 运行时 package 导航](../src/README.md)

## 运行指南

- [PC 与树莓派有线联调](wired-network.md)
- [P12 水下目标打标教程](p12-annotation-guide.md)
- [新人 AI 辅助开发教程](newcomer-ai-development-guide.md)

## 测试与实机门槛

- [总体测试策略](testing/strategy.md)
- [P4 heartbeat/failsafe 验收](testing/p4-safety.md)
- [任务一 AprilTag 与交通锥遍历验收](testing/task1-apriltag-cones.md)
- [P13 T35-L 单舵机夹爪验收](testing/p13-gripper.md)
- [P14 转盘视觉基础验收](testing/p14-valve-foundation.md)

## 模块文档

| 模块 | 文档 |
|---|---|
| ROS 2 ↔ STM32 bridge | [`auv_stm32_bridge`](../src/auv_stm32_bridge/README.md) |
| 相机与视觉 | [`auv_vision`](../src/auv_vision/README.md) |
| 九宫格语义地图 | [`auv_mapping`](../src/auv_mapping/README.md) |
| 格子路径规划 | [`auv_planning`](../src/auv_planning/README.md) |
| 路线执行控制 | [`auv_control`](../src/auv_control/README.md) |
| Mission FSM | [`auv_mission`](../src/auv_mission/README.md) |
| 数据标注 | [`annotation`](../annotation/README.md) |
| 数据集约定 | [`datasets`](../datasets/README.md) |
| 模型注册 | [`models`](../models/README.md) |
| 硬件资料 | [`hardware`](../hardware/README.md) |
