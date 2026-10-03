# ROS 2 runtime

本目录只存放部署到 Raspberry Pi / PC 的 ROS 2 packages。当前实现
`auv_interfaces`、`auv_stm32_bridge`、`auv_vision`（含 AprilTag、交通锥、海参和转盘视觉）、
`auv_mapping`、`auv_planning`、`auv_mission` 和 `auv_bringup`；后续
package 必须在能够独立构建和测试时再创建，规划见
[`docs/architecture/repository-layout.md`](../docs/architecture/repository-layout.md)。

系统 Python 供 ROS 2 / rclpy 使用。YOLO 训练依赖不得安装到系统 Python，也不得加入 ROS package 的运行依赖，除非已确定机器人端推理后端。
