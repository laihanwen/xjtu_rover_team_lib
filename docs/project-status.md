# 项目压缩状态

更新日期：2026-10-03。本文用于快速恢复开发上下文；具体参数与验收步骤以各模块文档为准。

## 当前状态

- P0–P11：软件链路和自动测试已建立，硬件相关能力仍以 README 中的安全边界为准。
- P12：海参 YOLO 训练/部署环境已建立，缺真实水下标注数据和正式权重。
- P13：T35-L 单舵机夹爪的软件链路完成；标定门保持关闭，缺真实开合端点和实机验收。
- P14：转盘 OpenCV 视觉基础完成；可发布圆盘中心、半径和无方向把手轴线，未接控制闭环。

## 不可跨越的安全门

- 推进器测试必须断电、拆桨或可靠固定；ARM、heartbeat、漏水和超时保护不得绕过。
- T35-L 必须由独立大电流电源供电并与 STM32 共地，禁止从控制板 5 V 引脚供电。
- P13 未实测 `CLOSE_US/OPEN_US` 前保持 `AUV_GRIPPER_CALIBRATED=0`。
- P14 当前只输出视觉观测，不得据此直接驱动推进器或未知执行器。

## 最近验证基线

```fish
source /opt/ros/lyrical/setup.fish
colcon build --symlink-install
source install/setup.fish
colcon test
colcon test-result --verbose
```

另有 STM32 宿主测试和 ARM GCC 对象编译检查；可烧录固件仍以 Keil 工程为准。

## 下一步

1. 完成 P13 夹爪端点标定和断桨实机验收。
2. 采集前摄像头下转盘的干态、水槽和无目标负样本 rosbag。
3. 标定前摄像头并在检测前去畸变，统计 P14 误检、漏检和角度抖动。
4. 明确转盘执行机构、反馈传感器和 STM32 通道。
5. 再实现多帧跟踪、累计角度、对准控制和 Mission/STM32 闭环。

关键入口：根 [`README.md`](../README.md)、
[`P13 验收`](testing/p13-gripper.md)、
[`P14 验收`](testing/p14-valve-foundation.md)、
[`P14 代码审查`](reviews/p14-foundation-code-review.md)。
