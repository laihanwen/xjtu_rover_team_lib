# 项目压缩状态

更新日期：2026-10-04。本文用于快速恢复开发上下文；具体参数与验收步骤以各模块文档为准。

## 当前状态

- P0–P11：软件链路和自动测试已建立，硬件相关能力仍以 README 中的安全边界为准。
- 任务一：AprilTag → 四锥语义地图 → 路线 → visited 软件闭环已建立；路线执行默认
  dry-run，等待水下标定、坐标方向和拆桨实机验收。
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

1. 按任务一验收手册完成下视相机水下标定、四锥录像回归和 dry-run。
2. 在拆桨固定条件下确认 grid row/col 到 surge/sway 的方向和 failsafe。
3. 完成 P13 夹爪端点标定和断桨实机验收。
4. 采集并标注 P12/P14 真实水下数据。

关键入口：根 [`README.md`](../README.md)、
[`P13 验收`](testing/p13-gripper.md)、
[`任务一验收`](testing/task1-apriltag-cones.md)、
[`P14 验收`](testing/p14-valve-foundation.md)、
[`P14 代码审查`](reviews/p14-foundation-code-review.md)。
