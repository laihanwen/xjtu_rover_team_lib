# 测试与验收

本目录用于整理任务级验收、风险检查和现场测试记录，覆盖从安全门到任务闭环的关键验证。

## 目录内容

- [strategy.md](./strategy.md) — 总体测试策略
- [p4-safety.md](./p4-safety.md) — heartbeat / failsafe 验收
- [task1-apriltag-cones.md](./task1-apriltag-cones.md) — 任务一 AprilTag 与交通锥遍历验收
- [p13-gripper.md](./p13-gripper.md) — P13 单舵机夹爪验收
- [p14-valve-foundation.md](./p14-valve-foundation.md) — P14 转盘视觉基础验收

## 验收顺序

1. 先完成安全门与 heartbeat 测试；
2. 再进行底层视觉与地图记录验证；
3. 最后执行任务一或后续任务闭环。

## 关键原则

- 首次实机测试必须断开推力或固定推进器。
- 所有验收文件都应该保留结果、风险和后续动作项。
- 只有在确认安全门和基础能力后，才进入更高风险的运动闭环。
