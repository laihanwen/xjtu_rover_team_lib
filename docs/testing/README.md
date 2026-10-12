# 测试与验收导航

更新：2026-10-12。日期记录保存当次事实，当前缺口以[待办](../auv-mapping-traversal-todo.md)为准。

## 当前流程

- [AUV 感知检查驾驶台](auv-perception-console-20261012.md)：标签、候选、局部关联、累计地图与新鲜度的可视化入口。
- [双视觉与同钟记录软件](dual-vision-sensor-development-20261012.md)：本地 20/20，真实 B 权重与新增完整 Linux Runtime 回归待验收。

- [Ubuntu 验证步骤](ubuntu-validation-assets-20261012.md)与[2026-10-12 实际结果](ubuntu-validation-results-20261012.md)：完整 Runtime 27/27 回归及真实样本统计，米制定位仍未验收。
- [原生测试策略](strategy.md)与[任务一验收](task1-apriltag-cones.md)。
- [连续素材采集](next-data-collection-20261010.md)与[素材验证流程](mapping-validation-procedure-20261010.md)。
- [局部 Runtime 开发及回归边界](local-runtime-development-20261011.md)与[版本对应关系](pi-regression-version-20261011.json)。
- [双摄帧率诊断](camera-performance-analysis-20261011.md)。

## 历史与后续任务参考

[P4 安全](p4-safety.md)、[P13 夹爪](p13-gripper.md)、[P14 转盘](p14-valve-foundation.md)包含历史设计或专项验收参考；其中 ROS 操作不作为轻量系统启动命令。其余日期报告按[索引](../catalog.md)检索，不能从历史通过记录推断新源码、当前设备或水池闭环已验收。
## 2026-10-12 本轮交接

- [Ubuntu 增量实际结果](ubuntu-task-one-results-20261012.md)：完整编译、33/34 回归、已修复问题与仍失败的局部闭环，附真实样本对照及证据包。
- [局部定位误差修复](ubuntu-local-pose-repair-20261012.md)：管线角度、测量平面约束、冲突后跳格拒绝及建图校验顺序；局部建图闭环仍未通过。
- [多帧覆盖建图与绕场扫描](multiframe-map-coverage-20261012.md)：局部视角累计、完整锥体轮廓确认、空格证据门控和闭合航点支持，附本轮验证边界。
- [下游分段验证进度](downstream-stage-validation-20261012.md)：独立上浮、重定位、四锥、退出尝试；10 项中 3 个负例通过，7 个正常场景未通过。
- [Ubuntu AI 增量测试交接](ubuntu-task-one-handoff-20261012.md)：最新源码、隔离测试脚本、旧素材复用与结果回传。
- [双视觉与同钟记录开发](dual-vision-sensor-development-20261012.md)：20/20 便携范围、退出控制与模型训练边界。
