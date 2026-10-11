# AUV / ROV 文档中心

更新：2026-10-11。当前操作与日期证据分开阅读；历史记录中的设备状态、配置和测试结果不能直接用于当前部署。

## 当前入口

- [项目状态](project-status.md)：源码、编译、部署与水池验收边界。
- [待办清单](auv-mapping-traversal-todo.md)：只保留未完成项目。
- [系统架构](architecture/system.md)与[开发边界](auv-development-boundary.md)：轻量原生路线，ROS 暂时弃用。
- [独立 ROV 一键烧录](one-click-flash-console.md)、[ROV 手册](../tools/rov/README.md)与[维护手册](../tools/rov/MAINTENANCE.md)。
- [Runtime 手册](../runtime/README.md)与[AUV 工具](../tools/auv/README.md)：独立 AUV 配置和入口，不能只切服务替代固件切换。
- [测试与验收](testing/README.md)、[部署导航](deployment/README.md)、[串口协议](protocol/serial-protocol.md)。
- [最新双摄标定记录](calibration/20261011/README.md)与[帧率诊断](testing/camera-performance-analysis-20261011.md)：640×480 显示矫正与测距/导航验收分开。
- [日志分析](log-analysis.md)、[场地确认](field-details-20261009.md)与[有线网络](wired-network.md)。

## 历史资料

`reports/`、`reviews/`、日期命名的测试/部署/标定记录保留当次证据；`archive/` 保存已停用方案。历史 ROS 实现位于 `legacy/ros2/`，双模式方案已弃用。当前缺口以待办和项目状态为准，不从旧报告推断设备在线或功能可用。

[完整文档索引](catalog.md)提供全仓库链接。实机证据优先保存到 PC 的忽略目录，凭据和原始录像不进入源码提交。
