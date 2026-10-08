# ROS 2 历史实现（暂时弃用）

2026-10-08 起，因当前平台性能不足和现有 ROS 系统设计问题，暂停 ROS 相关功能开发、部署与验收。自动控制开发中心迁至根目录 `core/` 与 `runtime/`，详见 [轻量系统路线](../../docs/lightweight-transition.md)。

这里保留 ROS 节点、msg/srv、launch、参数和旧 package 构建文件供查阅。原生算法、串口实现和对应测试已迁入 `core/`；本目录不再是可直接构建的独立 ROS workspace。`COLCON_IGNORE` 阻止默认 colcon 扫描，根 CMake 和部署包均不包含这里。

需要复现迁移前的完整 ROS 布局时，在独立 checkout 检出 `8064e662b6a61679dd9403eff7fc3ef8ae24d9c5`。不要直接运行归档 launch 或将它与当前 Runtime 同时连接真实 UART。恢复 ROS 前需重新评估资源占用、接口和控制所有权，并完成单独验收。

本次迁移未卸载系统 ROS，也未升级 Ubuntu、ROS 或修改 STM32 固件。
