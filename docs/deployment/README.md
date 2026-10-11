# 部署导航

更新：2026-10-11。ROV 与 AUV 使用独立固件和匹配的 Pi 服务，UART 只能由一个控制入口占用。

## 当前操作

- [ROV 一键烧录与驾驶台](../one-click-flash-console.md)。
- [ROV 维护与回滚](../../tools/rov/MAINTENANCE.md)。
- [AUV Runtime 构建与部署](../../runtime/README.md)。
- [AUV 任务工具](../../tools/auv/README.md)与[标签测试入口](../one-click-tag-test.md)。
- [网络检查](../wired-network.md)与[项目状态](../project-status.md)。

部署前核对实际服务、有效配置、源码/二进制版本及当前安全状态。先保存 PC 本地日志、配置与 SHA256 清单；编译或文档更新不授权实机重启、烧录或运动。摄像头采集只打开设备一次，预览/算法/录像复用缓存。

## 历史部署证据

[10 月 6 日记录](pi-20261006/README.md)、[10 月 8 日记录](pi-20261008.md)仅描述当次部署，不作为当前配置模板。最新日期维护记录见[文档索引](../catalog.md)，当前设备版本必须重新核查。
