# 独立 AUV 工具

更新：2026-10-10。只提供独立 AUV 构建和任务工具，不使用已弃用的 AUV_ROV_DUAL 或会话切换。

| 工具 | 职责 | 操作边界 |
|---|---|---|
| Build-AuvFirmware.ps1 | 构建 AUV_A0 / AUV_TAG_DOCK | 不烧录、不 ARM |
| Flash-AuvFirmware.ps1 | 烧录明确配置的固件 | 需当前实机安全确认 |
| Tag-Test.ps1 / Check-Tag-Test.cmd | 检查固件记录、配置、服务与启动门 | 不发送 ARM |
| Start-Tag-Test.cmd | 启动匹配 Runtime 服务待命 | 不自动开始任务或 ARM |
| Tag-Test.ps1 -Action task | 门控通过后请求任务 WAIT_ARM | 仍需显式 ARM，不能绕过未标定门 |
| review_manual_task.py | 离线视频标签可见性和日志证据复盘 | 不套用未知图像空间 K/D，不控制设备 |
| Flash-And-Console.ps1 | 兼容的一键入口，目前构建独立 ROV | 见一键说明，不能按目录名误认为 AUV 烧录 |
| switch_mode.py | 已停用的历史命令 | 拒绝执行会话交接 |

操作依据：[标签入口](../../docs/one-click-tag-test.md)、[一键 ROV](../../docs/one-click-flash-console.md)、[人工复盘](../../docs/reports/20261010/manual-task-review-20261010.md)。

离线复盘示例（修改为本机素材和已确认墙钟区间）：

```sh
python tools/auv/review_manual_task.py --videos /path/front.mp4 /path/down.mp4 --logs /path/diagnostic-session --start 2026-10-09T22:50:02+08:00 --end 2026-10-09T22:56:31+08:00 --output build/manual-review
```

需要隔离 Python OpenCV（含 aruco）；导出视频的检测统计不等同真实 Runtime 检出率。剪辑素材不能按单个 UTC 偏移关联日志，必须结合原始帧索引。
