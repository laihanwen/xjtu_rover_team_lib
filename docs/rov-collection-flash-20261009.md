# 采集用 ROV 实机恢复记录

2026-10-09 用户要求烧录，并确认停止任务/录制、推进器断电或可靠固定。独立 `Copy_cup` 构建 0 错误、0 警告，HEX SHA256 为 `acbf329a412454c478f2e302466e2f9a5a3eb5cbf244ba9fc799675bc5dc3c2d`，与原稳定 ROV 基线一致。采集助手运行在 PC，不作为 MCU 新算法。

原 AUV 固件已备份，ROV 烧录后完整 32 KiB 回读校验通过；前后均确认 DISARM 与八路中立输出。烧录记录位于本地忽略目录 `build/maintenance/20261009T132837292642Z`，`arm_sent=false`。

Pi `auv-tag-docking` 停止并取消自动启动；恢复 `auv-runtime` 摄像头服务与 `auv-rov` 独占 UART，两者 active 并恢复自动启动。AUV 独立程序及配置保留，不以当前部署宣称 AUV 可执行。

带采集助手的原驾驶台已启动：`http://127.0.0.1:8767/#collection-panel`。启动时手柄未连接，PC ROV 控制链路未建立，不能宣称已可遥控；启动未发送 ARM。需连接手柄并按原驾驶台门控操作。只读采集台 8768 可继续用于人工搬运观测，不能作为 ROV 控制入口。
