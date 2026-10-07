# 部署与运行手册

本目录用于保存现场部署、启动步骤、联调手册以及环境运行记录。

## 目录内容

- [pi-20261006/README.md](./pi-20261006/README.md) — Raspberry Pi 现场部署与工具链记录
- [pi-20261006/camera.json](./pi-20261006/camera.json) — 摄像头配置与采样参数
- [pi-20261006/dual-camera-startup.json](./pi-20261006/dual-camera-startup.json) — 双摄像头启动配置
- [pi-20261006/dual-camera.json](./pi-20261006/dual-camera.json) — 双摄像头联调配置
- [pi-20261006/dual-hls.json](./pi-20261006/dual-hls.json) — HLS 与视频输出配置
- [pi-20261006/preflight.json](./pi-20261006/preflight.json) — 预检项与现场检查配置

## 阅读建议

- 先完成 [项目状态](../project-status.md) 了解当前阶段与安全门。
- 再参考 Pi 现场部署说明，确认网络、启动顺序和摄像头配置。
- 若需要实机排错，优先查看 `pi-20261006` 目录中的启动与预检记录。
