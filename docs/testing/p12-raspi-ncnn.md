# P12 海参 YOLO 树莓派 4B NCNN 部署验收
本文档记录 P12 海参 YOLO 模型在 Raspberry Pi 4B 上以 NCNN 格式部署的实测性能、
固定视频回归方法和已知限制。下文所有性能数字都是本机实测值，不是厂商或框架的标称值。
## 1. 目的
- 记录 NCNN 部署在 Pi 4B 上的单帧推理延迟与端到端延迟。
- 固定一套现场视频回归流程，供后续视觉改动做前后对比。
- 明确当前模型类别顺序与 `vision/configs/classes.yaml` 的差异，避免误用。
## 2. 硬件与环境
| 项目 | 配置 |
|---|---|
| 计算平台 | Raspberry Pi 4B，4 核 |
| 摄像头 | USB UVC 摄像头 + CSI 摄像头（libcamera / Picamera2） |
| Python | 3.13 venv |
| ncnn | 1.0.20260526 |
| opencv-python | 5.0.0.93 |
| numpy | 2.5.3 |
| 模型 | YOLO11n，NCNN 格式 |
权重和 NCNN 产物属于 `models/artifacts/`，已被 Git 忽略，不得提交进仓库。
## 3. 性能账本
下表为 Pi 4B 实测值，非标称值。
| 输入尺寸 | `ncnn_threads` | 单帧推理耗时 |
|---|---|---|
| 416 | 3 | 约 391 ms |
| 640 | 3 | 约 760 ms |
| 416 | 4 | 与 3 线程持平甚至更差 |
线程数拐点：3 线程已是当前最优，给到 4 线程（Pi 4B 满核）没有收益甚至更差。
端到端延迟（含取帧、画框、编码、推流）约 0.9 s，瓶颈在 CPU 推理，不在推流。
因此 416 是当前选定输入尺寸，继续提高分辨率只会放大 CPU 推理瓶颈。
## 4. 固定视频回归
仓库「开发约定」要求视觉改动必须能用固定视频回放复现，本节流程为硬要求。
- 回归素材为 4 段现场采集视频 `1.mp4` ~ `4.mp4`。
- 视频本体不进 Git，仓库 `.gitignore` 忽略 `/videos/*`。
- 复现方法：把视频导入，逐帧喂给 NCNN 推理，记录每一帧的检测结果。
- 记录内容：各类别得分、能否稳定检出 `sea_cucumber`，以及漏检和误检的帧区间。
- 判读标准：四段视频都能稳定检出 `sea_cucumber` 才算通过；出现持续漏检时必须登记为
  回归失败。
本文档的回归结论以实测为准，视频文件需向作者索取。
## 5. 已知限制
- 类别顺序与 `vision/configs/classes.yaml` 不一致：本模型为
  `sea_cucumber / starfish / turtle`，后续重训时统一。
- 模型权重本身未进 Git，需要时向作者索取，或用 `vision/export/export_ncnn.py` 重新导出。
## 6. 关联文件
- `vision/export/export_ncnn.py`：NCNN 导出脚本。
- `vision/raspi_deploy/`：Pi 端部署工具。
- `models/manifests/`：模型清单，记录哈希与评估指标。
