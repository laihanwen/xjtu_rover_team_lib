> 2026-10-11 最新状态：用户确认当前矫正参数已经验证、可用。已保存到 Pi /etc/auv-runtime/runtime.yaml，前视 640×480 矫正重启后保留。以下临时配置与不稳定性检查为计算过程记录；用户显示验收不替代导航、测距和独立几何验收。

# 前视第三批水下录像：临时显示矫正

会话 20261011T101503_217412Z，两段前视原图 AVI，共 3213 帧，640×480。棋盘沿用 11×8 内角点、15 mm。原尺寸 0.5 名义秒采样 183 帧，仅一个有效姿态；0.1 名义秒加密采样 894 帧、4 个姿态。CLAHE、3 倍双三次放大、OpenCV SB NORMALIZE_IMAGE/EXHAUSTIVE/ACCURACY 检测 358 帧，得到 27 个不同姿态。

5 参数模型 21 训练/6 留出，RMS 0.46220 px、留出 0.47432 px。固定 k3=0 重拟合这 27 张选帧，21 训练/6 留出，RMS 0.46539 px、留出 0.47533 px；采用该较简单模型进行临时显示。再约束切向畸变为零的模型误差上升至训练 0.56472 px、留出 0.71142 px。首个自由模型原始 K 映射无折叠（最小雅可比约 0.551），但主点明显偏离中心；不能仅因误差小就判定几何可靠。

敏感性检查：前 13 个姿态 fx=1075.21、fy=1040.60、cx=268.07、cy=208.12；后 14 个姿态 fx=711.94、fy=697.11、cx=111.03、cy=107.74。焦距差约 50%，主点也变化明显；同次录像交错留出不能证明独立重复性。结果标记 provisional_unstable_intrinsics，仅试验显示，不能用于测距/导航，也未设置 calibration_verified=true。

实机只更新 /run/auv-front-resolution-test/runtime.yaml 的 camera_front K/D 和 preview_rectify=true，原配置备份 /run/auv-front-resolution-test/front-r3-before.yaml；未改 /etc 持久配置。PC /preview 前视切 default；下视已有新标定保持。双摄恢复、帧数持续增加、帧龄 <0.12 秒、fault 为空。验证末现场已经出水，岸上实时图只证明链路可用，不构成水下矫正视觉验收。

本次未调整白平衡、曝光或锐化，运动保持关闭；没有导航、水池完整闭环或长期性能验收。重启 Pi 会移除临时前视配置并回到 /etc 原有前视设置。当前可在驾驶台选择前视原图。下一批几何验证需要更清晰、静止的棋盘、充分的距离/倾角和边缘覆盖。视频和图片仍在 build/calibration-front-20261011-r3，不入源码提交。

如需撤销此次前视试验（停止录制后执行，会短暂中断双摄）：

```sh
sudo cp /run/auv-front-resolution-test/front-r3-before.yaml /run/auv-front-resolution-test/runtime.yaml
sudo systemctl restart auv-runtime
```

用户确认后将前视字段写入持久配置，并同步临时配置的质量标记为 user_verified_20261011_display。未重启服务、未中断录像；当前运行进程的旧质量标记会在下次正常重启时更新。检查 auv 服务账户可读取持久 YAML（root:auv，0640）。下视白平衡仍为之前的临时开机设置，不由本次参数保存自动持久化。
