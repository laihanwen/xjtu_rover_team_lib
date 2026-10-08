# 前视视频重新筛选与试验标定

来源：桌面 `标定板/前视` 的五段 AVI。结果：桌面 `标定结果_前视视频`。
棋盘：Q12-240-15，11×8 内角点，方格 0.015 m；视频尺寸 320×240。

先以每 0.2 秒一帧扫描五段视频，共 1240 帧。SB 检测器对透视压缩的
棋盘漏检，传统自适应阈值棋盘检测能识别第 4 段的完整棋盘。
传统检测全视频扫描仅获得 9 个满足角点 RMS 位移 3 px 去重条件的姿态。
随后以约每视频帧扫描 `前视4.avi`（301 个可解码采样帧），检出 47 帧，
以 1 px 位移去重保留 40 帧。可用姿态主要位于约 3–8 秒。

| 模型 | 训练帧 | 剔除帧 | 留出帧 | 训练 RMS/px | 留出 RMS/px |
| --- | ---: | ---: | ---: | ---: | ---: |
| 五参数 plumb_bob | 30 | 2 | 8 | 0.451359 | 0.448875 |
| k3 固定为 0 | 30 | 2 | 8 | 0.449412 | 0.445621 |

五参数试验 K：

```text
330.922779    0          148.058620
  0        351.869457    141.719251
  0          0            1
D = [-0.257442375, -1.258184396, -0.073975390, 0.002494382, 5.385083269]
```

## 质量结论

这是**试验参数，不是通过部署验证的正式标定**。两种模型的误差接近，
但主点从 (148.06, 141.72) 变成 (83.24, 67.71)，相差 98.38 px。
这表明当前棋盘尺寸、位置和姿态覆盖不足，内参与畸变之间存在明显耦合。
同一短片段交错留出的验证帧也不能验证不同距离或整幅视场的测量精度。
不能仅凭约 0.45 px 重投影误差将这些参数用于正式 AprilTag 测距或定位。

两个结果目录中的 `ros_parameters.yaml` 均设置 `calibration_configured: false`，
现有下视配置和运行配置未变。
建议补拍原始分辨率、完整且清晰的棋盘：覆盖中心、上/下/左/右，
包含不同距离及绕两个轴的倾斜；镜头、对焦、密封窗口和水下配置保持一致。

## 文件

- `front_refined/selected_frames`：40 张筛选原图。
- `front_refined/selected_sources.json`：每张原图对应的视频帧号及时间。
- `front_refined/rectified_images`：对应的试验矫正图。
- `front_refined/preview_*.jpg`：左原图角点，右试验矫正结果。
- `front_refined/report.json`：误差、训练/验证来源、参数标准差、质量标记。
- `front_refined/camera_info.yaml`、`calibration.npz`：五参数试验结果。
- `front_k3_fixed`：用于稳定性对照的简化模型，不作为推荐替代参数。
- `front/detections.json`：五段视频 1240 帧的扫描记录。

矫正图使用 report 中 rectified_camera_matrix 和零畸变；原图使用原始 K/D。
这些候选参数的尺寸是 320×240，不能直接用于当前默认 640×480 ROS 输入。

## 复现（已有隔离 OpenCV Python 环境，fish）

```fish
python vision/calibrate_underwater.py /path/to/前视 /path/to/full_scan --interval 0.2 --classic-only
python vision/calibrate_underwater.py /path/to/前视4.avi /path/to/front_refined --interval 0.033 --classic-only --min-pose-distance 1
python vision/calibrate_underwater.py /path/to/前视4.avi /path/to/front_k3_fixed --interval 0.033 --classic-only --min-pose-distance 1 --fix-k3
```

工具默认不启用生成的 ROS 参数；只有完成独立质量验证后才应启用。
