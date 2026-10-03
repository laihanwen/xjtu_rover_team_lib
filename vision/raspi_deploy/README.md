# raspi_deploy：树莓派双摄 NCNN 推理 + MJPEG 推流

## 1. 模块定位

本目录是**跑在树莓派上、不依赖 ROS 的双摄 NCNN 推理 + MJPEG 推流工具**，用于：

- 相机排线 / libcamera 的快速排查；
- 模型转成 NCNN 后的快速验证（不接 ROS，直接看画框效果）；
- Pi 4B 上的推理性能实测。

它**不替换** `src/auv_vision` 里的 C++ ONNX 节点 `auv_cucumber_detector`：前者是开发期的
调试 / 摸底工具，后者是比赛时跑在机器人上的正式节点，两者并存。

## 2. 目录内容

| 文件 | 说明 |
| --- | --- |
| `dual_cam_ncnn.py` | 主程序：双摄采集 + NCNN 推理 + 画框 + MJPEG 推流 + 多进程调度 |
| `mjpeg_stream.py` | 网页推流服务 `WebStreamer` / `MJPEGHandler` |
| `camera_probe.py` | 摄像头探测 `detect_cameras()`（Windows / Linux 双平台） |
| `config.yaml` | 全部可调参数 |
| `dual_cam.service` | systemd 服务单元模板 |
| `requirements.txt` | Pi 端依赖 |

## 3. 类别顺序警告

`config.yaml` 里的 `class_names` 顺序必须和所加载模型的 `metadata.yaml` 一致。
程序启动时会同时检查类别顺序与输入尺寸，不一致时会停用该路推理并打印原因，避免
画框类别张冠李戴。请优先统一训练配置、ROS 配置和此处配置，而不是反复手工换顺序。

## 4. 硬件前提

- Raspberry Pi 4B；
- CSI 摄像头（走 libcamera / Picamera2），需要 `rpicam-hello --list-cameras` 能枚举到；
- USB UVC 摄像头，设备节点在 `/dev/video*`。

## 5. 安装

```bash
sudo apt install -y python3-venv
cd <本目录>
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
# 让当前用户能访问摄像头设备，加完要重新登录 / 重启生效
sudo usermod -aG video <user>
```

## 6. 运行

```bash
# 直接运行（默认读取与本脚本同目录的 config.yaml）
python dual_cam_ncnn.py

# 指定其他配置文件
python dual_cam_ncnn.py --config /path/to/config.yaml
```

用 systemd 托管：先把 `dual_cam.service` 里的 `<RUN_USER>` / `<INSTALL_DIR>` 替换成本机
实际值，再执行：

```bash
sudo cp dual_cam.service /etc/systemd/system/dual_cam.service
sudo systemctl daemon-reload
sudo systemctl enable dual_cam.service    # 开机自启
sudo systemctl start  dual_cam.service    # 立即启动

systemctl status dual_cam.service
journalctl -u dual_cam.service -f         # 实时看日志，Ctrl+C 退出

sudo systemctl stop    dual_cam.service   # 停止
sudo systemctl restart dual_cam.service   # 重启
sudo systemctl disable dual_cam.service   # 取消开机自启
```

## 7. 浏览器访问

浏览器打开 `http://<树莓派IP>:8080`（端口可用 `stream.port` 修改），页面左右并排显示两路。
该服务监听所有网卡且没有登录认证，只应在可信、隔离的调试局域网中使用，不要直接暴露到公网。

## 8. 模型准备

权重不进 Git。本目录只需在 `config.yaml` 把 `cameras.<id>.model_dir` 指向包含
同名的 `model.ncnn.param`、`model.ncnn.bin` 和 `metadata.yaml` 的目录
（`models/artifacts/` 用于存放这些文件）。
模型导出使用 `../export/export_ncnn.py`。

程序按 Ultralytics 的方式执行 BGR→RGB 和等比例 letterbox。不要把配置中的 `pixel`
改回 `BGR`，也不要只复制 `.param/.bin` 而漏掉元数据。

## 9. 性能参考（Pi 4B 实测）

| 输入尺寸 | 单帧推理 | 说明 |
| --- | --- | --- |
| 416 | 约 391 ms | `ncnn_threads=3`，也是拐点；给到 4 线程无收益 |
| 640 | 约 760 ms | 分辨率翻倍，耗时约翻倍 |

416 输入对应理论上限约 2.5 次/秒（`1 / 0.391`）。所以 `infer_fps` 不要设得比硬件能力
高，设高了只会不停丢帧，不会真的更快。

## 10. 已知限制

- CAM0（CSI）当前只推流不推理（排线未到位）；
- 两路 `conf` 阈值尚未统一（0.25 / 0.40）；
- 无 ROS 接口，检测结果不会发布到 `/cucumber/detections`。
- 本工具会独占摄像头，不能与 ROS 相机节点同时打开同一设备；切换前先停止另一套程序。
