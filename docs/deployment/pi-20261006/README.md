# 树莓派部署与自检记录

日期：2026-10-06，Asia/Shanghai。主机：`pi@192.168.137.150`，Debian13/aarch64。报告中UTC时间为机器生成的原始时间。

## 部署结果

- 源码：`/home/pi/auv-releases/20261006-rov`；稳定入口：`/home/pi/auv-current`。原 `/home/pi/auv` 保留。
- 轻量服务：`auv-runtime.service`，开机自启；程序 `/usr/local/bin/auv_runtime`、`auvctl`；配置 `/etc/auv-runtime/runtime.yaml`。
- ROV服务：`auv-rov.service`，开机自启；程序 `/usr/local/lib/auv-rov/`；配置 `/etc/auv-rov/rov.env`。监听TCP8888，**仅预览帧，不打开UART、不ARM**；systemd PrivateDevices进一步隔离硬件设备。
- 网页：[192.168.137.150:8080](http://192.168.137.150:8080/)，只读状态和HLS。
- USB摄像头使用稳定by-id路径，采集和视频均设为320×240，硬件H264编码；640×480原配置留存 `/etc/auv-runtime/runtime.yaml.640x480`。
- `serial.device`为空、运动禁用、自动启动和自动ARM关闭；任务保持INIT。STM32接线及已烧录版本未确认，未发送真实串口帧，未烧录STM32。

用户要求继续处理供电告警，因此未执行安装脚本中会因历史/当前欠压退出的路径；采用单线程构建、分别运行原生测试，再安装已验证程序。未更改树莓派OS、ROS或Python版本，仅通过apt补充python3-opencv。

## 自检结果与边界

| 项目 | 结果 |
|---|---|
| 轻量核心、PTY安全、离线视觉、ARM模拟、自主模拟 | 5/5通过，全部为文件/虚拟串口 |
| STM32协议/安全/遥控/执行器/深度宿主测试 | 9/9通过 |
| ROV输入映射与组帧 | 3/3通过 |
| systemd服务与开机自启 | 两个服务active/enabled；最终NRestarts=0 |
| 控制socket、安全配置、INIT/DISARM、无故障、网页API | 通过 |
| 实际摄像头与HLS（15秒采样） | 通过；视觉11.47Hz，HLS列表和片段HTTP200 |
| PC跨机器HTTP与ROV分包接收/空闲断开 | 通过；仅发送中性帧到预览服务 |
| 供电 | **未通过**：自检0x50005，最终读取0x50000；欠压间歇出现，历史欠压/降频记录仍在 |
| STM32/心跳/IMU/深度/推进器实际输出 | **待验证**：没有确认真实串口及固件 |
| 运动方向、相机内参、定深/舵机标定、30分钟耐久 | 未执行 |

640×480视觉约3.07Hz未达10Hz目标，部署降为320×240后通过短时相机检查。原生运行时限制OpenCV内部线程为1，避免与自身的采集/视觉/控制线程竞争；测试增加有界启动等待，模拟视觉测试使用独立2秒位姿时限，不修改实机0.5秒位姿安全门。首次测试未通过后修正上述问题，再完整运行5项原生测试全部通过。

这次完成的是安全部署和短时软件/摄像头自检，不能将结果解释为完整带推进器验收。温度约41.3°C。未重启机器、未清除欠压历史、未注入真实漏水或急停故障。

原始报告：[基础检查](preflight.json)、[相机检查](camera.json)、[状态收集](collect.json)。树莓派同样保存在 `/home/pi/auv-test-reports/`。

## 日常查看

```sh
auvctl status
systemctl status auv-runtime auv-rov --no-pager
journalctl -u auv-runtime -u auv-rov --no-pager -n 50
cd /home/pi/auv-current
sh runtime/deploy/run_bench.sh preflight
sh runtime/deploy/run_bench.sh camera 15
```

推进器循环请求已由用户明确取消，未实施，不得恢复发送。用户确认推进器可靠固定、STM32烧录本次合并版、Pi连接STM32 USART2。合并版默认USART2为0xA5遥控入口，USART3为自主CRC入口；目前不向USART2发送CRC帧。补测CSI：SSH首次连接超时，随后恢复；识别到OV5647，rpicam-still成功采集640×480图像，OpenCV解码确认有效，保存在Pi的 `/home/pi/auv-test-reports/csi/csi-check.jpg`。这是CSI识别/静态采集验证，尚未将CSI接入轻量运行时或验收连续帧性能；运行时仍使用USB摄像头。

## 双摄优化与重启后验证

上述单摄记录为前一阶段结果。现已参考补充目录 `two_camera.py` / `two_camera_fast.py`，将轻量运行时改为 USB 与 CSI 并行采集，最新帧覆盖，共用一个显示编码器。USB 保留为任务一视觉输入，CSI 接入前视采集/预览；实际安装朝向尚未校准，CSI 未接入海参或转盘算法。CSI 增加有界 MJPEG 缓冲、8 秒首帧预热、连续采集后的 2 秒无帧重启和子进程清理。网页左 USB、右 CSI，独立 JPEG 接口同样可访问。

用户断电重启后恢复部署。开机检查发现静态 IP 就绪前 HTTP 绑定失败，已将服务设置为等待 `network-online.target`。同时发现 Pi 硬件编码后续 HLS 分段缺少参数头，HTTP 200 不代表可以解码；编码参数排查期间驱动曾报错并无法重新打开，重启后恢复打开，但分段问题仍存在。因此当前配置显式使用单线程 `libx264`、`ultrafast`、`zerolatency`，视频 640×240、15 fps，两路采集均为 320×240，CSI 请求 15 fps。硬件编码 HLS 尚未验收。

最终原生测试 **7/7 通过**，包含 MJPEG 分包/缓冲上限、双相机并行、CSI 卡住后 USB/控制继续运行、CSI 超时重启/恢复和子进程退出清理。模拟测试没有访问真实 UART。

实机预热完成后连续采样 30.13 秒：

| 指标 | 实测 |
|---|---|
| USB 实际采集 | 49.39 帧/秒；驱动实际帧率高于请求值，按计数测量 |
| CSI 实际采集 | 15.00 帧/秒 |
| USB 任务视觉处理 | 12.18 帧/秒 |
| 控制循环 | 20.02 Hz |
| 两路最新 JPEG | 均成功解码为 320×240 |
| HLS 后续分段、中途接入 | 640×240 H.264、15 fps；2 秒解码无错误 |
| 运动与串口 | INIT、DISARM、motion=false、serial=false、heartbeat=0 |
| 两个服务 | active/enabled；轻量服务 NRestarts=0 |
| 供电 | 仍间歇欠压，采样结束 0x50005；不计入硬件整体通过 |

原始报告：[双摄采样](dual-camera.json)、[HLS 解码](dual-hls.json)。首次采样在 CSI 首帧出现前启动，前三个样本未就绪而未通过，保留于 [启动阶段采样](dual-camera-startup.json)；最终验收在有界预热就绪后开始，未放宽运行时 0.5 秒帧新鲜度条件。

当前网页：<http://192.168.137.150:8080/>。本次没有发送推进器指令；USART2 接口和真实 STM32 遥测/输出验收仍待后续单独完成。30 秒采样不代替长期耐久测试。
