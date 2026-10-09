# ROV / AUV 轻量系统标定接入

> 2026-10-10：用户反馈前视显示矫正效果良好，保留现有试验参数；已矫正的屏幕录制不用于再次标定。正式前视度量内外参仍未验证，见 [最新筛查](../20261010/front-screen-review.md)。

已修改 `runtime/config/runtime.yaml`、`pi-rov.yaml`、`pi-auv-observation.yaml`、
`pi-auv-task-one.yaml`。摄像头均为 CSI 下视和现有 ROV 配置中的 USB 前视设备；
双摄均启用，采集尺寸 320×240，K/D 来自本目录的 down 和 front_trial 报告。
下视参数复制到 localization，但外参、传感器偏移和泳池深度等仍待测量，
`calibration_verified` 保持 false；运动与自动 ARM 保持关闭。
独立表面标定参数没有套用水下参数。

## 图像处理

- `camera.preview_rectify` / `camera_front.preview_rectify` 控制显示矫正。
- 每个采集线程缓存 OpenCV remap 表，保持原始 K 与尺寸，不裁切、不缩放内参。
- JPEG 快照、MJPEG 视频和 HLS 都使用矫正显示帧。
- 下视采集线程按任务阶段缓存水下/表面 remap 表，并发布独立矫正帧。
- AprilTag、九宫格、锥识别和遍历 PnP 网格定位复用这个矫正帧；阶段切换
  发生在采集之后时，从该帧原图按当前阶段重新矫正一次。
- localization 在矫正图上提取/跟踪特征、拟合平面，使用对应 K 和零畸变归一化。
  矫正黑边及跨无效边界的光流窗口不参与定位。共享帧只有 K/D 与 localization
  配置一致才复用；独立原始录像回放在适配器内部先矫正再跟踪。
- 上浮遍历使用独立验证的表面 K/D；此阶段暂停水下平面光流，位置来自
  矫正图的绝对网格定位。显示与录制元数据随表面阶段切换。
- AUV 本机录像保留原图，帧索引标记 `image_space: raw`。
- ROV 电脑端 AVI 录制来自矫正 MJPEG，会话 `session.json` 保存每路
  `camera_calibration`，包含 K、原始 D、显示帧零 D、标定 ID 和质量。

显示使用原始 K，与之前桌面预览的 alpha=0 新 K 不同，边缘可能有黑色无效区。
显示与电脑录制中的矫正图使用 K 和零 D；原图及本机录像使用原始 K/D。
`/api/status.camera_calibration` 可查看这两种图像空间和前视试验质量标记。
矫正开关关闭只影响预览，自主算法仍使用矫正帧。
`/api/status.vision_image_space` 和 `/api/localization.image_space` 标明算法图像空间。
自主模式缺少下视内参会拒绝启动。

配置分辨率与标定尺寸不一致会拒绝启动；实际帧尺寸不一致会拒绝处理。
不自动把 320×240 参数缩放到其他传感器采集模式。
模拟测试使用 `runtime/test/runtime_fixture.yaml`，避免真实畸变参数改变合成任务场景。

## 构建与运行（Pi / Ubuntu fish）

```fish
cmake -S . -B build-lightweight -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build-lightweight -j 3
ctest --test-dir build-lightweight --output-on-failure
# 选择一个模式，先确认相机和未ARM状态：
./build-lightweight/runtime/auv_runtime runtime/config/pi-rov.yaml
# 或运行 AUV 观察模式 / 任务一：
./build-lightweight/runtime/auv_runtime runtime/config/pi-auv-observation.yaml
./build-lightweight/runtime/auv_runtime runtime/config/pi-auv-task-one.yaml
```

三个运行命令是互斥选择，不能并行占用同一摄像头。现有服务读取 `/etc/auv-runtime`
配置，更新仓库模板不会自动覆盖设备上的配置；部署时应合并这些摄像头字段和
localization 内参/尺寸，保留设备已测量的其他设置。

## 验证状态

本机通过四个 YAML 配置的重复键、K/D 与标定来源一致性、尺寸、有限 remap 表、
原始定位内参与关闭运动门检查。ROV 录制测试 8 项通过，1 项可选独立解码测试
因未安装依赖而跳过；已验证会话保留矫正元数据。Python 语法和 diff 检查通过。

新增 C++ `camera_rectifier` 检查像素矫正、零畸变恒等、原图保留、有效区掩膜和尺寸拒绝；
光流定位测试比较非零畸变的原图回放与共享矫正图在平移后的估计结果，防止重复矫正；
配置测试覆盖分辨率错配与损坏前视参数。这些 C++ 测试尚未执行：本 Windows
主机缺少 OpenCV C++ 开发库，CMake 在 find_package(OpenCV) 阶段失败。
树莓派 `192.168.137.150` SSH 连接超时，未部署或重启实机服务。

前视模型不稳定，当前只应用于预览/录制。它不是测距或定位的有效标定。
实机需要验证画面方向、窗口/镜头配置、双摄帧率及矫正编码的 CPU 开销。
首次硬件检查保持 DISARM，电机断电或拆桨。
