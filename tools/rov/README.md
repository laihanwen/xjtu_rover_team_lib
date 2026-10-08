# ROV 操作与开发手册

当前说明基于 2026-10-07 源码。设备实际版本需检查服务与部署报告，不能根据此文件推断已烧录。历史合并说明保存在 [LEGACY_NOTES.md](LEGACY_NOTES.md)，其中旧参数不作为操作依据。

## 运行组成

| 端 | 入口 | 职责 |
|---|---|---|
| PC | `Start-RovTrial.ps1` / `trial_control_web.py` | 手柄、显式 ARM、驾驶台、日志、双摄录像 |
| Pi | `trial_server.py` / `auv-rov.service` | 独占 USART2 链路、CRC、许可与遥测 |
| Pi | `auv_runtime` / `auv-runtime.service` | CSI 下视、USB 前视、视频与状态；ROV 模式关闭串口和自主运动 |
| MCU | `MDK-ARM/Copy_cup.uvprojx` | 姿态/深度控制、混控、PWM 与失效处理 |
| PC 可选代理 | `console_view.py` | 8768 展示现有 8767 服务；不另起控制器 |

默认地址为 Pi `192.168.137.150`、控制桥 TCP `8888`、视频 HTTP `8080`、PC 驾驶台 `8767`。参数覆盖方式见各脚本 `--help`；网络地址不是协议的一部分。

## 启动与停止

在仓库根目录执行：

```powershell
./tools/rov/Start-RovTrial.ps1
# 分析振荡时可选：
./tools/rov/Start-RovTrial.ps1 --diagnostic-log
```

脚本使用本机 `build/joystick-deps`，不向系统 Python 安装依赖。RadioMaster Pocket 选择 USB Joystick/HID；只运行一个控制进程。

四轴回中、独立安全许可（默认左肩上档）后，操作者点击 ARM。STOP、Esc、独立安全许可撤销、手柄断开或控制许可失效会停止；重连不自动恢复 ARM。水平需岸上放平、未 ARM 时手动校准，每次 MCU 重启需重做。硬件急停由实机独立装置承担。

## 控制与参数

右杆前后控制前后平动、左右控制偏航；左杆前后控制升降、左右控制侧移。映射配置见工具内的 mapping 配置与 [历史映射检查](../../docs/reviews/rov-controller-pid-20261006.md)。

参数来源：`firmware/stm32/MDK-ARM/AuvRovConfig.h` 与 `Mate.c`。中位1492、起转偏移±48，低速±100、提速±125 μs；速度只在未 ARM 时切换，默认低速。保持当前已验证的 PID 极性，不从历史记录恢复旧参数。

回中自动保持源码已替代 SA/SD 门控：主动转向或升降释放相应锁定，回中重锁当前航向/深度；横滚俯仰保持校准水平。新源码尚未部署。详情见 [保持模式](HOLD_MODES.md)、[速度档位](SPEED_MODES.md)。

串口：USART1 H30 460800；USART2 Pi CRC 115200；USART3 M10 115200。M10 允许有界负相对读数，新鲜度窗口3000ms；水面零点未校准，不能把负数当作实际负水深。“未锁定”表示控制目标状态，须与“无有效读数”区分。

机械爪/云台禁用；没有漏水传感器。八推接线、矩阵与极性以当前代码及实测为准，T3右前下、T4左前下。旧 `Rover.py` / A5 TCP 入口不能连接 CRC 手动桥。

## 日志与录像

[诊断日志](LOGGING.md)：默认每5秒记录 IMU、PID、模式、数据年龄和操作者状态。`--diagnostic-log` 额外开启10Hz快照。新64字节诊断包含保持原因和优先混控运动比例；它不是实测推力，也不是完整控制环记录。

录像由 PC `video_recorder.py` 独立采集两路原始 JPEG，保存 AVI 与逐帧时间戳到 `data/rov-recordings`；每分钟分段，不需要 ARM。来源必须是 CSI 下视、USB 前视，并提供源时间戳和来源标记。缺少接口元数据时需升级 Runtime，不能绕过来源校验。

双摄优化源码已有 MJPEG 缓存、长连接和最新帧策略，模板目标320×240/30fps。**优化 Runtime 尚未完成本轮实机部署和验收**；2026-10-07 最近连接尝试中 Pi SSH/Ping 超时。30fps 是目标，不是测量结果。

## 编译、部署与检查

使用 [Maintain-Rov.ps1](MAINTENANCE.md)：`build` 编译、`flash` 备份并烧录/读回、`deploy` 构建部署 Pi、`check` 检查、`all` 顺序执行。日常更新优先维护脚本；首次安装见 Runtime 部署手册。脚本不会 ARM，固件/视频/控制服务需按版本一起验证。

固件主机测试、协议测试、录像测试和实机验证范围详见 [测试策略](../../docs/testing/strategy.md)。设备日志和报告分别位于 `logs/rov`、`build/maintenance`，不提交运行产物。

### 驾驶台初版布局（2026-10-07）

页面资源位于 `tools/rov/web/`，由 `trial_control_web.py` 提供；启动方式和 ARM 许可规则保持原样。顶部为水平校准、速度档位、ARM/STOP；随后是 CSI 下视与 USB 前视 MJPEG 预览及双摄录制；下方是传感器、闭环状态和轨迹区域，底部是会话事件和日志保存状态。

点击“连接双摄预览”直接连接配置的树莓派视频端口。暂停预览不停止录制；摄像头服务须支持 `/api/camera/{down,front}.mjpeg`。录制继续使用既有 DatasetRecorder，显示录制错误、帧数和接收 FPS。尚需实机确认回传与录制性能。

轨迹区域当前支持每秒姿态/深度采样、最近 120 秒曲线、最多 3600 条 JSONL 导出；刷新页面会清除浏览器会话记录，长期记录由后台日志保存。无效遥测记录为空值。二维轨迹预留位置，等待可信定位输入，不能用于当前导航。底部事件仅为浏览器会话输出，不是 MCU 原始串口日志。

视觉参考 [Tabler](https://github.com/tabler/tabler) 的卡片层次和仪表盘布局；CSS 为本地独立实现，不依赖外网 CDN。

### 轻量二维定位首版

已添加独立定位线程、二维轨迹接口、原点重置及离线回放工具。实际运行需填写并验证水下相机、安装关系、池深和深度零点，默认关闭。详见 [定位说明](../../docs/localization.md)。先前布局中的定位占位已由真实接口驱动；定位服务不可用时保持无有效位置。

摄像头舵机移植与 SC/SI 映射见 [摄像头舵机说明](../../docs/camera-servo-sc-si.md)。2026-10-07 已确认 SI=axis4、SC=axis6，实测端点已写入配置；实机方向尚未验证。
