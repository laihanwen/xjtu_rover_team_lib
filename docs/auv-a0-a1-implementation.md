# A0 / A1 实现与标定交付

2026-10-07。本次交付完成 A0/A1 软件增量；尚未完成树莓派部署和真实水下验收。范围遵循 [开发边界](auv-development-boundary.md)。

## 已实现

| 阶段 | 行为与验收证据 |
|---|---|
| A0 独立控制 | 独立 AUV 固件构建产物，ROV 工程保留；AUV 禁止 RC 回退。ARM 同时要求固件 commissioning、深度零点/方向、运动增益、限深和 IMU 校准。默认全部关闭。 |
| A0 启动与定位 | 自动启动前检查遥测、视觉、记录和静止定位原点；定位会话失效或改变即停止搜索，禁止运行中偷偷重置原点。 |
| A0 单一所有者 | 双摄各自采集后共享缓存；录像独立工作线程；串口使用非阻塞独占锁，与 ROV pyserial exclusive 模式互斥；systemd 服务相互冲突。 |
| A0 机载记录 | 每次启动独立 run_id、配置快照、发布清单、事件日志、前/下视分段 MJPEG 与 JSONL 帧索引；记录帧序号、时间、丢帧、遥测、字节偏移。磁盘不足/记录失效时禁止运动或进入 FAULT。 |
| A1 主动观测 | 经过实测标定的相对原点航点搜索；连续定位转换至机体 surge/sway；速度、半径、定位时效和航点超时约束。识别标签后保持当前位置并建图。 |
| A1 标签与地图 | 标签 ID 可限定，连续多帧确认后触发建图；单黄色边统一旋至下方；检查完整网格与唯一基准边；恰好四个稳定圆/方锥才能完成；保存九格地图、图像及真实轮廓。 |

`a1_observation` 在建图证据保存成功后进入 COMPLETE 并撤销推进输出。这个 COMPLETE 仅表示 A1 观测阶段结束。比赛要求的建图后上浮遍历属于 A2，未在本次新增。

## 构建与启动

树莓派原生构建需要 OpenCV >= 4.7、yaml-cpp、C++17 和现有 Runtime 依赖。执行仓库现有 Runtime 构建，不要求 ROS 常驻：

```fish
cmake -S . -B build-lightweight -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-lightweight -j 3
ctest --test-dir build-lightweight --output-on-failure
build-lightweight/runtime/auv_runtime --check-config runtime/config/pi-auv-observation.yaml
build-lightweight/runtime/auv_runtime runtime/config/pi-auv-observation.yaml
```

`--check-config` 仅校验配置，不开串口、摄像头或推进器。模板为 debug、关闭运动、关闭自动启动/ARM 和网络视频；可先记录与离线调试。服务模板 `runtime/deploy/auv-observation.service` 不会自动安装或启用；配置真实设备后再部署，避免与 ROV 同时占用硬件。

Windows 独立固件构建（不烧录）：

```powershell
./tools/auv/Build-AuvFirmware.ps1
```

产物在 `firmware/stm32/MDK-ARM/AUV_A0/`，原 ROV 工程仍由 `tools/rov/Maintain-Rov.ps1 -Action build` 构建。AUV 输出限幅复用低速基线，增益和运动方向必须重新核验。

## 实机标定与运行条件

先电机断电、拆桨或安全固定推进器。不要直接把标定标志改为 true 来绕过检查。

1. 确认 MCU 上实际安全输入/急停接口、ARM 拒绝与超时停机。未知 GPIO 不在本次猜测接线。测量表面深度零点，填 `AuvAutonomyConfig.h`；验证绝对目标深度、修正方向、surge/sway 增益、限深，再填写 commissioning。MCU 已扣除表面零点时 Pi 的 `localization.depth_zero_m` 保持 0，避免重复扣除。
2. 记录双摄实际设备路径并启用 front；标定下视内参/畸变、camera_to_body、摄像头/压力计位置、IMU 符号与偏置、池深。相机分辨率必须与内参匹配。用 `auv_localization_replay` 验证记录数据后才设置 `calibration_verified`。
3. 实测投放点到中心的安全航点、相对原点坐标、搜索半径、目标深度与航向，填 observation_search。验证机体 X 前/Y 左转换与路线净空，再启用 corridor_calibrated、motion 标定标志及搜索。不能凭默认值航行。
4. 采集真实水下黄色单边/黑网格/四锥/标签视频，检查光照、视野、姿态、误检和连续定位；按样本调整阈值和允许标签 ID。模板仅演示 tag36h11，实际标签须核对。
5. 填发布清单 Git commit、固件 SHA256、标定编号。选择 autonomous 且显式开启 auto_start/auto_arm，验证断网自主启动、录像、搜索、标签证据和地图方向。任何阶段失效应进入 FAULT/DISARM。

记录 `.mjpg` 是串接的 JPEG 帧，不是固定帧率 MP4。JSONL 保存实际采集时间及丢帧；遥测为记录时的最近状态，并非硬件同步。原始记录应保留，回放/转码使用索引时间重建，不能把文件帧率当实测帧率。事件日志的 Unix 时间与 steady 时间，以及 manifest 的时间基准用于关联证据。

## 本轮验证与限制

Windows 可移植测试覆盖搜索坐标转换、定位过期/会话变化、自动原点门控、配置拒绝、四方向单边地图、额外锥拒绝、真实轮廓、分段记录和磁盘不足。固件主机测试覆盖默认未标定 AUV 限速/限深与既有安全协议；AUV 和 ROV 均经过 Keil 编译。

Runtime 进行 Linux 目标语法检查；这不能替代树莓派完整链接、设备时序与实际性能测试。本轮树莓派 SSH 连接超时，未部署、烧录或发送 ARM。真实录像吞吐、失效停机、水下定位漂移、完整九格视野与实际搜索路线仍需实机验收。A1 数据与运动验证通过后才能进入 A2 遍历开发。
