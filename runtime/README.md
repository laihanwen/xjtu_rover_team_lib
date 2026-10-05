# 轻量级 mission-one 运行时

这个 C++ 进程与 ROS 节点共享 `auv_core`。它仅实现第一阶段任务：AprilTag、3×3 网格、锥形物分类、路径规划和网格遍历。STM32 继续负责姿态/深度 PID、混合器控制以及硬件心跳故障保护。

`mission.profile` 默认为 `task_one`。共享Mission FSM已经定义完整比赛阶段；设置为`full`后，四锥阶段会继续进入海参、抓取、运输、释放、转盘、返航和上浮流程。但在前视视觉和后续运动控制接通前，完整模式会按阶段超时进入FAULT，不能视为可下水的完整任务配置。

## 原生构建

在 Debian 13 / Raspberry Pi OS 上，确保已安装 `cmake`、`ninja-build`、`g++`、`libopencv-dev`、`libyaml-cpp-dev`、`libcpp-httplib-dev` 和 `ffmpeg`，然后执行：

```sh
cmake -S . -B build-lightweight -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-lightweight -j 3
ctest --test-dir build-lightweight --output-on-failure
```

原生 CTest 覆盖核心地图/规划/状态校验，以及 PTY 串口故障测试。如果系统 Python 也已安装 OpenCV 和 NumPy，还会额外加入合成的 AprilTag / 网格 / 锥形物视频回放，以及虚拟 STM32 的 ARM / ACK / 限制 / 漏水 / 进程退出测试。这些测试仅使用伪终端，不会访问真实串口。

复制并编辑 `runtime/config/runtime.yaml`。示例配置为 `debug` 模式并默认拒绝运动：`motion_commands_enabled: false`，串口设备为空，控制方向和相机内参均未校准。生产环境中的相机源必须使用稳定的 `/dev/v4l/by-id/...` 符号链接。`file:/absolute/path/video.mp4` 可用于离线回放，但若未完成标定，运动仍然保持禁用状态。调试模式启动时处于 INIT 和 DISARM 状态，并需要执行 `auvctl start`；它不会在故障后自动恢复运动。

```sh
./build-lightweight/runtime/auv_runtime runtime/config/runtime.yaml
./build-lightweight/runtime/auvctl status
./build-lightweight/runtime/auvctl start
./build-lightweight/runtime/auvctl disarm
```

`auvctl` 通过 `/run/auv-runtime/control.sock`（模式 `0660`）进行控制，适合通过 SSH 使用。SSH 用户必须属于 `auv` 组。执行 ARM 还需要满足额外条件：`auvctl arm --confirm SAFE_TO_ARM`，并且必须具备新鲜且安全的 STM32 STATUS、VISIT_CONES、已校准的运动配置，以及显式启用的运动控制。`pause`、`abort`、`disarm` 和故障条件都会撤销运动并请求 DISARM。若 Linux 异常退出，STM32 心跳丢失仍然是最终的安全屏障。

## 自主模式

完成相机、控制方向、速度限制和真实 STM32 安全验收后，比赛配置可以设置 `operation.mode: autonomous`、`auto_start: true` 和 `auto_arm: true`。自主模式会先保持 DISARM，等待启动延时，并要求相机采集、视觉处理与安全 STM32 STATUS 在 `startup_stable_sec` 内持续新鲜，然后自动启动 Mission；进入 `VISIT_CONES` 且完整 ARM 安全门仍满足时，才自动发送一次 ARM 请求。自主模式拒绝远程 `start`、`arm`、`pause`、`resume`、`abort` 和 `reset`，只保留只读 `status` 与紧急 `disarm`。紧急 DISARM 会把任务锁定到 FAULT。

STM32 heartbeat还受独立的控制循环看门狗约束。若控制循环超过 `safety.control_watchdog_timeout_sec` 未完成一次周期，即使串口线程仍存活，也会停止heartbeat、请求DISARM并等待STM32自身的heartbeat failsafe生效。这避免“通信线程健康但控制决策已经卡死”时维持危险输出。

每次系统上电最多允许一次自主任务启动。首次自动 START 会建立 `/run/auv-runtime/control.sock.autonomous-started` 锁存；服务崩溃或被 systemd 重启后不会再次自动开始或 ARM。锁存只在整机重启后清除。不要手工删除锁存来绕过现场安全流程。

自主模式不会降低任何运动标定要求，且 `auto_arm: true` 必须同时启用经过标定的运动配置。仓库默认配置始终保持调试模式和运动禁用。

完整任务模式还要求持续有效且已标定的夹爪遥测，否则不会通过自主启动就绪门。运行时在`GRAB`发送一次关闭命令、在`RELEASE`发送一次打开命令，并以STM32的`CLOSED/OPENED`状态作为Mission确认。暂停、终止、DISARM、FAULT和进程退出都会发送夹爪STOP；非ARM状态不会执行抓取或释放。

网页地址为 `http://192.168.137.201:8080/`，它是只读页面。`hls.js` 已本地打包。视频采用 FFmpeg 的 `h264_v4l2m2m` 编码，分辨率为 640×480，20 fps，码率 2 Mbit/s，HLS 分段长度为 0.5 s。若编码失败，状态会报告视频降级。软件编码 `libx264` 需要开启 `video.software_fallback_enabled: true`，或者显式修改 `video.encoder: libx264`。若构建时缺少 cpp-httplib，或网络地址不可用，HTTP 也会被降级处理。上述任一失败都不会中断任务控制。NDJSON 会记录带时间戳的事件、路径和周期性状态，并按大小或日期自动轮转。已完成的网格调试帧保存在 `logging.debug_dir`。

## 部署

在树莓派上检出代码后，执行 `runtime/deploy/install_pi.sh`。它会安装依赖、构建项目、运行测试、安装服务，并以 DISARMED 状态启动。现有的 `/etc/auv-runtime/runtime.yaml` 会被保留。检查状态可用：`systemctl status auv-runtime`、`journalctl -u auv-runtime` 和 `/var/log/auv-runtime/events.ndjson`。远程访问请使用 SSH 密钥；本仓库不包含任何密码处理逻辑。

从 PC 端执行：`runtime/deploy/deploy_from_pc.sh pi-user@192.168.137.201 /home/pi/auv`，它会通过 SSH 拷贝当前代码并执行 Pi 安装脚本。脚本禁用了 SSH 密码登录；远程 `sudo` 可能会通过终端提示。

请参考 [deploy/TESTING.md](deploy/TESTING.md) 中分阶段、只读的 Pi 检查流程。`run_bench.sh preflight` 会验证部署安全性，并将缺失的硬件标记为 pending；`camera`、`serial`、`fault-watch`、`endurance` 和 `collect` 提供聚焦检查和保存的 JSON 报告。耐久测试脚本会调用 `acceptance.py`，测量视觉/控制/心跳速率、滚动帧延迟、进程树 CPU 和 RSS、温度及节流状态，并要求具备有效的相机和 STM32 状态。缺失节流数据不计入通过。

在校准相机、行/列到机体坐标系的符号方向、速度限制和串口设备之前，不要设置 `motion_commands_enabled: true`。首次运动测试必须断开推进器电源，移除螺旋桨，或确保推进器牢固固定。2026-10-04 日，Debian 13 原生构建和两次 Pi CTest 已通过。30 分钟热性能运行、真实相机/HLS 测试、真实 STM32 试验台运行，以及无螺旋桨闭环验收仍待硬件完成。
