# XJTU AUV

> 面向水下机器人竞赛的自主 AUV 软件与 STM32 固件。
> ROS 2 负责视觉、语义地图、规划和任务决策；STM32 负责实时姿态、推力分配与安全保护。

![Ubuntu 26.04](https://img.shields.io/badge/Ubuntu-26.04_E2E2E2?logo=ubuntu&logoColor=white&labelColor=E95420)
![ROS 2 Lyrical](https://img.shields.io/badge/ROS_2-Lyrical-22314E?logo=ros)
![STM32F405](https://img.shields.io/badge/MCU-STM32F405-03234B?logo=stmicroelectronics)
![Stage P7](https://img.shields.io/badge/Stage-P7_AprilTag-2E8B57)

当前软件已完成 **P0–P7**：ROS 2 工作区、公共接口、STM32 bridge、串口协议、
heartbeat/failsafe、IMU/depth 遥测、双摄像头和 AprilTag。PC 可通过有线 ROS 2 网络
接收视频与状态并发送运动目标；下一阶段是 **P8 九宫格语义建图**。

> [!CAUTION]
> 当前代码通过软件测试不等于允许带桨运行。首次实机验证必须断开推进器动力、拆桨，
> 或可靠固定推进器。真实漏水、急停和传感器有效输入接入前，STM32 会拒绝 ARM。

## 导航

- [系统架构](#系统架构)
- [当前进度](#当前进度)
- [快速开始](#快速开始)
- [运行与验收](#运行与验收)
- [仓库结构](#仓库结构)
- [安全边界](#安全边界)
- [开发约定](#开发约定)
- [常见问题](#常见问题)

## 系统架构

```text
┌──────────────────── PC ────────────────────┐
│ rqt / RViz · 视频监看 · 运动目标 · 调试   │
└───────────────────┬────────────────────────┘
                    │ Gigabit Ethernet / ROS 2 DDS
┌───────────────────▼────────────────────────┐
│ Raspberry Pi                              │
│ Camera · AprilTag · Mapping · Planning    │
│ Mission FSM · Logging · STM32 Bridge      │
└───────────────────┬────────────────────────┘
                    │ UART / CRC16 / heartbeat
┌───────────────────▼────────────────────────┐
│ STM32F405                                 │
│ IMU · Depth · PID · Thruster Mixer · PWM  │
│ ARM/DISARM · timeout · leak/kill failsafe │
└────────────────────────────────────────────┘
```

Linux 端只发送 `vx`、`vy`、`depth_target`、`yaw_target` 等目标。高速姿态控制、
推力分配和最终 PWM 必须留在 STM32，不由 ROS 2 调度承担。

### 关键数据流

| 方向 | 接口 | 用途 |
|---|---|---|
| Camera → Pi/PC | `/camera/down/image_raw`、`/camera/front/image_raw` | 视频与视觉输入 |
| Vision → Pi/PC | `/apriltag/detections`、`/apriltag/debug_image` | 标签检测与调试画面 |
| PC/Pi → STM32 | `/cmd_vel`、`/cmd_depth`、`/cmd_yaw` | 完整运动目标 |
| STM32 → Pi/PC | `/imu/data`、`/depth`、`/stm32/status` | 遥测、安全状态和八路输出 |
| Operator → STM32 | `/stm32/set_armed` | 显式 ARM/DISARM |

Pi 与 STM32 的二进制协议、payload 和 CRC 定义见
[串口协议 v1](docs/protocol/serial-protocol.md)。

## 当前进度

| 阶段 | 能力 | 状态 |
|---:|---|---|
| P0 | ROS 2 workspace | ✅ 可构建、可测试 |
| P1 | `auv_interfaces` | ✅ msg/srv 已建立 |
| P2 | `auv_stm32_bridge` | ✅ 串口传输、状态发布与 ARM 服务 |
| P3 | Pi ↔ STM32 protocol | ✅ v1 帧、CRC、解析器与黄金向量 |
| P4 | heartbeat + failsafe | ✅ 软件实现；实机安全输入待接线 |
| P5 | IMU / depth topics | ✅ 遥测链路；真实深度驱动待定型 |
| P6 | 双摄像头 | ✅ down/front、重连与离线输入 |
| P7 | AprilTag | ✅ 二维检测与标定位姿；待水下实测 |
| P8 | 九宫格 Semantic Map | ⏭️ 下一阶段 |
| P9–P11 | Cone、规划、Mission FSM | 📝 已规划 |
| P12–P14 | YOLO、抓取、转盘 | 📝 已规划 |

### 当前控制能力边界

- `vx/vy` 已打通 PC → ROS 2 → UART → STM32 → Mixer，但当前是开环 PWM 前馈，
  不是 DVL 速度闭环。
- `yaw` 使用现有 IMU 航向 PID。
- `/cmd_depth` 已完成协议校验和超时联锁；真实深度传感器接入前，固件强制
  `Fz=0`，不会产生垂向推力。
- ARM 周期会锁定 Pi 或遥控器控制源。锁定源超过 250 ms 未更新时自动 DISARM，
  不会静默切换到另一控制源。

## 快速开始

### 环境

| 组件 | 项目基线 |
|---|---|
| OS | Ubuntu 26.04 `resolute` |
| ROS | ROS 2 `Lyrical`，位于 `/opt/ros/lyrical` |
| Shell | fish 4.x |
| ROS Python | 系统 `/usr/bin/python3` |
| 视觉训练 | 独立 uv 环境 |
| MCU | STM32F405RGT6 |

不要向系统 Python 执行 `pip install`；YOLO 训练环境与 ROS Python 必须隔离。

### 获取和构建

```fish
git clone https://github.com/laihanwen/xjtu_rover_team_lib.git
cd xjtu_rover_team_lib

source /opt/ros/lyrical/setup.fish
colcon build --symlink-install
source install/setup.fish
```

也可以使用仓库环境脚本：

```fish
source tools/setup_dev.fish
colcon build --symlink-install
source tools/setup_dev.fish
```

该脚本会固定使用系统 Python，并将 ROS 日志写入 `log/ros/`。

### 测试

```fish
colcon test
colcon test-result --verbose
git diff --check
```

当前工作区包含四个 ROS packages：

```text
auv_interfaces    auv_stm32_bridge    auv_vision    auv_bringup
```

## 运行与验收

### 1. 安全启动

```fish
source /opt/ros/lyrical/setup.fish
source install/setup.fish
ros2 launch auv_bringup system.launch.py
```

所有硬件节点默认关闭，推进系统保持 DISARM。按需显式启用：

```fish
ros2 launch auv_bringup system.launch.py \
  start_stm32_bridge:=true \
  start_cameras:=true \
  start_apriltag:=true
```

启动前需填写实际硬件路径：

- 摄像头：`src/auv_bringup/config/cameras.yaml`
- STM32 串口：`src/auv_bringup/config/stm32_bridge.yaml`
- AprilTag 标定：`src/auv_bringup/config/apriltag.yaml`

这些字段默认留空，避免误连 `/dev/videoN` 或未知串口。

### 2. 视频与 AprilTag

```fish
ros2 topic hz /camera/down/image_raw
ros2 topic hz /camera/front/image_raw
ros2 topic echo /apriltag/detections
rqt_image_view /camera/down/image_raw
```

相机节点也支持视频文件和 OpenCV 图像序列，可在无硬件时离线复现。详细配置见
[auv_vision 使用说明](src/auv_vision/README.md)。

### 3. PC—树莓派有线联调

两端使用同一 ROS 2 Lyrical、工作区接口和 `ROS_DOMAIN_ID`。完整静态 IP、DDS 发现、
视频查看、拆桨控制和拔网线验收步骤见：

> [PC 与树莓派有线联调手册](docs/wired-network.md)

最小网络环境示例：

```fish
source /opt/ros/lyrical/setup.fish
source install/setup.fish
set -gx ROS_DOMAIN_ID 42
set -gx ROS_LOCALHOST_ONLY 0
```

### 4. STM32 状态

```fish
ros2 topic echo /stm32/status
ros2 topic echo /imu/data --once
ros2 topic echo /depth --once
```

预期安全状态：

- 未配置串口：`connected: false`、`armed: false`
- 未接深度传感器：`valid: false`、`depth: nan`
- DISARM：`thruster_outputs` 八路均为零
- 命令或 heartbeat 超时：自动撤销 ARM

H30 当前只提供欧拉角。`/imu/data` 的角速度和线加速度为 NaN，对应 covariance
首项为 `-1`，表示数据不可用，不应被下游当作零值使用。

### 5. 固件检查

固件与 ROS 构建隔离。ARM GCC 可进行不链接、不烧写的对象编译检查：

```fish
cmake -S firmware/stm32 -B firmware/stm32/build/gcc-check \
  -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build firmware/stm32/build/gcc-check --target rov_ui_model
```

主机单元测试：

```fish
cmake -S firmware/stm32/test -B firmware/stm32/build/host-tests
cmake --build firmware/stm32/build/host-tests
ctest --test-dir firmware/stm32/build/host-tests --output-on-failure
```

可烧录固件仍以 `firmware/stm32/MDK-ARM/Copy_cup.uvprojx` 的 Keil 构建结果为准。

## 仓库结构

```text
xjtu_rover_team_lib/
├── AGENTS.md                 # 项目事实、硬件边界与 AI 必读约束
├── src/
│   ├── auv_interfaces/       # 公共 msg / srv
│   ├── auv_stm32_bridge/     # ROS 2 ↔ STM32 安全串口桥
│   ├── auv_vision/           # 相机与 AprilTag
│   └── auv_bringup/          # launch 与共享参数
├── firmware/stm32/           # STM32F405 CubeMX / Keil 工程与测试
├── vision/                   # 数据处理、训练、评估和导出
├── models/                   # 模型 manifest 与部署元数据
├── datasets/                 # 本地数据集管理约定
├── hardware/                 # BOM、接线、坐标系与机构资料
├── docs/                     # 架构、协议、联调与测试文档
└── tools/                    # 开发、构建和部署辅助脚本
```

推荐阅读顺序：

1. [项目约束与路线图](AGENTS.md)
2. [仓库布局与模块边界](docs/architecture/repository-layout.md)
3. [串口协议 v1](docs/protocol/serial-protocol.md)
4. [测试策略](docs/testing/strategy.md)
5. [P4 安全验收](docs/testing/p4-safety.md)
6. [STM32 固件说明](firmware/stm32/README.md)

## 安全边界

> [!IMPORTANT]
> DISARM、heartbeat timeout、通信超时、输出限幅和显式 ARM 已在软件中实现并有测试，
> 但真实漏水、kill 和传感器有效信号仍需按最终硬件接线。不得用常量绕过这些安全输入。

上机必须遵守：

- 上电默认 DISARM；任何恢复连接都不得自动 ARM。
- 首次测试断开电机动力、拆桨或可靠固定推进器。
- ARM 前确认漏水、急停、传感器有效和 heartbeat 状态。
- 先验证零目标，再逐台、单轴、低输出验证方向。
- 拔网线、停止任一命令 topic 或停止 heartbeat 后，必须在规定时间内归零并 DISARM。
- 不得猜测 GPIO、Timer、PWM 范围、推进器极性或安装方向。
- 不得把 Linux/ROS 2 作为高速姿态 PID 的执行层。

## 开发约定

### 分支与提交

```text
feature/<module>-<name>    新功能
fix/<module>-<name>        缺陷修复
docs/<name>                文档
```

推荐使用小而可验证的提交：

```text
feat(bridge): add motion target timeout gate
test(firmware): cover control source locking
docs(network): add wired bench checklist
```

提交前至少执行：

```fish
source tools/setup_dev.fish
colcon build --symlink-install
source tools/setup_dev.fish
colcon test
colcon test-result --verbose
git diff --check
```

视觉改动还应提供固定视频回归；协议改动应同步 ROS/STM32 黄金向量；固件改动必须经过
host 测试、交叉编译、无桨台架和 failsafe 故障注入。

### 模型与数据

- 模型权重放入本地 `models/artifacts/`，不直接提交普通 Git。
- 模型版本、类别、输入尺寸、SHA-256 和下载地址记录在 `models/manifests/`。
- 数据遵循 `datasets/raw → interim → processed → exports`。
- rosbag、视频、数据集、权重和编译产物均不得混入源码提交。

### 使用 AI 编程助手

新会话先要求工具读取 `AGENTS.md`、本 README 和任务目录内的说明。接受改动前确认：

- 没有把 Humble/Jazzy 的命令当作 Lyrical 默认命令；
- 没有在 fish 指令中默认使用 `setup.bash`；
- 没有污染系统 Python 或 CUDA/YOLO 环境；
- 没有猜测硬件参数或修改会被 CubeMX 覆盖的区域；
- 没有绕过 DISARM、heartbeat、漏水、kill 或传感器有效检查。

## 常见问题

### `ros2 launch` 提示找不到文件

正确文件名是 `system.launch.py`，修改源码后需要重新构建并加载：

```fish
colcon build --symlink-install --packages-select auv_bringup
source install/setup.fish
ros2 launch auv_bringup system.launch.py
```

### Git 提示“不是 Git 仓库”

先进入仓库目录：

```fish
cd ~/auv
git status
```

本开发副本可能使用 `.git-data` 保存元数据；这种情况下普通 Git 命令需由项目工具或
`git --git-dir=.git-data --work-tree=.` 调用。不要在其他目录直接执行 `git push`。

### 构建后找不到 ROS package

```fish
source /opt/ros/lyrical/setup.fish
source install/setup.fish
ros2 pkg prefix auv_interfaces
```

### 为什么 ARM 被拒绝？

ARM 需要有效串口协议、近期 heartbeat、无漏水、kill 未触发且传感器有效。当前实机
安全输入尚未全部接线时，拒绝 ARM 是预期行为，不应通过删除检查来解决。

### PC 能否同时接收视频并控制推进器？

可以。千兆以太网承载 ROS 2 DDS，树莓派再通过 UART 与 STM32 通信。控制命令不会
隐式 ARM，网络中断会使目标超时并由 STM32 撤销 ARM。请按
[有线联调手册](docs/wired-network.md)逐项验收。

---

第一次加入项目建议依次阅读：**README → [AGENTS.md](AGENTS.md) →
[仓库架构](docs/architecture/repository-layout.md) → 当前任务所属模块文档**。
