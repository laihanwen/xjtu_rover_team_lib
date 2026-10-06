# XJTU AUV

> 面向水下机器人竞赛的自主 AUV 软件与 STM32 固件。
> PC 的 ROS 2 与树莓派轻量 C++ 模式共用任务一核心；STM32 负责实时姿态、推力分配与安全保护。

![Ubuntu 26.04](https://img.shields.io/badge/Ubuntu-26.04_E2E2E2?logo=ubuntu&logoColor=white&labelColor=E95420)
![ROS 2 Lyrical](https://img.shields.io/badge/ROS_2-Lyrical-22314E?logo=ros)
![STM32F405](https://img.shields.io/badge/MCU-STM32F405-03234B?logo=stmicroelectronics)
![Stage P14](https://img.shields.io/badge/Stage-P14_Valve_Foundation-FF9800)

当前软件已完成 **P0–P11**：ROS 2 工作区、公共接口、STM32 bridge、串口协议、
heartbeat/failsafe、IMU/depth 遥测、双摄像头、AprilTag、九宫格建图、交通锥识别和
确定性格子路径规划、安全任务状态机，以及默认禁用动力的任务一路线执行器。
**P12 海参 YOLO** 的训练环境、数据工具和
ROS 推理节点已经建立，当前进入真实水下数据采集与标注阶段。**P13 单舵机抓取链路**
已完成 ROS 接口、串口协议和 STM32 安全状态机，等待机械端点标定与实机验收。
**P14 转盘视觉基础**已具备可配置 OpenCV 节点、ROS 接口和调试图，尚未进入执行器闭环。

> [!CAUTION]
> 当前代码通过软件测试不等于允许带桨运行。首次实机验证必须断开推进器动力、拆桨，
> 或可靠固定推进器。真实漏水、急停和传感器有效输入接入前，STM32 会拒绝 ARM。

## 项目入口

| 你要做什么 | 入口 |
|---|---|
| 查看当前状态、安全门和下一步 | [项目状态摘要](docs/project-status.md) |
| 20261006 ROV 合并、遥控映射、M10 与接线 | [ROV 合并版 README](tools/rov/README.md) |
| 第一次构建和启动 | [快速开始](#快速开始) |
| 运行相机、建图、规划和任务节点 | [运行与验收](#运行与验收) |
| 验收任务一闭环 | [AprilTag 与交通锥遍历](docs/testing/task1-apriltag-cones.md) |
| 查找架构、协议、网络和测试文档 | [文档中心](docs/README.md) |
| 理解目录职责和 ROS package 边界 | [仓库布局](docs/architecture/repository-layout.md) |
| 配置 P12 数据标注环境 | [P12 打标教程](docs/p12-annotation-guide.md) |
| 标定 P13 单舵机夹爪 | [P13 夹爪验收](docs/testing/p13-gripper.md) |
| 试用 P14 转盘视觉 | [P14 视觉验收](docs/testing/p14-valve-foundation.md) |
| 开发环境和协作约定 | [开发说明](docs/newcomer-ai-development-guide.md) |

## 系统架构

```text
                         ┌──────────────────────────────┐
                         │ PC 开发/操作端                │
                         │ ROS 2 · RViz/rqt · 记录/回放   │
                         └───────────────┬──────────────┘
                                         │ 千兆以太网 / ROS 2 DDS
┌────────────────────────────────────────▼────────────────────────────────────┐
│ Raspberry Pi / ROS 2 部署端                                                   │
│ auv_bringup → vision → mapping → planning → mission → control                 │
│                              │                                               │
│                              └──────── auv_stm32_bridge ───────┐             │
│                                                                  │ UART        │
│ Raspberry Pi / 轻量部署端                                        │ CRC16       │
│ auv_runtime + auv_core · 双摄像头 · Web 状态/视频 · 日志          │ heartbeat   │
└──────────────────────────────────────────────────────────────────▼───────────┘
                                                   ┌────────────────────────────┐
                                                   │ STM32F405 实时控制层        │
                                                   │ IMU/深度 · PID · Mixer/PWM  │
                                                   │ 推进器 · 夹爪 · ARM/DISARM   │
                                                   │ heartbeat · timeout · failsafe│
                                                   └────────────────────────────┘
```

系统有两种上层运行形态，共用协议、任务核心和安全边界：

| 运行形态 | 入口 | 适用场景 |
|---|---|---|
| ROS 2 模式 | `src/auv_*`、`colcon`、`ros2 launch auv_bringup system.launch.py` | PC 开发、RViz/rqt、模块化调试和完整 ROS topic |
| 轻量 Pi 模式 | `runtime/auv_runtime`、`runtime/config/runtime.yaml` | 树莓派现场部署、双摄像头采集、Web 监看和任务一闭环 |

两种模式共享 `src/auv_core` 中的任务一算法核心；ROS 2 模式通过 package 暴露标准
topic/service，轻量模式直接链接核心库并负责进程、串口、日志和 HTTP 运行时。两种模式
都只向 STM32 发送 `vx`、`vy`、`depth_target`、`yaw_target` 等目标，不能把高速姿态
控制、推力分配或最终 PWM 放回 Linux/ROS 2 调度层。

轻量模式支持 USB 下视相机与 CSI 前视相机同时采集，使用有界缓冲和最新帧覆盖避免
视频阻塞控制链路；网页提供只读状态和视频接口。配置、服务和实际部署结果见
[轻量运行时说明](runtime/README.md) 与 [树莓派部署记录](docs/deployment/pi-20261006/README.md)。

### 关键数据流

| 方向 | 接口 | 用途 |
|---|---|---|
| Camera → Pi/PC | `/camera/down/image_raw`、`/camera/front/image_raw` | 视频与视觉输入 |
| Vision → Pi/PC | `/apriltag/detections`、`/apriltag/debug_image` | 标签检测与调试画面 |
| Vision → Mapping | `/cones/detections`、`/cones/debug_image` | 稳定锥体分类与调试画面 |
| Front Vision → Mission | `/cucumber/detections` | 海参、海龟、海星检测 |
| Front Vision → Mission | `/valve/detection`、`/valve/debug_image` | 转盘候选、中心与把手方向 |
| Mapping → Pi/PC | `/semantic_map`、`/mapping/rectified_image` | 3×3 地图与标准俯视图 |
| Planning → Mission | `/planning/route` | 目标顺序与逐格最短路线 |
| Mission → Pi/PC | `/mission/state`、`/mission/command` | 状态、故障与人工控制 |
| PC/Pi → STM32 | `/cmd_vel`、`/cmd_depth`、`/cmd_yaw` | 完整运动目标 |
| STM32 → Pi/PC | `/imu/data`、`/depth`、`/stm32/status` | 遥测、安全状态和八路输出 |
| Operator → STM32 | `/stm32/set_armed` | 显式 ARM/DISARM |
| Operator/Mission → STM32 | `/gripper/set_state` | 夹爪 OPEN/CLOSE/STOP 动作请求 |
| STM32 → Pi/PC | `/gripper/status` | 夹爪标定、状态、目标与实际脉宽 |

Pi 与 STM32 的二进制协议、payload 和 CRC 定义见
[串口协议 v1](docs/protocol/serial-protocol.md)。

T35-L 单舵机的接线、端点标定和分阶段上电验收见
[P13 夹爪标定与验收](docs/testing/p13-gripper.md)。
转盘视觉基础的运行边界、摄像头验收和后续闭环门槛见
[P14 转盘视觉基础环境验收](docs/testing/p14-valve-foundation.md)。

### 软件模块边界

```text
auv_interfaces       公共 ROS msg/srv，定义跨节点接口
auv_stm32_bridge     UART、协议编解码、heartbeat、遥测与 ARM 服务
auv_vision           双摄像头、AprilTag、交通锥、海参、转盘视觉
auv_mapping          透视矫正、3×3 网格分割、语义地图
auv_planning         四邻域 A*、目标排列、确定性路线
auv_mission          任务 FSM、阶段超时、暂停/恢复/终止
auv_control          路线执行、运动目标门控，默认 dry-run
auv_bringup          launch 文件和跨 package 参数
auv_core             ROS 2 与轻量模式共享的任务一核心
runtime/             轻量进程、相机采集、Web、日志和部署服务
firmware/stm32/      实时控制、执行器、安全状态机和硬件工程
tools/rov/            ROV 遥控映射、试验控制、检查和部署辅助工具
```

依赖方向保持单向：视觉、建图和规划产生观测/路线，Mission 编排任务阶段，
Control 只产生受安全门控的运动目标，Bridge 负责跨进程传输，STM32 才执行闭环
控制和输出。算法模块不得直接访问 UART、GPIO、PWM 或执行器。

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
| P8 | 九宫格 Semantic Map | ✅ 几何建图已实现；待水下录像回归 |
| P9 | Cone detection | ✅ OpenCV 分类、时序稳定与地图融合；待水下调参 |
| P10 | Path planning | ✅ 四邻域 A*、目标排列枚举与确定性路线 |
| P11 | Mission FSM | ✅ 安全编排、超时、暂停/恢复/终止及虚拟全流程 |
| P11.1 | 任务一遍历执行 | 🚧 软件闭环完成；等待水下标定、方向校验和拆桨实机验收 |
| P12 | Sea cucumber YOLO | 🚧 训练环境与 ROS 推理已实现；等待真实标注数据/权重 |
| P13 | 单舵机抓取系统 | 🚧 ROS/串口/STM32 状态机已实现；等待端点标定和实机验收 |
| P14 | 转盘系统 | 🚧 OpenCV 检测接口、节点和调试环境已建立 |

### 当前控制能力边界

- `vx/vy` 已打通 PC → ROS 2 → UART → STM32 → Mixer，但当前是开环 PWM 前馈，
  不是 DVL 速度闭环。
- `yaw` 使用现有 IMU 航向 PID。
- `/cmd_depth` 已完成协议校验和超时联锁；真实深度传感器接入前，固件强制
  `Fz=0`，不会产生垂向推力。
- ARM 周期会锁定 Pi 或遥控器控制源。锁定源超过 250 ms 未更新时自动 DISARM，
  不会静默切换到另一控制源。
- T35-L 夹爪使用 PA8/TIM1_CH1；未完成开合端点标定前，固件保持 1500 μs 并拒绝动作。

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

当前工作区包含八个 ROS packages：

```text
auv_interfaces    auv_stm32_bridge    auv_vision
auv_mapping       auv_planning        auv_mission       auv_control
auv_bringup
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
  start_apriltag:=true \
  start_mapping:=true \
  start_cones:=true \
  start_planning:=true \
  start_mission:=true \
  start_route_executor:=true \
  start_cucumber:=true \
  start_valve:=true
```

启动前需填写实际硬件路径：

- 摄像头：`src/auv_bringup/config/cameras.yaml`
- STM32 串口：`src/auv_bringup/config/stm32_bridge.yaml`
- AprilTag 标定：`src/auv_bringup/config/apriltag.yaml`
- 九宫格参数：`src/auv_bringup/config/mapping.yaml`
- 交通锥阈值：`src/auv_bringup/config/cones.yaml`
- 路径规划起点与障碍类型：`src/auv_bringup/config/planning.yaml`
- Mission 状态超时：`src/auv_bringup/config/mission.yaml`
- 海参模型与推理参数：`src/auv_bringup/config/cucumber.yaml`
- 转盘视觉参数：`src/auv_bringup/config/valve.yaml`
- 下视相机内参与畸变：`src/auv_bringup/config/down_camera_calibration.yaml`

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

### 3. 九宫格语义建图

生产模式必须先在 `down_camera_calibration.yaml` 填入真实水下标定结果。启动后验证：

```fish
ros2 topic echo /semantic_map
ros2 topic hz /mapping/rectified_image
rqt_image_view /mapping/debug_image
```

网格连续稳定 3 帧后，`complete` 才会变为 `true`，并发布 600×600 标准俯视图。
P9 将稳定检测融合为 `circle_cone` 或 `square_cone`，没有新鲜检测的单元保持
`unknown`。图像行方向对应 row 递增，列方向对应 col 递增。算法参数和离线运行方法见
[auv_mapping 使用说明](src/auv_mapping/README.md)。

### 4. 路径规划

先在 `planning.yaml` 配置已确认的起始格；默认 `-1/-1` 会安全地输出无效路线，
不会猜测场地入口。启动后验证：

```fish
ros2 topic echo /planning/route
```

有效路线包含目标访问顺序、从起点开始的每个相邻格和总步数。规划器只计算路线，
不会发布运动指令或 ARM。任务一执行器默认 dry-run，可查看
`/planning/execution_state`；完成标定、拆桨和方向验收后，才可显式增加
`enable_route_motion:=true`，执行器也不会自动 ARM。详细说明见
[auv_planning 使用说明](src/auv_planning/README.md)。

### 5. Mission FSM

Mission 默认不会自动启动，也不会 ARM。确认 `/stm32/status` 安全后手动启动：

```fish
ros2 service call /mission/command auv_interfaces/srv/MissionCommand \
  "{command: 1}"
ros2 topic echo /mission/state
```

虚拟全流程验收：

```fish
ros2 run auv_mission mission_virtual_test
```

结果必须以 `VIRTUAL_MISSION_RESULT=PASS` 结束。详见
[auv_mission 使用说明](src/auv_mission/README.md)。

### 6. P12 数据标注

P12 使用独立的 Label Studio 环境，不修改 ROS 系统 Python，也不与
`vision/.venv` 混用。首次安装并启动：

```fish
cd /home/hanwen/auv/annotation
set -gx UV_CACHE_DIR /home/hanwen/auv/.cache/uv
uv sync --frozen
./start.sh
```

浏览器打开 <http://127.0.0.1:8080>，创建本地项目后，将
`annotation/labeling-config.xml` 粘贴到 **Labeling Setup → Code**。固定类别为：

```text
0 sea_cucumber
1 turtle
2 starfish
```

完成标注后从 Label Studio 导出 YOLO 数据，先检查 `classes.txt` 的类别和顺序，
再执行数据切分与校验。完整的图片准备、画框标准、负样本规则、导出整理和验收命令见：

> [P12 水下目标打标教程](docs/p12-annotation-guide.md)

### 7. PC—树莓派有线联调

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

### 8. STM32 状态

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

### 9. 固件检查

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
├── AGENTS.md                 # 项目事实、硬件边界与开发约束
├── src/
│   ├── auv_interfaces/       # 公共 msg / srv
│   ├── auv_stm32_bridge/     # ROS 2 ↔ STM32 安全串口桥
│   ├── auv_vision/           # 相机、AprilTag、锥体、海参与转盘视觉
│   ├── auv_mapping/          # 九宫格检测、Homography 与语义地图
│   ├── auv_planning/         # A*、目标排序与格子路线
│   ├── auv_mission/          # 安全任务状态机与阶段超时
│   ├── auv_control/          # 路线执行和运动目标安全门控
│   ├── auv_core/             # ROS 2 / 轻量模式共享任务核心
│   └── auv_bringup/          # launch 与共享参数
├── runtime/                  # 树莓派轻量 C++ 运行时、Web 和部署服务
├── firmware/stm32/           # STM32F405 CubeMX / Keil 工程与主机测试
├── tools/rov/                # ROV 试验、遥控器映射和现场检查工具
├── annotation/               # Label Studio 独立打标环境与类别配置
├── vision/                   # 数据处理、训练、评估和导出
├── models/                   # 模型 manifest 与部署元数据
├── datasets/                 # 本地数据集管理约定
├── hardware/                 # BOM、接线、坐标系与机构资料
├── docs/
│   ├── architecture/         # 仓库和系统架构
│   ├── protocol/             # Pi ↔ STM32 协议
│   ├── reviews/              # 代码审查与可行性边界
│   └── testing/              # 分阶段验收手册
├── tools/                    # 通用开发、构建和部署辅助脚本
└── logs|videos|maps|events/  # 被 Git 忽略的运行产物目录
```

建议阅读顺序：

1. [文档中心](docs/README.md)
2. [项目状态摘要](docs/project-status.md)
3. 当前任务对应的 package README 和验收手册
4. [项目约束与路线图](AGENTS.md)

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
- 原始采集文件保持只读；Label Studio 数据库位于 `annotation/data/`，不提交 Git。
- 视频相邻帧不得随机分散到 train/val/test，应按采集批次隔离以避免数据泄漏。
- rosbag、视频、数据集、权重和编译产物均不得混入源码提交。

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

确认 `git status` 正常后再执行提交或推送；不要在仓库外部直接运行 `git push`。

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

首次接触项目建议依次阅读：**README → [文档中心](docs/README.md) →
[仓库架构](docs/architecture/repository-layout.md) → 当前任务所属模块文档**。

## Lightweight Raspberry Pi mode

The standalone C++ runtime and safe deployment procedure are documented in [runtime/README.md](runtime/README.md). ROS 2 Lyrical PC development remains available through the existing colcon packages; both modes link the same mission-one core sources in `auv_core`.


2026-10-06 本地候选更新：水平改为岸上手动校准（每次STM32启动需重做），恢复IMU/心跳时间戳一致快照；推进器矩阵暂不修改。双摄原始MJPEG缓存、长连接录制与独立预览目标30FPS，真实Pi帧率待部署测量。使用方式见[ROV工具说明](tools/rov/README.md)，验证范围见[本次检查记录](docs/reviews/manual-level-video-20261006.md)。
