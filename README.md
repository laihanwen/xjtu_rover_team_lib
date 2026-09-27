# XJTU AUV · 水下具身智能机器人

> 面向水下机器人竞赛的自主 AUV：用 ROS 2 完成视觉、语义建图、路径规划与任务决策，用 STM32 完成实时姿态、深度、推进器和安全控制。

**当前阶段：基础设施 / P1**　·　ROS 2 工作区可构建　·　STM32F405 八推固件已归档　·　硬件测试需人工安全确认

## 快速导航

- [5 分钟开始开发](#5-分钟开始开发)
- [先理解系统](#先理解系统)
- [仓库地图](#仓库地图)
- [使用 AI 开发工具](#使用-ai-开发工具)
- [开发与提交](#开发与提交)
- [安全红线](#安全红线)
- [常见问题](#常见问题)

## 先理解系统

项目采用清晰的双层控制架构：

```text
┌──────────────── Raspberry Pi / PC ────────────────┐
│ ROS 2 · OpenCV · AprilTag · YOLO · Semantic Map  │
│ Path Planning · Mission FSM · Logging            │
└──────────────────────┬────────────────────────────┘
                       │ UART：运动目标 / 状态 / 心跳
┌──────────────────────▼────────────────────────────┐
│ STM32：IMU · Depth · PID · Thruster Mixer        │
│ ESC PWM · Actuator · Leak Detection · Failsafe   │
└───────────────────────────────────────────────────┘
```

Linux 端发送 `vx`、`vy`、`depth_target`、`yaw_target` 等目标；高速姿态和深度闭环必须留在 STM32。项目优先实现稳定、可比赛的结构化场景方案，不以复杂通用 SLAM 为第一目标。

### 当前进度

| 优先级 | 模块 | 状态 |
|---|---|---|
| P0 | ROS 2 workspace | ✅ 可构建、可测试 |
| P1 | `auv_interfaces` | ✅ 初始 msg/srv 已建立 |
| 固件基线 | STM32F405 八推全矢量控制 | ✅ 已导入、GCC 编译检查通过；安全接口待升级 |
| P2 | `auv_stm32_bridge` | ⏳ 下一阶段 |
| P3–P5 | 串口协议、failsafe、传感器 topics | 📝 已规划 |
| P6–P11 | 相机、视觉、建图、规划、Mission FSM | 📝 已规划 |
| P12–P14 | YOLO、抓取、转盘 | 📝 已规划 |

完整优先级和技术约束请阅读 [AGENTS.md](AGENTS.md)。

## 5 分钟开始开发

### 1. 环境要求

| 组件 | 项目基线 |
|---|---|
| OS | Ubuntu 26.04 `resolute` |
| ROS | ROS 2 `Lyrical`，安装于 `/opt/ros/lyrical` |
| Shell | fish 4.x |
| 构建 | colcon + CMake |
| ROS Python | 系统 `/usr/bin/python3` |
| 视觉训练 | uv 独立虚拟环境，不污染系统 Python |
| 固件检查（可选） | `arm-none-eabi-gcc` + Ninja |

快速检查：

```fish
test -f /opt/ros/lyrical/setup.fish; and echo "ROS 2 Lyrical: OK"
fish --version
colcon --help >/dev/null; and echo "colcon: OK"
python3 --version
```

### 2. 克隆并加载环境

```fish
git clone https://github.com/laihanwen/xjtu_rover_team_lib.git
cd xjtu_rover_team_lib
source tools/setup_dev.fish
```

环境脚本会：

- 加载 `/opt/ros/lyrical/setup.fish`
- 固定 colcon 使用系统 Python，避免误用 uv Python
- 自动加载已经构建的 workspace packages
- 将 ROS 日志写入仓库的 `log/ros/`

### 3. 构建与测试

```fish
colcon build

# 首次构建后重新加载，使 ROS 发现新生成的 packages
source tools/setup_dev.fish

colcon test
colcon test-result --verbose
```

预期能看到两个 packages：

```fish
colcon list
# auv_bringup
# auv_interfaces
```

### 4. 最小运行验证

```fish
ros2 interface show auv_interfaces/msg/Stm32Status
ros2 launch auv_bringup system.launch.py
```

启动入口当前只输出安全提示，不启动推进器或硬件节点。

### 5. STM32 固件编译检查（可选）

仓库中的固件是独立 CMake/Keil 工程，并通过 `COLCON_IGNORE` 与 ROS 构建隔离。安装 ARM GCC 后可以执行不生成烧录镜像的编译检查：

```fish
cmake -S firmware/stm32 -B firmware/stm32/build/gcc-check \
  -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build firmware/stm32/build/gcc-check --target rov_ui_model
```

部署和生成可烧录固件仍以 `firmware/stm32/MDK-ARM/Copy_cup.uvprojx` 为准。详细入口和已知警告见 [STM32 固件说明](firmware/stm32/README.md)。

## 仓库地图

```text
xjtu_rover_team_lib/
├── AGENTS.md              # 项目事实、技术决策和 AI 必读约束
├── src/                   # Raspberry Pi / PC 的 ROS 2 packages
│   ├── auv_interfaces/    # 公共 msg / srv
│   └── auv_bringup/       # 启动入口和共享安全配置
├── firmware/stm32/        # STM32 固件的独立构建边界
├── vision/                # 数据处理、训练、评估和模型导出
├── models/                # 模型清单与部署元数据
├── datasets/              # 本地数据集目录和管理约定
├── hardware/              # BOM、接线、坐标系和机构资料
├── docs/
│   ├── architecture/      # 仓库和系统架构
│   ├── protocol/          # Pi ↔ STM32 通信协议
│   └── testing/           # 无硬件、台架和水池测试策略
├── tools/                 # 开发、构建与部署辅助工具
└── logs|videos|maps...    # 运行输出，不提交 Git
```

进一步阅读：

- [仓库布局与模块边界](docs/architecture/repository-layout.md)
- [串口协议草案](docs/protocol/serial-protocol.md)
- [测试策略](docs/testing/strategy.md)
- [STM32 固件约束](firmware/stm32/README.md)
- [视觉研发约定](vision/README.md)
- [模型注册规则](models/README.md)
- [数据集管理](datasets/README.md)

## 使用 AI 开发工具

项目预计大量使用 Claude Code、Codex 等 AI 编程助手。无论使用哪一种工具，每个新会话都应先让它读取仓库事实，而不是凭经验猜测 ROS 版本或硬件参数。

### 推荐的首条指令

```text
请先完整阅读 AGENTS.md、README.md，以及本任务涉及目录中的 README。
检查当前仓库内容和 git status 后再行动，不要假设尚不存在的硬件参数或文件。
环境是 Ubuntu 26.04 + ROS 2 Lyrical + fish；ROS 使用系统 Python，视觉训练使用 uv 隔离环境。
实现后请执行与改动风险相称的构建和测试，并说明修改、运行方法和硬件风险。
```

### AI 协作检查清单

在接受 AI 生成的改动前，确认它没有：

- 使用 Humble/Jazzy 的包名或默认命令替代 Lyrical
- 在 fish 环境中默认执行 `setup.bash`
- 向系统 Python 执行 `pip install`
- 忽略现有 STM32F405 工程，或未经实测擅自修改 GPIO、Timer、PWM 范围和推进器方向
- 把实时 PID 放到 Linux / ROS 2 节点
- 修改 CubeMX 下次生成时会覆盖的区域
- 将模型权重、数据集、rosbag 或编译产物提交到 Git
- 绕过 DISARM、heartbeat、漏水检测或 failsafe

让 AI 修改代码时，建议一次只完成一个可验证目标，例如“定义串口帧解析器并添加黄金测试向量”，而不是笼统要求“完成 STM32 通信”。

## 开发与提交

### 分支建议

```text
feature/<module>-<short-name>  新功能
fix/<module>-<short-name>      缺陷修复
docs/<short-name>              文档
```

保持提交单一、可构建，例如：

```text
feat(bridge): add heartbeat frame decoder
test(protocol): add CRC golden vectors
docs(hardware): document thruster coordinate convention
```

### 提交前检查

ROS 代码变更至少运行：

```fish
source tools/setup_dev.fish
colcon build
source tools/setup_dev.fish
colcon test
colcon test-result --verbose
git diff --check
```

不同模块还需要对应验证：

- 视觉：固定输入视频或数据集上的离线回归结果
- 串口协议：ROS 与 STM32 共用的黄金帧测试
- STM32：host 单元测试、无桨台架测试和 failsafe 验证
- Mission：状态转换、超时和故障注入测试

## 模型与数据

模型权重和数据集不直接进入普通 Git：

- 权重放在本地 `models/artifacts/`
- 模型版本、类别、输入尺寸、SHA-256 和下载地址写入 `models/manifests/`
- 数据遵循 `datasets/raw → interim → processed → exports` 流程
- 稳定模型通过 GitHub Release、对象存储或团队约定位置分发

当前机器未统一安装 Git LFS/DVC，因此仓库暂不强制依赖它们；确定团队工作流后再引入。

## 安全红线

> [!CAUTION]
> 推进器、电调、电池和机械执行器可能造成人身伤害或设备损坏。任何首次测试必须断开电机电源、拆桨，或可靠固定推进器。

- 默认状态必须是 `DISARM`
- ARM 必须由显式命令触发
- STM32 必须独立执行 heartbeat timeout 和 failsafe
- Linux 程序退出或通信中断不能让推进器保持危险输出
- 漏水、传感器无效和通信超时必须进入安全状态
- 未经实测不得提交真实硬件参数的“猜测值”

### 当前固件的重要边界

现有 STM32 工程来自已完成的遥控 ROV 基线，不等同于完整的自主 AUV 安全固件。代码当前会在初始化阶段启动推进器 PWM，且尚未发现以下机制：

- 来自 Raspberry Pi 的显式 ARM / DISARM 状态机
- Pi 通信 heartbeat timeout
- 漏水传感器触发的 failsafe
- ROS 运动目标对应的版本化串口协议

在这些功能实现并通过无桨台架测试前，不得把 ROS 控制命令直接接入实机推进器。

## 常见问题

### CMake 选择了用户目录中的 Python

项目的 [colcon.defaults.yaml](colcon.defaults.yaml) 已固定：

```text
-DPython3_EXECUTABLE=/usr/bin/python3
```

请先 `source tools/setup_dev.fish`，不要通过修改系统 Python 或向 uv Python 安装 ROS 包来绕过问题。

### 构建后 ROS 找不到 package

重新加载环境：

```fish
source tools/setup_dev.fish
ros2 pkg prefix auv_interfaces
```

### 可以直接提交 `.pt` 或 `.onnx` 吗？

不可以。请把文件放入 `models/artifacts/`，计算 SHA-256，并为稳定模型创建 manifest。

### STM32 工程现在是什么状态？

仓库已包含 STM32F405RGT6 的 CubeMX、Keil、八推混控、姿态 PID、遥控和 IMU 代码。它是可编译的 ROV 固件基线，但还不是满足自主 AUV 安全要求的最终固件；下一步需要加入版本化串口协议、显式 ARM/DISARM、Pi heartbeat timeout、漏水检测和故障状态上报。

---

如果你第一次加入项目，建议按顺序阅读：**本 README → [AGENTS.md](AGENTS.md) → [仓库架构](docs/architecture/repository-layout.md) → 当前任务所属模块文档**。
