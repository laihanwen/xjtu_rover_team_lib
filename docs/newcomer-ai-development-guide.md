# AUV 新人 AI 辅助开发教程

这份教程写给此前没有接触过软件、ROS 2、计算机视觉或嵌入式开发的同学。目标不是让你一次学完所有知识，而是让你能在**不破坏现有工程、不绕过安全保护**的前提下，借助 AI 完成一个个可验证的小任务。

本教程以本仓库的真实状态为准：P0–P7 已完成，当前下一阶段是 **P8 九宫格语义建图**。不要重新创建已经存在的 package，也不要让 AI 按其他 ROS 版本重写项目。

> [!CAUTION]
> 软件能编译或测试通过，不代表推进器可以安全通电。所有第一次硬件测试都必须断开电机动力、拆桨或可靠固定推进器。未经人工确认，不得 ARM，不得烧录固件，不得给执行机构发送动作。

## 1. 先理解你正在开发什么

AUV 是 Autonomous Underwater Vehicle，即自主水下机器人。本项目分为两层：

```text
摄像头/任务目标
      │
      ▼
Raspberry Pi / PC（ROS 2，高层、非实时）
视觉 → 地图 → 路径 → Mission → 运动目标
      │ UART：vx、vy、depth、yaw、heartbeat
      ▼
STM32F405（底层、实时）
传感器 → PID → 推力分配 → ESC/PWM
      │
      ▼
推进器和机械执行机构
```

必须记住以下边界：

- PC/树莓派负责“看见什么、下一步去哪”。
- STM32 负责“怎样稳定地控制电机到达目标”。
- Linux/ROS 2 不执行高速姿态 PID。
- ROS 命令不能隐式 ARM；STM32 丢失 heartbeat 后必须自动 DISARM。
- GPIO、定时器、PWM 范围、推进器方向等未知参数必须实测，AI 不能猜。

几个常见词：

| 词语 | 新人可以怎样理解 |
|---|---|
| ROS 2 | 让多个机器人程序交换消息、参数和服务的框架 |
| node（节点） | 一个职责单一、可独立运行的程序 |
| topic（话题） | 连续数据通道，例如相机图像或深度 |
| msg（消息） | topic 上数据的固定格式 |
| service（服务） | 一问一答的操作，例如请求 ARM |
| launch | 一次启动多个节点并加载参数 |
| package（包） | 一组相关节点、接口、配置和测试 |
| TF | 描述相机、艇体等坐标系之间的关系 |
| firmware（固件） | 在 STM32 上运行的软件 |
| DISARM | 禁止推进器输出的安全状态 |
| rosbag2 | 录制和回放 ROS topic 数据的工具 |

## 2. 仓库地图：先找对地方再修改

在仓库根目录 `~/auv` 中：

| 路径 | 用途 | 新人是否常改 |
|---|---|---|
| `AGENTS.md` | 项目事实、版本和安全约束 | 通常只读 |
| `src/auv_interfaces/` | 公共 msg/srv | 接口变化时改 |
| `src/auv_stm32_bridge/` | ROS 2 与 STM32 串口桥 | 通信任务时改 |
| `src/auv_vision/` | 相机、AprilTag 和交通锥视觉 | 视觉任务常改 |
| `src/auv_mapping/` | 九宫格矫正与语义地图 | 建图任务时改 |
| `src/auv_planning/` | A* 与目标访问顺序 | 规划任务时改 |
| `src/auv_mission/` | 安全任务状态机 | 任务流程变化时改 |
| `src/auv_bringup/` | launch 和共享 YAML 参数 | 新节点完成后改 |
| `firmware/stm32/` | STM32F405 CubeMX/Keil 工程 | 仅固件任务改 |
| `vision/` | YOLO 训练、数据处理、实验 | P12 阶段常改 |
| `datasets/` | 数据集约定；大文件不进 Git | 数据任务使用 |
| `models/` | 模型版本清单和部署元数据 | 模型发布时改 |
| `docs/` | 架构、协议、测试和教程 | 文档任务改 |
| `tools/setup_dev.fish` | 加载项目开发环境 | 使用，不随意改 |
| `build/ install/ log/` | 自动生成的构建和日志文件 | 不手工编辑、不提交 |

当前已有七个 ROS package：

- `auv_interfaces`
- `auv_stm32_bridge`
- `auv_vision`
- `auv_mapping`
- `auv_planning`
- `auv_mission`
- `auv_bringup`

后续 package 仍应在真正开始实现且能独立测试时再创建。

## 3. 第一次打开终端

项目日常终端是 **fish**。以下代码块默认都是 fish 命令。

### 3.1 确认位置和环境

```fish
cd ~/auv
pwd
source tools/setup_dev.fish
echo $ROS_DISTRO
echo $AUV_WS
```

预期：

- `pwd` 指向仓库根目录；
- `ROS_DISTRO` 是 `lyrical`；
- `AUV_WS` 指向当前仓库。

`tools/setup_dev.fish` 会加载 `/opt/ros/lyrical/setup.fish`，使用系统 ROS Python，并把 ROS 日志放入仓库的 `log/ros/`。不要把教程中的命令擅自改成 `setup.bash`，也不要向系统 Python 执行 `pip install`。

### 3.2 第一次只读检查

```fish
git status --short
colcon list
ros2 pkg list | string match 'auv_*'
```

先看 `git status` 是为了知道哪些修改原本就存在。它们可能是队友的工作，不要让 AI 删除、覆盖或“顺手整理”。

如果普通 `git` 报告不是仓库，先执行 `source tools/setup_dev.fish`；本开发副本可能把 Git 元数据放在 `.git-data`，脚本会正确配置它。

## 4. 建立一个安全的工作基线

新人第一次参与项目时，先证明“未修改的工程本来能工作”。

```fish
cd ~/auv
source tools/setup_dev.fish
colcon build --symlink-install
source tools/setup_dev.fish
colcon test
colcon test-result --verbose
git diff --check
```

命令含义：

1. `colcon build` 编译 `src/` 中的 ROS packages。
2. 再次 `source` 让当前终端找到刚生成的程序和接口。
3. `colcon test` 运行单元测试和 lint。
4. `colcon test-result --verbose` 汇总具体结果。
5. `git diff --check` 检查空白错误。

如果失败，不要立刻让 AI 大改源码。把**完整命令、第一处真正错误、环境信息**给 AI，请它先诊断。后面的连锁报错通常只是第一处错误造成的。

可使用这个提示词：

```text
请先阅读 AGENTS.md、README.md 和与报错 package 对应的 README。
我在 ~/auv 使用 Ubuntu 26.04、ROS 2 Lyrical、fish。
我运行了：<原样粘贴命令>
第一处错误是：<原样粘贴错误>
请先只诊断根因，不要修改文件；说明你依据了哪些仓库内容，以及最小验证办法。
```

## 5. 怎样与 AI 合作，而不是盲目复制代码

AI 最适合做代码搜索、解释、生成小范围实现、补测试和整理文档。AI 不知道尚未记录的真实接线、机械安装、推进器极性和水下环境，因此这些事实必须由人提供。

### 5.1 每个新会话的开场模板

```text
你正在 ~/auv 仓库工作。先完整阅读 AGENTS.md、README.md，以及本任务目录内的 README。
环境是 Ubuntu 26.04 resolute、ROS 2 Lyrical、fish；ROS 使用系统 Python，YOLO 使用独立 uv 环境。

任务：<只写一个清楚的小目标>
验收标准：<列出可观察的结果和测试>
允许修改：<目录或文件>
禁止事项：不要猜硬件参数，不要 ARM/烧录/启动推进器，不要 pip install 到系统 Python，不要覆盖现有未提交修改。

请先检查当前代码和 git diff，再实施；完成后告诉我修改内容、原因、构建、运行、验证和剩余风险。
```

一个好任务应当像这样：

```text
为九宫格透视矫正实现一个无 ROS 依赖的 C++ 函数：输入四个有序角点和输出尺寸，返回 homography。
要求补充正常矩形、倾斜四边形和退化角点测试。只修改 auv_mapping，先不要接摄像头和 launch。
```

不要只说“帮我把 AUV 做完”。目标太大时，AI 会被迫猜接口、硬件和验收标准。

### 5.2 要求 AI 遵循的闭环

```text
理解任务 → 搜索现有实现 → 说明假设 → 小范围修改
    → 构建 → 自动测试 → 查看 diff → 给出人工验证步骤
```

每次至少向 AI 询问：

- 改了哪些文件？为什么是这些文件？
- 哪些是事实，哪些是假设或 TODO？
- 运行了什么测试？测试覆盖了什么、没覆盖什么？
- 会不会影响安全状态、协议兼容或 CubeMX 生成区？
- 我应当怎样用 topic、固定图片或录制视频观察结果？

### 5.3 三类操作的授权边界

AI 可以直接做的低风险操作：

- 阅读源码、文档、配置和 Git diff；
- 搜索符号与调用关系；
- 编译、运行无硬件单元测试；
- 在你指定的任务范围内修改源码和测试；
- 使用固定图片、视频、PTY 串口模拟器做离线验证。

应先向你确认的操作：

- 安装或升级依赖；
- 改公共 msg/srv 或串口协议；
- 改推进器 mixer、PID、PWM、GPIO、Timer；
- 烧录 STM32、访问真实串口或相机；
- 删除文件、重写历史、覆盖已有改动；
- 下载大型模型或数据集。

禁止自动执行的操作：

- ARM 推进系统；
- 带桨或未固定推进器测试；
- 用常量绕过 leak、kill、heartbeat、传感器有效性检查；
- 因“先跑起来”而删除 DISARM 或超时逻辑；
- 猜测硬件接线和方向后直接写入固件。

## 6. 学会阅读 ROS 2 数据流

不要从几千行源码开始。先看“谁发布什么、谁订阅什么”。当前关键链路是：

```text
down camera ──/camera/down/image_raw──> AprilTag detector
                                            │
                                            └──/apriltag/detections

/cmd_vel ─┐
/cmd_depth├──> stm32_bridge ──UART──> STM32
/cmd_yaw ─┘                         └──> /stm32/status /imu/data /depth
```

常用只读检查：

```fish
ros2 node list
ros2 topic list -t
ros2 topic info /camera/down/image_raw --verbose
ros2 interface show auv_interfaces/msg/SemanticMap
ros2 interface show auv_interfaces/srv/SetArmed
ros2 param list /auv_apriltag_detector
ros2 node info /auv_apriltag_detector
```

看到一个 topic 时依次问：

1. 谁发布？
2. 谁订阅？
3. 消息类型是什么？
4. 单位和坐标系是什么？
5. 没有数据、数据无效、超时时怎样表达？

例如 `/depth` 不能把“无传感器”伪装成深度 0；它通过 `valid: false` 和 `NaN` 表示不可用。安全系统必须区分“真实零值”和“没有有效数据”。

## 7. 启动现有系统：从无硬件模式开始

先构建并加载环境：

```fish
cd ~/auv
source tools/setup_dev.fish
colcon build --symlink-install
source tools/setup_dev.fish
```

默认 launch 不启动硬件节点：

```fish
ros2 launch auv_bringup system.launch.py
```

它应该输出工作区已准备好，且推进系统保持 DISARM。硬件节点都需要显式参数开启。

### 7.1 离线相机与 AprilTag

优先使用录制视频或图像序列，而不是直接依赖真实相机：

```fish
ros2 run auv_vision camera_node --ros-args \
  -r __node:=auv_camera_down \
  -p source:=/绝对路径/测试视频.mp4 \
  -p topic:=/camera/down/image_raw \
  -p frame_id:=camera_down_optical_frame \
  -p loop:=true
```

另开一个 fish 终端：

```fish
cd ~/auv
source tools/setup_dev.fish
ros2 topic hz /camera/down/image_raw
ros2 topic echo /camera/down/image_raw --field header --once
```

AprilTag 的具体参数、标定前后输出差异见 `src/auv_vision/README.md`。没有相机标定时 `pose_valid: false` 是正确结果，不应伪造位姿。

### 7.2 STM32 bridge 的安全观察

不配置串口时：

```fish
ros2 run auv_stm32_bridge stm32_bridge_node
```

另一个终端检查：

```fish
source tools/setup_dev.fish
ros2 topic echo /stm32/status --once
```

预期 `connected: false`、`armed: false`。这证明节点能够以安全默认值运行，不证明真实硬件已经联通。

> [!WARNING]
> 本教程故意不把 ARM 命令作为新人练习。只有完成 `docs/testing/p4-safety.md` 的无桨验收、确认真实安全输入和现场负责人许可后，才能进行 ARM 测试。

## 8. 一次标准开发任务怎样完成

以下流程适用于绝大多数任务。

### 第 1 步：写清验收标准

“实现网格检测”过于含糊。更好的标准是：

- 输入固定测试图片；
- 找到黄色外边界的四个角；
- 输出角点顺序固定为左上、右上、右下、左下；
- 退化输入返回失败而不是崩溃；
- 产生可查看的 debug image；
- 单元测试和已有测试全部通过。

### 第 2 步：检查已有代码和修改

```fish
git status --short
git diff
rg "SemanticMap|ConeDetection|homography" src docs
```

`rg` 是快速搜索工具。搜索能防止重复定义已有接口。

### 第 3 步：先设计纯算法，再接 ROS

例如九宫格：

```text
纯函数：图片 → 四角点 → 透视图 → cell 结果
                         │
                         ▼
ROS 节点：订阅图片、调用纯函数、发布 SemanticMap
```

纯算法不依赖 ROS topic，更容易用固定图片测试。节点只负责参数、消息转换和发布。

### 第 4 步：只构建相关 package

```fish
source tools/setup_dev.fish
colcon build --symlink-install --packages-select <包名>
source tools/setup_dev.fish
colcon test --packages-select <包名>
colcon test-result --verbose
```

接口 package 改动会影响下游，此时使用：

```fish
colcon build --symlink-install --packages-up-to <下游包名>
```

### 第 5 步：运行整仓回归

```fish
colcon build --symlink-install
source tools/setup_dev.fish
colcon test
colcon test-result --verbose
git diff --check
```

### 第 6 步：人工查看最终 diff

```fish
git status --short
git diff --stat
git diff
```

逐项检查：

- 是否只改了任务需要的文件；
- 是否出现大文件、权重、视频、日志或构建产物；
- 是否删除安全检查；
- 参数是否写死；
- 失败路径是否安全；
- 新接口是否更新了使用方和文档；
- AI 声称执行的测试是否真的有输出证据。

### 第 7 步：提交一个可解释的小改动

在你确认 diff 后：

```fish
git add <明确列出的文件>
git diff --cached
git commit -m "feat(mapping): add perspective transform core"
```

不要使用 `git add .` 盲目加入所有内容。不要让 AI 擅自 push、强制覆盖或改写 Git 历史。

## 9. 当前应从 P8 怎样继续

当前路线不是立刻训练一个大模型，而是先完成结构化场景的 OpenCV 语义地图。

### P8：九宫格语义地图

建议拆成以下可独立验收的小任务：

1. 收集并登记测试素材：空气中、不同光照、不同倾角，之后补水下素材。
2. 定义坐标和输出：角点顺序、俯视图尺寸、3×3 行列方向。
3. 建立 `auv_mapping` package 和最小测试骨架。
4. 实现颜色预处理：HSV/LAB、形态学操作，参数放 YAML。
5. 提取黄色边界和候选四角点。
6. 用 `cv2.getPerspectiveTransform`/`warpPerspective` 对应的 C++ OpenCV API 做矫正。
7. 把标准俯视图切成 3×3 cell。
8. 输出 `auv_interfaces/msg/SemanticMap`。
9. 发布 debug image，保留失败原因和置信度。
10. 用固定图片做回归，再接 ROS 图片 topic 和 bringup。

关键验收不是“某张图看起来可以”，而是：

- 相同输入得到相同结果；
- 没找到完整网格时 `complete: false`；
- 行列方向不随相机倾斜随机翻转；
- 参数可调而非散落在代码里的魔法数字；
- debug 输出能解释失败发生在哪一步。

### P9：交通锥

第一版使用 OpenCV：颜色分割、轮廓、`approxPolyDP` 和圆度

```text
C = 4πA / P²
```

区分圆形和方形。先用固定 cell 图像建立测试，真实水下数据证明规则方法不足后再考虑学习模型。

### P10：规划

- 3×3 格子内部用 A*；
- 4 个目标的访问顺序直接枚举 `4! = 24` 种；
- 把障碍、起点、目标和代价做成明确输入；
- 用确定性地图写单元测试。

不要为这个规模引入强化学习、遗传算法或蚁群算法。

### P11：Mission FSM

按 `AGENTS.md` 的状态顺序实现。先让 FSM 使用模拟事件，验证合法转换、超时、故障和恢复，再连接真实视觉和控制。每次状态变化都发布 `/mission/state` 并记录时间戳。

### P12 以后

海参识别使用 YOLO11n，训练放在独立 uv 环境，部署优先 ONNX Runtime 或 NCNN。训练数据、模型清单、树莓派性能和误检率都要分别验收。抓取和转盘属于机械/安全高风险任务，必须先做无动力模拟和限位设计。

## 10. 面向 P8 的 AI 提示词示例

### 10.1 只做设计评审

```text
请阅读 AGENTS.md、README.md、docs/architecture/repository-layout.md、
src/auv_interfaces/msg/SemanticMap.msg 和 SemanticCell.msg。
不要修改文件。请为 P8 的 auv_mapping package 给出最小设计：节点、纯算法类、参数、
topic、失败语义、debug 输出和单元测试。必须复用已有接口，不使用复杂 SLAM。
```

### 10.2 实现一个小切片

```text
请先检查当前 git diff，保留已有改动。为 P8 实现“已知四角点到固定俯视图”的纯 C++
OpenCV 模块及测试。角点顺序为左上、右上、右下、左下；退化输入明确失败。
暂时不要创建 ROS 节点、不要改 bringup、不要添加新系统依赖。
完成后构建相关 package，运行测试和 git diff --check，并报告没有覆盖的情况。
```

### 10.3 请 AI 审查而不修改

```text
请只做代码审查，不要修改。重点检查：坐标/单位、角点顺序、空输入、OpenCV 异常、
参数硬编码、ROS QoS、时间戳/frame_id、测试是否真的覆盖退化输入。
按严重程度列出问题，并给出文件和行号。
```

### 10.4 修复构建错误

```text
下面是我在 fish 中运行 `colcon build --symlink-install --packages-select auv_mapping`
后的完整输出：<粘贴输出>。
请结合当前 CMakeLists.txt/package.xml 诊断第一处根因。不要通过删除测试、关闭警告或
安装不明版本依赖来掩盖问题。若需要修改，请做最小修改并重新运行同一命令验证。
```

## 11. ROS、视觉、固件分别怎样学习

### 11.1 ROS 2 学习顺序

只学当前任务需要的部分：

1. node、topic、msg；
2. parameter、launch；
3. service；
4. TF 和坐标系；
5. rosbag2；
6. QoS、composition 等进阶内容。

练习时不要另起一个与项目无关的大教程工程。直接观察已有安全节点：列 topic、显示接口、运行离线相机、查看 AprilTag 空检测消息。

### 11.2 OpenCV 学习顺序

1. 图像、像素、BGR/HSV/LAB；
2. threshold 和 mask；
3. morphology；
4. contour、面积、周长；
5. Canny/Hough；
6. homography 与透视变换；
7. 标定和坐标变换。

每一步保存或发布 debug image。只看最终分类结果，很难知道水下光照变化让哪一步失效。

### 11.3 STM32 学习顺序

1. 先读 `firmware/stm32/README.md` 和 `DEVELOPMENT.md`；
2. 理解 UART 帧、CRC、heartbeat 和状态机；
3. 运行 host tests；
4. 做不烧写的交叉编译检查；
5. 在负责人监督下做断电/无桨台架；
6. 最后才考虑水池单自由度测试。

当前 Keil 有效入口位于 `firmware/stm32/MDK-ARM/`。同名 `Core/Src/Move.c`、`Mate.c` 是历史副本，不要改错文件。CubeMX 生成文件的 USER CODE 区域之外可能被重新生成覆盖。

## 12. 测试金字塔

测试按风险从低到高进行：

```text
纯函数单元测试
    ↓
ROS 单 package 测试
    ↓
固定图片/视频/rosbag 离线回归
    ↓
多节点无硬件集成
    ↓
断电或无桨台架
    ↓
单自由度水池测试
    ↓
分阶段任务测试
    ↓
完整 Mission
```

不能因为最高层测试暂时做不了，就跳过低层测试；也不能用几个单元测试通过来宣称实机安全。

视觉测试应保存输入素材版本、参数和期望输出。固件协议测试应让 ROS 与 STM32 使用同一批黄金字节向量。水池测试应保存 rosbag2、事件日志、配置快照、固件版本、模型版本和 Git commit。

## 13. 日志与问题报告

遇到问题时记录：

- 日期和操作人；
- Git commit 与 `git status --short`；
- 操作系统、ROS 版本、设备型号；
- 完整命令；
- 第一处错误；
- 使用的 YAML、模型和固件版本；
- 问题能否稳定复现；
- 安全状态：是否断电、拆桨、DISARM；
- 对视觉问题，保存原始图像而不是手机拍屏幕。

高质量问题模板：

```text
目标：播放固定视频并发布 /camera/down/image_raw
环境：Ubuntu 26.04，ROS 2 Lyrical，fish，commit <hash>
命令：<完整命令>
预期：topic 约 30 Hz
实际：5 秒后停止，第一处日志为 <错误>
输入：<视频路径、编码和校验和>
已尝试：<只写实际执行过的步骤>
安全：未连接 STM32 和推进器
```

## 14. 常见错误与处理

### 找不到 ROS package

```fish
cd ~/auv
source tools/setup_dev.fish
colcon build --symlink-install --packages-select <包名>
source tools/setup_dev.fish
ros2 pkg prefix <包名>
```

### 修改 msg 后 C++ 仍看到旧定义

重新构建接口及下游包，再重新 source。不要手工编辑 `install/` 中生成的头文件。

### AI 给了 Humble/Jazzy 命令

停止执行，要求它以 Ubuntu 26.04 `resolute`、ROS 2 `Lyrical` 和仓库现有 `package.xml` 为准重新核对。不要照抄不同发行版包名。

### Python 包冲突

ROS 运行代码使用系统 Python；YOLO 实验使用独立 uv 环境。不要执行 `sudo pip`，也不要在 ROS 系统解释器中安装训练依赖。

### 相机路径变化

不要长期依赖 `/dev/video0`。确认设备后优先填写稳定的 `/dev/v4l/by-id/...` 路径，并检查实际 V4L2 分辨率、帧率和编码能力。

### ARM 被拒绝

这是安全行为。检查 heartbeat、协议、漏水、kill、传感器有效状态和 ACK。不得删除检查来“修复”。

### AI 修改太多文件

先不要提交。让 AI 解释每个文件为什么必须修改，并回退与任务无关的部分。回退前确认不会覆盖你或队友原有的未提交修改。

## 15. 每次交付的完成清单

### 代码与接口

- [ ] 任务验收标准明确且全部有证据。
- [ ] 修改只位于正确 package，未重复现有功能。
- [ ] 单位、坐标系、时间戳和 frame_id 明确。
- [ ] 参数在 ROS parameters/YAML 中，硬件未知值保留 TODO。
- [ ] 无数据、无效、超时和异常路径行为明确。
- [ ] 公共 msg/srv 或串口协议变化已同步所有使用方和文档。

### 验证

- [ ] 相关 package 构建成功。
- [ ] 新增测试通过，且确认它确实覆盖验收标准。
- [ ] 全仓 `colcon test` 没有新增失败。
- [ ] `git diff --check` 通过。
- [ ] 已查看 `git diff`，没有日志、数据、权重和构建产物。
- [ ] 给出了离线复现步骤和人工观察结果。

### 安全

- [ ] 默认保持 DISARM。
- [ ] 没有绕过 heartbeat、通信超时、漏水、kill 或传感器有效检查。
- [ ] 没有猜 GPIO、PWM、推进器极性或安装方向。
- [ ] 固件修改位于正确有效源文件和安全的 USER CODE 区域。
- [ ] 实机测试注明断电/拆桨/固定方式和现场负责人。

### 文档与交接

- [ ] 说明修改了什么以及为什么。
- [ ] 给出 build、run、verify 命令。
- [ ] 写明仍未验证的硬件条件和风险。
- [ ] 记录 Git commit、配置、素材、模型或固件版本。

## 16. 建议的第一个真实练习

新人不要从推进器控制开始。推荐先做一个完全离线、可撤销的视觉练习：

1. 找一段普通视频或图像序列；
2. 用 `camera_node` 发布 `/camera/down/image_raw`；
3. 用 `ros2 topic hz` 验证帧率；
4. 用 `ros2 topic echo ... --field header --once` 查看时间戳和坐标系；
5. 阅读 `auv_vision` 的相机节点和测试，画出输入、参数、输出；
6. 请 AI 只审查你的数据流理解；
7. 最后把操作、现象和问题写成一页实验记录。

完成这个练习后，再认领 P8 中一个很小的纯算法切片。这样你会同时练到终端、ROS、代码阅读、AI 提示、验证和文档，却不会触碰推进器风险。

## 17. 必读资料顺序

1. 根目录 `README.md`：项目现状和快速入口。
2. `AGENTS.md`：不可违背的项目事实与安全边界。
3. 本教程：新人工作方法。
4. `docs/architecture/repository-layout.md`：package 边界。
5. 当前任务所在目录的 README。
6. 通信任务阅读 `docs/protocol/serial-protocol.md`。
7. 固件/安全任务阅读 `docs/testing/p4-safety.md`、`firmware/stm32/README.md` 和 `DEVELOPMENT.md`。
8. 网络联调阅读 `docs/wired-network.md`。

学习的标准不是“看完了”，而是你能说明数据从哪里来、到哪里去、失败后是否安全、怎样用命令证明功能真的存在。只要坚持小任务、强验证、先离线后实机，AI 就能成为可靠的加速器，而不是新的风险来源。
