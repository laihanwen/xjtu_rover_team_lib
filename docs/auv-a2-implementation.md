# A2 任务一闭环实现与验收

> 2026-10-10：新增到位容差内停平移等待新观测，Windows 便携 12/12 CTest 通过。这里的 surface 阶段名称沿用接口，不代表允许机器人露出水面；实际遍历净空和独立成像标定需实测。下视完整图可见性不足、前视定位链尚未迁移，见 [当前状态](project-status.md)。

2026-10-07。衔接 [A0/A1](auv-a0-a1-implementation.md)，新增原生 Runtime 配置 `runtime/config/pi-auv-task-one.yaml`，profile 为 `a2_task_one`。默认关闭运动、自动启动/ARM 与 A2 推进阶段，必须实测标定后启用。

## 运行边界

状态顺序为：SELF_CHECK → SEARCH_APRILTAG → BUILD_MAP → SURFACE_FOR_CONES → RELOCALIZE_SURFACE → PLAN_CONES → VISIT_CONES → COMPLETE。

前半段复用 A1 的投放点原点、安全航点进入中心、连续标签确认和四锥地图证据。标签可能在机体到达格中心前进入视野，所以建图后先按水下绝对网格定位，在实测安全的中心格接近区域内对准上浮位置；需要新帧持续确认，超出接近区域就中止。位置确认后才限速上浮。上浮时 surge/sway 为 0、航向保持，绝对深度目标逐步降低；必须实际验证该上浮柱的净空、横向漂移和机构状态。不能把“无水平命令”等同物理位置固定。

表面确认同时要求目标深度附近稳定、至少三个新的真实压力计采样，以及持续安全遥测。AUV 固件 DEPTH 扩展为 17 字节，携带真实采样序号和年龄；原 9 字节发送序号不能满足此门槛。ROV 固件仍发送 9 字节。详见 [串口协议](protocol/serial-protocol.md)。

上浮后不沿用水下光流积分。下视图像使用独立表面内参和畸变参数，以唯一黄色下边统一网格方向，使用实测格长与四角 PnP 求绝对机体位置和网格到机体的运动变换。摄像头相对机体偏移、压力计偏移参与计算。必须满足分辨率、网格置信度、重投影误差、姿态和池深/压力高度一致性；连续多个新帧中的四锥位置与形状还须匹配冻结的水下地图，才能规划。

这版要求水面阶段持续看见完整网格和黄色单边。视野、反光或水面折射不满足条件时进入 FAULT/DISARM，不用失效光流或规划位置代替实测定位。这是需要在真实场地验证的部署边界。

COMPLETE 仅表示 A2 四锥遍历阶段完成并停止推进，不表示已完成海参、转盘、返航或整场比赛。A2 禁止运行中 RESUME 和同进程 RESET；暂停/结束后需新进程、新 run_id 和重新定位，避免复用旧路线及覆盖证据。

## 规划与实际经过确认

规划使用有限状态 BFS：状态为“当前格 + 已进入交通锥位图”，规模至多 9×16。进入未访问锥格立即加入位图，之后任何中间路径再次进入该锥格都禁止。非锥格允许重复。它直接对完整经过路径求最短路；起点本身为锥格时计入首次访问。既有 `task_one`/`full` 的规划行为保持原默认值。

执行根据新的绝对表面定位更新机体实际进入格。记录进入事件后，只有在格内持续得到新帧确认才标记 CONE_VISITED；推进目标 ACK、计划节点或同一帧重复读取都不能确认访问。路径结束还须四个不同锥格均得到实测确认。提前离开未确认锥格、再次进入锥格、定位跳变、朝向突变、非相邻/对角跳格、离开规划走廊、超出场地机体净空、航点超时、表面深度偏离或失去 ARM 连续性都会中止。

规则对“进入格”的现场判定以及搜索/建图阶段是否计入重复，仍需队长依据现场口径确认。这版的无重复约束作用于 A2 水面遍历；保留前半程实测轨迹和 CELL_OBSERVED 记录供核验，不宣称水下搜索阶段从未经过任何锥节点。

## 构建、校验与回放

沿用原生构建（fish）：

```fish
cmake -S . -B build-lightweight -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-lightweight -j 3
ctest --test-dir build-lightweight --output-on-failure
build-lightweight/runtime/auv_runtime --check-config runtime/config/pi-auv-task-one.yaml
```

标定和安全固定试验通过后，再以此配置启动 Runtime。systemd 模板 `runtime/deploy/auv-task-one.service` 与 ROV、旧 Runtime 和 A1 服务冲突，不会在本次实现中自动启用。

独立 AUV 固件构建命令仍为 `./tools/auv/Build-AuvFirmware.ps1`，仅构建、不烧录。本轮 AUV HEX SHA256：`C75F4E269916785C188163EFE4564598697B4511DFED4B1326A1DB882838BAAC`。固件默认未 commissioning，不能 ARM。

机载记录新增 `planned_route.json`（明确标记 planned_only）、`trajectory.jsonl`（实测定位、帧号、采样时间、阶段、深度传感器序号/年龄）和 CELL_ENTERED/CONE_VISITED/SURFACE_CONFIRMED/A2_COMPLETE 事件。记录在独立写盘线程中处理，队列有上限；记录失败或队列溢出中止任务。水下原点坐标与黄色基准格坐标分别标记，不混成同一轨迹坐标系。

表面索引录像回放只读取文件，不打开硬件或发送 ARM：

```fish
build-lightweight/runtime/auv_surface_replay \
  runtime/config/pi-auv-task-one.yaml \
  /path/to/run/down-1.mjpg /path/to/run/down-1.frames.jsonl \
  /path/to/new-surface-poses.jsonl
```

先填 verified 的表面几何配置。回放按索引偏移解码原始 JPEG，输出每帧定位有效性、行列位置和拒绝原因；仅分析 RELOCALIZE_SURFACE/PLAN_CONES/VISIT_CONES 阶段，不生成任务完成结论。输出文件必须不存在。JPEG 索引中的遥测为录像写入时的最近状态，未实现相机与压力计硬件同步。

## 必须实测后填入的参数

- 延续 A0/A1 固件、相机、IMU、深度零点与安全航点标定。Pi 深度零点保持 0，因为 AUV MCU 已扣除表面零点。相机、定位与建图的水下内参必须一致。
- 实测池深与格长；完整表面视野、摄像头外参、摄像头/压力计偏移及独立表面内参。确认水面工况的折射、反光、倾斜和帧率确实支持该模型。
- 确认中心格接近区域和上浮柱安全，实测 center_approach_radius_cells 并开启 center_approach_verified；该区域必须计入机体净空，不能穿出中心格。验证上浮速率和无横向命令时的漂移。按现场规定确定 `depth_target_m` 是完全上浮还是允许的浅水高度；不把默认 0 当已验证判据。
- 实测机体/机构净空填 boundary_clearance_m，验证路径走廊、定位跳变容限和节点内确认裕量。压力计新采样周期须满足 sample timeout；上浮 timeout 要覆盖完整爬升和多次采样确认。
- 在 motion/search 已标定、双摄录像正常之后，开启 surface_traversal 并填写 ascent_clearance_verified、surface_localization_verified。自主模式仍须显式启用 auto_start/auto_arm，填完整发布清单。

## 本轮验证

8 项可移植测试全部通过，其中 A2 覆盖 126 种四锥布局×9 起点，共 1,134 个规划组合；覆盖四朝向投影、错误高度/分辨率拒绝、上浮限速、重复压力报文拒绝、按实测新帧遍历、定位过期/跳变/朝向突变、非法路径拒绝、阶段门控、配置标定门槛，以及索引 JPEG → 网格 → 绝对表面定位的离线链路。

16 项固件主机回归通过；AUV/ROV Keil 编译均为 0 错误、0 警告。Runtime 做 Linux 目标语法检查（含 HTTP 分支）；此检查不能替代 Pi 完整链接和进程级设备联调。

树莓派 `192.168.137.150` 本轮 SSH 仍超时，未部署、烧录或 ARM。真实上浮、表面定位误差、节点判定、录像吞吐和失效停机均未实机验收。A2 下水验收须从投放区一次启动重复试验，保存每次 run_id，报告成功次数/总次数、定位误差与失败原因；通过后才进入 A3。
