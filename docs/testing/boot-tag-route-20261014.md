# 开机单次「上升—直行—见标签返航」试验

状态：源码、便携状态机测试和 Ubuntu 虚拟双摄/STM32 回放已完成；设备部署、固件烧录和水下运动尚未验证。`pi-auv-boot-tag-route.yaml` 是本任务专用的自主模板，明确启用一次性自动 START/ARM 与运动，但仍保留未验证的标定标志为 false；不覆盖 `/etc/auv-runtime/runtime.yaml`。模板串口是占位路径，不能直接用于真机。现场必须核对当前固件、Pi 服务和串口占用。

## 任务与证据

专用 `mission.profile: boot_tag_route` 在 DISARM 状态等待独立 M10 样本：深度必须为 0.35–1.2 m，连续至少 10 s 波动不超过 0.03 m，且 MCU STATUS 安全、双摄与记录器新鲜。随后通过现有一次性锁存发出 START 与一次 ARM 请求。STM32 ACK 和 armed STATUS 都到达后，以起始深度减 0.2 m 为定深目标；到位后锁定当前 yaw，以归一化目标 0.04 前行，最多 15 s。任一相机解码到 ID18 立即停前行并倒退；没看到则到 15 s 也返航。有效连续下视定位优先判断回到原点；缺失时按前行时长、同档速度倒退，`RETURN_START/RETURN_OPEN_LOOP` 标记开环，不能解释为精确距离。无定位时 0.5 m 位移上限不可实测保证，仍受 15 s 和低速命令约束。若深度越界、传感器/STATUS/相机/记录失效、航向偏差、上浮或返航超时，立即故障停平移、DISARM 并记录原因。最终平移归零，等待独立深度样本稳定后 DISARM，不再自动重复；双摄录像和只读网页在任务结束后继续运行，直到服务停止或磁盘保护触发。STM32 的深度、yaw、横滚和俯仰 PID 是否真正工作取决于已烧录 AUV 固件配置，Pi 源码不能替代该验收。

`events.ndjson` 保留既有 `RUN_START/TIME_ANCHOR/ARM_REQUEST/ARM_CONFIRMED/RUN_END/FAULT`，新增 `DEPTH_STABLE/AUTO_START/ASCEND_START/ASCEND_DONE/FORWARD_START/FORWARD_END/TAG_SEEN/RETURN_START/RETURN_OPEN_LOOP/RETURN_END/HOLD_STABLE/BOOT_ROUTE_TRACE`。双摄 MJPG 与帧索引沿用同一 run 目录；回传用 `tools/auv/collect_mission_run.py --evidence-only`。相机戳是 Pi 收帧时间，不是曝光时刻，两摄不硬同步。

## 现场顺序与硬件阻断

1. 岸上断推进器动力、机体水平固定；烧录/读回核对 **AUV 固件**，不能使用独立 ROV 固件。核对 Pi 实际运行的是 AUV Runtime 而非 ROV 桥，确认两者不会同时占 UART 或相机。接通 STM32 后显式执行水平基准校准；该基准仅在 MCU RAM，**每次 STM32 重启都要重做**。Pi 自动服务不能替代这一步，因此真正无人值守的「整机上电立刻任务」在当前硬件上不成立。
2. 本地源码 `AuvAutonomyConfig.h` 已将自主前进/横移前馈设为有 ROV 映射依据的 300，但尚未编译烧录到 STM32。定深方向和最大深度仍为 0，`AUV_DEPTH_CONTROL_CALIBRATED` 与自主 commissioned 标志仍未通过；现有 AUV 固件会拒绝 ARM，不能用 Pi 配置强行跨越。需先在无桨/断推进器动力条件验证 STM32 源选择、目标方向、定深与航向环，再经受控水池实测符号和限幅，形成可读回的专用固件。
3. 本任务经项目负责人授权，不要求把 `directions_calibrated`、`limits_calibrated`、`depth_verified`、`corridor_verified` 逐项伪造为 true；专用 `explicit_motion_authorization: true` 仅放宽 Pi 对该路线的这些软件标志，不放宽 MCU、STATUS、深度和录像门控。现场先在断推进器动力条件确认 MCU 可以安全接受目标，并核对实际方向及输出限幅。单独准备 `/etc/auv-runtime/pi-auv-boot-tag-route.yaml`，只把占位串口改为真实设备路径，不修改 `/etc/auv-runtime/runtime.yaml`。保持 `/etc/auv-runtime/pi-auv-tag-docking.yaml` 为 DISARM 手动基线。确认 MCU 就绪后用 `sudo python3 /usr/local/share/auv-runtime/deploy/tag_docking_autoboot.py enable-route` 切换；脚本检查真实 STM32 串口、配置解析和 DISARM。回手动：同脚本 `disable`。不要删除 `/run/auv-runtime/control.sock.autonomous-started` 来重试。
4. 顺序为**岸上校平 → 保持 MCU 供电并放入水中（入水深度至少 0.35 m，预留上升及机体净空）→ 启动/已运行的 Pi 自动服务等待稳定判据并执行一次序列 → 回传**。如果「开机」指 MCU 从断电重新启动，则必须先在岸上重做校平，不得用任务自动 ARM 绕过；可让 Pi 服务预先运行并保持等待，或校平后再启用路线。现场复测岸上曾出现的 `error_flags=0x0A`（传感器无效 + kill）是否已清零，观察 STATUS、深度、yaw、推进方向、视频写盘与任务事件。任何异常立即 `auvctl disarm`，并用实体 kill 或切断推进器动力。故障后不再自动 ARM。用 `python3 tools/auv/collect_mission_run.py --evidence-only ...` 按该工具帮助填写目标和输出目录，保存证据后再断电。
5. 需要再做一次时，必须是真实整机重新上电：读取新 `/proc/sys/kernel/random/boot_id`，确认先前运行锁存因新启动自然消失；STM32 再次岸上水平校准，先无动力核对 STATUS/kill/传感器和锁存行为，再进行下一次受控水池试验。单纯重启 Pi 服务不得清除锁存，也不得触发第二次 ARM。

## 验证边界

便携状态机与配置测试验证状态转移、岸上阻断、标签触发、时间上限、故障停机和专用授权；Ubuntu 虚拟串口/双摄回放额外验证 ACK、记录和服务重启锁存。PID 实际稳定性、入水判据、推进方向与停止响应、光流尺度、返航精度以及双摄长期记录仍必须在相应设备/水池验证。深度正值稳定**不能物理证明传感器已浸水**；当前没有湿度或浸水开关，若需可证明的水中使能，必须增加独立物理互锁。

## Ubuntu 离线增量编译与测试

在仓库根目录执行。首次配置需要 `cmake`、`ninja-build`、`g++`、`pkg-config`、OpenCV C++ 开发包（至少 4.7）、`libyaml-cpp-dev`、`libcpp-httplib-dev`、`python3-yaml`；Python 回放还需 `python3-opencv` 与 `python3-numpy`。使用本机已有依赖即可，不需要连接 Pi 或控制硬件。

```bash
cmake -S . -B build-native -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build-native --target auv_runtime auvctl -j 3
ctest --test-dir build-native -R '^(boot_tag_route_simulation|runtime_pty|tag_docking_simulation|autonomous_simulation)$' --output-on-failure

cmake -S . -B build/boot-route-portable -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DAUV_PERCEPTION_ONLY=ON
cmake --build build/boot-route-portable --target test_boot_tag_route test_tag_docking -j 3
ctest --test-dir build/boot-route-portable -R '^(boot_tag_route|tag_docking)$' --output-on-failure

cmake -S firmware/stm32/test -B build/boot-route-firmware -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/boot-route-firmware --target test_auv_safety test_auv_autonomy -j 3
ctest --test-dir build/boot-route-firmware -R '^(auv_safety|auv_autonomy_defaults)$' --output-on-failure
python3 -m unittest tools.auv.test_collect_mission_run
```

上述检查分别验证 Linux Runtime 构建、任务状态机与相关故障、STM32 主机安全逻辑、合成回传包的时间对齐。按本次修改只需重跑受影响目标；固件测试与回传测试列作首次完整检查参考。`auv_runtime` 构建和回放通过仍不代表已烧录、已部署或已下水验收。若有失败，请保存完整编译输出、`ctest --output-on-failure` 输出和当前提交号；不要为了通过测试更改 MCU 安全标志或删自主启动锁存。

## 2026-10-14 Ubuntu 增量复测

在独立工作区对本轮门控与测试修复重新构建 `auv_runtime`、`test_tag_docking`、`test_boot_tag_route`，构建通过；便携标签状态机与配置门控 2/2 通过。Ubuntu Runtime 相关回归 4/4 通过，包含新增开机路线虚拟双摄/串口回放。模板 `--check-config` 通过，标定标志保持 false；`--check-config` 只检查配置语义，不证明占位串口存在。测试检查关闭专用授权或自动 ARM 会拒绝启动。固件源码、实机服务与已烧录镜像未修改，本次没有执行设备或水池验证，也未运行全仓库测试。
