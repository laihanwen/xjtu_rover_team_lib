# 开机单次「上升—直行—见标签返航」试验

状态：源码和便携状态机测试已完成；设备部署、固件烧录和水下运动尚未验证。`pi-auv-boot-tag-route.yaml` 是保持 debug、运动关闭、自动 START/ARM 关闭的专用模板，不覆盖 `/etc/auv-runtime/runtime.yaml`。现场必须核对当前固件和串口占用。

## 任务与证据

专用 `mission.profile: boot_tag_route` 在 DISARM 状态等待独立 M10 样本：深度必须为 0.35–1.2 m，10 s 内波动不超过 0.03 m，且 MCU STATUS 安全、双摄与记录器新鲜。随后通过现有一次性锁存发出 START 与一次 ARM 请求。STM32 ACK 和 armed STATUS 都到达后，锁定当前 yaw，目标深度比起始浅 0.2 m；到位后以归一化目标 0.04 前行，最多 15 s。任一相机解码到 ID18 立即停前行并倒退；没看到则到 15 s 也返航。有效连续下视定位优先判断回到原点；缺失时按前行时长倒退同样时长，`RETURN_START/RETURN_OPEN_LOOP` 标记开环，不能解释为精确距离。最终平移归零，等待独立深度样本稳定后 DISARM，不再自动重复；双摄录像和只读网页在任务结束后继续运行，直到服务停止或磁盘保护触发。

`events.ndjson` 保留既有 `RUN_START/TIME_ANCHOR/ARM_REQUEST/ARM_CONFIRMED/RUN_END/FAULT`，新增 `DEPTH_STABLE/AUTO_START/ASCEND_START/ASCEND_DONE/FORWARD_START/FORWARD_END/TAG_SEEN/RETURN_START/RETURN_OPEN_LOOP/RETURN_END/HOLD_STABLE/BOOT_ROUTE_TRACE`。双摄 MJPG 与帧索引沿用同一 run 目录；回传用 `tools/auv/collect_mission_run.py --evidence-only`。相机戳是 Pi 收帧时间，不是曝光时刻，两摄不硬同步。

## 现场顺序与硬件阻断

1. 岸上断推进器动力、机体水平固定；烧录/读回核对 **AUV 固件**，不能使用独立 ROV 固件。接通 STM32 后显式执行水平基准校准；该基准仅在 MCU RAM，**每次 STM32 重启都要重做**。Pi 自动服务不能替代这一步，因此真正无人值守的「整机上电立刻任务」在当前硬件上不成立。
2. 本地源码 `AuvAutonomyConfig.h` 已将自主前进/横移前馈设为有 ROV 映射依据的 300，但尚未编译烧录到 STM32。定深方向和最大深度仍为 0，`AUV_DEPTH_CONTROL_CALIBRATED` 与自主 commissioned 标志仍未通过；现有 AUV 固件会拒绝 ARM，不能用 Pi 配置强行跨越。需先在无桨/断推进器动力条件验证 STM32 源选择、目标方向、定深与航向环，再经受控水池实测符号和限幅，形成可读回的专用固件。
3. 仅在 MCU 就绪且方向、限幅、深度和路线净空已有实测证据后，单独准备 `/etc/auv-runtime/pi-auv-boot-tag-route.yaml`：填写真实串口，将 `directions_calibrated`、`limits_calibrated`、`depth_verified`、`corridor_verified` 依据各自实测设为 true，确认路线最大位移不超过已验证走廊半径，再显式开启运动、`autonomous`、自动 START/ARM 和专用授权。原始模板故意不能通过该路线的 `--check-config`；`enable-route` 也不能直接从未验收模板启动。保持 `/etc/auv-runtime/pi-auv-tag-docking.yaml` 为 DISARM 手动基线，核对专用配置后才运行 `sudo python3 /usr/local/share/auv-runtime/deploy/tag_docking_autoboot.py enable-route`。脚本检查真实 STM32 串口、配置解析和 DISARM 后切换服务；不修改 `runtime.yaml`。回手动：同脚本 `disable`。若有故障锁存，**不得删除** `/run/auv-runtime/control.sock.autonomous-started` 来重试；整机重启后读新 `/proc/sys/kernel/random/boot_id`，确认锁存由新启动自然清空，并重做水平校准。
4. 放入水中，至少 0.35 m，确保上升 0.2 m 后仍有安全净空，再让自动服务走完稳定判据。现场观察 STATUS、深度、yaw、推进方向、视频写盘与任务事件。任何异常立即 `auvctl disarm`，并用实体 kill 或切断推进器动力；服务在故障后不再自动 ARM。

## 验证边界

Windows 增量测试 `boot_tag_route` 与 `tag_docking` 验证状态转移、岸上阻断、标签触发、时间上限、故障停机和配置授权。锁存与 MCU ACK/timeout、PID 实际稳定性、入水判据、推进方向、光流尺度、返航精度以及双摄长期记录必须在相应设备/水池验证。深度正值稳定**不能物理证明传感器已浸水**；当前没有湿度或浸水开关，若需可证明的水中使能，必须增加独立物理互锁。当前岸上 `error_flags=0x0A`（传感器无效和 kill）尚须入水后确认清零。

## Ubuntu 离线增量编译与测试

在仓库根目录执行。首次配置需要 `cmake`、`ninja-build`、`g++`、`pkg-config`、OpenCV C++ 开发包（至少 4.7）、`libyaml-cpp-dev`、`libcpp-httplib-dev`、`python3-yaml`；Python 回放还需 `python3-opencv` 与 `python3-numpy`。使用本机已有依赖即可，不需要连接 Pi 或控制硬件。

```bash
cmake -S . -B build-native -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build-native --target auv_runtime auvctl -j 3
ctest --test-dir build-native -R '^(runtime_pty|tag_docking_simulation|geometry_probe_readonly|autonomous_simulation)$' --output-on-failure

cmake -S . -B build/boot-route-portable -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DAUV_PERCEPTION_ONLY=ON
cmake --build build/boot-route-portable --target test_boot_tag_route test_tag_docking -j 3
ctest --test-dir build/boot-route-portable -R '^(boot_tag_route|tag_docking)$' --output-on-failure

cmake -S firmware/stm32/test -B build/boot-route-firmware -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/boot-route-firmware --target test_auv_safety test_auv_autonomy -j 3
ctest --test-dir build/boot-route-firmware -R '^(auv_safety|auv_autonomy_defaults)$' --output-on-failure
python3 -m unittest tools.auv.test_collect_mission_run
```

上述检查分别验证 Linux Runtime 能否构建、任务状态机与相关故障、STM32 主机安全逻辑、合成回传包的时间对齐。`auv_runtime` 构建和回放通过仍不代表已烧录、已部署或已下水验收。若有失败，请保存完整编译输出、`ctest --output-on-failure` 输出和当前提交号；不要为了通过测试更改安全标志或删自主启动锁存。

## 2026-10-14 Ubuntu 增量复测

在独立工作区对本轮门控与测试修复重新构建 `auv_runtime`、`auvctl`、`test_tag_docking`、`test_boot_tag_route`，构建通过。Runtime 虚拟串口、标签仿真、几何只读探针、自主模式仿真 4/4 通过；便携标签状态机与配置门控 2/2 通过。默认开机路线模板运行 `auv_runtime --check-config` 返回非零，符合待验收模板要求；单测另检查各标志缺失时被拒绝，全部明确标记后才接受。固件源码、实机服务与已烧录镜像未修改，本次没有执行设备或水池验证，也未运行全仓库测试。
