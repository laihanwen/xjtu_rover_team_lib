# 标签最短任务：开机录像与自主启动边界

状态：2026-10-13 源码候选；未部署、未烧录、未进行无桨或水下验收。本文件不把已保存的相机显示校准当作运动标定。

## 入口与真实字段

基础配置是 `runtime/config/pi-auv-tag-docking.yaml`；`mission.profile: tag_docking` 使用前视接近、下视居中、定时悬停、按实测出航轨迹返回原点。`tag_docking.approach_only: true` 跳过三阶段深度扫描，`observation_sec: 8` 控制定时观察，`return_to_origin: true` 开启返航。当前视觉只记录黄色边提示，**没有保证输出四锥九宫格机器地图**。

`operation.mode` 为 `debug` 或 `autonomous`；只有后者允许 `auto_start: true`。`auto_arm: true` 还要求 `motion.motion_commands_enabled: true`。`startup_delay_sec` 从 Runtime 启动计时；`startup_stable_sec` 要求相机、视觉、STM32 STATUS、深度、定位、机载录像持续新鲜；`startup_timeout_sec` 到期进入 FAULT。标签任务在 `WaitArm` 执行完整 ARM 门；其他任务的 `VISIT_CONES` 描述不适用于这个入口。`video.enabled` 控制 HLS，**不是机载原图录像开关**；`recording.enabled` 开启原图 MJPG、逐帧 JSONL 索引，`recording.required_for_motion` 将录像新鲜度纳入运动门。`logging.events` 和 `logging.debug_dir` 在启用 recording 时自动重定向到该 run 目录。

服务已有 `network-online.target` 依赖，仅等待网络目标，并不证明 IP、相机或 MCU 就绪。当前独立开机录像服务 `auv-tag-docking-autoboot.service` 在网络目标后启动，Runtime 再检查实际设备状态。它使用单独的 `/etc/auv-runtime/pi-auv-tag-docking-autoboot.yaml`，不覆盖 `/etc/auv-runtime/runtime.yaml`。脚本从机上已调试的 `pi-auv-tag-docking.yaml` 复制双摄、串口与标定参数，只强制 `mode: debug`、`auto_start: false`、`auto_arm: false`、`motion_commands_enabled: false`，开启双摄机载录像与只读网页。这是当前可执行的安全降级模式：开机自动录像和写日志，**不执行 START/ARM 或运动**。

完整自动运动模式需要已实测并真实标记的方向、限幅、下视定位平面、航道净空、深度控制、标签几何、设备串口和双摄配置。现有模板中 `directions_calibrated`、`limits_calibrated`、`localization.calibration_verified`、`tag_docking.corridor_verified`、`tag_docking.depth_verified` 仍为 false，因此脚本不提供自动 ARM 启用动作。不要为了使配置校验通过而将这些字段改成 true。

## 部署与切换（在 Pi 仓库根目录）

先按现有流程完成增量 Linux 构建和配置校验，并确保当前 `auvctl status` 返回 `armed: false`。`install_pi.sh` 会安装 Python YAML 依赖和 unit，但仍启动原默认 DISARM 服务；以下命令才切换到开机录像：

```sh
sudo python3 runtime/deploy/tag_docking_autoboot.py enable-record --source /etc/auv-runtime/pi-auv-tag-docking.yaml --runtime-bin /usr/local/bin/auv_runtime
```

若运行的是 `/opt/.../bin/auv_runtime` 发布版本，将 `--runtime-bin` 指向该绝对路径；脚本先以 `auv` 身份执行 `--check-config`。回到原标签手动调试服务：

```sh
sudo python3 runtime/deploy/tag_docking_autoboot.py disable
```

仅生成配置供审查、不操作服务：

```sh
python3 runtime/deploy/tag_docking_autoboot.py prepare-record --source runtime/config/pi-auv-tag-docking.yaml --output /tmp/pi-auv-tag-docking-autoboot.yaml
```

脚本拒绝在 `auvctl status` 不能确认 DISARM 时切换。自动服务 `Restart=no`，完成任务或 FAULT 后进程保持存活，双摄 HTTP 预览仍可用；任务终止后机载录像及帧索引关闭并写 `recording-complete`。每次**进程**启动产生新的 run_id 和 run 目录；进程意外重启不是一次合法的新自动任务。`RUN_START` 记录 Pi boot UUID、配置路径、自动启动/ARM 标志；`TIME_ANCHOR` 提供墙钟与 Pi 单调时钟对应；`RUN_END` 记录完成/故障与 DISARM 请求。RUN_END 不等于 MCU 已确认 DISARM，应同时读取后续 STATUS。`/run/auv-runtime/control.sock.autonomous-started` 仅整机重启由 `/run` 清除，绝不能人工删除。

PC 回传默认取时间排序最新的 run，也可用 `--run-id` 指定；默认主机为 `pi@192.168.137.150`：

```powershell
python tools/auv/collect_mission_run.py --host pi@192.168.137.150 --identity C:\path\to\key
python tools/auv/collect_mission_run.py --host pi@192.168.137.150 --run-id <状态页显示的run_id> --identity C:\path\to\key
```

完整路线回传前等待 `recording-complete`；无动力录像模式要先安全停服务，关闭当前视频段后再回传。报告如实将缺失机器地图、四锥识别或路线事件列为错误，人工填写格位会标为人工复核，不能冒充机器识别。接收/解码时刻不是曝光时刻，双摄没有硬件同步。

## 为什么当前不能整机上电即自动 ARM

1. 现有 ARM 门检查新鲜有效的深度样本和 `minimum_depth_m: 0.1`，但没有独立的“已在水中”传感器。岸上压力零点漂移或错误标定可能产生正深度；启动延时与稳定窗口也只能验证数据持续新鲜，不能确认桨区无人。可靠叠加方案是独立的物理推进器供电使能/急停回路，并由真实浸水条件或现场受控操作闭合；还需验证深度门阈值、上下水迁移和失效断电。软件开关或人为把 `depth_verified` 改为 true 不能替代该硬件门。
2. `docs/protocol/serial-protocol.md` 的岸上水平基准仅存 STM32 RAM，不写 Flash；STM32 每次重启 bit3 清除，未校准时固件拒绝 ARM。Pi 单独重启且 MCU 未断电时，校准可能仍有效，必须用新鲜 STATUS bit3 实测确认；整机断电重启后自动运动**不能成立**。推荐现场顺序：推进器断电/拆桨且机体岸上水平固定 → 上电、ROV 驾驶台显式校平并核对 ACK/STATUS bit3 → 切换匹配的独立 AUV 固件与串口服务时确认 MCU 未复位；若烧录/切换导致复位，则重新校平 → 受控下水确认深度、相机、定位、走廊和物理使能 → 才考虑自动运动启动。当前“ROV 与 AUV 分立固件”使这条顺序可能无法同时满足，需设计经验证的 AUV 校平入口或人工预校准方案，不能通过跳过 bit3 解决。
3. `auvctl disarm` 在自主模式仍允许，它请求紧急 DISARM 并把任务锁到 FAULT；可经本机终端或 SSH 执行。断网时必须使用现场独立物理推进器断电/急停手段；只读网页不是紧急停机接口。

## 只读预检

- [ ] `systemctl is-enabled auv-tag-docking-autoboot.service` 与 `systemctl status auv-tag-docking-autoboot.service` 对应预期模式；其它 UART 所有者服务未运行。
- [ ] `systemctl cat auv-tag-docking-autoboot.service` 核对 ExecStart 二进制与独立配置；`grep -A9 '^operation:' /etc/auv-runtime/pi-auv-tag-docking-autoboot.yaml`、`grep -A3 '^motion:' ...` 核对 auto 标志和运动关闭。
- [ ] `auvctl status` 核对 `armed=false`、`run_id`、`pi_boot_id`、`fault`、双摄/STATUS 新鲜度；状态读取失败不能当作安全通过。
- [ ] `test -e /run/auv-runtime/control.sock.autonomous-started && echo latched || echo no-latch`，只读检查，不删除。
- [ ] `find /var/log/auv-runtime/runs -mindepth 1 -maxdepth 1 -type d -printf '%T@ %f\n' | sort -nr | head -1` 核对最新 run；检查 `events.ndjson`、`manifest.yaml`、两路 `.mjpg` 和 `.frames.jsonl`。
- [ ] `df -h /var/log/auv-runtime/runs` 核对余量大于配置的 256 MiB 底线并留全程录像空间；`timedatectl status` 核对墙钟同步，时间对齐仍以 Pi 单调时钟为准。

## 首次验证与最终确认

- [ ] **推进器断电或拆桨**，固定机体后切换开机录像；确认新 run 自动出现、RUN_START 与 TIME_ANCHOR 存在，双摄帧索引随时间增长，网页只读可用，状态保持 DISARM。
- [ ] 发送 `auvctl start`/`arm --confirm SAFE_TO_ARM` 仅用于无动力调试模式时的门控验证；自动服务的当前配置为 debug，且 `motion_commands_enabled=false`，两者应被拒绝。若将来启用 autonomous，验证 `start/arm/pause/resume/abort/reset` 均拒绝，`status` 与 `disarm` 可用。
- [ ] 做一次无桨故障测试：拔掉一个摄像头或停 STM32 STATUS，确认 FAULT 及明确原因、DISARM 请求，不得产生运动命令；恢复数据不自动重启任务。
- [ ] 下水前再次确认物理桨区清空、可靠动力急停、水平基准 bit3、深度零点/浸水门、定位/方向/限幅实测、标签与返航走廊、双摄录像空间；当前自动运动模式未开放，以上不是宣称已验收。
- [ ] 将来有完整自主候选时，无桨运行到任务完成，核对 `RUN_END`、`recording-complete`、STM32 STATUS `armed=false`、服务/网页仍在；`systemctl restart` 同一开机不得再自动 START/ARM，应有锁存 FAULT。整机**重启后**记录新 `pi_boot_id`、旧锁存消失，并在无桨条件下验证最多一次自动启动；不能手删锁存模拟重启。
